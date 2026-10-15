"""Task 2 protocol/thread regression tests; no Pi hardware or model required."""
import importlib.util
import itertools
import json
import socket
import struct
import tempfile
import time
import unittest
from pathlib import Path
from threading import Event, Thread
from unittest.mock import Mock, patch

from task2 import Motion, Task2, android_status, task2_control_status
from task2_vision import Vision

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("task2_pc", ROOT / "pc" / "task2_pc.py")
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class Link:
    def __init__(self, sock):
        self.sock = sock
        self.stream = sock.makefile("rb")

    def send(self, line):
        self.sock.sendall((line + "\n").encode())

    def receive(self):
        return self.stream.readline().decode().strip()

    def send_image(self, image, header):
        self.send(header)
        self.sock.sendall(struct.pack(">I", len(image)) + image)


class Coordination(unittest.TestCase):
    def test_mid_frame_disconnect_is_classified_as_session_end(self):
        left, right = socket.socketpair()
        left.sendall(b"abc")
        left.shutdown(socket.SHUT_WR)
        with right.makefile("rb") as stream:
            with self.assertRaisesRegex(server.PeerDisconnected,
                                        "disconnected during image transfer"):
                server.exact(stream, 4)
        left.close()
        right.close()

    def test_parking_completion_uses_same_heading_limit_as_entry(self):
        import math
        import task2t_park as geometry
        for error, expected in [(11, 0), (-11, 0), (13, 1)]:
            with self.subTest(error=error), patch.object(geometry, "PARK_HEADING_TOL_DEG", 12):
                stm = Mock()
                stm.query_fields.return_value = ["1", "0", "0", "0"]
                task = Task2(stm, Mock(), Event(), Mock())
                task.frame.pose = Mock(return_value=geometry.Pose(
                    -14.5, 0, math.pi + math.radians(error)))
                task._us = Mock(return_value=15)
                task._ir = Mock(return_value=24)
                with patch("task2.route.Task2.run", return_value=0):
                    self.assertEqual(task.run(), expected)

    def test_standalone_startup_uses_supported_calibration_api(self):
        import task2t as route
        with patch.dict("sys.modules", {"serial": Mock()}):
            from communications import cal_profile
        stm = Mock()
        stm.set_fast.return_value = True
        task = route.Task2(stm)
        with patch.dict("sys.modules", {"serial": Mock(),
                                        "communications.cal_profile": cal_profile}), \
                patch.object(cal_profile, "firmware_protocol", return_value=8), \
                patch.object(cal_profile, "prepare_for_task", autospec=True,
                             return_value=True) as prepare, \
                patch.object(route, "FAST", True):
            self.assertTrue(task._startup())
        self.assertEqual(prepare.call_args.kwargs["required"], True)
        stm.set_fast.assert_called_once_with(True)
        self.assertTrue(task.fast)

    def test_begin_resets_heading_after_manual_reposition(self):
        stm = Mock()
        stm.is_busy.return_value = False
        heading = {"actual": -146.6, "commanded": 0.0}
        def zero():
            heading.update(actual=0.0, commanded=0.0)
            return True
        stm.zero_odometry.side_effect = zero
        stm.read_heading.side_effect = lambda: dict(heading)
        motion = Motion(stm, Event(), Mock())
        with patch.dict("sys.modules", {"serial": Mock()}):
            self.assertTrue(motion.prepare_start())
        self.assertEqual([call[0] for call in stm.mock_calls],
                         ["is_busy", "zero_odometry", "read_heading"])

    def test_begin_refuses_busy_or_unverified_heading_reset(self):
        for busy, zeroed, actual in [(True, True, 0), (None, True, 0),
                                     (False, False, 0), (False, True, 12.2)]:
            with self.subTest(busy=busy, zeroed=zeroed, actual=actual):
                stm = Mock()
                stm.is_busy.return_value = busy
                stm.zero_odometry.return_value = zeroed
                stm.read_heading.return_value = {"commanded": 0, "actual": actual}
                with patch.dict("sys.modules", {"serial": Mock()}):
                    self.assertFalse(Motion(stm, Event(), Mock()).prepare_start())
                stm.send_line.assert_not_called()
                if busy is not False:
                    stm.zero_odometry.assert_not_called()

    def test_cancelled_begin_does_not_zero_heading(self):
        stm, stop = Mock(), Event()
        stop.set()
        with patch.dict("sys.modules", {"serial": Mock()}):
            with self.assertRaisesRegex(RuntimeError, "cancelled"):
                Motion(stm, stop, Mock()).prepare_start()
        self.assertEqual(stm.mock_calls, [])

    def test_route_tracks_startup_fast_mode_and_sends_precision_switch(self):
        with patch("task2.route.FAST", True):
            stm = Mock()
            stm.set_fast.return_value = True
            task = Task2(stm, Mock(), Event(), Mock())
            self.assertTrue(task.fast)
            task._set_fast(False)
            stm.set_fast.assert_called_once_with(False)
            self.assertFalse(task.fast)
            task._set_fast(True)
            self.assertTrue(task.fast)
            self.assertEqual(stm.set_fast.call_count, 2)

    def test_failed_precision_switch_does_not_claim_slow_mode(self):
        with patch("task2.route.FAST", True):
            stm = Mock()
            stm.set_fast.return_value = False
            task = Task2(stm, Mock(), Event(), Mock())
            task._set_fast(False)
            self.assertTrue(task.fast)

    def test_pending_vote_expires_and_cannot_leak_between_stages(self):
        vision = Vision(Mock(), Event(), "run")
        vision.begin(2)
        self.assertFalse(vision.arrow_pending())
        vision.candidate_time = time.monotonic()
        self.assertTrue(vision.arrow_pending())
        vision.candidate_time -= vision.max_age + 1
        self.assertFalse(vision.arrow_pending())
        vision.candidate_time = time.monotonic()
        vision.begin(1)
        self.assertFalse(vision.arrow_pending())
        vision.candidate_time = time.monotonic()
        vision.idle()
        self.assertFalse(vision.arrow_pending())

    def test_completed_parking_does_not_require_exact_target_or_centre(self):
        import math
        import task2t_park as geometry
        stm = Mock()
        stm.query_fields.return_value = ["1", "-5", "-2", "0", "6225"]
        task = Task2(stm, Mock(), Event(), Mock())
        task.frame.pose = Mock(return_value=geometry.Pose(-14.5, 4.6, math.radians(-180.1)))
        task._us = Mock(return_value=15.7)
        task._ir = Mock(side_effect=[None, 19.0])
        with patch("task2.route.Task2.run", return_value=0):
            self.assertEqual(task.run(), 0)

    def test_confirmed_bay_wall_recovers_impossible_back_wall_odometry(self):
        import math
        import task2t_park as geometry
        for echoes, expected in [([16.7, 16.8], 0),
                                 ([16.7, 25.0], 1)]:
            with self.subTest(echoes=echoes):
                stm = Mock()
                stm.query_fields.return_value = ["1", "0", "0", "0"]
                task = Task2(stm, Mock(), Event(), Mock())
                task.frame.pose = Mock(return_value=geometry.Pose(-34.5, 3.4, math.pi))
                task._us = Mock(side_effect=echoes)
                task._ir = Mock(side_effect=[37.0, 16.0])
                with patch("task2.route.Task2.run", return_value=0):
                    self.assertEqual(task.run(), expected)

    def test_unidentified_side_walls_cannot_override_outside_pose(self):
        import math
        import task2t_park as geometry
        stm = Mock()
        stm.query_fields.return_value = ["1", "0", "0", "0"]
        task = Task2(stm, Mock(), Event(), Mock())
        task.frame.pose = Mock(return_value=geometry.Pose(-34.5, 3.4, math.pi))
        task._us = Mock(return_value=16.7)
        task._ir = Mock(side_effect=[None, 16.0])
        with patch("task2.route.Task2.run", return_value=0):
            self.assertEqual(task.run(), 1)
        self.assertEqual(task._us.call_count, 2)

    def test_parking_rejects_unsafe_missing_or_moving_measurements(self):
        import math
        import task2t_park as geometry
        for us, left, right, rate, x in [(None, 31, 19, 0, -14.5),
                                       (1, 31, 19, 0, -14.5),
                                       (30, 31, 19, 0, -14.5),
                                       (15.7, 48, 2, 0, -14.5),
                                       (15.7, 31, 19, 30, -14.5),
                                       (15.7, 31, 19, 0, 5)]:
            with self.subTest(us=us, left=left, right=right, rate=rate, x=x):
                stm = Mock()
                stm.query_fields.return_value = ["1", "0", str(rate), "0"]
                task = Task2(stm, Mock(), Event(), Mock())
                task.frame.pose = Mock(return_value=geometry.Pose(x, 0, math.pi))
                task._us = Mock(return_value=us)
                task._ir = Mock(side_effect=[left, right])
                with patch("task2.route.Task2.run", return_value=0):
                    self.assertEqual(task.run(), 1)

    def test_wpose_units_and_invalid_values(self):
        module_spec = importlib.util.spec_from_file_location(
            "stm_task2_test", Path(__file__).parent / "communications" / "stm.py")
        module = importlib.util.module_from_spec(module_spec)
        # Serial hardware is not needed to test the wire field conversion.
        with patch.dict("sys.modules", {"serial": Mock()}):
            module_spec.loader.exec_module(module)
        self.assertEqual(module.validate_token("!FAST1"), (True, ""))
        self.assertEqual(module.validate_token("!FAST0"), (True, ""))
        stm = module.STM.__new__(module.STM)
        stm.query_fields = Mock(return_value=["123", "-456", "-915"])
        self.assertEqual(stm.read_wpose(),
                         {"x_mm": 123, "y_mm": -456, "heading_deg": -91.5})
        for fields in (None, ["1", "2"], ["nan", "0", "0"]):
            stm.query_fields.return_value = fields
            self.assertIsNone(stm.read_wpose())

    def test_existing_android_status_grammar(self):
        self.assertEqual(android_status("START"), "STATUS,RUNNING,0,5")
        self.assertEqual(android_status("READY"), "STATUS,OK")
        self.assertEqual(android_status("DONE"), "STATUS,DONE")
        self.assertEqual(android_status("ARROW,1,LEFT"), "TASK2,ARROW,1,LEFT")

    def test_task2_readiness_query_covers_startup_and_active_run(self):
        self.assertEqual(task2_control_status(False, False), "TASK2,NOT_READY")
        self.assertEqual(task2_control_status(True, False), "TASK2,READY")
        self.assertEqual(task2_control_status(True, True), "TASK2,RUNNING")

    def test_stop_blocks_new_movement(self):
        raw, stop = Mock(), Event()
        motion = Motion(raw, stop, lambda: None)
        stop.set()
        with self.assertRaises(RuntimeError):
            motion.send_line("FR90")
        raw.send_line.assert_not_called()
        motion.abort()
        raw.abort.assert_called_once()

    def test_timeout_brakes(self):
        raw = Mock()
        raw.receive.return_value = None
        motion = Motion(raw, Event(), lambda: None)
        with self.assertRaises(RuntimeError):
            motion.wait_reply(timeout=0.03)
        raw.abort.assert_called_once()

    def test_fast_mode_uses_serialized_motion_link(self):
        raw = Mock()
        raw.set_fast.return_value = True
        motion = Motion(raw, Event(), lambda: None)
        self.assertTrue(motion.set_fast(True, timeout=1.5))
        raw.set_fast.assert_called_once_with(True, 1.5)

    def test_stage_two_vision_starts_after_first_obstacle_is_cleared(self):
        vision = Mock()
        status = Mock()
        task = Task2(Mock(), vision, Event(), status)
        with patch("task2.route.Task2.pass_obstacle1", return_value=142.0):
            self.assertEqual(task.pass_obstacle1(80.0, 1.0), 142.0)
        vision.begin.assert_not_called()
        self.assertFalse(task.stage2_started)
        task._lane_reached()
        vision.begin.assert_called_once_with(2)
        status.assert_any_call("BYPASS_1")
        status.assert_any_call("APPROACH_2")
        self.assertTrue(task.stage2_started)

        with patch("task2.route.Task2.photo_obstacle2", return_value=-1.0):
            task.photo_obstacle2(Mock(), 1.0)
        vision.begin.assert_called_once_with(2)

    def test_cancel_interrupts_arrow_wait(self):
        stop = Event()
        vision = Vision(Mock(), stop, "run")
        stop.set()
        with self.assertRaises(RuntimeError):
            vision.wait_arrow(8)

    def test_fresh_ignored_frames_extend_arrow_wait(self):
        vision = Vision(Mock(), Event(), "run")
        answer = []
        errors = []

        def wait():
            try:
                answer.append(vision.wait_arrow(.05))
            except Exception as exc:
                errors.append(exc)

        thread = Thread(target=wait)
        thread.start()
        # Keep the camera alive beyond the original total timeout. These
        # frames may be bullseyes/blanks; neither is an arrow decision.
        for frame in (b"bullseye-1", b"bullseye-2"):
            time.sleep(.035)
            vision.publish(frame)
        with vision.condition:
            vision.result = "right"
            vision.condition.notify_all()
        thread.join(1)

        self.assertEqual(answer, ["right"])
        self.assertEqual(errors, [])
        self.assertFalse(thread.is_alive())

    def test_socket_pipeline_two_votes_per_stage_and_evidence(self):
        left, right = socket.socketpair()
        left.settimeout(2)
        right.settimeout(2)
        link, stop, errors = Link(left), Event(), []
        vision = Vision(link, stop, "test-run")
        stitches = []
        detections = {1: 0, 2: 0}
        thresholds = {1: [], 2: []}
        def detector(path, allowed_classes, output_dir, selection_policy,
                     target_confidence):
            self.assertEqual(allowed_classes, {"left", "right"})
            output_dir.mkdir(exist_ok=True)
            (output_dir / path.name).write_bytes(path.read_bytes())
            stage = 1 if "stage1" in path.name else 2
            self.assertEqual(selection_policy, "nearest")
            thresholds[stage].append(target_confidence)
            detections[stage] += 1
            if detections[stage] == 1:
                # Even a high-confidence bullseye is only a marker.  It must
                # be discarded while the stationary camera keeps looking for
                # the two matching arrow votes below.
                return "bullseye", .99
            return ("left" if stage == 1 else "right", .95)
        def guarded(fn):
            try:
                fn()
            except Exception as exc:
                errors.append(exc)
        with tempfile.TemporaryDirectory() as directory:
            pc = Thread(target=lambda: guarded(lambda: server.serve(
                right, detector, directory, {"left": "left", "right": "right"},
                lambda saved, output: stitches.append(dict(saved)))))
            pi = Thread(target=lambda: guarded(vision.serve))
            pc.start()
            pi.start()
            try:
                for stage, expected in ((1, "left"), (2, "right")):
                    vision.begin(stage)
                    deadline = time.monotonic() + 3
                    while vision.result is None and time.monotonic() < deadline:
                        vision.publish(b"fake jpeg")
                        time.sleep(.025)
                    self.assertEqual(vision.wait_arrow(.1), expected)
                    vision.idle()
                self.assertEqual(set(stitches[-1]), {1, 2})
                self.assertEqual(len(stitches), 2)
                run = Path(directory)
                raw = sorted(run.glob("stage*_frame*.jpg"))
                annotated = sorted((run / "annotated").glob("stage*_frame*.jpg"))
                self.assertEqual(len(raw), 2)
                self.assertEqual(len(annotated), 2)
                self.assertEqual([path.name for path in raw],
                                 [path.name for path in annotated])
                self.assertEqual(detections, {1: 3, 2: 3})
                self.assertEqual(set(thresholds[1]), {0.70})
                self.assertEqual(set(thresholds[2]), {0.50})
            finally:
                stop.set()
                pi.join(1)
                left.shutdown(socket.SHUT_RDWR)
                pc.join(1)
                link.stream.close()
                left.close()
                right.close()
            self.assertFalse(pi.is_alive())
            self.assertFalse(pc.is_alive())
            self.assertEqual(errors, [])

    def test_stage_two_accepts_two_unique_votes_below_stage_one_threshold(self):
        pc = Mock()
        stop = Event()
        vision = Vision(pc, stop, "run")
        vision.begin(2)
        vision.publish(b"first")
        replies = ["READY,TASK2"]
        for frame in (1, 2):
            replies.append("ARROW," + json.dumps({
                "run_id": "run", "stage": 2, "frame_id": frame,
                "min_confidence": 0.50, "direction": "left",
                "confidence": 0.68,
            }))
        replies.append("SAVED,2")
        pc.receive.side_effect = replies

        thread = Thread(target=vision.serve)
        thread.start()
        deadline = time.monotonic() + 1
        while pc.send_image.call_count < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        vision.publish(b"second")
        deadline = time.monotonic() + 1
        while vision.result is None and time.monotonic() < deadline:
            time.sleep(0.01)
        stop.set()
        with vision.condition:
            vision.condition.notify_all()
        thread.join(1)

        self.assertEqual(vision.result, "left")
        self.assertFalse(thread.is_alive())
        requests = [json.loads(call.kwargs["header"].split(",", 1)[1])
                    for call in pc.send_image.call_args_list]
        self.assertEqual([request["min_confidence"] for request in requests],
                         [0.50, 0.50])

    def test_mismatched_response_rejected(self):
        pc = Mock()
        stop = Event()
        vision = Vision(pc, stop, "r")
        vision.begin(1)
        vision.publish(b"frame")
        pc.receive.side_effect = ["READY,TASK2", "ARROW," + json.dumps(
            {"run_id": "other-run", "stage": 1, "frame_id": 1,
             "direction": "left", "confidence": 1})]
        with self.assertRaisesRegex(RuntimeError, "mismatched"):
            vision.serve()
        self.assertIsNone(vision.result)


class SimulatedRoutes(unittest.TestCase):
    def test_obstacle1_left_recovery_from_measured_floor_pose(self):
        """The 2026-10-14 floor run needed more than the old R8 allowance."""
        import math
        import task2t as route
        import task2t_park as geometry

        pose = geometry.Pose(78.6, -2.3, math.radians(-2.6))
        x1 = 122.7
        lane = (geometry.OBS1_SIZE_CM / 2 + geometry.HALF_WIDTH_CM
                + route.OBS1_PASS_GAP_CM)
        course = geometry.Course(
            x1, x1 + geometry.OBS1_SIZE_CM + 60, route.CONSERVATIVE_W2,
            wall_y=route.CONSERVATIVE_WALL_Y)
        boxes = [box for box in course.boxes() if box.name != "back fence"]
        leads = [0] + list(range(-1, -route.OBS1_RECOVERY_REVERSE_CM - 1, -1))
        candidates = [
            route._straight(lead) + route.kick(lane - pose.y, geometry.RADIUS_CM)
            for lead in leads
        ]
        clear = [
            (geometry.path_clearance(
                geometry.run(pose, segments, geometry.RADIUS_CM), boxes)[0], segments)
            for segments in candidates
        ]
        clearance, segments = min(
            (item for item in clear if item[0] >= geometry.CLEARANCE_CM),
            key=lambda item: geometry._cost(item[1], geometry.RADIUS_CM))
        self.assertGreaterEqual(clearance, geometry.CLEARANCE_CM)
        self.assertTrue(segments[0].token().startswith("R"))

    def test_four_arrow_pairs_across_narrow_wide_and_close_courses(self):
        from task2 import Task2
        import task2t_park as geometry
        from task2t_sim import FakeSTM, run_clearance

        class Arrows:
            def __init__(self, pair):
                self.pair = iter(pair)

            def begin(self, stage):
                pass

            def idle(self):
                pass

            def wait_arrow(self, timeout):
                return next(self.pair)

        for d1, d2, width in ((60, 60, 30), (60, 60, 90),
                              (150, 150, 90), (90, 82, 38),
                              (68.7, 82.8, 71)):
            for pair in itertools.product(("left", "right"), repeat=2):
                with self.subTest(d1=d1, d2=d2, width=width, arrows=pair):
                    course = geometry.Course(d1, d1 + 10 + d2, width)
                    stm = FakeSTM(course, geometry.WorldFrame(None).start,
                                  geometry.RADIUS_CM)
                    task = Task2(stm, Arrows(pair), Event(), lambda phase: None)
                    with patch("time.sleep"), patch.object(task, "sleep_fn", lambda _: None):
                        self.assertEqual(task.run(), 0)
                    self.assertGreater(run_clearance(stm, course)[0],
                                       geometry.CLEARANCE_FLOOR_CM)

    def test_early_vision_pairs_across_narrow_wide_and_close_courses(self):
        import task2t_park as geometry
        from task2t_sim import FakeSTM, run_clearance

        class EarlyArrows:
            def __init__(self, pair):
                self.pair, self.stage = pair, None

            def begin(self, stage):
                self.stage = stage

            def idle(self):
                self.stage = None

            def peek_arrow(self):
                return self.pair[self.stage - 1] if self.stage is not None else None

            def wait_arrow(self, timeout):
                return self.peek_arrow()

        for d1, d2, width in ((60, 60, 30), (60, 60, 90),
                              (150, 150, 90), (90, 82, 38),
                              (68.7, 82.8, 71)):
            for pair in itertools.product(("left", "right"), repeat=2):
                with self.subTest(d1=d1, d2=d2, width=width, arrows=pair):
                    course = geometry.Course(d1, d1 + 10 + d2, width)
                    stm = FakeSTM(course, geometry.WorldFrame(None).start,
                                  geometry.RADIUS_CM)
                    task = Task2(stm, EarlyArrows(pair), Event(), lambda phase: None)
                    with patch("time.sleep"), patch.object(task, "sleep_fn", lambda _: None):
                        self.assertEqual(task.run(), 0)
                    self.assertGreater(run_clearance(stm, course)[0],
                                       geometry.CLEARANCE_FLOOR_CM)

    def test_all_arrow_pairs_with_left_ir_reading_three_cm_short(self):
        import task2t_park as geometry
        from task2t_sim import FakeSTM, run_clearance

        class ShortLeftIR(FakeSTM):
            def query_fields(self, command, timeout=2.0):
                fields = super().query_fields(command, timeout)
                if command.upper().lstrip("?") == "IR" and fields:
                    value = int(fields[0])
                    if value != 65535:
                        fields[0] = str(max(1, value - 3))
                return fields

        class Arrows:
            def __init__(self, pair):
                self.pair = iter(pair)

            def begin(self, stage):
                pass

            def idle(self):
                pass

            def wait_arrow(self, timeout):
                return next(self.pair)

        for width in (30, 70, 90):
            for pair in itertools.product(("left", "right"), repeat=2):
                with self.subTest(width=width, arrows=pair):
                    course = geometry.Course(72.7, 165.1, width)
                    stm = ShortLeftIR(course, geometry.WorldFrame(None).start,
                                      geometry.RADIUS_CM)
                    task = Task2(stm, Arrows(pair), Event(), lambda phase: None)
                    with patch("time.sleep"), patch.object(task, "sleep_fn", lambda _: None):
                        self.assertEqual(task.run(), 0)
                    self.assertGreater(run_clearance(stm, course)[0],
                                       geometry.CLEARANCE_FLOOR_CM)
                    self.assertTrue(all(-geometry.BAY_DEPTH_CM < x < 0
                                        and abs(y) < geometry.BAY_HALF_WIDTH_CM
                                        for x, y in geometry.footprint(stm.pose)), stm.pose)

    def test_four_arrow_pairs_survive_stale_ir_edges_and_park(self):
        """Exercise the complete mission with the intermittent IR seen on the floor."""
        import task2t as route
        import task2t_park as geometry
        from task2t_sim import FakeSTM, run_clearance

        class Arrows:
            def __init__(self, pair):
                self.pair = iter(pair)

            def begin(self, stage):
                pass

            def idle(self):
                pass

            def wait_arrow(self, timeout):
                return next(self.pair)

        class StaleEdgeSTM(FakeSTM):
            """After an edge, occasionally repeat the last real side-IR echo."""
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.last_ir = [None, None]
                self.miss_count = [0, 0]

            def query_fields(self, cmd, timeout=2.0):
                fields = super().query_fields(cmd, timeout)
                if cmd.upper().lstrip("?") != "IR" or fields is None:
                    return fields
                noisy = list(fields)
                for index, value in enumerate(fields):
                    reading = float(value)
                    maximum = (geometry.IR_MAX_GAP_CM["left"] if index == 0
                               else geometry.IR_MAX_GAP_CM["right"])
                    if reading != 65535 and reading <= maximum:
                        self.last_ir[index] = value
                        self.miss_count[index] = 0
                        continue
                    if self.last_ir[index] is None:
                        continue
                    self.miss_count[index] += 1
                    # Real trace after the edge: blank, stale echo, blank, blank.
                    # Two of the latest three still say that the face ended.
                    if self.miss_count[index] % 4 == 2:
                        noisy[index] = self.last_ir[index]
                return noisy

        expected_phases = ["APPROACH_1", "BYPASS_1", "APPROACH_2",
                           "ROUND_2", "RETURN_AND_PARK"]
        for pair in itertools.product(("left", "right"), repeat=2):
            with self.subTest(arrows=pair):
                course = geometry.Course(90, 182, 50)
                stm = StaleEdgeSTM(course, geometry.WorldFrame(None).start,
                                   geometry.RADIUS_CM)
                phases = []
                task = Task2(stm, Arrows(pair), Event(), phases.append)
                with patch("time.sleep"), patch.object(
                        task, "sleep_fn", lambda _: None):
                    self.assertEqual(task.run(), 0)
                mission_phases = []
                for phase in phases:
                    if (phase in expected_phases
                            and (not mission_phases or phase != mission_phases[-1])):
                        mission_phases.append(phase)
                self.assertEqual(mission_phases, expected_phases)
                clearance, obstacle = run_clearance(stm, course)
                self.assertGreater(clearance, geometry.CLEARANCE_FLOOR_CM,
                                   obstacle)
                corners = geometry.footprint(stm.pose)
                self.assertTrue(all(-geometry.BAY_DEPTH_CM < x < 0
                                    and abs(y) < geometry.BAY_HALF_WIDTH_CM
                                    for x, y in corners), stm.pose)


if __name__ == "__main__":
    unittest.main()
