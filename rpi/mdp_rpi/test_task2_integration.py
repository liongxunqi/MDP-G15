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
    def test_completed_parking_does_not_require_exact_target_or_centre(self):
        import math
        import task2t_park as geometry
        stm = Mock()
        stm.query_fields.return_value = ["1", "-5", "-2", "0", "6225"]
        task = Task2(stm, Mock(), Event(), Mock())
        task.frame.pose = Mock(return_value=geometry.Pose(-21.9, 4.6, math.radians(-180.1)))
        task._us = Mock(return_value=21.7)
        task._ir = Mock(side_effect=[31.0, 19.0])
        with patch("task2.route.Task2.run", return_value=0):
            self.assertEqual(task.run(), 0)

    def test_parking_rejects_unsafe_missing_or_moving_measurements(self):
        import math
        import task2t_park as geometry
        for us, left, right, rate, x in [(None, 31, 19, 0, -22),
                                       (1, 31, 19, 0, -22),
                                       (30, 31, 19, 0, -22),
                                       (21.7, None, 19, 0, -22),
                                       (21.7, 48, 2, 0, -22),
                                       (21.7, 31, 19, 30, -22),
                                       (21.7, 31, 19, 0, 5)]:
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

    def test_socket_pipeline_two_votes_per_stage_and_evidence(self):
        left, right = socket.socketpair()
        left.settimeout(2)
        right.settimeout(2)
        link, stop, errors = Link(left), Event(), []
        vision = Vision(link, stop, "test-run")
        stitches = []
        detections = {1: 0, 2: 0}
        def detector(path, allowed_classes, output_dir):
            self.assertEqual(allowed_classes, {"left", "right"})
            output_dir.mkdir(exist_ok=True)
            (output_dir / path.name).write_bytes(path.read_bytes())
            stage = 1 if "stage1" in path.name else 2
            detections[stage] += 1
            if detections[stage] == 1:
                return None, .2
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
                              (150, 150, 90), (90, 82, 38)):
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
                              (150, 150, 90), (90, 82, 38)):
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


if __name__ == "__main__":
    unittest.main()
