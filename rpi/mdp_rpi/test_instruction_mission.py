"""Offline tests of per-instruction execution and continuous pose feedback."""

import sys
import json
import struct
import types
import unittest
from pathlib import Path
from threading import Event, Lock, Thread
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
for name, attrs in {
    "serial": {"Serial": object, "SerialException": Exception},
    "bluetooth": {"BluetoothSocket": object, "BluetoothError": Exception,
                  "RFCOMM": 0, "PORT_ANY": 0, "SERIAL_PORT_CLASS": None,
                  "SERIAL_PORT_PROFILE": None, "advertise_service": lambda *a, **k: None},
    "picamera": {"PiCamera": object},
    "dotenv": {"load_dotenv": lambda: None},
}.items():
    if name not in sys.modules:
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        sys.modules[name] = module

from communications.stm import classify_reply, validate_token
from communications.pc import PC
from instruction_mission import InstructionMission, arena_grid_position, parse_start_pose
from feedback_control import FeedbackControl
with patch("run_log.setup"):
    from task1 import Task1


class FakeSTM:
    def __init__(self, poses, replies):
        self.poses = iter(poses)
        self.replies = iter(replies)
        self.events = []
        self.awaiting_reply = False

    def query_fields(self, query):
        self.events.append(query)
        return next(self.poses)

    def send_line(self, tokens):
        if self.awaiting_reply:
            return False
        self.events.append(tuple(tokens))
        self.awaiting_reply = True
        return True

    def wait_reply(self):
        self.awaiting_reply = False
        return next(self.replies)


class EndLoop(BaseException):
    pass


START = {"x": 2, "y": 2, "dir": "N"}


class InstructionTests(unittest.TestCase):
    def test_turn_always_pauses_two_seconds_before_gyro_queries(self):
        task = Task1.__new__(Task1)
        task.gyro_settle_enabled = True
        task.gyro_settle_timeout = 1.0
        task.gyro_settle_samples = 3
        task.gyro_settle_rate_dps = 1.0
        task.stm = Mock()
        task.stm.query_fields.return_value = ["1", "0", "0"]
        events = Mock()
        events.attach_mock(task.stm.query_fields, "query")
        with patch("task1.sleep") as pause:
            events.attach_mock(pause, "pause")
            task._wait_for_gyro_settle("FR45")
        self.assertEqual(events.mock_calls[0], unittest.mock.call.pause(2.0))
        self.assertEqual(task.stm.query_fields.call_count, 3)

    def test_photo_motion_rejects_range_increase_after_forward_move(self):
        task = Task1.__new__(Task1)
        task.us_adjust_retries = 3
        task.us_adjust_tolerance_cm = 3.0
        task.us_adjust_max_step_cm = 30
        task.us_settle_s = 0
        task.camera_to_sensor_cm = 10.5
        task._read_ultrasonic_cm = Mock(side_effect=[31, 40])
        task._execute_photo_adjustment = Mock(return_value=True)
        self.assertFalse(task._move_for_photo(Mock(), "2", 30))
        self.assertTrue(task.photo_alignment_needed)
        self.assertEqual(task._execute_photo_adjustment.call_count, 1)

    def test_fine_turn_checkpoints_are_valid_mission_instructions(self):
        mission = InstructionMission(
            [["FR15", "FL30", "RR15", "RL30", "S"]], START
        )
        self.assertEqual(
            mission.segments[0], ["FR15", "FL30", "RR15", "RL30", "S"]
        )

    def test_ten_cm_cells_cover_twenty_by_twenty_arena(self):
        for mm, expected in [(0, 0), (99.9, .999), (100, 1), (200, 2),
                             (299, 2.99), (1100, 11), (1950, 19.5), (1999, 19.99)]:
            with self.subTest(mm=mm):
                self.assertAlmostEqual(arena_grid_position(mm), expected)
        self.assertEqual(arena_grid_position(200 - 1e-12), 2)

    def test_open_floor_overhang_is_reported_not_clamped(self):
        for mm, expected in [(-1, -.01), (2000, 20), (2100, 21)]:
            with self.subTest(mm=mm):
                self.assertEqual(arena_grid_position(mm), expected)
        for mm in (float("nan"), float("inf")):
            with self.subTest(mm=mm), self.assertRaises(ValueError):
                arena_grid_position(mm)

    def test_fractional_coordinates_sent_after_each_instruction(self):
        stm = FakeSTM([["0", "0", "0"], ["125", "-75", "0"],
                       ["115", "-75", "0"]], ["OK", "OK"])
        mission = InstructionMission([["F10", "R1"]], START)
        for expected in [("ROBOT,2.75,3.25,0", None), ("ROBOT,2.75,3.15,0", 0)]:
            mission.send_next(stm)
            stm.wait_reply()
            self.assertEqual(mission.complete_instruction(stm), expected)

    def test_heading_is_north_zero_clockwise_and_preserves_tenths(self):
        for measured, expected in [(0, "0"), (-900, "90"), (900, "270"),
                                   (1800, "180"), (-1800, "180"),
                                   (-26, "2.6"), (26, "357.4")]:
            with self.subTest(measured=measured):
                stm = FakeSTM([["0", "0", "0"], ["0", "0", str(measured)]], ["OK"])
                mission = InstructionMission([["S"]], START)
                mission.send_next(stm)
                stm.wait_reply()
                self.assertEqual(mission.complete_instruction(stm),
                                 (f"ROBOT,2,2,{expected}", 0))

    def test_heading_wraps_relative_to_board_origin(self):
        stm = FakeSTM([["0", "0", "1790"], ["0", "0", "-1790"]], ["OK"])
        mission = InstructionMission([["FL2"]], START)
        mission.send_next(stm)
        stm.wait_reply()
        self.assertEqual(mission.complete_instruction(stm), ("ROBOT,2,2,358", 0))

    def test_wpose_is_query_not_ack(self):
        self.assertTrue(validate_token("?WPOSE")[0])
        self.assertEqual(classify_reply("WPOSE,100,0,-900"), "query")
        self.assertEqual(classify_reply("OK"), "movement")

    def test_progress_identifies_completed_instruction_and_next_at_boundaries(self):
        mission = InstructionMission([["F10", "S"], ["R1"]], START)
        self.assertIsNone(mission.progress)
        stm = FakeSTM([["0", "0", "0"]] + [["125", "-75", "-26"]] * 3, ["OK"] * 3)
        for si, ii, token, next_si, next_ii, segment_done, plan_done in [
            (0, 0, "F10", 0, 1, False, False),
            (0, 1, "S", 1, 0, True, False),
            (1, 0, "R1", None, None, True, True),
        ]:
            mission.send_next(stm)
            stm.wait_reply()
            mission.complete_instruction(stm)
            self.assertEqual(mission.progress, {
                "event": "instruction_completed",
                "segment_index": si, "instruction_index": ii, "token": token,
                "segment": mission.segments[si],
                "segment_completed": segment_done, "motion_plan_completed": plan_done,
                "next_segment_index": next_si, "next_instruction_index": next_ii,
                "x_grid": 2.75, "y_grid": 3.25, "heading_deg": 2.6,
            })

    def test_progress_segment_is_snapshot_not_live_execution_list(self):
        mission = InstructionMission([["F10", "F10", "S"]], START)
        stm = FakeSTM([["0", "0", "0"]] * 3, ["OK"] * 2)
        mission.send_next(stm)
        stm.wait_reply()
        mission.complete_instruction(stm)
        first = mission.progress
        self.assertIsNot(first["segment"], mission.segments[0])
        mission.send_next(stm)
        stm.wait_reply()
        mission.complete_instruction(stm)
        self.assertEqual(first["instruction_index"], 0)
        self.assertEqual(mission.progress["instruction_index"], 1)
        self.assertEqual(first["segment"], ["F10", "F10", "S"])
        self.assertNotIn("execution_id", mission.progress)

    def test_transform_rotated_origin_and_reverse(self):
        stm = FakeSTM([["100", "50", "900"], ["100", "250", "900"],
                       ["100", "150", "900"]], ["OK", "OK"])
        mission = InstructionMission([["F20", "R10"]], START)
        self.assertTrue(mission.send_next(stm))
        self.assertFalse(mission.send_next(stm))
        stm.wait_reply()
        self.assertEqual(mission.complete_instruction(stm), ("ROBOT,2,4,0", None))
        mission.send_next(stm)
        stm.wait_reply()
        self.assertEqual(mission.complete_instruction(stm), ("ROBOT,2,3,0", 0))
        self.assertTrue(mission.done)
        self.assertFalse(mission.send_next(stm))

    def test_turn_wrap_and_forward_displacement(self):
        stm = FakeSTM([["0", "0", "0"], ["300", "-300", "-900"],
                       ["300", "-500", "-900"], ["300", "-500", "-900"]],
                      ["OK"] * 3)
        mission = InstructionMission([["FR90", "F20", "S"]], START)
        expected = [("ROBOT,5,5,90", None), ("ROBOT,7,5,90", None),
                    ("ROBOT,7,5,90", 0)]
        for result in expected:
            mission.send_next(stm)
            stm.wait_reply()
            self.assertEqual(mission.complete_instruction(stm), result)

    def test_missing_pose_prevents_first_move(self):
        stm = FakeSTM([None], [])
        with self.assertRaises(ValueError):
            InstructionMission([["F20"]], START).send_next(stm)
        self.assertEqual(stm.events, ["?WPOSE"])

    def test_missing_pose_after_ok_blocks_progress(self):
        stm = FakeSTM([["0", "0", "0"], ["invalid"]], ["OK"])
        mission = InstructionMission([["F20", "F30"]], START)
        mission.send_next(stm)
        stm.wait_reply()
        with self.assertRaises(ValueError):
            mission.complete_instruction(stm)
        self.assertTrue(mission.pending)
        self.assertFalse(mission.send_next(stm))

    def test_all_segments_validated_before_motion(self):
        for segments in ([], [["F10"], ["RST"]], [["F10"], []], ["F10"],
                         [["F10"], ["bad"]]):
            with self.subTest(segments=segments), self.assertRaises(ValueError):
                InstructionMission(segments, START)

    def test_planner_start_is_required_instead_of_defaulting(self):
        for start in (None, {}, {"x": 0, "y": 0},
                      {"x": float("nan"), "y": 0, "dir": "N"},
                      {"x": 20, "y": 0, "dir": "N"}):
            with self.subTest(start=start), self.assertRaises(ValueError):
                InstructionMission([["S"]], start)

    def test_pulled_planner_bottom_left_start_anchors_robot_updates(self):
        stm = FakeSTM([["0", "0", "0"], ["125", "-75", "-26"]], ["OK"])
        mission = InstructionMission([["F10"]], {"x": 0, "y": 0, "dir": "N"})
        mission.send_next(stm)
        stm.wait_reply()
        self.assertEqual(mission.complete_instruction(stm), ("ROBOT,0.75,1.25,2.6", 0))

    def test_fractional_east_start_rotates_odometry_without_cardinal_snapping(self):
        stm = FakeSTM([["10", "20", "0"], ["110", "20", "-100"]], ["OK"])
        mission = InstructionMission([["F10"]], {"x": 1.25, "y": 3.5, "dir": 90})
        mission.send_next(stm)
        stm.wait_reply()
        self.assertEqual(mission.complete_instruction(stm), ("ROBOT,2.25,3.5,100", 0))

    def test_task1_retries_one_token_and_photographs_only_at_boundary(self):
        task = Task1.__new__(Task1)
        task.segments = [["FR90", "F20", "S"], ["R10", "S"]]
        task.segment_obstacles = ["7", None]
        task.obstacle_order = ["7"]
        task.segments_index = 0
        task.instruction_mission = None
        task.start_pose = dict(START)
        task.odometry_start = dict(START)
        task.feedback_control = FeedbackControl()
        task._completed_photo_count = 0
        task._idx_lock = Lock()
        task._resend_counts = {}
        task.max_resends = 3
        task.segment_delay = 0
        task.capture_settle = 0
        task.halted = False
        task.started = True
        task.a5_mode = False
        task._read_ultrasonic_cm = Mock(return_value=30)
        task.stm = FakeSTM([["0", "0", "0"]] + [["300", "-300", "-900"]] * 5,
                           ["OK", "RESEND", "OK", "OK", "OK", "OK"])
        events = task.stm.events
        task.android = Mock()
        task.android.send.side_effect = events.append
        task.pc = Mock()
        task._detect_and_send_image = lambda oid: events.append(("photo", oid)) or True
        task._halt_mission = Mock()
        self.assertTrue(task._send_next_segment())

        def stop_idle(_):
            raise EndLoop()

        with patch("task1.sleep", side_effect=stop_idle), self.assertRaises(EndLoop):
            task.stm_receive()
        self.assertEqual([e for e in events if isinstance(e, tuple)],
                         [("FR90",), ("F20",), ("F20",), ("S",),
                          ("photo", "7"), ("R10",), ("S",)])
        self.assertEqual(sum(isinstance(e, str) and e.startswith("ROBOT,") for e in events), 5)
        first = events.index(("FR90",))
        self.assertEqual(events[first + 2:first + 4], ["?WPOSE", "ROBOT,5,5,90"])
        self.assertEqual(events[-1], "STATUS,DONE")
        pc_messages = [call.args[0] for call in task.pc.send.call_args_list]
        self.assertEqual(len(pc_messages), 6)
        self.assertEqual(pc_messages[-1], "STITCH,1\n")
        self.assertTrue(all(m.startswith("PROGRESS,") for m in pc_messages[:-1]))
        progress = [json.loads(m.split(",", 1)[1]) for m in pc_messages[:-1]]
        self.assertEqual([(p["x_grid"], p["y_grid"], p["heading_deg"]) for p in progress],
                         [(5, 5, 90)] * 5)
        self.assertEqual([(p["segment_index"], p["instruction_index"], p["token"]) for p in progress],
                         [(0, 0, "FR90"), (0, 1, "F20"), (0, 2, "S"), (1, 0, "R10"), (1, 1, "S")])
        self.assertTrue(all("execution_id" not in p for p in progress))
        self.assertEqual([p["segment"] for p in progress],
                         [task.segments[0]] * 3 + [task.segments[1]] * 2)
        self.assertTrue(progress[-1]["motion_plan_completed"])
        self.assertFalse(any(c.args[0].startswith("PROGRESS,") for c in task.android.send.call_args_list))
        task._halt_mission.assert_not_called()

    def test_task1_passes_planner_start_to_android_unchanged(self):
        task = Task1.__new__(Task1)
        task._idx_lock = Lock()
        task.start_pose = {"x": 0.25, "y": 0.5, "dir": 12.5}
        task.android = Mock()
        task._halt_mission = Mock()
        task._send_start_status()
        task.android.send.assert_called_once_with("STATUS,START,0.25,0.5,12.5")
        task._halt_mission.assert_not_called()

    def test_android_display_start_does_not_control_odometry_origin(self):
        task = Task1.__new__(Task1)
        task.segments = [["FR90"]]
        task.segments_index = 0
        task.instruction_mission = None
        task.start_pose = {"x": 0, "y": 0, "dir": "N"}
        task.odometry_start = {"x": 2, "y": 2, "dir": "N"}
        task.feedback_control = FeedbackControl()
        task._idx_lock = Lock()
        task.halted = False
        task.stm = FakeSTM([["0", "0", "0"]], [])
        task.android = Mock()

        self.assertTrue(task._send_next_segment())
        self.assertEqual(task.instruction_mission.start_pose["x"], 2)
        self.assertEqual(task.instruction_mission.start_pose["y"], 2)
        self.assertEqual(task.stm.events, ["?WPOSE", ("FR90",)])

    def test_android_lowercase_stop_aborts_active_mission(self):
        task = Task1.__new__(Task1)
        task.started = True
        task.halted = False
        task.android = Mock()
        task.android.receive.side_effect = ["s", EndLoop()]
        task.stm = Mock()
        task._halt_mission = Mock()

        with self.assertRaises(EndLoop):
            task.android_receive()

        task.stm.abort.assert_called_once_with()
        task._halt_mission.assert_called_once_with("operator requested emergency stop")

    def test_task1_refuses_to_invent_a_start_pose(self):
        task = Task1.__new__(Task1)
        task._idx_lock = Lock()
        task.start_pose = None
        task.android = Mock()
        task._halt_mission = Mock()
        task._send_start_status()
        task.android.send.assert_not_called()
        task._halt_mission.assert_called_once_with("Cannot start without planner PATH.start")


class PCTransferTests(unittest.TestCase):
    def test_image_header_and_payload_cannot_be_interrupted_by_pose(self):
        pc = PC()
        header_sent = Event()
        pose_started = Event()
        chunks = []

        def sendall(data):
            chunks.append(data)
            if data == b"DETECT,7\n":
                header_sent.set()
                self.assertTrue(pose_started.wait(2))
                self.assertFalse(pc._send_lock.acquire(blocking=False))

        pc.client_socket = Mock()
        pc.client_socket.sendall.side_effect = sendall

        def send_pose():
            if header_sent.wait(2):
                pose_started.set()
                pc.send("ROBOT,2.75,3.25,2.6")

        sender = Thread(target=send_pose, daemon=True)
        sender.start()
        try:
            pc.send_image(b"jpeg", header="DETECT,7")
        finally:
            sender.join(3)
        self.assertFalse(sender.is_alive())
        self.assertEqual(chunks, [b"DETECT,7\n", struct.pack(">I", 4), b"jpeg",
                                  b"ROBOT,2.75,3.25,2.6\n"])

    def test_legacy_image_call_has_no_text_header(self):
        pc = PC()
        pc.client_socket = Mock()
        pc.send_image(b"jpeg")
        self.assertEqual([c.args[0] for c in pc.client_socket.sendall.call_args_list],
                         [struct.pack(">I", 4), b"jpeg"])

    def test_capture_sends_detect_and_image_together(self):
        task = Task1.__new__(Task1)
        task.detect_retries = 0
        task.detect_timeout = 1
        task.camera = Mock()
        task.camera.capture_image.return_value = b"jpeg"
        task.pc = Mock()
        task.image_done = Mock()
        task._detection_lock = Lock()
        task._expected_detection_id = None
        task._last_detection = None
        task._last_capture_attempts = 0

        def receive_object(timeout):
            task._last_detection = {
                "obstacle_id": "7", "confidence": .9, "class_id": "7",
            }
            return True

        task.image_done.wait.side_effect = receive_object
        result = task._detect_and_send_image("7")
        self.assertEqual(result["class_id"], "7")
        task.pc.send_image.assert_called_once_with(b"jpeg", header="DETECT,7")
        task.pc.send.assert_not_called()

    def test_low_confidence_retries_safe_distances_and_restores_plan_pose(self):
        task = Task1.__new__(Task1)
        task.a5_mode = False
        task.photo_standoffs = {"3": [20, 30, 33, 35, 40, 45]}
        task.selected_standoffs = {"3": 30}
        task.ultrasonic_adjustments = {"3": True}
        task.photo_retry_standoffs = [20, 33, 35, 40, 45]
        task.photo_retry_limit = 5
        task.capture_settle = 0
        task.halted = False
        task._detect_and_send_image = Mock(side_effect=[
            {"obstacle_id": "3", "confidence": 0.0, "class_id": "NONE"},
            {"obstacle_id": "3", "confidence": 0.0, "class_id": "NONE"},
            {"obstacle_id": "3", "confidence": 0.82, "class_id": "C"},
        ])
        task._move_for_photo = Mock(return_value=True)

        result = task._detect_with_distance_retry("3", Mock())

        self.assertEqual(result["class_id"], "C")
        self.assertEqual(
            [call.args[2] for call in task._move_for_photo.call_args_list],
            [30, 20, 33],
        )
        self.assertEqual(task._detect_and_send_image.call_count, 3)
        self.assertEqual(
            [call.args[1] for call in task._detect_and_send_image.call_args_list],
            [30, 20, 33],
        )

    def test_wall_facing_photo_does_not_invent_an_unsafe_retry_pose(self):
        task = Task1.__new__(Task1)
        task.a5_mode = False
        task.photo_standoffs = {"1": [20]}
        task.selected_standoffs = {"1": 20}
        task.ultrasonic_adjustments = {"1": False}
        task.photo_retry_standoffs = [20, 33, 35, 40, 45]
        task.photo_retry_limit = 5
        task.capture_settle = 0
        task._detect_and_send_image = Mock(return_value={
            "obstacle_id": "1", "confidence": 0.0, "class_id": "NONE",
        })
        task._move_for_photo = Mock(return_value=True)

        task._detect_with_distance_retry("1", Mock())

        task._move_for_photo.assert_not_called()

    def test_oblique_retry_moves_to_nearest_safe_distance_not_twenty_cm(self):
        task = Task1.__new__(Task1)
        task.a5_mode = False
        task.photo_standoffs = {"8": [25, 30, 35]}
        task.selected_standoffs = {"8": 35}
        task.ultrasonic_adjustments = {"8": False}
        task.photo_retry_standoffs = [20, 25, 30, 33, 35, 40, 45]
        task.photo_retry_limit = 5
        task.capture_settle = 0
        task.halted = False
        task._detect_and_send_image = Mock(side_effect=[
            {"obstacle_id": "8", "confidence": 0.0, "class_id": "NONE"},
            {"obstacle_id": "8", "confidence": 0.9, "class_id": "8"},
        ])
        task._move_for_photo = Mock(return_value=True)

        result = task._detect_with_distance_retry("8", Mock())

        self.assertEqual(result["class_id"], "8")
        task._move_for_photo.assert_called_once()
        self.assertEqual(task._move_for_photo.call_args.args[2], 30)

    def test_photo_adjustment_reports_pose_without_advancing_plan_indexes(self):
        mission = InstructionMission([["S"]], START)
        mission.origin = (0, 0, 0)
        stm = Mock()
        stm.send_line.return_value = True
        stm.wait_reply.return_value = "OK"
        stm.query_fields.side_effect = [["50"], ["0", "0", "0"], ["30"]]
        task = Task1.__new__(Task1)
        task.stm = stm
        task.android = Mock()
        task.pc = Mock()
        task.max_resends = 3
        task.us_adjust_retries = 3
        task.us_adjust_tolerance_cm = 3.0
        task.us_adjust_max_step_cm = 30
        task.us_settle_s = 0
        task.halted = False
        task._halt_mission = Mock()

        self.assertTrue(task._move_for_photo(mission, "3", 40))

        self.assertEqual(mission.key, (0, 0))
        self.assertFalse(mission.pending)
        stm.send_line.assert_called_once_with(["F20"])
        task.android.send.assert_called_once_with("ROBOT,2,2,0")
        photo_progress = task.pc.send.call_args.args[0]
        self.assertTrue(photo_progress.startswith("PHOTO_PROGRESS,"))
        payload = json.loads(photo_progress.split(",", 1)[1])
        self.assertEqual(payload["target_standoff_cm"], 40)
        self.assertEqual(payload["measured_us_cm_before"], 50)

    def test_successful_initial_photo_adjustment_is_undone_before_next_segment(self):
        task = Task1.__new__(Task1)
        task.a5_mode = False
        task.photo_standoffs = {"3": [30]}
        task.selected_standoffs = {"3": 30}
        task.ultrasonic_adjustments = {"3": True}
        task.photo_retry_standoffs = [20, 40]
        task.photo_retry_limit = 2
        task.capture_settle = 0
        task.halted = False
        task._detect_and_send_image = Mock(return_value={
            "obstacle_id": "3", "confidence": 0.9, "class_id": "C",
        })
        movements = []

        def move(_mission, _obstacle, _distance, **kwargs):
            kwargs["movement_tokens"].append("F10")
            return True

        task._move_for_photo = Mock(side_effect=move)
        task._restore_planned_photo_pose = Mock(return_value=True)
        mission = Mock()

        task._detect_with_distance_retry("3", mission)

        task._restore_planned_photo_pose.assert_called_once_with(
            mission, "3", 30, ["F10"],
        )

    def test_photo_pose_restore_inverts_every_move_in_reverse_order(self):
        task = Task1.__new__(Task1)
        task._execute_photo_adjustment = Mock(return_value=True)
        mission = Mock()

        restored = task._restore_planned_photo_pose(
            mission, "3", 30, ["F10", "R5", "F2"],
        )

        self.assertTrue(restored)
        self.assertEqual(
            [call.args[2] for call in task._execute_photo_adjustment.call_args_list],
            ["R2", "F5", "R10"],
        )

    def test_photo_adjustment_reverses_when_too_close(self):
        mission = InstructionMission([["S"]], START)
        mission.origin = (0, 0, 0)
        task = Task1.__new__(Task1)
        task.stm = Mock()
        task.stm.send_line.return_value = True
        task.stm.wait_reply.return_value = "OK"
        task.stm.query_fields.side_effect = [["20"], ["0", "0", "0"], ["30"]]
        task.android = Mock()
        task.pc = Mock()
        task.max_resends = 3
        task.us_adjust_retries = 3
        task.us_adjust_tolerance_cm = 3.0
        task.us_adjust_max_step_cm = 30
        task.us_settle_s = 0
        task.halted = False
        task._halt_mission = Mock()

        self.assertTrue(task._move_for_photo(mission, "3", 40))
        task.stm.send_line.assert_called_once_with(["R10"])

    def test_photo_adjustment_refuses_implausible_echo_without_moving(self):
        task = Task1.__new__(Task1)
        task.stm = Mock()
        task.stm.query_fields.return_value = ["120"]
        task.android = Mock()
        task.pc = Mock()
        task.max_resends = 3
        task.us_adjust_retries = 3
        task.us_adjust_tolerance_cm = 3.0
        task.us_adjust_max_step_cm = 30
        task.us_settle_s = 0
        task.halted = False
        task._halt_mission = Mock()

        self.assertFalse(task._move_for_photo(Mock(), "3", 40))
        task.stm.send_line.assert_not_called()
        task._halt_mission.assert_not_called()

    def test_task1_rejects_fu_in_initial_or_replacement_mission(self):
        task = Task1.__new__(Task1)
        task.a5_mode = False
        with self.assertRaisesRegex(ValueError, "may not contain FU"):
            task._validate_task1_mission(InstructionMission([["FU30", "S"]], START))

        task.a5_mode = True
        mission = InstructionMission([["FU30", "S"]], START)
        self.assertIs(task._validate_task1_mission(mission), mission)


if __name__ == "__main__":
    unittest.main()
