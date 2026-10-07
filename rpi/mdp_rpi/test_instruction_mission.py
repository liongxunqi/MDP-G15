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
    def test_ten_cm_cells_cover_twenty_by_twenty_arena(self):
        for mm, expected in [(0, 0), (99.9, .999), (100, 1), (200, 2),
                             (299, 2.99), (1100, 11), (1950, 19.5), (1999, 19.99)]:
            with self.subTest(mm=mm):
                self.assertAlmostEqual(arena_grid_position(mm), expected)
        self.assertEqual(arena_grid_position(200 - 1e-12), 2)

    def test_outside_arena_is_not_clamped_to_a_valid_cell(self):
        for mm in (-1, 2000, 2100, float("nan"), float("inf")):
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

    def test_turn_wrap_and_full_fu_displacement(self):
        stm = FakeSTM([["0", "0", "0"], ["300", "-300", "-900"],
                       ["300", "-500", "-900"], ["300", "-500", "-900"]],
                      ["OK"] * 3)
        mission = InstructionMission([["FR90", "FU30", "S"]], START)
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
        task.feedback_control = FeedbackControl()
        task._completed_photo_count = 0
        task._idx_lock = Lock()
        task._resend_counts = {}
        task.max_resends = 3
        task.segment_delay = 0
        task.halted = False
        task.started = True
        task.a5_mode = False
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
        self.assertEqual(pc_messages[-1], "STITCH,0\n")
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
        task.image_done.wait.return_value = True
        self.assertTrue(task._detect_and_send_image("7"))
        task.pc.send_image.assert_called_once_with(b"jpeg", header="DETECT,7")
        task.pc.send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
