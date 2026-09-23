"""Offline checks for the current standalone Task A5 configuration."""

import os
import struct
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
os.environ["A5_STANDOFF_CM"] = "45"
os.environ["A5_ORBIT_INSET_CM"] = "20"
os.environ["A5_CAPTURE_SETTLE_S"] = "2.0"
os.environ["A5_ARC_PROFILE"] = "1"
os.environ["A5_REAR_AXLE_TO_SENSOR_CM"] = "23"
os.environ["A5_ORBIT_ADVANCE_CORRECTION_CM"] = "10"
os.environ["A5_ORBIT_CLEARANCE_CM"] = "15"
os.environ["A5_ORBIT_FR_DEG"] = "90"
os.environ["A5_ORBIT_FL_DEG"] = "90"
os.environ["A5_ORBIT_FR_COMMAND_OFFSET_DEG"] = "0"
os.environ["A5_ORBIT_FL_COMMAND_OFFSET_DEG"] = "0"
os.environ["A5_ORBIT_RR_COMMAND_OFFSET_DEG"] = "0"
os.environ["A5_ORBIT_RL_COMMAND_OFFSET_DEG"] = "0"
os.environ["A5_ORBIT_HEADING_TOLERANCE_DEG"] = "2"
os.environ["A5_ORBIT_MAX_CORRECTION_DEG"] = "10"
os.environ["A5_ORBIT_MIN_CORRECTION_DEG"] = "5"
os.environ["A5_ORBIT_MAX_CORRECTIONS"] = "1"
os.environ["A5_ORBIT_CORRECTION_COAST_DEG"] = "3.5"
os.environ["A5_ORBIT"] = "R6,FR90,F15,FL90,F5,FL90,R6"

if "dotenv" not in sys.modules:
    dotenv = types.ModuleType("dotenv")
    dotenv.load_dotenv = lambda: None
    sys.modules["dotenv"] = dotenv

if "serial" not in sys.modules:
    serial = types.ModuleType("serial")
    serial.Serial = object
    serial.SerialException = Exception
    sys.modules["serial"] = serial

if "picamera" not in sys.modules:
    picamera = types.ModuleType("picamera")
    picamera.PiCamera = object
    sys.modules["picamera"] = picamera

from communications.stm import validate_line  # noqa: E402
from task_a5 import (  # noqa: E402
    ARC_PROFILE,
    DERIVED_ORBIT_LINE,
    ORBIT_GEOMETRY_STANDOFF_CM,
    ORBIT_INSET_CM,
    ORBIT_LINE,
    STANDOFF_CM,
    TaskA5,
)


class FakeSocket:
    def __init__(self, response):
        self.response = response
        self.sent = []

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def settimeout(self, timeout):
        pass

    def sendall(self, data):
        self.sent.append(data)

    def recv(self, size):
        return self.response


class A5ConfigurationTests(unittest.TestCase):
    def bare_task(self):
        task = TaskA5.__new__(TaskA5)
        task.dry_run = False
        return task

    def test_a5_uses_clean_profile(self):
        self.assertEqual(ARC_PROFILE, 1)

    def test_profile_one_orbit_matches_45cm_standoff(self):
        self.assertEqual(STANDOFF_CM, 45)
        self.assertEqual(ORBIT_INSET_CM, 20)
        self.assertEqual(ORBIT_GEOMETRY_STANDOFF_CM, 25)
        self.assertEqual(DERIVED_ORBIT_LINE, "R20,FR90,F15,FL90,R12,FL90,R6")
        self.assertEqual(ORBIT_LINE, "R6,FR90,F15,FL90,F5,FL90,R6")
        ok, reason = validate_line(ORBIT_LINE.split(","))
        self.assertTrue(ok, reason)

    def test_45cm_standoff_emits_fu46(self):
        task = self.bare_task()
        sent = []
        task._us_cm = lambda: 100
        task._send = lambda line: sent.append(line) or True

        self.assertTrue(task._stand_off(45))
        self.assertEqual(sent, ["FU46"])

    def test_jpeg_is_sent_to_pc_detector(self):
        task = self.bare_task()
        image = b"jpeg-data"
        sock = FakeSocket(b"OBJECT,1,0.8750,up_arrow")

        with mock.patch("task_a5.socket.create_connection", return_value=sock):
            self.assertEqual(task._detect_on_pc(image), ("up_arrow", 0.875))

        self.assertEqual(sock.sent, [struct.pack(">I", len(image)), image])

    def test_a5_accepts_detector_result_below_old_55_percent_gate(self):
        task = self.bare_task()
        task.camera = mock.Mock()
        task.camera.capture_image.return_value = b"jpeg-data"
        task._detect_on_pc = mock.Mock(return_value=("W", 0.30))

        with mock.patch("task_a5.sleep"):
            self.assertEqual(task.look(1), ("W", 0.30))

    def test_bullseye_remains_a_wrong_face_at_any_confidence(self):
        task = self.bare_task()
        task.camera = mock.Mock()
        task.camera.capture_image.return_value = b"jpeg-data"
        task._detect_on_pc = mock.Mock(return_value=("bullseye", 0.99))

        with mock.patch("task_a5.sleep"):
            self.assertIsNone(task.look(1))

    def run_orbit(self, turns):
        task = self.bare_task()
        sent = []
        replies = iter([[str(turn)] for turn in turns])
        task._send = lambda line: sent.append(line) or True
        task.stm = mock.Mock()
        task.stm.query_fields.side_effect = lambda command: next(replies)
        task._stand_off = mock.Mock(return_value=True)

        with mock.patch("task_a5.sleep"):
            completed = task.orbit(1)
        return completed, task, sent

    def test_exact_segment_needs_no_correction(self):
        completed, task, sent = self.run_orbit([-900, 900, 900])

        self.assertTrue(completed)
        self.assertEqual(
            sent,
            ["R6", "FR90", "F15", "FL90", "F5", "FL90", "R6", "R20"],
        )
        task._stand_off.assert_called_once_with(45, max_approach_cm=95)

    def test_correction_occurs_only_after_complete_segment(self):
        # Raw segment: -92.2 + 85.8 + 87.9 = 81.5 degrees. One executable
        # final arc corrects it; no correction is interleaved into the route.
        completed, task, sent = self.run_orbit([-922, 858, 879, 85])

        self.assertTrue(completed)
        raw_segment = ["R6", "FR90", "F15", "FL90", "F5", "FL90", "R6"]
        self.assertEqual(sent[:len(raw_segment)], raw_segment)
        self.assertEqual(sent[len(raw_segment):], ["FL5", "R20"])
        task._stand_off.assert_called_once()

    def test_arc_signs_cover_forward_and_reverse(self):
        self.assertEqual(TaskA5._arc_heading("FR90"), -90.0)
        self.assertEqual(TaskA5._arc_heading("FL90"), 90.0)
        self.assertEqual(TaskA5._arc_heading("RR90"), 90.0)
        self.assertEqual(TaskA5._arc_heading("RL90"), -90.0)

    def test_angle_error_never_halts_after_correction_limit(self):
        task = self.bare_task()
        sent = []
        turns = iter([["-20"]])
        task._send = lambda line: sent.append(line) or True
        task.stm = mock.Mock()
        task.stm.query_fields.side_effect = lambda command: next(turns)

        final_heading = task._correct_segment_heading(-90.0, -20.0)

        self.assertEqual(final_heading, -22.0)
        self.assertEqual(sent, ["FR10"])

    def test_latest_floor_turns_are_summed_with_their_signs(self):
        completed, task, sent = self.run_orbit([-924, 906, 864, 54])

        self.assertTrue(completed)
        raw_segment = ["R6", "FR90", "F15", "FL90", "F5", "FL90", "R6"]
        self.assertEqual(sent[:len(raw_segment)], raw_segment)
        self.assertEqual(sent[len(raw_segment):], ["FL5", "R20"])
        task._stand_off.assert_called_once()

    def test_three_degree_error_uses_moving_arc_not_fl2(self):
        task = self.bare_task()
        sent = []
        task._send = lambda line: sent.append(line) or True
        task.stm = mock.Mock()
        task.stm.query_fields.return_value = ["50"]

        final_heading = task._correct_segment_heading(90.0, 87.0)

        self.assertEqual(final_heading, 92.0)
        self.assertEqual(sent, ["FL5"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
