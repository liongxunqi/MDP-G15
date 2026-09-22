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
os.environ["A5_ARC_PROFILE"] = "1"
os.environ["A5_REAR_AXLE_TO_SENSOR_CM"] = "23"
os.environ["A5_ORBIT_ADVANCE_CORRECTION_CM"] = "10"
os.environ["A5_ORBIT_CLEARANCE_CM"] = "15"
os.environ["A5_ORBIT_FR_DEG"] = "88"
os.environ["A5_ORBIT_FL_DEG"] = "94"
os.environ["A5_ORBIT"] = "R20,FR88,F15,FL94,F8,FL94,R26"

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
        self.assertEqual(DERIVED_ORBIT_LINE, "R20,FR88,F15,FL94,F8,FL94,R26")
        self.assertEqual(ORBIT_LINE, "R20,FR88,F15,FL94,F8,FL94,R26")
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

    def test_orbit_checks_each_corrected_arc_then_reacquires_standoff(self):
        task = self.bare_task()
        sent = []
        turns = iter([["-900"], ["900"], ["900"]])
        task._send = lambda line: sent.append(line) or True
        task.stm = mock.Mock()
        task.stm.query_fields.side_effect = lambda command: next(turns)
        task._stand_off = mock.Mock(return_value=True)

        with mock.patch("task_a5.sleep"):
            self.assertTrue(task.orbit(1))

        self.assertEqual(
            sent,
            ["R20", "FR88", "F15", "FL94", "F8", "FL94", "R26"],
        )
        self.assertEqual(task.stm.query_fields.call_count, 3)
        task._stand_off.assert_called_once_with(45, max_approach_cm=95)

    def test_orbit_stops_when_gyro_reports_a_bad_arc(self):
        task = self.bare_task()
        sent = []
        task._send = lambda line: sent.append(line) or True
        task.stm = mock.Mock()
        task.stm.query_fields.return_value = ["-700"]
        task._stand_off = mock.Mock(return_value=True)

        self.assertFalse(task.orbit(1))
        self.assertEqual(sent, ["R20", "FR88"])
        task._stand_off.assert_not_called()


if __name__ == "__main__":
    unittest.main(verbosity=2)
