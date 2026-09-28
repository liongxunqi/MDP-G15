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
os.environ.pop("A5_ARC_PROFILE", None)
os.environ["A5_REAR_AXLE_TO_SENSOR_CM"] = "23"
os.environ["A5_ORBIT_ADVANCE_CORRECTION_CM"] = "10"
os.environ["A5_ORBIT_CLEARANCE_CM"] = "15"
os.environ["A5_ORBIT_FR_DEG"] = "90"
os.environ["A5_ORBIT_FL_DEG"] = "90"
os.environ["A5_ORBIT_HEADING_WARN_DEG"] = "1.0"
os.environ["A5_ORBIT"] = "R6,FR90,F15,FL90,F13,FL90,R9"

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

    def test_a5_uses_tight_profile(self):
        self.assertEqual(ARC_PROFILE, 0)

    def test_tight_orbit_matches_45cm_standoff(self):
        self.assertEqual(STANDOFF_CM, 45)
        self.assertEqual(ORBIT_INSET_CM, 20)
        self.assertEqual(ORBIT_GEOMETRY_STANDOFF_CM, 25)
        self.assertEqual(DERIVED_ORBIT_LINE, "R20,FR90,F15,FL90,R4,FL90,R9")
        self.assertEqual(ORBIT_LINE, "R6,FR90,F15,FL90,F13,FL90,R9")
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

    def run_orbit(self, heading_error):
        task = self.bare_task()
        sent = []
        task._send = lambda line: sent.append(line) or True
        task.heading_log = mock.Mock()
        task.heading_log.record.return_value = {"error": heading_error}
        task._stand_off = mock.Mock(return_value=True)

        with mock.patch("task_a5.sleep"):
            completed = task.orbit(1)
        return completed, task, sent

    def test_orbit_sends_route_then_restores_standoff(self):
        completed, task, sent = self.run_orbit(0.2)

        self.assertTrue(completed)
        self.assertEqual(
            sent,
            ["R6", "FR90", "F15", "FL90", "F13", "FL90", "R9", "R20"],
        )
        task._stand_off.assert_called_once_with(45, max_approach_cm=95)

    def test_orbit_never_injects_a_correction_arc(self):
        # The firmware carries heading error into the next move itself. A
        # large residual is logged and warned about, never "fixed" with an
        # extra arc from here.
        with self.assertLogs(level="WARNING") as logs:
            completed, task, sent = self.run_orbit(-3.0)

        self.assertTrue(completed)
        self.assertEqual(sent[-1], "R20")
        self.assertFalse(any(t[:2] in ("FL", "FR", "RL", "RR") for t in sent[7:]))
        self.assertTrue(any("off after orbit" in m for m in logs.output))
        task.heading_log.record.assert_called_once_with("orbit after face 1")

    def test_dry_run_orbit_does_not_query_heading(self):
        task = self.bare_task()
        task.dry_run = True
        task.heading_log = mock.Mock()
        task._send = lambda line: True
        task._stand_off = mock.Mock(return_value=True)

        with mock.patch("task_a5.sleep"):
            self.assertTrue(task.orbit(1))
        task.heading_log.record.assert_not_called()

if __name__ == "__main__":
    unittest.main(verbosity=2)
