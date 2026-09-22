"""
test_task_a5.py  —  offline test for A.5 mode inside task1.py
──────────────────────────────────────────────────────────────
Runs anywhere: no robot, no PC, no Android, no pyserial, no picamera2.

    python3 test_task_a5.py          # from rpi/mdp_rpi/

WHAT IT IS PROTECTING
─────────────────────
A.5 mode is three small changes to task1.py, and two of them are only wrong in
ways you would find out in front of a supervisor:

  * the FAN-OUT must produce four faces from one obstacle, with ids that fold
    back to the one Android knows about;
  * the FILTER must not forward bullseyes, because every fanned id folds back
    to the same obstacle and ArenaReducer.applyTarget() is last-write-wins —
    a bullseye arriving after the real image would replace it on the tablet.

The second is the reason this file exists. It is invisible on the Pi's own
logs, which cheerfully report the right answer while Android shows the wrong
one.

Geometry is not covered here. The planner owns that now, which is most of the
point of this approach.
"""

import logging
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))


def _stub(name, **attrs):
    """Register a stand-in module so task1.py can be imported off-robot."""
    if name in sys.modules:
        return
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod


# task1.py reaches the hardware layers at import time. Nothing under test
# touches them - A.5 mode is pure message handling - so stubs are enough, and
# they fail loudly on any real use rather than quietly opening a port.
_stub("serial", Serial=object, SerialException=Exception)
_stub("bluetooth", BluetoothSocket=object, BluetoothError=Exception,
      RFCOMM=0, PORT_ANY=0, advertise_service=lambda *a, **k: None,
      SERIAL_PORT_CLASS=None, SERIAL_PORT_PROFILE=None)
_stub("picamera", PiCamera=object)
_stub("picamera2", Picamera2=object)
# POSIX-only, so absent on a Windows laptop. Only the run lock uses it.
_stub("fcntl", flock=lambda *a: None, LOCK_EX=2, LOCK_NB=4, LOCK_UN=8)

from task1 import A5_FACE_NAME, Task1             # noqa: E402


def setUpModule():
    logging.disable(logging.CRITICAL)


def tearDownModule():
    logging.disable(logging.NOTSET)


class FakeAndroid:
    def __init__(self):
        self.sent = []

    def send(self, message):
        self.sent.append(message)


def bare_task() -> Task1:
    """
    A Task1 with only the A.5 state populated.

    __new__ rather than __init__ on purpose: the constructor opens Bluetooth,
    a socket, a serial port and a camera, and none of those are involved in
    deciding what A.5 does with a detection.
    """
    t = Task1.__new__(Task1)
    t.a5_mode = True
    t._a5_base_id = None
    t._a5_face_of = {}
    t._a5_found = None
    t.android = FakeAndroid()
    return t


def targets(task) -> list:
    return [m for m in task.android.sent if m.startswith("TARGET")]


# ── Fan-out ───────────────────────────────────────────────────────────────────

class FanOutTests(unittest.TestCase):

    def test_one_obstacle_becomes_four_faces(self):
        t = bare_task()
        out = t._a5_expand([{"id": 1, "x": 10, "y": 10, "d": 0}])

        self.assertEqual(len(out), 4)
        self.assertEqual(sorted(o["d"] for o in out), [0, 2, 4, 6])
        self.assertEqual(len({o["id"] for o in out}), 4, "ids must be distinct")
        for o in out:
            self.assertEqual((o["x"], o["y"]), (10, 10), "all four share the cell")

    def test_the_face_android_sent_is_ignored(self):
        # Not knowing the face is the whole of A.5. Honouring the operator's
        # guess would photograph one face and call it done.
        t = bare_task()
        out = t._a5_expand([{"id": 3, "x": 5, "y": 7, "d": 4}])
        self.assertEqual(sorted(o["d"] for o in out), [0, 2, 4, 6])

    def test_ids_fold_back_to_the_one_android_knows(self):
        t = bare_task()
        out = t._a5_expand([{"id": 7, "x": 1, "y": 1, "d": 2}])
        self.assertEqual(t._a5_base_id, 7)
        for o in out:
            self.assertEqual(o["id"] // 10, 7)

    def test_every_face_is_named(self):
        t = bare_task()
        out = t._a5_expand([{"id": 2, "x": 4, "y": 4, "d": 0}])
        names = {t._a5_face_of[str(o["id"])] for o in out}
        self.assertEqual(names, set(A5_FACE_NAME.values()))

    def test_wrong_obstacle_count_is_passed_through_not_mangled(self):
        t = bare_task()
        two = [{"id": 1, "x": 1, "y": 1, "d": 0}, {"id": 2, "x": 5, "y": 5, "d": 2}]
        self.assertEqual(t._a5_expand(two), two)
        self.assertEqual(t._a5_expand([]), [])

    def test_unusable_id_is_passed_through(self):
        t = bare_task()
        bad = [{"id": "not-a-number", "x": 1, "y": 1, "d": 0}]
        self.assertEqual(t._a5_expand(bad), bad)


# ── What Android hears ────────────────────────────────────────────────────────

class ReportTests(unittest.TestCase):

    def setUp(self):
        self.t = bare_task()
        self.t._a5_expand([{"id": 1, "x": 10, "y": 10, "d": 0}])

    def face_id(self, name: str) -> str:
        for fid, n in self.t._a5_face_of.items():
            if n == name:
                return fid
        raise AssertionError(f"no face {name}")

    def test_a_valid_image_is_reported_under_the_base_id(self):
        self.t._a5_report(self.face_id("E"), "7", 0.91)
        self.assertEqual(targets(self.t), ["TARGET,1,7"])

    def test_a_bullseye_is_never_forwarded(self):
        self.t._a5_report(self.face_id("N"), "bullseye", 0.99)
        self.assertEqual(targets(self.t), [], "the marker is not a target")

    def test_nothing_detected_is_never_forwarded(self):
        self.t._a5_report(self.face_id("N"), "NONE", 0.0)
        self.assertEqual(targets(self.t), [])

    def test_a_bullseye_after_the_image_cannot_overwrite_it(self):
        # THE BUG THIS FILE EXISTS FOR. Every fanned id folds back to obstacle
        # 1, and ArenaReducer.applyTarget() overwrites targetId each time, so
        # a later bullseye would replace the real answer on the tablet.
        self.t._a5_report(self.face_id("W"), "A", 0.88)
        self.t._a5_report(self.face_id("N"), "bullseye", 0.99)
        self.t._a5_report(self.face_id("E"), "dot", 0.95)
        self.assertEqual(targets(self.t), ["TARGET,1,A"])

    def test_a_second_valid_image_does_not_overwrite_the_first(self):
        self.t._a5_report(self.face_id("S"), "5", 0.90)
        self.t._a5_report(self.face_id("E"), "9", 0.95)
        self.assertEqual(targets(self.t), ["TARGET,1,5"])
        self.assertEqual(self.t._a5_found[1], "5")

    def test_the_winning_face_is_recorded(self):
        self.t._a5_report(self.face_id("S"), "Z", 0.80)
        face, name, conf = self.t._a5_found
        self.assertEqual((face, name), ("S", "Z"))
        self.assertAlmostEqual(conf, 0.80)

    def test_all_four_wrong_leaves_nothing_found(self):
        for name in ("bullseye", "NONE", "bullseye", "dot"):
            self.t._a5_report(self.face_id("N"), name, 0.9)
        self.assertIsNone(self.t._a5_found)
        self.assertEqual(targets(self.t), [])

    def test_an_unknown_face_id_still_reports(self):
        # A reply tagged with something we did not send is odd, but dropping a
        # valid image over it would be worse than labelling the face "?".
        self.t._a5_report("999", "B", 0.93)
        self.assertEqual(targets(self.t), ["TARGET,1,B"])
        self.assertEqual(self.t._a5_found[0], "?")


# ── Task 1 must be untouched ──────────────────────────────────────────────────

class Task1UnaffectedTests(unittest.TestCase):

    def test_a5_mode_defaults_off(self):
        t = Task1.__new__(Task1)
        import inspect
        sig = inspect.signature(Task1.__init__)
        self.assertIs(sig.parameters["a5_mode"].default, False)

    def test_expand_is_never_called_when_the_flag_is_off(self):
        t = bare_task()
        t.a5_mode = False
        sent = []
        t.path_requested = False
        t.pc = types.SimpleNamespace(send=sent.append)
        t.obstacles = [{"id": 1, "x": 2, "y": 3, "d": 0}]

        t._request_path_from_pc()
        self.assertEqual(len(sent), 1)
        self.assertIn('"id": 1', sent[0])
        self.assertNotIn('"id": 10', sent[0], "Task 1 must not be fanned out")

    def test_flag_on_sends_four(self):
        t = bare_task()
        sent = []
        t.path_requested = False
        t.pc = types.SimpleNamespace(send=sent.append)
        t.obstacles = [{"id": 1, "x": 2, "y": 3, "d": 0}]

        t._request_path_from_pc()
        self.assertEqual(sent[0].count('"id"'), 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
