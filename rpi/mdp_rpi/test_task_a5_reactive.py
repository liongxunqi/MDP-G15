"""
test_task_a5_reactive.py  —  offline test for the A.5 sequence
──────────────────────────────────────────────────────
Runs anywhere, on any machine, with no robot, no PC, no camera and no
pyserial. Drives TaskA5 with fakes and checks the decisions it makes.

    python3 test_task_a5_reactive.py          # from rpi/mdp_rpi/

What it is actually protecting
───────────────────────────────
A.5 is one continuous run in front of a supervisor. Everything worth getting
wrong is a decision — orbit or stop, trust this detection or not, keep driving
after a FAIL or not — and every one of those decisions is testable without
hardware. What is left for the floor is geometry: whether ORBIT_LINE lands the
robot square on the next face. That part needs a tape measure and cannot be
faked, so this file does not pretend to cover it.

ORBIT_LINE itself IS checked here, against the real protocol validator in
communications/stm.py — a typo there would otherwise surface as a RESEND
halfway through the demo.
"""

import logging
import os
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

# Set before task_a5 is imported, because it reads these at module scope. The
# settle exists to let a real chassis stop rocking before the ultrasonic
# measures; there is no chassis here, and 23 tests paying 0.3 s each is four
# seconds of nothing.
os.environ.setdefault("A5_SETTLE_S", "0")

# communications/stm.py imports pyserial at module scope, and pyserial is not
# installed on a laptop. Nothing under test touches a port, so a stub is enough
# and is honest about it: if a test ever reaches real serial code it will fail
# loudly on a missing attribute rather than quietly opening something.
if "serial" not in sys.modules:
    stub = types.ModuleType("serial")
    stub.Serial = object
    stub.SerialException = Exception
    sys.modules["serial"] = stub

from communications.stm import validate_line          # noqa: E402  the real one
import task_a5_reactive as task_a5                                        # noqa: E402
from task_a5_reactive import ORBIT_LINE, TaskA5                # noqa: E402


def setUpModule():
    """
    Quiet by default — 23 runs' worth of narration buries the failures.
    Set A5_TEST_LOGS=1 to watch a run play out line by line instead.
    """
    if os.getenv("A5_TEST_LOGS") != "1":
        logging.disable(logging.CRITICAL)


def tearDownModule():
    logging.disable(logging.NOTSET)


# ── Fakes ─────────────────────────────────────────────────────────────────────

class FakeSTM:
    """
    Stands in for communications.stm.STM.

    send_line() runs the REAL validator, so a line this fake accepts is a line
    the firmware's parser would also accept.
    """

    def __init__(self, replies=None, us_cm=40, version=3, profile=0):
        self.lines = []                       # every line that got sent
        self.replies = list(replies or [])    # one per line; default OK
        self.us_cm = us_cm
        self.version = version
        self.profile = profile
        self.aborted = False
        self.disconnected = False

    def send_line(self, tokens) -> bool:
        if isinstance(tokens, str):
            tokens = [t for t in tokens.strip().split(",") if t.strip()]
        ok, reason = validate_line(list(tokens))
        if not ok:
            raise AssertionError(f"task_a5 built an invalid line {tokens}: {reason}")
        self.lines.append(",".join(tokens))
        return True

    def wait_reply(self, timeout=None):
        return self.replies.pop(0) if self.replies else "OK"

    def query_fields(self, command, timeout=2.0):
        tag = command.strip().upper().lstrip("?")
        if tag == "US":
            return None if self.us_cm is None else [str(self.us_cm)]
        if tag == "VER":
            return ["C30D", str(self.version)]
        if tag == "STAT":
            # STAT,<state>,<busy>,<imu_ok>,<profile>
            return ["0", "0", "1", str(self.profile)]
        raise AssertionError(f"unexpected query {command}")

    def abort(self):
        self.aborted = True

    def disconnect(self):
        self.disconnected = True


class FakeCamera:
    def __init__(self, image=b"\xff\xd8jpeg"):
        self.image = image
        self.captures = 0
        self.stopped = False

    def capture_image(self):
        self.captures += 1
        return self.image

    def stop_camera(self):
        self.stopped = True


class FakePC:
    """
    Answers each DETECT with the next scripted class.

    `faces` is a list of (class_name, confidence). A None entry means the link
    drops at that point.
    """

    def __init__(self, faces):
        self.faces = list(faces)
        self.sent = []
        self.images = []
        self._inbox = []
        self.disconnected = False

    def send(self, message):
        message = message.strip()
        self.sent.append(message)
        if message.upper().startswith("DETECT"):
            tag = message.split(",", 1)[1]
            if not self.faces:
                raise AssertionError("more DETECTs than the script has faces")
            answer = self.faces.pop(0)
            if answer is None:
                self._inbox.append(None)
            else:
                name, conf = answer
                self._inbox.append(f"OBJECT,{tag},{conf:.4f},{name}")

    def send_image(self, image):
        self.images.append(image)

    def receive(self):
        return self._inbox.pop(0) if self._inbox else None

    def disconnect(self):
        self.disconnected = True


def build(faces=(), *, replies=None, us_cm=40, version=3, profile=0,
          detect=True, dry_run=False, max_faces=4):
    """A TaskA5 wired to fakes, plus the fakes themselves."""
    stm = FakeSTM(replies=replies, us_cm=us_cm, version=version, profile=profile)
    camera = FakeCamera() if detect else None
    pc = FakePC(faces) if detect else None
    task = TaskA5(dry_run=dry_run, detect=detect, max_faces=max_faces,
                  stm=stm, camera=camera, pc=pc)
    return task, stm, camera, pc


def moves(stm):
    """The movement lines only — queries never go through send_line."""
    return stm.lines


# ── The line A.5 drives ───────────────────────────────────────────────────────

class OrbitLineTests(unittest.TestCase):

    def test_orbit_line_passes_the_real_validator(self):
        tokens = [t for t in ORBIT_LINE.split(",") if t.strip()]
        ok, reason = validate_line(tokens)
        self.assertTrue(ok, f"ORBIT_LINE {ORBIT_LINE!r} is not sendable: {reason}")

    def test_orbit_line_fits_one_firmware_queue(self):
        # PROTOCOL.md §2: 16 primitives, 128 bytes. Sending the orbit as one
        # line is the whole reason it is a single round trip.
        tokens = [t for t in ORBIT_LINE.split(",") if t.strip()]
        self.assertLessEqual(len(tokens), 16)
        self.assertLessEqual(len((ORBIT_LINE + "\n").encode()), 128)

    def test_orbit_uses_no_unverified_reverse_arc(self):
        # RR/RL are implemented in firmware but never run on the floor, and
        # their sign convention is still marked VERIFY. Keep them out until
        # someone has actually driven one.
        for token in ORBIT_LINE.upper().split(","):
            self.assertFalse(token.strip().startswith(("RR", "RL")),
                             f"{token} is an unverified reverse arc")


# ── The approach ──────────────────────────────────────────────────────────────

class ApproachTests(unittest.TestCase):

    def test_standoff_corrects_for_the_sensor_bias(self):
        # The ultrasound reads ~1.3 cm long, so 25 cm of standoff is FU26.
        task, stm, _, _ = build([("A", 0.9)])
        task.standoff_cm = 25
        self.assertEqual(task.run(), 0)
        self.assertEqual(moves(stm)[0], "FU26")

    def test_no_echo_stops_before_anything_moves(self):
        task, stm, _, _ = build([("A", 0.9)], us_cm=0xFFFF)
        self.assertEqual(task.run(), 2)
        self.assertEqual(moves(stm), [], "drove at something it could not see")

    def test_a_wall_far_away_is_not_the_obstacle(self):
        task, stm, _, _ = build([("A", 0.9)], us_cm=400)
        self.assertEqual(task.run(), 2)
        self.assertEqual(moves(stm), [])

    def test_firmware_older_than_v3_is_refused(self):
        task, stm, _, _ = build([("A", 0.9)], version=2)
        self.assertEqual(task.run(), 2)
        self.assertEqual(moves(stm), [], "used FU against firmware without it")

    def test_wrong_arc_profile_warns_but_still_runs(self):
        # CLEAN is r=318mm against the 291mm the orbit was traced for. Someone
        # may have meant it, so say so loudly and carry on rather than refusing.
        task, stm, _, _ = build([("A", 0.9)], profile=1)
        with self.assertLogs(level="WARNING") as captured:
            logging.disable(logging.NOTSET)
            try:
                self.assertEqual(task.run(), 0)
            finally:
                if os.getenv("A5_TEST_LOGS") != "1":
                    logging.disable(logging.CRITICAL)
        self.assertTrue(any("arc profile" in line for line in captured.output),
                        f"no profile-mismatch warning in {captured.output}")
        self.assertEqual(moves(stm), ["FU26"])


# ── Deciding what a face is ───────────────────────────────────────────────────

class LookTests(unittest.TestCase):

    def test_valid_image_on_the_first_face_ends_the_run(self):
        task, stm, camera, pc = build([("7", 0.91)])
        self.assertEqual(task.run(), 0)
        self.assertEqual(camera.captures, 1)
        self.assertEqual(moves(stm), ["FU26"], "orbited after already finding it")

    def test_bullseye_is_the_marker_not_a_target(self):
        task, stm, camera, _ = build([("bullseye", 0.99), ("W", 0.80)])
        self.assertEqual(task.run(), 0)
        self.assertEqual(camera.captures, 2)
        self.assertEqual(moves(stm), ["FU26", ORBIT_LINE, "FU26"])

    def test_nothing_detected_is_also_a_wrong_face(self):
        task, _, camera, _ = build([("NONE", 0.0), ("up_arrow", 0.77)])
        self.assertEqual(task.run(), 0)
        self.assertEqual(camera.captures, 2)

    def test_a_low_confidence_guess_is_not_trusted(self):
        # detect.py already floors at 0.25; A5_MIN_CONFIDENCE is the second gate.
        weak = task_a5.MIN_CONFIDENCE - 0.10
        task, _, camera, _ = build([("X", weak), ("X", 0.95)])
        self.assertEqual(task.run(), 0)
        self.assertEqual(camera.captures, 2)

    def test_each_face_is_tagged_separately(self):
        task, _, _, pc = build([("bullseye", 0.9), ("dot", 0.9), ("Z", 0.9)])
        self.assertEqual(task.run(), 0)
        detects = [m for m in pc.sent if m.startswith("DETECT")]
        self.assertEqual(detects, ["DETECT,A5F1", "DETECT,A5F2", "DETECT,A5F3"])
        self.assertEqual(len(pc.images), 3, "an image per DETECT, in order")


# ── Going around ──────────────────────────────────────────────────────────────

class OrbitTests(unittest.TestCase):

    def test_re_acquires_the_standoff_after_every_orbit(self):
        task, stm, _, _ = build([("bullseye", 0.9), ("bullseye", 0.9), ("M", 0.9)])
        self.assertEqual(task.run(), 0)
        self.assertEqual(moves(stm),
                         ["FU26", ORBIT_LINE, "FU26", ORBIT_LINE, "FU26"])

    def test_gives_up_after_the_last_face_without_orbiting_again(self):
        # Four faces means three orbits: a fourth would drive back to face 1.
        task, stm, _, _ = build([("bullseye", 0.9)] * 4)
        self.assertEqual(task.run(), 1)
        self.assertEqual(moves(stm).count(ORBIT_LINE), 3)

    def test_faces_cap_is_honoured(self):
        task, stm, camera, _ = build([("bullseye", 0.9)] * 2, max_faces=2)
        self.assertEqual(task.run(), 1)
        self.assertEqual(camera.captures, 2)
        self.assertEqual(moves(stm).count(ORBIT_LINE), 1)


# ── When something breaks ─────────────────────────────────────────────────────

class FailureTests(unittest.TestCase):

    def test_a_stalled_move_stops_the_run(self):
        # FAIL,TIMEOUT on the orbit: the robot is not where we think it is, so
        # nothing further may be driven from that pose.
        task, stm, _, _ = build([("bullseye", 0.9), ("A", 0.9)],
                                replies=["OK", "FAIL,TIMEOUT"])
        self.assertEqual(task.run(), 2)
        self.assertEqual(moves(stm), ["FU26", ORBIT_LINE], "kept driving after FAIL")

    def test_noecho_on_the_approach_stops_the_run(self):
        task, stm, _, _ = build([("A", 0.9)], replies=["FAIL,NOECHO"])
        self.assertEqual(task.run(), 2)
        self.assertEqual(moves(stm), ["FU26"])

    def test_a_resend_is_not_retried(self):
        # RESEND is a parse failure — the same bytes would earn the same answer.
        task, stm, _, _ = build([("A", 0.9)], replies=["RESEND"])
        self.assertEqual(task.run(), 2)
        self.assertEqual(moves(stm).count("FU26"), 1)

    def test_a_lost_link_stops_the_run(self):
        task, stm, _, _ = build([("A", 0.9)], replies=[None])
        self.assertEqual(task.run(), 2)

    def test_a_dead_pc_stops_the_run_rather_than_orbiting_blind(self):
        task, stm, _, _ = build([None, ("A", 0.9)])
        self.assertEqual(task.run(), 2)
        self.assertEqual(moves(stm), ["FU26"], "orbited with no detector")

    def test_a_failed_capture_is_treated_as_a_wrong_face(self):
        task, stm, camera, _ = build([("A", 0.9)], max_faces=2)
        camera.image = None
        # One DETECT fewer than faces, because a failed capture never sends one.
        self.assertEqual(task.run(), 1)
        self.assertEqual(moves(stm).count(ORBIT_LINE), 1)


# ── The two bench modes ───────────────────────────────────────────────────────

class ModeTests(unittest.TestCase):

    def test_dry_run_drives_nothing(self):
        task, stm, camera, _ = build([("bullseye", 0.9), ("A", 0.9)], dry_run=True)
        self.assertEqual(task.run(), 0)
        self.assertEqual(moves(stm), [], "dry run put lines on the wire")
        self.assertEqual(camera.captures, 2, "dry run should still prove the camera")

    def test_no_detect_walks_the_whole_orbit(self):
        task, stm, camera, pc = build(detect=False, max_faces=4)
        self.assertIsNone(camera)
        self.assertIsNone(pc)
        self.assertEqual(task.run(), 1)
        self.assertEqual(moves(stm), ["FU26"] + [ORBIT_LINE, "FU26"] * 3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
