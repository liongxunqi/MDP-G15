"""
pc/test_progress.py  —  the PC against the RPi's per-instruction protocol
──────────────────────────────────────────────────────────────────────────
    python3 pc/test_progress.py

No robot, RPi, YOLO or OpenCV needed. Two parts.

UNIT   PlanMonitor on hand-made PROGRESS lines: telemetry mode sends nothing back;
       feedback mode answers CONTINUE echoing feedback_id and both indexes; bad
       input never raises; deviation warnings fire once per segment.

E2E    A fake RPi walks a REAL plan from the REAL planner, instruction by
       instruction, the way rpi/mdp_rpi/task1.py now does: send each instruction,
       report the pose as PROGRESS (README "Task 1 PC Feedback Contract"), and in
       feedback mode WAIT for CONTINUE before going on. At each photo boundary it
       sends DETECT + the image frame and reads OBJECT back. The "robot" is the
       team's replay model (algo/tests/test_mission_sim.py), optionally with a
       turn bias so the PC has something to flag.

What it checks end to end: no "Unrecognised message" warnings for PROGRESS, one
CONTINUE per decision with the right ids, every instruction compared with the
plan, no stall (the RPi would wait 30 s per instruction otherwise), and that PROGRESS
interleaved with image frames doesn't corrupt either.

Note: rpi/mdp_rpi/instruction_mission.py is not in the repo snapshot this was
written against, so the RPi side here is an emulation of the documented
protocol, not the real class.
"""

import json
import logging
import math
import os
import socket
import struct
import sys
import threading
import types
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "algo"))
sys.path.insert(0, str(REPO / "algo" / "tests"))

fake_detect = types.ModuleType("image_recognition.detect")
fake_detect.detect = lambda image_path: (None, None)
sys.modules["image_recognition.detect"] = fake_detect
try:
    import cv2  # noqa: F401
except ImportError:
    sys.modules["cv2"] = types.ModuleType("cv2")

import progress_monitor as pm       # noqa: E402
import task1_pc                     # noqa: E402
import path_planner as pp           # noqa: E402
import test_mission_sim as sim      # noqa: E402
from stm_tokens import PROFILE_TIGHT, TURN_RADIUS_MM   # noqa: E402

RECEIVED = HERE / "received_images"


def progress(seg, ins, token, x_grid, y_grid, hdg, fid=None, **extra):
    d = {"event": "instruction_completed", "segment_index": seg, "instruction_index": ins,
         "token": token, "x_grid": x_grid, "y_grid": y_grid, "heading_deg": hdg,
         "awaiting_decision": fid is not None}
    if fid is not None:
        d["feedback_id"] = fid
    d.update(extra)
    return json.dumps(d, separators=(",", ":"))


class LogCapture(logging.Handler):
    def __init__(self):
        super().__init__(logging.DEBUG)
        self.records = []

    def emit(self, record):
        self.records.append(record)

    def messages(self, level):
        return [r.getMessage() for r in self.records if r.levelno >= level]


# ── unit ──────────────────────────────────────────────────────────────────────

class MonitorUnit(unittest.TestCase):
    PLAN = {"expected": [[[300.0, 300.0, 0.0], [300.0, 400.0, 0.0]], [[500.0, 500.0, 90.0]]]}

    def setUp(self):
        self.cap = LogCapture()
        logging.getLogger().addHandler(self.cap)
        self.addCleanup(logging.getLogger().removeHandler, self.cap)
        logging.getLogger().setLevel(logging.DEBUG)

    def test_telemetry_mode_sends_nothing_back(self):
        m = pm.PlanMonitor(self.PLAN)
        self.assertIsNone(m.handle(progress(0, 0, "F10", 3.0, 3.0, 0.0)))

    def test_feedback_mode_answers_continue_with_the_ids_echoed_exactly(self):
        m = pm.PlanMonitor(self.PLAN)
        reply = m.handle(progress(0, 1, "F10", 3.0, 4.0, 0.0, fid="opaque-123"))
        tag, body = reply.split(",", 1)
        self.assertEqual(tag, "CONTINUE")
        self.assertEqual(json.loads(body),
                         {"feedback_id": "opaque-123", "segment_index": 0, "instruction_index": 1})

    def test_a_non_string_feedback_id_is_echoed_as_it_came(self):
        m = pm.PlanMonitor(self.PLAN)
        reply = m.handle(progress(0, 0, "F10", 3.0, 3.0, 0.0, fid=7))
        self.assertEqual(json.loads(reply.split(",", 1)[1])["feedback_id"], 7)

    def test_garbage_never_raises_and_never_replies(self):
        m = pm.PlanMonitor(self.PLAN)
        for bad in ("not json", "{}", '{"segment_index":"x"}', "[]", "null", ""):
            self.assertIsNone(m.handle(bad), bad)

    def test_awaiting_without_a_feedback_id_cannot_be_answered(self):
        m = pm.PlanMonitor(self.PLAN)
        p = json.loads(progress(0, 0, "F10", 3.0, 3.0, 0.0))
        p["awaiting_decision"] = True
        self.assertIsNone(m.handle(json.dumps(p)))
        self.assertTrue(any("no feedback_id" in x for x in self.cap.messages(logging.ERROR)))

    def test_deviation_warns_once_per_segment(self):
        m = pm.PlanMonitor(self.PLAN)
        m.handle(progress(0, 0, "F10", 3.0, 3.0, 0.0))                    # exact
        self.assertEqual(self.cap.messages(logging.WARNING), [])
        m.handle(progress(0, 1, "F10", 3.0, 5.0, 0.0))                    # 100 mm off
        m.handle(progress(0, 1, "F10", 3.0, 5.2, 0.0))                    # still segment 0
        self.assertEqual(len(self.cap.messages(logging.WARNING)), 1)
        m.handle(progress(1, 0, "FR90", 5.0, 5.0, 120.0))                 # 30 deg off, new segment
        self.assertEqual(len(self.cap.messages(logging.WARNING)), 2)

    def test_heading_difference_wraps_around_north(self):
        m = pm.PlanMonitor({"expected": [[[0.0, 0.0, 359.0]]]})
        m.handle(progress(0, 0, "FR90", 0.0, 0.0, 1.0))                   # 2 deg apart, not 358
        self.assertEqual(self.cap.messages(logging.WARNING), [])
        self.assertAlmostEqual(m.max_hdg, 2.0, places=3)

    def test_outside_the_plan_and_no_plan_are_handled(self):
        m = pm.PlanMonitor(self.PLAN)
        self.assertIsNone(m.handle(progress(9, 9, "F1", 1.0, 1.0, 0.0)))
        n = pm.PlanMonitor(None)
        self.assertEqual(n.handle(progress(0, 0, "F1", 1.0, 1.0, 0.0, fid="a")).split(",")[0], "CONTINUE")
        self.assertIn("none compared", n.summary())

    def test_summary_reports_the_worst_instruction(self):
        m = pm.PlanMonitor(self.PLAN)
        m.handle(progress(0, 0, "F10", 3.0, 3.0, 0.0))
        m.handle(progress(0, 1, "F10", 3.0, 4.5, 4.0))
        s = m.summary()
        self.assertIn("segment 0 instruction 1", s)
        self.assertIn("50 mm at worst", s)
        self.assertIn("4.0 deg", s)


# ── end to end ────────────────────────────────────────────────────────────────

LAYOUT = [   # the team's "sample-5": five spread-out obstacles, mixed faces
    {"id": 1, "x": 1, "y": 18, "d": 4}, {"id": 2, "x": 7, "y": 13, "d": 2},
    {"id": 3, "x": 11, "y": 4, "d": 0}, {"id": 4, "x": 16, "y": 14, "d": 6},
    {"id": 5, "x": 13, "y": 9, "d": 6},
]


def emulate_rpi(server, out, feedback, turn_bias=(0.0, 0.0)):
    """The RPi side, as documented. Records what the PC did into `out`."""
    out.update(sent_instructions=0, decisions=0, bad_replies=[], objects=[], error=None)
    try:
        conn, _ = server.accept()
        conn.settimeout(20)
        f = conn.makefile("rb")
        conn.sendall(f"OBSTACLES,{json.dumps(LAYOUT)}\n".encode())
        line = f.readline().decode().strip()
        assert line.startswith("PATH,"), line
        plan = json.loads(line.split(",", 1)[1])
        out["photos_planned"] = sum(1 for t in plan["segment_obstacles"] if t is not None)

        start = plan["start_mm"]
        shift = (task1_shift_x(start), task1_shift_y(start))
        robot = sim.Replay(LAYOUT, TURN_RADIUS_MM[PROFILE_TIGHT])
        if turn_bias != (0.0, 0.0):                      # make the "robot" turn wrongly
            real_arc = robot.arc
            robot.arc = lambda fwd, right, deg: real_arc(
                fwd, right, deg + (turn_bias[0] if right else turn_bias[1]))

        counter = 0
        for si, (seg, target) in enumerate(zip(plan["segments"], plan["segment_obstacles"])):
            for ii, tok in enumerate(seg):
                robot.run(tok)
                out["sent_instructions"] += 1
                counter += 1
                fid = f"fb-{counter}" if feedback else None
                x = (robot.x + shift[0]) / 100.0
                y = (robot.y + shift[1]) / 100.0
                hdg = (90.0 - math.degrees(robot.th)) % 360.0
                last = ii == len(seg) - 1
                conn.sendall(("PROGRESS," + progress(
                    si, ii, tok, round(x, 3), round(y, 3), round(hdg, 1), fid,
                    segment=seg, segment_completed=last,
                    remaining_photo_ids=[], decision_timeout_s=30.0,
                )  + "\n").encode())
                if feedback:                              # stall here until the PC decides
                    reply = f.readline().decode().strip()
                    tag, _, body = reply.partition(",")
                    try:
                        d = json.loads(body)
                    except ValueError:
                        d = {}
                    if tag == "CONTINUE" and d == {"feedback_id": fid, "segment_index": si,
                                                   "instruction_index": ii}:
                        out["decisions"] += 1
                    else:
                        out["bad_replies"].append((fid, reply))
            if target is not None:                        # photo boundary: header + frame
                jpeg = b"\xff\xd8fake-jpeg\xff\xd9"
                conn.sendall(f"DETECT,{target}\n".encode() + struct.pack(">I", len(jpeg)) + jpeg)
                out["objects"].append(f.readline().decode().strip())
        conn.sendall(b"STITCH,1\n")
        conn.shutdown(socket.SHUT_WR)
    except Exception as exc:                              # noqa: BLE001
        out["error"] = repr(exc)


def task1_shift_x(start):
    return pp.RPI_ANCHOR_X_MM - start["x"]


def task1_shift_y(start):
    return pp.RPI_ANCHOR_Y_MM - start["y"]


class EndToEnd(unittest.TestCase):
    def run_pc(self, feedback, turn_bias=(0.0, 0.0)):
        cap = LogCapture()
        logging.getLogger().addHandler(cap)
        logging.getLogger().setLevel(logging.DEBUG)
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        task1_pc.RPI_IP, task1_pc.RPI_PORT = server.getsockname()
        out = {}
        t = threading.Thread(target=emulate_rpi, args=(server, out, feedback, turn_bias), daemon=True)
        t.start()
        existed = RECEIVED.exists()
        before = set(RECEIVED.glob("*")) if existed else set()
        try:
            task1_pc.main()
            t.join(30)
        finally:
            logging.getLogger().removeHandler(cap)
            server.close()
            for f in (set(RECEIVED.glob("*")) - before) if RECEIVED.exists() else []:
                f.unlink()
            if not existed:
                try:
                    RECEIVED.rmdir()
                except OSError:
                    pass
        self.assertIsNone(out["error"], out["error"])
        return out, cap

    def test_feedback_mode_never_stalls_and_every_decision_is_answered(self):
        out, cap = self.run_pc(feedback=True)
        self.assertGreater(out["sent_instructions"], 10)
        self.assertEqual(out["bad_replies"], [])
        self.assertEqual(out["decisions"], out["sent_instructions"])
        self.assertGreater(out["photos_planned"], 0)
        self.assertEqual(len(out["objects"]), out["photos_planned"])     # one OBJECT per photo
        self.assertTrue(all(o.startswith("OBJECT,") for o in out["objects"]), out["objects"])
        self.assertEqual([m for m in cap.messages(logging.WARNING) if "Unrecognised" in m], [])

    def test_telemetry_mode_is_quiet_and_every_instruction_is_compared(self):
        out, cap = self.run_pc(feedback=False)
        self.assertEqual(out["decisions"], 0)
        # The stub detector finds nothing, so "Nothing detected" and the stitch
        # warning are expected; PROGRESS itself must add no warnings.
        unrelated = ("Nothing detected", "Stitch")
        self.assertEqual([m for m in cap.messages(logging.WARNING)
                          if not any(u in m for u in unrelated)], [])
        summary = [m for m in cap.messages(logging.INFO) if m.startswith("PROGRESS:")][-1]
        self.assertIn(f"{out['sent_instructions']} instruction(s), {out['sent_instructions']} compared",
                      summary)

    def test_a_wrongly_turning_robot_is_flagged(self):
        out, cap = self.run_pc(feedback=False, turn_bias=(4.0, -4.0))
        flagged = [m for m in cap.messages(logging.WARNING) if "off the plan" in m]
        self.assertGreaterEqual(len(flagged), 1)


if __name__ == "__main__":
    unittest.main(verbosity=1)
