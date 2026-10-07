"""
pc/progress_monitor.py  —  the PC's half of the RPi's per-instruction protocol
────────────────────────────────────────────────────────────────────────────────
The RPi now sends every token of a segment to the STM as its own line. After
each OK it reads ?WPOSE and sends the PC one line:

    PROGRESS,{"segment_index":0,"instruction_index":1,"token":"F15",
              "x_grid":2.75,"y_grid":3.25,"heading_deg":92.6,
              "awaiting_decision":true,"feedback_id":"...", ...}

(rpi/mdp_rpi/README.md, "Task 1 PC Feedback Contract"). This module does two
things with it.

1. DECIDE.  When "awaiting_decision" is true the RPi stops and waits up to
   TASK1_FEEDBACK_TIMEOUT_S (30 s) for a reply - per instruction. So the PC must
   answer at once. It always answers CONTINUE, echoing feedback_id and both
   indexes exactly. REPLACE is deliberately NOT sent: the planner works in
   cardinal headings only, so a re-plan from a measured pose would correct
   position but not the heading error, which is the dominant one (see
   algo/tests/test_drift.py). Add it here once the planner can start from a
   non-cardinal heading.

2. WATCH.  The planner puts "expected" in PATH: the pose the RPi should report
   after each instruction, in the RPi's own frame (mm, heading north = 0,
   clockwise). PROGRESS is in the same frame, so the comparison needs no
   conversion. Deviations over PROGRESS_POS_WARN_MM / PROGRESS_HDG_WARN_DEG are
   logged (once per segment, so a bad leg is one line, not forty), and summary()
   gives the per-run numbers - the measurement the arc-trim question needs.

Telemetry-only mode (TASK1_FEEDBACK_WAIT=0, the default) sends PROGRESS too, with
awaiting_decision false and no feedback_id; nothing is replied then.
"""

import json
import logging
import math
import os
from typing import Optional

POS_WARN_MM = float(os.getenv("PROGRESS_POS_WARN_MM", "80"))
HDG_WARN_DEG = float(os.getenv("PROGRESS_HDG_WARN_DEG", "6"))


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class PlanMonitor:
    def __init__(self, plan: Optional[dict] = None):
        exp = plan.get("expected") if isinstance(plan, dict) else None
        self.expected = exp if isinstance(exp, list) else None
        self.count = 0
        self.compared = 0
        self.decisions = 0
        self.max_pos = 0.0
        self.max_hdg = 0.0
        self.sum_pos = 0.0
        self.worst = None                      # (pos_mm, seg, ins, token)
        self._warned_segments = set()
        self._warned_no_plan = False
        self._warned_bad = False

    # ── public ────────────────────────────────────────────────────────────────

    def handle(self, payload_text: str) -> Optional[str]:
        """Process the JSON after "PROGRESS,". Returns the full reply line to send
        to the RPi (no newline), or None. Never raises on bad input."""
        try:
            p = json.loads(payload_text)
            seg, ins = int(p["segment_index"]), int(p["instruction_index"])
            x_mm, y_mm = float(p["x_grid"]) * 100.0, float(p["y_grid"]) * 100.0
            hdg = float(p["heading_deg"])
        except (ValueError, KeyError, TypeError) as exc:
            if not self._warned_bad:
                self._warned_bad = True
                logging.warning(f"Malformed PROGRESS ({exc!r}); further ones are skipped quietly.")
            return None

        self.count += 1
        token = p.get("token", "?")
        self._compare(seg, ins, token, x_mm, y_mm, hdg)

        if p.get("awaiting_decision"):
            if "feedback_id" not in p:
                logging.error(f"PROGRESS awaits a decision but has no feedback_id: {payload_text[:200]}")
                return None
            self.decisions += 1
            return "CONTINUE," + json.dumps(
                {"feedback_id": p["feedback_id"], "segment_index": seg, "instruction_index": ins},
                separators=(",", ":"),
            )
        return None

    def summary(self) -> str:
        if not self.count:
            return "PROGRESS: none received."
        if not self.compared:
            return (f"PROGRESS: {self.count} instruction(s) reported, none compared "
                    f"(this plan carries no 'expected' poses).")
        w = self.worst
        return (f"PROGRESS: {self.count} instruction(s), {self.compared} compared with the plan - "
                f"position off by {self.sum_pos / self.compared:.0f} mm on average, "
                f"{self.max_pos:.0f} mm at worst (segment {w[1]} instruction {w[2]} '{w[3]}'); "
                f"heading off by up to {self.max_hdg:.1f} deg. "
                f"{self.decisions} decision(s) answered CONTINUE.")

    # ── internals ─────────────────────────────────────────────────────────────

    def _compare(self, seg, ins, token, x_mm, y_mm, hdg) -> None:
        if self.expected is None:
            if not self._warned_no_plan:
                self._warned_no_plan = True
                logging.info("PROGRESS received but this plan has no 'expected' poses "
                             "(PC restarted mid-run?) - not comparing.")
            return
        try:
            ex, ey, eh = self.expected[seg][ins]
        except (IndexError, TypeError, ValueError):
            logging.warning(f"PROGRESS for segment {seg} instruction {ins} is outside the plan.")
            return

        dx, dy = x_mm - ex, y_mm - ey
        pos, dh = math.hypot(dx, dy), _wrap180(hdg - eh)
        self.compared += 1
        self.sum_pos += pos
        self.max_hdg = max(self.max_hdg, abs(dh))
        if pos >= self.max_pos:
            self.max_pos, self.worst = pos, (pos, seg, ins, token)

        logging.info(f"[seg {seg} ins {ins}] {token:<5} done: ({x_mm:.0f},{y_mm:.0f}) mm @ {hdg:.1f}deg, "
                     f"plan ({ex:.0f},{ey:.0f}) @ {eh:.1f}deg -> off {pos:.0f} mm "
                     f"({dx:+.0f},{dy:+.0f}), {dh:+.1f} deg")
        if (pos > POS_WARN_MM or abs(dh) > HDG_WARN_DEG) and seg not in self._warned_segments:
            self._warned_segments.add(seg)
            logging.warning(f"Robot is off the plan in segment {seg} at instruction {ins} '{token}': "
                            f"{pos:.0f} mm / {dh:+.1f} deg (limits {POS_WARN_MM:.0f} mm / "
                            f"{HDG_WARN_DEG:.0f} deg).")
