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
   TASK1_FEEDBACK_TIMEOUT_S (30 s) for a reply - per instruction. If measured
   error crosses either limit, a supplied recovery planner returns to the
   current segment's original endpoint and preserves the later segments. The PC
   then sends REPLACE; otherwise it answers CONTINUE immediately.

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
MAX_REPLACEMENTS = int(os.getenv("PROGRESS_MAX_REPLACEMENTS", "12"))
ARENA_SIZE_MM = 2000.0
# Keep the next reported reference point this far from an arena edge. The
# planner normally provides this clearance; this second check carries the
# robot's measured translation and heading error into the next primitive so a
# still-acceptable error cannot turn a nominally legal arc into an overrun.
BOUNDARY_LOOKAHEAD_MM = float(os.getenv("PROGRESS_BOUNDARY_LOOKAHEAD_MM", "50"))


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class PlanMonitor:
    def __init__(self, plan: Optional[dict] = None, recovery_planner=None):
        self.plan = plan if isinstance(plan, dict) else None
        self.recovery_planner = recovery_planner
        exp = plan.get("expected") if isinstance(plan, dict) else None
        self.expected = exp if isinstance(exp, list) else None
        self.count = 0
        self.compared = 0
        self.decisions = 0
        self.max_pos = 0.0
        self.max_hdg = 0.0
        self.sum_pos = 0.0
        self.worst = None                      # (pos_mm, seg, ins, token)
        self.replacements = 0
        self.continues = 0
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
        deviation = self._compare(seg, ins, token, x_mm, y_mm, hdg)

        if p.get("awaiting_decision"):
            if "feedback_id" not in p:
                logging.error(f"PROGRESS awaits a decision but has no feedback_id: {payload_text[:200]}")
                return None
            self.decisions += 1
            boundary_risk = self._next_boundary_risk(seg, ins, x_mm, y_mm, hdg)
            deviation_requires_recovery = (
                deviation is not None and
                (deviation[0] > POS_WARN_MM or abs(deviation[1]) > HDG_WARN_DEG)
            )
            if (self.recovery_planner is not None and
                    (deviation_requires_recovery or boundary_risk is not None)):
                # The replacement cap prevents noisy odometry from endlessly
                # rewriting an otherwise safe route. A predicted boundary
                # crossing is different: continuing the old primitive is not
                # a safe fallback, so always give recovery planning a chance.
                if self.replacements >= MAX_REPLACEMENTS and boundary_risk is None:
                    logging.error(
                        "Automatic replacement limit (%d) reached; continuing existing route.",
                        MAX_REPLACEMENTS,
                    )
                else:
                    try:
                        replacement = self._plan_replacement(p, boundary_risk)
                    except Exception:  # a planner bug must not consume the RPi's 30 s wait
                        logging.exception("Recovery planner crashed; continuing existing route.")
                        replacement = None
                    if self._install_replacement(replacement):
                        self.replacements += 1
                        body = {
                            "feedback_id": p["feedback_id"],
                            "segment_index": seg,
                            "instruction_index": ins,
                            "segments": replacement["segments"],
                            "segment_obstacles": replacement["segment_obstacles"],
                        }
                        logging.warning(
                            "Replacing remaining route after segment %d instruction %d; "
                            "%d segment(s) in the replacement (automatic replacement #%d).",
                            seg, ins, len(replacement["segments"]), self.replacements,
                        )
                        return "REPLACE," + json.dumps(body, separators=(",", ":"))
                    if boundary_risk is not None:
                        logging.error(
                            "Boundary recovery could not produce a safe replacement; "
                            "the existing route is being retained as a last resort."
                        )
            self.continues += 1
            return "CONTINUE," + json.dumps(
                {"feedback_id": p["feedback_id"], "segment_index": seg, "instruction_index": ins},
                separators=(",", ":"),
            )
        return None

    def _plan_replacement(self, progress, boundary_risk):
        """Call recovery planning, preserving a photo at a segment boundary.

        When look-ahead crosses into the first instruction of the next segment,
        that next segment is what must be replanned. The RPi has not taken the
        current segment's photo yet, so prepend a stationary S segment carrying
        its obstacle assignment before the recovered route.
        """
        report = progress
        current_target = None
        next_segment = None if boundary_risk is None else boundary_risk.get("next_segment_index")
        current_segment = int(progress["segment_index"])
        if next_segment is not None and next_segment != current_segment:
            mappings = self.plan["segment_obstacles"]
            current_target = mappings[current_segment]
            report = dict(progress)
            report["segment_index"] = next_segment
            report["instruction_index"] = -1
            report["remaining_photo_ids"] = [
                str(value) for value in mappings[next_segment:] if value is not None
            ]

        replacement = self.recovery_planner(self.plan, report)
        if replacement is None or current_target is None:
            return replacement
        pose = [[
            round(float(progress["x_grid"]) * 100.0, 1),
            round(float(progress["y_grid"]) * 100.0, 1),
            round(float(progress["heading_deg"]) % 360.0, 1),
        ]]
        replacement = dict(replacement)
        replacement["segments"] = [["S"]] + list(replacement["segments"])
        replacement["segment_obstacles"] = [current_target] + list(
            replacement["segment_obstacles"]
        )
        replacement["expected"] = [pose] + list(replacement["expected"])
        return replacement

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
                f"{self.decisions} decision(s): {self.continues} CONTINUE, "
                f"{self.replacements} REPLACE.")

    # ── internals ─────────────────────────────────────────────────────────────

    def _install_replacement(self, replacement) -> bool:
        if replacement is None:
            return False
        try:
            segments = replacement["segments"]
            mappings = replacement["segment_obstacles"]
            expected = replacement["expected"]
            valid = (
                isinstance(segments, list) and bool(segments) and
                isinstance(mappings, list) and isinstance(expected, list) and
                len(segments) == len(mappings) == len(expected) and
                all(isinstance(line, list) and bool(line) for line in segments) and
                all(len(line) == len(poses) for line, poses in zip(segments, expected))
            )
        except (KeyError, TypeError):
            valid = False
        if not valid:
            logging.error("Recovery planner returned an invalid replacement; continuing existing route.")
            return False
        self.plan = replacement
        self.expected = expected
        self._warned_segments.clear()
        self._warned_no_plan = False
        return True

    def _compare(self, seg, ins, token, x_mm, y_mm, hdg):
        if self.expected is None:
            if not self._warned_no_plan:
                self._warned_no_plan = True
                logging.info("PROGRESS received but this plan has no 'expected' poses "
                             "(PC restarted mid-run?) - not comparing.")
            return None
        try:
            ex, ey, eh = self.expected[seg][ins]
        except (IndexError, TypeError, ValueError):
            logging.warning(f"PROGRESS for segment {seg} instruction {ins} is outside the plan.")
            return None

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
        return pos, dh

    def _next_boundary_risk(self, seg, ins, x_mm, y_mm, hdg):
        """Predict the next endpoint after applying the measured pose error.

        This is intentionally a one-instruction look-ahead. It runs while the
        RPi is waiting for CONTINUE/REPLACE, so a risky primitive has not yet
        reached the STM. Returns a small diagnostic object when recovery should
        be forced, otherwise None.
        """
        if self.expected is None or not isinstance(self.plan, dict):
            return None
        try:
            lines = self.plan["segments"]
            line = lines[seg]
            next_seg, next_ins = seg, ins + 1
            if next_ins >= len(line):
                next_seg, next_ins = seg + 1, 0
                while next_seg < len(lines) and not lines[next_seg]:
                    next_seg += 1
                if next_seg >= len(lines):
                    return None
            next_token = str(lines[next_seg][next_ins]).strip().upper()
            # S has no displacement and therefore cannot create a new overrun.
            if next_token == "S":
                return None
            ex, ey, eh = (float(value) for value in self.expected[seg][ins])
            nx, ny, _ = (float(value) for value in self.expected[next_seg][next_ins])
        except (KeyError, IndexError, TypeError, ValueError):
            return None

        # Rotate the planner's next displacement by the measured heading error,
        # then apply it at the measured position. Bearings are north-zero and
        # clockwise, hence the clockwise 2-D rotation below.
        dh = _wrap180(hdg - eh)
        angle = math.radians(dh)
        dx, dy = nx - ex, ny - ey
        projected_x = x_mm + dx * math.cos(angle) + dy * math.sin(angle)
        projected_y = y_mm - dx * math.sin(angle) + dy * math.cos(angle)
        low = BOUNDARY_LOOKAHEAD_MM
        high = ARENA_SIZE_MM - BOUNDARY_LOOKAHEAD_MM
        if low <= projected_x <= high and low <= projected_y <= high:
            return None

        risk = {
            "next_token": next_token,
            "next_segment_index": next_seg,
            "next_instruction_index": next_ins,
            "x_mm": projected_x,
            "y_mm": projected_y,
        }
        logging.warning(
            "Boundary look-ahead after segment %d instruction %d: next %s projects "
            "to (%.0f,%.0f) mm; safe reference range is %.0f..%.0f mm. "
            "Forcing route replacement before that instruction.",
            seg, ins, next_token, projected_x, projected_y, low, high,
        )
        return risk
