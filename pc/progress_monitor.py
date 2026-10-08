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
   TASK1_FEEDBACK_TIMEOUT_S (7 s by default) for a reply - per instruction. If measured
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
IR_TURN_CLEARANCE_CM = float(os.getenv("PROGRESS_IR_TURN_CLEARANCE_CM", "20"))
IR_LEFT_NEAR_RAW = int(os.getenv("PROGRESS_IR_LEFT_NEAR_RAW", "1816"))
IR_RIGHT_NEAR_RAW = int(os.getenv("PROGRESS_IR_RIGHT_NEAR_RAW", "1555"))
ARENA_SIZE_MM = 2000.0
# Keep the next reported reference point this far from an arena edge. The
# planner normally provides this clearance; this second check carries the
# robot's measured translation and heading error into the next primitive so a
# still-acceptable error cannot turn a nominally legal arc into an overrun.
BOUNDARY_LOOKAHEAD_MM = float(os.getenv("PROGRESS_BOUNDARY_LOOKAHEAD_MM", "50"))


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class PlanMonitor:
    def __init__(self, plan: Optional[dict] = None, recovery_planner=None,
                 safety_checker=None):
        self.plan = plan if isinstance(plan, dict) else None
        self.recovery_planner = recovery_planner
        self.safety_checker = safety_checker
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
        self.skips = 0
        self.holds = 0
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
        event = p.get("event")
        preflight = event == "instruction_preflight"
        photo_preflight = event == "photo_adjustment_preflight"
        deviation = None if preflight else self._compare(
            seg, ins, token, x_mm, y_mm, hdg
        ) if not photo_preflight else None

        if p.get("awaiting_decision"):
            if "feedback_id" not in p:
                logging.error(f"PROGRESS awaits a decision but has no feedback_id: {payload_text[:200]}")
                return None
            self.decisions += 1
            boundary_risk = self._sensor_risk(p)
            if boundary_risk is None and (preflight or photo_preflight):
                boundary_risk = self._preflight_safety_risk(p)
            elif boundary_risk is None:
                boundary_risk = self._next_boundary_risk(seg, ins, x_mm, y_mm, hdg, p)
            if photo_preflight:
                action = "SKIP" if boundary_risk is not None else "CONTINUE"
                if action == "SKIP":
                    self.skips += 1
                    logging.warning(
                        "Skipping unsafe photo-distance move %s for obstacle %s.",
                        token, p.get("obstacle_id", "?"),
                    )
                else:
                    self.continues += 1
                return action + "," + json.dumps(
                    {"feedback_id": p["feedback_id"], "segment_index": seg,
                     "instruction_index": ins}, separators=(",", ":"),
                )
            deviation_requires_recovery = (
                deviation is not None and
                (deviation[0] > POS_WARN_MM or abs(deviation[1]) > HDG_WARN_DEG)
            )
            if (self.recovery_planner is not None and
                    (deviation_requires_recovery or boundary_risk is not None)):
                # The replacement cap prevents noisy odometry or a persistent
                # sensor return from endlessly rewriting the route.
                if self.replacements >= MAX_REPLACEMENTS:
                    logging.error(
                        "Automatic replacement limit (%d) reached; continuing existing route.",
                        MAX_REPLACEMENTS,
                    )
                else:
                    try:
                        replacement = self._plan_replacement(p, boundary_risk)
                    except Exception:  # a planner bug must not consume the RPi's decision wait
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

    def _hold_reply(self, progress, seg, ins, reason):
        self.holds += 1
        return "HOLD," + json.dumps(
            {"feedback_id": progress["feedback_id"], "segment_index": seg,
             "instruction_index": ins, "reason": reason},
            separators=(",", ":"),
        )

    @staticmethod
    def _replacement_starts_with_arc(replacement) -> bool:
        try:
            token = str(replacement["segments"][0][0]).strip().upper()
        except (KeyError, IndexError, TypeError):
            return False
        return token.startswith(("FR", "FL", "RR", "RL"))

    def _preflight_safety_risk(self, progress):
        if self.safety_checker is None:
            return None
        token = str(progress.get("token", "")).strip().upper()
        try:
            risk = self.safety_checker(self.plan, progress, token)
        except Exception:
            logging.exception("Preflight swept-footprint safety check crashed.")
            risk = {"reason": "safety checker crashed", "next_token": token}
        if risk is None:
            return None
        risk = dict(risk)
        risk.update(
            next_segment_index=int(progress["segment_index"]),
            next_instruction_index=0,
        )
        logging.warning(
            "Preflight blocks %s: %s at (%s,%s) mm.",
            token, risk.get("reason", "unsafe"),
            risk.get("x_mm", "?"), risk.get("y_mm", "?"),
        )
        return risk

    def _plan_replacement(self, progress, boundary_risk):
        """Replan from the live pose for the current, not a future, segment.

        The RPi preflights every new segment after the preceding photo has been
        taken. Consequently no synthetic stationary photo segment is needed,
        and a replacement cannot cause the same segment-boundary decision to
        repeat indefinitely.
        """
        report = dict(progress)
        if boundary_risk is not None:
            report["blocked_token"] = boundary_risk.get("next_token")
            report["recovery_reason"] = boundary_risk.get("reason")
        return self.recovery_planner(self.plan, report)

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
                f"{self.replacements} REPLACE, {self.skips} SKIP, {self.holds} HOLD.")

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

    def _next_boundary_risk(self, seg, ins, x_mm, y_mm, hdg, progress=None):
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
                # The RPi takes the current segment's photo first, then sends a
                # fresh pose/IR preflight for this next segment. Do not recover
                # across that boundary using stale pre-photo sensor data.
                return None
            next_token = str(lines[next_seg][next_ins]).strip().upper()
            # S has no displacement and therefore cannot create a new overrun.
            if next_token == "S":
                return None
            ex, ey, eh = (float(value) for value in self.expected[seg][ins])
            nx, ny, _ = (float(value) for value in self.expected[next_seg][next_ins])
        except (KeyError, IndexError, TypeError, ValueError):
            return None

        if self.safety_checker is not None:
            report = dict(progress or {})
            report.update(x_grid=x_mm / 100.0, y_grid=y_mm / 100.0,
                          heading_deg=hdg)
            try:
                risk = self.safety_checker(self.plan, report, next_token)
            except Exception:
                logging.exception("Swept-footprint safety check crashed.")
                risk = {"reason": "safety checker crashed", "next_token": next_token}
            if risk is None:
                return None
            risk = dict(risk)
            risk.update(next_segment_index=next_seg,
                        next_instruction_index=next_ins)
            logging.warning(
                "Swept-footprint look-ahead blocks segment %d instruction %d %s: %s "
                "at (%s,%s) mm (open-floor allowance %s mm).",
                next_seg, next_ins, next_token, risk.get("reason", "unsafe"),
                risk.get("x_mm", "?"), risk.get("y_mm", "?"),
                risk.get("arena_overhang_mm", "?"),
            )
            return risk

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

    def _sensor_risk(self, progress):
        """Use live left/right IR as a conservative arc veto."""
        seg = int(progress["segment_index"])
        ins = int(progress["instruction_index"])
        next_seg, next_ins, next_token = self._next_instruction(seg, ins)

        # A completed photo segment is followed by capture/distance adjustment,
        # then a fresh preflight for the next segment.  Do not use the current
        # obstacle's close IR return to veto an arc that runs after that photo.
        # Geometric look-ahead already observes the same segment boundary rule.
        if next_seg is not None and next_seg != seg:
            return None

        if next_token and next_token[:2] in ("FR", "FL", "RR", "RL"):
            left_cm = self._number(progress.get("ir_left_cm"))
            right_cm = self._number(progress.get("ir_right_cm"))
            left_raw = self._number(progress.get("ir_left_raw"))
            right_raw = self._number(progress.get("ir_right_raw"))
            close = []
            if ((left_cm is not None and left_cm != 65535 and
                 left_cm <= IR_TURN_CLEARANCE_CM) or
                    (left_raw is not None and left_raw >= IR_LEFT_NEAR_RAW)):
                close.append("left")
            if ((right_cm is not None and right_cm != 65535 and
                 right_cm <= IR_TURN_CLEARANCE_CM) or
                    (right_raw is not None and right_raw >= IR_RIGHT_NEAR_RAW)):
                close.append("right")
            if close:
                logging.warning(
                    "IR veto before %s: close return on %s (L=%s cm/%s raw, "
                    "R=%s cm/%s raw).",
                    next_token, "/".join(close), left_cm, left_raw,
                    right_cm, right_raw,
                )
                return {
                    "reason": "IR side clearance too small",
                    "next_token": next_token,
                    "next_segment_index": next_seg,
                    "next_instruction_index": next_ins,
                }

        return None

    def _next_instruction(self, seg, ins):
        try:
            lines = self.plan["segments"]
            next_seg, next_ins = seg, ins + 1
            if next_ins >= len(lines[seg]):
                next_seg, next_ins = seg + 1, 0
            while next_seg < len(lines) and next_ins >= len(lines[next_seg]):
                next_seg, next_ins = next_seg + 1, 0
            if next_seg >= len(lines):
                return None, None, None
            return next_seg, next_ins, str(lines[next_seg][next_ins]).strip().upper()
        except (KeyError, IndexError, TypeError):
            return None, None, None

    @staticmethod
    def _number(value):
        try:
            number = float(value)
            return number if math.isfinite(number) else None
        except (TypeError, ValueError):
            return None
