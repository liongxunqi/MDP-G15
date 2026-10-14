"""
task2t.py - Task 2, the fastest car, on the TIGHT arc profile.

TIGHT version of task2.py, planned with task2t_park.py (TIGHT's radius and
arc speed) and run on a TIGHT calibration, in the firmware's fast mode.

    python3 task2t.py                 # on the robot
    python3 task2t_sim.py 90 80 50    # the same code against a simulated STM

TEST BUILD. The run starts on Enter, not the Android button, and you type
each obstacle's arrow (left / right) where the photo would be taken.
Everything else is measured by the robot.

The route, in the world frame of task2t_park.py (origin the bay mouth, +x out):

  1. Ultrasound obstacle 1, then use odometry F moves to a standoff and ask
     for its arrow.
  2. Turn 45 deg to that side, a short diagonal, straighten: on the lane just
     clear of obstacle 1, the ultrasound measures obstacle 2. If it misses (a
     narrow obstacle 2 is off the beam), widen the lane and find obstacle 2
     with the inner IR going by.
  3. Photo pose: 45 deg out, a diagonal, swing back in, so the camera looks
     at obstacle 2's arrow from 45 deg, PHOTO_DIST_CM away. Ask its arrow.
  4. Turn to run along obstacle 2's face with a side IR on it. Matching
     arrows use a checked forward front-face approach or outer route, never
     a reverse handoff. If neither is clear, do not drive. IR confirms the
     face before declaring its end.
  5. Round that end, behind obstacle 2, round the other end: heading home on
     the far side. task2t_park.ParkRunner does the rest - IR fix on obstacle
     1, S onto the centreline, and park using the measured world pose.

Speed: the firmware's fast mode (!FAST1) runs straights at 150 rpm and
halves the settle after every move; T2T_FAST=0 turns it off. Each phase is
one line to the STM, so the robot does not wait on the Pi between moves.

Calibration: T2T_CAL_PROFILE in .env (falls back to T2_CAL_PROFILE, then
CAL_PROFILE). A SAFE calibration is refused - the plan assumes TIGHT's radius.
"""

import argparse
import logging
import math
import os
import sys
import time
from typing import Callable, Iterator, List, Optional, Sequence, Tuple

import task2t_park as tp
from task2t_park import Course, Pose, Seg

# Body gap kept past obstacle 1 on the way out. The lane is chosen as tight
# as this allows so the ultrasound, at the robot's centreline, still lands on
# obstacle 2's face when it is only 30 cm wide.
OBS1_PASS_GAP_CM = tp._env_f("T2_OBS1_PASS_GAP_CM", 6.0)
# The final obstacle-1 approach can land with enough pose error that the
# requested side's 45-degree kick is no longer clear. Allow backing farther
# into the open approach lane first; candidate ranking still chooses the
# shortest safe recovery.
OBS1_RECOVERY_REVERSE_CM = 35
# Before the final odometry straight closes the gap, the bulk straight must
# leave the same obstacle in the ultrasonic beam. A larger jump means the beam
# moved onto a wall/second obstacle; continuing would chase that echo.
OBS1_APPROACH_US_TOL_CM = tp._env_f("T2_OBS1_APPROACH_US_TOL_CM", 15.0)
OBS1_APPROACH_ODO_TOL_CM = tp._env_f("T2_OBS1_APPROACH_ODO_TOL_CM", 15.0)
# The STM cannot replace a movement primitive in flight.  During obstacle 1's
# approach we therefore run one fast straight, poll the already-running vision
# vote, and use RST as soon as the requested lane change is collision-safe.
# This avoids braking after every short straight while retaining an RPi-side
# reaction to an arrow detected well before the planned photo position.
EARLY_ABORT_SETTLE_S = tp._env_f("T2_EARLY_ABORT_SETTLE_S", 0.10)
# Turns at obstacle 1 and into the photo pose are this sharp.
KICK_DEG = 45
# Photo of obstacle 2: the camera this far from the arrow, looking at it
# along the robot's heading, ideally from 45 deg. Walls close in can force it
# nearer or straighter - it steps down to PHOTO_MIN_CM and a smaller swing.
PHOTO_DIST_CM = tp._env_f("T2T_PHOTO_DIST_CM", 50.0)
PHOTO_MIN_CM = tp._env_f("T2T_PHOTO_MIN_CM", 34.0)
PHOTO_SWINGS = (90, 80, 70, 60)       # 90 = looking in from 45 deg
CAMERA_FROM_AXLE_CM = tp._env_f("T2_CAMERA_FROM_AXLE_CM", tp.FRONT_FROM_AXLE_CM)
# Stand-in for recognition time when the arrow comes off a prompt.
OBS_PAUSE_S = tp._env_f("T2T_OBS_PAUSE_S", 0.0)
# Running along obstacle 2's face: the IR this far off it.
FACE_GAP_CM = tp._env_f("T2T_FACE_GAP_CM", 22.0)
# The most an IR reading can say and still be believed as a face. With
# nothing in range the IR does NOT read blank: its background output maps to
# ~30-50 cm (the right unit read 29-48 for 90 cm of empty floor). Anything
# past this is treated as nothing there.
IR_FACE_MAX_CM = tp._env_f("T2T_IR_FACE_MAX_CM", 26.0)
# A checked handoff can finish a few centimetres farther from the front face
# than its nominal endpoint when the calibrated arc under-turns.  Accept a
# stable reading near the pose-predicted gap, but keep a hard ceiling so the
# IR's 30-50 cm empty-floor plateau is not treated as a face indiscriminately.
FRONT_FACE_GAP_TOL_CM = tp._env_f("T2T_FRONT_FACE_GAP_TOL_CM", 8.0)
FRONT_FACE_HARD_MAX_CM = tp._env_f("T2T_FRONT_FACE_HARD_MAX_CM", 34.0)
# Behind obstacle 2, the IR on its back face at most this far off it.
BACK_GAP_MAX_CM = tp._env_f("T2T_BACK_GAP_MAX_CM", 34.0)
# Coming round behind obstacle 2, the IR at most this far past its near end.
# The side IR is 17 cm ahead of the axle. At a measured 5-degree heading
# error, a clean two-arc corner places it about 23.5 cm into the rear face.
# Twenty-four still leaves 6 cm of the brief's narrowest 30 cm obstacle for
# the stationary acquisition and moving edge confirmation.
BACK_START_MAX_CM = tp._env_f("T2T_BACK_START_MAX_CM", 24.0)
# A 35 cm sensing lane is outside every <=60 cm face but still leaves a
# turnable corridor when WPOSE has accumulated several centimetres of drift.
# Forty centimetres distinguished the same boxes but stranded the robot too
# close to the wall before its forward-only corner.
SENSING_LANE_CM = tp._env_f("T2T_SENSING_LANE_CM", 35.0)
# Obstacle 2's measured centre is taken as odometry drift, up to this much.
Y_RESYNC_MAX_CM = tp._env_f("T2T_Y_RESYNC_MAX_CM", 15.0)
# Heading home past obstacle 2, the body this far off its side.
HOME_GAP_CM = tp._env_f("T2T_HOME_GAP_CM", 15.0)
# Added to the measured half-width before planning round it.
END_MARGIN_CM = tp._env_f("T2T_END_MARGIN_CM", 2.0)
TURN_CLEARANCE_CM = max(tp.CLEARANCE_FLOOR_CM,
                        tp._env_f("T2T_TURN_CLEARANCE_CM", 8.0))
# Obstacle 1 is encountered before its true swept-path error can be measured.
# Keep a larger hardware margin there than the generic close-pass floor; the
# floor run touched the box when the model reported 6.4 cm of clearance.
OBS1_TURN_CLEARANCE_CM = max(
    TURN_CLEARANCE_CM, tp._env_f("T2T_OBS1_TURN_CLEARANCE_CM", 10.0))
FAST = os.getenv("T2T_FAST", "1").strip() != "0"
# Brief: obstacle 2 is 30-90 wide, 60-150 past obstacle 1. Keep 5 cm of
# measurement tolerance at either end. The old 45 cm lower bound admitted a
# short floor/edge echo as obstacle 2 and shifted its collision model forward
# by more than the robot's remaining clearance.
W2_RANGE = (30.0, 90.0)
GAP12_RANGE = (55.0, 155.0)
# Before obstacle 2's width is measured: plan against its widest case and
# the walls of its narrowest, together.
CONSERVATIVE_W2 = W2_RANGE[1]
CONSERVATIVE_WALL_Y = W2_RANGE[0] / 2.0 + tp.WALL_CLEARANCE_CM
# The ultrasound missed obstacle 2 from the obstacle 1 lane, so its half-width
# is at most about the lane's: NARROW_HALF_MAX. The robot then widens to
# NARROW_LANE, clear of any such obstacle and of a wall 50 cm past the
# narrowest, and finds obstacle 2 with the inner IR going by.
NARROW = -1.0
NARROW_HALF_MAX = tp._env_f("T2_NARROW_HALF_MAX_CM", 22.0)
NARROW_LANE_CM = NARROW_HALF_MAX + tp.HALF_WIDTH_CM + tp._env_f("T2_NARROW_GAP_CM", 12.0)

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [T2] %(levelname)s - %(message)s",
                    datefmt="%H:%M:%S")


def _straight(d: float) -> List[Seg]:
    n = int(round(abs(d)))
    if n < 1:
        return []
    return [Seg("F" if d > 0 else "B", n)]


def _other(turn: str) -> str:
    return "R" if turn == "L" else "L"


def kick(dy: float, r: float) -> List[Seg]:
    """
    Shift sideways by dy (world y) from heading +x and end on heading +x:
    KICK_DEG out, a diagonal, KICK_DEG back. A shift too small for two full
    KICK_DEG arcs falls back to two shallower ones.
    """
    if abs(dy) < 0.5:
        return []
    k = math.radians(KICK_DEG)
    arcs = 2 * r * (1 - math.cos(k))
    if abs(dy) < arcs:
        return tp.lane_change(0.0, dy, r)
    out = tp._turn_toward(0.0, dy)
    return [Seg(out, KICK_DEG)] + _straight((abs(dy) - arcs) / math.sin(k)) \
        + [Seg(_other(out), KICK_DEG)]


def _ask_arrow(input_fn: Callable[[str], str], which: str) -> float:
    while True:
        ans = input_fn(f"{which} arrow - left or right? ").strip().lower()
        if ans in ("l", "left"):
            return 1.0
        if ans in ("r", "right"):
            return -1.0
        print("Type left or right (l / r).")


def _path_clear(p: Pose, segs: Sequence[Seg], r: float, boxes, need: float,
                step: float = 2.0) -> Optional[float]:
    """Closest approach along segs, or None the moment it dips below need -
    the loop planner tries hundreds of candidates and most fail early."""
    best = float("inf")
    for q in tp.run(p, segs, r, step):
        for b in boxes:
            c = tp.clearance(q, b)
            if c < need:
                return None
            best = min(best, c)
    return best


class Task2:

    def __init__(self, stm, input_fn: Callable[[str], str] = input,
                 sleep_fn: Callable[[float], None] = time.sleep,
                 startup: bool = True, wait_for_go: bool = True,
                 early_arrow_fn: Optional[Callable[[], Optional[str]]] = None,
                 lane_reached_fn: Optional[Callable[[], None]] = None):
        self.stm = stm
        self.input_fn = input_fn
        self.sleep_fn = sleep_fn
        self.startup = startup
        self.wait_for_go = wait_for_go
        self.early_arrow_fn = early_arrow_fn
        self.lane_reached_fn = lane_reached_fn
        self.stage2_early_stop = False
        self.frame = tp.WorldFrame(stm)
        self.th = 0.0                 # commanded heading, radians
        self.r = tp.RADIUS_CM
        self.fast = False             # what the firmware is set to
        # Y coordinate of the ultrasonic ray that actually hit obstacle 2.
        # It supplies a measured lower bound on that obstacle's width for the
        # same-side outer-eight handoff. None means obstacle 2 was found by IR.
        self.obs2_us_y: Optional[float] = None
        # A checked US miss from a farther parallel lane supplies the matching
        # upper half-width bound. Together these bounds avoid treating every
        # medium obstacle as the 90 cm worst case.
        self.obs2_half_max: Optional[float] = None
        self.obs2_near_y: Optional[float] = None
        self.last_face_start_y: Optional[float] = None
        self.last_stopped_pose: Optional[Pose] = None

    # ── wire ──────────────────────────────────────────────────────────────────

    def _line(self, segs: Sequence) -> bool:
        """Send one line and wait for it. Tracks the commanded heading."""
        tokens = [s.token() if isinstance(s, Seg) else s for s in segs]
        if not tokens:
            return True
        line = ",".join(tokens)
        if not self.stm.send_line(line):
            return False
        reply = self.stm.wait_reply()
        if reply is None or not reply.startswith("OK"):
            logging.error("STM answered %r to %s - stopping.", reply, line)
            return False
        self._track(segs)
        return True

    def _track(self, segs: Sequence) -> None:
        for s in segs:
            if isinstance(s, Seg):
                self.th += math.radians(s.turn_deg)

    def _set_fast(self, on: bool) -> None:
        """Fast mode, if the run uses it and the firmware has it."""
        on = on and FAST
        if on == self.fast or not hasattr(self.stm, "set_fast"):
            return
        if self.stm.set_fast(on):
            self.fast = on
        else:
            logging.warning("!FAST%d refused - carrying on at %s speed.",
                            int(on), "fast" if self.fast else "normal")

    def _ir(self, side: str) -> Optional[float]:
        fields = self.stm.query_fields("?IR", timeout=0.5)
        if not fields or len(fields) < 2:
            return None
        try:
            v = int(fields[0] if side == "left" else fields[1])
        except ValueError:
            return None
        return None if v == 0xFFFF else float(v)

    def _us(self) -> Optional[float]:
        """True gap from the front bumper, cm (bias removed)."""
        fields = self.stm.query_fields("?US")
        try:
            v = int(fields[0])
        except (TypeError, ValueError, IndexError):
            return None
        return None if v == 0xFFFF else v - tp.US_BIAS_CM

    def _pose(self) -> Pose:
        p = self.frame.pose()
        if p is None:
            raise RuntimeError("No ?WPOSE.")
        return Pose(p.x, p.y, self.th)     # the carry-over holds commanded

    def _wait_for_stopped_pose(self, samples: int = 20,
                               interval_s: float = 0.05) -> Optional[Pose]:
        """Return only after two consecutive WPOSE samples are stationary.

        RST starts braking but does not wait for the chassis to stop. Planning
        from the first post-RST sample caused the floor run to gain another
        13 cm before its corner was rechecked. This bounded wait normally
        finishes in well under a second and replaces the old blind delay.
        """
        previous = None
        stable = 0
        for _ in range(samples):
            self.sleep_fn(interval_s)
            try:
                current = self._pose()
            except RuntimeError:
                continue
            if previous is not None:
                moved = math.hypot(current.x - previous.x,
                                   current.y - previous.y)
                turned = abs(math.degrees(math.atan2(
                    math.sin(current.th - previous.th),
                    math.cos(current.th - previous.th))))
                if moved <= 0.3 and turned <= 0.5:
                    stable += 1
                    if stable >= 2:
                        logging.info("RST settled at %s.", current)
                        return current
                else:
                    stable = 0
            previous = current
        logging.error("Robot did not produce a stable WPOSE after RST; "
                      "corner movement withheld.")
        return None

    @staticmethod
    def _front(p: Pose) -> float:
        return p.x + tp.FRONT_FROM_AXLE_CM * math.cos(p.th)

    def _check(self, p: Pose, segs, course: Course, fence: bool = False) -> float:
        """Closest approach along segs. The back fence is left out on the way
        out: the robot starts 2 cm off it and only drives away from it. The
        side fences stay in - with obstacle 1 close to the bay, the 45 deg
        turn starts with the tail still in the mouth."""
        boxes = course.boxes() if fence else [
            b for b in course.boxes() if b.name != "back fence"]
        return tp.path_clearance(tp.run(p, segs, self.r), boxes)[0]

    def _first_clear(self, p: Pose, candidates, course: Course, what: str):
        """
        The first candidate keeping CLEARANCE_CM. Failing that, the best one
        keeping CLEARANCE_FLOOR_CM, with a warning: by then the robot is
        already where it is, and stopping dead loses the run where a close
        pass only risks a touch.
        """
        here = self._check(p, [], course)
        floor = min(tp.CLEARANCE_FLOOR_CM, here - 0.1)
        best, best_c = None, -1.0
        for segs in candidates:
            c = self._check(p, segs, course)
            if c >= tp.CLEARANCE_CM - 0.01:
                return segs
            if c > best_c:
                best, best_c = segs, c
        if best is not None and best_c >= floor:
            logging.warning("%s keeps only %.1f cm (want %.0f) - taking it.",
                            what, best_c, tp.CLEARANCE_CM)
            return best
        logging.error("No clear %s from %s (best %.1f cm).", what, p, best_c)
        return None

    # ── outbound ──────────────────────────────────────────────────────────────

    def _early_obstacle1_side(self, p: Pose, x1: float, lane: float,
                              log_blocked: bool = True) -> Optional[float]:
        """Consume a ready arrow only when its immediate lane change is clear."""
        if self.early_arrow_fn is None:
            return None
        answer = self.early_arrow_fn()
        if not isinstance(answer, str):
            return None
        answer = answer.strip().lower()
        if answer not in ("l", "left", "r", "right"):
            return None
        side = 1.0 if answer in ("l", "left") else -1.0
        course = Course(x1, x1 + tp.OBS1_SIZE_CM + 60.0, CONSERVATIVE_W2,
                        wall_y=CONSERVATIVE_WALL_Y)
        segs = kick(side * lane - p.y, self.r)
        clearance = self._check(p, segs, course)
        if clearance < OBS1_TURN_CLEARANCE_CM - 0.01:
            if not log_blocked:
                return None
            logging.info("Arrow %s is ready, but the immediate lane change at "
                         "x=%.1f keeps only %.1f cm (need %.1f); continuing "
                         "the approach.", answer, p.x, clearance,
                         OBS1_TURN_CLEARANCE_CM)
            return None
        logging.info("Arrow %s confirmed early; lane change is clear at x=%.1f "
                     "with %.1f cm clearance.", answer, p.x, clearance)
        # The real Task 2 input callback returns the already-confirmed result,
        # reports it to Android, and idles stage 1. Simulated/manual runs do not
        # install early_arrow_fn and therefore never enter here.
        return _ask_arrow(self.input_fn, "Obstacle 1")

    def obstacle1(self) -> Optional[Tuple[float, float]]:
        """Approach and ask. Returns (x1, side) or None."""
        p = self._pose()
        g = self._us()
        if g is None or not (40.0 <= g <= 200.0):
            logging.error("Ultrasound reads %s ahead at the start - obstacle 1 "
                          "should be 60-150 cm off the bay. Is the robot square?", g)
            return None
        x1 = self._front(p) + g
        course = Course(x1, x1 + tp.OBS1_SIZE_CM + 60.0, CONSERVATIVE_W2,
                        wall_y=CONSERVATIVE_WALL_Y)
        lane = tp.OBS1_SIZE_CM / 2 + tp.HALF_WIDTH_CM + OBS1_PASS_GAP_CM
        # The latest place the 45 deg turn can start and still clear it: the
        # standoff. Same on either side, so it is found before the arrow is.
        # Too early, the tail is still in the bay mouth; too late, obstacle 1
        # is in the way - so this is a window, and close to the bay it can
        # shrink to nothing at the full margin.
        # Clearance from obstacle 1 only falls as the turn starts later, so
        # the latest start that keeps it is a binary search; the fences are
        # then checked once, on that one. They only get closer the earlier it
        # starts, so if it fails there it fails everywhere at full margin.
        no_fence = [b for b in course.boxes() if "fence" not in b.name]
        kick_segs = kick(lane, self.r)

        def obs_ok(L: int) -> bool:
            return _path_clear(p, _straight(L) + kick_segs, self.r, no_fence,
                               tp.CLEARANCE_CM - 0.01) is not None

        lo, hi = 0, int(x1 - p.x) - 1
        lead = None
        if hi >= 0 and obs_ok(lo):
            while lo < hi:
                mid = (lo + hi + 1) // 2
                if obs_ok(mid):
                    lo = mid
                else:
                    hi = mid - 1
            if self._check(p, _straight(lo) + kick_segs, course) >= tp.CLEARANCE_CM - 0.01:
                lead = lo
        if lead is None:
            clear = [(L, self._check(p, _straight(L) + kick_segs, course))
                     for L in range(0, int(x1 - p.x), 2)]
            lead, c = max(clear, key=lambda t: t[1], default=(None, -1.0))
            if lead is None or c < tp.CLEARANCE_FLOOR_CM:
                logging.error("Obstacle 1 is too close to the bay to get round.")
                return None
            logging.warning("Turn past obstacle 1 keeps only %.1f cm (want %.0f) "
                            "- obstacle 1 is close to the bay.", c, tp.CLEARANCE_CM)
        standoff = x1 - (p.x + lead + tp.FRONT_FROM_AXLE_CM)
        travel = g - standoff
        bulk = travel - tp.APPROACH_LEAD_CM
        logging.info("Obstacle 1 face at x=%.1f (%.1f ahead). Standing off at "
                     "%.1f.", x1, g, standoff)
        approach_start = p
        bulk_along = 0.0
        early_side = self._early_obstacle1_side(p, x1, lane)
        if early_side is not None:
            return x1, early_side
        if bulk >= 5:
            bulk_cm = int(bulk)
            before = p
            before_gap = g
            stopped_side = []

            def poll_early_arrow() -> bool:
                q = self.frame.pose()
                if q is None:
                    return False
                side = self._early_obstacle1_side(q, x1, lane, log_blocked=False)
                if side is None:
                    return False
                stopped_side.append(side)
                self.stm.abort()
                return True

            logging.info("Continuous obstacle-1 approach: F%d; vision may "
                         "interrupt it at the first clear lane-change pose.",
                         bulk_cm)
            reply = tp.run_polling(self.stm, f"F{bulk_cm}", poll_early_arrow)
            if reply == "STOPPED" and stopped_side:
                self.sleep_fn(EARLY_ABORT_SETTLE_S)
                return x1, stopped_side[0]
            if reply is None or not reply.startswith("OK"):
                logging.error("STM answered %r during the continuous obstacle-1 "
                              "approach.", reply)
                return None

            p = self._pose()
            g = self._us()
            expected_gap = before_gap - bulk_cm
            bulk_along = ((p.x - before.x) * math.cos(before.th)
                          + (p.y - before.y) * math.sin(before.th))
            if g is None:
                logging.error("Lost obstacle 1 ultrasound after F%d; refusing "
                              "to continue.", bulk_cm)
                return None
            if g > expected_gap + OBS1_APPROACH_US_TOL_CM:
                logging.error("Obstacle 1 echo changed after F%d: expected %.1f "
                              "cm, read %.1f cm; refusing to chase a different "
                              "surface.", bulk_cm, expected_gap, g)
                return None
            if abs(bulk_along - bulk_cm) > OBS1_APPROACH_ODO_TOL_CM:
                logging.error("Obstacle 1 approach pose disagrees with F%d: "
                              "WPOSE reports %.1f cm; refusing to continue.",
                              bulk_cm, bulk_along)
                return None
            if g < expected_gap - OBS1_APPROACH_US_TOL_CM:
                # A farther return may be obstacle 2 or a wall, but a nearer
                # return on the outbound centreline can only make the current
                # turn more urgent. The floor run that read 22.7 cm after F67
                # was already at its intended standoff; rejecting it prevented
                # a confirmed arrow from ever reaching pass_obstacle1().
                # Re-anchor the face from live pose + US, do not advance again,
                # and let pass_obstacle1() validate the complete turn envelope
                # (including any retreat in the cleared approach lane).
                x1 = self._front(p) + g
                logging.warning("Obstacle 1 is %.1f cm closer than the F%d "
                                "prediction (expected %.1f, read %.1f). "
                                "Re-anchored face at x=%.1f; no further "
                                "approach movement will be sent.",
                                expected_gap - g, bulk_cm, expected_gap, g, x1)
                self.sleep_fn(OBS_PAUSE_S)
                return x1, _ask_arrow(self.input_fn, "Obstacle 1")
            x1 = self._front(p) + g

        remaining = g - standoff
        final_move = _straight(remaining)
        if final_move:
            logging.info("Final obstacle-1 standoff move from ?US: %s",
                         ",".join(seg.token() for seg in final_move))
            if not self._line(final_move):
                return None
        final_pose = self._pose()
        final_gap = self._us()
        if final_gap is None or abs(final_gap - standoff) > 6.0:
            logging.error("Final F move did not reach obstacle 1 safely: "
                          "wanted %.1f cm, "
                          "read %s; refusing the bypass turn.",
                          standoff,
                          "no echo" if final_gap is None else "%.1f cm" % final_gap)
            return None
        expected_total = bulk_along + g - final_gap
        actual_total = ((final_pose.x - approach_start.x) * math.cos(approach_start.th)
                        + (final_pose.y - approach_start.y) * math.sin(approach_start.th))
        if abs(actual_total - expected_total) > OBS1_APPROACH_ODO_TOL_CM:
            logging.error("Obstacle 1 final pose and ultrasound disagree: WPOSE "
                          "reports %.1f cm travel, sensors imply %.1f cm; refusing "
                          "the bypass turn.", actual_total, expected_total)
            return None
        x1 = self._front(final_pose) + final_gap
        self.sleep_fn(OBS_PAUSE_S)
        return x1, _ask_arrow(self.input_fn, "Obstacle 1")

    def pass_obstacle1(self, x1: float, side1: float) -> Optional[float]:
        """45 deg onto the lane on side1, ultrasound obstacle 2. Returns its
        face x, NARROW if the ultrasound missed it, or None."""
        p = self._pose()
        lane = tp.OBS1_SIZE_CM / 2 + tp.HALF_WIDTH_CM + OBS1_PASS_GAP_CM
        course = Course(x1, x1 + tp.OBS1_SIZE_CM + 60.0, CONSERVATIVE_W2,
                        wall_y=CONSERVATIVE_WALL_Y)
        # Final-approach and straight-heading drift can move the safe turn window. Try the
        # turn from the detection pose first. If its complete swept envelope
        # is unsafe, back up only within the already-traversed approach lane;
        # never move closer to obstacle 1 merely to begin the turn.
        leads = [0] + list(range(-1, -OBS1_RECOVERY_REVERSE_CM - 1, -1))
        segs = None
        best_clearance = -1.0
        for lead in leads:
            candidate = _straight(lead) + kick(side1 * lane - p.y, self.r)
            clearance = self._check(p, candidate, course)
            best_clearance = max(best_clearance, clearance)
            if clearance >= OBS1_TURN_CLEARANCE_CM - 0.01:
                segs = candidate
                logging.info("45 deg turn past obstacle 1: clearance %.1f cm "
                             "(hardware margin %.1f cm).", clearance,
                             OBS1_TURN_CLEARANCE_CM)
                break
        if segs is None:
            logging.error("No obstacle-1 lane change keeps the %.1f cm "
                          "hardware margin (best %.1f cm).",
                          OBS1_TURN_CLEARANCE_CM, best_clearance)
            return None
        if segs and segs[0].kind == "B":
            logging.warning("Immediate 45 deg turn is unsafe; backing up %d cm "
                            "in the cleared approach lane first.", segs[0].value)
        else:
            logging.info("Immediate 45 deg turn is clear.")
        if not self._line(segs):
            return None
        p = self._pose()
        self.stage2_early_stop = False
        if self.lane_reached_fn is not None:
            self.lane_reached_fn()

        # An early vision-triggered kick can finish well before the pose from
        # which the original route located obstacle 2.  Stay on the clear
        # bypass lane and make up only that missing longitudinal distance.
        # The target is the same radius-before-obstacle-1 handoff used by the
        # normal route; it is independent of when the arrow was recognised.
        handoff_x = x1 - self.r
        cruise_cm = int(math.floor(handoff_x - p.x))
        if cruise_cm > 0:
            cruise = _straight(cruise_cm)
            clearance = self._check(p, cruise, course)
            if clearance < tp.CLEARANCE_FLOOR_CM:
                logging.error("The bypass-lane cruise to obstacle 2 keeps only "
                              "%.1f cm clearance; refusing it.", clearance)
                return None
            logging.info("Continuous bypass-lane cruise F%d to the obstacle-2 "
                         "handoff (clearance %.1f cm).", cruise_cm, clearance)
            ready = (self.early_arrow_fn is not None
                     and self.early_arrow_fn() in ("left", "right"))
            if ready:
                self.stage2_early_stop = True
                logging.info("Obstacle 2 arrow was confirmed at the start of "
                             "the bypass cruise; completing this checked "
                             "forward positioning, then skipping the photo "
                             "detour.")
                if not self._line(cruise):
                    return None
            elif self.early_arrow_fn is None:
                if not self._line(cruise):
                    return None
            else:
                def stop_for_arrow() -> bool:
                    # WPOSE both keeps the measured frame current and paces
                    # polling while the STM executes the straight movement.
                    self.frame.pose()
                    answer = self.early_arrow_fn()
                    if answer not in ("left", "right"):
                        return False
                    self.stm.abort()                 # RST: no reply follows
                    self.sleep_fn(0.4)
                    self.stage2_early_stop = True
                    logging.info("Obstacle 2 arrow confirmed during the bypass "
                                 "cruise; RST at the measured handoff pose.")
                    return True

                reply = tp.run_polling(
                    self.stm, cruise[0].token(), stop_for_arrow)
                if reply is None or not (reply.startswith("OK") or reply == "STOPPED"):
                    logging.error("STM answered %r during the obstacle-2 "
                                  "bypass cruise.", reply)
                    return None
            p = self._pose()

        g = self._us()
        x2 = None if g is None else self._front(p) + g
        gap12 = None if x2 is None else x2 - (x1 + tp.OBS1_SIZE_CM)
        if gap12 is not None and GAP12_RANGE[0] <= gap12 <= GAP12_RANGE[1]:
            self.obs2_us_y = p.y
            logging.info("Obstacle 2 face at x=%.1f, %.1f cm past obstacle 1.",
                         x2, gap12)
            return x2
        if gap12 is not None and gap12 < GAP12_RANGE[0]:
            logging.error("Ultrasound on the lane reads %.1f - something only "
                          "%.1f cm past obstacle 1. Not driving at it.", g, gap12)
            return None
        # From this lane the robot's centreline is outside the face of any
        # obstacle 2 narrower than about 2 x 21 cm, so a flat face gives no
        # echo. Nothing in range therefore means: narrow.
        logging.info("Ultrasound sees nothing plausible on the lane (%s) - "
                     "obstacle 2 is narrow.", g)
        return NARROW

    def find_narrow(self, x1: float, side1: float) -> Optional[float]:
        """
        Obstacle 2 is narrow but its distance is unknown. Widen to
        NARROW_LANE_CM on side1, drive on with the inner IR watching, and RST
        the moment obstacle 2's end face shows. Returns its face x.
        """
        p = self._pose()
        course = Course(x1, x1 + tp.OBS1_SIZE_CM + GAP12_RANGE[0],
                        2 * NARROW_HALF_MAX, wall_y=CONSERVATIVE_WALL_Y)
        widen = tp.lane_change(0.0, side1 * NARROW_LANE_CM - p.y, self.r)
        if self._first_clear(p, [widen], course, "widening to the narrow lane") is None:
            return None
        if not self._line(widen):
            return None

        # Heading +x on the side1 lane, the centreline is on the right for
        # side1 = left, and the left for side1 = right.
        inner = "right" if side1 > 0 else "left"
        far = x1 + tp.OBS1_SIZE_CM + GAP12_RANGE[1] + tp.OBS2_DEPTH_CM
        # Obstacle 2's end face, if it is 30..2*NARROW_HALF_MAX wide.
        expect_min = NARROW_LANE_CM - tp.IR_SIDE_OFFSET_CM - NARROW_HALF_MAX - 4
        expect_max = NARROW_LANE_CM - tp.IR_SIDE_OFFSET_CM - W2_RANGE[0] / 2.0 + 4
        hits = []

        def poll() -> bool:
            g = self._ir(inner)
            q = self.frame.pose()
            if q is None or g is None:
                return False
            ir_x = q.x + tp.IR_FROM_AXLE_CM
            # Obstacle 1 shows first; only take what is past it.
            if ir_x > x1 + tp.OBS1_SIZE_CM + 15 and expect_min <= g <= expect_max:
                hits.append((q, g))
                # The side is only 10 cm thick. Three moving samples can
                # miss it entirely; brake on evidence and confirm at rest.
                self.stm.abort()                      # RST: no reply comes
                self.sleep_fn(0.10)
                return True
            return False

        confirmed = False
        for attempt in range(3):
            hits.clear()
            remaining = int(far - (self._pose().x + tp.IR_FROM_AXLE_CM))
            if remaining <= 0:
                break
            reply = tp.run_polling(self.stm, f"F{remaining}", poll)
            if reply is None or not (reply.startswith("OK") or reply == "STOPPED"):
                logging.error("STM answered %r while looking for obstacle 2.", reply)
                return None
            if not hits:
                logging.error("Narrow-lane scan finished without a plausible "
                              "obstacle-2 sighting on the %s IR.", inner)
                return None
            first_pose, first_gap = hits[0]
            logging.info("Narrow obstacle candidate on %s IR: %.1f cm; "
                         "confirming at rest (attempt %d/3).",
                         inner, first_gap, attempt + 1)
            # At most four IR queries (0.5s timeout each). Normally the first
            # two agree: only 0.10s braking + 0.06s sample spacing is added.
            consecutive = 0
            for _ in range(4):
                self.sleep_fn(0.03)
                gap = self._ir(inner)
                if (gap is not None and expect_min <= gap <= expect_max
                        and abs(gap - first_gap) <= 3.0):
                    consecutive += 1
                    if consecutive == 2:
                        confirmed = True
                        break
                else:
                    consecutive = 0
            if confirmed:
                # Keep the first moving pose for the front-edge estimate;
                # using the braking endpoint would bias it farther forward.
                logging.info("Narrow obstacle confirmed on %s IR; retaining "
                             "first-sighting pose %s.", inner, first_pose)
                break
            logging.warning("Side IR candidate was seen but not confirmed; %s.",
                            "resuming only the remaining narrow-lane scan"
                            if attempt < 2 else "retry limit reached")
        if not confirmed:
            logging.error("Narrow obstacle IR confirmation exhausted; "
                          "not treating an unconfirmed reading as its edge.")
            return None
        x2 = hits[0][0].x + tp.IR_FROM_AXLE_CM - tp.IR_LAG_CM
        edge_samples = [q.y - side1 * (tp.IR_SIDE_OFFSET_CM + gap)
                        for q, gap in hits]
        edge_samples.sort()
        self.obs2_near_y = edge_samples[len(edge_samples) // 2]
        logging.info("Obstacle 2 side seen %.0f cm off the %s IR: face at "
                     "x=%.1f; near edge y=%.1f.", min(g for _, g in hits),
                     inner, x2, self.obs2_near_y)
        return x2

    def photo_candidates(self, p: Pose, course: Course, side1: float
                         ) -> Iterator[Tuple[List[Seg], float, int]]:
        """
        Moves to the photo pose, best first: KICK_DEG out to side1, a
        diagonal, and a swing back in, sized so the camera ends PHOTO_DIST_CM
        from the arrow (obstacle 2's face centre) looking straight at it.
        With the full swing of 90 it looks in from 45 deg. Yields (segs,
        distance, swing); nearer and shallower ones follow for when walls
        leave no room for the ideal.
        """
        out = tp._turn_toward(p.th, side1)
        inward = _other(out)
        k = math.radians(KICK_DEG) * side1
        u = (math.cos(p.th + k), math.sin(p.th + k))   # along the diagonal
        e = (math.cos(p.th), math.sin(p.th))           # along the lead
        d = PHOTO_DIST_CM
        while d >= PHOTO_MIN_CM - 0.01:
            for swing in PHOTO_SWINGS:
                end0 = tp.run(p, [Seg(out, KICK_DEG), Seg(inward, swing)], self.r)[-1]
                cam0 = (end0.x + CAMERA_FROM_AXLE_CM * math.cos(end0.th),
                        end0.y + CAMERA_FROM_AXLE_CM * math.sin(end0.th))
                want = (course.x2 - d * math.cos(end0.th),
                        0.0 - d * math.sin(end0.th))
                # cam0 + lead*e + diag*u = want, for lead and diag.
                dx, dy = want[0] - cam0[0], want[1] - cam0[1]
                det = e[0] * u[1] - e[1] * u[0]
                if abs(det) < 1e-9:
                    continue
                lead = (dx * u[1] - dy * u[0]) / det
                diag = (e[0] * dy - e[1] * dx) / det
                if diag < -0.5:
                    continue
                yield (_straight(lead) + [Seg(out, KICK_DEG)] + _straight(diag)
                       + [Seg(inward, swing)], d, swing)
            d -= 4.0
        # Already out wide (the narrow-obstacle lane), or walls too close for
        # any of those: no kick, just on or back and swing in, looking at the
        # arrow from whatever distance that gives. swing - 45 is the angle it
        # looks in from, as above.
        for look in (45, 35, 25, 15):
            end0 = tp.run(p, [Seg(inward, look)], self.r)[-1]
            cam0 = (end0.x + CAMERA_FROM_AXLE_CM * math.cos(end0.th),
                    end0.y + CAMERA_FROM_AXLE_CM * math.sin(end0.th))
            v = (math.cos(end0.th), math.sin(end0.th))
            # cam0 + lead*e = arrow - dist*v, i.e. lead*e + dist*v = arrow - cam0.
            ax, ay = course.x2 - cam0[0], 0.0 - cam0[1]
            det = e[0] * v[1] - e[1] * v[0]
            if abs(det) < 1e-9:
                continue
            lead = (ax * v[1] - ay * v[0]) / det
            dist = (e[0] * ay - e[1] * ax) / det
            if PHOTO_MIN_CM - 0.01 <= dist <= PHOTO_DIST_CM + 6.0:
                yield _straight(lead) + [Seg(inward, look)], dist, look + KICK_DEG

    def photo_obstacle2(self, course: Course, side1: float) -> float:
        """Pull up for the photo of obstacle 2 and ask its arrow."""
        p = self._pose()
        # Manual and simulator callers do not have an asynchronous camera
        # callback. Ask from the straight bypass lane, which also preserves the
        # space required by the forward-only obstacle-2 handoff. Production
        # installs early_arrow_fn and recognises continuously during this lane.
        if self.early_arrow_fn is None:
            logging.info("Obstacle 2 recognition from the straight bypass "
                         "lane; retaining this pose for ROUND_2.")
            return _ask_arrow(self.input_fn, "Obstacle 2")
        answer = self.early_arrow_fn() if self.early_arrow_fn is not None else None
        if self.early_arrow_fn is not None and answer not in ("left", "right"):
            # Let an in-flight lane image finish before a photo arc uses up
            # the space needed for a forward-only approach.
            for _ in range(3):
                self.sleep_fn(0.1)
                answer = self.early_arrow_fn()
                if answer in ("left", "right"):
                    break
        if answer in ("left", "right"):
            logging.info("Obstacle 2 arrow confirmed; retaining this pose. "
                         "ROUND_2 will plan the handoff once from live WPOSE.")
            return _ask_arrow(self.input_fn, "Obstacle 2")
        ranked = list(self.photo_candidates(p, course, side1))
        segs = self._first_clear(p, [c for c, _, _ in ranked], course,
                                 "move to the obstacle 2 photo pose")
        if segs is None:
            raise RuntimeError("No clear photo pose for obstacle 2.")
        _, d, swing = next(t for t in ranked if t[0] is segs)
        stopped_for_arrow = False
        if self.early_arrow_fn is not None and answer not in ("left", "right"):
            # Candidate generation and swept-path checking are deliberately
            # outside the motion loop. Vision can complete during that work,
            # so close the final check/send race before issuing any movement.
            answer = self.early_arrow_fn()
            if answer in ("left", "right"):
                logging.info("Obstacle 2 arrow completed while checking the "
                             "photo path; retaining the current pose.")
                return _ask_arrow(self.input_fn, "Obstacle 2")
            line = ",".join(s.token() for s in segs)

            def stop_for_arrow() -> bool:
                q = self.frame.pose()
                result = self.early_arrow_fn()
                if result not in ("left", "right"):
                    return False
                self.stm.abort()                     # RST: no reply follows
                self.sleep_fn(0.4)
                if q is not None:
                    self.th = q.th
                logging.info("Obstacle 2 arrow confirmed during the photo "
                             "manoeuvre; RST at the measured handoff pose.")
                return True

            reply = tp.run_polling(self.stm, line, stop_for_arrow)
            if reply is None or not (reply.startswith("OK") or reply == "STOPPED"):
                raise RuntimeError("Move to the photo pose failed: %r" % reply)
            stopped_for_arrow = reply == "STOPPED"
            if not stopped_for_arrow:
                self._track(segs)
        elif not self._line(segs):
            raise RuntimeError("Move to the photo pose failed.")

        if stopped_for_arrow:
            self.stage2_early_stop = True
            logging.info("Early detection retained; skipping the remaining "
                         "photo move and duplicate handoff search.")
            return _ask_arrow(self.input_fn, "Obstacle 2")
        logging.info("Photo pose: camera %.0f cm from the arrow, looking in "
                     "from %d deg.", d, swing - KICK_DEG)
        self.sleep_fn(OBS_PAUSE_S)
        return _ask_arrow(self.input_fn, "Obstacle 2")

    # ── obstacle 2 ────────────────────────────────────────────────────────────

    def face_candidates(self, p: Pose, course: Course, side1: float,
                        reverse: bool, straight_limit: int = 30
                        ) -> Iterator[List[Seg]]:
        """
        From the photo pose to running along obstacle 2's face: turn on to
        heading -side1*90, the IR FACE_GAP_CM off the face. Forward runs
        toward -side1 from there; a reverse run toward side1 must start with
        the IR well inside the face, so it creeps that way first.
        """
        inward = tp._turn_toward(p.th, -side1)
        turn = int(round(90 - abs(math.degrees(p.th))))
        th_t = -side1 * math.pi / 2
        options = []
        for s in range(-straight_limit, straight_limit + 2, 2):
            head = _straight(s) + ([Seg(inward, turn)] if turn >= 1 else [])
            end = tp.run(p, head, self.r, step=1e6)[-1]
            gap = course.x2 - (end.x + tp.IR_SIDE_OFFSET_CM)
            if not (tp.IR_MIN_GAP_CM <= gap <= IR_FACE_MAX_CM - 4):
                continue
            creep = []
            if reverse:
                ir_y = end.y + tp.IR_FROM_AXLE_CM * math.sin(th_t)
                creep = _straight(side1 * ir_y - (-5.0))
                creep = creep if creep and creep[0].kind == "F" else []
            options.append((abs(gap - FACE_GAP_CM), head + creep))
        for _, segs in sorted(options, key=lambda t: t[0]):
            yield segs

    def forward_front_candidates(self, p: Pose, course: Course,
                                 side: float) -> Iterator[List[Seg]]:
        """Face the requested end, then measure it by driving along the front."""
        delta = math.atan2(math.sin(side * math.pi / 2 - p.th),
                           math.cos(side * math.pi / 2 - p.th))
        angle = int(round(abs(math.degrees(delta))))
        if not tp.MIN_ARC_DEG <= angle <= 180:
            return
        turn = Seg("L" if delta > 0 else "R", angle)
        options = []
        for lead in range(0, 62, 2):
            segs = _straight(lead) + [turn]
            end = tp.run(p, segs, self.r, step=1e6)[-1]
            gap = course.x2 - (end.x + tp.IR_SIDE_OFFSET_CM)
            ir_y = end.y + tp.IR_FROM_AXLE_CM * math.sin(end.th)
            # Start over a portion of the face supported by the US hit,
            # not beyond an assumed maximum-width edge.
            known_half = (abs(self.obs2_us_y - course.obs2_y)
                          if self.obs2_us_y is not None else W2_RANGE[0] / 2)
            if (tp.IR_MIN_GAP_CM <= gap <= IR_FACE_MAX_CM - 4
                    and abs(ir_y - course.obs2_y) <= known_half - 4):
                options.append((abs(gap - FACE_GAP_CM), segs))
        for _, segs in sorted(options, key=lambda item: item[0]):
            yield segs

    def same_side_forward_candidates(self, p: Pose, course: Course,
                                     side: float,
                                     straight_limit: int = 60,
                                     direct_only: bool = False
                                     ) -> Iterator[List[Seg]]:
        """Sweep from the same-side photo pose directly behind obstacle 2.

        When both arrows point to the same side, the bypass lane is already
        on the end of obstacle 2 that the robot needs to round. Move closer,
        arc outward, then use a wider opposite arc to arrive behind it. This
        is one continuous, all-forward outer eight: no front-face reverse and
        no second maneuver from a wall-tight +/-90 degree intermediate pose.
        """
        outward = "L" if side > 0 else "R"
        inward = "R" if side > 0 else "L"
        checked = self.same_side_course(course)
        back = checked.x2 + tp.OBS2_DEPTH_CM
        lo = back + tp.IR_SIDE_OFFSET_CM + tp.IR_MIN_GAP_CM + 4.0
        hi = back + tp.IR_SIDE_OFFSET_CM + BACK_GAP_MAX_CM
        options = []
        heading_error = math.degrees(p.th)
        # IR may already have placed us outside the near end. In that case
        # a straight lead and one inward arc need no additional lane shift.
        direct_turn = int(round(90.0 + side * heading_error))
        if tp.MIN_ARC_DEG <= direct_turn <= 180:
            for lead in range(0, straight_limit + 1, 2):
                segs = _straight(lead) + [Seg(inward, direct_turn)]
                end = tp.run(p, segs, self.r, step=1e6)[-1]
                if lo <= end.x <= hi:
                    options.append((tp._cost(segs, self.r),
                                    abs(end.x - (lo + hi) / 2.0), segs))
        if direct_only:
            for _, _, segs in sorted(options, key=lambda item: (item[0], item[1])):
                yield segs
            return
        # Net turn is -side*90 degrees. The second arc therefore exceeds the
        # first by 90 degrees (with the small measured heading error folded
        # in). Candidate distances are searched, not fixed to a floor run.
        for first in (30, 45, 60, 75):
            second = int(round(first + 90.0 + side * heading_error))
            if not tp.MIN_ARC_DEG <= second <= 180:
                continue
            for lead in range(0, straight_limit + 1, 4):
                for middle in range(0, straight_limit + 1, 4):
                    segs = (_straight(lead) + [Seg(outward, first)]
                            + _straight(middle) + [Seg(inward, second)])
                    end = tp.run(p, segs, self.r, step=1e6)[-1]
                    if not (lo <= end.x <= hi):
                        continue
                    options.append((tp._cost(segs, self.r),
                                    abs(end.x - (lo + hi) / 2.0), segs))
        for _, _, segs in sorted(options, key=lambda item: (item[0], item[1])):
            yield segs

    def same_side_lane_candidates(self, p: Pose, course: Course,
                                 side: float) -> Iterator[List[Seg]]:
        """Shift outward, straighten, pass the end, then turn behind it.

        Unlike a single large inward arc, the straight between the lane
        shift and final turn controls where the robot crosses the front face.
        Endpoint calculations are cheap; selection checks the full trace.
        """
        courses = list(self.same_side_courses(course))
        edge = max(c.w2 / 2.0 for c in courses)
        wall = min((c.wall_y if c.wall_y is not None else
                    c.w2 / 2.0 + tp.WALL_CLEARANCE_CM) for c in courses)
        low = edge + tp.HALF_WIDTH_CM + tp.CLEARANCE_FLOOR_CM
        high = wall - tp.HALF_WIDTH_CM - tp.CLEARANCE_FLOOR_CM
        if any(c.walls for c in courses) and low > high:
            return
        outward, inward = ("L", "R") if side > 0 else ("R", "L")
        final = int(round(90 + side * math.degrees(p.th)))
        if not tp.MIN_ARC_DEG <= final <= 180:
            return
        back = course.x2 + tp.OBS2_DEPTH_CM + tp.IR_SIDE_OFFSET_CM
        options = []
        targets = (low, (low + high) / 2.0, high)
        for angle in (15, 30, 45):
            diagonal_y = side * math.sin(p.th + side * math.radians(angle))
            if diagonal_y <= 0.1:
                continue
            for lead in (0, 4, 8, 12):
                base = (_straight(lead) + [Seg(outward, angle),
                                          Seg(inward, angle)])
                end0 = tp.run(p, base, self.r, step=1e6)[-1]
                for target in targets:
                    diagonal = int(round((target - side *
                        (end0.y - course.obs2_y)) / diagonal_y))
                    for middle in (diagonal - 1, diagonal, diagonal + 1):
                        if not 0 <= middle <= 80:
                            continue
                        head = (_straight(lead) + [Seg(outward, angle)]
                                + _straight(middle) + [Seg(inward, angle)])
                        end = tp.run(p, head + [Seg(inward, final)],
                                     self.r, step=1e6)[-1]
                        if math.cos(p.th) < 0.5:
                            continue
                        for gap in range(int(tp.IR_MIN_GAP_CM + 4),
                                         int(BACK_GAP_MAX_CM) + 1, 4):
                            forward = int(round((back + gap - end.x)
                                                / math.cos(p.th)))
                            if 0 <= forward <= 60:
                                segs = head + _straight(forward) + [Seg(inward, final)]
                                options.append((tp._cost(segs, self.r), segs))
        seen = set()
        for _, segs in sorted(options, key=lambda item: item[0]):
            key = tuple(segs)
            if key not in seen:
                seen.add(key)
                yield segs

    def same_side_course(self, course: Course) -> Course:
        """Couple obstacle width and wall position for the outer-eight test.

        The generic pre-measurement course deliberately combines the widest
        obstacle with the nearest wall, even though those cannot occur
        together. If ultrasound hit obstacle 2 from y, its half-width reaches
        at least |y|; under the brief the wall is then another 50 cm out.
        Wider real obstacles also move that wall farther out, so this lower
        bound is conservative for the outward sweep.
        """
        if self.obs2_us_y is None:
            return course
        width = max(W2_RANGE[0], min(W2_RANGE[1],
                                    2.0 * abs(self.obs2_us_y - course.obs2_y)))
        return Course(course.x1, course.x2, width,
                      obs1_y=course.obs1_y, walls=course.walls,
                      wall_y=width / 2.0 + tp.WALL_CLEARANCE_CM,
                      obs2_y=course.obs2_y)

    def same_side_courses(self, course: Course) -> Iterator[Course]:
        """All obstacle widths compatible with the measurements so far.

        A US hit at y only supplies a lower width bound. Check the outer
        eight against every wider brief width, coupling each obstacle width
        to its corresponding wall position. A narrow-lane IR hit measures
        the edge itself and can use that width directly.
        """
        if self.obs2_near_y is not None:
            width = max(W2_RANGE[0], min(W2_RANGE[1],
                        2.0 * abs(self.obs2_near_y - course.obs2_y)))
            widths = [width]
        elif self.obs2_us_y is not None:
            minimum = max(W2_RANGE[0],
                          2.0 * abs(self.obs2_us_y - course.obs2_y))
            # Wall clearance improves monotonically as width grows, while
            # obstacle clearance worsens. Their respective worst cases are
            # therefore the two endpoints; intermediate widths cannot add a
            # stricter constraint.
            maximum = W2_RANGE[1]
            if self.obs2_half_max is not None:
                maximum = min(maximum, 2.0 * self.obs2_half_max)
            widths = [minimum, max(minimum, maximum)]
        else:
            yield course
            return
        for width in sorted(set(widths)):
            yield Course(course.x1, course.x2, width,
                         obs1_y=course.obs1_y, walls=course.walls,
                         wall_y=width / 2.0 + tp.WALL_CLEARANCE_CM,
                         obs2_y=course.obs2_y)

    def opposite_diagonal_candidates(self, p: Pose, course: Course,
                                     side1: float) -> Iterator[List[Seg]]:
        """Cross diagonally, then align with the front face toward side 2."""
        total = int(round(90 + side1 * math.degrees(p.th)))
        if not 2 * tp.MIN_ARC_DEG <= total <= 180 or math.cos(p.th) < 0.5:
            return
        inward = "R" if side1 > 0 else "L"
        minimum_width = min(c.w2 for c in self.same_side_courses(course))
        options = []
        for angle in (30, 45):
            if total - angle < tp.MIN_ARC_DEG:
                continue
            for diagonal in range(4, 45, 4):
                tail = [Seg(inward, angle), Seg("F", diagonal),
                        Seg(inward, total - angle)]
                end0 = tp.run(p, tail, self.r, step=1e6)[-1]
                for gap in (FACE_GAP_CM, tp.IR_MIN_GAP_CM + 4, IR_FACE_MAX_CM - 4):
                    lead = int(round((course.x2 - tp.IR_SIDE_OFFSET_CM - gap
                                      - end0.x) / math.cos(p.th)))
                    if not 0 <= lead <= 60:
                        continue
                    segs = _straight(lead) + tail
                    end = tp.run(p, segs, self.r, step=1e6)[-1]
                    ir_y = end.y + tp.IR_FROM_AXLE_CM * math.sin(end.th)
                    # Finish before the opposite edge, leaving IR a visible
                    # stretch of face before it tracks that edge.
                    if -side1 * (ir_y - course.obs2_y) > minimum_width / 2 - 5:
                        continue
                    options.append((tp._cost(segs, self.r), segs))
        for _, segs in sorted(options, key=lambda item: item[0]):
            yield segs

    def face_handoff_candidates(self, p: Pose, course: Course, side1: float,
                                side2: float, straight_limit: int = 30
                                ) -> Iterator[Tuple[List[Seg], bool]]:
        """Include approach alternatives in the same swept-path validation."""
        approaches = [0]
        heading = math.atan2(math.sin(p.th), math.cos(p.th))
        if abs(heading) <= math.radians(15):
            reserve = max(2 * self.r, tp.FRONT_FROM_AXLE_CM + tp.CLEARANCE_CM)
            distance = int(math.floor(
                (course.x2 - self._front(p)) / math.cos(heading) - reserve))
            if distance >= 2:
                approaches.extend(sorted(set((distance // 2, distance))))
        prefixes = [_straight(distance) for distance in approaches]
        # Vision may interrupt a photo arc before the robot is parallel.
        # Only after the original choices, try a forward heading recovery.
        # Its entire sweep stays part of the candidate, never a blind move.
        angle = int(round(abs(math.degrees(heading))))
        if tp.MIN_ARC_DEG <= angle <= 45:
            prefixes.append([Seg("L" if heading < 0 else "R", angle)])
        deferred = []
        seen = set()
        for prefix in prefixes:
            staged = tp.run(p, prefix, self.r, step=1e6)[-1] if prefix else p
            for segs, reverse in self._turn_handoff_candidates(
                    staged, course, side1, side2, straight_limit):
                combined = []
                for seg in prefix + segs:
                    # A straight approach adjoining a straight lead needs
                    # one distance command, not two stop/start cycles.
                    if (combined and seg.kind == combined[-1].kind == "F"
                            and combined[-1].value + seg.value <= 999):
                        combined[-1] = Seg("F", combined[-1].value + seg.value)
                    else:
                        combined.append(seg)
                key = (tuple(combined), reverse)
                if key in seen:
                    continue
                seen.add(key)
                if reverse:
                    deferred.append((combined, reverse))
                else:
                    yield combined, reverse
        # Do not let a reverse choice at the original pose pre-empt a
        # forward route from another checked approach or alignment prefix.
        yield from deferred

    def _turn_handoff_candidates(self, p: Pose, course: Course, side1: float,
                                side2: float, straight_limit: int = 30
                                ) -> Iterator[Tuple[List[Seg], bool]]:
        """Yield ``(segments, reverse_face_run)`` in preferred order."""
        same_side = side2 == side1
        if same_side:
            # Prefer measuring the requested edge along the front face. A
            # direct route behind the box is a fallback, not the first choice.
            for segs in self.forward_front_candidates(p, course, side2):
                yield segs, False
            for segs in self.same_side_forward_candidates(
                    p, course, side2, straight_limit=max(60, straight_limit),
                    direct_only=True):
                yield segs, False
            for segs in self.same_side_lane_candidates(p, course, side2):
                yield segs, False
            for segs in self.same_side_forward_candidates(
                    p, course, side2, straight_limit=max(60, straight_limit)):
                yield segs, False
        else:
            for segs in self.opposite_diagonal_candidates(p, course, side1):
                yield segs, False
        if same_side:
            return
        for segs in self.face_candidates(p, course, side1, same_side,
                                         straight_limit=straight_limit):
            yield segs, same_side

    def face_handoff_clearance(self, p: Pose, segs: Sequence[Seg],
                               course: Course, side1: float, side2: float,
                               reverse: bool) -> float:
        if side1 == side2 and not reverse:
            return min(self._check(p, segs, checked)
                       for checked in self.same_side_courses(course))
        return self._check(p, segs, course)

    def select_face_handoff(self, p: Pose, course: Course, side1: float,
                            side2: float
                            ) -> Optional[Tuple[List[Seg], bool, float]]:
        """Bound planning time; never execute a partially checked candidate."""
        started = time.monotonic()
        budget = max(0.01, tp._env_f("T2_HANDOFF_PLAN_BUDGET_S", 2.0))
        deadline = started + budget
        checked = 0
        best = None
        first_safe = None

        def finish(candidate):
            segs, reverse, clearance = candidate
            logging.info("Handoff search: %.3fs, %d candidates; %s: %s; "
                         "clearance %.2f cm", time.monotonic() - started,
                         checked, "reverse fallback" if reverse else "forward",
                         ",".join(s.token() for s in segs), clearance)
            if clearance < TURN_CLEARANCE_CM:
                logging.warning("Wider handoff unavailable within search budget: "
                                "using %.2f cm clearance (preferred %.1f cm).",
                                clearance, TURN_CLEARANCE_CM)
            return candidate

        try:
            for segs, reverse in self.face_handoff_candidates(
                    p, course, side1, side2, straight_limit=60):
                if time.monotonic() >= deadline:
                    raise TimeoutError
                if best is not None and (reverse != best[1] or checked - first_safe >= 16):
                    return finish(best)
                courses = (list(self.same_side_courses(course))
                           if side1 == side2 and not reverse else [course])
                clearance = self._handoff_clearance(p, segs, courses, deadline)
                checked += 1
                if clearance < tp.CLEARANCE_FLOOR_CM:
                    continue
                candidate = segs, reverse, clearance
                if clearance >= TURN_CLEARANCE_CM:
                    return finish(candidate)
                if first_safe is None:
                    first_safe = checked
                # At equal clearance, avoid extra command-boundary stops.
                if (best is None or clearance > best[2] + 1e-6
                        or (abs(clearance - best[2]) <= 1e-6
                            and (len(segs), tp._cost(segs, self.r))
                            < (len(best[0]), tp._cost(best[0], self.r)))):
                    best = candidate
        except TimeoutError:
            if best is not None:
                return finish(best)
            logging.error("Handoff planning budget %.2fs exhausted after %d "
                          "candidates; no unchecked movement will be sent.",
                          budget, checked)
            return None
        if best is not None:
            return finish(best)
        logging.info("No clear handoff from %s after %d candidates (%.3fs).",
                      p, checked, time.monotonic() - started)
        return None

    def _handoff_clearance(self, p: Pose, segs: Sequence[Seg],
                           courses: Sequence[Course], deadline: float) -> float:
        # A circumscribed circle gives a lower distance bound. Skip exact
        # polygon checks only when that bound cannot lower the current minimum.
        boxes = list(dict.fromkeys(b for c in courses for b in c.boxes()
                                   if b.name != "back fence"))
        reach = math.hypot(max(tp.FRONT_FROM_AXLE_CM, tp.REAR_FROM_AXLE_CM),
                           tp.HALF_WIDTH_CM)
        # Reject impossible joins before sampling the full sweep. Passing
        # this inexpensive filter never replaces the full collision check.
        for pose in tp.run(p, segs, self.r, step=1e6):
            if time.monotonic() >= deadline:
                raise TimeoutError
            for box in boxes:
                dx = max(box.x0 - pose.x, pose.x - box.x1, 0.0)
                dy = max(box.y0 - pose.y, pose.y - box.y1, 0.0)
                if math.hypot(dx, dy) - reach >= tp.CLEARANCE_FLOOR_CM:
                    continue
                clearance = tp.clearance(pose, box)
                if clearance < tp.CLEARANCE_FLOOR_CM:
                    return clearance
        best = float("inf")
        for pose in tp.run(p, segs, self.r):
            if time.monotonic() >= deadline:
                raise TimeoutError
            for box in boxes:
                dx = max(box.x0 - pose.x, pose.x - box.x1, 0.0)
                dy = max(box.y0 - pose.y, pose.y - box.y1, 0.0)
                if math.hypot(dx, dy) - reach >= best:
                    continue
                best = min(best, tp.clearance(pose, box))
                if best < tp.CLEARANCE_FLOOR_CM:
                    return best
        return best

    def follow_face(self, token: str, x_face: float, facing: float,
                    toward: float, window: float,
                    check_front: bool = False) -> Optional[Tuple[float, float]]:
        """
        Drive `token` (a straight, forward or reverse) along one face of
        obstacle 2 with the side IR on it, and RST where the face ends.

        x_face: the face's x as far as anything else knows - for the log
        only. facing: +1 if the IR looks along +x at it (the front face), -1
        along -x (the back face). toward: which way along y the robot is
        going. window: a reading this far off the face's running median is
        no longer the face.

        What counts as the face does NOT depend on x_face. That came from the
        ultrasound, taken off-centre from the obstacle 1 lane, and on the
        floor it was out by more than any sensible window, so the face was
        never recognised. Square to obstacle 2 and in front of (or behind) it,
        the only thing the IR can reach is obstacle 2: any steady in-range
        reading is the face, and its end is where readings vanish or jump.

        Returns (y where the face ends, the face's x as the IR measured it),
        both in odometry, or None.

        Wall guard: a wall can stand 50 cm past obstacle 2's end, and the end
        is at least as far on as the face the IR last saw. So whatever the
        readings do, the robot's leading edge never goes more than 44 cm (50
        less the 6 cm margin) past that - before anything is seen, past where
        the IR started, which was put in front of the face.
        """
        self.last_face_start_y = None
        self.last_stopped_pose = None
        p = self._pose()
        side = self._ir_facing(p.th, facing)
        ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)     # IR y - axle y
        forward = not token.upper().startswith("R")
        lead_off = tp.FRONT_FROM_AXLE_CM if forward else tp.REAR_FROM_AXLE_CM
        seen, lost, raw = [], [], []
        ref = [p.y + ahead]                              # face known to here
        guard = []
        front_blocked = []
        # Empty floor can look like a 30-50 cm target, so the front face keeps
        # the strict cutoff.  Behind obstacle 2 the collision-checked turn has
        # already placed this IR beside the back face; floor testing measured
        # that face at 28-33 cm.  BACK_GAP_MAX_CM exists for precisely this
        # wider rear lane and must not be clipped by the front-face limit.
        expected_gap = facing * (
            x_face - (p.x + facing * tp.IR_SIDE_OFFSET_CM))
        if facing < 0:
            lo = tp.IR_MIN_GAP_CM - 4.0
            hi = min(BACK_GAP_MAX_CM, tp.IR_MAX_GAP_CM[side])
        else:
            lo = max(tp.IR_MIN_GAP_CM - 4.0,
                     expected_gap - FRONT_FACE_GAP_TOL_CM)
            hi = min(tp.IR_MAX_GAP_CM[side], FRONT_FACE_HARD_MAX_CM,
                     max(IR_FACE_MAX_CM,
                         expected_gap + FRONT_FACE_GAP_TOL_CM))
            logging.info("Front-face IR acquisition window %.0f..%.0f cm "
                         "from predicted %.1f cm gap.", lo, hi, expected_gap)

        def stop() -> bool:
            self.stm.abort()                             # RST: no reply comes
            self.last_stopped_pose = self._wait_for_stopped_pose()
            return True

        def poll(moving: bool = True) -> bool:
            g = self._ir(side)
            if check_front:
                distance = self._us()
                if distance is None or not math.isfinite(distance) or distance <= 20.0:
                    front_blocked.append(distance)
                    logging.error("Forward face scan blocked by US reading %r cm.", distance)
                    return stop() if moving else True
            q = self.frame.pose()
            if q is None:
                if check_front:
                    front_blocked.append(None)
                    return stop() if moving else True
                return False
            raw.append(g)
            ir_x = q.x + facing * tp.IR_SIDE_OFFSET_CM
            steady = True
            if seen:                                     # the gap it has been reading
                recent = sorted(gap for _, _, gap in seen[-5:])
                steady = g is not None and abs(g - recent[len(recent) // 2]) <= window
            if g is not None and lo <= g <= hi and steady:
                seen.append((q.y + ahead, ir_x + facing * g, g))
                ref[0] = q.y + ahead
                lost.clear()
            elif seen:
                # Two sightings and three misses in a row: a narrow face
                # only gives a few readings before its end.
                lost.append(q.y + ahead)
                if len(seen) >= 2 and len(lost) >= 3:
                    return stop() if moving else True
            if toward * (q.y + toward * lead_off - ref[0]) > tp.WALL_CLEARANCE_CM - tp.CLEARANCE_CM:
                guard.append(q.y)
                return stop() if moving else True
            return False

        logging.info("Along obstacle 2's %s face, %s IR on it: %s",
                     "front" if facing > 0 else "back", side, token)
        # A narrow back face can leave the beam before two moving polls.
        # Confirm what is already alongside us before beginning the scan.
        for _ in range(2):
            self.sleep_fn(0.03)
            if poll(moving=False):
                logging.error("Face scan cannot start: clearance/pose guard triggered.")
                return None
        logging.info("Face scan stationary acquisition: %d valid sightings on %s IR.",
                     len(seen), side)
        reply = tp.run_polling(self.stm, token, poll)
        if reply is None or not (reply.startswith("OK") or reply == "STOPPED"):
            logging.error("STM answered %r along obstacle 2.", reply)
            return None
        # OK can arrive between the last moving sample and the third miss.
        # Finish observing at rest, never extend the movement to seek an edge.
        if reply.startswith("OK") and len(seen) >= 2 and len(lost) < 3 and not guard:
            logging.info("Face scan completed; confirming its edge at rest "
                         "without additional movement.")
            for _ in range(3):
                self.sleep_fn(0.03)
                if poll(moving=False):
                    break
        failed = None
        if front_blocked:
            failed = "front clearance or pose unavailable; not an obstacle edge"
        elif guard:
            failed = "stopped short of where a wall could be, never seeing its end"
        elif len(seen) < 2 or len(lost) < 3:
            failed = "never lost it" if seen else "never saw it"
        if failed:
            # The poll lines are filtered out of the log; say what it read.
            vals = sorted(g for g in raw if g is not None)
            logging.error(
                "The %s IR saw obstacle 2's face %d times and %s. It read %d "
                "times: %d blank, %s. Expected about %.0f cm if the face is at "
                "x=%.0f; it only counts %.0f..%.0f cm.", side, len(seen), failed,
                len(raw), len(raw) - len(vals),
                (f"min {vals[0]:.0f} / median {vals[len(vals) // 2]:.0f} / max "
                 f"{vals[-1]:.0f} cm") if vals else "no numbers",
                facing * (x_face - (p.x + facing * tp.IR_SIDE_OFFSET_CM)),
                x_face, lo, hi)
            return None
        if reply == "STOPPED" and self.last_stopped_pose is None:
            logging.error("Face edge was seen, but the robot did not settle; "
                          "not planning a corner from a moving pose.")
            return None
        # A reading describes the beam IR_LAG_CM back along the way it went.
        end_y = (seen[-1][0] + lost[0]) / 2.0 - toward * tp.IR_LAG_CM
        self.last_face_start_y = seen[0][0] - toward * tp.IR_LAG_CM
        xs = sorted(x for _, x, _ in seen)
        return end_y, xs[len(xs) // 2]

    @staticmethod
    def _ir_facing(th: float, facing: float) -> str:
        """The side IR looking along facing * x, at heading +-90."""
        return "right" if math.sin(th) * facing > 0 else "left"

    def round_near_end(self, p: Pose, course: Course, toward: float,
                       reverse: bool, near_y: float,
                       forward_only: bool = False) -> Optional[List[Seg]]:
        """
        Round obstacle 2's `toward` end and on to behind it: two 90 deg turns
        the same way (clockwise for toward = +y), ending heading -toward*90
        with the IR on the back face within BACK_GAP_MAX_CM of it, ready to run along
        it. The shortest clear one wins.

        The transition is forward-only. Once the side IR has found the edge,
        the robot continues around that edge instead of steering toward the
        obstacle and reversing back out.

        near_y: where that end is. The IR, ahead of the axle, must start the
        back-face run no more than BACK_START_MAX_CM past it, or a 30 cm
        obstacle has gone by before the IR has seen enough of it.
        """
        same = "R" if toward > 0 else "L"
        boxes = [b for b in course.boxes() if "fence" not in b.name]
        back = course.x2 + tp.OBS2_DEPTH_CM
        lo = back + tp.IR_SIDE_OFFSET_CM + tp.IR_MIN_GAP_CM + 4.0
        hi = back + tp.IR_SIDE_OFFSET_CM + BACK_GAP_MAX_CM

        def firsts():
            # The edge has been measured and RST has settled. Continue around
            # that edge; never turn the wrong way and reverse back toward it.
            for a in range(0, 46, 2):
                if not reverse:
                    yield a, _straight(a) + [Seg(same, 90)]
                    continue
                # Retain compatibility with old reverse-face callers without
                # adding a backward movement to the edge transition itself.
                for th1 in (45, 30, 60, 0):
                    yield a, (_straight(a)
                              + ([Seg(same, th1)] if th1 else [])
                              + [Seg(_other(same), 90 - th1)])

        # The brief permits only 50 cm between obstacle 2 and the wall. With
        # the 30 cm TIGHT radius, a measured edge pose can leave about 5 cm
        # body clearance even though the nominal 6 cm preference cannot fit.
        # This is exactly what CLEARANCE_FLOOR_CM is for: retain collision
        # checking, but do not stop beside a cleared edge over one centimetre.
        needs = (max(TURN_CLEARANCE_CM, tp.CLEARANCE_CM),
                 tp.CLEARANCE_CM, tp.CLEARANCE_FLOOR_CM)
        for need in dict.fromkeys(needs):
            best, best_len = None, float("inf")
            for a, first in firsts():
                if a >= best_len:
                    break
                # The opening alone already touching: no b can help.
                if _path_clear(p, first, self.r, boxes, need) is None:
                    continue
                for b in range(0, 80, 2):
                    if a + b >= best_len:
                        break
                    segs = first + _straight(b) + [Seg(same, 90)]
                    end = tp.run(p, segs, self.r)[-1]
                    ir_y = end.y + tp.IR_FROM_AXLE_CM * math.sin(end.th)
                    if toward * (ir_y - near_y) < -BACK_START_MAX_CM:
                        break                      # more b only moves it in x
                    if end.x < lo:
                        continue
                    if end.x > hi:
                        break
                    if _path_clear(p, segs, self.r, boxes, need) is None:
                        continue
                    best, best_len = segs, a + b
                    break
            if best is not None:
                if need < tp.CLEARANCE_CM - 0.01:
                    gap, name = tp.path_clearance(
                        tp.run(p, best, self.r), boxes)
                    logging.warning("Round obstacle 2's end uses the measured-"
                                    "pose fallback: %.1f cm clearance to %s "
                                    "(%.1f cm floor).", gap, name, need)
                return best
        return None

    def execute_obstacle2_corner(self, segs: Sequence[Seg], course: Course) -> bool:
        """Check the remaining body sweep from measured poses between moves."""
        self._set_fast(False)
        if self.fast:
            logging.error("Obstacle 2 corner: precision speed unavailable; not moving.")
            return False
        boxes = course.boxes()
        required = tp.CLEARANCE_CM
        for i, seg in enumerate(segs):
            p = self._pose()
            if not all(math.isfinite(v) for v in (p.x, p.y, p.th)):
                logging.error("Obstacle 2 corner: invalid measured pose.")
                return False
            gap, name = tp.path_clearance(tp.run(p, segs[i:], self.r), boxes)
            if i == 0 and gap < required:
                if gap < tp.CLEARANCE_FLOOR_CM:
                    logging.error("Obstacle 2 corner has only %.1f cm "
                                  "clearance to %s; below the %.1f cm floor. "
                                  "Not moving.", gap, name,
                                  tp.CLEARANCE_FLOOR_CM)
                    return False
                required = tp.CLEARANCE_FLOOR_CM
                logging.warning("Obstacle 2 corner is using its checked %.1f "
                                "cm clearance to %s (%.1f cm floor).",
                                gap, name, required)
            if gap < required:
                logging.error("Obstacle 2 corner: remaining sweep has %.1f cm "
                              "clearance to %s; requires %.1f cm. Not sending %s.",
                              gap, name, required, seg.token())
                return False
            expected = tp.run(p, [seg], self.r)[-1]
            logging.info("Obstacle 2 corner %d/%d: %s, measured pose %s, "
                         "remaining clearance %.1f cm to %s (precision speed).",
                         i + 1, len(segs), seg.token(), p, gap, name)
            if not self._line([seg]):
                return False
            actual = self._pose()
            if not all(math.isfinite(v) for v in (actual.x, actual.y, actual.th)):
                logging.error("Obstacle 2 corner: invalid completion pose.")
                return False
            offset = math.hypot(actual.x - expected.x, actual.y - expected.y)
            angle = abs(math.degrees(math.atan2(math.sin(actual.th - expected.th),
                                                math.cos(actual.th - expected.th))))
            logging.info("Obstacle 2 corner %s measured error: %.1f cm / %.1f deg.",
                         seg.token(), offset, angle)
            # A finite error does not by itself make the next move unsafe.
            # The next loop iteration plans the complete remaining sweep from
            # this measured pose before sending anything. Stopping here used
            # to strand the robot just beyond an otherwise successful turn.
            if offset > 3.0 or angle > 5.0:
                logging.warning("Obstacle 2 corner deviated from its nominal "
                                "endpoint; replanning the remaining sweep from "
                                "measured WPOSE before continuing.")
        return True

    def turn_home(self, p: Pose, course: Course, toward: float,
                  forward_only: bool = False
                  ) -> Optional[List[Seg]]:
        """
        From running along the back face past obstacle 2's far end: on (or
        back) and the last 90 deg turn, heading home HOME_GAP_CM off that end.
        """
        same = "R" if toward > 0 else "L"
        far = course.obs2_y - toward * course.w2 / 2.0
        y_home = far - toward * (tp.HALF_WIDTH_CM + HOME_GAP_CM)
        boxes = [b for b in course.boxes() if "fence" not in b.name]
        end = tp.run(p, [Seg(same, 90)], self.r)[-1]
        c = toward * (end.y - y_home)
        options = [c + d for d in (0, 3, 6, 9, -3)]
        if forward_only:
            options = sorted(set(max(0.0, cc) for cc in options))
        for need in (TURN_CLEARANCE_CM, tp.CLEARANCE_CM - 0.01, tp.CLEARANCE_FLOOR_CM):
            for cc in options:
                segs = _straight(cc) + [Seg(same, 90)]
                if _path_clear(p, segs, self.r, boxes, need) is not None:
                    return segs
        return None

    def prepare_forward_lane(self, course: Course, side: float) -> bool:
        """Use an early, checked outward lane shift to reduce width uncertainty.

        A matching front echo supplies a larger lower bound on the face's
        extent. Two misses are an upper bound only because an inner-lane hit
        has already confirmed the same face; an unconfirmed miss stays unsafe.
        """
        if self.obs2_near_y is not None or self.obs2_us_y is None:
            return False
        p = self._pose()
        if abs(math.atan2(math.sin(p.th), math.cos(p.th))) > math.radians(2):
            return False
        target = min(SENSING_LANE_CM, W2_RANGE[1] / 2.0 - 5.0)
        if side * (p.y - course.obs2_y) >= target - 1.0:
            return False
        segs = tp.lane_change(p.th, course.obs2_y + side * target - p.y, self.r)
        if not segs or any(s.kind not in ("F", "L", "R") for s in segs):
            return False
        clearance = min(self._check(p, segs, c) for c in self.same_side_courses(course))
        if clearance < tp.CLEARANCE_FLOOR_CM:
            logging.warning("Too late for the forward sensing lane: %.1f cm clearance.",
                            clearance)
            return False
        logging.info("Early forward sensing lane: %s; clearance %.1f cm.",
                     ",".join(s.token() for s in segs), clearance)
        if not self._line(segs):
            return False
        q = self._pose()
        hits = [self._us(), self._us()]
        if all(g is None for g in hits):
            # Obstacle 2 was already confirmed from the inner lane. Two misses
            # from this farther, checked parallel lane mean its face ends before
            # this ray. Use that as an upper width bound and replan from WPOSE.
            dx = course.x2 - self._front(q)
            ray_y = q.y + math.tan(q.th) * dx
            self.obs2_half_max = abs(ray_y - course.obs2_y)
            logging.info("Forward sensing lane is outside obstacle 2's face; "
                         "measured half-width is %.1f..%.1f cm. Replanning "
                         "from the measured pose.",
                         abs(self.obs2_us_y - course.obs2_y),
                         self.obs2_half_max)
            return True
        if (any(g is None or not math.isfinite(g) or g <= 0 for g in hits)
                or abs(hits[0] - hits[1]) > 3.0
                or any(abs(q.x + (tp.FRONT_FROM_AXLE_CM + g) * math.cos(q.th)
                           - course.x2) > 8.0 for g in hits)):
            logging.error("Forward sensing lane did not confirm obstacle 2: US=%s; "
                          "not assuming the unseen space is clear.", hits)
            return False
        self.obs2_us_y = (q.y + (tp.FRONT_FROM_AXLE_CM + sum(hits) / 2.0)
                         * math.sin(q.th))
        logging.info("Obstacle 2 front reconfirmed at ray y=%.1f; replanning "
                     "the forward route from the measured pose.", self.obs2_us_y)
        return True

    def around_obstacle2(self, course: Course, side1: float, side2: float
                         ) -> Optional[Course]:
        """
        Go round obstacle 2 in side2's direction, measuring both its ends on
        the way. Returns the measured course, or None.

          1. Turn on to its front face, run along it toward side2 (forward if
             that is away from side1, in reverse if toward it): its side2 end.
          2. Round that end to behind it.
          3. Run along its back face: its far end.
          4. The last turn, heading home off that far end.

        Both ends are measured in odometry, which has drifted by the time the
        robot gets here, so nothing assumes obstacle 2 sits on the centreline.
        """
        p = self._pose()
        # Candidate sequences already include their straight leads. Validate
        # the approach AND turn from this pose before sending either one.
        selected = self.select_face_handoff(p, course, side1, side2)
        if selected is None and side1 == side2:
            if self.prepare_forward_lane(course, side2):
                p = self._pose()
                selected = self.select_face_handoff(p, course, side1, side2)
        if selected is None:
            logging.error("No checked forward obstacle-2 route available; not moving.")
            return None
        segs, reverse, clearance = selected
        if side1 == side2 and (reverse or any(s.kind in ("B", "l", "r") for s in segs)):
            logging.error("Same-side handoff rejected: reverse is disabled.")
            return None
        logging.info("turn onto obstacle 2's face: clearance %.1f cm", clearance)
        if not self._line(segs):
            return None
        if side2 == side1:
            logging.info("Same-side obstacle arrows: forward-only handoff; reverse disabled.")

        # The ends have to be found accurately, not quickly.
        self._set_fast(False)
        try:
            outer = (side2 == side1 and not reverse
                     and side2 * math.sin(self._pose().th) < 0)
            if outer:
                # The selected handoff is the complete all-forward outer
                # eight and already ends behind obstacle 2. Traverse its back
                # face now; the old front-face reverse is not part of this
                # branch.
                p = self._pose()
                ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)
                limit = course.obs2_y - side2 * (W2_RANGE[1] / 2.0 + 15.0)
                run = max(10.0, -side2 * (limit - (p.y + ahead)))
                got = self.follow_face(
                    f"F{int(round(run))}",
                    course.x2 + tp.OBS2_DEPTH_CM, -1.0, -side2, 12.0)
                if got is None or self.last_face_start_y is None:
                    return None
                far, back_x = got
                near = self.last_face_start_y
                if self.obs2_near_y is not None:
                    near = self.obs2_near_y
                else:
                    # Entering the back face midway does not measure its near
                    # edge. The course puts the cuboid on the centreline;
                    # reflect the measured far edge, without inventing a
                    # lateral odometry correction from the first IR sighting.
                    near = 2.0 * course.obs2_y - far
                    logging.info("Outer-eight near edge inferred from the "
                                 "course centreline, not measured by IR: %.1f.",
                                 near)
                x2 = back_x - tp.OBS2_DEPTH_CM
                logging.info("Outer-eight measured obstacle 2 from its back: "
                             "%s-end bound y=%.1f, far end y=%.1f, front "
                             "x=%.1f.",
                             "left" if side2 > 0 else "right", near, far, x2)
            else:
                p = self._pose()
                limit = side2 * (W2_RANGE[1] / 2.0 + 15.0)   # IR past any end
                ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)
                run = max(10.0, side2 * (limit - (p.y + ahead)))
                got = self.follow_face(
                    f"{'R' if reverse else 'F'}{int(round(run))}",
                    course.x2, +1.0, side2, 18.0,
                    check_front=side1 == side2)
                if got is None:
                    return None
                near, x2 = got
                # The IR, square to the face from under 30 cm, over the
                # ultrasound taken off-centre from the lane: on the floor that
                # read 24 cm short.
                if abs(x2 - course.x2) > 8.0:
                    logging.warning("IR puts obstacle 2's face at x=%.1f, the "
                                    "ultrasound said %.1f - going by the IR.",
                                    x2, course.x2)
                logging.info("Obstacle 2's %s end at y=%.1f; face at x=%.1f "
                             "(IR; ultrasound said %.1f).",
                             "left" if side2 > 0 else "right", near, x2,
                             course.x2)

                # Until the far end is seen: as wide as the brief allows that way.
                near_y = near + side2 * END_MARGIN_CM
                wide = W2_RANGE[1] + 2 * END_MARGIN_CM
                # The margin moves the end out and the wall IN: both errs safe.
                partial = Course(
                    course.x1, x2, wide,
                    obs2_y=near_y - side2 * wide / 2.0,
                    wall_y=(wide / 2.0 + tp.WALL_CLEARANCE_CM
                            - 2 * END_MARGIN_CM))
                p = self.last_stopped_pose or self._pose()
                segs = self.round_near_end(p, partial, side2, reverse, near,
                                           forward_only=True)
                if segs is None:
                    logging.error("No clear way round obstacle 2's end from %s.", p)
                    return None
                if not self.execute_obstacle2_corner(segs, partial):
                    return None

                self._set_fast(False)
                p = self._pose()
                ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)
                limit = near - side2 * (W2_RANGE[1] + 15.0)
                run = max(10.0, -side2 * (limit - (p.y + ahead)))
                got = self.follow_face(
                    f"F{int(round(run))}", x2 + tp.OBS2_DEPTH_CM,
                    -1.0, -side2, 12.0)
                if got is None:
                    return None
                far, _ = got
        finally:
            self._set_fast(True)

        near_y = near + side2 * END_MARGIN_CM
        far_y = far - side2 * END_MARGIN_CM
        w2 = abs(near_y - far_y)
        centre = (near + far) / 2.0
        logging.info("Obstacle 2 runs y=%.1f..%.1f: %.0f cm wide, centred %.1f "
                     "off the centreline (odometry).", min(near, far),
                     max(near, far), abs(near - far), centre)
        if not (W2_RANGE[0] - 10 <= abs(near - far) <= W2_RANGE[1] + 10):
            logging.error("That is outside the brief's 30-90 cm - the IR saw "
                          "something else.")
            return None
        # The brief puts obstacle 2 on the centreline, so where its centre
        # reads is odometry drift - ~3 m of it by now, and the IR fix on
        # obstacle 1 on the way home only looks a window either side of
        # where it expects. Take it out. Capped: more than that and it is
        # more likely the obstacle really is off the line.
        shift = -max(-Y_RESYNC_MAX_CM, min(Y_RESYNC_MAX_CM, centre))
        if abs(shift) >= 0.5:
            self.frame.y_shift += shift
            near_y, far_y = near_y + shift, far_y + shift
            logging.info("Odometry y moved %+.1f cm to put obstacle 2 on the "
                         "centreline.", shift)
        measured = Course(course.x1, x2, w2, obs2_y=(near_y + far_y) / 2.0,
                          wall_y=w2 / 2.0 + tp.WALL_CLEARANCE_CM - 2 * END_MARGIN_CM)
        p = self._pose()
        segs = self.turn_home(p, measured, side2, forward_only=True)
        if segs is None:
            logging.error("No clear last turn home from %s.", p)
            return None
        logging.info("Round obstacle 2 to the %s; heading home: %s",
                     "left" if side2 > 0 else "right",
                     ",".join(s.token() for s in segs))
        if not self._line(segs):
            return None
        return measured

    # ── run ───────────────────────────────────────────────────────────────────

    def run(self) -> int:
        if self.startup and not self._startup():
            return 2
        if not self.frame.anchor():
            logging.error("No ?WPOSE - flash firmware with protocol >= 6.")
            return 2
        if self.wait_for_go:
            self.input_fn("Robot in the bay, square, on the centreline. "
                          "Press Enter to GO. ")
        t0 = time.monotonic()
        try:
            hit = self.obstacle1()
            if hit is None:
                return 1
            x1, side1 = hit
            x2 = self.pass_obstacle1(x1, side1)
            if x2 is None:
                return 1
            w2 = CONSERVATIVE_W2
            if x2 == NARROW:
                x2 = self.find_narrow(x1, side1)
                if x2 is None:
                    return 1
                w2 = 2 * NARROW_HALF_MAX
            course = Course(x1, x2, w2, wall_y=CONSERVATIVE_WALL_Y)
            side2 = self.photo_obstacle2(course, side1)
            course = self.around_obstacle2(course, side1, side2)
            if course is None:
                return 1
            if not tp.ParkRunner(self.stm, self.frame, course).run():
                return 1
        except KeyboardInterrupt:
            logging.warning("Interrupted - RST.")
            self.stm.abort()
            return 2
        except RuntimeError as exc:
            logging.error("%s", exc)
            return 1
        logging.info("Task 2 route finished in %.1f s; caller must verify parking.",
                     time.monotonic() - t0)
        return 0

    def _startup(self) -> bool:
        from communications import cal_profile
        from communications.stm import WPOSE_MIN_PROTOCOL
        try:
            self.stm.connect()
        except Exception as exc:
            logging.error("Could not open the serial link: %s", exc)
            return False
        proto = cal_profile.firmware_protocol(self.stm)
        if proto is None or proto < WPOSE_MIN_PROTOCOL:
            logging.error("Firmware protocol %s; Task 2 needs >= %d (?WPOSE).",
                          proto, WPOSE_MIN_PROTOCOL)
            return False
        name = (os.getenv("T2T_CAL_PROFILE", "").strip()
                or os.getenv("T2_CAL_PROFILE", "").strip() or None)
        if not cal_profile.prepare_for_task(self.stm, name=name,
                                            allowed_arcs={cal_profile.ARC_TIGHT},
                                            fast=FAST):
            logging.error("STM startup failed (see above) - not running Task 2.")
            return False
        from communications.stm import FAST_MIN_PROTOCOL
        self.fast = FAST and proto >= FAST_MIN_PROTOCOL
        return True


class _QuietPolling(logging.Filter):
    """The STM client logs every query; the IR watches poll ?IR and ?WPOSE ~50
    times a second, which buries everything else. Drop those lines."""

    def filter(self, record):
        msg = record.getMessage()
        return not (msg.startswith("STM → stm: ?IR")
                    or msg.startswith("STM → stm: ?WPOSE")
                    or msg.startswith("STM ← stm: IR,")
                    or msg.startswith("STM ← stm: WPOSE,"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--verbose", action="store_true",
                    help="log every ?IR / ?WPOSE poll too")
    if not ap.parse_args().verbose:
        for h in logging.getLogger().handlers:
            h.addFilter(_QuietPolling())
    from communications.stm import STM
    stm = STM()
    try:
        return Task2(stm).run()
    finally:
        try:
            stm.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
