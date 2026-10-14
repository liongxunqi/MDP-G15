"""
task2_safe.py - Legacy manual SAFE-profile bench route, not the competition entry point.

    python3 task2.py                 # on the robot
    python3 task2_sim.py 90 80 50    # the same code against a simulated STM

TEST BUILD. The run starts on Enter, not the Android button. At obstacle 1
it asks you for the arrow (left / right); at obstacle 2 it pauses 5 s where
the photo would be taken. Everything else is measured by the robot.

The route, in the world frame of task2_park.py (origin the bay mouth, +x out).
Its path goes round obstacle 1 - out on one side, home on the other - and
turns back in front of obstacle 2:

  1. Ultrasound obstacle 1, F + FU to a standoff, ask for its arrow.
  2. Lane change to obstacle 1's arrow side, just clear of it, and ultrasound
     obstacle 2 from there. If that misses (a narrow obstacle 2 is off the
     beam), widen the lane, drive on until the inner IR sees obstacle 2's
     side, RST, and take its distance from that.
  3. Photo pose: turned 45 deg toward the centreline, aimed at the arrow, the
     camera 30 cm from it. Pause 5 s.
  4. U-turn across the front of obstacle 2 - the rest of the turn, a straight
     that sets the lane, the last 90 deg - onto the home lane on the OTHER
     side of obstacle 1.
  5. task2_park.ParkRunner: IR fix on obstacle 1, S onto the centreline, FU
     off the back fence.

Task 2 drives the SAFE arc profile, on its own calibration: set
T2_CAL_PROFILE in .env (falls back to CAL_PROFILE).
"""

import argparse
import logging
import math
import os
import sys
import time
from typing import Callable, List, Optional, Sequence, Tuple

import task2_park as tp
from task2_park import Course, Pose, Seg

# Body gap kept past obstacle 1 on the way out. The lane is chosen as tight
# as this allows so the ultrasound, at the robot's centreline, still lands on
# obstacle 2's face when it is only 30 cm wide.
OBS1_PASS_GAP_CM = tp._env_f("T2_OBS1_PASS_GAP_CM", 6.0)
OBS2_PAUSE_S = tp._env_f("T2_OBS2_PAUSE_S", 5.0)
# Photo of obstacle 2: turned this far toward the centreline, the robot's
# camera this far from the arrow. The camera sits CAMERA_FROM_AXLE_CM ahead
# of the rear axle (default: the front bumper - measure yours).
PHOTO_TURN_DEG = int(tp._env_f("T2_PHOTO_TURN_DEG", 45))
PHOTO_DIST_CM = tp._env_f("T2_PHOTO_DIST_CM", 30.0)
CAMERA_FROM_AXLE_CM = tp._env_f("T2_CAMERA_FROM_AXLE_CM", tp.FRONT_FROM_AXLE_CM)
# Lane the U-turn lands on, |y| from the centreline: past obstacle 1 with the
# IR ~20 cm off it, for the fix on the way home.
HOME_LANE_CM = tp._env_f("T2_HOME_LANE_CM", 30.0)
# Brief: obstacle 2 is 30-90 wide, 60-150 past obstacle 1. Measurements
# outside these (with slack) mean the sensor saw something else.
W2_RANGE = (30.0, 90.0)
GAP12_RANGE = (45.0, 175.0)
# Obstacle 2's width is never measured: plan against its widest case and the
# walls of its narrowest, together.
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


def _ask_arrow(input_fn: Callable[[str], str]) -> float:
    while True:
        ans = input_fn("Obstacle 1 arrow - left or right? ").strip().lower()
        if ans in ("l", "left"):
            return 1.0
        if ans in ("r", "right"):
            return -1.0
        print("Type left or right (l / r).")


class Task2:

    def __init__(self, stm, input_fn: Callable[[str], str] = input,
                 sleep_fn: Callable[[float], None] = time.sleep,
                 startup: bool = True, wait_for_go: bool = True):
        self.stm = stm
        self.input_fn = input_fn
        self.sleep_fn = sleep_fn
        self.startup = startup
        self.wait_for_go = wait_for_go
        self.frame = tp.WorldFrame(stm)
        self.th = 0.0                 # commanded heading, radians
        self.r = tp.RADIUS_CM

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
        for s in segs:
            if isinstance(s, Seg) and s.kind in ("L", "R"):
                self.th += math.radians(s.value) * (1 if s.kind == "L" else -1)
        return True

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

    @staticmethod
    def _front(p: Pose) -> float:
        return p.x + tp.FRONT_FROM_AXLE_CM * math.cos(p.th)

    def _check(self, p: Pose, segs, course: Course, fence: bool = False) -> float:
        """Closest approach along segs. The fence is left out on the way out:
        the robot starts 2 cm off it and only drives away from it."""
        boxes = course.boxes() if fence else [
            b for b in course.boxes() if "fence" not in b.name]
        return tp.path_clearance(tp.run(p, segs, self.r), boxes)[0]

    def _first_clear(self, p: Pose, candidates, course: Course, what: str):
        """
        The first candidate keeping CLEARANCE_CM. Failing that, the best one
        keeping CLEARANCE_FLOOR_CM, with a warning: by then the robot is
        already where it is, and stopping dead loses the run where a close
        pass only risks a touch.
        """
        # Never stricter than the gap the robot already has where it stands:
        # a move that only backs away from it is not the problem.
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

    # ── phases ────────────────────────────────────────────────────────────────

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
        # The latest place the lane change can start and still clear it: the
        # standoff. Same on either side, so it is found before the arrow is.
        lead = None
        for L in range(0, int(x1 - p.x)):
            segs = _straight(L) + tp.lane_change(0.0, lane, self.r)
            if self._check(p, segs, course) < tp.CLEARANCE_CM - 0.01:
                break
            lead = L
        if lead is None:
            logging.error("Obstacle 1 is too close to the bay to get round.")
            return None
        standoff = x1 - (p.x + lead + tp.FRONT_FROM_AXLE_CM)
        travel = g - standoff
        bulk = travel - tp.FU_LEAD_CM
        tokens = ([f"F{int(bulk)}"] if bulk >= 5 else []) + \
                 [f"FU{int(round(standoff + tp.US_BIAS_CM))}"]
        logging.info("Obstacle 1 face at x=%.1f (%.1f ahead). Standing off at "
                     "%.1f.", x1, g, standoff)
        if not self._line(tokens):
            return None
        g = self._us()
        if g is not None:
            x1 = self._front(self._pose()) + g
        return x1, _ask_arrow(self.input_fn)

    def pass_obstacle1(self, x1: float, side1: float) -> Optional[float]:
        """Lane change to side1, ultrasound obstacle 2. Returns its face x,
        NARROW if the ultrasound missed it, or None."""
        p = self._pose()
        lane = tp.OBS1_SIZE_CM / 2 + tp.HALF_WIDTH_CM + OBS1_PASS_GAP_CM
        course = Course(x1, x1 + tp.OBS1_SIZE_CM + 60.0, CONSERVATIVE_W2,
                        wall_y=CONSERVATIVE_WALL_Y)
        # FU lands a cm or two off; a lead of -8..30 absorbs it.
        segs = self._first_clear(
            p, (_straight(lead) + tp.lane_change(0.0, side1 * lane - p.y, self.r)
                for lead in range(-8, 30)), course, "lane change round obstacle 1")
        if segs is None or not self._line(segs):
            return None
        p = self._pose()
        g = self._us()
        x2 = None if g is None else self._front(p) + g
        gap12 = None if x2 is None else x2 - (x1 + tp.OBS1_SIZE_CM)
        if gap12 is not None and GAP12_RANGE[0] <= gap12 <= GAP12_RANGE[1]:
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
        p = self._pose()
        far = x1 + tp.OBS1_SIZE_CM + GAP12_RANGE[1] + tp.OBS2_DEPTH_CM
        run = far - (p.x + tp.IR_FROM_AXLE_CM)
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
                if len(hits) >= 3:
                    self.stm.abort()                  # RST: no reply comes
                    self.sleep_fn(0.4)
                    return True
            return False

        reply = tp.run_polling(self.stm, f"F{int(run)}", poll)
        if reply is None or not (reply.startswith("OK") or reply == "STOPPED"):
            logging.error("STM answered %r while looking for obstacle 2.", reply)
            return None
        if len(hits) < 3:
            logging.error("Drove %.0f cm on the narrow lane and never saw "
                          "obstacle 2 on the %s IR.", run, inner)
            return None
        x2 = hits[0][0].x + tp.IR_FROM_AXLE_CM - tp.IR_LAG_CM
        logging.info("Obstacle 2 side seen %.0f cm off the %s IR: face at "
                     "x=%.1f.", min(g for _, g in hits), inner, x2)
        return x2

    def photo_obstacle2(self, course: Course, side1: float) -> None:
        """
        Pull up for the photo of obstacle 2: turned PHOTO_TURN_DEG toward the
        centreline, the camera PHOTO_DIST_CM from the arrow (the face
        centre). A lane change sets the line it looks along, a straight (on,
        or back if the robot is already past) sets the distance; the lane
        that aims it best wins. Then the pause. Falls back to pausing where
        it is.
        """
        p = self._pose()
        toward = tp._turn_toward(0.0, -side1)
        turn = tp.arc(Pose(0.0, 0.0, 0.0), 1 if toward == "L" else -1,
                      PHOTO_TURN_DEG, self.r)
        cam = (CAMERA_FROM_AXLE_CM * math.cos(turn.th),
               CAMERA_FROM_AXLE_CM * math.sin(turn.th))
        ranked = []
        for lane in range(int(abs(p.y)), 61):
            shift = tp.lane_change(0.0, side1 * lane - p.y, self.r)
            base = tp.run(p, shift, self.r)[-1]
            # Camera after a straight of L then the turn, as L varies: only x
            # moves. Solve |camera - arrow| = PHOTO_DIST_CM for L.
            cy = base.y + turn.y + cam[1]
            if abs(cy) >= PHOTO_DIST_CM:
                continue
            lead = (course.x2 - (base.x + turn.x + cam[0])
                    - math.sqrt(PHOTO_DIST_CM ** 2 - cy ** 2))
            segs = shift + _straight(lead) + [Seg(toward, PHOTO_TURN_DEG)]
            end = tp.run(p, segs, self.r)[-1]
            cx = end.x + CAMERA_FROM_AXLE_CM * math.cos(end.th)
            cy = end.y + CAMERA_FROM_AXLE_CM * math.sin(end.th)
            d = math.hypot(course.x2 - cx, cy)
            aim = abs(math.degrees(math.atan2(-cy, course.x2 - cx) - end.th))
            ranked.append((aim, segs, d))
        ranked.sort(key=lambda t: t[0])
        segs = self._first_clear(p, [c for _, c, _ in ranked], course,
                                 "move to the obstacle 2 photo pose")
        if segs is None:
            logging.warning("No clear photo pose - pausing here instead.")
        else:
            off, _, d = next(t for t in ranked if t[1] is segs)
            if not self._line(segs):
                raise RuntimeError("Move to the photo pose failed.")
            logging.info("Photo pose: turned %d deg toward obstacle 2, camera "
                         "%.0f cm from the arrow, aimed %.1f deg off it.",
                         PHOTO_TURN_DEG, d, off)
        logging.info("Pausing %.0f s (photo of obstacle 2).", OBS2_PAUSE_S)
        self.sleep_fn(OBS2_PAUSE_S)

    def u_turn(self, course: Course, side1: float) -> bool:
        """
        From the photo pose, the rest of the turn toward the centreline, a
        straight across the front of obstacle 2 that sets the lane, and the
        last 90 deg - heading home on HOME_LANE_CM on the other side of
        obstacle 1. Backs off first if that is what keeps it clear.
        """
        p = self._pose()
        toward = tp._turn_toward(0.0, -side1)
        first = int(round(90 - abs(math.degrees(p.th))))

        def candidates():
            for back in range(0, 44, 4):
                head = _straight(-back) + ([Seg(toward, first)] if first >= 1 else [])
                y_e0 = tp.run(p, head + [Seg(toward, 90)], self.r)[-1].y
                across = max(0.0, side1 * y_e0 + HOME_LANE_CM)
                yield head + _straight(across) + [Seg(toward, 90)]

        segs = self._first_clear(p, candidates(), course, "U-turn at obstacle 2")
        if segs is None:
            return False
        logging.info("U-turn at obstacle 2: %s", ",".join(s.token() for s in segs))
        return self._line(segs)

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
            self.photo_obstacle2(course, side1)
            if not self.u_turn(course, side1):
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
        logging.info("TASK 2 DONE in %.1f s (from Enter, including the %.0f s "
                     "pause and your typing).", time.monotonic() - t0, OBS2_PAUSE_S)
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
        name = os.getenv("T2_CAL_PROFILE", "").strip() or None
        if not cal_profile.prepare_for_task(self.stm, name=name,
                                            allowed_arcs={cal_profile.ARC_SAFE}):
            logging.error("STM startup failed (see above) - not running Task 2.")
            return False
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
