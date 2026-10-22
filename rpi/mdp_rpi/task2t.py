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

  1. Ultrasound obstacle 1, F + FU to a standoff, ask for its arrow.
  2. Turn 45 deg to that side, a short diagonal, straighten: on the lane just
     clear of obstacle 1, the ultrasound measures obstacle 2. If it misses (a
     narrow obstacle 2 is off the beam), widen the lane and find obstacle 2
     with the inner IR going by.
  3. Photo pose: 45 deg out, a diagonal, swing back in, so the camera looks
     at obstacle 2's arrow from 45 deg, PHOTO_DIST_CM away. Ask its arrow.
  4. Turn to run along obstacle 2's face with a side IR on it - forward if
     the arrow points away from the robot's side, in reverse if toward it -
     and stop where the face ends. That end is obstacle 2's half-width.
  5. Round that end, behind obstacle 2, round the other end: heading home on
     the far side. task2t_park.ParkRunner does the rest - IR fix on obstacle
     1, S onto the centreline, FU off the back fence.

Speed: the firmware's fast mode (!FAST1) ramps straights up to 200 rpm and
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
FACE_GAP_CM = tp._env_f("T2T_FACE_GAP_CM", 18.0)
# The most an IR reading can say and still be believed as a face. With
# nothing in range the IR does NOT read blank: its background output maps to
# ~30-50 cm (the right unit read 29-48 for 90 cm of empty floor). Anything
# past this is treated as nothing there.
IR_FACE_MAX_CM = tp._env_f("T2T_IR_FACE_MAX_CM", 26.0)
# Behind obstacle 2, the IR on its back face at most this far off it.
BACK_GAP_MAX_CM = tp._env_f("T2T_BACK_GAP_MAX_CM", 34.0)
# Coming round behind obstacle 2, the IR at most this far past its near end -
# a 30 cm obstacle leaves 8 cm of back face to see.
BACK_START_MAX_CM = tp._env_f("T2T_BACK_START_MAX_CM", 22.0)
# Obstacle 2's measured centre is taken as odometry drift, up to this much.
Y_RESYNC_MAX_CM = tp._env_f("T2T_Y_RESYNC_MAX_CM", 15.0)
# Heading home past obstacle 2, the body this far off its side.
HOME_GAP_CM = tp._env_f("T2T_HOME_GAP_CM", 15.0)
# Added to the measured half-width before planning round it.
END_MARGIN_CM = tp._env_f("T2T_END_MARGIN_CM", 2.0)
FAST = os.getenv("T2T_FAST", "1").strip() != "0"
# Brief: obstacle 2 is 30-90 wide, 60-150 past obstacle 1. Measurements
# outside these (with slack) mean the sensor saw something else.
W2_RANGE = (30.0, 90.0)
GAP12_RANGE = (45.0, 175.0)
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
                 startup: bool = True, wait_for_go: bool = True):
        self.stm = stm
        self.input_fn = input_fn
        self.sleep_fn = sleep_fn
        self.startup = startup
        self.wait_for_go = wait_for_go
        self.frame = tp.WorldFrame(stm)
        self.th = 0.0                 # commanded heading, radians
        self.r = tp.RADIUS_CM
        self.fast = False             # what the firmware is set to

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
        self.sleep_fn(OBS_PAUSE_S)
        return x1, _ask_arrow(self.input_fn, "Obstacle 1")

    def pass_obstacle1(self, x1: float, side1: float) -> Optional[float]:
        """45 deg onto the lane on side1, ultrasound obstacle 2. Returns its
        face x, NARROW if the ultrasound missed it, or None."""
        p = self._pose()
        lane = tp.OBS1_SIZE_CM / 2 + tp.HALF_WIDTH_CM + OBS1_PASS_GAP_CM
        course = Course(x1, x1 + tp.OBS1_SIZE_CM + 60.0, CONSERVATIVE_W2,
                        wall_y=CONSERVATIVE_WALL_Y)
        # FU lands a cm or two off; a lead of -8..30 absorbs it.
        segs = self._first_clear(
            p, (_straight(lead) + kick(side1 * lane - p.y, self.r)
                for lead in range(-8, 30)), course, "45 deg turn past obstacle 1")
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
        ranked = list(self.photo_candidates(p, course, side1))
        segs = self._first_clear(p, [c for c, _, _ in ranked], course,
                                 "move to the obstacle 2 photo pose")
        if segs is None:
            raise RuntimeError("No clear photo pose for obstacle 2.")
        _, d, swing = next(t for t in ranked if t[0] is segs)
        if not self._line(segs):
            raise RuntimeError("Move to the photo pose failed.")
        logging.info("Photo pose: camera %.0f cm from the arrow, looking in "
                     "from %d deg.", d, swing - KICK_DEG)
        self.sleep_fn(OBS_PAUSE_S)
        return _ask_arrow(self.input_fn, "Obstacle 2")

    # ── obstacle 2 ────────────────────────────────────────────────────────────

    def face_candidates(self, p: Pose, course: Course, side1: float,
                        reverse: bool) -> Iterator[List[Seg]]:
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
        for s in range(-30, 32, 2):
            head = _straight(s) + ([Seg(inward, turn)] if turn >= 1 else [])
            end = tp.run(p, head, self.r)[-1]
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

    def follow_face(self, token: str, x_face: float, facing: float,
                    toward: float, window: float) -> Optional[Tuple[float, float]]:
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
        p = self._pose()
        side = self._ir_facing(p.th, facing)
        ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)     # IR y - axle y
        forward = not token.upper().startswith("R")
        lead_off = tp.FRONT_FROM_AXLE_CM if forward else tp.REAR_FROM_AXLE_CM
        seen, lost, raw = [], [], []
        ref = [p.y + ahead]                              # face known to here
        guard = []
        lo, hi = tp.IR_MIN_GAP_CM - 4.0, min(IR_FACE_MAX_CM, tp.IR_MAX_GAP_CM[side])

        def stop() -> bool:
            self.stm.abort()                             # RST: no reply comes
            self.sleep_fn(0.4)
            return True

        def poll() -> bool:
            g = self._ir(side)
            q = self.frame.pose()
            if q is None:
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
                    return stop()
            if toward * (q.y + toward * lead_off - ref[0]) > tp.WALL_CLEARANCE_CM - tp.CLEARANCE_CM:
                guard.append(q.y)
                return stop()
            return False

        logging.info("Along obstacle 2's %s face, %s IR on it: %s",
                     "front" if facing > 0 else "back", side, token)
        reply = tp.run_polling(self.stm, token, poll)
        if reply is None or not (reply.startswith("OK") or reply == "STOPPED"):
            logging.error("STM answered %r along obstacle 2.", reply)
            return None
        failed = None
        if guard:
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
        # A reading describes the beam IR_LAG_CM back along the way it went.
        end_y = (seen[-1][0] + lost[0]) / 2.0 - toward * tp.IR_LAG_CM
        xs = sorted(x for _, x, _ in seen)
        return end_y, xs[len(xs) // 2]

    @staticmethod
    def _ir_facing(th: float, facing: float) -> str:
        """The side IR looking along facing * x, at heading +-90."""
        return "right" if math.sin(th) * facing > 0 else "left"

    def round_near_end(self, p: Pose, course: Course, toward: float,
                       reverse: bool, near_y: float) -> Optional[List[Seg]]:
        """
        Round obstacle 2's `toward` end and on to behind it: two 90 deg turns
        the same way (clockwise for toward = +y), ending heading -toward*90
        with the IR on the back face within BACK_GAP_MAX_CM of it, ready to run along
        it. The shortest clear one wins.

        From a reverse run the robot faces away from that end and has to get
        from facing sideways to facing +x. One 90 deg forward arc does that
        but moves it a whole radius toward the centreline, so it must first
        back out a radius past the lane it wants - into a wall standing at
        the brief's minimum 50 cm. Backing out on a reverse arc and finishing
        with a forward one gets there with ~19 cm less room, so those are
        tried as well: reverse theta, then forward 90 - theta.

        near_y: where that end is. The IR, ahead of the axle, must start the
        back-face run no more than BACK_START_MAX_CM past it, or a 30 cm
        obstacle has gone by before the IR has seen enough of it.
        """
        same = "R" if toward > 0 else "L"
        boxes = [b for b in course.boxes() if "fence" not in b.name]
        back = course.x2 + tp.OBS2_DEPTH_CM
        lo = back + tp.IR_SIDE_OFFSET_CM + tp.IR_MIN_GAP_CM + 4.0
        hi = back + tp.IR_SIDE_OFFSET_CM + min(BACK_GAP_MAX_CM, IR_FACE_MAX_CM - 4)

        def firsts():
            # a = how far on along the face first, smallest moves first. A
            # negative a goes back: the RST overshoots the end, and from a
            # reverse run that overshoot is toward the wall.
            for a in sorted(range(-20, 46, 2), key=abs):
                if not reverse:
                    yield abs(a), _straight(a) + [Seg(same, 90)]
                    continue
                for th1 in (45, 30, 60, 0):     # on, then turn
                    yield abs(a), (_straight(-a)
                                   + ([Seg(same.lower(), th1)] if th1 else [])
                                   + [Seg(_other(same), 90 - th1)])

        for need in (tp.CLEARANCE_CM - 0.01, tp.CLEARANCE_FLOOR_CM):
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
                    logging.warning("Round obstacle 2's end keeps only the "
                                    "%.1f cm floor.", need)
                return best
        return None

    def turn_home(self, p: Pose, course: Course, toward: float
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
        for need in (tp.CLEARANCE_CM - 0.01, tp.CLEARANCE_FLOOR_CM):
            for cc in options:
                segs = _straight(cc) + [Seg(same, 90)]
                if _path_clear(p, segs, self.r, boxes, need) is not None:
                    return segs
        return None

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
        reverse = side2 == side1
        p = self._pose()
        segs = self._first_clear(p, self.face_candidates(p, course, side1, reverse),
                                 course, "turn onto obstacle 2's face")
        if segs is None or not self._line(segs):
            return None

        # The ends have to be found accurately, not quickly.
        self._set_fast(False)
        try:
            p = self._pose()
            limit = side2 * (W2_RANGE[1] / 2.0 + 15.0)       # IR y past any end
            ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)
            run = max(10.0, side2 * (limit - (p.y + ahead)))
            got = self.follow_face(f"{'R' if reverse else 'F'}{int(round(run))}",
                                   course.x2, +1.0, side2, 18.0)
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
            logging.info("Obstacle 2's %s end at y=%.1f; face at x=%.1f (IR; "
                         "ultrasound said %.1f).", "left" if side2 > 0 else "right",
                         near, x2, course.x2)

            # Until the far end is seen: as wide as the brief allows that way.
            near_y = near + side2 * END_MARGIN_CM
            wide = W2_RANGE[1] + 2 * END_MARGIN_CM
            # The margin moves the end out and the wall IN: both errs safe.
            partial = Course(course.x1, x2, wide, obs2_y=near_y - side2 * wide / 2.0,
                             wall_y=wide / 2.0 + tp.WALL_CLEARANCE_CM - 2 * END_MARGIN_CM)
            p = self._pose()
            segs = self.round_near_end(p, partial, side2, reverse, near)
            if segs is None:
                logging.error("No clear way round obstacle 2's end from %s.", p)
                return None
            if not self._line(segs):
                return None

            p = self._pose()
            ahead = tp.IR_FROM_AXLE_CM * math.sin(p.th)
            limit = near - side2 * (W2_RANGE[1] + 15.0)
            run = max(10.0, -side2 * (limit - (p.y + ahead)))
            got = self.follow_face(f"F{int(round(run))}",
                                   x2 + tp.OBS2_DEPTH_CM, -1.0, -side2, 12.0)
            if got is None:
                return None
            far, _ = got
        finally:
            self._set_fast(True)

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
        segs = self.turn_home(p, measured, side2)
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
        logging.info("TASK 2 DONE in %.1f s (from Enter, including your typing "
                     "of both arrows).", time.monotonic() - t0)
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
