"""
task2t_park.py - Task 2 (fastest car) on TIGHT: the drive home and the park.

TIGHT copy of task2_park.py. Identical except the radius (T2T_RADIUS_CM,
default TIGHT's 29.1) and the arc speed (TIGHT arcs at 150 rpm). Change
the planning in both files, or they drift apart.

The run is disqualified if the robot touches the carpark fence and penalised
for every obstacle hit, so the return is built around one rule: the last
manoeuvre into the bay is planned from a position MEASURED near home, never
from dead reckoning carried around the whole course.

    1. Line A  - from the far side of obstacle 2, merge to a pass lane beside
                 obstacle 1 and drive past it. The side IR facing obstacle 1
                 is polled the whole way. Obstacle 1 sits on the bay
                 centreline, so its reading is the robot's true lateral
                 offset from the bay, whatever the obstacle 2 loop drifted.
    2. Line B  - an S-curve sized from that measured offset puts the robot on
                 the centreline, square, before the mouth; a collision-checked
                 odometry F stops it off the back fence.

Everything here except ParkRunner is pure geometry and runs on a laptop:

    python task2t_park.py 60 60 90 right          # print the plan, worst case
    python task2t_park.py 100 80 40 left --plot   # and draw it (task2t_park.png)

WORLD FRAME (cm). Origin at the bay mouth on its centreline. +x points out of
the bay toward the obstacles, +y is to the left looking out from the bay,
heading is counter-clockwise from +x. The robot's pose is its rear-axle
midpoint: the point the odometry integrates and the arcs turn about.
"""

import argparse
import json
import logging
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

from dotenv import load_dotenv

load_dotenv()


def _env_f(name: str, default: float) -> float:
    return float(os.getenv(name, str(default)))


# ── Robot ─────────────────────────────────────────────────────────────────────
# TIGHT's floor-chord radius. Its own variable, so the SAFE task's
# T2_RADIUS_CM can stay set for SAFE; falls back to T2_RADIUS_CM, then to the
# 29.1 the planner and A5 use for TIGHT. Set T2T_RADIUS_CM to what
# `calibrate.py radius` measured on the TIGHT profile.
RADIUS_CM = _env_f("T2T_RADIUS_CM", _env_f("T2_RADIUS_CM", 30.0))
ROBOT_LENGTH_CM = _env_f("T2_ROBOT_LENGTH_CM", 23.0)
ROBOT_WIDTH_CM = _env_f("T2_ROBOT_WIDTH_CM", 18.8)
# Front ultrasonic, and the front bumper with it, ahead of the rear axle.
FRONT_FROM_AXLE_CM = _env_f("T2_FRONT_FROM_AXLE_CM", 20.5)
REAR_FROM_AXLE_CM = ROBOT_LENGTH_CM - FRONT_FROM_AXLE_CM
# Side IRs: how far ahead of the rear axle, and how far out from the
# centreline their faces sit (what a reading is measured from). Measured
# ~10.5 cm face to face on the robot, so 5.25 each side - inside the body:
# a reading is ~4 cm more than the gap from the body's side.
IR_FROM_AXLE_CM = _env_f("T2_IR_FROM_AXLE_CM", 17.0)
IR_SIDE_OFFSET_CM = _env_f("T2_IR_SIDE_OFFSET_CM", 5.25)
HALF_WIDTH_CM = ROBOT_WIDTH_CM / 2.0
# Usable IR span (ir.h): saturates under 10; the right unit quits past ~65.
# The pass lane is chosen so the expected gap sits well inside these.
IR_MIN_GAP_CM = _env_f("T2_IR_MIN_GAP_CM", 14.0)
IR_MAX_GAP_CM = {"left": _env_f("T2_IR_MAX_GAP_LEFT_CM", 65.0),
                 "right": _env_f("T2_IR_MAX_GAP_RIGHT_CM", 50.0)}
US_BIAS_CM = 1.3   # communications/stm.py: the HC-SR04 reads this much long
# How far the robot travels between the IR seeing something and the Pi
# getting the number (ADC median filter + ?IR round trip), at cruise. Only
# the longitudinal fix uses it. NOT MEASURED: ~50 ms at 34.5 cm/s.
IR_LAG_CM = _env_f("T2_IR_LAG_CM", 2.0)

# ── Course (task brief) ───────────────────────────────────────────────────────
# Three 60 x 10 cuboids: inside 60 wide x 50 deep, mouth open toward +x.
# The brief says 60 wide x 50 deep inside. Measure the real one: a practice
# bay read ~70 wide off the side IRs. Too wide here plans too close to the
# fence; too narrow and centring can reject good IR readings as not-the-fence.
BAY_DEPTH_CM = _env_f("T2_BAY_DEPTH_CM", 50.0)
BAY_HALF_WIDTH_CM = _env_f("T2_BAY_WIDTH_CM", 60.0) / 2.0
FENCE_T_CM = 10.0
OBS1_SIZE_CM = 10.0
OBS2_DEPTH_CM = 10.0
# The usable course ends exactly 50 cm beyond either side of obstacle 2.
# This is a hard route boundary even when no physical wall returns an IR echo.
WALL_CLEARANCE_CM = 50.0
ASSUME_WALLS = os.getenv("T2_ASSUME_WALLS", "1") != "0"

# ── Margins ───────────────────────────────────────────────────────────────────
# Body-to-anything gap the plan must keep, on top of the footprint itself.
# Execution error near home is small (the IR fix happens < 1 m out), so this
# is mostly radius and braking slop on the final S.
CLEARANCE_CM = _env_f("T2_CLEARANCE_CM", 6.0)
# Re-plans from a measured pose fall back to this rather than stop dead: the
# robot is already where it is, and a stop loses the run.
CLEARANCE_FLOOR_CM = _env_f("T2_CLEARANCE_FLOOR_CM", 3.5)
# WPOSE, envelope sampling, and the physical path can differ by a few
# millimetres. Keep CLEARANCE_CM as the planning target, but do not reject an
# otherwise valid measured continuation for a sub-centimetre miss. The hard
# collision floor below is never reduced by this tolerance.
EXECUTION_CLEARANCE_TOL_CM = max(
    0.0, _env_f("T2_EXECUTION_CLEARANCE_TOL_CM", 0.6))


def execution_clearance(target: float) -> float:
    """Minimum accepted measured clearance for a nominal planning target."""
    return max(CLEARANCE_FLOOR_CM,
               float(target) - EXECUTION_CLEARANCE_TOL_CM)
# The candidate with the biggest clearance wins, up to this; past it time wins.
CLEARANCE_ENOUGH_CM = _env_f("T2_CLEARANCE_ENOUGH_CM", 12.0)
# Front bumper to back fence when parked. With this chassis, the rear bumper
# is then 12 cm inside the mouth of the 50 cm-deep bay.
PARK_GAP_CM = _env_f("T2_PARK_GAP_CM", 15.0)
PARK_GAP_TOL_CM = _env_f("T2_PARK_GAP_TOL_CM", 2.5)
PARK_MIN_BACK_GAP_CM = max(
    CLEARANCE_FLOOR_CM, _env_f("T2_PARK_MIN_BACK_GAP_CM", 8.0))
PARK_STEP_CM = int(_env_f("T2_PARK_STEP_CM", 8.0))
PARK_CRUISE_RESERVE_CM = max(PARK_STEP_CM, _env_f("T2_PARK_CRUISE_RESERVE_CM", 10.0))
PARK_ECHO_TOL_CM = 8.0
PARK_MAX_STEPS = int(_env_f("T2_PARK_MAX_STEPS", 8.0))
# From just outside the mouth the front sensor can legitimately see the back
# wall at the 50 cm bay depth plus most of the axle-to-sensor offset.  The old
# 60 cm default rejected the measured 63.7 cm RR echo by 0.7 cm even though the
# complete straight sweep was clear.
PARK_MAX_US_CM = _env_f("T2_PARK_MAX_US_CM", BAY_DEPTH_CM + 15.0)
PARK_HEADING_TOL_DEG = _env_f("T2_PARK_HEADING_TOL_DEG", 12.0)
PARK_SIDE_STOP_CM = _env_f("T2_PARK_SIDE_STOP_CM", 10.0)
RETURN_CHECKPOINT_CM = max(20, int(_env_f("T2_RETURN_CHECKPOINT_CM", 30.0)))
RETURN_REPLAN_ERROR_CM = _env_f("T2_RETURN_REPLAN_ERROR_CM", 3.0)
RETURN_REPLAN_LIMIT = max(1, int(_env_f("T2_RETURN_REPLAN_LIMIT", 2.0)))
# Includes the observed post-RST coast; tune only from stopping measurements.
RETURN_STOP_RESERVE_CM = max(20.0, _env_f("T2_RETURN_STOP_RESERVE_CM", 20.0))
# Leave this much for the last obstacle-1 approach F after re-reading ?US.
APPROACH_LEAD_CM = _env_f("T2_APPROACH_LEAD_CM", 10.0)
# Straight run the IR gets before it reaches obstacle 1, so the S that put the
# robot in the lane has finished settling before the readings start.
IR_LEAD_CM = _env_f("T2_IR_LEAD_CM", 8.0)
# Line A stops this far after the IR has cleared obstacle 1's near face.
IR_TAIL_CM = _env_f("T2_IR_TAIL_CM", 4.0)
MIN_ARC_DEG = 5        # a smaller arc does not move the chassis (task_a5.py)
# Centring in the bay. The S stops with both side IRs this far inside the
# mouth, where each sees a side fence; half the difference of the two readings
# is the robot's offset from the bay centreline. Off by more than the deadband,
# one small lane change squares it up before the final parking F.
CENTRE_CHECK_IN_CM = _env_f("T2_CENTRE_CHECK_IN_CM", 5.0)
CENTRE_DEADBAND_CM = _env_f("T2_CENTRE_DEADBAND_CM", 1.0)
PARK_FINAL_DIRECT_CM = 20.0
# Readings that do not add up to the bay's width less the robot's are not the
# two fences (an IR not inside yet, or reading something else): no correction.
CENTRE_SUM_TOL_CM = _env_f("T2_CENTRE_SUM_TOL_CM", 10.0)
CENTRE_MIN_CLEAR_CM = _env_f("T2_CENTRE_MIN_CLEAR_CM", 5.0)
# Parking only has to finish wholly inside the bay. Do not introduce a
# reverse-plus-S correction merely to make the two side gaps equal when the
# measured straight sweep already keeps the hard collision floor.
PARK_STRAIGHT_CLEARANCE_CM = max(CLEARANCE_FLOOR_CM, CENTRE_MIN_CLEAR_CM)
# Left IR minus right IR with the robot centred and square in the bay by hand.
# The two units are fitted separately (ir.h) and need not agree; whatever
# this is, the robot parks half of it off centre unless it is set here.
IR_LR_BIAS_CM = _env_f("T2_IR_LR_BIAS_CM", 0.0)

# ── Rough timing, only for ranking candidates ────────────────────────────────
# Estimates only: motor RPM ratios do not establish speed on a turning arc.
# Override with timed floor measurements, including the mode used in the run.
CRUISE_CM_S = _env_f("T2_EST_CRUISE_CM_S", 34.5)
ARC_CM_S = _env_f("T2_EST_ARC_CM_S", 30.9)
LINE_OVERHEAD_S = _env_f("T2_EST_LINE_OVERHEAD_S", 0.1)
PRIMITIVE_OVERHEAD_S = _env_f("T2_EST_PRIMITIVE_OVERHEAD_S", 0.5)
# Provisional estimates, replace with current floor measurements. The default
# cruise estimate deliberately does not assume a measured fast-mode gain.
FAST_CRUISE_CM_S = _env_f("T2_EST_FAST_CRUISE_CM_S", CRUISE_CM_S)
APPROACH_CM_S = _env_f("T2_EST_APPROACH_CM_S", 13.8)
NORMAL_OVERHEAD_S = _env_f("T2_EST_NORMAL_OVERHEAD_S", PRIMITIVE_OVERHEAD_S + 0.2)
TIMING_PROFILE = {}
if os.getenv("T2_TIMING_PROFILE"):
    with open(os.environ["T2_TIMING_PROFILE"], encoding="utf-8") as source:
        TIMING_PROFILE = json.load(source)
    for mode in ("fast", "normal"):
        for token, seconds in TIMING_PROFILE.get(mode, {}).items():
            if not isinstance(token, str) or not math.isfinite(float(seconds)) or float(seconds) <= 0:
                raise ValueError("Invalid measured primitive timing")
if not (0 < FRONT_FROM_AXLE_CM < ROBOT_LENGTH_CM and ROBOT_WIDTH_CM > 0
        and RADIUS_CM > 0 and CRUISE_CM_S > 0 and ARC_CM_S > 0
        and FAST_CRUISE_CM_S > 0 and APPROACH_CM_S > 0
        and PRIMITIVE_OVERHEAD_S >= 0 and NORMAL_OVERHEAD_S >= 0):
    raise ValueError("Invalid Task 2 body geometry or timing estimates")

HEADING_HOME = math.pi


def parking_heading_error(heading: float) -> float:
    """Wrapped heading error shared by entry, monitoring and completion."""
    delta = heading - HEADING_HOME
    return abs(math.degrees(math.atan2(math.sin(delta), math.cos(delta))))


def stable_us(read, attempts: int = 4, tolerance: float = 2.0) -> Optional[float]:
    """Require consecutive compatible echoes; a dropout breaks the pair."""
    previous = None
    for _ in range(attempts):
        try:
            gap = read()
        except StopIteration:  # finite test doubles
            gap = None
        if gap is None or not math.isfinite(gap) or gap <= 0:
            previous = None
            continue
        if previous is not None and abs(gap - previous) <= tolerance:
            return (gap + previous) / 2.0
        previous = gap
    return None


# ═════════════════════════════════════════════════════════════════════════════
# Geometry
# ═════════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    th: float          # radians, CCW from +x

    def __str__(self) -> str:
        return f"({self.x:.1f}, {self.y:.1f}, {math.degrees(self.th):.1f}°)"


@dataclass(frozen=True)
class Seg:
    """One primitive: 'F' / 'B' cm forward / reverse, 'L' / 'R' degrees of
    forward arc, 'l' / 'r' degrees of REVERSE arc steered left / right.
    Reversing flips the body rotation: 'l' turns the robot clockwise, 'r'
    counter-clockwise (Motion_DriveArc's sign table)."""
    kind: str
    value: int

    def token(self) -> str:
        return {"F": "F", "B": "R", "L": "FL", "R": "FR",
                "l": "RL", "r": "RR"}[self.kind] + str(self.value)

    @property
    def turn_deg(self) -> int:
        """Body rotation, degrees, counter-clockwise positive."""
        return {"L": 1, "R": -1, "l": -1, "r": 1}.get(self.kind, 0) * self.value


@dataclass(frozen=True)
class Box:
    x0: float
    x1: float
    y0: float
    y1: float
    name: str


def straight(p: Pose, d: float) -> Pose:
    return Pose(p.x + d * math.cos(p.th), p.y + d * math.sin(p.th), p.th)


def arc(p: Pose, turn: int, deg: float, r: float) -> Pose:
    """turn +1 left (CCW), -1 right. The rear axle runs on a circle of r."""
    cx = p.x - turn * r * math.sin(p.th)
    cy = p.y + turn * r * math.cos(p.th)
    th = p.th + turn * math.radians(deg)
    return Pose(cx + turn * r * math.sin(th), cy - turn * r * math.cos(th), th)


def run(p: Pose, segs: Sequence[Seg], r: float = RADIUS_CM,
        step: float = 1.0) -> List[Pose]:
    """Every pose along segs, about `step` cm apart, start and end included."""
    out = [p]
    for s in segs:
        if s.kind in ("F", "B"):
            d = s.value if s.kind == "F" else -s.value
            n = max(1, int(math.ceil(s.value / step)))
            for i in range(1, n + 1):
                out.append(straight(p, d * i / n))
        else:
            # Steered left or right; a reverse arc runs the same circle
            # backwards, so the heading changes the other way.
            turn = 1 if s.kind in ("L", "l") else -1
            sign = -1 if s.kind in ("l", "r") else 1
            n = max(1, int(math.ceil(math.radians(s.value) * r / step)))
            for i in range(1, n + 1):
                out.append(arc(p, turn, sign * s.value * i / n, r))
        p = out[-1]
    return out


def footprint(p: Pose) -> List[Tuple[float, float]]:
    c, s = math.cos(p.th), math.sin(p.th)
    pts = []
    for lx, ly in ((FRONT_FROM_AXLE_CM, HALF_WIDTH_CM),
                   (FRONT_FROM_AXLE_CM, -HALF_WIDTH_CM),
                   (-REAR_FROM_AXLE_CM, -HALF_WIDTH_CM),
                   (-REAR_FROM_AXLE_CM, HALF_WIDTH_CM)):
        pts.append((p.x + lx * c - ly * s, p.y + lx * s + ly * c))
    return pts


def _seg_dist(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _overlaps(poly, b: Box) -> bool:
    """Separating axis test, rectangle vs axis-aligned box."""
    xs = [q[0] for q in poly]
    ys = [q[1] for q in poly]
    if max(xs) < b.x0 or min(xs) > b.x1 or max(ys) < b.y0 or min(ys) > b.y1:
        return False
    corners = ((b.x0, b.y0), (b.x1, b.y0), (b.x1, b.y1), (b.x0, b.y1))
    for i in range(2):
        ax = poly[i + 1][0] - poly[i][0]
        ay = poly[i + 1][1] - poly[i][1]
        pa = [q[0] * ax + q[1] * ay for q in poly]
        pb = [q[0] * ax + q[1] * ay for q in corners]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return False
    return True


def clearance(p: Pose, b: Box) -> float:
    """Gap from the robot's footprint to a box, cm. Negative means touching."""
    # Cheap reject: the footprint fits in a circle of ~22 cm about the axle.
    reach = math.hypot(max(FRONT_FROM_AXLE_CM, REAR_FROM_AXLE_CM), HALF_WIDTH_CM)
    far = max(b.x0 - p.x, p.x - b.x1, 0.0) ** 2 + max(b.y0 - p.y, p.y - b.y1, 0.0) ** 2
    if far > (reach + CLEARANCE_ENOUGH_CM + 5) ** 2:
        return math.sqrt(far) - reach
    poly = footprint(p)
    if _overlaps(poly, b):
        return -1.0
    # For disjoint rectangles the minimum is a vertex-to-edge distance.
    # Eight point-to-rectangle distances replace 32 segment projections.
    best2 = float("inf")
    for x, y in poly:
        dx = max(b.x0 - x, x - b.x1, 0.0)
        dy = max(b.y0 - y, y - b.y1, 0.0)
        best2 = min(best2, dx * dx + dy * dy)
    c, sn = math.cos(p.th), math.sin(p.th)
    for x, y in ((b.x0, b.y0), (b.x1, b.y0),
                 (b.x1, b.y1), (b.x0, b.y1)):
        dx, dy = x - p.x, y - p.y
        longitudinal = c * dx + sn * dy
        lateral = -sn * dx + c * dy
        outside_x = max(-REAR_FROM_AXLE_CM - longitudinal,
                        longitudinal - FRONT_FROM_AXLE_CM, 0.0)
        outside_y = max(abs(lateral) - HALF_WIDTH_CM, 0.0)
        best2 = min(best2, outside_x * outside_x + outside_y * outside_y)
    return math.sqrt(best2)


def path_clearance(poses: Sequence[Pose], boxes: Sequence[Box]) -> Tuple[float, str]:
    best, name = float("inf"), ""
    for p in poses:
        for b in boxes:
            c = clearance(p, b)
            if c < best:
                best, name = c, b.name
    return best, name


# ═════════════════════════════════════════════════════════════════════════════
# Course
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class Course:
    """What the outbound leg measured. x values are near (bay-side) faces."""
    x1: float              # obstacle 1 near face
    x2: float              # obstacle 2 near face
    w2: float              # obstacle 2 width
    obs1_y: float = 0.0    # obstacle 1 centre; on the bay centreline by brief
    walls: bool = ASSUME_WALLS
    # Hard side boundary |y|. None: w2/2 + exactly 50. Before the
    # width is measured, pass the narrowest case's 65 with w2 = 90 - the
    # conservative box for both at once.
    wall_y: Optional[float] = None
    # Obstacle 2's centre y. The brief puts it on the centreline, but the
    # robot measures both its ends in odometry y - which has drifted by then -
    # so it is wherever those ends say. The walls are placed about it too.
    obs2_y: float = 0.0

    def obs1(self) -> Box:
        h = OBS1_SIZE_CM / 2.0
        return Box(self.x1, self.x1 + OBS1_SIZE_CM,
                   self.obs1_y - h, self.obs1_y + h, "obstacle 1")

    def obs2(self) -> Box:
        h = self.w2 / 2.0
        return Box(self.x2, self.x2 + OBS2_DEPTH_CM,
                   self.obs2_y - h, self.obs2_y + h, "obstacle 2")

    def fence(self) -> List[Box]:
        back = -BAY_DEPTH_CM
        w = BAY_HALF_WIDTH_CM
        return [Box(back - FENCE_T_CM, back, -w - FENCE_T_CM, w + FENCE_T_CM, "back fence"),
                Box(back - FENCE_T_CM, 0.0, w, w + FENCE_T_CM, "left fence"),
                Box(back - FENCE_T_CM, 0.0, -w - FENCE_T_CM, -w, "right fence")]

    def wall_boxes(self) -> List[Box]:
        if not self.walls:
            return []
        y = self.wall_y if self.wall_y is not None else self.w2 / 2.0 + WALL_CLEARANCE_CM
        c = self.obs2_y
        x0, x1 = 0.0, self.x2 + OBS2_DEPTH_CM + 150.0
        return [Box(x0, x1, c + y, c + y + 10.0, "left wall"),
                Box(x0, x1, c - y - 10.0, c - y, "right wall")]

    def boxes(self) -> List[Box]:
        return [self.obs1(), self.obs2()] + self.fence() + self.wall_boxes()


# ═════════════════════════════════════════════════════════════════════════════
# Planning
# ═════════════════════════════════════════════════════════════════════════════

class NoPlan(RuntimeError):
    pass


def _turn_toward(th: float, dy: float) -> str:
    """'L' if a move of dy along world y is to the robot's left at heading th."""
    left_y = math.cos(th)            # y component of the robot's left vector
    return "L" if left_y * dy > 0 else "R"


def lane_change(th: float, dy: float, r: float = RADIUS_CM) -> List[Seg]:
    """
    Shift sideways by dy (world y) and end on the same heading. Two equal arcs
    up to 2r of shift, 90° arcs with a straight between beyond that. The arc is
    rounded to whole degrees; the residual is < 0.4 cm at r = 32.
    """
    if abs(dy) < 0.5:
        return []
    first = _turn_toward(th, dy)
    second = "R" if first == "L" else "L"
    if abs(dy) <= 2 * r:
        deg = round(math.degrees(math.acos(1.0 - abs(dy) / (2 * r))))
        if deg < MIN_ARC_DEG:
            return []
        return [Seg(first, deg), Seg(second, deg)]
    mid = round(abs(dy) - 2 * r)
    segs = [Seg(first, 90)]
    if mid >= 1:
        segs.append(Seg("F", mid))
    return segs + [Seg(second, 90)]


def _f(cm: float) -> List[Seg]:
    n = int(round(cm))
    return [Seg("F", n)] if n >= 1 else []


def _cost(segs: Sequence[Seg], r: float, fast: bool = True) -> float:
    t = 0.0
    for s in segs:
        if s.kind in ("F", "B"):
            # Protocol 8 reduces straights to 40 RPM over the final 22 cm
            # in fast mode, or 15 cm in normal mode. Do not price a short
            # positioning move as a full-speed cruise.
            approach = min(s.value, 22.0 if fast else 15.0)
            cruise = FAST_CRUISE_CM_S if fast else CRUISE_CM_S
            t += (s.value - approach) / cruise + approach / APPROACH_CM_S
        else:
            t += math.radians(s.value) * r / ARC_CM_S
    return t


def motion_cost(segs: Sequence[Seg], r: float, fast: bool = True) -> float:
    """Estimated execution seconds; measured full-command times take priority."""
    mode = "fast" if fast else "normal"
    timings = TIMING_PROFILE.get(mode, {})
    overhead = PRIMITIVE_OVERHEAD_S if fast else NORMAL_OVERHEAD_S
    return sum(float(timings[s.token()]) if s.token() in timings
               else _cost([s], r, fast) + overhead for s in segs)


def final_cost(segs: Sequence[Seg], end: Pose, r: float = RADIUS_CM) -> float:
    return (motion_cost(segs, r) + motion_cost(park_segments(end)[0], r, fast=False)
            + 2 * LINE_OVERHEAD_S)


def checkpoint_segments(segs: Sequence[Seg]) -> List[Seg]:
    """Keep finite straights continuous; the executor monitors them in flight."""
    return list(segs)


def ir_side(y: float) -> str:
    """Which IR faces the centreline when heading home from lateral y."""
    return "right" if _turn_toward(HEADING_HOME, -y) == "R" else "left"


def park_segments(p: Pose) -> Tuple[List[Seg], List[str]]:
    """From square on the centreline heading home, to parked. Returns the
    collision-checked straight and the normal F tokens sent to STM."""
    front = p.x - FRONT_FROM_AXLE_CM
    travel = front - (-BAY_DEPTH_CM + PARK_GAP_CM)
    whole = _f(travel)
    return whole, [s.token() for s in whole]


@dataclass
class FinalPlan:
    segs: List[Seg]                 # lead-in straight + S-curve
    park_tokens: List[str]
    clearance: float
    closest: str
    end: Pose                       # where the S leaves it, before parking
    parked: Pose

    def tokens(self) -> List[str]:
        return [s.token() for s in self.segs] + self.park_tokens


def plan_final(p: Pose, course: Course, r: float = RADIUS_CM,
               min_clear: float = CLEARANCE_CM, max_lead: int = 40,
               allow_reverse: bool = True,
               log_fallback: bool = True) -> FinalPlan:
    """Minimum estimated time in the finite forward/reverse S-join family."""
    p = Pose(p.x, p.y, HEADING_HOME)
    boxes = course.boxes()
    if path_clearance([p], boxes)[0] < CLEARANCE_FLOOR_CM:
        raise NoPlan("Measured pose already violates the clearance floor: %s" % p)
    options = []
    leads = list(range(0, max_lead + 1))
    if allow_reverse:
        leads += list(range(-1, -21, -1))
    for lead in leads:
        prefix = _f(lead) if lead >= 0 else [Seg("B", -lead)]
        segs = prefix + lane_change(p.th, -p.y, r)
        end = run(p, segs, r, step=1e6)[-1]
        if end.x - FRONT_FROM_AXLE_CM < -BAY_DEPTH_CM + PARK_GAP_CM:
            continue
        parking, tokens = park_segments(end)
        options.append((final_cost(segs, end, r), segs, end, parking, tokens))
    fallback = None
    for _, segs, end, parking, tokens in sorted(options, key=lambda q: q[0]):
        poses = run(p, segs + parking, r)
        gap, name = path_clearance(poses, boxes)
        candidate = FinalPlan(segs, tokens, gap, name, end, poses[-1])
        if gap >= max(min_clear - 0.01, CLEARANCE_FLOOR_CM):
            return candidate
        if fallback is None and gap >= CLEARANCE_FLOOR_CM:
            fallback = candidate
    if fallback is not None:
        if log_fallback:
            logging.warning("Final join needs the configured clearance floor: "
                            "%.1f cm to %s.", fallback.clearance,
                            fallback.closest)
        return fallback
    raise NoPlan(f"No collision-free final S from {p}.")


def shorten_final(p: Pose, course: Course, baseline: FinalPlan,
                  r: float = RADIUS_CM) -> FinalPlan:
    """Try arc-straight-arc joins after the IR fix, retaining the checked S.

    A diagonal must end square at the garage, not just aim at its centre.
    Solve its straight length from lateral displacement, then check the whole
    body sweep through both arcs and the final parking approach.
    """
    best = baseline
    def duration(plan):
        return final_cost(plan.segs, plan.end, r)

    best_time = duration(best)
    if abs(p.y) < CENTRE_DEADBAND_CM:
        return best
    first = _turn_toward(HEADING_HOME, -p.y)
    second = "R" if first == "L" else "L"
    sign = 1 if first == "L" else -1
    boxes = course.boxes()
    for angle in (15, 30, 45, 60):
        diagonal_heading = HEADING_HOME + sign * math.radians(angle)
        delta = math.atan2(math.sin(diagonal_heading - p.th),
                           math.cos(diagonal_heading - p.th))
        first_angle = int(round(math.degrees(delta) * sign))
        if not MIN_ARC_DEG <= first_angle <= 90:
            continue
        arc1, arc2 = Seg(first, first_angle), Seg(second, angle)
        for lead in (0, 5, 10, 15, 20):
            start = straight(p, lead)
            end_arcs = run(start, [arc1, arc2], r, step=1e6)[-1]
            heading = start.th + math.radians(arc1.turn_deg)
            distance = -end_arcs.y / math.sin(heading)
            if not 1 <= distance <= 200:
                continue
            segs = _f(lead) + [arc1] + _f(distance) + [arc2]
            end = run(p, segs, r, step=1e6)[-1]
            # Remain square before entering the mouth so the existing two-IR
            # centring and ultrasonic parking routine can run unchanged.
            if end.x < IR_FROM_AXLE_CM - CENTRE_CHECK_IN_CM or abs(end.y) > 0.75:
                continue
            parking, tokens = park_segments(end)
            estimate = final_cost(segs, end, r)
            if estimate >= best_time:
                continue
            poses = run(p, segs + parking, r)
            clearance, closest = path_clearance(poses, boxes)
            if clearance < max(CLEARANCE_CM, CLEARANCE_FLOOR_CM):
                continue
            best = FinalPlan(segs, tokens, clearance, closest, end, poses[-1])
            best_time = estimate
    return best


def plan_direct_return(p: Pose, course: Course,
                       r: float = RADIUS_CM) -> Optional[FinalPlan]:
    """A checked diagonal ending square just outside the bay mouth.

    The diagonal stays outside the side fences.  The measured parking step is
    the only movement allowed to cross the mouth, so a mirrored return that
    lands off-centre cannot sweep its final arc into the carpark wall.
    """
    error = math.atan2(math.sin(p.th - HEADING_HOME),
                       math.cos(p.th - HEADING_HOME))
    if abs(error) > math.radians(10) or abs(p.y) < CENTRE_DEADBAND_CM:
        return None
    first = _turn_toward(HEADING_HOME, -p.y)
    second = "R" if first == "L" else "L"
    sign = 1 if first == "L" else -1
    options = []
    for angle in range(15, 61):
        first_angle = int(round(angle - sign * math.degrees(error)))
        if first_angle < MIN_ARC_DEG:
            continue
        for lead in range(0, 21, 2):
            prefix = _f(lead) + [Seg(first, first_angle)]
            at = run(p, prefix, r, step=1e6)[-1]
            last = Seg(second, angle)
            end0 = run(at, [last], r, step=1e6)[-1]
            distance = int(round(-end0.y / math.sin(at.th)))
            if not 1 <= distance <= 300:
                continue
            segs = prefix + [Seg("F", distance), last]
            end = run(p, segs, r, step=1e6)[-1]
            check_x = IR_FROM_AXLE_CM - CENTRE_CHECK_IN_CM
            if (not max(0.0, check_x - 4.0) <= end.x <= check_x
                    or abs(end.y) > 0.75):
                continue
            parking, tokens = park_segments(end)
            options.append((final_cost(segs, end, r), segs,
                            parking, tokens, end))
    boxes = course.boxes()
    # This shortcut omits the obstacle-1 IR fix, but its complete body sweep is
    # checked against every obstacle, wall and garage fence.  Retain the normal
    # planning margin and the independent garage IR/US checks before completion.
    margin = execution_clearance(CLEARANCE_CM)
    for _, segs, parking, tokens, end in sorted(options, key=lambda x: x[0]):
        poses = run(p, segs + parking, r)
        clearance, closest = path_clearance(poses, boxes)
        if clearance >= margin:
            return FinalPlan(segs, tokens, clearance, closest, end, poses[-1])
    return None


def centre_offset(left: Optional[float], right: Optional[float]) -> Optional[float]:
    """
    The rear axle's y from the two side IRs, heading home inside the bay
    mouth, or None if the readings are not both side fences. Heading home the
    left IR faces -y: left = y + c, right = c - y, c the fence-to-sensor gap
    when centred, so y = (left - right) / 2 whatever the mounting offset.
    """
    if left is None or right is None:
        return None
    expect = 2 * (BAY_HALF_WIDTH_CM - IR_SIDE_OFFSET_CM)
    if abs(left + right - expect) > CENTRE_SUM_TOL_CM:
        return None
    return (left - right - IR_LR_BIAS_CM) / 2.0


def single_fence_offset(left: Optional[float], right: Optional[float]
                        ) -> Optional[float]:
    """Infer bay y from one usable side fence when the other IR is out of range."""
    if (left is None) == (right is None):
        return None
    centre_gap = BAY_HALF_WIDTH_CM - IR_SIDE_OFFSET_CM
    if left is not None:
        if not IR_MIN_GAP_CM <= left <= IR_MAX_GAP_CM["left"]:
            return None
        y = left - centre_gap - IR_LR_BIAS_CM / 2.0
    else:
        if not IR_MIN_GAP_CM <= right <= IR_MAX_GAP_CM["right"]:
            return None
        y = centre_gap - right - IR_LR_BIAS_CM / 2.0
    limit = BAY_HALF_WIDTH_CM - HALF_WIDTH_CM - CLEARANCE_FLOOR_CM
    return y if abs(y) <= limit else None


def plan_centre(p: Pose, course: Course, r: float = RADIUS_CM
                ) -> Optional[Tuple[List[Seg], List[str], float]]:
    """
    From inside the mouth at a measured y: a lane change onto the centreline
    (backing out first if there is not the depth left for it), then the park.
    Returns (segs, park tokens, clearance) or None if nothing fits.
    """
    heading_error = math.atan2(math.sin(HEADING_HOME - p.th),
                               math.cos(HEADING_HOME - p.th))
    if abs(heading_error) > math.radians(PARK_HEADING_TOL_DEG):
        return None
    boxes = course.boxes()
    target_front = -BAY_DEPTH_CM + PARK_GAP_CM
    for back in range(0, 61, 3):
        segs = [Seg("B", back)] if back else []
        retreat = run(p, segs, r)
        gaps = [path_clearance([q], boxes)[0] for q in retreat]
        # A drifted pose may already be inside the preferred fence margin.
        # Only a straight retreat with nondecreasing, positive clearance can
        # escape it; all subsequent steering keeps the ordinary margin.
        floor = max(CENTRE_MIN_CLEAR_CM, CLEARANCE_FLOOR_CM)
        escaping_margin = gaps[0] < floor
        if (min(gaps) < 0 or gaps[-1] < floor
                or (escaping_margin and any(
                    b < a - 0.01 for a, b in zip(gaps, gaps[1:])))
                or (not escaping_margin and min(gaps) < floor)):
            continue
        recovery_start = retreat[-1]
        prefix_count = len(segs)
        angle = int(round(abs(math.degrees(heading_error))))
        if angle >= MIN_ARC_DEG:
            segs.append(Seg("L" if heading_error > 0 else "R", angle))
        at = run(p, segs, r, step=1e6)[-1]
        shift = [] if abs(at.y) <= CENTRE_DEADBAND_CM else lane_change(at.th, -at.y, r)
        if shift and segs and segs[-1].kind == shift[0].kind:
            # Preserve the same arc without an unnecessary motor stop between
            # heading correction and the first part of the lane change.
            segs[-1] = Seg(segs[-1].kind, segs[-1].value + shift[0].value)
            shift = shift[1:]
        segs += shift
        poses = run(p, segs, r)
        end = poses[-1]
        if end.x - FRONT_FROM_AXLE_CM < target_front:
            continue
        straight_segs, park_tokens = park_segments(end)
        poses += run(end, straight_segs, r)[1:]
        checked = run(recovery_start, segs[prefix_count:] + straight_segs, r)
        c, _ = path_clearance(checked, boxes)
        if c >= floor:
            return segs, park_tokens, c
        if not shift and not angle:
            break
    return None


@dataclass
class ReturnPlan:
    lane: float                     # |y| of the pass lane beside obstacle 1
    sigma: float                    # +1 home on the left (+y), -1 the right
    side: str                       # IR that watches obstacle 1
    line_a: List[Seg]               # merge into the lane + run past obstacle 1
    x_stop: float                   # where line A should leave the rear axle
    expected_ir_cm: float
    final: FinalPlan                # nominal; recomputed from the IR fix
    clearance: float
    closest: str
    seconds: float

    def summary(self) -> str:
        return (f"lane {self.lane:.0f} cm, {self.side} IR expects "
                f"{self.expected_ir_cm:.1f} cm; line A {','.join(s.token() for s in self.line_a)}"
                f" ; line B {','.join(self.final.tokens())} ; clearance "
                f"{self.clearance:.1f} cm ({self.closest}), ~{self.seconds:.1f} s")


def plan_return(p0: Pose, course: Course, r: float = RADIUS_CM,
                min_clear: float = CLEARANCE_CM) -> ReturnPlan:
    """
    From heading home on the far side of obstacle 2, the fastest plan that
    keeps its required clearance. Time ranks first; clearance breaks ties.
    The finite lane and sensing-stop family is retained.
    """
    p0 = Pose(p0.x, p0.y, HEADING_HOME)
    sigma = 1.0 if p0.y > 0 else -1.0
    side = ir_side(p0.y)
    half1 = OBS1_SIZE_CM / 2.0
    lane_min = half1 + IR_SIDE_OFFSET_CM + IR_MIN_GAP_CM
    lane_max = half1 + IR_SIDE_OFFSET_CM + IR_MAX_GAP_CM[side]
    y0 = abs(p0.y - course.obs1_y)

    # Staying on the lane it is on is allowed a little inside lane_min (the IR
    # still reads from 10 cm); any other lane is reached by a lane change,
    # inward or out.
    lanes = set()
    if lane_min - 4.0 <= y0 <= lane_max:
        lanes.add(round(y0, 1))
    lane = math.ceil(lane_min)
    while lane <= lane_max:
        lanes.add(float(lane))
        lane += 5
    if not lanes:
        raise NoPlan(f"Robot is {y0:.0f} cm off obstacle 1 - outside every "
                     f"usable pass lane ({lane_min:.0f}..{lane_max:.0f}).")

    boxes = course.boxes()
    ir_ready_x = course.x1 + OBS1_SIZE_CM + IR_FROM_AXLE_CM + IR_LEAD_CM
    stop_x_max = course.x1 + IR_FROM_AXLE_CM - IR_TAIL_CM
    best = None
    for lane in sorted(lanes):
        y_lane = course.obs1_y + sigma * lane
        merge = lane_change(p0.th, y_lane - p0.y, r)
        found = None
        # A lane change has to be done and settled before the IR reaches
        # obstacle 1; already on the lane, it only has to not be past it.
        need_x = ir_ready_x if merge else ir_ready_x - IR_LEAD_CM + 1.0
        for lead in range(0, 400, 2):
            segs = _f(lead) + merge
            poses = run(p0, segs, r)
            if poses[-1].x < need_x:
                break
            c, name = path_clearance(poses, boxes)
            if c >= min_clear - 0.01:
                found = (segs, poses, c, name)
                break
            if not merge:
                break
        if found is None:
            continue
        segs, poses, c1, n1 = found
        e1 = poses[-1]
        # Line A runs on past obstacle 1; line B's S must start no later than
        # where it can start without touching obstacle 1, and no earlier than
        # stop_x_max (the IR has to have seen all of obstacle 1 first).
        x_stop = min(stop_x_max, e1.x)
        for _ in range(60):
            run_a = segs + _f(e1.x - x_stop)
            poses_a = run(p0, run_a, r)
            c_a, n_a = path_clearance(poses_a, boxes)
            if c_a < min_clear - 0.01:
                break
            try:
                final = plan_final(poses_a[-1], course, r, min_clear,
                                   max_lead=0, allow_reverse=False,
                                   log_fallback=False)
            except NoPlan:
                final = None
            if final is not None and final.clearance >= min_clear - 0.01:
                clear = min(c_a, final.clearance)
                closest = n_a if c_a <= final.clearance else final.closest
                parking, _ = park_segments(final.end)
                secs = (motion_cost(checkpoint_segments(run_a), r)
                        + final_cost(final.segs, final.end, r)
                        + LINE_OVERHEAD_S * len(checkpoint_segments(run_a)))
                cand = ReturnPlan(lane, sigma, side, run_a, poses_a[-1].x,
                                  lane - half1 - IR_SIDE_OFFSET_CM, final,
                                  clear, closest, secs)
                key = (secs, -clear)
                if best is None or key < best[0]:
                    best = (key, cand)
                break
            x_stop -= 2
    if best is None:
        raise NoPlan(f"No collision-free return from {p0} on {course}.")
    return best[1]


# ═════════════════════════════════════════════════════════════════════════════
# The IR fix
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class IrFix:
    y: float                      # measured rear-axle lateral position
    x_shift: Optional[float]      # correction to the odometry x, or None
    gap_cm: float
    samples: int


def ir_fix(samples: Sequence[Tuple[float, float]], plan: ReturnPlan,
           course: Course) -> Optional[IrFix]:
    """
    samples: (odometry x of the rear axle, IR cm) taken while passing obstacle
    1, heading home. Readings outside the plausible window (the floor beyond
    the obstacle, or the obstacle 2 wall) are ignored.

    Lateral: the median of the closest few readings, which are the ones with
    the obstacle filling the beam. Longitudinal: the midpoint of the window in
    which the obstacle was seen is its centre, once IR_LAG_CM is taken off -
    the filter and the query round trip mean each reading describes the beam
    a little way back along the path, and that shifts both edges the same way.
    """
    window = 15.0
    good = [(x, g) for x, g in samples
            if IR_MIN_GAP_CM - 6 <= g <= plan.expected_ir_cm + window
            and abs(g - plan.expected_ir_cm) <= window]
    if len(good) < 3:
        return None
    gaps = sorted(g for _, g in good)
    near = gaps[:max(3, len(gaps) // 3)]
    gap = near[len(near) // 2]
    y = course.obs1_y + plan.sigma * (OBS1_SIZE_CM / 2.0 + IR_SIDE_OFFSET_CM + gap)

    x_shift = None
    seen = [x for x, g in good if g <= gap + 6]
    if len(seen) >= 3:
        span = max(seen) - min(seen)
        if span <= OBS1_SIZE_CM + 20:
            mid = (max(seen) + min(seen)) / 2.0
            # Heading home the beam is ahead of the axle, at -x, and a late
            # reading describes it IR_LAG_CM back up the path, at +x.
            measured_centre = mid - IR_FROM_AXLE_CM + IR_LAG_CM
            shift = (course.x1 + OBS1_SIZE_CM / 2.0) - measured_centre
            if abs(shift) <= 15:
                x_shift = shift
    return IrFix(y, x_shift, gap, len(good))


# ═════════════════════════════════════════════════════════════════════════════
# Odometry frame
# ═════════════════════════════════════════════════════════════════════════════

class WorldFrame:
    """
    ?WPOSE (mm, since boot, never reset) anchored to the world frame at the
    start pose. The robot starts square in the bay on its centreline, its
    rear bumper T2_START_REAR_GAP_CM off the back fence.
    """

    def __init__(self, stm, start_rear_gap_cm: Optional[float] = None):
        self.stm = stm
        gap = (start_rear_gap_cm if start_rear_gap_cm is not None
               else _env_f("T2_START_REAR_GAP_CM", 2.0))
        self.start = Pose(-BAY_DEPTH_CM + gap + REAR_FROM_AXLE_CM, 0.0, 0.0)
        self.origin = None
        self.x_shift = 0.0
        # Correction to odometry y, from a landmark known to be centred.
        self.y_shift = 0.0
        self.y_override: Optional[float] = None
        self.needs_lateral_fix = False

    def anchor(self) -> bool:
        w = self.stm.read_wpose()
        if w is None:
            return False
        self.origin = w
        return True

    def pose(self) -> Optional[Pose]:
        w = self.stm.read_wpose()
        if w is None or self.origin is None:
            return None
        h0 = math.radians(self.origin["heading_deg"])
        dx = (w["x_mm"] - self.origin["x_mm"]) / 10.0
        dy = (w["y_mm"] - self.origin["y_mm"]) / 10.0
        c, s = math.cos(-h0), math.sin(-h0)
        x = self.start.x + c * dx - s * dy + self.x_shift
        y = self.start.y + s * dx + c * dy + self.y_shift
        th = math.radians(w["heading_deg"]) - h0
        return Pose(x, y, th)


# ═════════════════════════════════════════════════════════════════════════════
# Executor
# ═════════════════════════════════════════════════════════════════════════════

# After sending a movement line, wait this long before the first query. A
# query that lands while the firmware is still taking the line in is dropped
# and answered RESEND (PROTOCOL.md §2) - and the client files that RESEND as
# the MOVEMENT reply. Seen on the floor: F29 "failed" and its OK came after.
POLL_SETTLE_S = 0.1


def run_polling(stm, line: str, poll: Callable[[], bool],
                timeout_s: float = 25.0) -> Optional[str]:
    """
    Send a movement line and call poll() until its reply arrives. poll()
    returning True stops early (the caller has sent RST): returns "STOPPED".
    A RESEND once polling has begun is a query that collided with the line,
    not the line itself - every line here is validated before it is sent -
    so it is logged and the wait goes on for the line's real reply.
    """
    if not stm.send_line(line):
        return None
    time.sleep(POLL_SETTLE_S)
    polled = False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        reply = stm.receive()
        if reply is not None:
            if polled and reply.strip().upper() == "RESEND":
                logging.warning("A poll collided with %s and was dropped "
                                "(RESEND) - still waiting for the line.", line)
                continue
            return reply
        polled = True
        if poll():
            return "STOPPED"
    return None


class ParkRunner:
    """Drives lines A and B. Call when the robot is heading home on the far
    side of obstacle 2, with no line outstanding."""

    def __init__(self, stm, frame: WorldFrame, course: Course,
                 dry_run: bool = False):
        self.stm = stm
        self.frame = frame
        self.course = course
        self.dry_run = dry_run

    def _send(self, segs_or_tokens) -> bool:
        tokens = [t.token() if isinstance(t, Seg) else t for t in segs_or_tokens]
        if not tokens:
            return True
        line = ",".join(tokens)
        if self.dry_run:
            logging.info("[dry-run] would send: %s", line)
            return True
        if not self.stm.send_line(line):
            return False
        reply = self.stm.wait_reply()
        return self._accept(line, reply)

    @staticmethod
    def _accept(line: str, reply: Optional[str]) -> bool:
        if reply is not None and reply.startswith("OK"):
            return True
        logging.error("STM answered %r to %s - stopping, the robot is not "
                      "where the plan thinks it is.", reply, line)
        return False

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
        """True front-bumper gap in cm, with the HC-SR04 bias removed."""
        fields = self.stm.query_fields("?US", timeout=0.5)
        try:
            value = int(fields[0])
        except (TypeError, ValueError, IndexError):
            return None
        return None if value == 0xFFFF else float(value) - US_BIAS_CM

    def _park_cruise(self, travel: int, start: Pose, start_gap: float,
                     final: bool = False) -> bool:
        """One finite approach at precision speed, with live sensor checks."""
        failed = []

        def poll():
            time.sleep(0.02)
            pose, gap = self.frame.pose(), self._us()
            if gap is None:
                # The first query can collide with the motion command and
                # yield RESEND. Retry once; the finite F still ends outside
                # the final approach reserve if feedback stays unavailable.
                gap = self._us()
            left, right = self._ir("left"), self._ir("right")
            side_unsafe = any(v is not None and v < PARK_SIDE_STOP_CM
                              for v in (left, right))
            reason = None
            if pose is None or gap is None or not math.isfinite(gap):
                reason = "lost parking pose or ultrasonic reading"
            else:
                heading_error = parking_heading_error(pose.th)
                along = ((pose.x - start.x) * math.cos(start.th)
                         + (pose.y - start.y) * math.sin(start.th))
                expected = start_gap - along
                # An isolated far return can be the floor or the open mouth.
                # Confirm it once while the finite F remains bounded. Never
                # debounce a near return or extend the commanded travel.
                if (gap > expected + PARK_ECHO_TOL_CM
                        and heading_error <= PARK_HEADING_TOL_DEG
                        and not side_unsafe):
                    retry = self._us()
                    if (retry is not None and math.isfinite(retry)
                            and retry > 0):
                        gap = retry
                if heading_error > PARK_HEADING_TOL_DEG:
                    reason = "heading left the parking tolerance"
                elif abs(gap - expected) > PARK_ECHO_TOL_CM:
                    reason = "ultrasonic echo disagrees with approach travel"
                elif (not final
                      and gap < PARK_GAP_CM + PARK_CRUISE_RESERVE_CM
                      - PARK_GAP_TOL_CM):
                    reason = "entered the reserved final-approach distance early"
                elif final and gap < PARK_GAP_CM - PARK_GAP_TOL_CM:
                    reason = "passed the final parking distance"
            if side_unsafe:
                reason = "insufficient garage side clearance"
            if reason is None:
                return False
            failed.append(reason)
            logging.error("Parking cruise aborted: %s (US=%s, IR=%s/%s).",
                          reason, gap, left, right)
            self.stm.abort()
            return True

        reply = run_polling(self.stm, f"F{travel}", poll)
        if failed:
            return False
        if reply is None or not reply.startswith("OK"):
            self.stm.abort()
            return self._accept(f"F{travel}", reply)
        return True

    def _stable_us(self, attempts: int = 4,
                   tolerance: float = 2.0) -> Optional[float]:
        return stable_us(self._us, attempts, tolerance)

    def _park_to_back_wall(self) -> bool:
        """Measure once, enter the garage in one monitored F, then only verify."""
        pose = self.frame.pose()
        gap = self._stable_us()
        if pose is None or gap is None:
            logging.error("Parking needs stable ?WPOSE and ?US; got pose=%s, gap=%s.",
                          pose, gap)
            return False
        heading_error = parking_heading_error(pose.th)
        if heading_error > PARK_HEADING_TOL_DEG:
            logging.error("Parking heading is %.1f deg off home; refusing a "
                          "straight approach.", heading_error)
            return False
        if gap < PARK_GAP_CM - PARK_GAP_TOL_CM:
            logging.error("Parking overshot: back-wall gap %.1f cm is below "
                          "the %.1f cm target.", gap, PARK_GAP_CM)
            return False
        if gap <= PARK_GAP_CM + PARK_GAP_TOL_CM:
            logging.info("Parking back-wall gap reached: %.1f cm (target %.1f +/- "
                         "%.1f cm).", gap, PARK_GAP_CM, PARK_GAP_TOL_CM)
            return True

        # A robot just outside the mouth can legitimately see slightly more
        # than the bay depth. Larger echoes are a different surface.
        # Accept the bay wall from the mouth, or a proportionally farther echo
        # when WPOSE confirms the axle is still outside it.  Using their sum
        # admitted a 75 cm background echo from an almost-at-mouth pose.
        max_wall_gap = max(
            PARK_MAX_US_CM,
            BAY_DEPTH_CM + max(0.0, pose.x) + 3.0)
        if gap > max_wall_gap:
            logging.error("Parking ?US reads %.1f cm, too far to identify the %.0f "
                          "cm-deep bay's back wall.", gap, BAY_DEPTH_CM)
            return False
        left, right = self._ir("left"), self._ir("right")
        if any(v is not None and v < PARK_SIDE_STOP_CM for v in (left, right)):
            logging.error("Parking side clearance is unsafe (IR %s / %s); not "
                          "advancing.", left, right)
            return False

        travel = max(1, int(math.floor(gap - PARK_GAP_CM)))
        sweep = run(pose, [Seg("F", travel)])
        side_y = centre_offset(left, right)
        y_correction = 0.0 if side_y is None else side_y - pose.y
        straight_margin = (PARK_STRAIGHT_CLEARANCE_CM if side_y is not None
                           else execution_clearance(CLEARANCE_CM))
        if any(abs(y + y_correction) >= (
                   BAY_HALF_WIDTH_CM - straight_margin)
               for sample in sweep for _, y in footprint(sample)):
            logging.error("Single parking F%d would approach a side fence.", travel)
            return False
        logging.info("Single measured garage entry: F%d from stable ?US %.1f cm; "
                     "no trim motions follow.", travel, gap)
        if not self._park_cruise(travel, pose, gap, final=True):
            return False

        final_gap = self._stable_us()
        if final_gap is None:
            logging.error("Parking completed F%d but the back-wall echo could not be "
                          "verified at rest.", travel)
            return False
        if not (PARK_GAP_CM - PARK_GAP_TOL_CM
                <= final_gap <= PARK_GAP_CM + PARK_GAP_TOL_CM):
            logging.error("Parking completed F%d at %.1f cm from the back wall; "
                          "no corrective movement is sent.", travel, final_gap)
            return False
        logging.info("Parking back-wall gap reached in one push: %.1f cm (target "
                     "%.1f +/- %.1f cm).", final_gap, PARK_GAP_CM,
                     PARK_GAP_TOL_CM)
        return True

    def run(self) -> bool:
        return self._run(0)

    @staticmethod
    def _checkpoint_segments(segs: Sequence[Seg]) -> List[Seg]:
        """Measure between primitives, without splitting long straights."""
        return checkpoint_segments(segs)

    def _run_line_a(self, plan: ReturnPlan, samples) -> Optional[bool]:
        """Run line A with measured checkpoints.

        True means complete, None requests a fresh plan from the current pose,
        and False is a communication or already-unsafe failure.
        """
        parts = self._checkpoint_segments(plan.line_a)
        boxes = self.course.boxes()
        blocked = []

        def poll() -> bool:
            g = self._ir(plan.side)
            w = self.frame.pose()
            if g is not None and w is not None:
                samples.append((w.x, g))
            if w is None or not all(math.isfinite(v) for v in (w.x, w.y, w.th)):
                blocked.append("missing measured pose")
            elif seg.kind in ("F", "B"):
                sign = 1 if seg.kind == "F" else -1
                progress = sign * ((w.x - start.x) * math.cos(start.th)
                                   + (w.y - start.y) * math.sin(start.th))
                remaining = max(0, int(math.ceil(seg.value - progress)))
                gap, name = path_clearance(run(w, [Seg(seg.kind, remaining)]), boxes)
                if gap < CLEARANCE_FLOOR_CM:
                    blocked.append("remaining straight approaches %s (%.1f cm)" % (name, gap))
                # RST can coast on the installed firmware. Keep a reserve;
                # never use a long straight to run up to a detected surface.
                if seg.kind == "F":
                    front = self._us()
                    if front is not None and (not math.isfinite(front)
                                               or front < RETURN_STOP_RESERVE_CM + CLEARANCE_CM):
                        blocked.append("front range %s cm" % front)
                side_floor = HALF_WIDTH_CM - IR_SIDE_OFFSET_CM + CLEARANCE_FLOOR_CM
                if g is not None and (not math.isfinite(g) or g < side_floor):
                    blocked.append("side range %s cm" % g)
            if blocked:
                logging.error("Continuous return aborted: %s.", "; ".join(blocked))
                self.stm.abort()
                return True
            return False

        for index, seg in enumerate(parts):
            start = self.frame.pose()
            if start is None:
                logging.error("No ?WPOSE before return checkpoint %s.", seg.token())
                return False
            preview = run(start, [seg])
            clearance, closest = path_clearance(preview, boxes)
            if clearance < CLEARANCE_FLOOR_CM:
                logging.warning("Return checkpoint %s is no longer clear from "
                                "%s (%.1f cm to %s); replanning.", seg.token(),
                                start, clearance, closest)
                return None

            line = seg.token()
            reply = run_polling(self.stm, line, poll)
            if blocked:
                return False
            if not self._accept(line, reply):
                return False
            actual = self.frame.pose()
            if actual is None:
                logging.error("No ?WPOSE after return checkpoint %s.", line)
                return False
            actual_clearance, actual_closest = path_clearance([actual], boxes)
            if actual_clearance < CLEARANCE_FLOOR_CM:
                logging.error("Measured return pose violates the clearance "
                              "floor after %s: %s (%.1f cm to %s).", line,
                              actual, actual_clearance, actual_closest)
                return False

            expected = preview[-1]
            position_error = math.hypot(actual.x - expected.x,
                                        actual.y - expected.y)
            heading_error = abs(math.degrees(math.atan2(
                math.sin(actual.th - expected.th),
                math.cos(actual.th - expected.th))))
            # Do not abandon a paired S/lane-change after its first arc.  The
            # intermediate heading is intentionally far from home, and the
            # return planner assumes the pair has brought it square again.  In
            # the 012106 run, a harmless 3.1 cm miss after FR45 caused a full
            # five-second replan from a 135-degree pose; that replan treated it
            # as 180 degrees and sent F78 in the wrong direction.  Finish the
            # pair, then assess the measured square pose.
            next_seg = parts[index + 1] if index + 1 < len(parts) else None
            paired_arc = (
                seg.kind in ("L", "R")
                and next_seg is not None
                and next_seg.kind in ("L", "R")
                and seg.kind != next_seg.kind)
            if (index + 1 < len(parts) and not paired_arc
                    and (position_error > RETURN_REPLAN_ERROR_CM
                         or heading_error > 4.0)):
                logging.warning("Return checkpoint %s landed %.1f cm / %.1f deg "
                                "off nominal; continuing from measured WPOSE. "
                                "The next primitive is independently checked.",
                                line, position_error, heading_error)
        return True

    def _run(self, replans: int) -> bool:
        p0 = Pose(0, 0, 0) if self.dry_run else self.frame.pose()
        if p0 is None:
            logging.error("No ?WPOSE - cannot plan the return.")
            return False
        direct = plan_direct_return(p0, self.course)
        can_fix = p0.x - IR_FROM_AXLE_CM >= self.course.x1 + OBS1_SIZE_CM + 1.0
        plan = None
        if can_fix:
            try:
                plan = plan_return(p0, self.course)
            except NoPlan:
                try:
                    plan = plan_return(p0, self.course, min_clear=CLEARANCE_FLOOR_CM)
                except NoPlan:
                    pass
        direct_seconds = final_cost(direct.segs, direct.end) if direct is not None else float("inf")
        needs_fix = getattr(self.frame, "needs_lateral_fix", False) is True
        direct_clearance = getattr(direct, "clearance", None)
        direct_has_full_margin = (
            isinstance(direct_clearance, (int, float))
            and direct_clearance >= CLEARANCE_CM)
        # A single-edge obstacle-2 measurement used to force the landmark path
        # unconditionally.  A direct sweep with the full configured clearance
        # is already robust to that uncertainty and avoids two extra S-curves.
        # Marginal shortcuts still retain the obstacle-1 IR correction.
        direct_beats_fallback = (direct is not None and (
            plan is None or direct_seconds + LINE_OVERHEAD_S < plan.seconds))
        if direct is not None and direct_beats_fallback and (
                not needs_fix or direct_has_full_margin or plan is None):
            logging.info("Return selected by estimated time: direct %.2fs, "
                         "IR-fix route %s; %s, clearance %.1f cm.", direct_seconds,
                         "unavailable" if plan is None else "%.2fs" % plan.seconds,
                         ",".join(seg.token() for seg in direct.segs), direct.clearance)
            if not self._send(direct.segs):
                return False
            if self.dry_run:
                return self._send(direct.park_tokens)
            return self._park_from_mouth()
        if not can_fix:
            logging.warning("IR is already past obstacle 1 at %s; measured final join.", p0)
            return self._finish(p0)
        if plan is None:
            logging.error("No checked return from %s.", p0)
            return False
        logging.info("Return from %s: %s", p0, plan.summary())

        if self.dry_run:
            return self._send(plan.line_a) and self._send(plan.final.tokens())

        # Line A, polling the IR facing obstacle 1 until the reply comes in.
        samples = []
        completed = self._run_line_a(plan, samples)
        if completed is None:
            if replans >= RETURN_REPLAN_LIMIT:
                logging.error("Return exceeded its %d measured replans.",
                              RETURN_REPLAN_LIMIT)
                return False
            return self._run(replans + 1)
        if not completed:
            return False

        here = self.frame.pose()
        if here is None:
            logging.error("No ?WPOSE after line A.")
            return False
        fix = ir_fix(samples, plan, self.course)
        if fix is None:
            logging.warning("Obstacle 1 never showed on the %s IR (%d samples) - "
                            "parking on odometry alone, y=%.1f.",
                            plan.side, len(samples), here.y)
        else:
            logging.info("IR fix: gap %.1f cm over %d samples -> y %.1f "
                         "(odometry said %.1f); x shift %s.", fix.gap_cm,
                         fix.samples, fix.y, here.y,
                         "n/a" if fix.x_shift is None else f"{fix.x_shift:+.1f}")
            corrected = Pose(here.x + (fix.x_shift or 0.0), fix.y, here.th)
            corrected_clearance, corrected_closest = path_clearance(
                [corrected], self.course.boxes())
            if corrected_clearance < CLEARANCE_FLOOR_CM:
                # Empty-floor/background IR plateaus can contain a few
                # repeated values and resemble a short obstacle sighting.
                # Never let such a correction move the world frame into a
                # wall or obstacle; retain the measured WPOSE instead.
                logging.warning("Ignoring obstacle-1 IR correction to y=%.1f: "
                                "it would leave %.1f cm to %s (floor %.1f cm).",
                                fix.y, corrected_clearance, corrected_closest,
                                CLEARANCE_FLOOR_CM)
            else:
                if fix.x_shift is not None:
                    self.frame.x_shift += fix.x_shift
                self.frame.y_shift += corrected.y - here.y
                self.frame.needs_lateral_fix = False
                here = corrected

        return self._finish(here)

    def _finish(self, here: Pose) -> bool:
        """
        Line B: the S onto the centreline, on until both side IRs are just
        inside the mouth. Measure the offset off the two side fences. Line C:
        square it up if needed, then park with a measured odometry F.
        """
        try:
            final = plan_final(here, self.course)
        except NoPlan as exc:
            logging.error("%s", exc)
            return False
        shorter = shorten_final(here, self.course, final)
        if shorter is not final:
            logging.info("Shorter checked diagonal garage join: %s; clearance %.1f cm.",
                         ",".join(s.token() for s in shorter.segs), shorter.clearance)
            final = shorter
        check_x = IR_FROM_AXLE_CM - CENTRE_CHECK_IN_CM   # rear axle x there
        line_b = list(final.segs)
        if final.end.x > check_x + 1.0:
            line_b += _f(final.end.x - check_x)
        logging.info("Line B from %s: %s (clearance %.1f cm, %s)", here,
                     ",".join(s.token() for s in line_b), final.clearance,
                     final.closest)
        if not self._send(line_b):
            return False

        if self.dry_run:
            return self._send(final.park_tokens)
        return self._park_from_mouth()

    def _park_from_mouth(self) -> bool:
        """Verify side fences and finish using the measured back-wall gap."""
        if hasattr(self.stm, "set_fast") and not self.stm.set_fast(False):
            logging.error("Cannot select precision speed at the parking mouth.")
            return False
        time.sleep(0.2)                      # let the IR filters settle
        left, right = self._ir("left"), self._ir("right")
        y = centre_offset(left, right)
        if y is None:
            y = single_fence_offset(left, right)
            if y is not None:
                logging.info("One bay fence is visible (IR %s / %s); inferred "
                             "lateral offset %+.1f cm.", left, right, y)
        at = self.frame.pose()
        line_c = None
        if at is None:
            logging.error("Cannot read world pose at the parking mouth.")
            return False
        if y is None:
            logging.warning("Both parking walls are not visible (IR %s / %s); "
                            "using the monitored ?US approach from the mouth.",
                            left, right)
        measured = Pose(at.x, at.y if y is None else y, at.th)

        # A direct return can arrive safely off-centre but a few degrees off
        # square.  Driving that heading all the way through the bay makes the
        # footprint drift into a fence; plan_centre used to answer with a
        # reverse plus two arcs.  Square the heading here with one checked arc
        # first.  If the resulting straight is clear, this is the only trim
        # motion before the single ultrasonic parking push.
        signed_heading_error = math.atan2(
            math.sin(HEADING_HOME - measured.th),
            math.cos(HEADING_HOME - measured.th))
        correction_deg = int(round(abs(math.degrees(signed_heading_error))))
        if MIN_ARC_DEG <= correction_deg <= PARK_HEADING_TOL_DEG:
            correction = Seg(
                "L" if signed_heading_error > 0 else "R", correction_deg)
            corrected = run(measured, [correction], step=1e6)[-1]
            corrected_preview, _ = park_segments(corrected)
            correction_clearance, correction_closest = path_clearance(
                run(measured, [correction] + corrected_preview),
                self.course.fence())
            if correction_clearance >= PARK_STRAIGHT_CLEARANCE_CM:
                logging.info("Parking heading trim: %s replaces a centring "
                             "S; checked straight clearance %.1f cm to %s.",
                             correction.token(), correction_clearance,
                             correction_closest)
                if not self._send([correction]):
                    return False
                at = self.frame.pose()
                if at is None:
                    logging.error("Cannot read world pose after parking heading trim.")
                    return False
                left, right = self._ir("left"), self._ir("right")
                y = centre_offset(left, right)
                measured = Pose(at.x, at.y if y is None else y, at.th)

        preview, _ = park_segments(measured)
        heading_error = parking_heading_error(at.th)
        straight_clearance, straight_closest = path_clearance(
            run(measured, preview), self.course.fence())
        needs_alignment = (
            heading_error > PARK_HEADING_TOL_DEG
            or straight_clearance < PARK_STRAIGHT_CLEARANCE_CM)
        if not needs_alignment:
            logging.info("In the mouth: left IR %s, right IR %s -> %s; straight "
                         "garage entry keeps %.1f cm to %s, so Line C is "
                         "skipped.", left, right,
                         "offset unavailable" if y is None else
                         f"{measured.y:+.1f} cm off centre",
                         straight_clearance, straight_closest)
        if needs_alignment:
            plan = plan_centre(measured, self.course)
            if plan is None:
                logging.error("Off centre by %+.1f cm with no clear parking "
                              "correction; refusing final approach.", measured.y)
                return False
            else:
                segs, park_tokens, c = plan
                # `park_tokens` is the old odometry-only final drive. The
                # bounded ?US loop below owns that distance now; appending
                # both made the robot enter the bay twice and overshoot.
                line_c = [s.token() for s in segs]
                logging.info("In the mouth: left IR %s, right IR %s -> "
                             "%+.1f cm off centre. Line C: %s (clearance %.1f "
                             "cm).", left, right, measured.y,
                             ",".join(line_c) or "centred; ?US approach follows", c)
        if line_c is not None and not self._send(line_c):
            return False

        if not self._park_to_back_wall():
            return False

        time.sleep(0.2)
        us = self._stable_us()
        left, right = self._ir("left"), self._ir("right")
        y = centre_offset(left, right)
        logging.info("Parked. Front gap ?US=%s cm (want %.0f); side "
                     "gaps left %s / right %s%s.", us, PARK_GAP_CM, left, right,
                     "" if y is None else f" -> {y:+.1f} cm off centre")
        return True


# ═════════════════════════════════════════════════════════════════════════════
# Laptop entry point
# ═════════════════════════════════════════════════════════════════════════════

def nominal_start(course: Course, side: str, lane_out: float = 20.0) -> Pose:
    """Where the obstacle 2 loop is expected to leave the robot: heading home,
    just past obstacle 2's far face, `lane_out` cm of body gap off its side."""
    sigma = 1.0 if side == "left" else -1.0
    y = sigma * (course.w2 / 2.0 + HALF_WIDTH_CM + lane_out)
    return Pose(course.x2 + OBS2_DEPTH_CM + FRONT_FROM_AXLE_CM, y, HEADING_HOME)


def _plot(course: Course, p0: Pose, plan: ReturnPlan, path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle

    fig, ax = plt.subplots(figsize=(10, 6))
    for b in course.boxes():
        ax.add_patch(Rectangle((b.x0, b.y0), b.x1 - b.x0, b.y1 - b.y0,
                               color="0.3"))
    poses = run(p0, plan.line_a)
    poses += run(poses[-1], plan.final.segs)[1:]
    straight_segs, _ = park_segments(poses[-1])
    poses += run(poses[-1], straight_segs)[1:]
    ax.plot([q.x for q in poses], [q.y for q in poses], "b-", lw=1)
    for q in poses[::12] + [poses[-1]]:
        ax.add_patch(Polygon(footprint(q), fill=False, ec="tab:blue", lw=0.4))
    ax.axvline(plan.x_stop, color="tab:orange", ls=":", lw=1)
    ax.set_aspect("equal")
    ax.set_title(plan.summary(), fontsize=7)
    fig.savefig(path, dpi=150, bbox_inches="tight")


def main() -> int:
    ap = argparse.ArgumentParser(description="Plan the Task 2 return and park.")
    ap.add_argument("d1", type=float, help="bay mouth to obstacle 1, cm (60-150)")
    ap.add_argument("d2", type=float, help="obstacle 1 to obstacle 2, cm (60-150)")
    ap.add_argument("w2", type=float, help="obstacle 2 width, cm (30-90)")
    ap.add_argument("side", choices=("left", "right"),
                    help="side the robot comes home on")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    course = Course(x1=a.d1, x2=a.d1 + OBS1_SIZE_CM + a.d2, w2=a.w2)
    p0 = nominal_start(course, a.side)
    plan = plan_return(p0, course)
    print(f"start {p0}\n{plan.summary()}\nparked {plan.final.parked}")
    if a.plot:
        _plot(course, p0, plan, "task2t_park.png")
        print("wrote task2t_park.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
