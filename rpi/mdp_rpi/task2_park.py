"""
task2_park.py - Task 2 (fastest car): the drive home and the park.

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
                 the centreline, square, before the mouth; F covers the bulk
                 and FU stops it a measured distance off the back fence.

Everything here except ParkRunner is pure geometry and runs on a laptop:

    python task2_park.py 60 60 90 right          # print the plan, worst case
    python task2_park.py 100 80 40 left --plot   # and draw it (task2_park.png)

WORLD FRAME (cm). Origin at the bay mouth on its centreline. +x points out of
the bay toward the obstacles, +y is to the left looking out from the bay,
heading is counter-clockwise from +x. The robot's pose is its rear-axle
midpoint: the point the odometry integrates and the arcs turn about.
"""

import argparse
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
# SAFE's radius is NOT measured yet (motion.c still carries TIGHT's 29.1 as a
# placeholder, and SAFE is expected wider). 32 is a deliberately pessimistic
# guess: a wider radius needs more room for every S-curve. Set T2_RADIUS_CM
# to the floor-chord result before a real run.
RADIUS_CM = _env_f("T2_RADIUS_CM", 32.0)
ROBOT_LENGTH_CM = 23.0
ROBOT_WIDTH_CM = 18.8
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
# "Minimum clearance 50 cm" either side of obstacle 2 - there may be a wall.
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
# The candidate with the biggest clearance wins, up to this; past it time wins.
CLEARANCE_ENOUGH_CM = _env_f("T2_CLEARANCE_ENOUGH_CM", 12.0)
# Front bumper to back fence when parked. Rear is then
# BAY_DEPTH - PARK_GAP - LENGTH = 15 cm inside the mouth.
PARK_GAP_CM = _env_f("T2_PARK_GAP_CM", 12.0)
# FU handles only the last stretch; F at cruise covers the rest (PROTOCOL §4.1).
FU_LEAD_CM = _env_f("T2_FU_LEAD_CM", 25.0)
PARK_MODE = os.getenv("T2_PARK_MODE", "fu").strip().lower()   # fu | odom
# Straight run the IR gets before it reaches obstacle 1, so the S that put the
# robot in the lane has finished settling before the readings start.
IR_LEAD_CM = _env_f("T2_IR_LEAD_CM", 8.0)
# Line A stops this far after the IR has cleared obstacle 1's near face.
IR_TAIL_CM = _env_f("T2_IR_TAIL_CM", 4.0)
MIN_ARC_DEG = 5        # a smaller arc does not move the chassis (task_a5.py)
# Centring in the bay. The S stops with both side IRs this far inside the
# mouth, where each sees a side fence; half the difference of the two readings
# is the robot's offset from the bay centreline. Off by more than the deadband,
# one small lane change squares it up before the final FU.
CENTRE_CHECK_IN_CM = _env_f("T2_CENTRE_CHECK_IN_CM", 5.0)
CENTRE_DEADBAND_CM = _env_f("T2_CENTRE_DEADBAND_CM", 1.0)
CENTRE_CREEP_CM = 4
# Readings that do not add up to the bay's width less the robot's are not the
# two fences (an IR not inside yet, or reading something else): no correction.
CENTRE_SUM_TOL_CM = _env_f("T2_CENTRE_SUM_TOL_CM", 10.0)
CENTRE_MIN_CLEAR_CM = _env_f("T2_CENTRE_MIN_CLEAR_CM", 3.0)
# Left IR minus right IR with the robot centred and square in the bay by hand.
# The two units are fitted separately (ir.h) and need not agree; whatever
# this is, the robot parks half of it off centre unless it is set here.
IR_LR_BIAS_CM = _env_f("T2_IR_LR_BIAS_CM", 0.0)

# ── Rough timing, only for ranking candidates ────────────────────────────────
CRUISE_CM_S = 34.5                       # PROTOCOL.md §4.1 at 150 rpm
ARC_CM_S = CRUISE_CM_S * 120.0 / 150.0   # SAFE arcs at 120 rpm
LINE_OVERHEAD_S = 0.6                    # stop + round trip per line
FU_OVERHEAD_S = 1.2                      # PROTOCOL.md §4.1

HEADING_HOME = math.pi


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
    """One primitive: 'F' / 'B' cm forward / reverse, or 'L' / 'R' degrees of
    forward arc."""
    kind: str
    value: int

    def token(self) -> str:
        return {"F": "F", "B": "R", "L": "FL", "R": "FR"}[self.kind] + str(self.value)


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
            turn = 1 if s.kind == "L" else -1
            n = max(1, int(math.ceil(math.radians(s.value) * r / step)))
            for i in range(1, n + 1):
                out.append(arc(p, turn, s.value * i / n, r))
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
    corners = ((b.x0, b.y0), (b.x1, b.y0), (b.x1, b.y1), (b.x0, b.y1))
    best = float("inf")
    for i in range(4):
        a, c = poly[i], poly[(i + 1) % 4]
        for q in corners:
            best = min(best, _seg_dist(q[0], q[1], a[0], a[1], c[0], c[1]))
        a, c = corners[i], corners[(i + 1) % 4]
        for q in poly:
            best = min(best, _seg_dist(q[0], q[1], a[0], a[1], c[0], c[1]))
    return best


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
    # Wall inner face |y|. None: the brief's minimum, w2/2 + 50. Before the
    # width is measured, pass the narrowest case's 65 with w2 = 90 - the
    # conservative box for both at once.
    wall_y: Optional[float] = None

    def obs1(self) -> Box:
        h = OBS1_SIZE_CM / 2.0
        return Box(self.x1, self.x1 + OBS1_SIZE_CM,
                   self.obs1_y - h, self.obs1_y + h, "obstacle 1")

    def obs2(self) -> Box:
        h = self.w2 / 2.0
        return Box(self.x2, self.x2 + OBS2_DEPTH_CM, -h, h, "obstacle 2")

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
        x0, x1 = 0.0, self.x2 + OBS2_DEPTH_CM + 150.0
        return [Box(x0, x1, y, y + 10.0, "left wall"),
                Box(x0, x1, -y - 10.0, -y, "right wall")]

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


def _cost(segs: Sequence[Seg], r: float) -> float:
    t = 0.0
    for s in segs:
        if s.kind in ("F", "B"):
            t += s.value / CRUISE_CM_S
        else:
            t += math.radians(s.value) * r / ARC_CM_S
    return t


def ir_side(y: float) -> str:
    """Which IR faces the centreline when heading home from lateral y."""
    return "right" if _turn_toward(HEADING_HOME, -y) == "R" else "left"


def park_segments(p: Pose) -> Tuple[List[Seg], List[str]]:
    """From square on the centreline heading home, to parked. Returns the
    whole straight in (what gets collision-checked) and the tokens sent: F at
    cruise for the bulk, FU on the back fence for the last FU_LEAD_CM."""
    front = p.x - FRONT_FROM_AXLE_CM
    travel = front - (-BAY_DEPTH_CM + PARK_GAP_CM)
    whole = _f(travel)
    if PARK_MODE == "odom":
        return whole, [s.token() for s in whole]
    bulk = _f(travel - FU_LEAD_CM) if travel - FU_LEAD_CM >= 5 else []
    fu = int(round(PARK_GAP_CM + US_BIAS_CM))
    return whole, [s.token() for s in bulk] + [f"FU{fu}"]


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
               min_clear: float = CLEARANCE_CM, max_lead: int = 40) -> FinalPlan:
    """
    From a measured pose heading home, the S onto the centreline and the park.
    Takes the earliest start that keeps min_clear: the sooner the S is done
    the longer the robot runs square before the mouth.
    """
    p = Pose(p.x, p.y, HEADING_HOME)      # the carry-over squares the heading
    boxes = course.boxes()
    best = None
    for lead in range(0, max_lead + 1):
        segs = _f(lead) + lane_change(p.th, -p.y, r)
        poses = run(p, segs, r)
        end = poses[-1]
        straight_segs, park_tokens = park_segments(end)
        front = end.x - FRONT_FROM_AXLE_CM
        if front < -BAY_DEPTH_CM + PARK_GAP_CM:
            break                          # the S itself ends too deep
        poses += run(end, straight_segs, r)[1:]
        c, name = path_clearance(poses, boxes)
        cand = FinalPlan(segs, park_tokens, c, name, end, poses[-1])
        if c >= min_clear - 0.01:
            return cand
        if best is None or c > best.clearance:
            best = cand
    if best is not None and best.clearance > 0:
        logging.warning("Final S keeps only %.1f cm (wanted %.1f) - closest is "
                        "the %s. Taking it anyway.", best.clearance, min_clear,
                        best.closest)
        return best
    raise NoPlan(f"No collision-free final S from {p}.")


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


def plan_centre(p: Pose, course: Course, r: float = RADIUS_CM
                ) -> Optional[Tuple[List[Seg], List[str], float]]:
    """
    From inside the mouth at a measured y: a lane change onto the centreline
    (backing out first if there is not the depth left for it), then the park.
    Returns (segs, park tokens, clearance) or None if nothing fits.
    """
    p = Pose(p.x, p.y, HEADING_HOME)
    shift = [] if abs(p.y) <= CENTRE_DEADBAND_CM else lane_change(p.th, -p.y, r)
    boxes = course.boxes()
    target_front = -BAY_DEPTH_CM + PARK_GAP_CM
    for back in range(0, 31, 3):
        segs = ([Seg("B", back)] if back else []) + shift
        poses = run(p, segs, r)
        end = poses[-1]
        if end.x - FRONT_FROM_AXLE_CM < target_front:
            continue
        straight_segs, park_tokens = park_segments(end)
        poses += run(end, straight_segs, r)[1:]
        c, _ = path_clearance(poses, boxes)
        if c >= CENTRE_MIN_CLEAR_CM:
            return segs, park_tokens, c
        if not shift:
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
    keeps CLEARANCE_CM everywhere, preferring more clearance up to
    CLEARANCE_ENOUGH_CM.
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
                final = plan_final(poses_a[-1], course, r, min_clear, max_lead=0)
            except NoPlan:
                final = None
            if final is not None and final.clearance >= min_clear - 0.01:
                clear = min(c_a, final.clearance)
                closest = n_a if c_a <= final.clearance else final.closest
                secs = (_cost(run_a, r) + _cost(final.segs, r)
                        + (final.end.x - final.parked.x) / CRUISE_CM_S
                        + 2 * LINE_OVERHEAD_S
                        + (FU_OVERHEAD_S if PARK_MODE == "fu" else 0.0))
                cand = ReturnPlan(lane, sigma, side, run_a, poses_a[-1].x,
                                  lane - half1 - IR_SIDE_OFFSET_CM, final,
                                  clear, closest, secs)
                key = (min(clear, CLEARANCE_ENOUGH_CM), -secs)
                if best is None or key > best[0]:
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
        self.y_override: Optional[float] = None

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
        y = self.start.y + s * dx + c * dy
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

    def run(self) -> bool:
        p0 = Pose(0, 0, 0) if self.dry_run else self.frame.pose()
        if p0 is None:
            logging.error("No ?WPOSE - cannot plan the return.")
            return False
        if p0.x - IR_FROM_AXLE_CM < self.course.x1 + OBS1_SIZE_CM + 1.0:
            # The side IR is already level with or past obstacle 1 (obstacle
            # 2 close behind it): no fix to take. Park on odometry - the
            # route since the bay is short.
            logging.warning("IR is already past obstacle 1 at %s - parking on "
                            "odometry, no IR fix.", p0)
            return self._finish(p0)
        try:
            plan = plan_return(p0, self.course)
        except NoPlan:
            try:
                plan = plan_return(p0, self.course, min_clear=CLEARANCE_FLOOR_CM)
                logging.warning("Return keeps only %.1f cm (want %.0f) - taking "
                                "it.", plan.clearance, CLEARANCE_CM)
            except NoPlan as exc:
                logging.error("%s", exc)
                return False
        logging.info("Return from %s: %s", p0, plan.summary())

        if self.dry_run:
            return self._send(plan.line_a) and self._send(plan.final.tokens())

        # Line A, polling the IR facing obstacle 1 until the reply comes in.
        line = ",".join(s.token() for s in plan.line_a)
        samples = []

        def poll() -> bool:
            g = self._ir(plan.side)
            w = self.frame.pose()
            if g is not None and w is not None:
                samples.append((w.x, g))
            return False

        reply = run_polling(self.stm, line, poll)
        if not self._accept(line, reply):
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
            if fix.x_shift is not None:
                self.frame.x_shift += fix.x_shift
            here = Pose(here.x + (fix.x_shift or 0.0), fix.y, here.th)

        return self._finish(here)

    def _finish(self, here: Pose) -> bool:
        """
        Line B: the S onto the centreline, on until both side IRs are just
        inside the mouth. Measure the offset off the two side fences. Line C:
        square it up if needed, F, FU off the back fence.
        """
        try:
            final = plan_final(here, self.course)
        except NoPlan as exc:
            logging.error("%s", exc)
            return False
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
        time.sleep(0.2)                      # let the IR filters settle
        left, right = self._ir("left"), self._ir("right")
        y = centre_offset(left, right)
        # Short of the mouth (a turn that ran wide, odometry long), the IRs
        # see past the fences' ends. Creep in and look again.
        for _ in range(3):
            if y is not None:
                break
            logging.info("Side IRs read %s / %s - not both fences yet; "
                         "creeping in %d cm.", left, right, CENTRE_CREEP_CM)
            if not self._send([f"F{CENTRE_CREEP_CM}"]):
                return False
            time.sleep(0.2)
            left, right = self._ir("left"), self._ir("right")
            y = centre_offset(left, right)
        at = self.frame.pose()
        line_c = None
        if y is None or at is None:
            logging.warning("Side IRs read %s / %s inside the mouth - not both "
                            "fences. Parking without centring.", left, right)
        else:
            plan = plan_centre(Pose(at.x, y, HEADING_HOME), self.course)
            if plan is None:
                logging.warning("Off centre by %+.1f cm but no clear correction "
                                "fits - parking as is.", y)
            else:
                segs, park_tokens, c = plan
                line_c = [s.token() for s in segs] + park_tokens
                logging.info("In the mouth: left IR %.0f, right IR %.0f -> "
                             "%+.1f cm off centre. Line C: %s (clearance %.1f "
                             "cm).", left, right, y, ",".join(line_c), c)
        if line_c is None:
            at = at or here
            _, line_c = park_segments(Pose(at.x, at.y, HEADING_HOME))
        if not self._send(line_c):
            return False

        time.sleep(0.2)
        us = self.stm.query_fields("?US")
        left, right = self._ir("left"), self._ir("right")
        y = centre_offset(left, right)
        logging.info("Parked. Front gap ?US=%s (want %.0f + %.1f bias); side "
                     "gaps left %s / right %s%s.", us, PARK_GAP_CM, US_BIAS_CM,
                     left, right,
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
        _plot(course, p0, plan, "task2_park.png")
        print("wrote task2_park.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
