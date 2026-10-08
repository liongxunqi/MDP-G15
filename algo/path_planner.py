"""Task 1 path planning: visit order + motion. Replaces compute_path()'s stub in task1_pc.py.

Per obstacle the planner picks a photo pose: nose toward the image, the
ultrasonic reading the chosen standoff. Each leg drives onto the straight
approach line in front of that pose, then FU<n> closes the rest. Legs avoid
EVERY obstacle, including the one being approached, and the next leg starts
from where the photo was taken.

The RPi sends every token of a segment to the STM as its own line, reads
?WPOSE after each OK, and (per PROTOCOL.md 12) STOPS the mission if that
position is outside the arena. Two things follow, both handled here:

  * the planner's reference point must stay where the RPi will not halt, in the
    RPi's frame (see RPI_* below) - not just where the body fits; and
  * the first leg may not leave the arena near the start corner, and may never
    begin with a reverse (see _start_boxes).

PATH also carries "expected": the pose the RPi should report after every
instruction, in the RPi's frame, so the PC can compare it with PROGRESS.
"""

import itertools
import logging
import math
import os
import re
import time
from collections import Counter, namedtuple
from typing import Dict, List, Optional, Tuple

from stm_tokens import (
    ARENA_CM,
    FU_MAX_CM,
    FU_MIN_CM,
    PROFILE_TIGHT,
    ROBOT_LENGTH_CM,
    ROBOT_WIDTH_CM,
    REAR_AXLE_TO_SENSOR_CM,
    TURN_RADIUS_MM,
    chunk_tokens,
    fwd,
    fwd_until,
    rev,
    stop,
)

import grid_search

ARENA_MM = ARENA_CM * 10
OBSTACLE_SIZE_MM = 100.0

# Clearance kept between the robot's real outline and every obstacle, on top
# of the 23 x 18.8 cm footprint itself. Covers odometry drift and turn slop.
COLLISION_MARGIN_MM = 30.0

# Ultrasonic reading at the photo, cm - the FU<n> target - tried in this order.
# 30 is where YOLO is most confident; further out is next best. The camera is
# usable down to 20 cm, but that close pose is reserved for a face near an arena
# edge where the normal distances would put the rear axle beyond the boundary.
STANDOFF_PREFERENCE_CM = (30, 32, 33, 35, 38, 40, 42, 45)
STANDOFF_LAST_RESORT_CM = (28, 20)
STANDOFF_CANDIDATES_CM = STANDOFF_PREFERENCE_CM + STANDOFF_LAST_RESORT_CM
FU_COMPENSATE = False

# Closed-loop recovery. A replacement returns to the endpoint of the segment
# that was already being executed, then reuses the untouched later segments.
# The grid search may join this much of the endpoint's final straight line.
RECOVERY_APPROACH_LINE_MM = 600.0
RECOVERY_LATERAL_TOL_MM = 25.0
# grid_search has 45-degree heading states. Small measured heading errors are normal and
# may be snapped for recovery planning; larger errors need a planner that models
# arbitrary headings and are deliberately not guessed here.
RECOVERY_MAX_HEADING_SNAP_DEG = float(os.getenv(
    "RECOVERY_MAX_HEADING_SNAP_DEG", "8"
))

# How far back from the photo pose a leg may join the approach line, and how
# far off it sideways. Off-line by more than this and the camera misses the
# 10 cm image, or the ultrasonic locks onto a neighbouring obstacle.
APPROACH_LINE_MAX_MM = 600.0
APPROACH_LATERAL_TOL_MM = 25.0
# FU is slow and its beam spreads (PROTOCOL.md 4.1), so F<n> covers most of
# the approach line and FU only the last stretch - at most FU_RUNIN_MM, and
# less if a neighbouring obstacle sits inside the beam from further back.
# FU stops at the NEAREST echo: a neighbour closer than the image would stop
# the robot short, or make it reverse to reach the standoff.
FU_RUNIN_MM = 150.0
SONAR_HALF_ANGLE_DEG = 10.0

# The RPi gives up on PATH after PATH_TIMEOUT_S (30 s). Stop trying other
# standoffs once this much time has gone and send the best plan so far.
PLAN_TIME_BUDGET_S = 20.0

EXHAUSTIVE_LIMIT = 8

FACE_FROM_D = {0: "N", 2: "E", 4: "S", 6: "W"}

ROBOT_HALF_LENGTH_MM = ROBOT_LENGTH_CM * 10 / 2.0
ROBOT_HALF_WIDTH_MM = ROBOT_WIDTH_CM * 10 / 2.0
# FU<n> measures from the front sensor, but the path and STM WPOSE both track
# the rear axle. This measured offset is therefore part of every photo pose.
REAR_AXLE_TO_SENSOR_MM = float(os.getenv(
    "REAR_AXLE_TO_SENSOR_CM", str(REAR_AXLE_TO_SENSOR_CM)
)) * 10.0

# Start pose: robot facing N, pushed into the start zone's bottom-left corner -
# rear on the bottom line, left side on the left line, START_GAP_MM off each so
# nothing sits on the tape. Two edges are easy to line up by touch; the centre
# of a 40x40 cm box is not. START_X_MM / START_Y_MM in the PC's environment
# override the computed centre for fine-tuning on the day.
START_GAP_MM = float(os.getenv("START_GAP_MM", "10"))
START_X_MM = float(os.getenv("START_X_MM", ROBOT_HALF_WIDTH_MM + START_GAP_MM))
START_Y_MM = float(os.getenv("START_Y_MM", ROBOT_HALF_LENGTH_MM + START_GAP_MM))
START_THETA = math.pi / 2

# ── The frame the RPi reports positions in ───────────────────────────────────
# After each instruction the RPi reads ?WPOSE and anchors the robot's rear-axle
# reference at this position. By default that is the planner's calculated start,
# not an unrelated (0,0) or (2,2). The values remain configurable for a measured
# floor mark. PATH.start is only the bottom-left cell of Android's 2 x 2 drawing.
# Displacements are the same in both frames, so
# rpi_position = planner_position + (anchor - planner_start).
RPI_ANCHOR_X_MM = float(os.getenv("RPI_ANCHOR_X_MM", str(START_X_MM)))
RPI_ANCHOR_Y_MM = float(os.getenv("RPI_ANCHOR_Y_MM", str(START_Y_MM)))
# Keep the reference point this far INSIDE the RPi's arena check: dead reckoning
# drifts, and the check is a hard stop. Set RPI_ARENA_GUARD=0 only if the RPi no
# longer halts on out-of-arena positions.
RPI_ARENA_GUARD = os.getenv("RPI_ARENA_GUARD", "1").strip().lower() not in ("0", "false", "no")
RPI_ARENA_MARGIN_MM = float(os.getenv("RPI_ARENA_MARGIN_MM", "50"))

# The leg that leaves START may not put the body outside the arena within this
# distance of the start, and may never reverse out of the start pose.
START_GUARD_MM = float(os.getenv("START_GUARD_MM", "700"))
# ...but a body 10 mm from the line still sweeps a few mm across it in a normal
# forward-right arc (6.8 mm at FR90), and a person places it to a few mm. Allow
# that much; a forward-left arc from the corner (302 mm out) stays forbidden.
START_GUARD_TOL_MM = float(os.getenv("START_GUARD_TOL_MM", "25"))
# When NO clean way out of the start exists (obstacles close by, robot against two
# lines), a leg that reverses or leaves the arena is used rather than returning
# an empty plan - but only if nothing clean exists for that obstacle, and it is
# charged this much so any clean alternative wins the tour.
START_RELAXED_PENALTY_MM = float(os.getenv("START_RELAXED_PENALTY_MM", "2500"))
_REAR_BLOCK_DEPTH_MM = 600.0     # how far behind the start pose the "no reverse" block reaches
_REAR_BLOCK_WIDTH_MM = 150.0     # ... and how far it extends past each side of the body
_BIG_MM = 10_000.0

_MERGEABLE_RE = re.compile(r"^(FR|FL|RR|RL|F|R)(\d+)$")


class Pose:
    __slots__ = ("x", "y", "theta")

    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, _wrap(theta)

    def __repr__(self):
        return f"Pose({self.x:.1f}, {self.y:.1f}, {math.degrees(self.theta):.1f}deg)"


# fu_runin_mm: how far before the photo pose FU may start with only the
# target in its beam; None when even FU from the photo pose itself would hear
# a neighbour first, and the approach has to be F<n> alone.
PhotoOption = namedtuple("PhotoOption", "standoff_cm pose line fu_runin_mm")


def _wrap(theta):
    return math.atan2(math.sin(theta), math.cos(theta))


def _viewing_pose(obstacle: dict, standoff_mm: float) -> Optional[Pose]:
    """Rear-axle pose when the front sensor reads standoff_mm to the face."""
    d = obstacle.get("d")
    if d not in FACE_FROM_D:
        return None
    face = FACE_FROM_D[d]

    cx = obstacle["x"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    cy = obstacle["y"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    dist = OBSTACLE_SIZE_MM / 2.0 + standoff_mm + REAR_AXLE_TO_SENSOR_MM

    if face == "N":
        return Pose(cx, cy + dist, -math.pi / 2)
    if face == "S":
        return Pose(cx, cy - dist, math.pi / 2)
    if face == "E":
        return Pose(cx + dist, cy, math.pi)
    return Pose(cx - dist, cy, 0.0)


def _obstacle_aabb_mm(obstacle: dict) -> Tuple[float, float, float, float]:
    """The obstacle grown by COLLISION_MARGIN_MM on every side."""
    x0 = obstacle["x"] * 100.0 - COLLISION_MARGIN_MM
    y0 = obstacle["y"] * 100.0 - COLLISION_MARGIN_MM
    size = OBSTACLE_SIZE_MM + 2 * COLLISION_MARGIN_MM
    return (x0, y0, x0 + size, y0 + size)


def _frame_shift(start: "Pose") -> Tuple[float, float]:
    """Add this to a planner-frame position to get the RPi-frame position."""
    return RPI_ANCHOR_X_MM - start.x, RPI_ANCHOR_Y_MM - start.y


def _rpi_ref_bounds(start: "Pose") -> Tuple[float, float, float, float]:
    """(lo_x, hi_x, lo_y, hi_y) in the PLANNER's frame that keeps the reference
    point inside the arena as the RPi sees it, RPI_ARENA_MARGIN_MM in from each
    edge. Applied exactly, at every heading, via grid_search.Boxes.ref_bounds."""
    sx, sy = _frame_shift(start)
    m = RPI_ARENA_MARGIN_MM
    return (m - sx, ARENA_MM - m - sx, m - sy, ARENA_MM - m - sy)


def _start_boxes(start: "Pose") -> List[Tuple[float, float, float, float]]:
    """Extra blockers for the leg that leaves START, and only that leg.

    1. Outside the arena, within START_GUARD_MM of the start: the body must stay
       on the real floor while leaving the corner. The planner otherwise lets it
       hang up to ARENA_OVERHANG_MM past an edge (for obstacles near far walls),
       and from a start 10 mm off the line that meant reversing straight out of
       the arena.
    2. A block directly behind the start pose: the robot cannot reverse out of
       where it was placed, so the first instruction is never a reverse (and
       neither is anything that drives back through the start line).

    START_THETA is cardinal; "behind" is taken along the nearest cardinal."""
    g, tol = START_GUARD_MM, START_GUARD_TOL_MM
    boxes = []
    if start.y < g:
        boxes.append((start.x - g, -_BIG_MM, start.x + g, -tol))
    if start.y > ARENA_MM - g:
        boxes.append((start.x - g, ARENA_MM + tol, start.x + g, _BIG_MM))
    if start.x < g:
        boxes.append((-_BIG_MM, start.y - g, -tol, start.y + g))
    if start.x > ARENA_MM - g:
        boxes.append((ARENA_MM + tol, start.y - g, _BIG_MM, start.y + g))

    w = _REAR_BLOCK_WIDTH_MM
    face = _nearest_cardinal(start.theta)
    hl, hw = ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM
    if face == "N":
        r = start.y - hl
        boxes.append((start.x - hw - w, r - _REAR_BLOCK_DEPTH_MM, start.x + hw + w, r))
    elif face == "S":
        r = start.y + hl
        boxes.append((start.x - hw - w, r, start.x + hw + w, r + _REAR_BLOCK_DEPTH_MM))
    elif face == "E":
        r = start.x - hl
        boxes.append((r - _REAR_BLOCK_DEPTH_MM, start.y - hw - w, r, start.y + hw + w))
    else:
        r = start.x + hl
        boxes.append((r, start.y - hw - w, r + _REAR_BLOCK_DEPTH_MM, start.y + hw + w))
    return boxes


def _pose_clear(x, y, theta, boxes) -> bool:
    return not grid_search._point_blocked(
        x, y, theta, boxes, ARENA_MM, ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM,
    )


def _sonar_sees_target_first(pose: Pose, back_mm: float, standoff_mm: float,
                             others: List[dict]) -> bool:
    """From back_mm behind pose - anywhere across the approach line's width -
    is the image face the nearest echo in the cone?"""
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    squares = [(o["x"] * 100.0, o["y"] * 100.0) for o in others]
    target_range = standoff_mm + back_mm
    steps = 4
    for side in (-APPROACH_LATERAL_TOL_MM, 0.0, APPROACH_LATERAL_TOL_MM):
        sx = pose.x + (REAR_AXLE_TO_SENSOR_MM - back_mm) * c - side * s
        sy = pose.y + (REAR_AXLE_TO_SENSOR_MM - back_mm) * s + side * c
        for i in range(-steps, steps + 1):
            a = pose.theta + math.radians(SONAR_HALF_ANGLE_DEG) * i / steps
            ca, sa = math.cos(a), math.sin(a)
            limit = target_range / max(abs(math.cos(a - pose.theta)), 1e-6)
            d = 0.0
            while d < limit:
                px, py = sx + d * ca, sy + d * sa
                if any(x0 <= px <= x0 + OBSTACLE_SIZE_MM and y0 <= py <= y0 + OBSTACLE_SIZE_MM
                       for x0, y0 in squares):
                    return False
                d += 10.0
    return True


def _fu_runin(pose: Pose, standoff_mm: float, others: List[dict]) -> Optional[float]:
    runin = None
    back = 0.0
    while back <= FU_RUNIN_MM:
        if not _sonar_sees_target_first(pose, back, standoff_mm, others):
            break
        runin = back
        back += 25.0
    return runin


def _photo_options(obstacle: dict, boxes, obstacles: List[dict]) -> List[PhotoOption]:
    """Every standoff in STANDOFF_CANDIDATES_CM whose photo pose is clear, in
    preference order, each with the stretch of approach line that is clear."""
    options = []
    for standoff_cm in STANDOFF_CANDIDATES_CM:
        pose = _viewing_pose(obstacle, standoff_cm * 10.0)
        if pose is None or not _pose_clear(pose.x, pose.y, pose.theta, boxes):
            continue
        length = 0.0
        while length + 10.0 <= APPROACH_LINE_MAX_MM:
            back = length + 10.0
            if not _pose_clear(pose.x - back * math.cos(pose.theta),
                               pose.y - back * math.sin(pose.theta), pose.theta, boxes):
                break
            length = back
        line = grid_search.ApproachLine(pose.x, pose.y, pose.theta, length, APPROACH_LATERAL_TOL_MM)
        others = [o for o in obstacles if o is not obstacle
                  and (o["x"], o["y"]) != (obstacle["x"], obstacle["y"])]
        runin = _fu_runin(pose, standoff_cm * 10.0, others)
        options.append(PhotoOption(standoff_cm, pose, line, runin))
    return options


def _search_leg(from_pose: Pose, option: PhotoOption, radius_mm, boxes):
    """(tokens, poses, cost) onto option's approach line, or None. Cost
    includes the straight run-in along the line, so legs compare fairly."""
    try:
        tokens, raw_poses, cost = grid_search.search_leg(
            from_pose.x, from_pose.y, from_pose.theta, option.line,
            radius_mm, boxes, ARENA_MM, ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM,
        )
    except grid_search.NoPathFound:
        return None
    poses = [Pose(x, y, t) for x, y, t in raw_poses]
    back, _ = option.line.offsets(poses[-1].x, poses[-1].y)
    return tokens, poses, cost + max(back, 0.0)


def _merge_runs(tokens: List[str], poses: List[Pose]):
    """F5,F5,F5 -> F15 and FR45,FR45 -> FR90: one primitive instead of several
    saves the STM a brake-and-align stop each. poses[i] is the pose AFTER
    tokens[i]."""
    out_t, out_p = [], []
    for tok, pose in zip(tokens, poses):
        m = _MERGEABLE_RE.match(tok)
        prev = _MERGEABLE_RE.match(out_t[-1]) if out_t else None
        if m and prev and m.group(1) == prev.group(1):
            out_t[-1] = f"{m.group(1)}{int(prev.group(2)) + int(m.group(2))}"
            out_p[-1] = pose
        else:
            out_t.append(tok)
            out_p.append(pose)
    return out_t, out_p


def _edge_cost(cache, from_id, to_id) -> float:
    entry = cache.get((from_id, to_id))
    return math.inf if entry is None else entry[2]


def _route_cost(order, cache) -> float:
    total = 0.0
    prev_id = "START"
    for obs in order:
        total += _edge_cost(cache, prev_id, obs["id"])
        prev_id = obs["id"]
    return total


def _reachable(visitable, cache):
    """Split obstacles into those with at least one finite edge INTO them
    (from the start or another obstacle) and those with none, which can never
    appear in any tour."""
    ids = [o["id"] for o in visitable]
    ok, dropped = [], []
    for obs in visitable:
        sources = ["START"] + [i for i in ids if i != obs["id"]]
        if any(math.isfinite(_edge_cost(cache, src, obs["id"])) for src in sources):
            ok.append(obs)
        else:
            dropped.append(obs)
    return ok, dropped


def _prefix_score(order, cache):
    """(obstacles reached before the first impossible leg, cost of those legs)."""
    visited, cost, prev_id = 0, 0.0, "START"
    for obs in order:
        step = _edge_cost(cache, prev_id, obs["id"])
        if not math.isfinite(step):
            break
        visited += 1
        cost += step
        prev_id = obs["id"]
    return visited, cost


def _visit_order(visitable, cache, quiet=False):
    """Always returns every obstacle, best first. When no complete tour exists
    - two dead ends, say - the order that reaches the MOST obstacles before
    its first impossible leg wins, cheapest among those. plan_mission() then
    skips whatever comes after that leg instead of losing the whole plan."""
    if len(visitable) <= 1:
        return list(visitable)

    if len(visitable) > EXHAUSTIVE_LIMIT:
        return _greedy_order(visitable, cache)

    best_key, best_order = None, None
    for perm in itertools.permutations(visitable):
        visited, cost = _prefix_score(perm, cache)
        key = (-visited, cost)
        if best_key is None or key < best_key:
            best_key, best_order = key, list(perm)
    assert best_key is not None and best_order is not None  # len >= 2, so a perm exists

    visited = -best_key[0]
    if visited < len(visitable) and not quiet:
        logging.warning(
            f"No complete tour: best order reaches {visited}/{len(visitable)} "
            f"obstacles; skipping {[o['id'] for o in best_order[visited:]]}."
        )
    if not quiet:
        logging.info(f"Visit order: {[o['id'] for o in best_order]}  cost={best_key[1]:.0f}mm")
    return best_order


def _greedy_order(visitable, cache):
    """Nearest-neighbour, then 2-opt. Always returns an order, never None."""
    remaining = list(visitable)
    order = []
    prev_id = "START"
    while remaining:
        nearest = min(remaining, key=lambda o: _edge_cost(cache, prev_id, o["id"]))
        remaining.remove(nearest)
        order.append(nearest)
        prev_id = nearest["id"]

    improved = True
    best_cost = _route_cost(order, cache)
    while improved:
        improved = False
        for i in range(len(order)):
            for j in range(i + 1, len(order)):
                candidate = order[:i] + list(reversed(order[i:j + 1])) + order[j + 1:]
                cand_cost = _route_cost(candidate, cache)
                if cand_cost < best_cost - 1e-6:
                    order, best_cost = candidate, cand_cost
                    improved = True
    logging.info(f"Visit order: {[o['id'] for o in order]}  cost={best_cost:.0f}mm")
    return order


class _Legs:
    """Every leg searched so far, keyed by which standoff each end uses, so
    switching an obstacle's standoff back and forth never searches twice."""

    def __init__(self, start, options, radius_mm, boxes, deadline, start_boxes=None):
        self.start, self.options = start, options
        self.radius_mm, self.boxes = radius_mm, boxes
        # Legs that begin at START are searched against these instead.
        self.start_boxes = start_boxes if start_boxes is not None else boxes
        self.deadline = deadline
        self.legs = {}
        self.relaxed = set()     # keys of START legs that had to ignore the start rules

    def _search(self, from_id, choice, to_id, key):
        from_pose = self._from_pose(from_id, choice)
        option = self.options[to_id][choice[to_id]]
        if from_id != "START":
            return _search_leg(from_pose, option, self.radius_mm, self.boxes)
        leg = _search_leg(from_pose, option, self.radius_mm, self.start_boxes)
        if leg is None and self.start_boxes is not self.boxes:
            leg = _search_leg(from_pose, option, self.radius_mm, self.boxes)
            if leg is not None:
                tokens, poses, cost = leg
                leg = (tokens, poses, cost + START_RELAXED_PENALTY_MM)
                self.relaxed.add(key)
        return leg

    def _from_pose(self, from_id, choice):
        if from_id == "START":
            return self.start
        return self.options[from_id][choice[from_id]].pose

    def view(self, visitable, choice) -> dict:
        """{(from_id, to_id): (tokens, poses, cost) or None} for the current choice."""
        ids = [o["id"] for o in visitable]
        cache = {}
        for from_id in ["START"] + ids:
            from_c = None if from_id == "START" else choice[from_id]
            for to_id in ids:
                if to_id == from_id:
                    continue
                key = (from_id, from_c, to_id, choice[to_id])
                if key not in self.legs and time.monotonic() > self.deadline:
                    # Out of time: treat it as impossible rather than make the
                    # RPi wait past its PATH timeout. Not cached - it isn't known.
                    cache[(from_id, to_id)] = None
                    continue
                if key not in self.legs:
                    self.legs[key] = self._search(from_id, choice, to_id, key)
                cache[(from_id, to_id)] = self.legs[key]
        return cache


def _nearest_cardinal(theta: float) -> str:
    deg = math.degrees(theta) % 360
    if deg >= 315 or deg < 45:
        return "E"
    if deg < 135:
        return "N"
    if deg < 225:
        return "W"
    return "S"


# Android draws the robot as an N x N block of 10 cm cells and places it by the
# block's BOTTOM-LEFT cell (arena-ui ArenaConfig.robotFootprintCells, default 2).
ANDROID_ROBOT_CELLS = 2


def _pose_to_dir_entry(pose: Pose) -> dict:
    """The bottom-left cell of the block Android draws, chosen so that block is
    centred on the robot's real centre. A robot centred at (20, 20) cm sends
    (1, 1): cells 1-2, i.e. 10-30 cm, centred at 20 cm."""
    n = ANDROID_ROBOT_CELLS
    top = 20 - n   # last anchor that keeps the whole block on the 20 x 20 grid
    # floor(v + 0.5): halves always round up (Python's round() goes to even).
    gx = max(0, min(top, math.floor(pose.x / 100.0 - n / 2.0 + 0.5)))
    gy = max(0, min(top, math.floor(pose.y / 100.0 - n / 2.0 + 0.5)))
    return {"x": int(gx), "y": int(gy), "dir": _nearest_cardinal(pose.theta)}


# An obstacle with no way in from ANY source at this many consecutive standoffs is
# written off. The standoffs differ by centimetres, so the approach line is in
# the same place at the next one: it is a dead end, not a near miss. Without
# this, every dead end was retried at all 8 standoffs, each retry re-searching a
# whole matrix of legs - 20 s of planning spent on obstacles that cannot be shot.
# (Obstacles that ARE reachable keep every standoff.)
MAX_DEAD_OPTION_TRIES = 3


def _choose_tour(visitable, legs: _Legs, options):
    """Visit order + standoff per obstacle. Starts every obstacle at its most
    preferred standoff; while the tour misses some, moves those (and the dead
    end that cut the tour short) to their next standoff, keeping the best."""
    deadline = legs.deadline
    choice = {o["id"]: 0 for o in visitable}
    dead_tries = {o["id"]: 0 for o in visitable}   # standoffs tried with no way in
    best = None
    while True:
        cache = legs.view(visitable, choice)
        ok, unreachable = _reachable(visitable, cache)
        for o in unreachable:
            dead_tries[o["id"]] += 1
        order = _visit_order(ok, cache, quiet=True) + unreachable
        reached, cost = _prefix_score(order, cache)
        if best is None or (reached, -cost) > (best[0], -best[1]):
            best = (reached, cost, order, dict(choice), cache)
        if reached == len(visitable):
            break
        if time.monotonic() > deadline:
            logging.warning(f"Planning budget of {PLAN_TIME_BUDGET_S:.0f}s used — "
                            "sending the best plan so far.")
            break
        movers = [o for o in order[reached:] if dead_tries[o["id"]] < MAX_DEAD_OPTION_TRIES]
        if not movers:
            break       # everything cut off is a dead end at every standoff tried
        retry = movers + (order[reached - 1:reached] if reached else [])
        changed = False
        for obs in retry:
            if choice[obs["id"]] + 1 < len(options[obs["id"]]):
                choice[obs["id"]] += 1
                changed = True
        if not changed:
            break

    reached, cost, order, choice, cache = best
    if reached < len(order):
        logging.warning(
            f"No complete tour: best order reaches {reached}/{len(order)} "
            f"obstacles; skipping {[o['id'] for o in order[reached:]]}."
        )
    logging.info(f"Visit order: {[o['id'] for o in order]}  cost={cost:.0f}mm")
    return order, choice, cache


def _plan_frame_shift(plan: dict) -> Tuple[float, float]:
    """Translation from planner coordinates to the RPi's reported frame."""
    shift = plan.get("frame_shift_mm")
    if isinstance(shift, dict):
        try:
            return float(shift["x"]), float(shift["y"])
        except (KeyError, TypeError, ValueError):
            pass
    start = plan.get("start_mm")
    if isinstance(start, dict):
        try:
            return RPI_ANCHOR_X_MM - float(start["x"]), RPI_ANCHOR_Y_MM - float(start["y"])
        except (KeyError, TypeError, ValueError):
            pass
    return 0.0, 0.0


def _straight_distance_clear(x: float, y: float, theta: float, distance_mm: float,
                             boxes) -> bool:
    """Collision-check an arbitrary final straight, not only a 50 mm A* step."""
    samples = max(1, int(math.ceil(abs(distance_mm) / 20.0)))
    for i in range(1, samples + 1):
        d = distance_mm * i / samples
        if not _pose_clear(x + d * math.cos(theta), y + d * math.sin(theta), theta, boxes):
            return False
    return True


def plan_segment_recovery(plan: dict, obstacles: List[dict], progress: dict,
                          arc_profile: int = PROFILE_TIGHT) -> Optional[dict]:
    """Return a replacement that recovers the current segment endpoint.

    The completed prefix is discarded, the current segment is replaced by a
    collision-checked route from the measured pose to its original endpoint,
    and every later segment is copied unchanged. This preserves the original
    visit order and avoids recomputing good future work merely because one
    primitive drifted. None means a safe recovery could not be produced.
    """
    try:
        seg = int(progress["segment_index"])
        actual_x = float(progress["x_grid"]) * 100.0
        actual_y = float(progress["y_grid"]) * 100.0
        actual_hdg = float(progress["heading_deg"]) % 360.0
        segments = plan["segments"]
        mappings = plan["segment_obstacles"]
        expected = plan["expected"]
        target_x, target_y, target_hdg = (float(v) for v in expected[seg][-1])
    except (KeyError, IndexError, TypeError, ValueError):
        logging.exception("Cannot build recovery: plan/progress shape is inconsistent.")
        return None

    if not (len(segments) == len(mappings) == len(expected)):
        logging.error("Cannot build recovery: PATH arrays are not parallel.")
        return None

    reported_remaining = progress.get("remaining_photo_ids")
    if isinstance(reported_remaining, list):
        planned_remaining = [str(v) for v in mappings[seg:] if v is not None]
        if Counter(str(v) for v in reported_remaining) != Counter(planned_remaining):
            logging.error(
                "Cannot build recovery: RPi pending photos %s do not match PC route %s.",
                reported_remaining, planned_remaining,
            )
            return None

    shift_x, shift_y = _plan_frame_shift(plan)
    ax, ay = actual_x - shift_x, actual_y - shift_y
    tx, ty = target_x - shift_x, target_y - shift_y
    actual_theta = math.radians(90.0 - actual_hdg)
    actual_lattice = grid_search._HEADINGS[grid_search._heading_index(actual_theta)]
    snap_error = abs(math.degrees(_wrap(actual_theta - actual_lattice)))
    if snap_error > RECOVERY_MAX_HEADING_SNAP_DEG:
        logging.error(
            "Cannot safely recover segment %d: measured heading %.1f deg is %.1f deg "
            "from the nearest 45-degree heading (limit %.1f deg).",
            seg, actual_hdg, snap_error, RECOVERY_MAX_HEADING_SNAP_DEG,
        )
        return None

    target_theta = math.radians(90.0 - target_hdg)
    target_theta = grid_search._HEADINGS[grid_search._heading_index(target_theta)]
    boxes = grid_search.Boxes([_obstacle_aabb_mm(o) for o in obstacles])
    if RPI_ARENA_GUARD:
        m = RPI_ARENA_MARGIN_MM
        boxes.ref_bounds = (
            m - shift_x, ARENA_MM - m - shift_x,
            m - shift_y, ARENA_MM - m - shift_y,
        )

    if not _pose_clear(ax, ay, actual_lattice, boxes):
        logging.error("Cannot safely recover segment %d: measured pose intersects a boundary/obstacle.", seg)
        return None
    if not _pose_clear(tx, ty, target_theta, boxes):
        logging.error("Cannot safely recover segment %d: original endpoint is no longer clear.", seg)
        return None

    tokens, poses = [], []
    terminal = Pose(ax, ay, actual_lattice)

    # Most drift is longitudinal (the FU failure in the physical logs was
    # 169 mm). Correct that directly instead of making the 50 mm search grid
    # overshoot and then drive back a few centimetres.
    dx, dy = tx - ax, ty - ay
    along = dx * math.cos(target_theta) + dy * math.sin(target_theta)
    lateral = -dx * math.sin(target_theta) + dy * math.cos(target_theta)
    same_heading = grid_search._heading_index(actual_lattice) == grid_search._heading_index(target_theta)
    direct_cm = int(round(abs(along) / 10.0))
    direct_mm = math.copysign(direct_cm * 10.0, along) if direct_cm else 0.0
    if (same_heading and abs(lateral) <= RECOVERY_LATERAL_TOL_MM and direct_cm and
            _straight_distance_clear(ax, ay, target_theta, direct_mm, boxes)):
        tokens = [fwd(direct_cm) if direct_mm > 0 else rev(direct_cm)]
        terminal = Pose(
            ax + direct_mm * math.cos(target_theta),
            ay + direct_mm * math.sin(target_theta),
            target_theta,
        )
        poses = [terminal]
    else:
        # A long approach line gives A* more possible endpoints, but the first
        # endpoint it finds can have an obstacle between it and the real target.
        # Retry shorter lines until both the searched route and its final
        # straight are collision-free. This matters near arena edges where the
        # long-line route can also choose the wrong side of a neighbouring box.
        approach_lengths = sorted({
            float(value) for value in
            (RECOVERY_APPROACH_LINE_MM, 500, 400, 300, 200, 150, 100, 75, 50, 25)
            if 0 < float(value) <= RECOVERY_APPROACH_LINE_MM
        }, reverse=True)
        last_error = None
        for approach_length in approach_lengths:
            goal = grid_search.ApproachLine(
                tx, ty, target_theta, approach_length, RECOVERY_LATERAL_TOL_MM,
            )
            try:
                raw_tokens, raw_poses, _ = grid_search.search_leg(
                    ax, ay, actual_lattice, goal, TURN_RADIUS_MM[arc_profile], boxes,
                    ARENA_MM, ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM,
                )
            except grid_search.NoPathFound as exc:
                last_error = exc
                continue

            candidate_poses = [Pose(x, y, theta) for x, y, theta in raw_poses[1:]]
            candidate_tokens, candidate_poses = _merge_runs(
                list(raw_tokens), candidate_poses
            )
            candidate_terminal = (
                Pose(ax, ay, actual_lattice) if not candidate_poses
                else candidate_poses[-1]
            )
            back, _ = goal.offsets(candidate_terminal.x, candidate_terminal.y)
            run_cm = max(0, int(round(back / 10.0)))
            run_mm = run_cm * 10.0
            if (run_cm and not _straight_distance_clear(
                    candidate_terminal.x, candidate_terminal.y,
                    target_theta, run_mm, boxes)):
                last_error = ValueError(
                    f"final straight from {approach_length:.0f} mm approach is obstructed"
                )
                continue
            if run_cm:
                candidate_tokens.append(fwd(run_cm))
                candidate_terminal = Pose(
                    candidate_terminal.x + run_mm * math.cos(target_theta),
                    candidate_terminal.y + run_mm * math.sin(target_theta),
                    target_theta,
                )
                candidate_poses.append(candidate_terminal)
            tokens, poses, terminal = (
                candidate_tokens, candidate_poses, candidate_terminal
            )
            break
        else:
            logging.error(
                "Cannot safely recover segment %d: %s.",
                seg, last_error or "no collision-free approach",
            )
            return None

    if not tokens:
        logging.warning(
            "Recovery requested for segment %d, but the grid planner produced no corrective movement.",
            seg,
        )
        return None

    tokens.append(stop())
    poses.append(terminal)
    recovery_segments = chunk_tokens(tokens)
    recovery_expected = []
    offset = 0
    for line in recovery_segments:
        line_poses = poses[offset:offset + len(line)]
        recovery_expected.append([
            [round(p.x + shift_x, 1), round(p.y + shift_y, 1),
             round((90.0 - math.degrees(p.theta)) % 360.0, 1)]
            for p in line_poses
        ])
        offset += len(line)

    current_target = mappings[seg]
    recovery_mappings = [None] * len(recovery_segments)
    recovery_mappings[-1] = current_target
    replacement = {
        "segments": recovery_segments + [list(s) for s in segments[seg + 1:]],
        "segment_obstacles": recovery_mappings + list(mappings[seg + 1:]),
        "expected": recovery_expected + list(expected[seg + 1:]),
        "frame_shift_mm": {"x": shift_x, "y": shift_y},
        "start_mm": dict(plan.get("start_mm", {})),
    }
    logging.warning(
        "Recovery for segment %d: measured (%.0f,%.0f) @ %.1f deg -> original endpoint "
        "(%.0f,%.0f) @ %.1f deg using %s; preserving %d later segment(s).",
        seg, actual_x, actual_y, actual_hdg, target_x, target_y, target_hdg,
        ",".join(tokens), len(segments) - seg - 1,
    )
    return replacement


def plan_mission(obstacles: List[dict], arc_profile: int = PROFILE_TIGHT,
                 details: Optional[dict] = None) -> dict:
    """Plan Task 1. Pass a dict as `details` to get each photo's pose and
    standoff back (tests and the simulator use it; the RPi does not)."""
    t0 = time.monotonic()
    radius_mm = TURN_RADIUS_MM[arc_profile]
    start = Pose(START_X_MM, START_Y_MM, START_THETA)
    # Every obstacle, the one being approached included: even the closest
    # photo pose (28 cm) is far outside its margin, so nothing needs excluding.
    obstacle_boxes = [_obstacle_aabb_mm(o) for o in obstacles]
    boxes = grid_search.Boxes(obstacle_boxes)
    if RPI_ARENA_GUARD:
        boxes.ref_bounds = _rpi_ref_bounds(start)

    # Extra blockers for the leg leaving START. If the start pose already breaks
    # them (START_X_MM / START_Y_MM set so the body is over the line) they would
    # trap the search at its first step, so drop them and say so.
    start_boxes = grid_search.Boxes(list(boxes) + _start_boxes(start))
    start_boxes.ref_bounds = boxes.ref_bounds
    if not _pose_clear(start.x, start.y, start.theta, start_boxes):
        logging.warning(
            "Start pose is already outside the arena or over the start guard "
            f"({start}); not restricting the first leg. Check START_X_MM / START_Y_MM."
        )
        start_boxes = boxes

    options: Dict[object, List[PhotoOption]] = {}
    visitable = []
    skipped = {}
    for obs in obstacles:
        if obs.get("d") not in FACE_FROM_D:
            logging.info(f"Obstacle {obs.get('id')}: SKIP (d={obs.get('d')}).")
            continue
        opts = _photo_options(obs, boxes, obstacles)
        if not opts:
            if boxes.ref_bounds is not None and _photo_options(obs, obstacle_boxes, obstacles):
                logging.error(
                    f"Obstacle {obs['id']}: its photo spot is only reachable by taking the robot "
                    f"past the point where the RPi stops the mission (position outside the "
                    f"arena, PROTOCOL.md 12) — skipping. RPI_ARENA_GUARD=0 allows it, at the "
                    f"risk of that stop."
                )
                skipped[obs["id"]] = "photo spot beyond the RPi arena limit"
            else:
                logging.error(
                    f"Obstacle {obs['id']}: no clear photo spot at "
                    f"{min(STANDOFF_CANDIDATES_CM)}-{max(STANDOFF_CANDIDATES_CM)} cm — skipping."
                )
                skipped[obs["id"]] = "no photo spot"
            continue
        options[obs["id"]] = opts
        visitable.append(obs)

    legs = _Legs(start, options, radius_mm, boxes, t0 + PLAN_TIME_BUDGET_S, start_boxes)
    order, choice, cache = _choose_tour(visitable, legs, options)

    segments: List[List[str]] = []
    segment_obstacles: List[Optional[str]] = []
    dirs: List[dict] = []
    # expected[i][j] = [x_mm, y_mm, heading_deg] the RPi should report after
    # instruction j of segment i: its frame (north = 0, clockwise positive).
    expected: List[List[List[float]]] = []
    shift_x, shift_y = _frame_shift(start)
    photo_poses, standoffs = {}, {}
    safe_standoffs = {
        str(obstacle_id): [option.standoff_cm for option in obstacle_options]
        for obstacle_id, obstacle_options in options.items()
    }

    cur_id = "START"
    start_relaxed = False
    for obs in order:
        option = options[obs["id"]][choice[obs["id"]]]
        leg = cache.get((cur_id, obs["id"]))
        if leg is None:
            logging.error(f"Obstacle {obs['id']}: no route from {cur_id} — skipping.")
            skipped[obs["id"]] = "no route"
            continue

        search_tokens, search_poses, _ = leg
        if cur_id == "START" and ("START", None, obs["id"], choice[obs["id"]]) in legs.relaxed:
            start_relaxed = True
            gap = min(start.x - ROBOT_HALF_WIDTH_MM, start.y - ROBOT_HALF_LENGTH_MM)
            logging.warning(
                f"First leg (to obstacle {obs['id']}) could not leave the start cleanly: with the "
                f"robot {gap:.0f} mm from the nearest line, obstacles close to the start leave no "
                f"route that stays on the floor without reversing. "
                f"It starts with {search_tokens[0] if search_tokens else '?'} and may put the body "
                f"over the line. Place the robot further into the zone (START_GAP_MM / "
                f"START_X_MM / START_Y_MM) to avoid this."
            )
        tokens, poses = _merge_runs(list(search_tokens), list(search_poses[1:]))

        final = option.pose
        back, _ = option.line.offsets(search_poses[-1].x, search_poses[-1].y)
        runin = option.fu_runin_mm
        # F<n> up to where FU may start (all the way, if FU can't be trusted).
        # Rounded UP before FU, so it never starts further out than the beam
        # was checked; to the nearest cm without FU, so F can't overshoot.
        if runin is None:
            run_cm = round(back / 10.0)
        else:
            run_cm = math.ceil((back - runin) / 10.0 - 1e-6)
        if run_cm > 0:
            rest = back - run_cm * 10.0
            tokens.append(fwd(run_cm))
            poses.append(Pose(final.x - rest * math.cos(final.theta),
                              final.y - rest * math.sin(final.theta), final.theta))
        if runin is None:
            logging.warning(f"Obstacle {obs['id']}: a neighbour is inside the ultrasonic "
                            f"beam — final approach on odometry only, no FU.")
        else:
            assert FU_MIN_CM <= option.standoff_cm <= FU_MAX_CM
            tokens.append(fwd_until(option.standoff_cm, compensate=FU_COMPENSATE))
            poses.append(final)
        tokens.append(stop())
        poses.append(final)

        if option.standoff_cm != STANDOFF_CANDIDATES_CM[0]:
            level = logging.WARNING if option.standoff_cm in STANDOFF_LAST_RESORT_CM else logging.INFO
            logging.log(level, f"Obstacle {obs['id']}: photo at {option.standoff_cm} cm"
                               f"{' (last resort)' if level == logging.WARNING else ''}.")

        line_start = 0
        lines = chunk_tokens(tokens)
        for i, line in enumerate(lines):
            segments.append(line)
            expected.append([
                [round(q.x + shift_x, 1), round(q.y + shift_y, 1),
                 round((90.0 - math.degrees(q.theta)) % 360.0, 1)]
                for q in poses[line_start:line_start + len(line)]
            ])
            line_start += len(line)
            dirs.append(_pose_to_dir_entry(poses[line_start - 1]))
            segment_obstacles.append(str(obs["id"]) if i == len(lines) - 1 else None)

        photo_poses[obs["id"]] = (final.x, final.y, final.theta)
        standoffs[obs["id"]] = option.standoff_cm
        cur_id = obs["id"]

    logging.info(f"Planned {len(photo_poses)}/{len(obstacles)} photo(s) in "
                 f"{time.monotonic() - t0:.1f}s.")
    if details is not None:
        details["photo_poses"] = photo_poses
        details["standoff_cm"] = standoffs
        details["skipped"] = skipped
        details["frame_shift_mm"] = (shift_x, shift_y)
        details["start_relaxed"] = start_relaxed

    return {
        "segments": segments,
        "obstacle_ids": [o["id"] for o in order],
        "segment_obstacles": segment_obstacles,
        "dirs": dirs,
        "start": _pose_to_dir_entry(start),
        # Android's drawing anchor above is not an odometry point. Send the
        # reporting origin separately so the RPi has no hardcoded (2,2).
        "odometry_start": {
            "x": RPI_ANCHOR_X_MM / 100.0,
            "y": RPI_ANCHOR_Y_MM / 100.0,
            "dir": "N",
        },
        "start_mm": {"x": round(start.x, 1), "y": round(start.y, 1),
                     "heading": _nearest_cardinal(start.theta)},
        "frame_shift_mm": {"x": round(shift_x, 1), "y": round(shift_y, 1)},
        # Camera retries may move only between these collision-checked poses.
        # Keys are strings because JSON object keys arrive that way on the RPi.
        "photo_standoffs": safe_standoffs,
        "selected_standoffs": {
            str(obstacle_id): standoff
            for obstacle_id, standoff in standoffs.items()
        },
        "expected": expected,
    }
