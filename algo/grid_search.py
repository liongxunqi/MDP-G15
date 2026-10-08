"""Hybrid A* motion planner with collision-checked arc primitives."""

import heapq
import logging
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from stm_tokens import TokenError, arc, fwd, rev

STEP_MM = 50.0
REV_COST_MULT = 1.15
# Each 90 degrees of turning costs this much on top of its arc length: a turn
# eats floor, takes longer than a straight, and is where odometry error comes
# from. Charged pro rata per TURN_STEP, so two consecutive 45-degree checkpoints
# cost what a single 90-degree primitive used to.
TURN_PENALTY_MM = 100.0
# A search that has not found the goal by now is not going to, and a failing
# one burns the whole cap - a plan with many dead ends (obstacles hugging a
# wall, with the RPi's arena limit in force) can spend its whole 20 s budget on
# them. With 4 headings the most any of 168 successful leg searches used was
# 2,622, so this was 8,000. The 45-degree lattice has 8 headings: across 766
# successful searches on 35 layouts the median was 656 and the most 15,837, so
# 24,000 is 1.5x that.
MAX_EXPANSIONS = 24_000
# States closer than this (and on the same heading) count as the same place.
# Without it, arcs land on fresh sub-millimetre coordinates every time and the
# search re-explores the same floor over and over.
DEDUP_CELL_MM = 25.0
ARC_SAMPLES = 12   # collision checks per 45 degree arc: one every 3.75 degrees
# >1 trades a little path length for a much faster search (weighted A*).
HEURISTIC_WEIGHT = 1.5
# Keep the complete chassis inside the 2 m arena. This remains configurable for
# offline experiments, but Task 1 must never rely on empty floor beyond a line:
# there may be a wall immediately outside it on assessment day.
ARENA_OVERHANG_MM = float(os.getenv("ARENA_OVERHANG_MM", "0"))

# Every arc turns the body 45 degrees, so the lattice has 8 headings: the four
# cardinals plus the diagonals, counter-clockwise from E. Straights run on the
# diagonals too. path_planner keeps each arc separate so pose and IR feedback
# can be checked halfway through what would otherwise be a blind 90-degree arc.
TURN_STEP_DEG = 45
TURN_STEP = math.radians(TURN_STEP_DEG)
_HEADINGS = tuple(math.atan2(math.sin(i * TURN_STEP), math.cos(i * TURN_STEP)) for i in range(8))
FINE_TURN_STEPS_DEG = (15, 30, 45)
_FINE_TURN_QUANTUM_DEG = 15
_FINE_TURN_QUANTUM = math.radians(_FINE_TURN_QUANTUM_DEG)
_FINE_HEADINGS = tuple(
    math.atan2(math.sin(i * _FINE_TURN_QUANTUM), math.cos(i * _FINE_TURN_QUANTUM))
    for i in range(360 // _FINE_TURN_QUANTUM_DEG)
)


class NoPathFound(Exception):
    pass


class Boxes(list):
    """Obstacle boxes, optionally carrying ref_bounds = (lo_x, hi_x, lo_y, hi_y):
    the robot's REFERENCE POINT (its centre) must stay inside, whatever the heading.
    It rides on the boxes object so the search functions need no extra argument.
    The planner uses it to keep the position the RPi will compute from ?WPOSE
    inside the arena, where it halts the mission (stm/PROTOCOL.md 12). Blocking
    BODIES with boxes only approximates that: the body's bounding half-extent
    runs from 94 mm to 148 mm with heading, so a box tight for one heading is
    wrong for another."""
    ref_bounds = None
    # Motion states and ?WPOSE track the rear axle. Collision checks translate
    # that reference to the physical centre of the chassis at every sample.
    body_centre_ahead_mm = 0.0
    arena_overhang_mm = None


@dataclass(frozen=True)
class _State:
    x: int
    y: int
    h: int


def _heading_index(theta: float, headings=_HEADINGS) -> int:
    theta = math.atan2(math.sin(theta), math.cos(theta))
    return min(range(len(headings)), key=lambda i: abs(math.atan2(
        math.sin(theta - headings[i]), math.cos(theta - headings[i])
    )))


def _arc_delta(theta0: float, dir_sign: int, kappa_sign: int, phi: float, radius: float):
    # dir_sign: +1 fwd, -1 rev. kappa_sign: +1 left, -1 right.
    theta_f = theta0 + dir_sign * kappa_sign * phi
    dx = radius * kappa_sign * (math.sin(theta_f) - math.sin(theta0))
    dy = radius * kappa_sign * (math.cos(theta0) - math.cos(theta_f))
    return dx, dy, theta_f


def _straight_forward_token():
    return fwd(round(STEP_MM / 10.0))


def _straight_reverse_token():
    return rev(round(STEP_MM / 10.0))


_ARC_PRIMITIVES = [
    ("FL", +1, +1, lambda deg: arc(True, False, deg)),
    ("FR", +1, -1, lambda deg: arc(True, True, deg)),
    ("RL", -1, +1, lambda deg: arc(False, False, deg)),
    ("RR", -1, -1, lambda deg: arc(False, True, deg)),
]


def _obstacle_aabb_mm(obstacle: dict, virtual_half_mm: float) -> Tuple[float, float, float, float]:
    cx = obstacle["x"] * 100.0 + 50.0
    cy = obstacle["y"] * 100.0 + 50.0
    return (cx - virtual_half_mm, cy - virtual_half_mm, cx + virtual_half_mm, cy + virtual_half_mm)


def _point_blocked(x, y, theta, boxes, arena_mm, half_length_mm, half_width_mm) -> bool:
    """Whether the chassis at rear-axle pose (x, y, theta) is unsafe."""
    c, s = abs(math.cos(theta)), abs(math.sin(theta))
    # exact rotated-rect bounding half-extent, not just the 4 cardinal cases
    half_extent_x = half_length_mm * c + half_width_mm * s
    half_extent_y = half_length_mm * s + half_width_mm * c
    configured_overhang = getattr(boxes, "arena_overhang_mm", None)
    overhang = ARENA_OVERHANG_MM if configured_overhang is None else configured_overhang
    centre_ahead = float(getattr(boxes, "body_centre_ahead_mm", 0.0))
    body_x = x + centre_ahead * math.cos(theta)
    body_y = y + centre_ahead * math.sin(theta)
    lo_x, hi_x = half_extent_x - overhang, arena_mm - half_extent_x + overhang
    lo_y, hi_y = half_extent_y - overhang, arena_mm - half_extent_y + overhang
    if not (lo_x <= body_x <= hi_x and lo_y <= body_y <= hi_y):
        return True
    rb = getattr(boxes, "ref_bounds", None)
    if rb is not None and not (rb[0] <= x <= rb[1] and rb[2] <= y <= rb[3]):
        return True
    for box in boxes:
        if _rect_hits_box(body_x, body_y, theta, half_length_mm, half_width_mm,
                          half_extent_x, half_extent_y, box):
            return True
    return False


def _rect_hits_box(x, y, theta, hl, hw, ext_x, ext_y, box) -> bool:
    """Separating-axis test, rotated rectangle vs axis-aligned box."""
    xmin, ymin, xmax, ymax = box
    # World axes: the rectangle's bounding extents against the box.
    if x + ext_x <= xmin or x - ext_x >= xmax or y + ext_y <= ymin or y - ext_y >= ymax:
        return False
    # The rectangle's own axes: project the box onto them.
    bx, by = (xmin + xmax) / 2.0 - x, (ymin + ymax) / 2.0 - y
    bhx, bhy = (xmax - xmin) / 2.0, (ymax - ymin) / 2.0
    c, s = math.cos(theta), math.sin(theta)
    if abs(bx * c + by * s) >= hl + bhx * abs(c) + bhy * abs(s):
        return False
    if abs(-bx * s + by * c) >= hw + bhx * abs(s) + bhy * abs(c):
        return False
    return True


def _arc_clear(x0, y0, theta0, dir_sign, kappa_sign, radius, boxes, arena_mm,
                half_length_mm, half_width_mm, phi=TURN_STEP, samples=None) -> bool:
    if samples is None:
        # Keep the same 3.75-degree sampling density for every arc size.
        samples = max(1, int(math.ceil(ARC_SAMPLES * phi / TURN_STEP)))
    for i in range(1, samples + 1):
        sample_phi = phi * i / samples
        dx, dy, theta_i = _arc_delta(
            theta0, dir_sign, kappa_sign, sample_phi, radius
        )
        if _point_blocked(x0 + dx, y0 + dy, theta_i, boxes, arena_mm, half_length_mm, half_width_mm):
            return False
    return True


def _straight_clear(x0, y0, theta, forward, boxes, arena_mm,
                     half_length_mm, half_width_mm, samples=3) -> bool:
    sign = 1 if forward else -1
    for i in range(1, samples + 1):
        d = STEP_MM * i / samples
        x = x0 + sign * d * math.cos(theta)
        y = y0 + sign * d * math.sin(theta)
        if _point_blocked(x, y, theta, boxes, arena_mm, half_length_mm, half_width_mm):
            return False
    return True


def _neighbours(state: _State, radius_mm, boxes, arena_mm, half_length_mm,
                half_width_mm, headings=_HEADINGS,
                turn_steps_deg=(TURN_STEP_DEG,)):
    theta = headings[state.h]

    for forward in (True, False):
        if _straight_clear(state.x, state.y, theta, forward, boxes, arena_mm, half_length_mm, half_width_mm):
            sign = 1 if forward else -1
            nx = state.x + sign * STEP_MM * math.cos(theta)
            ny = state.y + sign * STEP_MM * math.sin(theta)
            try:
                token = _straight_forward_token() if forward else _straight_reverse_token()
            except TokenError:
                continue
            cost = STEP_MM * (1.0 if forward else REV_COST_MULT)
            yield _State(round(nx), round(ny), state.h), token, cost

    # Try larger safe arcs first. In open space one FR45 is preferable to three
    # FR15 commands; when its later sweep is blocked, FR15/FR30 expose an
    # intermediate heading where the search can change manoeuvre and the RPi can
    # take another pose/IR checkpoint.
    for degrees in sorted(set(turn_steps_deg), reverse=True):
        phi = math.radians(degrees)
        for name, dir_sign, kappa_sign, token_fn in _ARC_PRIMITIVES:
            if not _arc_clear(
                state.x, state.y, theta, dir_sign, kappa_sign, radius_mm,
                boxes, arena_mm, half_length_mm, half_width_mm, phi=phi,
            ):
                continue
            dx, dy, theta_f = _arc_delta(
                theta, dir_sign, kappa_sign, phi, radius_mm
            )
            nx, ny = state.x + dx, state.y + dy
            try:
                token = token_fn(degrees)
            except TokenError:
                continue
            arc_len = radius_mm * phi
            cost = (arc_len * (1.0 if dir_sign == 1 else REV_COST_MULT)
                    + TURN_PENALTY_MM * phi / (math.pi / 2))
            yield _State(
                round(nx), round(ny), _heading_index(theta_f, headings)
            ), token, cost


def _reconstruct(came_from, token_of, end: _State):
    tokens: List[str] = []
    states: List[_State] = [end]
    cur = end
    while came_from.get(cur) is not None:
        tokens.append(token_of[cur])
        cur = came_from[cur]
        states.append(cur)
    tokens.reverse()
    states.reverse()
    return tokens, states


@dataclass(frozen=True)
class ApproachLine:
    """Where a leg may end: anywhere on the straight line the robot will drive
    along, nose first, to reach its photo pose.

    (x, y, theta) is the photo pose itself. The line runs `length_mm` back from
    it, opposite to theta. The leg may stop anywhere on it; the planner closes
    the rest with an ordinary straight instruction, so long as it is within `lateral_tol_mm` of
    the line and already on the photo heading. That keeps the camera on the
    image, where a single goal point with a loose tolerance did not.
    """
    x: float
    y: float
    theta: float
    length_mm: float
    lateral_tol_mm: float
    reverse: bool = False

    def offsets(self, x: float, y: float) -> Tuple[float, float]:
        """(distance back from the photo pose along the line, sideways offset)."""
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(self.theta), math.sin(self.theta)
        along = dx * c + dy * s
        return (along if self.reverse else -along), -dx * s + dy * c

    def contains(self, state: "_State", headings=_HEADINGS) -> bool:
        if state.h != _heading_index(self.theta, headings):
            return False
        back, side = self.offsets(state.x, state.y)
        return -1.0 <= back <= self.length_mm and abs(side) <= self.lateral_tol_mm

    def distance(self, x: float, y: float) -> float:
        """Straight-line distance to the nearest point of the line (A* heuristic)."""
        back, side = self.offsets(x, y)
        back = min(max(back, 0.0), self.length_mm)
        direction = 1.0 if self.reverse else -1.0
        return math.hypot(
            x - (self.x + direction * back * math.cos(self.theta)),
            y - (self.y + direction * back * math.sin(self.theta)),
        )


def _astar(start_x, start_y, start_theta, goal: ApproachLine,
           radius_mm, boxes, arena_mm, half_length_mm, half_width_mm,
           forbidden_first_tokens=(), headings=_HEADINGS,
           turn_steps_deg=(TURN_STEP_DEG,), max_expansions=MAX_EXPANSIONS,
           deadline=None):
    start = _State(
        round(start_x), round(start_y), _heading_index(start_theta, headings)
    )
    forbidden_first = {str(token).strip().upper()
                       for token in forbidden_first_tokens}

    def cell(st: _State):
        return (round(st.x / DEDUP_CELL_MM), round(st.y / DEDUP_CELL_MM), st.h)

    frontier: List[Tuple[float, int, _State]] = []
    counter = 0
    heapq.heappush(frontier, (0.0, counter, start))
    came_from: Dict[_State, Optional[_State]] = {start: None}
    token_of: Dict[_State, str] = {}
    cost_so_far: Dict[_State, float] = {start: 0.0}
    best_in_cell: Dict[tuple, float] = {cell(start): 0.0}
    closed = set()

    expansions = 0
    while frontier:
        _, _, current = heapq.heappop(frontier)
        key = cell(current)
        if key in closed:
            continue
        closed.add(key)
        expansions += 1
        if expansions > max_expansions:
            return None
        if deadline is not None and time.monotonic() >= deadline:
            return None

        if goal.contains(current, headings):
            tokens, states = _reconstruct(came_from, token_of, current)
            return tokens, states, cost_so_far[current]

        for nxt, token, step_cost in _neighbours(
                current, radius_mm, boxes, arena_mm, half_length_mm,
                half_width_mm, headings, turn_steps_deg):
            if came_from[current] is None and token.upper() in forbidden_first:
                continue
            new_cost = cost_so_far[current] + step_cost
            nkey = cell(nxt)
            if nkey in closed or new_cost >= best_in_cell.get(nkey, math.inf):
                continue
            best_in_cell[nkey] = new_cost
            cost_so_far[nxt] = new_cost
            came_from[nxt] = current
            token_of[nxt] = token
            counter += 1
            heapq.heappush(frontier, (new_cost + HEURISTIC_WEIGHT * goal.distance(nxt.x, nxt.y),
                                      counter, nxt))

    return None


def search_leg(
    start_x: float, start_y: float, start_theta: float,
    goal: ApproachLine,
    radius_mm: float, boxes, arena_mm: float,
    half_length_mm: float, half_width_mm: float,
    forbidden_first_tokens=(),
    turn_steps_deg=(TURN_STEP_DEG,),
    max_expansions=MAX_EXPANSIONS,
    deadline=None,
) -> Tuple[List[str], List[Tuple[float, float, float]], float]:
    steps = tuple(sorted(set(int(value) for value in turn_steps_deg)))
    if not steps or any(value <= 0 or value % 15 for value in steps):
        raise ValueError("turn steps must be positive multiples of 15 degrees")
    fine = any(value % TURN_STEP_DEG for value in steps)
    headings = _FINE_HEADINGS if fine else _HEADINGS
    result = _astar(start_x, start_y, start_theta, goal,
                    radius_mm, boxes, arena_mm, half_length_mm, half_width_mm,
                    forbidden_first_tokens, headings, steps,
                    max_expansions, deadline)
    if result is None:
        raise NoPathFound(
            f"No path from ({start_x:.0f},{start_y:.0f}) to the approach line "
            f"of ({goal.x:.0f},{goal.y:.0f})."
        )
    tokens, states, cost = result
    poses = [(st.x, st.y, headings[st.h]) for st in states]
    return tokens, poses, cost
