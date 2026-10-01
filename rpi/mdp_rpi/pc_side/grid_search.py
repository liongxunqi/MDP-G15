"""Hybrid A* motion planner: grid + quarter-turn primitives, collision-checked."""

import heapq
import logging
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from stm_tokens import TokenError, arc, fwd, rev

STEP_MM = 50.0
GOAL_TOL_STAGES_MM = (40.0, 80.0, 150.0, 250.0)
REV_COST_MULT = 1.15
MAX_EXPANSIONS = 200_000

_HEADINGS = (0.0, math.pi / 2, math.pi, -math.pi / 2)  # E, N, W, S


class NoPathFound(Exception):
    pass


@dataclass(frozen=True)
class _State:
    x: int
    y: int
    h: int


def _heading_index(theta: float) -> int:
    theta = math.atan2(math.sin(theta), math.cos(theta))
    return min(range(4), key=lambda i: abs(math.atan2(
        math.sin(theta - _HEADINGS[i]), math.cos(theta - _HEADINGS[i])
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

_QUARTER_TURN = math.pi / 2


def _obstacle_aabb_mm(obstacle: dict, virtual_half_mm: float) -> Tuple[float, float, float, float]:
    cx = obstacle["x"] * 100.0 + 50.0
    cy = obstacle["y"] * 100.0 + 50.0
    return (cx - virtual_half_mm, cy - virtual_half_mm, cx + virtual_half_mm, cy + virtual_half_mm)


def _point_blocked(x, y, theta, boxes, arena_mm, half_length_mm, half_width_mm) -> bool:
    # exact rotated-rect bounding half-extent, not just the 4 cardinal cases
    half_extent_x = half_length_mm * abs(math.cos(theta)) + half_width_mm * abs(math.sin(theta))
    half_extent_y = half_length_mm * abs(math.sin(theta)) + half_width_mm * abs(math.cos(theta))
    if not (half_extent_x <= x <= arena_mm - half_extent_x
            and half_extent_y <= y <= arena_mm - half_extent_y):
        return True
    for xmin, ymin, xmax, ymax in boxes:
        if xmin <= x <= xmax and ymin <= y <= ymax:
            return True
    return False


def _arc_clear(x0, y0, theta0, dir_sign, kappa_sign, radius, boxes, arena_mm,
                half_length_mm, half_width_mm, samples=6) -> bool:
    for i in range(1, samples + 1):
        phi = _QUARTER_TURN * i / samples
        dx, dy, theta_i = _arc_delta(theta0, dir_sign, kappa_sign, phi, radius)
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


def _neighbours(state: _State, radius_mm, boxes, arena_mm, half_length_mm, half_width_mm):
    theta = _HEADINGS[state.h]

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

    for name, dir_sign, kappa_sign, token_fn in _ARC_PRIMITIVES:
        if _arc_clear(state.x, state.y, theta, dir_sign, kappa_sign, radius_mm, boxes, arena_mm, half_length_mm, half_width_mm):
            dx, dy, theta_f = _arc_delta(theta, dir_sign, kappa_sign, _QUARTER_TURN, radius_mm)
            nx, ny = state.x + dx, state.y + dy
            try:
                token = token_fn(90)
            except TokenError:
                continue
            arc_len = radius_mm * _QUARTER_TURN
            cost = arc_len * (1.0 if dir_sign == 1 else REV_COST_MULT)
            yield _State(round(nx), round(ny), _heading_index(theta_f)), token, cost


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


def _astar(start_x, start_y, start_theta, goal_x, goal_y, goal_theta,
           radius_mm, boxes, arena_mm, half_length_mm, half_width_mm, goal_tol_mm):
    start = _State(round(start_x), round(start_y), _heading_index(start_theta))
    goal_h = _heading_index(goal_theta)

    frontier: List[Tuple[float, int, _State]] = []
    counter = 0
    heapq.heappush(frontier, (0.0, counter, start))
    came_from: Dict[_State, Optional[_State]] = {start: None}
    token_of: Dict[_State, str] = {}
    cost_so_far: Dict[_State, float] = {start: 0.0}

    expansions = 0
    while frontier:
        _, _, current = heapq.heappop(frontier)
        expansions += 1
        if expansions > MAX_EXPANSIONS:
            return None

        if (math.hypot(current.x - goal_x, current.y - goal_y) <= goal_tol_mm
                and current.h == goal_h):
            tokens, states = _reconstruct(came_from, token_of, current)
            return tokens, states, cost_so_far[current]

        for nxt, token, step_cost in _neighbours(current, radius_mm, boxes, arena_mm, half_length_mm, half_width_mm):
            new_cost = cost_so_far[current] + step_cost
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + math.hypot(nxt.x - goal_x, nxt.y - goal_y)
                counter += 1
                heapq.heappush(frontier, (priority, counter, nxt))
                came_from[nxt] = current
                token_of[nxt] = token

    return None


def search_leg(
    start_x: float, start_y: float, start_theta: float,
    goal_x: float, goal_y: float, goal_theta: float,
    radius_mm: float, boxes, arena_mm: float,
    half_length_mm: float, half_width_mm: float,
) -> Tuple[List[str], List[Tuple[float, float, float]], float]:
    for stage, tol in enumerate(GOAL_TOL_STAGES_MM):
        result = _astar(start_x, start_y, start_theta, goal_x, goal_y, goal_theta,
            radius_mm, boxes, arena_mm, half_length_mm, half_width_mm, tol)
        if result is not None:
            tokens, states, cost = result
            if stage > 0:
                logging.warning(f"search_leg: relaxed tolerance to {tol}mm")
            poses = [(s.x, s.y, _HEADINGS[s.h]) for s in states]
            return tokens, poses, cost

    raise NoPathFound(
        f"No path from ({start_x:.0f},{start_y:.0f}) to "
        f"({goal_x:.0f},{goal_y:.0f}) at {GOAL_TOL_STAGES_MM[-1]}mm tolerance."
    )
