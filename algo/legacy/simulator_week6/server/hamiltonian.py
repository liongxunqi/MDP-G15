"""Shortest-time Hamiltonian path: visit every obstacle's viewpoint exactly
once, starting from the robot's start pose, minimizing total move cost.

Per the briefing (slide 16): with only a handful of obstacles, an
exhaustive search over every visiting order is affordable and guarantees
the true optimum — no need for the greedy nearest-neighbour heuristic
(slide 14) or its swap-based improvement (slide 15). We do exactly that:
score every pairwise leg once with `pathfinding.find_path`, then brute-force
every ordering and keep the cheapest.
"""

import time
from dataclasses import dataclass, field
from itertools import permutations
from typing import List, Optional, Tuple

from arena import Arena, RobotPose
from pathfinding import PathResult, find_path


@dataclass
class BlockedObstacle:
    id: int
    reason: str


@dataclass
class PlanResult:
    visit_order: List[int]              # obstacle ids, in visiting order
    legs: List[PathResult]              # one leg per consecutive stop
    total_cost: float
    runtime_seconds: float
    blocked: List[BlockedObstacle] = field(default_factory=list)  # only populated when no full tour was found


def plan_route(arena: Arena, start: RobotPose) -> PlanResult:
    """Always returns a `PlanResult` — if no obstacle is reachable at all,
    `visit_order`/`legs` come back empty rather than raising."""
    start_time = time.time()

    obstacles = arena.obstacles
    n = len(obstacles)
    if n == 0:
        return PlanResult([], [], 0.0, time.time() - start_time)

    viewpoints = {obs.id: arena.viewpoint_for(obs) for obs in obstacles}

    # Score every (from, to) pair once — "from" is either the start pose or
    # another obstacle's viewpoint, "to" is always an obstacle's viewpoint.
    stops: List[Tuple[Optional[int], RobotPose]] = [(None, start)] + [
        (obs.id, viewpoints[obs.id]) for obs in obstacles
    ]
    leg_cache: dict = {}
    for i, (_, from_pose) in enumerate(stops):
        for j, (to_id, to_pose) in enumerate(stops):
            if i == j or to_id is None:
                continue
            leg_cache[(i, j)] = find_path(arena, from_pose, to_pose)

    best_order: Optional[Tuple[int, ...]] = None
    best_cost = float("inf")
    best_legs: List[PathResult] = []

    obstacle_indices = list(range(1, len(stops)))  # index into `stops`, skipping start (index 0)
    for perm in permutations(obstacle_indices):
        total = 0.0
        legs: List[PathResult] = []
        prev = 0
        feasible = True
        for idx in perm:
            leg = leg_cache.get((prev, idx))
            if leg is None:
                feasible = False
                break
            total += leg.cost
            legs.append(leg)
            prev = idx
            if total >= best_cost:
                feasible = False
                break
        if feasible and total < best_cost:
            best_cost = total
            best_order = perm
            best_legs = legs

    runtime = time.time() - start_time
    if best_order is None:
        blocked = _diagnose_unreachable(arena, obstacles, viewpoints, leg_cache, stops)
        return PlanResult([], [], 0.0, runtime, blocked)

    visit_order = [stops[idx][0] for idx in best_order]
    return PlanResult(visit_order, best_legs, best_cost, runtime)


def _diagnose_unreachable(
    arena: Arena,
    obstacles: list,
    viewpoints: dict,
    leg_cache: dict,
    stops: List[Tuple[Optional[int], RobotPose]],
) -> List[BlockedObstacle]:
    """Only called once no complete tour was found — points at *why*, per
    obstacle, instead of leaving the caller with just an empty result and
    "0 of N obstacles reached" to puzzle over."""
    blocked: List[BlockedObstacle] = []
    for idx, obs in enumerate(obstacles, start=1):
        vp = viewpoints[obs.id]
        if not arena.in_bounds(vp):
            blocked.append(BlockedObstacle(obs.id, "its scanning spot falls outside the grid"))
            continue
        culprit = arena.blocking_obstacle(vp)
        if culprit is not None:
            blocked.append(BlockedObstacle(obs.id, f"its scanning spot overlaps obstacle #{culprit.id}"))
            continue
        reachable = any(leg_cache.get((i, idx)) is not None for i in range(len(stops)) if i != idx)
        if not reachable:
            blocked.append(BlockedObstacle(obs.id, "no path reaches its scanning spot — it's walled in by other obstacles"))
    return blocked
