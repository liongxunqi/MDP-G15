"""Grid pathfinding between two robot poses.

Motion model (own design, not the discrete-diagonal BFS from briefing
slide 41-42 — that one is explicitly for cheaply *ranking* which obstacle
is nearest, ignoring facing; this is the real, facing-aware motion the
robot actually executes):

  - FORWARD / BACKWARD: step one cell along/against the current facing.
  - FORWARD_LEFT / FORWARD_RIGHT / BACKWARD_LEFT / BACKWARD_RIGHT: turn 90
    degrees while covering ground, rather than the physically-impossible
    turn-on-the-spot (checklist item A.4 explicitly notes the physical
    robot cannot do that). A turn isn't a 1-cell pivot — the physical
    robot has a measured ~20cm turning radius, so a 90-degree turn
    actually sweeps `TURN_RADIUS_CELLS` (2) cells forward and 2 cells
    sideways, not 1 of each. We approximate the arc as two straight legs
    (drive the radius forward, then swing the radius sideways into the
    new heading) so both legs' footprints can be collision-checked, and
    so the simulator can animate the swept path instead of jumping
    straight to the end pose — an L-shaped stand-in for the true
    quarter-circle, in the same spirit as the rest of this module's
    discrete approximations.

Turn moves cost more than straight ones (`TURN_COST` below), scaled by
how much farther they actually travel, since a turning robot in real
life also travels slower — a simple, tunable stand-in for "time", which
is what the shortest-*time* Hamiltonian path (checklist B.3) is meant to
minimize.

Search is a standard Dijkstra/A* over the (x, y, facing) state space —
small enough (20*20*4 states) that plain Dijkstra would already be
instant; the Manhattan-distance heuristic just cuts the explored set down
further.
"""

import heapq
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from arena import Arena, RobotPose, delta, left_of, opposite, right_of

STRAIGHT_COST = 1.0
TURN_RADIUS_CELLS = 2    # measured ~20cm physical turning radius -> 2 cells
TURN_COST = STRAIGHT_COST * TURN_RADIUS_CELLS * 1.4


@dataclass
class Move:
    name: str
    waypoints_fn: Callable[[RobotPose], List[RobotPose]]
    cost: float


def _forward(pose: RobotPose) -> List[RobotPose]:
    dx, dy = delta(pose.facing)
    return [RobotPose(pose.x + dx, pose.y + dy, pose.facing)]


def _backward(pose: RobotPose) -> List[RobotPose]:
    dx, dy = delta(opposite(pose.facing))
    return [RobotPose(pose.x + dx, pose.y + dy, pose.facing)]


def _diagonal_turn(pose: RobotPose, going_forward: bool, turn_left: bool) -> List[RobotPose]:
    """Two waypoints approximating the turning-radius arc: drive
    `TURN_RADIUS_CELLS` forward (or backward) first, then swing the same
    distance sideways while rotating into the new heading. Both legs are
    returned so the caller can validate each one for collisions, rather
    than only the final pose — a 3-cell jump could otherwise clip an
    obstacle that a 1-cell jump never could."""
    r = TURN_RADIUS_CELLS
    step_facing = pose.facing if going_forward else opposite(pose.facing)
    dx1, dy1 = delta(step_facing)
    midpoint = RobotPose(pose.x + dx1 * r, pose.y + dy1 * r, pose.facing)

    lateral_facing = left_of(pose.facing) if turn_left else right_of(pose.facing)
    dx2, dy2 = delta(lateral_facing)
    new_facing = left_of(pose.facing) if turn_left else right_of(pose.facing)
    end = RobotPose(midpoint.x + dx2 * r, midpoint.y + dy2 * r, new_facing)

    return [midpoint, end]


MOVES: List[Move] = [
    Move("FORWARD", _forward, STRAIGHT_COST),
    Move("BACKWARD", _backward, STRAIGHT_COST),
    Move("FORWARD_LEFT", lambda p: _diagonal_turn(p, True, True), TURN_COST),
    Move("FORWARD_RIGHT", lambda p: _diagonal_turn(p, True, False), TURN_COST),
    Move("BACKWARD_LEFT", lambda p: _diagonal_turn(p, False, True), TURN_COST),
    Move("BACKWARD_RIGHT", lambda p: _diagonal_turn(p, False, False), TURN_COST),
]


def _heuristic(pose: RobotPose, goal: RobotPose) -> float:
    return (abs(pose.x - goal.x) + abs(pose.y - goal.y)) * STRAIGHT_COST


@dataclass
class PathResult:
    steps: List[Tuple[str, RobotPose]]  # [(move_name, pose_after_move), ...], "START" first
    cost: float                          # total move cost (turns cost more — see TURN_COST)


def find_path(arena: Arena, start: RobotPose, goal: RobotPose) -> Optional[PathResult]:
    """Cheapest (by move cost, i.e. shortest-*time*) sequence of moves from
    `start` to `goal` (exact pose match, including facing), or `None` if
    `goal` is unreachable."""
    if not arena.is_valid(start):
        return None

    open_heap: List[Tuple[float, float, Tuple[int, int, str]]] = [(0.0, 0.0, start.key())]
    g_score: Dict[Tuple[int, int, str], float] = {start.key(): 0.0}
    came_from: Dict[Tuple[int, int, str], Tuple[Tuple[int, int, str], str, List[RobotPose]]] = {}
    poses: Dict[Tuple[int, int, str], RobotPose] = {start.key(): start}
    closed = set()

    while open_heap:
        _, g, key = heapq.heappop(open_heap)
        if key in closed:
            continue
        closed.add(key)
        pose = poses[key]

        if pose == goal:
            return PathResult(_reconstruct(came_from, key, start), g)

        for move in MOVES:
            waypoints = move.waypoints_fn(pose)
            if not all(arena.is_valid(wp) for wp in waypoints):
                continue
            next_pose = waypoints[-1]
            next_key = next_pose.key()
            if next_key in closed:
                continue
            tentative_g = g + move.cost
            if tentative_g < g_score.get(next_key, float("inf")):
                g_score[next_key] = tentative_g
                came_from[next_key] = (key, move.name, waypoints)
                poses[next_key] = next_pose
                f = tentative_g + _heuristic(next_pose, goal)
                heapq.heappush(open_heap, (f, tentative_g, next_key))

    return None


def _reconstruct(came_from, key, start: RobotPose) -> List[Tuple[str, RobotPose]]:
    """Flattens each move's waypoints (2 legs for a turn, 1 for a straight
    move) into individual animation steps, so the simulator shows the
    turning-radius arc being swept out rather than jumping straight to the
    end pose."""
    path: List[Tuple[str, RobotPose]] = []
    while key in came_from:
        prev_key, move_name, waypoints = came_from[key]
        for wp in reversed(waypoints):
            path.append((move_name, wp))
        key = prev_key
    path.append(("START", start))
    return path[::-1]
