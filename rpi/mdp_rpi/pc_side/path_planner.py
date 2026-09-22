"""Task 1 path planning: visit order + motion.

Replaces compute_path()'s stub in task1_pc.py.

  1. Visit order — exhaustive permutation search over obstacles (<=7),
     scored by real A* path cost between viewing poses (grid_search.py).
     Falls back to nearest-neighbour + 2-opt above that.
  2. Viewing pose — for each obstacle, the (x, y, theta) the robot's
     centre must reach to photograph it: STANDOFF_MM out from the image
     face, facing the obstacle.
  3. Motion — obstacle-aware hybrid A* (grid_search.py) using this team's
     measured turn radii, converted straight into STM tokens and chunked
     per PROTOCOL.md §2.
  4. Post-photo — back away REV_AFTER_PHOTO_CM before planning the next leg,
     since the obstacle now blocks the robot.
  5. dirs[] — one {x,y,dir} entry per output line, from the exact states
     reached by the tokens that get sent.

dubins.py (closed-form Dubins solver) is no longer used here — an earlier
version of this planner used it directly, but it can't reverse mid-manoeuvre
and only warns about obstacle collisions instead of avoiding them. Kept
around as validated reference code.
"""

import itertools
import logging
import math
from typing import Dict, List, Optional, Tuple

from stm_tokens import (
    ARENA_CM,
    FU_MAX_CM,
    FU_MIN_CM,
    PROFILE_TIGHT,
    ROBOT_WIDTH_CM,
    TURN_RADIUS_MM,
    TokenError,
    chunk_tokens,
    fwd,
    fwd_until,
    rev,
    stop,
)

import grid_search

ARENA_MM = ARENA_CM * 10
OBSTACLE_SIZE_MM = 100.0

ROBOT_PLANNING_SIZE_MM = 300.0
ROBOT_PLANNING_HALF_MM = ROBOT_PLANNING_SIZE_MM / 2.0
VIRTUAL_OBSTACLE_HALF_MM = (OBSTACLE_SIZE_MM + ROBOT_PLANNING_SIZE_MM) / 2.0

STANDOFF_MM = 300.0
REV_AFTER_PHOTO_CM = 15

# Anchor the search this far short of the true standoff pose; the last bit
# is driven with fwd_until() on ultrasonic feedback instead of odometry.
STANDOFF_ANCHOR_BUFFER_MM = 150.0
FU_COMPENSATE = False

EXHAUSTIVE_LIMIT = 7

FACE_FROM_D = {0: "N", 2: "E", 4: "S", 6: "W"}

START_X_MM = 200.0
START_Y_MM = 200.0
START_THETA = math.pi / 2

ROBOT_MARGIN_MM = ROBOT_WIDTH_CM * 10 / 2.0


class Pose:
    __slots__ = ("x", "y", "theta")

    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, _wrap(theta)

    def __repr__(self):
        return f"Pose({self.x:.1f}, {self.y:.1f}, {math.degrees(self.theta):.1f}deg)"


def _wrap(theta):
    return math.atan2(math.sin(theta), math.cos(theta))


def _viewing_pose(obstacle: dict) -> Optional[Pose]:
    """Robot-centre pose to photograph this obstacle. None for SKIP (d=8)."""
    d = obstacle.get("d")
    if d not in FACE_FROM_D:
        return None
    face = FACE_FROM_D[d]

    cx = obstacle["x"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    cy = obstacle["y"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    half = OBSTACLE_SIZE_MM / 2.0
    dist_from_face = STANDOFF_MM + ROBOT_PLANNING_HALF_MM

    if face == "N":
        return Pose(cx, cy + half + dist_from_face, -math.pi / 2)
    if face == "S":
        return Pose(cx, cy - half - dist_from_face, math.pi / 2)
    if face == "E":
        return Pose(cx + half + dist_from_face, cy, math.pi)
    if face == "W":
        return Pose(cx - half - dist_from_face, cy, 0.0)
    return None


def _anchor_pose(obstacle: dict) -> Optional[Pose]:
    final = _viewing_pose(obstacle)
    if final is None:
        return None
    extra = STANDOFF_ANCHOR_BUFFER_MM
    return Pose(
        final.x - extra * math.cos(final.theta),
        final.y - extra * math.sin(final.theta),
        final.theta,
    )


def _obstacle_aabb_mm(obstacle: dict) -> Tuple[float, float, float, float]:
    cx = obstacle["x"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    cy = obstacle["y"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    return (
        cx - VIRTUAL_OBSTACLE_HALF_MM, cy - VIRTUAL_OBSTACLE_HALF_MM,
        cx + VIRTUAL_OBSTACLE_HALF_MM, cy + VIRTUAL_OBSTACLE_HALF_MM,
    )


def _obstacle_boxes(obstacles: List[dict], exclude_id) -> List[Tuple[float, float, float, float]]:
    return [_obstacle_aabb_mm(o) for o in obstacles if o.get("id") != exclude_id]


def _search(from_pose: Pose, to_pose: Pose, radius_mm, obstacles, exclude_id):
    boxes = _obstacle_boxes(obstacles, exclude_id)
    tokens, raw_poses, cost = grid_search.search_leg(
        from_pose.x, from_pose.y, from_pose.theta,
        to_pose.x, to_pose.y, to_pose.theta,
        radius_mm, boxes, ARENA_MM, ROBOT_MARGIN_MM,
    )
    poses = [Pose(x, y, t) for x, y, t in raw_poses]
    return tokens, poses, cost


def _build_edge_cache(start, visitable, anchor_by_id, radius_mm, obstacles):
    """Precompute every edge the TSP search could need, once, so exhaustive
    search over N obstacles costs N*(N+1) A* calls, not N!."""
    cache = {}
    from_nodes = [("START", start)]
    for obs in visitable:
        final = _viewing_pose(obs)
        bx = final.x - REV_AFTER_PHOTO_CM * 10.0 * math.cos(final.theta)
        by = final.y - REV_AFTER_PHOTO_CM * 10.0 * math.sin(final.theta)
        from_nodes.append((obs["id"], Pose(bx, by, final.theta)))

    for from_id, from_pose in from_nodes:
        for obs in visitable:
            if from_id == obs["id"]:
                continue
            anchor = anchor_by_id[obs["id"]]
            try:
                cache[(from_id, obs["id"])] = _search(from_pose, anchor, radius_mm, obstacles, obs["id"])
            except grid_search.NoPathFound as exc:
                logging.warning(f"TSP edge {from_id} -> obstacle {obs['id']}: {exc}")
                cache[(from_id, obs["id"])] = None
    return cache


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
    """
    Split the obstacles into those the robot can get to and those it cannot.

    An obstacle with no finite edge INTO it from anywhere — the start or any
    other obstacle — cannot appear in any tour, and leaving it in makes every
    permutation cost infinity. That is not a hypothetical: a block within
    about 65 cm of a wall puts its wall-facing approach outside the arena, and
    A.5 declares all four faces, so it hits one whenever the obstacle is near
    an edge.

    Dropping them is the honest outcome. Three faces is still a search; the
    one pointing into a wall was never photographable.
    """
    ids = [o["id"] for o in visitable]
    ok, dropped = [], []
    for obs in visitable:
        sources = ["START"] + [i for i in ids if i != obs["id"]]
        if any(math.isfinite(_edge_cost(cache, src, obs["id"])) for src in sources):
            ok.append(obs)
        else:
            dropped.append(obs)
    return ok, dropped


def _visit_order(visitable, cache):
    if len(visitable) <= 1:
        return list(visitable)

    if len(visitable) <= EXHAUSTIVE_LIMIT:
        best_order, best_cost = None, math.inf
        for perm in itertools.permutations(visitable):
            cost = _route_cost(list(perm), cache)
            if cost < best_cost:
                best_cost, best_order = cost, list(perm)

        if best_order is None:
            # Every permutation cost infinity, so no complete tour exists.
            # This used to fall straight through to the log line below and
            # raise TypeError on None — a crash rather than a plan, which on
            # the wire means the RPi waits for a PATH that never comes.
            #
            # Visiting some obstacles beats visiting none, so fall back to the
            # greedy order. The per-leg search in plan_mission() catches
            # NoPathFound and skips, so an unreachable leg is dropped there.
            logging.warning(
                "No finite tour over %d obstacle(s) — falling back to nearest-"
                "neighbour order and skipping whatever cannot be reached.",
                len(visitable),
            )
            return _greedy_order(visitable, cache)

        logging.info(f"Visit order (exhaustive): {[o['id'] for o in best_order]}  cost={best_cost:.0f}mm")
        return best_order

    return _greedy_order(visitable, cache)


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
    logging.info(f"Visit order (NN+2opt): {[o['id'] for o in order]}  cost={best_cost:.0f}mm")
    return order


def _nearest_cardinal(theta: float) -> str:
    deg = math.degrees(theta) % 360
    if deg >= 315 or deg < 45:
        return "E"
    if deg < 135:
        return "N"
    if deg < 225:
        return "W"
    return "S"


def _pose_to_dir_entry(pose: Pose) -> dict:
    gx = max(0, min(19, round(pose.x / 100.0)))
    gy = max(0, min(19, round(pose.y / 100.0)))
    return {"x": int(gx), "y": int(gy), "dir": _nearest_cardinal(pose.theta)}


def plan_mission(obstacles: List[dict], arc_profile: int = PROFILE_TIGHT) -> dict:
    radius_mm = TURN_RADIUS_MM[arc_profile]
    start = Pose(START_X_MM, START_Y_MM, START_THETA)

    anchor_by_id = {}
    final_by_id = {}
    visitable = []
    for obs in obstacles:
        final = _viewing_pose(obs)
        if final is None:
            logging.info(f"Obstacle {obs.get('id')}: SKIP (d={obs.get('d')}) — no viewing pose.")
            continue
        anchor = _anchor_pose(obs)
        for pose in (final, anchor):
            if not (0 <= pose.x <= ARENA_MM and 0 <= pose.y <= ARENA_MM):
                logging.warning(
                    f"Obstacle {obs.get('id')}: pose {pose} falls outside the arena "
                    "— too close to a wall for this standoff."
                )
        anchor_by_id[obs["id"]] = anchor
        final_by_id[obs["id"]] = final
        visitable.append(obs)

    edge_cache = _build_edge_cache(start, visitable, anchor_by_id, radius_mm, obstacles)

    # Drop anything nothing can reach before the tour is searched, rather than
    # letting it make every permutation infinite. See _reachable().
    visitable, unreachable = _reachable(visitable, edge_cache)
    for obs in unreachable:
        logging.error(
            f"Obstacle {obs.get('id')}: no approach exists from anywhere — "
            f"dropped. Its viewing pose is most likely outside the arena, "
            f"which happens when the obstacle sits within about 65cm of a wall "
            f"and the image face points at it."
        )

    if not visitable:
        logging.error(
            "No obstacle can be reached — returning an empty path rather than "
            "no path at all, so the caller gets an answer instead of waiting "
            "for one."
        )
        return {"segments": [], "obstacle_ids": [], "segment_obstacles": [], "dirs": []}

    order = _visit_order(visitable, edge_cache)

    segments: List[List[str]] = []
    segment_obstacles: List[Optional[str]] = []
    dirs: List[dict] = []

    fu_target_cm = round(STANDOFF_MM / 10.0)
    assert FU_MIN_CM <= fu_target_cm <= FU_MAX_CM

    cur = start
    cur_id = "START"
    for obs in order:
        anchor = anchor_by_id[obs["id"]]
        final = final_by_id[obs["id"]]

        cached = edge_cache.get((cur_id, obs["id"]))
        if cached is not None:
            tokens, poses, _ = cached
            tokens, poses = list(tokens), list(poses)
        else:
            try:
                tokens, poses, _ = _search(cur, anchor, radius_mm, obstacles, obs["id"])
            except grid_search.NoPathFound as exc:
                logging.error(f"Obstacle {obs['id']}: unreachable — {exc}. Skipping.")
                continue

        try:
            tokens.append(fwd_until(fu_target_cm, compensate=FU_COMPENSATE))
        except TokenError as exc:
            logging.error(f"fwd_until() rejected for obstacle {obs['id']}: {exc} — using fwd() instead.")
            tokens.append(fwd(round(STANDOFF_ANCHOR_BUFFER_MM / 10.0)))
        poses.append(Pose(final.x, final.y, final.theta))

        tokens.append(stop())
        poses.append(Pose(final.x, final.y, final.theta))

        lines = chunk_tokens(tokens)
        line_start = 0
        for i, line in enumerate(lines):
            segments.append(line)
            line_len = len(line)
            pose_after_line = poses[line_start + line_len - 1]
            dirs.append(_pose_to_dir_entry(pose_after_line))
            segment_obstacles.append(obs["id"] if i == len(lines) - 1 else None)
            line_start += line_len

        cur = Pose(final.x, final.y, final.theta)

        try:
            rev_tokens = [rev(REV_AFTER_PHOTO_CM), stop()]
        except TokenError as exc:
            logging.error(f"Could not build reverse-away tokens: {exc}")
            rev_tokens = []

        if rev_tokens:
            rev_lines = chunk_tokens(rev_tokens)
            bx = cur.x - REV_AFTER_PHOTO_CM * 10.0 * math.cos(cur.theta)
            by = cur.y - REV_AFTER_PHOTO_CM * 10.0 * math.sin(cur.theta)
            back_pose = Pose(bx, by, cur.theta)
            for line in rev_lines:
                segments.append(line)
                segment_obstacles.append(None)
                dirs.append(_pose_to_dir_entry(back_pose))
            cur = back_pose

        cur_id = obs["id"]

    return {
        "segments": segments,
        "obstacle_ids": [o["id"] for o in order],
        "segment_obstacles": segment_obstacles,
        "dirs": dirs,
    }
