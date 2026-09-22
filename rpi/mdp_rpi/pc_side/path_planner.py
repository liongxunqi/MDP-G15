"""Task 1 path planning: visit order + motion. Replaces compute_path()'s stub in task1_pc.py."""

import itertools
import logging
import math
from typing import Dict, List, Optional, Tuple

from stm_tokens import (
    ARENA_CM,
    FU_MAX_CM,
    FU_MIN_CM,
    PROFILE_TIGHT,
    ROBOT_LENGTH_CM,
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

STANDOFF_MM = 450.0
REV_AFTER_PHOTO_CM = 15
STANDOFF_ANCHOR_BUFFER_MM = 350.0
FU_COMPENSATE = False

EXHAUSTIVE_LIMIT = 7

FACE_FROM_D = {0: "N", 2: "E", 4: "S", 6: "W"}

# centre of the 40x40cm start zone, not the robot's bare minimum clearance
START_X_MM = 200.0
START_Y_MM = 200.0
START_THETA = math.pi / 2

ROBOT_HALF_LENGTH_MM = ROBOT_LENGTH_CM * 10 / 2.0
ROBOT_HALF_WIDTH_MM = ROBOT_WIDTH_CM * 10 / 2.0


class Pose:
    __slots__ = ("x", "y", "theta")

    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, _wrap(theta)

    def __repr__(self):
        return f"Pose({self.x:.1f}, {self.y:.1f}, {math.degrees(self.theta):.1f}deg)"


def _wrap(theta):
    return math.atan2(math.sin(theta), math.cos(theta))


def _viewing_pose(obstacle: dict, standoff_mm: float = STANDOFF_MM,
                   robot_half_mm: float = ROBOT_PLANNING_HALF_MM) -> Optional[Pose]:
    d = obstacle.get("d")
    if d not in FACE_FROM_D:
        return None
    face = FACE_FROM_D[d]

    cx = obstacle["x"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    cy = obstacle["y"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    half = OBSTACLE_SIZE_MM / 2.0
    dist_from_face = standoff_mm + robot_half_mm

    if face == "N":
        return Pose(cx, cy + half + dist_from_face, -math.pi / 2)
    if face == "S":
        return Pose(cx, cy - half - dist_from_face, math.pi / 2)
    if face == "E":
        return Pose(cx + half + dist_from_face, cy, math.pi)
    if face == "W":
        return Pose(cx - half - dist_from_face, cy, 0.0)
    return None


def _anchor_pose(obstacle: dict, standoff_mm: float = STANDOFF_MM,
                  buffer_mm: float = STANDOFF_ANCHOR_BUFFER_MM,
                  robot_half_mm: float = ROBOT_PLANNING_HALF_MM) -> Optional[Pose]:
    final = _viewing_pose(obstacle, standoff_mm, robot_half_mm)
    if final is None:
        return None
    return Pose(
        final.x - buffer_mm * math.cos(final.theta),
        final.y - buffer_mm * math.sin(final.theta),
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


def _pose_can_escape(pose: Pose, own_box, radius_mm: float) -> bool:
    state = grid_search._State(round(pose.x), round(pose.y), grid_search._heading_index(pose.theta))
    for _ in grid_search._neighbours(state, radius_mm, [own_box], ARENA_MM, ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM):
        return True
    return False


def _search(from_pose: Pose, to_pose: Pose, radius_mm, obstacles, exclude_id):
    boxes = _obstacle_boxes(obstacles, exclude_id)
    tokens, raw_poses, cost = grid_search.search_leg(
        from_pose.x, from_pose.y, from_pose.theta,
        to_pose.x, to_pose.y, to_pose.theta,
        radius_mm, boxes, ARENA_MM, ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM,
    )
    poses = [Pose(x, y, t) for x, y, t in raw_poses]
    return tokens, poses, cost


def _pose_in_bounds(pose: Pose) -> bool:
    return not grid_search._point_blocked(
        pose.x, pose.y, pose.theta, [], ARENA_MM, ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM,
    )


BACKAWAY_FALLBACKS_CM = (REV_AFTER_PHOTO_CM, 10, 5, 0)


def _feasible_backaway(final_pose: Pose, own_box, radius_mm: float) -> int:
    for cm in BACKAWAY_FALLBACKS_CM:
        if cm == 0:
            return 0
        bx = final_pose.x - cm * 10.0 * math.cos(final_pose.theta)
        by = final_pose.y - cm * 10.0 * math.sin(final_pose.theta)
        back_pose = Pose(bx, by, final_pose.theta)
        if _pose_in_bounds(back_pose) and _pose_can_escape(back_pose, own_box, radius_mm):
            return cm
    return 0


def _build_edge_cache(start, visitable, anchor_by_id, final_by_id, radius_mm, obstacles):
    cache = {}
    from_nodes = [("START", start)]
    for obs in visitable:
        final = final_by_id[obs["id"]]
        backaway_cm = _feasible_backaway(final, _obstacle_aabb_mm(obs), radius_mm)
        bx = final.x - backaway_cm * 10.0 * math.cos(final.theta)
        by = final.y - backaway_cm * 10.0 * math.sin(final.theta)
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
            logging.error("No feasible visit order — every obstacle unreachable.")
            return []
        logging.info(f"Visit order: {[o['id'] for o in best_order]}  cost={best_cost:.0f}mm")
        return best_order

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


STANDOFF_FALLBACKS_MM = (STANDOFF_MM, 250.0, 200.0, 150.0, 100.0, 70.0, FU_MIN_CM * 10.0)
ANCHOR_BUFFER_FALLBACKS_MM = (STANDOFF_ANCHOR_BUFFER_MM, 100.0, 50.0, 0.0)
ROBOT_HALF_FALLBACKS_MM = (ROBOT_PLANNING_HALF_MM, ROBOT_HALF_LENGTH_MM)


def _feasible_approach(obstacle: dict, radius_mm: float) -> Optional[Tuple[float, float, float]]:
    own_box = _obstacle_aabb_mm(obstacle)
    for robot_half_mm in ROBOT_HALF_FALLBACKS_MM:
        for standoff in STANDOFF_FALLBACKS_MM:
            final = _viewing_pose(obstacle, standoff, robot_half_mm)
            if final is None:
                return None
            if not _pose_in_bounds(final) or not _pose_can_escape(final, own_box, radius_mm):
                continue
            for buffer_mm in ANCHOR_BUFFER_FALLBACKS_MM:
                anchor = _anchor_pose(obstacle, standoff, buffer_mm, robot_half_mm)
                if _pose_in_bounds(anchor):
                    if (standoff, buffer_mm, robot_half_mm) != (STANDOFF_FALLBACKS_MM[0], ANCHOR_BUFFER_FALLBACKS_MM[0], ROBOT_HALF_FALLBACKS_MM[0]):
                        logging.warning(
                            f"Obstacle {obstacle.get('id')}: reduced to {standoff:.0f}mm "
                            f"standoff, {buffer_mm:.0f}mm buffer, {robot_half_mm:.0f}mm half-length."
                        )
                    return standoff, buffer_mm, robot_half_mm
    return None


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
    standoff_by_id = {}
    visitable = []
    for obs in obstacles:
        if obs.get("d") not in FACE_FROM_D:
            logging.info(f"Obstacle {obs.get('id')}: SKIP (d={obs.get('d')}).")
            continue
        approach = _feasible_approach(obs, radius_mm)
        if approach is None:
            logging.error(f"Obstacle {obs['id']}: no legal viewing pose — skipping.")
            continue
        standoff, buffer_mm, robot_half_mm = approach
        anchor_by_id[obs["id"]] = _anchor_pose(obs, standoff, buffer_mm, robot_half_mm)
        final_by_id[obs["id"]] = _viewing_pose(obs, standoff, robot_half_mm)
        standoff_by_id[obs["id"]] = standoff
        visitable.append(obs)

    edge_cache = _build_edge_cache(start, visitable, anchor_by_id, final_by_id, radius_mm, obstacles)
    order = _visit_order(visitable, edge_cache)

    segments: List[List[str]] = []
    segment_obstacles: List[Optional[str]] = []
    dirs: List[dict] = []

    cur = start
    cur_id = "START"
    for obs in order:
        anchor = anchor_by_id[obs["id"]]
        final = final_by_id[obs["id"]]
        fu_target_cm = round(standoff_by_id[obs["id"]] / 10.0)
        assert FU_MIN_CM <= fu_target_cm <= FU_MAX_CM

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
            segment_obstacles.append(str(obs["id"]) if i == len(lines) - 1 else None)
            line_start += line_len

        cur = Pose(final.x, final.y, final.theta)

        backaway_cm = _feasible_backaway(final, _obstacle_aabb_mm(obs), radius_mm)
        rev_tokens = []
        if backaway_cm > 0:
            try:
                rev_tokens = [rev(backaway_cm), stop()]
            except TokenError as exc:
                logging.error(f"Could not build reverse-away tokens: {exc}")
        else:
            logging.warning(f"Obstacle {obs['id']}: no room to back away — skipping.")

        if rev_tokens:
            rev_lines = chunk_tokens(rev_tokens)
            bx = cur.x - backaway_cm * 10.0 * math.cos(cur.theta)
            by = cur.y - backaway_cm * 10.0 * math.sin(cur.theta)
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
