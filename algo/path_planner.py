"""Task 1 path planning: visit order + motion. Replaces compute_path()'s stub in task1_pc.py.

Per obstacle the planner picks a photo pose: nose toward the image, the
ultrasonic reading the chosen standoff. Each leg drives onto the straight
approach line in front of that pose, then FU<n> closes the rest. Legs avoid
EVERY obstacle, including the one being approached, and the next leg starts
from where the photo was taken.
"""

import itertools
import logging
import math
import re
import time
from collections import namedtuple
from typing import Dict, List, Optional, Tuple

from stm_tokens import (
    ARENA_CM,
    FU_MAX_CM,
    FU_MIN_CM,
    PROFILE_TIGHT,
    ROBOT_LENGTH_CM,
    ROBOT_WIDTH_CM,
    TURN_RADIUS_MM,
    chunk_tokens,
    fwd,
    fwd_until,
    stop,
)

import grid_search

ARENA_MM = ARENA_CM * 10
OBSTACLE_SIZE_MM = 100.0

# Clearance kept between the robot's real outline and every obstacle, on top
# of the 23 x 18.8 cm footprint itself. Covers odometry drift and turn slop.
COLLISION_MARGIN_MM = 30.0

# Ultrasonic reading at the photo, cm - the FU<n> target - tried in this order.
# 30 is where YOLO is most confident; further out is next best; 28 works but
# its confidence is shaky, so it is only used when nothing else fits.
STANDOFF_PREFERENCE_CM = (30, 32, 35, 38, 40, 42, 45)
STANDOFF_LAST_RESORT_CM = (28,)
STANDOFF_CANDIDATES_CM = STANDOFF_PREFERENCE_CM + STANDOFF_LAST_RESORT_CM
FU_COMPENSATE = False

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

# centre of the 40x40cm start zone, not the robot's bare minimum clearance
START_X_MM = 200.0
START_Y_MM = 200.0
START_THETA = math.pi / 2

ROBOT_HALF_LENGTH_MM = ROBOT_LENGTH_CM * 10 / 2.0
ROBOT_HALF_WIDTH_MM = ROBOT_WIDTH_CM * 10 / 2.0

_STRAIGHT_RE = re.compile(r"^([FR])(\d+)$")


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
    """Robot centre when its front sensor reads standoff_mm to the image face."""
    d = obstacle.get("d")
    if d not in FACE_FROM_D:
        return None
    face = FACE_FROM_D[d]

    cx = obstacle["x"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    cy = obstacle["y"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    dist = OBSTACLE_SIZE_MM / 2.0 + standoff_mm + ROBOT_HALF_LENGTH_MM

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
        sx = pose.x + (ROBOT_HALF_LENGTH_MM - back_mm) * c - side * s
        sy = pose.y + (ROBOT_HALF_LENGTH_MM - back_mm) * s + side * c
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


def _merge_straights(tokens: List[str], poses: List[Pose]):
    """F5,F5,F5 -> F15. poses[i] is the pose AFTER tokens[i]."""
    out_t, out_p = [], []
    for tok, pose in zip(tokens, poses):
        m = _STRAIGHT_RE.match(tok)
        prev = _STRAIGHT_RE.match(out_t[-1]) if out_t else None
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

    def __init__(self, start, options, radius_mm, boxes, deadline):
        self.start, self.options = start, options
        self.radius_mm, self.boxes = radius_mm, boxes
        self.deadline = deadline
        self.legs = {}

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
                    self.legs[key] = _search_leg(
                        self._from_pose(from_id, choice),
                        self.options[to_id][choice[to_id]], self.radius_mm, self.boxes,
                    )
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


def _pose_to_dir_entry(pose: Pose) -> dict:
    gx = max(0, min(19, round(pose.x / 100.0)))
    gy = max(0, min(19, round(pose.y / 100.0)))
    return {"x": int(gx), "y": int(gy), "dir": _nearest_cardinal(pose.theta)}


def _choose_tour(visitable, legs: _Legs, options):
    """Visit order + standoff per obstacle. Starts every obstacle at its most
    preferred standoff; while the tour misses some, moves those (and the dead
    end that cut the tour short) to their next standoff, keeping the best."""
    deadline = legs.deadline
    choice = {o["id"]: 0 for o in visitable}
    best = None
    while True:
        cache = legs.view(visitable, choice)
        ok, unreachable = _reachable(visitable, cache)
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
        retry = order[reached:] + (order[reached - 1:reached] if reached else [])
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


def plan_mission(obstacles: List[dict], arc_profile: int = PROFILE_TIGHT,
                 details: Optional[dict] = None) -> dict:
    """Plan Task 1. Pass a dict as `details` to get each photo's pose and
    standoff back (tests and the simulator use it; the RPi does not)."""
    t0 = time.monotonic()
    radius_mm = TURN_RADIUS_MM[arc_profile]
    start = Pose(START_X_MM, START_Y_MM, START_THETA)
    # Every obstacle, the one being approached included: even the closest
    # photo pose (28 cm) is far outside its margin, so nothing needs excluding.
    boxes = [_obstacle_aabb_mm(o) for o in obstacles]

    options: Dict[object, List[PhotoOption]] = {}
    visitable = []
    skipped = {}
    for obs in obstacles:
        if obs.get("d") not in FACE_FROM_D:
            logging.info(f"Obstacle {obs.get('id')}: SKIP (d={obs.get('d')}).")
            continue
        opts = _photo_options(obs, boxes, obstacles)
        if not opts:
            logging.error(
                f"Obstacle {obs['id']}: no clear photo spot at "
                f"{min(STANDOFF_CANDIDATES_CM)}-{max(STANDOFF_CANDIDATES_CM)} cm — skipping."
            )
            skipped[obs["id"]] = "no photo spot"
            continue
        options[obs["id"]] = opts
        visitable.append(obs)

    legs = _Legs(start, options, radius_mm, boxes, t0 + PLAN_TIME_BUDGET_S)
    order, choice, cache = _choose_tour(visitable, legs, options)

    segments: List[List[str]] = []
    segment_obstacles: List[Optional[str]] = []
    dirs: List[dict] = []
    photo_poses, standoffs = {}, {}

    cur_id = "START"
    for obs in order:
        option = options[obs["id"]][choice[obs["id"]]]
        leg = cache.get((cur_id, obs["id"]))
        if leg is None:
            logging.error(f"Obstacle {obs['id']}: no route from {cur_id} — skipping.")
            skipped[obs["id"]] = "no route"
            continue

        search_tokens, search_poses, _ = leg
        tokens, poses = _merge_straights(list(search_tokens), list(search_poses[1:]))

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

    return {
        "segments": segments,
        "obstacle_ids": [o["id"] for o in order],
        "segment_obstacles": segment_obstacles,
        "dirs": dirs,
    }
