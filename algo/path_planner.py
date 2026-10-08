"""Task 1 path planning: visit order + motion. Replaces compute_path()'s stub in task1_pc.py.

Per obstacle the planner picks a photo pose: nose toward the image, the
ultrasonic reading the chosen standoff. Each leg drives onto the straight
approach line in front of that pose, then an ordinary F<n> closes the rest.
The RPi verifies the range with ?US and applies bounded F/R corrections. Legs avoid
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
    PROFILE_TIGHT,
    ROBOT_LENGTH_CM,
    ROBOT_WIDTH_CM,
    REAR_AXLE_TO_CAMERA_CM,
    REAR_AXLE_TO_SENSOR_CM,
    TURN_RADIUS_MM,
    US_BIAS_CM,
    chunk_tokens,
    fwd,
    rev,
    stop,
)

import grid_search

ARENA_MM = ARENA_CM * 10
OBSTACLE_SIZE_MM = 100.0

# Clearance kept between the robot's real outline and every obstacle, on top
# of the 23 x 18.8 cm footprint itself. Covers odometry drift and turn slop.
COLLISION_MARGIN_MM = 30.0

# Ultrasonic reading at the photo, cm, tried in this order.
# 30 is where YOLO is most confident; further out is next best. The camera is
# usable down to 20 cm, but that close pose is reserved for a face near an arena
# edge where the normal distances would put the rear axle beyond the boundary.
STANDOFF_PREFERENCE_CM = (30, 32, 33, 35, 38, 40, 42, 45)
STANDOFF_LAST_RESORT_CM = (28, 25, 20)
STANDOFF_CANDIDATES_CM = STANDOFF_PREFERENCE_CM + STANDOFF_LAST_RESORT_CM

# Closed-loop recovery. A replacement returns to the endpoint of the segment
# that was already being executed, then reuses the untouched later segments.
# The grid search may join this much of the endpoint's final straight line.
RECOVERY_APPROACH_LINE_MM = 600.0
RECOVERY_LATERAL_TOL_MM = 25.0
# Recovery may use one short, swept-footprint-checked alignment arc to reach the
# nearest 45-degree planner heading. Normal search remains on eight headings;
# only a failed local connection expands to the 24-heading fallback.
RECOVERY_MAX_HEADING_SNAP_DEG = float(os.getenv(
    "RECOVERY_MAX_HEADING_SNAP_DEG", "8"
))
# Second attempt when a recovery cannot be planned with the full margin. The
# robot is physically already inside the 30 mm envelope (odometry drift), and
# refusing to recover left the PC sending CONTINUE into the predicted collision.
# A route planned with a thinner margin is still safer than the old route.
RECOVERY_RELAXED_MARGIN_MM = float(os.getenv("RECOVERY_RELAXED_MARGIN_MM", "10"))

# Task 1 stays on the calibrated 45-degree heading lattice. The fine-lattice
# implementation remains available for isolated development tests, but mission
# planning does not opt into uncalibrated 15/30-degree arcs.
FINE_TURN_FALLBACK = False
FINE_TURN_STEPS_DEG = grid_search.FINE_TURN_STEPS_DEG
FINE_TURN_MAX_EXPANSIONS = int(os.getenv(
    "FINE_TURN_MAX_EXPANSIONS", "48000"
))

# How far back from the photo pose a leg may join the approach line, and how
# far off it sideways. Off-line by more than this and the camera misses the
# 10 cm image, or the ultrasonic locks onto a neighbouring obstacle.
APPROACH_LINE_MAX_MM = 600.0
APPROACH_LATERAL_TOL_MM = 25.0
# The Pi validates the final camera range with ?US. Check that the intended
# image remains the nearest echo over the last part of the approach rather
# than correcting toward a neighbouring obstacle.
US_VERIFY_RUNIN_MM = 150.0
SONAR_HALF_ANGLE_DEG = 10.0
CAMERA_AIM_TOL_DEG = 10.0

# Head-on remains mandatory whenever it fits. If it does not, an edge target
# may use either calibrated diagonal heading; both complete routes are checked.
EDGE_VIEW_ANGLES_DEG = (-45.0, 45.0)
EDGE_POSITION_BIAS_DEG = 8.0
EDGE_APPROACH_LATERAL_TOL_MM = 5.0
EDGE_LEG_SEARCH_BUDGET_S = 1.0
COMPACT_VIEW_DISTANCES_CM = (30, 25, 35)
# Per-stage A* slice for targets the tour skipped. 3 s was not enough on the
# robot laptop: obstacle 6 at (4,13) W was dropped there but found on a faster PC.
COMPACT_SEARCH_SLICE_S = float(os.getenv("COMPACT_SEARCH_SLICE_S", "5"))
# The RPi gives up on PATH after PATH_TIMEOUT_S (45 s). Stop trying other
# standoffs once this much time has gone and send the best plan so far.
# Planning can run to the assembly deadline, so PATH_TIMEOUT_S must stay well
# above it (leave ~10 s for debounce, transfer and a slow laptop).
# Cutting these instead cost a reachable obstacle on the 2026-10-09 layout.
PLAN_TIME_BUDGET_S = float(os.getenv("PLAN_TIME_BUDGET_S", "20"))
PLAN_ASSEMBLY_DEADLINE_S = float(os.getenv("PLAN_ASSEMBLY_DEADLINE_S", "35"))

EXHAUSTIVE_LIMIT = 8

FACE_FROM_D = {0: "N", 2: "E", 4: "S", 6: "W"}

ROBOT_HALF_LENGTH_MM = ROBOT_LENGTH_CM * 10 / 2.0
ROBOT_HALF_WIDTH_MM = ROBOT_WIDTH_CM * 10 / 2.0
# ?US measures from the front sensor, but the path and STM WPOSE both track
# the rear axle. This measured offset is therefore part of every photo pose.
REAR_AXLE_TO_SENSOR_MM = float(os.getenv(
    "REAR_AXLE_TO_SENSOR_CM", str(REAR_AXLE_TO_SENSOR_CM)
)) * 10.0
REAR_AXLE_TO_CAMERA_MM = float(os.getenv(
    "REAR_AXLE_TO_CAMERA_CM", str(REAR_AXLE_TO_CAMERA_CM)
)) * 10.0
# Camera standoff and ultrasonic range use different origins. Derive their
# separation from the two measured rear-axle offsets so they cannot drift.
CAMERA_TO_SENSOR_MM = REAR_AXLE_TO_SENSOR_MM - REAR_AXLE_TO_CAMERA_MM
if CAMERA_TO_SENSOR_MM < 0:
    raise ValueError("REAR_AXLE_TO_CAMERA_CM cannot exceed REAR_AXLE_TO_SENSOR_CM")
# The measured camera mount is approximately the geometric chassis centre.
# Motion and feedback remain rear-axle referenced; collision checks translate
# each sampled pose by this amount before testing the 23 x 18.8 cm rectangle.
BODY_CENTRE_AHEAD_MM = REAR_AXLE_TO_CAMERA_MM

# Start pose: robot facing N, pushed into the start zone's bottom-left corner -
# rear on the bottom line, left side on the left line, START_GAP_MM off each so
# nothing sits on the tape. Two edges are easy to line up by touch; the centre
# of a 40x40 cm box is not. START_X_MM / START_Y_MM in the PC's environment
# override the computed centre for fine-tuning on the day.
START_GAP_MM = float(os.getenv("START_GAP_MM", "10"))
# The first right arc needs 20 mm side clearance to keep the swept chassis
# inside x=0. Rear clearance can remain 10 mm because the robot first moves N.
START_SIDE_GAP_MM = float(os.getenv("START_SIDE_GAP_MM", "20"))
START_X_MM = float(os.getenv("START_X_MM", ROBOT_HALF_WIDTH_MM + START_SIDE_GAP_MM))
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
# Optional reported-coordinate guard. The normal Task 1 wire frame starts at
# (0,0), so this is off; the stricter full-chassis arena check remains active.
RPI_ARENA_GUARD = os.getenv("RPI_ARENA_GUARD", "0").strip().lower() not in ("0", "false", "no")
RPI_ARENA_MARGIN_MM = float(os.getenv("RPI_ARENA_MARGIN_MM", "50"))

# The leg that leaves START may not put the body outside the arena within this
# distance of the start, and may never reverse out of the start pose.
START_GUARD_MM = float(os.getenv("START_GUARD_MM", "700"))
# ...but a body 10 mm from the line still sweeps a few mm across it in a normal
# forward-right arc (6.8 mm at FR90), and a person places it to a few mm. Allow
# that much; a forward-left arc from the corner (302 mm out) stays forbidden.
START_GUARD_TOL_MM = float(os.getenv("START_GUARD_TOL_MM", "0"))
# When NO clean way out of the start exists (obstacles close by, robot against two
# lines), a leg that reverses or leaves the arena is used rather than returning
# an empty plan - but only if nothing clean exists for that obstacle, and it is
# charged this much so any clean alternative wins the tour.
START_RELAXED_PENALTY_MM = float(os.getenv("START_RELAXED_PENALTY_MM", "2500"))
_REAR_BLOCK_DEPTH_MM = 600.0     # how far behind the start pose the "no reverse" block reaches
_REAR_BLOCK_WIDTH_MM = 150.0     # ... and how far it extends past each side of the body
_BIG_MM = 10_000.0

_MERGEABLE_RE = re.compile(r"^(F|R)(\d+)$")
_ARC_RE = re.compile(r"^(FR|FL|RR|RL)(\d+)$")


class Pose:
    __slots__ = ("x", "y", "theta")

    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, _wrap(theta)

    def __repr__(self):
        return f"Pose({self.x:.1f}, {self.y:.1f}, {math.degrees(self.theta):.1f}deg)"


# us_runin_mm: how far before the photo pose the target remains the nearest
# ultrasonic echo. None means ?US cannot safely identify this target.
PhotoOption = namedtuple(
    "PhotoOption",
    "standoff_cm pose line us_runin_mm view_angle_deg approach_reverse",
)


def _collision_boxes(items=()):
    boxes = grid_search.Boxes(items)
    boxes.body_centre_ahead_mm = BODY_CENTRE_AHEAD_MM
    return boxes


def _wrap(theta):
    return math.atan2(math.sin(theta), math.cos(theta))


def _viewing_pose(obstacle: dict, standoff_mm: float,
                  view_angle_deg: float = 0.0) -> Optional[Pose]:
    """Rear-axle pose whose camera ray reaches the centre of the image face.

    view_angle_deg is measured from the outward face normal. Zero is head-on;
    +/-45 degrees are diagonal fallbacks supported by the motion lattice.
    """
    d = obstacle.get("d")
    if d not in FACE_FROM_D:
        return None
    face = FACE_FROM_D[d]

    cx = obstacle["x"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    cy = obstacle["y"] * 100.0 + OBSTACLE_SIZE_MM / 2.0
    outward = {
        "N": math.pi / 2,
        "S": -math.pi / 2,
        "E": 0.0,
        "W": math.pi,
    }[face]
    # Start the viewing ray at the centre of the selected face, not at the
    # obstacle centre. This keeps an oblique standoff equal to the real camera
    # distance from that face instead of shortening and laterally shifting it.
    face_x = cx + OBSTACLE_SIZE_MM / 2.0 * math.cos(outward)
    face_y = cy + OBSTACLE_SIZE_MM / 2.0 * math.sin(outward)
    heading_ray = outward + math.radians(view_angle_deg)
    # Keep an exactly calibrated diagonal chassis heading, but place the camera
    # slightly farther around the face. The target remains at the detector's
    # 10-degree aim limit while the body gains edge and run-in clearance.
    position_angle_deg = view_angle_deg
    if view_angle_deg:
        position_angle_deg += math.copysign(EDGE_POSITION_BIAS_DEG, view_angle_deg)
    position_ray = outward + math.radians(position_angle_deg)
    return Pose(
        face_x + standoff_mm * math.cos(position_ray)
        - REAR_AXLE_TO_CAMERA_MM * math.cos(heading_ray + math.pi),
        face_y + standoff_mm * math.sin(position_ray)
        - REAR_AXLE_TO_CAMERA_MM * math.sin(heading_ray + math.pi),
        heading_ray + math.pi,
    )


def _obstacle_aabb_mm(obstacle: dict,
                      margin_mm: float = None) -> Tuple[float, float, float, float]:
    """The obstacle grown by COLLISION_MARGIN_MM (or margin_mm) on every side."""
    margin = COLLISION_MARGIN_MM if margin_mm is None else margin_mm
    x0 = obstacle["x"] * 100.0 - margin
    y0 = obstacle["y"] * 100.0 - margin
    size = OBSTACLE_SIZE_MM + 2 * margin
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
    # Search poses are rear-axle positions; the footprint is centred ahead.
    hl, hw = ROBOT_HALF_LENGTH_MM - BODY_CENTRE_AHEAD_MM, ROBOT_HALF_WIDTH_MM
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
    if not hasattr(boxes, "body_centre_ahead_mm"):
        boxes = _collision_boxes(boxes)
    else:
        boxes.body_centre_ahead_mm = BODY_CENTRE_AHEAD_MM
    return not grid_search._point_blocked(
        x, y, theta, boxes, ARENA_MM, ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM,
    )


def _sonar_sees_target_first(pose: Pose, back_mm: float, standoff_mm: float,
                             others: List[dict]) -> bool:
    """From back_mm behind pose - anywhere across the approach line's width -
    is the image face the nearest echo in the cone?"""
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    squares = [(o["x"] * 100.0, o["y"] * 100.0) for o in others]
    sensor_standoff_mm = standoff_mm - CAMERA_TO_SENSOR_MM
    if sensor_standoff_mm <= 0:
        return False
    target_range = sensor_standoff_mm + back_mm
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


def _us_runin(pose: Pose, standoff_mm: float, others: List[dict]) -> Optional[float]:
    runin = None
    back = 0.0
    while back <= US_VERIFY_RUNIN_MM:
        if not _sonar_sees_target_first(pose, back, standoff_mm, others):
            break
        runin = back
        back += 25.0
    return runin


def _camera_view_clear(pose: Pose, obstacle: dict, others: List[dict]) -> bool:
    """The selected face is centred enough and no other block hides it.

    The camera is modelled at its measured mounting position. Sampling the segment
    to the image-face centre is deliberately conservative: a neighbouring
    obstacle in that narrow central view can otherwise be classified for the
    wrong obstacle ID.
    """
    face = FACE_FROM_D.get(obstacle.get("d"))
    if face is None:
        return False
    outward = {
        "N": math.pi / 2,
        "S": -math.pi / 2,
        "E": 0.0,
        "W": math.pi,
    }[face]
    target_x = (obstacle["x"] * 100.0 + OBSTACLE_SIZE_MM / 2.0 +
                OBSTACLE_SIZE_MM / 2.0 * math.cos(outward))
    target_y = (obstacle["y"] * 100.0 + OBSTACLE_SIZE_MM / 2.0 +
                OBSTACLE_SIZE_MM / 2.0 * math.sin(outward))
    camera_x = pose.x + REAR_AXLE_TO_CAMERA_MM * math.cos(pose.theta)
    camera_y = pose.y + REAR_AXLE_TO_CAMERA_MM * math.sin(pose.theta)
    dx, dy = target_x - camera_x, target_y - camera_y
    distance = math.hypot(dx, dy)
    if distance <= 0:
        return False
    target_heading = math.atan2(dy, dx)
    heading_error = abs(math.degrees(_wrap(target_heading - pose.theta)))
    if heading_error > CAMERA_AIM_TOL_DEG:
        return False

    steps = max(1, int(distance // 5.0))
    boxes = [
        (other["x"] * 100.0, other["y"] * 100.0,
         other["x"] * 100.0 + OBSTACLE_SIZE_MM,
         other["y"] * 100.0 + OBSTACLE_SIZE_MM)
        for other in others
    ]
    for i in range(1, steps):
        fraction = i / steps
        x = camera_x + dx * fraction
        y = camera_y + dy * fraction
        if any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in boxes):
            return False
    return True


def _photo_options(obstacle: dict, boxes, obstacles: List[dict],
                   allow_oblique: bool = False) -> List[PhotoOption]:
    """Every standoff in STANDOFF_CANDIDATES_CM whose photo pose is clear, in
    preference order, each with the stretch of approach line that is clear."""
    options = []
    angles = EDGE_VIEW_ANGLES_DEG if allow_oblique else (0.0,)
    for standoff_cm in STANDOFF_CANDIDATES_CM:
        for view_angle_deg in angles:
            pose = _viewing_pose(obstacle, standoff_cm * 10.0, view_angle_deg)
            if pose is None or not _pose_clear(pose.x, pose.y, pose.theta, boxes):
                continue
            others = [o for o in obstacles if o is not obstacle
                      and (o["x"], o["y"]) != (obstacle["x"], obstacle["y"])]
            # A* may finish slightly to either side of the approach line. Every
            # allowed offset must still have a clear, correctly aimed view.
            lateral_tolerance = (EDGE_APPROACH_LATERAL_TOL_MM if view_angle_deg
                                 else APPROACH_LATERAL_TOL_MM)
            if not all(
                _camera_view_clear(
                    Pose(
                        pose.x - side * math.sin(pose.theta),
                        pose.y + side * math.cos(pose.theta),
                        pose.theta,
                    ),
                    obstacle,
                    others,
                )
                for side in (-lateral_tolerance, 0.0, lateral_tolerance)
            ):
                continue
            approach_reverse = bool(view_angle_deg)
            approach_direction = 1.0 if approach_reverse else -1.0
            length = 0.0
            while length + 10.0 <= APPROACH_LINE_MAX_MM:
                back = length + 10.0
                if not _pose_clear(
                        pose.x + approach_direction * back * math.cos(pose.theta),
                        pose.y + approach_direction * back * math.sin(pose.theta),
                        pose.theta, boxes):
                    break
                length = back
            # A static diagonal endpoint is not enough: require room for at
            # least one full reverse grid step so the last arc finishes inboard.
            if approach_reverse and length < grid_search.STEP_MM:
                continue
            line = grid_search.ApproachLine(
                pose.x, pose.y, pose.theta, length, lateral_tolerance,
                reverse=approach_reverse,
            )
            # A flat face can reflect ultrasound away from the receiver at an
            # oblique camera angle. Keep the collision-checked view, but mark it
            # odometry-only so the RPi never chases a wall echo with F/R.
            runin = (None if view_angle_deg else
                     _us_runin(pose, standoff_cm * 10.0, others))
            options.append(PhotoOption(
                standoff_cm, pose, line, runin, view_angle_deg,
                approach_reverse,
            ))
    return options


def _outward_face_clearance_mm(obstacle: dict) -> float:
    """Distance from the image plane to the arena edge it faces."""
    face = FACE_FROM_D.get(obstacle.get("d"))
    if face == "N":
        return ARENA_MM - (obstacle["y"] + 1.0) * 100.0
    if face == "S":
        return obstacle["y"] * 100.0
    if face == "E":
        return ARENA_MM - (obstacle["x"] + 1.0) * 100.0
    if face == "W":
        return obstacle["x"] * 100.0
    return math.inf


def _on_coarse_heading(theta: float) -> bool:
    units = (math.degrees(theta) % 360.0) / grid_search.TURN_STEP_DEG
    return abs(units - round(units)) < 1e-6


def _search_motion(start_x, start_y, start_theta, goal, radius_mm, boxes,
                   forbidden_first_tokens=(), deadline=None):
    """Use the fast 45-degree lattice first, then a bounded fine fallback."""
    fine_required = (not _on_coarse_heading(start_theta) or
                     not _on_coarse_heading(goal.theta))
    if not fine_required:
        try:
            return grid_search.search_leg(
                start_x, start_y, start_theta, goal, radius_mm, boxes, ARENA_MM,
                ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM,
                forbidden_first_tokens=forbidden_first_tokens,
                deadline=deadline,
            )
        except grid_search.NoPathFound:
            if (not FINE_TURN_FALLBACK or
                    (deadline is not None and time.monotonic() >= deadline)):
                raise
    elif not FINE_TURN_FALLBACK:
        raise grid_search.NoPathFound(
            "A non-45-degree endpoint requires the fine turn lattice."
        )

    return grid_search.search_leg(
        start_x, start_y, start_theta, goal, radius_mm, boxes, ARENA_MM,
        ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM,
        forbidden_first_tokens=forbidden_first_tokens,
        turn_steps_deg=FINE_TURN_STEPS_DEG,
        max_expansions=FINE_TURN_MAX_EXPANSIONS,
        deadline=deadline,
    )


def _uses_fine_turn(tokens: List[str]) -> bool:
    for token in tokens:
        match = re.fullmatch(r"(?:FR|FL|RR|RL)(\d+)", token.upper())
        if match is not None and int(match.group(1)) % grid_search.TURN_STEP_DEG:
            return True
    return False


def _search_leg(from_pose: Pose, option: PhotoOption, radius_mm, boxes,
                deadline=None):
    """(tokens, poses, cost) onto option's approach line, or None. Cost
    includes the straight run-in along the line, so legs compare fairly."""
    if option.view_angle_deg:
        edge_deadline = time.monotonic() + EDGE_LEG_SEARCH_BUDGET_S
        deadline = edge_deadline if deadline is None else min(deadline, edge_deadline)
    try:
        tokens, raw_poses, cost = _search_motion(
            from_pose.x, from_pose.y, from_pose.theta, option.line,
            radius_mm, boxes, deadline=deadline,
        )
    except grid_search.NoPathFound:
        return None
    poses = _replay_motion(from_pose, tokens, radius_mm, boxes)
    if poses is None:
        return None

    back, _ = option.line.offsets(poses[-1].x, poses[-1].y)
    if not (-1.0 <= back <= option.line.length_mm):
        return None
    _, side = option.line.offsets(poses[-1].x, poses[-1].y)
    if abs(side) > option.line.lateral_tol_mm:
        return None
    return tokens, poses, cost + max(back, 0.0)


def _replay_motion(from_pose: Pose, tokens: List[str], radius_mm, boxes):
    """Replay emitted tokens continuously and reject any unsafe sweep."""
    poses = [Pose(from_pose.x, from_pose.y, from_pose.theta)]
    current = poses[0]
    for token in tokens:
        match = re.fullmatch(r"(FR|FL|RR|RL|F|R)(\d+)", token.upper())
        if match is None:
            return None
        op, magnitude_text = match.groups()
        magnitude = int(magnitude_text)
        if op in ("F", "R"):
            distance = magnitude * 10.0 * (1.0 if op == "F" else -1.0)
            count = max(1, int(math.ceil(abs(distance) / 10.0)))
            samples = [
                Pose(
                    current.x + distance * i / count * math.cos(current.theta),
                    current.y + distance * i / count * math.sin(current.theta),
                    current.theta,
                )
                for i in range(1, count + 1)
            ]
        else:
            direction = 1 if op[0] == "F" else -1
            curvature = 1 if op[1] == "L" else -1
            total = math.radians(magnitude)
            count = max(1, int(math.ceil(magnitude / 2.0)))
            samples = []
            for i in range(1, count + 1):
                dx, dy, theta = grid_search._arc_delta(
                    current.theta, direction, curvature,
                    total * i / count, radius_mm,
                )
                samples.append(Pose(current.x + dx, current.y + dy, theta))
        if any(not _pose_clear(sample.x, sample.y, sample.theta, boxes)
               for sample in samples):
            return None
        current = samples[-1]
        poses.append(current)
    return poses


class _MultiApproachGoal:
    """A* goal that accepts the approach line of any candidate target."""

    def __init__(self, candidates):
        self.candidates = candidates
        first = candidates[0][1].line
        # _search_motion only uses theta to choose the calibrated heading lattice.
        self.x, self.y, self.theta = first.x, first.y, first.theta

    def contains(self, state, headings):
        return any(option.line.contains(state, headings)
                   for _, option in self.candidates)

    def distance(self, x, y):
        return min(option.line.distance(x, y)
                   for _, option in self.candidates)


def _option_reached(pose: Pose, candidates):
    heading = grid_search._heading_index(pose.theta)
    for obstacle, option in candidates:
        if heading != grid_search._heading_index(option.line.theta):
            continue
        back, side = option.line.offsets(pose.x, pose.y)
        if (-1.0 <= back <= option.line.length_mm and
                abs(side) <= option.line.lateral_tol_mm):
            return obstacle, option
    return None


def _search_any_approach(from_pose: Pose, candidates, radius_mm, boxes,
                         deadline):
    """Reach one of several camera approach lines with a single A* run."""
    if not candidates:
        return None
    goal = _MultiApproachGoal(candidates)
    try:
        tokens, _, cost = _search_motion(
            from_pose.x, from_pose.y, from_pose.theta, goal,
            radius_mm, boxes, deadline=deadline,
        )
    except grid_search.NoPathFound:
        return None
    poses = _replay_motion(from_pose, tokens, radius_mm, boxes)
    if poses is None:
        return None
    reached = _option_reached(poses[-1], candidates)
    if reached is None:
        return None
    obstacle, option = reached
    back, _ = option.line.offsets(poses[-1].x, poses[-1].y)
    return obstacle, option, (tokens, poses, cost + max(back, 0.0))


def _merge_runs(tokens: List[str], poses: List[Pose]):
    """Merge compatible straight runs and adjacent arcs up to 90 degrees."""
    out_t, out_p = [], []
    for tok, pose in zip(tokens, poses):
        m = _MERGEABLE_RE.match(tok)
        prev = _MERGEABLE_RE.match(out_t[-1]) if out_t else None
        arc_match = _ARC_RE.match(tok)
        prev_arc = _ARC_RE.match(out_t[-1]) if out_t else None
        if m and prev and m.group(1) == prev.group(1):
            out_t[-1] = f"{m.group(1)}{int(prev.group(2)) + int(m.group(2))}"
            out_p[-1] = pose
        elif (arc_match and prev_arc and
              arc_match.group(1) == prev_arc.group(1) and
              int(arc_match.group(2)) + int(prev_arc.group(2)) <= 90):
            out_t[-1] = (
                f"{arc_match.group(1)}"
                f"{int(prev_arc.group(2)) + int(arc_match.group(2))}"
            )
            out_p[-1] = pose
        else:
            out_t.append(tok)
            out_p.append(pose)
    return out_t, out_p


def _contains_fu(segments: List[List[str]]) -> bool:
    """Task 1 contract: neither initial nor recovery paths may contain FU."""
    return any(
        isinstance(token, str) and token.strip().upper().startswith("FU")
        for segment in segments for token in segment
    )


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
            return _search_leg(
                from_pose, option, self.radius_mm, self.boxes,
                deadline=self.deadline,
            )
        leg = _search_leg(
            from_pose, option, self.radius_mm, self.start_boxes,
            deadline=self.deadline,
        )
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
# Three camera angles are considered at each range. Try three complete range
# bands before declaring a target unreachable; the previous value of 3 would
# now test only 30 cm (head-on, +45, -45) and never reach another distance.
MAX_DEAD_OPTION_TRIES = 9


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


def _compact_diagonal_candidates(options):
    """One bounded set of useful distances on each side of a known face."""
    selected = []
    for angle in EDGE_VIEW_ANGLES_DEG:
        side = [option for option in options
                if option.view_angle_deg == angle]
        for distance in COMPACT_VIEW_DISTANCES_CM:
            option = next((item for item in side
                           if item.standoff_cm == distance), None)
            if option is not None:
                selected.append(option)
    return selected


def _terminal_after_leg(leg, option):
    """Continuous pose after the rounded final straight camera run-in."""
    _, poses, _ = leg
    last = poses[-1]
    back, _ = option.line.offsets(last.x, last.y)
    run_cm = round(back / 10.0)
    if run_cm <= 0:
        return Pose(last.x, last.y, option.pose.theta)
    direction = -1.0 if option.approach_reverse else 1.0
    run_mm = run_cm * 10.0
    return Pose(
        last.x + direction * run_mm * math.cos(option.pose.theta),
        last.y + direction * run_mm * math.sin(option.pose.theta),
        option.pose.theta,
    )


def _plan_compact_sequence(start, visitable, head_options, diagonal_options,
                           radius_mm, boxes, start_boxes, deadline):
    """Greedily reach any safe next face with one A* search per step.

    Normal head-on poses remain the first goal set. Only when none of those is
    reachable within the bounded slice does the search expose diagonal poses.
    Distances beyond 25/30/35 cm are left to local camera retry handling.
    """
    remaining = list(visitable)
    sequence = []
    current = start
    first_leg = True
    best_sequence = []
    banned_next = {}
    visit_index = {obstacle["id"]: index
                   for index, obstacle in enumerate(visitable)}
    while remaining and time.monotonic() < deadline:
        prefix = tuple(item[0]["id"] for item in sequence)
        banned = banned_next.get(prefix, set())
        primary = []
        alternate_heads = []
        fallback = []
        for obstacle in remaining:
            if obstacle["id"] in banned:
                continue
            heads = head_options.get(obstacle["id"], [])
            if heads:
                primary.append((obstacle, heads[0]))
                alternate_heads.extend((obstacle, option) for option in heads[1:])
            else:
                primary.extend(
                    (obstacle, option)
                    for option in _compact_diagonal_candidates(
                        diagonal_options.get(obstacle["id"], [])
                    )
                )
            fallback.extend(
                (obstacle, option)
                for option in _compact_diagonal_candidates(
                    diagonal_options.get(obstacle["id"], [])
                )
            )

        found = None
        search_boxes = start_boxes if first_leg else boxes
        # Targets with no head-on pose need diagonals immediately. For the
        # others, try alternate head-on distances before their diagonal views.
        stages = [(candidates, limit) for candidates, limit in
                  ((primary, COMPACT_SEARCH_SLICE_S), (alternate_heads, 0.5),
                   (fallback, COMPACT_SEARCH_SLICE_S)) if candidates]
        for stage_index, (candidates, search_slice) in enumerate(stages):
            if not candidates or time.monotonic() >= deadline:
                continue
            # Reserve time for diagonal candidates when head-on searches are
            # difficult; otherwise a valid edge view can be starved entirely.
            remaining_time = deadline - time.monotonic()
            search_slice = min(search_slice, remaining_time / (len(stages) - stage_index))
            found = _search_any_approach(
                current, candidates, radius_mm, search_boxes,
                min(deadline, time.monotonic() + search_slice),
            )
            if found is not None:
                break
        relaxed_start = False
        if found is None:
            if not sequence:
                break
            removed = sequence.pop()
            removed_obstacle = removed[0]
            previous_prefix = tuple(item[0]["id"] for item in sequence)
            banned_next.setdefault(previous_prefix, set()).add(
                removed_obstacle["id"]
            )
            remaining.append(removed_obstacle)
            remaining.sort(key=lambda item: visit_index[item["id"]])
            current = (start if not sequence else
                       _terminal_after_leg(sequence[-1][2], sequence[-1][1]))
            first_leg = not sequence
            logging.info(
                "Compact route backtracked from obstacle %s to try a different "
                "visit order.", removed_obstacle["id"],
            )
            continue

        obstacle, option, leg = found
        sequence.append((obstacle, option, leg, relaxed_start))
        if len(sequence) > len(best_sequence):
            best_sequence = list(sequence)
        remaining.remove(obstacle)
        current = _terminal_after_leg(leg, option)
        first_leg = False

    # Backtracking may be interrupted by the deadline. Keep the longest valid
    # route already found instead of returning the partially unwound stack.
    if len(best_sequence) > len(sequence):
        sequence = best_sequence
        visited = {item[0]["id"] for item in sequence}
        remaining = [obs for obs in visitable if obs["id"] not in visited]
    return sequence, remaining


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


def assess_primitive_safety(plan: dict, obstacles: List[dict], progress: dict,
                            token: str, arc_profile: int = PROFILE_TIGHT,
                            margin_mm: float = None):
    """Check a primitive from the measured pose using the planner's full body.

    Returns None when every sampled footprint is clear, otherwise a diagnostic
    dict suitable for PlanMonitor. Unlike the old endpoint-only guard, this
    checks the complete 45/90-degree sweep and uses PATH.frame_shift_mm before
    applying the configured open-floor overhang.
    """
    try:
        wire_x = float(progress["x_grid"]) * 100.0
        wire_y = float(progress["y_grid"]) * 100.0
        bearing = float(progress["heading_deg"]) % 360.0
        shift_x, shift_y = _plan_frame_shift(plan)
        x, y = wire_x - shift_x, wire_y - shift_y
        theta = math.radians(90.0 - bearing)
        command = str(token).strip().upper()
    except (KeyError, TypeError, ValueError):
        return {"reason": "invalid measured pose", "next_token": str(token)}

    boxes = _collision_boxes([_obstacle_aabb_mm(o, margin_mm) for o in obstacles])
    if RPI_ARENA_GUARD:
        boxes.ref_bounds = _rpi_ref_bounds(Pose(
            float(plan.get("start_mm", {}).get("x", START_X_MM)),
            float(plan.get("start_mm", {}).get("y", START_Y_MM)),
            START_THETA,
        ))

    samples = [(x, y, theta)]
    match = re.fullmatch(r"(FR|FL|RR|RL|F|R)(\d+)", command)
    if command == "S":
        return None
    if match is None:
        return {"reason": "unsupported primitive", "next_token": command}

    op, magnitude_text = match.groups()
    magnitude = int(magnitude_text)
    if op in ("F", "R"):
        distance = magnitude * 10.0 * (1.0 if op == "F" else -1.0)
        count = max(1, int(math.ceil(abs(distance) / 20.0)))
        samples.extend(
            (x + distance * i / count * math.cos(theta),
             y + distance * i / count * math.sin(theta), theta)
            for i in range(1, count + 1)
        )
    else:
        direction = 1 if op[0] == "F" else -1
        curvature = 1 if op[1] == "L" else -1
        total = math.radians(magnitude)
        count = max(1, int(math.ceil(magnitude / 2.0)))
        radius = TURN_RADIUS_MM[arc_profile]
        for i in range(1, count + 1):
            dx, dy, sample_theta = grid_search._arc_delta(
                theta, direction, curvature, total * i / count, radius
            )
            samples.append((x + dx, y + dy, sample_theta))

    for sample_x, sample_y, sample_theta in samples:
        if grid_search._point_blocked(
            sample_x, sample_y, sample_theta, boxes, ARENA_MM,
            ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM,
        ):
            return {
                "reason": "swept footprint intersects obstacle/arena envelope",
                "next_token": command,
                "x_mm": round(sample_x + shift_x, 1),
                "y_mm": round(sample_y + shift_y, 1),
                "heading_deg": round(
                    (90.0 - math.degrees(sample_theta)) % 360.0, 1
                ),
                "arena_overhang_mm": grid_search.ARENA_OVERHANG_MM,
            }
    return None


def plan_segment_recovery(plan: dict, obstacles: List[dict], progress: dict,
                          arc_profile: int = PROFILE_TIGHT) -> Optional[dict]:
    """Recover with the full collision margin, then with a thinner one."""
    replacement = _plan_segment_recovery(
        plan, obstacles, progress, arc_profile, COLLISION_MARGIN_MM,
    )
    if replacement is None and RECOVERY_RELAXED_MARGIN_MM < COLLISION_MARGIN_MM:
        logging.warning(
            "Retrying recovery with a %.0f mm obstacle margin instead of %.0f mm.",
            RECOVERY_RELAXED_MARGIN_MM, COLLISION_MARGIN_MM,
        )
        replacement = _plan_segment_recovery(
            plan, obstacles, progress, arc_profile, RECOVERY_RELAXED_MARGIN_MM,
        )
    return replacement


def _plan_segment_recovery(plan: dict, obstacles: List[dict], progress: dict,
                           arc_profile: int, margin_mm: float) -> Optional[dict]:
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
    if _contains_fu(segments):
        logging.error("Cannot build recovery from a legacy Task 1 path containing FU.")
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
    boxes = _collision_boxes([_obstacle_aabb_mm(o, margin_mm) for o in obstacles])
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
    blocked = str(progress.get("blocked_token", "")).strip().upper()
    blocked_match = re.fullmatch(r"(FR|FL|RR|RL|F|R)(\d+)", blocked)
    forbidden_first = set()
    recovery_reason = str(progress.get("recovery_reason", ""))
    if recovery_reason.startswith("IR "):
        # A close side return does not identify one exact steering primitive.
        # Force one straight clearing move before any arc; the RPi will sample
        # IR again after that move and may permit or replan the turn then.
        forbidden_first.update(
            f"{op}{grid_search.TURN_STEP_DEG}"
            for op in ("FR", "FL", "RR", "RL")
        )
    elif blocked_match:
        blocked_op = blocked_match.group(1)
        if blocked_op in ("FR", "FL", "RR", "RL"):
            forbidden_first.add(f"{blocked_op}{grid_search.TURN_STEP_DEG}")
        elif blocked_op == "F":
            forbidden_first.add(grid_search._straight_forward_token())
        else:
            forbidden_first.add(grid_search._straight_reverse_token())

    # Most measured drift is longitudinal. Correct that directly instead of
    # making the 50 mm search grid overshoot and then drive back a few centimetres.
    dx, dy = tx - ax, ty - ay
    along = dx * math.cos(target_theta) + dy * math.sin(target_theta)
    lateral = -dx * math.sin(target_theta) + dy * math.cos(target_theta)
    same_heading = grid_search._heading_index(actual_lattice) == grid_search._heading_index(target_theta)
    direct_cm = int(round(abs(along) / 10.0))
    direct_mm = math.copysign(direct_cm * 10.0, along) if direct_cm else 0.0
    direct_op_blocked = (
        (direct_mm > 0 and blocked_match and blocked_match.group(1) == "F") or
        (direct_mm < 0 and blocked_match and blocked_match.group(1) == "R")
    )
    if (same_heading and abs(lateral) <= RECOVERY_LATERAL_TOL_MM and direct_cm and
            not direct_op_blocked and
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
                    forbidden_first_tokens=forbidden_first,
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
    if _contains_fu(replacement["segments"]):
        logging.error("Recovery planner produced a forbidden FU token; refusing replacement.")
        return None
    logging.warning(
        "Recovery for segment %d: measured (%.0f,%.0f) @ %.1f deg -> original endpoint "
        "(%.0f,%.0f) @ %.1f deg using %s; preserving %d later segment(s).",
        seg, actual_x, actual_y, actual_hdg, target_x, target_y, target_hdg,
        ",".join(tokens), len(segments) - seg - 1,
    )
    return replacement


def plan_mission(obstacles: List[dict], arc_profile: int = PROFILE_TIGHT,
                 details: Optional[dict] = None, *,
                 start_pose: Optional["Pose"] = None,
                 frame_shift: Optional[Tuple[float, float]] = None,
                 target_ids=None,
                 time_budget_s: Optional[float] = None,
                 assembly_deadline_s: Optional[float] = None,
                 start_margin_mm: Optional[float] = None) -> dict:
    """Plan Task 1. Pass a dict as `details` to get each photo's pose and
    standoff back (tests and the simulator use it; the RPi does not).

    The keyword arguments replan mid-run (replan_from_pose): start from the
    robot's live pose instead of the start zone, photograph only target_ids
    (every obstacle still blocks), report positions with the given frame
    shift, and finish within the given budgets. The first leg is checked
    against obstacles grown by start_margin_mm, because the robot may already
    be inside the normal margin."""
    t0 = time.monotonic()
    budget_s = PLAN_TIME_BUDGET_S if time_budget_s is None else time_budget_s
    deadline_s = PLAN_ASSEMBLY_DEADLINE_S if assembly_deadline_s is None else assembly_deadline_s
    radius_mm = TURN_RADIUS_MM[arc_profile]
    mid_run = start_pose is not None
    start = start_pose if mid_run else Pose(START_X_MM, START_Y_MM, START_THETA)
    # Every obstacle, the one being approached included: even the closest
    # photo pose (28 cm) is far outside its margin, so nothing needs excluding.
    obstacle_boxes = [_obstacle_aabb_mm(o) for o in obstacles]
    boxes = _collision_boxes(obstacle_boxes)
    if RPI_ARENA_GUARD:
        boxes.ref_bounds = _rpi_ref_bounds(start)

    # Extra blockers for the leg leaving START. If the start pose already breaks
    # them (START_X_MM / START_Y_MM set so the body is over the line) they would
    # trap the search at its first step, so drop them and say so.
    if mid_run:
        # No start-zone rules mid-run; only a thinner margin for the first leg.
        start_boxes = _collision_boxes([
            _obstacle_aabb_mm(o, start_margin_mm) for o in obstacles
        ])
        start_boxes.ref_bounds = boxes.ref_bounds
        # Drift can report the body slightly over an arena line; let the
        # first leg start there so it can drive back in.
        start_boxes.arena_overhang_mm = REPLAN_START_OVERHANG_MM
    else:
        start_boxes = _collision_boxes(list(boxes) + _start_boxes(start))
        start_boxes.ref_bounds = boxes.ref_bounds
        # This is a per-leg hook for measured placement tolerance. It defaults to
        # zero: the calculated 20 mm side gap keeps even the first sweep inside.
        start_boxes.arena_overhang_mm = START_GUARD_TOL_MM
    if not mid_run and not _pose_clear(start.x, start.y, start.theta, start_boxes):
        logging.warning(
            "Start pose is already outside the arena or over the start guard "
            f"({start}); not restricting the first leg. Check START_X_MM / START_Y_MM."
        )
        start_boxes = boxes

    options: Dict[object, List[PhotoOption]] = {}
    head_options: Dict[object, List[PhotoOption]] = {}
    diagonal_options: Dict[object, List[PhotoOption]] = {}
    eligible = []
    skipped = {}
    wanted = None if target_ids is None else {str(v) for v in target_ids}
    for obs in obstacles:
        if wanted is not None and str(obs.get("id")) not in wanted:
            continue
        if obs.get("d") not in FACE_FROM_D:
            logging.info(f"Obstacle {obs.get('id')}: SKIP (d={obs.get('d')}).")
            continue
        head_options[obs["id"]] = _photo_options(
            obs, boxes, obstacles, allow_oblique=False,
        )
        eligible.append(obs)

    # Use compact search immediately only when none of the targets has a
    # head-on pose. Mixed layouts retain the normal tour, then try leftovers.
    compact_mode = bool(eligible) and all(not head_options[obs["id"]] for obs in eligible)
    if compact_mode:
        logging.info(
            "No target has a safe head-on pose; using compact-layout "
            "multi-goal planning."
        )

    visitable = []
    for obs in eligible:
        heads = head_options[obs["id"]]
        diagonals = _photo_options(
            obs, boxes, obstacles, allow_oblique=True,
        )
        diagonal_options[obs["id"]] = diagonals
        opts = heads + diagonals
        if not opts:
            unrestricted_boxes = _collision_boxes(obstacle_boxes)
            unrestricted_options = (
                _photo_options(obs, unrestricted_boxes, obstacles,
                               allow_oblique=False) or
                _photo_options(obs, unrestricted_boxes, obstacles,
                               allow_oblique=True)
            )
            if boxes.ref_bounds is not None and unrestricted_options:
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

    if not compact_mode and visitable:
        # A bounded connectivity probe avoids spending the all-pairs budget
        # on a dense layout whose ordinary head-on approaches are unreachable.
        candidates = [(obs, option) for obs in visitable
                      for option in head_options[obs["id"]]]
        compact_mode = _search_any_approach(
            start, candidates, radius_mm, start_boxes,
            min(time.monotonic() + 1.0, t0 + deadline_s),
        ) is None
        if compact_mode:
            logging.info("Normal start connection unavailable within probe budget; "
                         "trying compact-layout fallback.")

    compact_legs = {}
    compact_options = {}
    compact_relaxed = set()
    legs = None
    choice = {}
    cache = {}
    if compact_mode:
        sequence, unreachable = _plan_compact_sequence(
            start, visitable, head_options, diagonal_options,
            radius_mm, boxes, start_boxes,
            t0 + deadline_s,
        )
        order = [item[0] for item in sequence]
        for obstacle, option, leg, relaxed in sequence:
            compact_legs[obstacle["id"]] = leg
            compact_options[obstacle["id"]] = option
            if relaxed:
                compact_relaxed.add(obstacle["id"])
        for obstacle in unreachable:
            logging.error(
                "Obstacle %s: no safe route from the compact-layout search — skipping.",
                obstacle["id"],
            )
            skipped[obstacle["id"]] = "no route"
        logging.info("Compact visit order: %s.", [o["id"] for o in order])
    else:
        legs = _Legs(
            start, options, radius_mm, boxes,
            t0 + budget_s, start_boxes,
        )
        order, choice, cache = _choose_tour(visitable, legs, options)
        reached, _ = _prefix_score(order, cache)
        # Leave disconnected targets to the bounded fallback rather than
        # spending its remaining time retrying the same missing tour edges.
        order = order[:reached]

    segments: List[List[str]] = []
    segment_obstacles: List[Optional[str]] = []
    dirs: List[dict] = []
    # expected[i][j] = [x_mm, y_mm, heading_deg] the RPi should report after
    # instruction j of segment i: its frame (north = 0, clockwise positive).
    expected: List[List[List[float]]] = []
    shift_x, shift_y = _frame_shift(start) if frame_shift is None else frame_shift
    photo_poses, standoffs = {}, {}
    safe_standoffs = {}
    ultrasonic_adjustments = {}
    view_angles = {}

    cur_id = "START"
    cur_pose = start
    start_relaxed = False

    def ordered_targets():
        yield from order
        if compact_mode:
            return
        pending = [obs for obs in visitable if obs["id"] not in photo_poses]
        if not pending:
            return
        logging.info("Keeping the normal tour; compact fallback for remaining targets %s.",
                     [obs["id"] for obs in pending])
        for obs in pending:
            oid = obs["id"]
            diagonal_options[oid] = _photo_options(obs, boxes, obstacles, allow_oblique=True)
            options[oid] = head_options[oid] + diagonal_options[oid]
        sequence, unreachable = _plan_compact_sequence(
            cur_pose, pending, head_options, diagonal_options, radius_mm,
            boxes, start_boxes if cur_id == "START" else boxes,
            t0 + deadline_s,
        )
        for obstacle in unreachable:
            reason = ("planning deadline reached" if time.monotonic() >=
                      t0 + deadline_s else "no route found within search limits")
            skipped[obstacle["id"]] = reason
            logging.warning(
                "Obstacle %s omitted: %s (%d head-on and %d diagonal photo options).",
                obstacle["id"], reason, len(head_options[obstacle["id"]]),
                len(diagonal_options[obstacle["id"]]),
            )
        for obstacle, option, leg, relaxed in sequence:
            oid = obstacle["id"]
            compact_legs[oid], compact_options[oid] = leg, option
            if relaxed:
                compact_relaxed.add(oid)
            yield obstacle

    for obs in ordered_targets():
        selected_index = choice.get(obs["id"], 0)
        option = compact_options.get(
            obs["id"], options[obs["id"]][selected_index],
        )
        used_relaxed_start = False
        # Pairwise tour costs use each option's ideal pose. The real emitted
        # whole-centimetre run-in can retain a small lateral offset, so rebuild
        # every later leg from the endpoint the preceding segment will actually
        # execute. Otherwise small offsets accumulate and an apparently safe arc
        # can cross an arena line.
        if obs["id"] in compact_legs:
            leg = compact_legs.get(obs["id"])
            used_relaxed_start = obs["id"] in compact_relaxed
        elif cur_id == "START" and (cur_id, obs["id"]) in cache:
            leg = cache.get((cur_id, obs["id"]))
        else:
            leg = None
            selected = option
            alternatives = sorted(
                range(len(options[obs["id"]])),
                key=lambda index: (
                    index != selected_index,
                    options[obs["id"]][index].view_angle_deg != selected.view_angle_deg,
                    abs(options[obs["id"]][index].standoff_cm - selected.standoff_cm),
                    index,
                ),
            )
            for candidate_index in alternatives:
                if time.monotonic() > t0 + deadline_s:
                    break
                candidate = options[obs["id"]][candidate_index]
                search_boxes = start_boxes if cur_id == "START" else boxes
                candidate_leg = _search_leg(
                    cur_pose, candidate, radius_mm, search_boxes,
                    deadline=t0 + deadline_s,
                )
                if candidate_leg is not None:
                    option, leg = candidate, candidate_leg
                    if candidate_index != selected_index:
                        logging.warning(
                            "Obstacle %s: changed assembled camera pose from %d cm/%+.0f deg "
                            "to %d cm/%+.0f deg to connect from the preceding real endpoint.",
                            obs["id"], selected.standoff_cm, selected.view_angle_deg,
                            option.standoff_cm, option.view_angle_deg,
                        )
                    break
        if leg is None:
            logging.error(
                f"Obstacle {obs['id']}: no route from the preceding segment's "
                "continuous endpoint — skipping."
            )
            skipped[obs["id"]] = "no route"
            continue

        search_tokens, search_poses, _ = leg
        if _uses_fine_turn(search_tokens):
            logging.warning(
                "Obstacle %s: congested/edge route uses 15/30-degree turn "
                "checkpoints: %s.",
                obs["id"], ",".join(
                    token for token in search_tokens
                    if token[:2] in ("FR", "FL", "RR", "RL")
                ),
            )
        if cur_id == "START" and (used_relaxed_start or
                (legs is not None and obs["id"] in choice and
                 ("START", None, obs["id"], choice[obs["id"]]) in legs.relaxed)):
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
        # Reach the selected camera pose using ordinary odometry. Edge-angle
        # poses reverse outward after completing their turn inboard; head-on
        # poses retain the normal forward run-in.
        run_cm = round(back / 10.0)
        if run_cm > 0:
            run_mm = run_cm * 10.0
            direction = -1.0 if option.approach_reverse else 1.0
            tokens.append(rev(run_cm) if option.approach_reverse else fwd(run_cm))
            terminal = Pose(
                search_poses[-1].x + direction * run_mm * math.cos(final.theta),
                search_poses[-1].y + direction * run_mm * math.sin(final.theta),
                final.theta,
            )
            poses.append(terminal)
        else:
            terminal = Pose(search_poses[-1].x, search_poses[-1].y, final.theta)
        tokens.append(stop())
        poses.append(terminal)

        if option.standoff_cm != STANDOFF_CANDIDATES_CM[0]:
            level = logging.WARNING if option.standoff_cm in STANDOFF_LAST_RESORT_CM else logging.INFO
            logging.log(level, f"Obstacle {obs['id']}: photo at {option.standoff_cm} cm"
                               f"{' (last resort)' if level == logging.WARNING else ''}.")
        if option.view_angle_deg:
            if abs(option.view_angle_deg) > 45:
                logging.warning(
                    "Obstacle %s: using a last-resort %+.0f degree edge view; "
                    "ultrasonic correction is disabled for this oblique face.",
                    obs["id"], option.view_angle_deg,
                )
            else:
                logging.warning(
                    "Obstacle %s: using a %+.0f degree diagonal camera view because "
                    "the head-on route is more constrained.",
                    obs["id"], option.view_angle_deg,
                )

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

        photo_poses[obs["id"]] = (terminal.x, terminal.y, terminal.theta)
        skipped.pop(obs["id"], None)
        standoffs[obs["id"]] = option.standoff_cm
        view_angles[obs["id"]] = option.view_angle_deg
        ultrasonic_adjustments[str(obs["id"])] = (
            option.view_angle_deg == 0 and option.us_runin_mm is not None
        )
        safe_standoffs[str(obs["id"])] = sorted({
            candidate.standoff_cm for candidate in options[obs["id"]]
            if candidate.view_angle_deg == option.view_angle_deg
            and (not option.view_angle_deg or candidate.standoff_cm >= 25)
        })
        cur_pose = terminal
        cur_id = obs["id"]

    logging.info(f"Planned {len(photo_poses)}/"
                 f"{len(eligible) if mid_run else len(obstacles)} photo(s) in "
                 f"{time.monotonic() - t0:.1f}s.")
    if details is not None:
        details["photo_poses"] = photo_poses
        details["standoff_cm"] = standoffs
        details["view_angle_deg"] = view_angles
        details["skipped"] = skipped
        details["frame_shift_mm"] = (shift_x, shift_y)
        details["start_relaxed"] = start_relaxed

    result = {
        "segments": segments,
        "obstacle_ids": list(photo_poses),
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
        "ultrasonic_adjustments": ultrasonic_adjustments,
        "camera_to_sensor_cm": CAMERA_TO_SENSOR_MM / 10.0,
        "selected_view_angles": {
            str(obstacle_id): angle
            for obstacle_id, angle in view_angles.items()
        },
        "expected": expected,
    }
    if _contains_fu(result["segments"]):
        raise RuntimeError("Task 1 planner produced a forbidden FU token")
    return result


# ── Mid-run replan with sensor position correction ────────────────────────────
# Used where the PC would otherwise HOLD (no recovery route and a predicted
# collision). Odometry is corrected by comparing the ultrasonic and side-IR
# ranges with what the Android obstacle map predicts from the odometry pose,
# then the remaining photos are planned afresh from the corrected pose.

# A reading is trusted only if it is within this of the map's prediction.
# Anything further off is probably a different object (or no object) and is
# ignored, leaving odometry unchanged in that direction.
POSE_CORRECTION_MAX_RESIDUAL_MM = float(os.getenv("POSE_CORRECTION_MAX_RESIDUAL_MM", "100"))
# A second obstacle predicted this close to the reading makes the echo
# ambiguous; the reading is then not used.
POSE_CORRECTION_AMBIGUITY_MM = float(os.getenv("POSE_CORRECTION_AMBIGUITY_MM", "100"))
POSE_CORRECTION_US_MAX_CM = float(os.getenv("POSE_CORRECTION_US_MAX_CM", "150"))
POSE_CORRECTION_USE_IR = os.getenv("POSE_CORRECTION_USE_IR", "1").strip().lower() in ("1", "true", "yes")
# The Sharp IRs are accurate near the robot only (usable 10-80 cm, right unit to
# ~65 cm); keep to the close, steep part of the curve.
POSE_CORRECTION_IR_MAX_CM = float(os.getenv("POSE_CORRECTION_IR_MAX_CM", "50"))
# Left and right IR estimates of the sideways error must agree this well.
POSE_CORRECTION_IR_AGREE_MM = float(os.getenv("POSE_CORRECTION_IR_AGREE_MM", "50"))
# IR mounting, from the rear axle: distance ahead, and distance out to each
# side. NOT MEASURED - assumed at the body sides, level with the chassis
# centre. Measure and set these if the sensors sit elsewhere.
IR_MOUNT_AHEAD_MM = float(os.getenv("IR_MOUNT_AHEAD_MM", str(BODY_CENTRE_AHEAD_MM)))
IR_MOUNT_SIDE_MM = float(os.getenv("IR_MOUNT_SIDE_MM", str(ROBOT_HALF_WIDTH_MM)))
IR_HALF_ANGLE_DEG = float(os.getenv("IR_HALF_ANGLE_DEG", "3"))
US_BIAS_MM = US_BIAS_CM * 10.0

# The replan must answer inside the RPi's decision wait. These split whatever
# time is left between the tour search and assembling the route.
REPLAN_TOUR_SHARE = 0.6
REPLAN_ASSEMBLY_SHARE = 0.9
REPLAN_MIN_TIME_S = 1.0
REPLAN_START_OVERHANG_MM = float(os.getenv("REPLAN_START_OVERHANG_MM", "50"))


def _ray_box_distance(ox, oy, angle, box) -> Optional[float]:
    """Distance along a ray from (ox, oy) to an axis-aligned box, or None."""
    dx, dy = math.cos(angle), math.sin(angle)
    t_near, t_far = -math.inf, math.inf
    for origin, direction, low, high in ((ox, dx, box[0], box[2]), (oy, dy, box[1], box[3])):
        if abs(direction) < 1e-12:
            if origin < low or origin > high:
                return None
            continue
        t1, t2 = (low - origin) / direction, (high - origin) / direction
        t_near, t_far = max(t_near, min(t1, t2)), min(t_far, max(t1, t2))
    if t_near > t_far or t_far < 0:
        return None
    return max(0.0, t_near)


def _predicted_range(ox, oy, centre_angle, half_angle_deg, obstacles):
    """Nearest-echo prediction across a sensor cone, per obstacle.

    Returns [(distance_mm, obstacle_id)] sorted nearest first, one entry per
    obstacle the cone reaches."""
    steps = max(1, int(round(half_angle_deg)))
    best = {}
    for k in range(-steps, steps + 1):
        angle = centre_angle + math.radians(half_angle_deg * k / steps)
        for obstacle in obstacles:
            box = _obstacle_aabb_mm(obstacle, 0.0)
            distance = _ray_box_distance(ox, oy, angle, box)
            if distance is not None and distance < best.get(obstacle["id"], math.inf):
                best[obstacle["id"]] = distance
    return sorted((d, oid) for oid, d in best.items())


def _range_residual(ox, oy, angle, half_angle_deg, measured_mm, obstacles):
    """predicted - measured for an unambiguous reading, else None."""
    hits = _predicted_range(ox, oy, angle, half_angle_deg, obstacles)
    if not hits:
        return None, "no obstacle in view on the map"
    predicted, oid = hits[0]
    residual = predicted - measured_mm
    if abs(residual) > POSE_CORRECTION_MAX_RESIDUAL_MM:
        return None, (f"reads {measured_mm:.0f} mm, map says {predicted:.0f} mm to "
                      f"obstacle {oid}; too different to trust")
    if len(hits) > 1 and abs(hits[1][0] - measured_mm) <= POSE_CORRECTION_AMBIGUITY_MM:
        return None, (f"ambiguous between obstacles {oid} and {hits[1][1]}")
    return residual, f"obstacle {oid}: map {predicted:.0f} mm, read {measured_mm:.0f} mm"


def _sensor_number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number) or number <= 0 or number >= 65535:
        return None
    return number


def estimate_pose_correction(x, y, theta, progress: dict, obstacles: List[dict]):
    """Correct a planner-frame rear-axle pose with the live range sensors.

    Returns (dx_mm, dy_mm, notes). The ultrasonic corrects along the heading,
    the side IRs across it. A sensor whose reading does not match the map
    within POSE_CORRECTION_MAX_RESIDUAL_MM leaves that direction on odometry.
    Heading is not corrected: the gyro is far better than these ranges.
    """
    notes = []
    ux, uy = math.cos(theta), math.sin(theta)
    lx, ly = -uy, ux                       # unit vector to the robot's left

    along = 0.0
    us_cm = _sensor_number(progress.get("ultrasonic_cm"))
    if us_cm is not None and us_cm <= POSE_CORRECTION_US_MAX_CM:
        residual, why = _range_residual(
            x + REAR_AXLE_TO_SENSOR_MM * ux, y + REAR_AXLE_TO_SENSOR_MM * uy,
            theta, SONAR_HALF_ANGLE_DEG, us_cm * 10.0 - US_BIAS_MM, obstacles,
        )
        notes.append(f"ultrasonic: {why}")
        if residual is not None:
            along = residual
    else:
        notes.append("ultrasonic: no usable reading")

    sideways = []
    if POSE_CORRECTION_USE_IR:
        for name, side in (("left", 1.0), ("right", -1.0)):
            reading = _sensor_number(progress.get(f"ir_{name}_cm"))
            if reading is None or reading > POSE_CORRECTION_IR_MAX_CM:
                notes.append(f"IR {name}: no usable reading")
                continue
            residual, why = _range_residual(
                x + IR_MOUNT_AHEAD_MM * ux + side * IR_MOUNT_SIDE_MM * lx,
                y + IR_MOUNT_AHEAD_MM * uy + side * IR_MOUNT_SIDE_MM * ly,
                theta + side * math.pi / 2, IR_HALF_ANGLE_DEG,
                reading * 10.0, obstacles,
            )
            notes.append(f"IR {name}: {why}")
            if residual is not None:
                sideways.append(side * residual)   # + means "actually further left"
    lateral = 0.0
    if len(sideways) == 2 and abs(sideways[0] - sideways[1]) > POSE_CORRECTION_IR_AGREE_MM:
        notes.append("IR left/right disagree; sideways correction not used")
    elif sideways:
        lateral = sum(sideways) / len(sideways)

    return along * ux + lateral * lx, along * uy + lateral * ly, notes


def replan_from_pose(plan: dict, obstacles: List[dict], progress: dict,
                     deadline: float, arc_profile: int = PROFILE_TIGHT) -> Optional[dict]:
    """Plan the remaining photos afresh from the robot's (corrected) live pose.

    The result is a REPLACE body that may visit the remaining obstacles in a
    new order and leave out any it cannot reach. Its frame_shift_mm absorbs
    the sensor correction, so later PROGRESS reports (still raw odometry) are
    compared with, and recovered from, the corrected position. None if no
    route to any remaining obstacle fits before `deadline` (time.monotonic()).
    """
    try:
        wire_x = float(progress["x_grid"]) * 100.0
        wire_y = float(progress["y_grid"]) * 100.0
        bearing = float(progress["heading_deg"]) % 360.0
        remaining = [str(v) for v in progress.get("remaining_photo_ids") or []]
    except (KeyError, TypeError, ValueError):
        logging.exception("Cannot replan: PROGRESS pose is malformed.")
        return None
    if not remaining:
        return None
    known = {str(o.get("id")) for o in obstacles}
    if not set(remaining) <= known:
        logging.error("Cannot replan: pending photos %s are not all on the map %s.",
                      remaining, sorted(known))
        return None

    shift_x, shift_y = _plan_frame_shift(plan)
    x, y = wire_x - shift_x, wire_y - shift_y
    theta = math.radians(90.0 - bearing)
    lattice = grid_search._HEADINGS[grid_search._heading_index(theta)]
    snap_error = abs(math.degrees(_wrap(theta - lattice)))
    if snap_error > RECOVERY_MAX_HEADING_SNAP_DEG:
        logging.error("Cannot replan: heading %.1f deg is %.1f deg off the 45-degree lattice.",
                      bearing, snap_error)
        return None

    dx, dy, notes = estimate_pose_correction(x, y, theta, progress, obstacles)
    for note in notes:
        logging.info("Pose check — %s", note)
    # Corrected pose first; if that has no route (the correction may be
    # wrong), the plain odometry pose with whatever time is left.
    attempts = [(dx, dy, 0.6), (0.0, 0.0, 1.0)] if (dx or dy) else [(0.0, 0.0, 1.0)]
    result = None
    for cx, cy, share in attempts:
        left = deadline - time.monotonic()
        if left < REPLAN_MIN_TIME_S:
            logging.error("Cannot replan: only %.1fs left before the RPi stops waiting.", left)
            return None
        left *= share
        if cx or cy:
            logging.warning("Pose corrected by (%+.0f, %+.0f) mm from the range sensors.", cx, cy)
        elif dx or dy:
            logging.warning("No route from the corrected pose; trying the odometry pose.")
        result = plan_mission(
            obstacles, arc_profile=arc_profile,
            start_pose=Pose(x + cx, y + cy, lattice),
            frame_shift=(shift_x - cx, shift_y - cy),
            target_ids=remaining,
            time_budget_s=left * REPLAN_TOUR_SHARE,
            assembly_deadline_s=left * REPLAN_ASSEMBLY_SHARE,
            start_margin_mm=RECOVERY_RELAXED_MARGIN_MM,
        )
        if result["segments"]:
            dx, dy = cx, cy
            break
    if not result or not result["segments"]:
        logging.error("Replan found no route to any of %s.", remaining)
        return None
    planned = [str(v) for v in result["obstacle_ids"]]
    skipped = [oid for oid in remaining if oid not in planned]
    if skipped:
        logging.warning("Replan skips obstacle(s) %s: no route from here.", skipped)
    logging.warning("Replanned from the live pose: visit order %s.", planned)

    replacement = {key: result[key] for key in (
        "segments", "segment_obstacles", "expected", "frame_shift_mm",
        "photo_standoffs", "selected_standoffs", "ultrasonic_adjustments",
        "selected_view_angles", "camera_to_sensor_cm", "obstacle_ids",
    )}
    replacement["start_mm"] = plan.get("start_mm", result["start_mm"])
    replacement["skipped_photo_ids"] = skipped
    replacement["allow_skip"] = True
    replacement["pose_correction_mm"] = {"x": round(dx, 1), "y": round(dy, 1)}
    return replacement
