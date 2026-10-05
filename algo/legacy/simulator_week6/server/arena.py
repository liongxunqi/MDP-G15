"""The arena model: grid, obstacles, robot footprint, collision checks.

Everything here follows the "option 2" grid representation from the MDP
algorithms briefing (slide 9): a 20x20 grid of 10cm cells, robot footprint
30x30cm = 3x3 cells identified by its bottom-left cell + facing, obstacle
footprint 10x10cm = 1x1 cell.

Collision checking follows the briefing's own suggested simplification
(slide 36): since the robot's footprint is a fixed axis-aligned 3x3 box
regardless of which way it's facing (only which edge the camera sits on
changes — see slide 9's diagram), we don't need any rotated-footprint
geometry. A pose is valid iff its 3x3 box is inside the arena and doesn't
overlap any obstacle's box (padded by a small safety margin).
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

GRID_SIZE = 20          # 20x20 cells -> 200cm x 200cm (briefing slide 3, 9)
CELL_CM = 10             # each cell is 10cm
ROBOT_SIZE = 3           # 30cm footprint -> 3x3 cells (briefing slide 9)
START_ZONE_SIZE = 4      # 40cm start zone -> 4x4 cells (briefing slide 3)
CAMERA_CLEARANCE = 2     # measured recognition distance ~20cm -> 2 cells
OBSTACLE_MARGIN_CELLS = 0.5  # small safety pad so the robot doesn't graze a corner


class Facing(str, Enum):
    N = "N"
    E = "E"
    S = "S"
    W = "W"


# Unit step (dx, dy) for moving one cell in each facing.
_DELTA: Dict[Facing, Tuple[int, int]] = {
    Facing.N: (0, 1),
    Facing.E: (1, 0),
    Facing.S: (0, -1),
    Facing.W: (-1, 0),
}

# 90-degree turns, both ways.
_LEFT_OF: Dict[Facing, Facing] = {Facing.N: Facing.W, Facing.W: Facing.S, Facing.S: Facing.E, Facing.E: Facing.N}
_RIGHT_OF: Dict[Facing, Facing] = {v: k for k, v in _LEFT_OF.items()}


def delta(facing: Facing) -> Tuple[int, int]:
    return _DELTA[facing]


def left_of(facing: Facing) -> Facing:
    return _LEFT_OF[facing]


def right_of(facing: Facing) -> Facing:
    return _RIGHT_OF[facing]


def opposite(facing: Facing) -> Facing:
    return left_of(left_of(facing))


@dataclass(frozen=True)
class RobotPose:
    """Robot's bottom-left grid cell + facing. The 3x3 footprint this
    anchors is always the same axis-aligned box; only the camera's edge
    (see `camera_cell`) depends on `facing`."""
    x: int
    y: int
    facing: Facing

    def footprint(self) -> Tuple[int, int, int, int]:
        """(min_x, min_y, max_x, max_y) — max is exclusive, i.e. the box
        covers cells [min_x, max_x) x [min_y, max_y)."""
        return self.x, self.y, self.x + ROBOT_SIZE, self.y + ROBOT_SIZE

    def key(self) -> Tuple[int, int, str]:
        return self.x, self.y, self.facing.value


@dataclass(frozen=True)
class Obstacle:
    id: int
    x: int
    y: int
    facing: Facing  # which side the image is on


class Arena:
    def __init__(self, obstacles: List[Obstacle]):
        self.obstacles = obstacles

    def in_bounds(self, pose: RobotPose) -> bool:
        min_x, min_y, max_x, max_y = pose.footprint()
        return 0 <= min_x and 0 <= min_y and max_x <= GRID_SIZE and max_y <= GRID_SIZE

    def is_valid(self, pose: RobotPose) -> bool:
        """Is `pose` inside the arena and clear of every obstacle?"""
        return self.in_bounds(pose) and self.blocking_obstacle(pose) is None

    def blocking_obstacle(self, pose: RobotPose) -> Optional[Obstacle]:
        """The first obstacle whose (margin-padded) box overlaps `pose`'s
        footprint, or None if the footprint is clear. Bounds are not
        checked here — see `in_bounds` — since an out-of-grid pose has no
        "blocking obstacle" of its own; it's just off the grid."""
        r_min_x, r_min_y, r_max_x, r_max_y = pose.footprint()
        m = OBSTACLE_MARGIN_CELLS
        for obs in self.obstacles:
            o_min_x, o_min_y = obs.x - m, obs.y - m
            o_max_x, o_max_y = obs.x + 1 + m, obs.y + 1 + m
            overlaps_x = r_min_x < o_max_x and o_min_x < r_max_x
            overlaps_y = r_min_y < o_max_y and o_min_y < r_max_y
            if overlaps_x and overlaps_y:
                return obs
        return None

    def viewpoint_for(self, obstacle: Obstacle) -> RobotPose:
        """Where the robot should stand (bottom-left cell + facing) to get
        its camera `CAMERA_CLEARANCE` cells from the obstacle's image side.

        Derivation follows the briefing's own worked example (slide 10: an
        image at grid cell (a, b) facing South puts the robot's bottom-left
        corner at (a-1, b - (ROBOT_SIZE + CAMERA_CLEARANCE)) facing North,
        with CAMERA_CLEARANCE=2 for a ~20cm recognition distance — matching
        our own sensor's measured distance, so this lines up with the
        slide's literal (a-1, b-5) example): center the 3x3 footprint on
        the obstacle's column/row, then back the camera-side edge off by
        CAMERA_CLEARANCE cells. The same derivation generalizes to the
        other three facings below.
        """
        a, b = obstacle.x, obstacle.y
        facing = obstacle.facing
        c = CAMERA_CLEARANCE

        if facing == Facing.S:
            # robot approaches from below, facing North
            return RobotPose(a - 1, b - (ROBOT_SIZE + c), Facing.N)
        if facing == Facing.N:
            # robot approaches from above, facing South
            return RobotPose(a - 1, b + 1 + c, Facing.S)
        if facing == Facing.E:
            # robot approaches from the right, facing West
            return RobotPose(a + 1 + c, b - 1, Facing.W)
        # Facing.W: robot approaches from the left, facing East
        return RobotPose(a - (ROBOT_SIZE + c), b - 1, Facing.E)
