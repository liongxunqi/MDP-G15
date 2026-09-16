"""
MDP Algorithm Simulator

Controls:
  - Click grid cells to place obstacles (mode = "obstacle")
  - Radio "side": which face of the obstacle has the image (N/E/S/W)
  - "Plan route": A* + TSP over all obstacle approach poses
  - "Play": animates the robot along the planned path
  - Arrow keys: rotate the robot's start heading
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional, Iterable
from enum import IntEnum, Enum

Cell = Tuple[int, int]

ROBOT_HALF = 1  # robot footprint is 3x3 cells (30cm x 30cm), 1 cell from center each way

class Heading(IntEnum):
    """Robot heading in quarter turns: 0=E, 1=N, 2=W, 3=S."""
    E = 0
    N = 1
    W = 2
    S = 3

class Side(str, Enum):
    """Which face of an obstacle must be photographed."""
    N = "N"
    E = "E"
    S = "S"
    W = "W"

@dataclass(frozen=True)
class Pose:
    """Grid pose with orientation."""
    x: int
    y: int
    h: Heading

@dataclass(frozen=True)
class Primitive:
    """A discrete motion primitive in the robot's local frame (name, heading delta, end offset, swept cells, cost)."""
    name: str
    dtheta: int
    end_local: Tuple[int, int]
    swept_local: List[Cell]
    cost: float
    reverse_ok: bool = False

@dataclass
class PrimitiveLibrary:
    name: str
    primitives: List[Primitive] = field(default_factory=list)

@dataclass
class OccupancyGrid:
    """Binary occupancy grid. True = occupied, False = free."""
    width: int
    height: int
    grid: List[List[bool]] = field(init=False)

    def __post_init__(self):
        self.grid = [[False] * self.height for _ in range(self.width)]

    def load_obstacles(self, occ: Iterable[Cell]) -> None:
        for (x, y) in occ:
            if 0 <= x < self.width and 0 <= y < self.height:
                self.grid[x][y] = True

    def is_free(self, x, y=None) -> bool:
        if y is None and isinstance(x, tuple):
            x, y = x
        return self.in_bounds(x, y) and (not self.grid[x][y])

    def in_bounds(self, x, y=None) -> bool:
        if y is None and isinstance(x, tuple):
            x, y = x
        return 0 <= x < self.width and 0 <= y < self.height

    def robot_center_in_bounds(self, x, y=None, half: int = ROBOT_HALF) -> bool:
        """True if a robot CENTER at (x,y) keeps its 3x3 footprint fully inside the grid."""
        if y is None and isinstance(x, tuple):
            x, y = x
        return (half <= x <= self.width - 1 - half) and (half <= y <= self.height - 1 - half)

    def set_occ(self, x: int, y: int, value: bool = True) -> None:
        if self.in_bounds(x, y):
            self.grid[x][y] = value

    def clear(self) -> None:
        for x in range(self.width):
            for y in range(self.height):
                self.grid[x][y] = False

    def occupied_cells(self) -> Iterable[Cell]:
        for x in range(self.width):
            for y in range(self.height):
                if self.grid[x][y]:
                    yield (x, y)

    def block_rect(self, x0: int, y0: int, x1: int, y1: int) -> None:
        """Mark inclusive rectangle [x0,x1]x[y0,y1] as occupied (clamped to bounds)."""
        x0, x1 = max(0, x0), min(self.width - 1, x1)
        y0, y1 = max(0, y0), min(self.height - 1, y1)
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                self.grid[x][y] = True

    def copy(self) -> "OccupancyGrid":
        g = OccupancyGrid(self.width, self.height)
        for x in range(self.width):
            g.grid[x][:] = self.grid[x][:]
        return g

    def with_inflation(self, r: int) -> "OccupancyGrid":
        """Return a cloned grid inflated by Chebyshev radius r."""
        g = self.copy()
        g.inflate(r)
        return g

    def inflate(self, r: int) -> None:
        """Inflate occupied cells by Chebyshev radius r (square dilation)."""
        if r <= 0:
            return
        new = [[False] * self.height for _ in range(self.width)]
        occ = list(self.occupied_cells())
        for (x, y) in occ:
            x_min = max(0, x - r)
            x_max = min(self.width - 1, x + r)
            y_min = max(0, y - r)
            y_max = min(self.height - 1, y + r)
            for ix in range(x_min, x_max + 1):
                for iy in range(y_min, y_max + 1):
                    new[ix][iy] = True
        self.grid = new


@dataclass(frozen=True)
class Obstacle:
    """A single 1x1 cell obstacle with a required photo side."""
    cell: Cell
    side: Side

@dataclass(frozen=True)
class ApproachPose:
    """The pose where the robot must stand to photograph an obstacle."""
    pose: Pose
    obstacle: Obstacle

@dataclass
class Scenario:
    grid: OccupancyGrid
    start: Pose
    obstacles: List[Obstacle] = field(default_factory=list)
    primitive_lib: PrimitiveLibrary = field(default_factory=lambda: PrimitiveLibrary("default", []))

@dataclass(frozen=True)
class PlanLeg:
    """One planned leg from start to end, with the full pose/primitive sequence and cost."""
    start: Pose
    end: Pose
    states: List[Pose]
    prims: List[Primitive]
    cost: float

@dataclass
class TourPlan:
    """A full multi-target tour: visiting order, approach poses, stitched legs, and total cost."""
    order: List[Obstacle]
    approaches: List[ApproachPose]
    legs: List[PlanLeg]
    states_all: List[Pose]
    prims_all: List[Primitive]
    total_cost: float
    total_obstacles: int = 0  # every obstacle handed to the planner, regardless of outcome
    unreachable: List[Tuple[Obstacle, str]] = field(default_factory=list)  # (obstacle, reason)

@dataclass(frozen=True)
class Command:
    """A compressed command like ('F1', 3) meaning execute primitive 'F1' three times."""
    name: str
    count: int = 1

@dataclass
class CommandStream:
    commands: List[Command] = field(default_factory=list)

@dataclass
class Session:
    scenario: Scenario
    tour: Optional[TourPlan] = None

# -------- Kinematics --------
from typing import Tuple, List, Iterable

def rotate_local(dx: int, dy: int, h: Heading) -> Tuple[int, int]:
    """Rotate local (dx,dy) by heading h to get global (dx,dy)."""
    if h == Heading.E:
        return (dx, dy)
    elif h == Heading.N:
        return (-dy, dx)
    elif h == Heading.W:
        return (-dx, -dy)
    elif h == Heading.S:
        return (dy, -dx)
    else:
        raise ValueError("Invalid heading")
    
def apply_primitive(pose: Pose, primitive: Primitive) -> Pose:
    """Apply a motion primitive to a pose, returning the new pose."""
    new_h = Heading((pose.h + primitive.dtheta) % 4)
    dx_global, dy_global = rotate_local(primitive.end_local[0], primitive.end_local[1], pose.h)
    new_x = pose.x + dx_global
    new_y = pose.y + dy_global
    return Pose(new_x, new_y, new_h)

def swept_cells_world(pose: Pose, primitive: Primitive) -> Iterable[Cell]:
    """Get the list of world cells swept by the robot when executing a primitive from a given pose."""
    cells = []
    for (dx, dy) in primitive.swept_local:
        dx_global, dy_global = rotate_local(dx, dy, pose.h)
        cells.append((pose.x + dx_global, pose.y + dy_global))
    return cells

def collision_free(pose: Pose, primitive: Primitive, grid: OccupancyGrid) -> bool:
    """Check if executing a primitive from a given pose is collision-free on the occupancy grid."""
    for cell in swept_cells_world(pose, primitive):
        x, y = cell
        if not grid.in_bounds(x, y) or not grid.is_free(x, y):
            return False
    return True

# ============================== TARGETS (approach poses) ==============================
from typing import List, Iterable, Tuple, Optional

# ------------------------------
# Core rule: obstacle side -> approach pose
# ------------------------------

def approach_for_side(ob: Obstacle, standoff: int = 1) -> Pose:
    """
    Return the SINGLE approach pose implied by one obstacle and its side,
    using a 'standoff' (default 1 cell away) and facing the obstacle.

    Mapping (ox,oy) = obstacle cell:
      Side.N -> stand at (ox, oy+standoff), face South
      Side.S -> stand at (ox, oy-standoff), face North
      Side.E -> stand at (ox+standoff, oy), face West
      Side.W -> stand at (ox-standoff, oy), face East
    """
    ox, oy = ob.cell
    s = ob.side

    if s == Side.N:
        return Pose(ox, oy + standoff, Heading.S)
    if s == Side.S:
        return Pose(ox, oy - standoff, Heading.N)
    if s == Side.E:
        return Pose(ox + standoff, oy, Heading.W)
    if s == Side.W:
        return Pose(ox - standoff, oy, Heading.E)

    raise ValueError(f"Unknown side: {s}")


def derive_approaches(
    obstacles: List[Obstacle],
    grid: OccupancyGrid,
    standoffs: Iterable[int] = (4,),
    drop_if_invalid: bool = True,
) -> Tuple[List[ApproachPose], List[Tuple[Obstacle, str]]]:
    """Convert obstacles to ApproachPose(s). Returns (approaches, skipped) where skipped
    holds (obstacle, human-readable reason) for any obstacle with no valid standing spot."""
    out: List[ApproachPose] = []
    skipped: List[Tuple[Obstacle, str]] = []
    for ob in obstacles:
        chosen: Optional[Pose] = None
        any_in_bounds = False
        for d in standoffs:
            cand = approach_for_side(ob, standoff=d)
            in_bounds = grid.robot_center_in_bounds(cand.x, cand.y)
            any_in_bounds = any_in_bounds or in_bounds
            if in_bounds and grid.is_free(cand.x, cand.y):
                chosen = cand
                break
        if chosen is not None:
            out.append(ApproachPose(pose=chosen, obstacle=ob))
        else:
            if not any_in_bounds:
                reason = (f"too close to the arena boundary on the {ob.side.value} side -- "
                          f"no standing spot in the 20-50cm recognition range keeps the "
                          f"robot's 3x3 footprint inside the arena")
            else:
                reason = f"the {ob.side.value}-side standing spot(s) are blocked by other obstacles"
            skipped.append((ob, reason))
            print(f"[targets] WARNING: obstacle at {ob.cell} side={ob.side.value}: {reason}")
            if not drop_if_invalid:
                raise RuntimeError(f"No valid approach for {ob}: {reason}")
    return out, skipped

# -------- A* planner --------
from typing import Iterable, List, Optional, Tuple, Dict
import heapq
from math import sqrt
from itertools import count

def octile(dx: int, dy: int) -> float:
    F = sqrt(2) - 1
    return F * min(dx, dy) + max(dx, dy)

def cheapest_turn_cost(lib: PrimitiveLibrary) -> float:
    turn_costs = [p.cost for p in lib.primitives if p.dtheta != 0]
    return min(turn_costs) if turn_costs else 0.0  # safe fallback

def heading_penalty(h_from: int, h_to: int, cap: float) -> float:
    diff = abs(h_from - h_to) % 4
    if diff == 0: return 0.0
    if diff == 2: return cap
    return cap / 2  # 90° either direction

def heuristic(a: Pose, b: Pose, lib: PrimitiveLibrary) -> float:
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    base_cost = octile(dx, dy)
    turn_cost = heading_penalty(a.h, b.h, cheapest_turn_cost(lib))
    return base_cost + turn_cost

def expand_neighbors(p: Pose, lib: PrimitiveLibrary, grid: OccupancyGrid):
    """Generate valid neighboring poses from pose p using the primitive library and occupancy grid."""
    for prim in lib.primitives:
        new_pose = apply_primitive(p, prim)
        if grid.robot_center_in_bounds(new_pose.x, new_pose.y) and collision_free(p, prim, grid):
            yield (new_pose, prim, prim.cost)

def reconstruct(start: Pose, goal: Pose, parent: Dict[Pose, Tuple[Pose, Primitive]], g_cost: float) -> PlanLeg:
    """Reconstruct states and prims from start -> goal using the parent map."""
    states_rev: List[Pose] = [goal]
    prims_rev: List[Primitive] = []
    cur = goal
    while cur in parent:
        prev, prim = parent[cur]
        prims_rev.append(prim)
        states_rev.append(prev)
        cur = prev
    states = list(reversed(states_rev))
    prims = list(reversed(prims_rev))
    return PlanLeg(start=states[0], end=states[-1], states=states, prims=prims, cost=g_cost)



def astar_leg(grid: OccupancyGrid, start: Pose, goal: Pose, lib: PrimitiveLibrary) -> Optional[PlanLeg]:
    """A* search for a plan leg from start to goal."""
    tie = count()  # tiebreaker for heap items with equal f-score

    open_set = []
    h0 = heuristic(start, goal, lib)
    heapq.heappush(open_set, (h0, 0.0, next(tie), start))

    came_from: Dict[Pose, Tuple[Pose, Primitive]] = {}
    g_score: Dict[Pose, float] = {start: 0.0}

    while open_set:
        _, current_g, _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct(start, current, came_from, current_g)

        if current_g > g_score.get(current, float("inf")):
            continue

        for neighbor, prim, cost in expand_neighbors(current, lib, grid):
            tentative_g = current_g + cost
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = (current, prim)
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal, lib)
                heapq.heappush(open_set, (f, tentative_g, next(tie), neighbor))

    return None

# -------- Tour planner (TSP over approach poses) --------
from typing import List, Optional, Tuple, Dict
from itertools import permutations


def precompute_legs(
    scn: Scenario,
    stops: List[Pose],
) -> Tuple[Dict[Tuple[int, int], Optional[PlanLeg]], Dict[Tuple[int, int], float]]:
    """Run A* between every ordered pair of stop indices; returns legs and costs keyed by (i, j)."""
    legs: Dict[Tuple[int, int], Optional[PlanLeg]] = {}
    costs: Dict[Tuple[int, int], float] = {}

    n = len(stops)
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            leg = astar_leg(scn.grid, stops[i], stops[j], scn.primitive_lib)
            legs[(i, j)] = leg
            costs[(i, j)] = float("inf") if leg is None else leg.cost
    return legs, costs


def choose_order(
    costs: Dict[Tuple[int, int], float],
    n_targets: int,
) -> List[int]:
    """Choose visiting order from node 0 (start): exact TSP for <=8 targets, else greedy nearest-neighbour."""
    all_targets = list(range(1, n_targets + 1))

    if n_targets <= 8:
        best_perm = None
        best_cost = float("inf")
        for perm in permutations(all_targets):
            cur = 0
            total = 0.0
            ok = True
            for k in perm:
                c = costs.get((cur, k), float("inf"))
                if c == float("inf"):
                    ok = False
                    break
                total += c
                cur = k
            if ok and total < best_cost:
                best_cost = total
                best_perm = perm
        if best_perm is None:
            return greedy_order(costs, n_targets)
        return [0] + list(best_perm)

    return greedy_order(costs, n_targets)


def greedy_order(costs: Dict[Tuple[int, int], float], n_targets: int) -> List[int]:
    """Greedy nearest-neighbour from node 0, skipping unreachable hops."""
    unvisited = set(range(1, n_targets + 1))
    order = [0]
    cur = 0
    while unvisited:
        reachable = [(k, costs.get((cur, k), float("inf"))) for k in unvisited]
        reachable = [(k, c) for (k, c) in reachable if c != float("inf")]
        if not reachable:
            break
        nxt = min(reachable, key=lambda t: t[1])[0]
        order.append(nxt)
        unvisited.remove(nxt)
        cur = nxt
    return order


def stitch(
    stops: List[Pose],
    legs: Dict[Tuple[int, int], PlanLeg],
    order: List[int],
    obstacles_in_order: List[Obstacle],
    approaches_in_order: List[ApproachPose],
) -> TourPlan:
    """Concatenate legs following 'order' into a single TourPlan with stitched states and summed cost."""
    seq_legs: List[PlanLeg] = []
    states_all: List[Pose] = []
    prims_all: List = []
    total_cost = 0.0

    for i in range(len(order) - 1):
        a, b = order[i], order[i + 1]
        leg = legs[(a, b)]
        seq_legs.append(leg)
        total_cost += leg.cost
        if not states_all:
            states_all.extend(leg.states)
        else:
            states_all.extend(leg.states[1:])  # avoid duplicating the shared state
        prims_all.extend(leg.prims)

    return TourPlan(
        order=obstacles_in_order,
        approaches=approaches_in_order,
        legs=seq_legs,
        states_all=states_all,
        prims_all=prims_all,
        total_cost=total_cost,
    )


def plan_tour(scn: Scenario) -> TourPlan:
    """Top-level planner: derive approaches, A* between all stops, TSP order, stitch into a TourPlan."""
    total_obstacles = len(scn.obstacles)

    # Standoffs capped at 2-5 cells (20-50cm) per checklist A.2 recognition-distance spec.
    approaches_all, skipped = derive_approaches(
        scn.obstacles, scn.grid, standoffs=(2, 3, 4, 5)
    )
    if not approaches_all:
        return TourPlan([], [], [], [], [], 0.0, total_obstacles, skipped)

    stops: List[Pose] = [scn.start] + [ap.pose for ap in approaches_all]
    legs_dict, costs = precompute_legs(scn, stops)

    n_targets = len(approaches_all)
    node_order = choose_order(costs, n_targets)
    if node_order == [0]:
        unreachable = skipped + [(ap.obstacle, "no path from the start (blocked by obstacles)")
                                  for ap in approaches_all]
        return TourPlan([], [], [], [], [], 0.0, total_obstacles, unreachable)

    idxs = node_order[1:]
    obstacles_in_order: List[Obstacle] = [approaches_all[i - 1].obstacle for i in idxs]
    approaches_in_order: List[ApproachPose] = [approaches_all[i - 1] for i in idxs]

    # Prune to the reachable prefix if any hop in the chosen order turns out unreachable
    valid_until = len(node_order) - 1
    for i in range(len(node_order) - 1):
        a, b = node_order[i], node_order[i + 1]
        leg = legs_dict.get((a, b), None)
        if leg is None:
            valid_until = i
            break

    if valid_until < len(node_order) - 1:
        node_order = node_order[: valid_until + 1]
        idxs = node_order[1:]
        if not idxs:
            unreachable = skipped + [(ap.obstacle, "no path from the start (blocked by obstacles)")
                                      for ap in approaches_all]
            return TourPlan([], [], [], [], [], 0.0, total_obstacles, unreachable)
        obstacles_in_order = [approaches_all[i - 1].obstacle for i in idxs]
        approaches_in_order = [approaches_all[i - 1] for i in idxs]

    legs_needed: Dict[Tuple[int, int], PlanLeg] = {
        (a, b): legs_dict[(a, b)] for a, b in zip(node_order[:-1], node_order[1:])
    }
    tour = stitch(stops, legs_needed, node_order, obstacles_in_order, approaches_in_order)
    tour.total_obstacles = total_obstacles
    routed_cells = {ob.cell for ob in obstacles_in_order}
    tour.unreachable = skipped + [
        (ap.obstacle, "couldn't fit it into the route (blocked path from the rest of the tour)")
        for ap in approaches_all if ap.obstacle.cell not in routed_cells
    ]
    return tour

# -------- Scene validation (dict -> Pose/Obstacle, e.g. for a REST API layer) --------


class SceneValidationError(Exception):
    """Raised when a scene dict can't be converted into a valid Pose/Obstacle scene."""
    pass


def validate_scene_dict(data: dict) -> Tuple[Pose, List[Obstacle]]:
    """Validate a raw scene dict and convert it into (start_pose, obstacles)."""
    if not isinstance(data, dict) or "start" not in data or "obstacles" not in data:
        raise SceneValidationError("Request missing required 'start'/'obstacles' fields.")

    start = data["start"]
    if not isinstance(start, dict) or not all(k in start for k in ("x", "y", "heading")):
        raise SceneValidationError("Request missing start x/y/heading.")

    try:
        sx, sy = int(start["x"]), int(start["y"])
    except (TypeError, ValueError):
        raise SceneValidationError("Start x/y must be integers.")

    if not (0 <= sx < GRID_W and 0 <= sy < GRID_H):
        raise SceneValidationError(f"Start position ({sx},{sy}) is outside the {GRID_W}x{GRID_H} grid.")
    if not (ROBOT_HALF <= sx <= GRID_W - 1 - ROBOT_HALF and ROBOT_HALF <= sy <= GRID_H - 1 - ROBOT_HALF):
        raise SceneValidationError(
            f"Start position ({sx},{sy}) doesn't leave room for the robot's 3x3 "
            f"footprint inside the {GRID_W}x{GRID_H} grid; x and y must each be "
            f"between {ROBOT_HALF} and {GRID_W - 1 - ROBOT_HALF}."
        )

    heading_raw = str(start.get("heading", "")).strip().upper()
    if heading_raw not in ("N", "E", "S", "W"):
        raise SceneValidationError(f"Invalid start heading: {start.get('heading')!r}.")
    start_pose = Pose(sx, sy, Heading[heading_raw])

    obstacles_raw = data["obstacles"]
    if not isinstance(obstacles_raw, list):
        raise SceneValidationError("Request 'obstacles' must be a list.")

    seen: set = set()
    obstacles: List[Obstacle] = []
    for i, ob in enumerate(obstacles_raw):
        if not isinstance(ob, dict) or not all(k in ob for k in ("x", "y", "side")):
            raise SceneValidationError(f"Obstacle #{i + 1} is missing x/y/side.")
        try:
            ox, oy = int(ob["x"]), int(ob["y"])
        except (TypeError, ValueError):
            raise SceneValidationError(f"Obstacle #{i + 1} x/y must be integers.")
        if not (0 <= ox < GRID_W and 0 <= oy < GRID_H):
            raise SceneValidationError(
                f"Obstacle #{i + 1} at ({ox},{oy}) is outside the {GRID_W}x{GRID_H} grid."
            )
        side_raw = str(ob.get("side", "")).strip().upper()
        if side_raw not in ("N", "E", "S", "W"):
            raise SceneValidationError(f"Obstacle #{i + 1} has invalid side: {ob.get('side')!r}.")
        if (ox, oy) == (sx, sy):
            raise SceneValidationError(
                f"Obstacle #{i + 1} at ({ox},{oy}) overlaps the robot start position."
            )
        if (ox, oy) in seen:
            raise SceneValidationError(f"Duplicate obstacle position ({ox},{oy}).")
        seen.add((ox, oy))
        obstacles.append(Obstacle(cell=(ox, oy), side=Side(side_raw)))

    return start_pose, obstacles

# -------- Simulator UI (matplotlib) --------
import math
import sys
import time
from typing import Dict, Tuple, Optional, List

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch, ArrowStyle
from matplotlib.widgets import Button, RadioButtons
import numpy as np

GRID_W = 20
GRID_H = 20
INFLATION_R = ROBOT_HALF  # inflate obstacles by the robot's footprint half-width (C-space)
START_ZONE_SIZE = 4  # 4x4-cell (40cm x 40cm) start zone, bottom-left corner

F1 = Primitive(
    name="F1", dtheta=0, end_local=(1, 0),
    swept_local=[(1, 0)], cost=1.0, reverse_ok=False
)
B1 = Primitive(
    name="B1", dtheta=0, end_local=(-1, 0),
    swept_local=[(-1, 0)], cost=2.0, reverse_ok=True
)
L11 = Primitive(
    name="L(1,1)", dtheta=+1, end_local=(1, 1),
    swept_local=[(1, 0), (1, 1)], cost=3, reverse_ok=False
)
R11 = Primitive(
    name="R(1,1)", dtheta=-1, end_local=(1, -1),
    swept_local=[(1, 0), (1, -1)], cost=3, reverse_ok=False
)
L12 = Primitive(
    name="L(1,2)", dtheta=+1, end_local=(1, 2),
    swept_local=[(1, 0), (2, 0), (1, 1), (1, 2)], cost=4, reverse_ok=False
)
R12 = Primitive(
    name="R(1,2)", dtheta=-1, end_local=(1, -2),
    swept_local=[(1, 0), (2, 0), (1, -1), (1, -2)], cost=4, reverse_ok=False
)

PRIM_LIB = PrimitiveLibrary(name="basic", primitives=[F1, B1, L11, R11,])


class UIState:
    def __init__(self):
        self.mode = "obstacle"
        self.side = Side.N
        self.start_pose = Pose(1, 1, Heading.N)
        self.obstacles: Dict[Tuple[int, int], Side] = {}  # (x, y) -> Side
        self.grid = OccupancyGrid(GRID_W, GRID_H)

        self.fig = None
        self.ax = None
        self.im_bg = None
        self.rects: Dict[Tuple[int, int], Rectangle] = {}
        self.side_labels: Dict[Tuple[int, int], plt.Text] = {}
        self.side_edges: Dict[Tuple[int, int], "plt.Line2D"] = {}

        self.robot_rect: Optional[Rectangle] = None
        self.robot_heading_patch: Optional[FancyArrowPatch] = None

        self.path_line = None
        self.approach_markers: List[plt.Artist] = []
        self.timer = None
        self.anim_index = 0
        self.anim_states: List[Pose] = []

        # anim_states index (where a leg ends) -> Obstacle recognized there
        self.recognized_at_index: Dict[int, Obstacle] = {}
        self.recognized_reported: set = set()
        self.total_targets = 0
        self.recognized_text = None
        self.cost_text = None
        self.unreachable_text = None

        self.radio_mode = None
        self.radio_side = None
        self.btn_plan = None
        self.btn_clear = None
        self.btn_play = None
        self.btn_stop = None

    def toggle_obstacle(self, x: int, y: int):
        key = (x, y)
        if key in self.obstacles:
            del self.obstacles[key]
            self.grid.set_occ(x, y, False)
            self._remove_obstacle_patch(key)
        else:
            self.obstacles[key] = self.side
            self.grid.set_occ(x, y, True)
            self._add_obstacle_patch(key, self.side)
        self.redraw()

    def set_obstacle_side(self, x: int, y: int):
        key = (x, y)
        if key in self.obstacles:
            self.obstacles[key] = self.side
            self._update_side_label(key, self.side)
            self.redraw()

    def set_start(self, x: int, y: int):
        # keep the 3x3 footprint fully inside the arena even at the edges
        x = max(ROBOT_HALF, min(GRID_W - 1 - ROBOT_HALF, x))
        y = max(ROBOT_HALF, min(GRID_H - 1 - ROBOT_HALF, y))
        self.start_pose = Pose(x, y, self.start_pose.h)
        self._draw_robot(self.start_pose)

    def init_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(9.5, 8.6))
        # leave room on the right for the widget panel, and below for the status panel
        self.ax.set_position([0.06, 0.22, 0.68, 0.72])
        self.ax.set_aspect("equal")
        self.ax.set_xlim(0, GRID_W)
        self.ax.set_ylim(0, GRID_H)
        self.ax.set_xticks(range(GRID_W + 1))
        self.ax.set_yticks(range(GRID_H + 1))
        self.ax.grid(True, which='both', color='#ccc', linewidth=0.8)
        self.ax.set_title("MDP Algorithm Group 15", fontsize=11)

        self.im_bg = self.ax.imshow(
            [[0]*GRID_W for _ in range(GRID_H)],
            origin='lower', extent=(0, GRID_W, 0, GRID_H), alpha=0.0
        )

        # start zone marker, fixed to the arena corner
        self.ax.add_patch(Rectangle(
            (0, 0), START_ZONE_SIZE, START_ZONE_SIZE,
            facecolor="#cde8cd", edgecolor="#2e7d32",
            linewidth=1.5, alpha=0.6, zorder=0,
        ))
        self.ax.text(
            START_ZONE_SIZE / 2, START_ZONE_SIZE / 2, "START\nZONE",
            ha="center", va="center", fontsize=8, color="#2e7d32",
            fontweight="bold", zorder=0,
        )

        # Status panel below the plot (matches the browser version's layout),
        # rather than overlaid text inside the axes.
        self.fig.text(0.06, 0.13, "", fontsize=9, color="#227722",
                       fontweight="bold", va="top")
        self.recognized_text = self.fig.texts[-1]
        self.fig.text(0.06, 0.095, "", fontsize=9, color="#333333",
                       fontweight="bold", va="top")
        self.cost_text = self.fig.texts[-1]
        self.fig.text(0.06, 0.045, "", fontsize=8, color="#cc3333",
                       fontweight="bold", va="top", wrap=True)
        self.unreachable_text = self.fig.texts[-1]

        self._draw_robot(self.start_pose)

        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        self._add_widgets()

        plt.show()

    def _add_widgets(self):
        ax_mode = plt.axes([0.82, 0.55, 0.16, 0.2])
        self.radio_mode = RadioButtons(ax_mode, ("obstacle", "start", "side"), active=0)
        self.radio_mode.on_clicked(self._on_mode_change)

        ax_side = plt.axes([0.82, 0.35, 0.16, 0.17])
        self.radio_side = RadioButtons(ax_side, ("N", "E", "S", "W"), active=0)
        self.radio_side.on_clicked(self._on_side_change)

        ax_plan = plt.axes([0.82, 0.28, 0.16, 0.06])
        self.btn_plan = Button(ax_plan, "Plan route")
        self.btn_plan.on_clicked(self._on_plan)

        ax_play = plt.axes([0.82, 0.20, 0.16, 0.06])
        self.btn_play = Button(ax_play, "Play")
        self.btn_play.on_clicked(self._on_play)

        ax_stop = plt.axes([0.82, 0.12, 0.16, 0.06])
        self.btn_stop = Button(ax_stop, "Stop")
        self.btn_stop.on_clicked(self._on_stop)

        ax_clear = plt.axes([0.82, 0.04, 0.16, 0.06])
        self.btn_clear = Button(ax_clear, "Clear all")
        self.btn_clear.on_clicked(self._on_clear)

    def _on_mode_change(self, label):
        self.mode = label

    def _on_side_change(self, label):
        self.side = Side(label)

    # ------------- Drawing primitives -------------
    def _add_obstacle_patch(self, key: Tuple[int, int], side: Side):
        x, y = key
        r = Rectangle((x, y), 1, 1, facecolor="#444", edgecolor="#111")
        self.ax.add_patch(r)
        self.rects[key] = r
        t = self.ax.text(x + 0.5, y + 0.5, side.value, color="white",
                         ha="center", va="center", fontsize=10, fontweight="bold")
        self.side_labels[key] = t
        edge, = self.ax.plot(*self._side_edge_xy(x, y, side), color="#ff3333",
                             linewidth=4, solid_capstyle="round", zorder=3)
        self.side_edges[key] = edge

    @staticmethod
    def _side_edge_xy(x: int, y: int, side: Side):
        if side == Side.N:
            return [x, x + 1], [y + 1, y + 1]
        if side == Side.S:
            return [x, x + 1], [y, y]
        if side == Side.E:
            return [x + 1, x + 1], [y, y + 1]
        return [x, x], [y, y + 1]  # Side.W

    def _remove_obstacle_patch(self, key: Tuple[int, int]):
        if key in self.rects:
            self.rects[key].remove()
            del self.rects[key]
        if key in self.side_labels:
            self.side_labels[key].remove()
            del self.side_labels[key]
        if key in self.side_edges:
            self.side_edges[key].remove()
            del self.side_edges[key]

    def _update_side_label(self, key: Tuple[int, int], side: Side):
        if key in self.side_labels:
            self.side_labels[key].set_text(side.value)
        if key in self.side_edges:
            x, y = key
            xs, ys = self._side_edge_xy(x, y, side)
            self.side_edges[key].set_data(xs, ys)

    def _draw_robot(self, pose: Pose):
        if self.robot_rect is not None:
            self.robot_rect.remove()
            self.robot_rect = None
        if self.robot_heading_patch is not None:
            self.robot_heading_patch.remove()
            self.robot_heading_patch = None

        footprint = 2 * ROBOT_HALF + 1
        x0 = pose.x - ROBOT_HALF
        y0 = pose.y - ROBOT_HALF
        self.robot_rect = Rectangle((x0, y0), footprint, footprint, facecolor="none",
                                    edgecolor="#0077ff", linewidth=2)
        self.ax.add_patch(self.robot_rect)

        cx, cy = pose.x + 0.0, pose.y + 0.0
        dx, dy = {
            Heading.E: (0.8, 0.0),
            Heading.N: (0.0, 0.8),
            Heading.W: (-0.8, 0.0),
            Heading.S: (0.0, -0.8),
        }[pose.h]
        arr = FancyArrowPatch(
            (cx, cy), (cx + dx, cy + dy),
            arrowstyle=ArrowStyle("Simple,tail_width=0.5,head_width=6,head_length=8"),
            linewidth=1.5, color="#0077ff"
        )
        self.ax.add_patch(arr)
        self.robot_heading_patch = arr
        self.fig.canvas.draw_idle()

    def _draw_approach_markers(self, approaches):
        for a in self.approach_markers:
            a.remove()
        self.approach_markers.clear()

        for ap in approaches:
            px, py = ap.pose.x, ap.pose.y
            m = self.ax.plot(px, py, marker="o", markersize=6, linestyle="None",
                             markeredgecolor="black", markerfacecolor="#33cc66")[0]
            self.approach_markers.append(m)

    def _draw_path(self, states: List[Pose]):
        if self.path_line is not None:
            self.path_line.remove()
            self.path_line = None

        if not states:
            self.fig.canvas.draw_idle()
            return

        xs = [p.x for p in states]
        ys = [p.y for p in states]
        self.path_line, = self.ax.plot(xs, ys, linewidth=2.0)
        self.fig.canvas.draw_idle()

    def redraw(self):
        self.fig.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax:
            return
        x = int(event.xdata)
        y = int(event.ydata)
        if not (0 <= x < GRID_W and 0 <= y < GRID_H):
            return

        if self.mode == "obstacle":
            self.toggle_obstacle(x, y)
        elif self.mode == "start":
            self.set_start(x, y)
        elif self.mode == "side":
            self.set_obstacle_side(x, y)

    def on_key(self, event):
        if event.key in ("left", "right"):
            h = self.start_pose.h
            h = Heading((h + 1) % 4) if event.key == "left" else Heading((h - 1) % 4)
            self.start_pose = Pose(self.start_pose.x, self.start_pose.y, h)
            self._draw_robot(self.start_pose)

    def _on_clear(self, _):
        for k in list(self.obstacles.keys()):
            self._remove_obstacle_patch(k)
        self.obstacles.clear()
        self.grid.clear()

        if self.path_line is not None:
            self.path_line.remove()
            self.path_line = None
        for a in self.approach_markers:
            a.remove()
        self.approach_markers.clear()
        self.anim_states = []
        self.anim_index = 0
        self.recognized_at_index = {}
        self.recognized_reported = set()
        self.total_targets = 0
        self._update_recognized_text()
        self._update_unreachable_text([])
        if self.cost_text is not None:
            self.cost_text.set_text("")
        self._draw_robot(self.start_pose)
        self.redraw()

    def _update_recognized_text(self):
        if self.recognized_text is not None:
            n = len(self.recognized_reported)
            self.recognized_text.set_text(
                f"Recognized: {n}/{self.total_targets}" if self.total_targets else ""
            )
            self.redraw()

    def _on_plan(self, _):
        g = OccupancyGrid(GRID_W, GRID_H)
        for (x, y) in self.obstacles.keys():
            g.set_occ(x, y, True)
        g_inf = g.with_inflation(INFLATION_R)

        obs_list = [Obstacle(cell=k, side=v) for (k, v) in self.obstacles.items()]
        scn = Scenario(
            grid=g_inf,
            start=self.start_pose,
            obstacles=obs_list,
            primitive_lib=PRIM_LIB
        )

        tour = plan_tour(scn)

        print("-" * 80, "\n", tour.approaches)
        print("-" * 80, "\n", tour.states_all)
        print(f"[plan] total_cost = {tour.total_cost:.2f} "
              f"(relative time units; F1=1, B1=2, turn=3 -- not measured seconds)")

        self._draw_approach_markers(tour.approaches)
        self._draw_path(tour.states_all)

        self.anim_states = tour.states_all
        self.anim_index = 0

        # anim_states index where each leg ends -> the Obstacle recognized there
        self.recognized_at_index = {}
        running_idx = -1
        for i, leg in enumerate(tour.legs):
            running_idx += len(leg.states) if i == 0 else len(leg.states) - 1
            self.recognized_at_index[running_idx] = tour.order[i]
        self.recognized_reported = set()
        # Always report against every obstacle you placed, not just the ones
        # the tour managed to route to -- a dropped obstacle must show up as
        # X/Y being short, never as a silently-shrunk "X/X".
        self.total_targets = tour.total_obstacles
        self._update_recognized_text()
        self._update_unreachable_text(tour.unreachable)
        self._update_cost_text(tour)

        if self.anim_states:
            self._draw_robot(self.anim_states[0])
        elif not tour.order:
            print("No tour found (no valid approaches or no reachable targets).")

    def _update_cost_text(self, tour):
        if self.cost_text is None:
            return
        if not tour.order:
            self.cost_text.set_text("")
        else:
            n_moves = len(tour.prims_all)
            self.cost_text.set_text(
                f"Path cost: {tour.total_cost:.0f} units over {n_moves} moves "
                f"(F1=1, B1=2, turn=3 -- relative units, not seconds)"
            )
        self.redraw()

    def _update_unreachable_text(self, unreachable):
        if self.unreachable_text is None:
            return
        if not unreachable:
            self.unreachable_text.set_text("")
        else:
            lines = [f"Unreachable: {ob.cell} side={ob.side.value} -- {reason}"
                     for ob, reason in unreachable]
            self.unreachable_text.set_text("\n".join(lines))
            for ob, reason in unreachable:
                print(f"[plan] UNREACHABLE: obstacle at {ob.cell} side={ob.side.value}: {reason}")
        self.redraw()

    def _on_play(self, _):
        if not self.anim_states:
            return
        if self.timer is not None:
            self.timer.stop()
            self.timer = None

        self.timer = self.fig.canvas.new_timer(interval=200)  # ms
        self.timer.add_callback(self._step_anim)
        self.timer.start()

    def _on_stop(self, _):
        if self.timer is not None:
            self.timer.stop()
            self.timer = None

    def _step_anim(self):
        if not self.anim_states:
            return
        self.anim_index += 1
        if self.anim_index >= len(self.anim_states):
            self.anim_index = len(self.anim_states) - 1
            if self.timer is not None:
                self.timer.stop()
                self.timer = None
            return
        self._draw_robot(self.anim_states[self.anim_index])

        ob = self.recognized_at_index.get(self.anim_index)
        if ob is not None and self.anim_index not in self.recognized_reported:
            self.recognized_reported.add(self.anim_index)
            print(f"[recognize] Image found: obstacle at {ob.cell} side={ob.side.value} "
                  f"({len(self.recognized_reported)}/{self.total_targets})")
            self._update_recognized_text()


def main():
    ui = UIState()
    ui.init_plot()

if __name__ == "__main__":
    main()
