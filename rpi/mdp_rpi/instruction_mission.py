"""Execute primitives and report ROBOT,x_grid,y_grid,heading_deg to Android.

Grid units are 100 mm; heading is north-zero, clockwise, in [0, 360).
The arena start anchor comes from the PC planner's PATH.odometry_start object.
No axle-to-chassis-centre conversion or automatic replanning is performed here.

After each successful OK/pose read, `progress` identifies the completed token
using zero-based PATH segments[i][j] indexes. Task1 sends it as PROGRESS,<json>
to PC only, before sending the next token. Task1 optionally adds a correlated
feedback_id and waits for a PC decision when TASK1_FEEDBACK_WAIT is enabled.
`segment` contains the full completed token's segment, including instructions
not yet executed. Null next indexes mean motion is finished, not that final
image processing is done. There is no execution ID or plan revision field.
"""

import math
import re

from communications.stm import validate_line

GRID_CELLS = 20
CELL_SIZE_MM = 100.0  # 10 cm per cell; the arena is 2000 mm square.


_CARDINAL_BEARINGS = {
    "N": 0.0, "NORTH": 0.0,
    "E": 90.0, "EAST": 90.0,
    "S": 180.0, "SOUTH": 180.0,
    "W": 270.0, "WEST": 270.0,
}


def parse_start_pose(start_pose):
    """Validate a planner-provided grid pose used by the Task 1 protocol."""
    if not isinstance(start_pose, dict):
        raise ValueError("PATH must include planner start {x, y, dir}")
    try:
        x = float(start_pose["x"])
        y = float(start_pose["y"])
        raw_bearing = start_pose["dir"]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("PATH start must contain numeric x/y and dir") from exc
    if not all(math.isfinite(value) for value in (x, y)):
        raise ValueError("PATH start coordinates must be finite")
    if not (0 <= x < GRID_CELLS and 0 <= y < GRID_CELLS):
        raise ValueError("PATH start is outside the 20x20 arena")

    label = str(raw_bearing).strip()
    bearing = _CARDINAL_BEARINGS.get(label.upper())
    if bearing is None:
        try:
            bearing = float(label)
        except ValueError as exc:
            raise ValueError("PATH start direction must be a bearing or N/E/S/W") from exc
        if not math.isfinite(bearing):
            raise ValueError("PATH start bearing must be finite")
        bearing %= 360.0
    return {
        "x": x,
        "y": y,
        "dir": raw_bearing,
        "bearing_deg": bearing,
    }


def arena_grid_position(position_mm):
    # Remove floating-point rotation noise at exact cell boundaries.
    position_mm = round(position_mm, 6)
    if not math.isfinite(position_mm):
        raise ValueError("Measured arena position must be finite")
    # Preserve the true fractional coordinate instead of rounding to a cell.
    # The PC preflight prevents planned boundary crossings; an out-of-range
    # report remains valuable evidence of physical drift and must not be hidden.
    return position_mm / CELL_SIZE_MM


class InstructionMission:
    def __init__(self, segments, start_pose):
        if not isinstance(segments, list) or not segments:
            raise ValueError("PATH must contain nonempty segments")
        self.segments = []
        for segment in segments:
            if not isinstance(segment, list) or not all(isinstance(t, str) for t in segment):
                raise ValueError("Each segment must be a list of instruction strings")
            tokens = [t.strip().upper() for t in segment]
            valid, reason = validate_line(tokens)
            if not valid:
                raise ValueError(reason)
            if any(not re.fullmatch(r"(?:FR|FL|RR|RL|FU|F|R)\d+|S", t) for t in tokens):
                raise ValueError("PATH may contain only movement instructions and S")
            self.segments.append(tokens)
        self.segment_index = 0
        self.instruction_index = 0
        self.pending = False
        self.origin = None
        # Every segment is checked immediately before its first primitive.
        # Keeping the set on the mission also means a replacement route starts
        # with no inherited clearance decisions from the route it replaced.
        self.preflighted_segments = set()
        self.progress = None
        self.start_pose = parse_start_pose(start_pose)

    @property
    def done(self):
        return self.segment_index == len(self.segments)

    @property
    def key(self):
        return self.segment_index, self.instruction_index

    @property
    def token(self):
        return self.segments[self.segment_index][self.instruction_index]

    @staticmethod
    def read_pose(stm):
        fields = stm.query_fields("?WPOSE")
        if fields is None or len(fields) != 3:
            raise ValueError("STM must support ?WPOSE continuous odometry; no valid reply received")
        try:
            x, y, heading = (int(value) for value in fields)
        except (ValueError, TypeError) as exc:
            raise ValueError("Invalid STM WPOSE coordinates") from exc
        if not -1800 <= heading <= 1800:
            raise ValueError("Invalid STM WPOSE heading")
        return x, y, heading / 10.0

    def send_next(self, stm):
        if self.pending or self.done:
            return False
        if self.origin is None:
            # Anchor the STM boot-relative frame at this planner-provided start.
            self.origin = self.read_pose(stm)
        if not stm.send_line([self.token]):
            return False
        self.pending = True
        return True

    def complete_instruction(self, stm):
        if not self.pending:
            raise ValueError("Unexpected OK without an outstanding instruction")
        message, pose = self.report_pose(stm)
        x_grid = pose["x_grid"]
        y_grid = pose["y_grid"]
        heading_deg = pose["heading_deg"]
        # Snapshot the completed instruction before advancing either index.
        segment_index, instruction_index = self.key
        token = self.token
        self.pending = False
        self.instruction_index += 1
        completed_segment = None
        if self.instruction_index == len(self.segments[self.segment_index]):
            completed_segment = self.segment_index
            self.segment_index += 1
            self.instruction_index = 0
        self.progress = {
            "event": "instruction_completed",
            "segment_index": segment_index,
            "instruction_index": instruction_index,
            "token": token,
            "segment": list(self.segments[segment_index]),
            "segment_completed": completed_segment is not None,
            "motion_plan_completed": self.done,
            "next_segment_index": None if self.done else self.segment_index,
            "next_instruction_index": None if self.done else self.instruction_index,
            "x_grid": x_grid,
            "y_grid": y_grid,
            "heading_deg": heading_deg,
        }
        return message, completed_segment

    def report_pose(self, stm):
        """Read and transform WPOSE without advancing the planned instruction."""
        x, y, heading = self.read_pose(stm)
        if self.origin is None:
            raise ValueError("Cannot report pose before the mission odometry origin is set")
        ox, oy, oh = self.origin
        start_bearing = self.start_pose["bearing_deg"]
        angle = math.radians(90.0 - start_bearing - oh)
        dx, dy = x - ox, y - oy
        arena_x = self.start_pose["x"] * CELL_SIZE_MM + dx * math.cos(angle) - dy * math.sin(angle)
        arena_y = self.start_pose["y"] * CELL_SIZE_MM + dx * math.sin(angle) + dy * math.cos(angle)
        # Wire heading: north = 0 degrees, clockwise positive, in [0, 360).
        heading_deg = round((start_bearing - (heading - oh)) % 360.0, 1) % 360.0
        # Preserve sub-cell positions; one grid unit is 100 mm, not one mm.
        x_grid, y_grid = (arena_grid_position(mm) for mm in (arena_x, arena_y))
        gx, gy = (format(value, ".8f").rstrip("0").rstrip(".")
                  for value in (x_grid, y_grid))
        message = f"ROBOT,{gx},{gy},{heading_deg:g}"
        return message, {
            "x_grid": x_grid,
            "y_grid": y_grid,
            "heading_deg": heading_deg,
        }
