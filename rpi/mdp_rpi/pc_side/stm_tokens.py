"""
pc_side/stm_tokens.py
─────────────────────
Token builders and Ackermann geometry for the STM32 movement protocol.

This module is the PC-side counterpart to communications/stm.py. It is
deliberately self-contained — no pyserial, no RPi imports — because pc_side/ is
deployed to the algorithm PC, a different machine from the Pi. The §2 caps are
therefore restated here rather than imported. If PROTOCOL.md changes, both
copies must change.

Everything here is real and tested. The path *selection* that consumes it (in
task1_pc.compute_path) is still a stub — see the note there.
"""

import math
import re
from typing import List, Optional, Tuple

# ── PROTOCOL.md §2 — line caps ────────────────────────────────────────────────
MAX_PRIMITIVES = 16
MAX_LINE_BYTES = 128

# ── PROTOCOL.md §7 — measured turn radii, by floor chord ──────────────────────
# The planner MUST use the radius of the profile the robot is actually running
# (task1.Task1.arc_profile / STM_ARC_PROFILE). Planning a TIGHT path and running
# it on CLEAN puts every turn 27mm wide, and the error compounds across turns.
PROFILE_TIGHT, PROFILE_CLEAN, PROFILE_SLOW = 0, 1, 2
TURN_RADIUS_MM = {
    PROFILE_TIGHT: 291,
    PROFILE_CLEAN: 318,
    PROFILE_SLOW: 306,
}
PROFILE_NAMES = {PROFILE_TIGHT: "TIGHT", PROFILE_CLEAN: "CLEAN", PROFILE_SLOW: "SLOW"}

# ── PROTOCOL.md §7 — chassis ──────────────────────────────────────────────────
ROBOT_WIDTH_CM = 18.8
ROBOT_LENGTH_CM = 23.0
ARENA_CM = 200.0

_TOKEN_RE = re.compile(r"^(FR|FL|RR|RL|F|R)(\d+)$|^(S|RST)$", re.IGNORECASE)


class TokenError(ValueError):
    """Raised when a token cannot be built legally."""


# ── Token builders ────────────────────────────────────────────────────────────
# PROTOCOL.md §4: arguments are unsigned decimal only. F-5, F1x and a bare F are
# parse failures, NOT clamped. These builders raise rather than emit something
# the firmware would reject with RESEND.

def fwd(cm: int) -> str:
    """Forward n cm. fwd(0) is legal and means 'until obstacle' (ultrasonic, 15cm)."""
    cm = int(round(cm))
    if cm < 0:
        raise TokenError(f"fwd({cm}): negative distance — use rev() instead")
    return f"F{cm}"


def fwd_until_obstacle() -> str:
    """F0 — forward until the ultrasonic stops it at 15cm. §4."""
    return "F0"


def rev(cm: int) -> str:
    """Reverse n cm. Zero is NOT legal for R (§4)."""
    cm = int(round(cm))
    if cm <= 0:
        raise TokenError(f"rev({cm}): must be positive — zero is only valid for F")
    return f"R{cm}"


def arc(forward: bool, right: bool, degrees: int) -> str:
    """
    Arc token. forward/right select FR / FL / RR / RL.
    Zero degrees is NOT legal for arcs (§4).
    """
    degrees = int(round(degrees))
    if degrees <= 0:
        raise TokenError(f"arc({degrees}): must be positive — zero is only valid for F")
    if degrees > 360:
        raise TokenError(f"arc({degrees}): more than a full turn")
    return f"{'F' if forward else 'R'}{'R' if right else 'L'}{degrees}"


def stop() -> str:
    """S — brake and recentre steering. Replies OK. NOT backward; that is rev()."""
    return "S"


def is_valid_token(token: str) -> bool:
    m = _TOKEN_RE.match(token.strip())
    if not m:
        return False
    op, digits = m.group(1), m.group(2)
    if op is None:
        return True  # S or RST
    value = int(digits)
    return not (value == 0 and op.upper() != "F")


# ── Ackermann geometry (PROTOCOL.md §7) ───────────────────────────────────────
# The chassis CANNOT turn on the spot. Every turn is an arc with forward or
# reverse travel, and the swept area has to be planned for.

def arc_displacement(degrees: float, profile: int = PROFILE_TIGHT) -> Tuple[float, float]:
    """
    Displacement of an arc in the robot's own frame, in mm: (along, lateral).

    For radius R and angle t: along = R*sin(t), lateral = R*(1-cos(t)).
    At 90 degrees both equal R — which is why a single 90 degree turn at 291mm
    eats roughly a seventh of a 2m arena in each axis.
    """
    radius = TURN_RADIUS_MM[profile]
    t = math.radians(degrees)
    return radius * math.sin(t), radius * (1.0 - math.cos(t))


def arc_swept_box(degrees: float, profile: int = PROFILE_TIGHT) -> Tuple[float, float]:
    """
    Bounding box (mm) the robot's centre sweeps through during the arc. A
    conservative clearance check should inflate this by the chassis half-width
    (ROBOT_WIDTH_CM * 10 / 2) on every side.
    """
    radius = TURN_RADIUS_MM[profile]
    t = math.radians(min(abs(degrees), 360.0))
    # Centre traces a circular arc; the extreme along-axis point is at t=90deg.
    along = radius * (1.0 if t >= math.pi / 2 else math.sin(t))
    lateral = radius * (1.0 - math.cos(t))
    return along, lateral


def arc_fits_in_arena(x_mm: float, y_mm: float, degrees: float,
                      profile: int = PROFILE_TIGHT) -> bool:
    """Crude guard: would this arc, starting at (x,y), leave the arena?"""
    along, lateral = arc_swept_box(degrees, profile)
    margin = ROBOT_WIDTH_CM * 10 / 2
    arena_mm = ARENA_CM * 10
    return (
        0 <= x_mm - lateral - margin and x_mm + lateral + margin <= arena_mm
        and 0 <= y_mm - along - margin and y_mm + along + margin <= arena_mm
    )


# ── Line assembly (PROTOCOL.md §2) ────────────────────────────────────────────

def line_bytes(tokens: List[str]) -> int:
    """Wire size of a line, including the terminating newline."""
    return len((",".join(tokens) + "\n").encode("utf-8"))


def chunk_tokens(tokens: List[str]) -> List[List[str]]:
    """
    Split a token stream into legal lines: at most MAX_PRIMITIVES primitives and
    MAX_LINE_BYTES bytes each.

    This is why segments are no longer 1:1 with obstacles. A single obstacle
    approach that needs more than 16 primitives becomes several lines, each
    answered with its own OK, and only the LAST of them should trigger a photo.
    """
    if not tokens:
        return []

    lines: List[List[str]] = []
    current: List[str] = []

    for tok in tokens:
        if not is_valid_token(tok):
            raise TokenError(f"refusing to emit invalid token {tok!r}")
        candidate = current + [tok]
        if len(candidate) > MAX_PRIMITIVES or line_bytes(candidate) > MAX_LINE_BYTES:
            if not current:
                raise TokenError(f"single token {tok!r} cannot fit a line")
            lines.append(current)
            current = [tok]
        else:
            current = candidate

    if current:
        lines.append(current)
    return lines
