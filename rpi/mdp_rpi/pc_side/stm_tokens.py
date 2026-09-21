"""
pc_side/stm_tokens.py
─────────────────────
Token builders and Ackermann geometry for the STM32 movement protocol.

This module is the PC-side counterpart to communications/stm.py. It is
deliberately self-contained — no pyserial, no RPi imports — because pc_side/ is
deployed to the algorithm PC, a different machine from the Pi. The §2 caps are
therefore restated here rather than imported. If PROTOCOL.md changes, both
copies must change.

Everything here is real and tested. The live path selection in
task1_pc.compute_path is supplied by path_planner.py and uses these builders.

Protocol 3 adds fwd_until() (FU<n>, §4.1). It is the token to reach for when a
photo has to be taken from a known standoff: F0 trips on a stale reading and
scatters by 4-5 cm, FU measures from a standstill and lands within ±2 cm.
"""

import math
import re
from typing import List, Optional, Tuple

# ── PROTOCOL.md §2 — line caps ────────────────────────────────────────────────
MAX_PRIMITIVES = 16
MAX_LINE_BYTES = 128

# ── PROTOCOL.md §4.1 — FU<n> standoff bounds, cm ──────────────────────────────
# Restated here rather than imported, for the same reason the §2 caps are: this
# module ships to the algorithm PC and must not import pyserial. Keep in step
# with communications/stm.py.
#
# Out of range is a parse failure, not a clamp — so a planner that emits FU3
# does not get a short approach, it gets the whole line rejected.
FU_MIN_CM = 5
FU_MAX_CM = 200
FU_TOL_CM = 2

# The sensor reads about this much LONG (§4.1). FU removes the scatter but not
# this offset, because every pass of the approach reads through it — ask for
# FU20 and the robot settles ~18.7 cm out, repeatably. fwd_until(compensate=True)
# corrects for it. MEASURE IT ON THE DAY before trusting this number.
US_BIAS_CM = 1.3

# ── PROTOCOL.md §8 — measured turn radii, by floor chord ──────────────────────
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

# ── PROTOCOL.md §8 — chassis ──────────────────────────────────────────────────
ROBOT_WIDTH_CM = 18.8
ROBOT_LENGTH_CM = 23.0
ARENA_CM = 200.0

# FU before F, exactly as the firmware's own parser orders its prefixes — "fu"
# would otherwise be eaten by "f".
_TOKEN_RE = re.compile(r"^(FR|FL|FU|RR|RL|F|R)(\d+)$|^(S|RST)$", re.IGNORECASE)


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


def fwd_until(cm: int, compensate: bool = False) -> str:
    """
    FU<n> — forward until the ultrasound reads n cm, protocol 3 (§4.1).

    Prefer this to F0 whenever the standoff matters. F0 is a trip-wire on a
    reading that is already ~120 ms old, so it overshoots by a different 3.9 to
    5.3 cm every run; FU measures from a standstill and closes the gap on
    odometry, landing within FU_TOL_CM.

    It buys that with time — roughly 1.2 s for a short approach, 4 s from a
    metre — so do not send it for the whole journey. F150,FU20 rather than
    FU20 from across the arena, which also keeps the measurement inside the
    range where the beam is still narrow enough to trust.

    compensate=True subtracts the sensor's known long bias, so the robot ends
    up n cm from the obstacle rather than n cm by the sensor's reckoning. Off
    by default: US_BIAS_CM is a measurement, and silently shifting every
    approach by an unverified constant is worse than a known offset.

    Raises TokenError outside FU_MIN_CM..FU_MAX_CM — the firmware treats that
    as a parse failure and RESENDs the entire line, so catching it here is the
    difference between one bad segment and a stalled mission.
    """
    target = int(round(cm + US_BIAS_CM)) if compensate else int(round(cm))
    if not (FU_MIN_CM <= target <= FU_MAX_CM):
        raise TokenError(
            f"fwd_until({cm}): {target} cm is outside {FU_MIN_CM}..{FU_MAX_CM} "
            "(PROTOCOL.md §4.1) — the firmware rejects the whole line rather "
            "than clamping"
        )
    return f"FU{target}"


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
    # FU carries a standoff, not a travel distance, so it has a floor as well
    # as a ceiling (§4.1). Checked before the zero rule, which would otherwise
    # pass FU0 — legal-looking to that rule and a parse failure on the wire.
    if op.upper() == "FU":
        return FU_MIN_CM <= value <= FU_MAX_CM
    return not (value == 0 and op.upper() != "F")


# ── Ackermann geometry (PROTOCOL.md §8) ───────────────────────────────────────
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
