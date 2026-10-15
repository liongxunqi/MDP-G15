"""
task2mn_gordon_link.py - firmware protocol 10 (per-move speed, continuous
lines) for gordon, in the style of task2mn_link.py.

PROTOCOL.md section 13 has the wire side. In short, any F/R/FR/FL/RR/RL token
may end in V<n> (cruise n x 10 rpm for this move) and then C (do not brake:
roll into the next token). A token without a suffix is its protocol 9 self.

    f120v20c,fr90v15c,f40v15    120 cm at 200 rpm, into a 90 deg right arc at
                                150 rpm, into 40 cm at 150 rpm, then stop

This module is pure Python apart from enable() and read_seg():

  - Move, tokens, check_line(): the token grammar and the firmware parser's
    line rules, as a reference model - a line check_line() accepts, the
    firmware accepts.
  - fly_path(): where a line actually takes the robot. A hand-over is not a
    corner: the steering swings while the wheels turn, so each continued move
    starts with a short spiral, and a continued arc lets go early by the
    yaw it will still gain while the steering swings. Arc radius depends on
    speed. The planner checks clearance on this path, and the simulator drives
    it (with its own, deliberately different, numbers).
  - line_seconds(): time for a line under one speed model, applied the same way
    to protocol 9 lines, so the two can be compared.

Calibration (PROTOCOL.md 13.4): R_AT_V, STEER_SLEW_S and the hand-over model
are NOT MEASURED. Until they are, gordon runs arcs at V15 - TIGHT's own speed,
whose radius is calibrated - and only straights fast.
"""

import math
import os
import re
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import task2mn_park as tp
from task2mn_park import Pose, Seg

PROTOCOL = 10
LINE_MAX_BYTES = 192
MAX_PRIMITIVES = 16
V_STRAIGHT = (4, 25)            # x10 rpm
V_ARC = (6, 17)
CM_S_PER_RPM = 0.34505          # wheel travel, motion.h MOTION_MM_S_PER_RPM / 10


def _env_f(name: str, default: float) -> float:
    return float(os.getenv(name, str(default)))


# Steering servo: a full arc deflection takes this long to swing (motion.h
# MOTION_STEER_SLEW_S - keep the two the same). NOT MEASURED.
STEER_SLEW_S = _env_f("T2G_STEER_SLEW_S", 0.08)
# Planner radius at each arc speed, cm. Only V15 (TIGHT's own 150 rpm) is
# measured - it is task2mn_park.RADIUS_CM. The others grow with speed (the
# tyres slip more); placeholders for the floor-chord runs of PROTOCOL.md
# 13.4 step 1, +4 % per 100 rpm.
UNDERSTEER_PER_100RPM = _env_f("T2G_UNDERSTEER_PER_100RPM", 0.04)
# Straights: the heading hold steers a residual out over about this far.
HOLD_CM = 15.0

_TOKEN_RE = re.compile(r"^(FR|FL|RR|RL|F|R)(\d+)(?:V(\d+))?(C?)$", re.IGNORECASE)
_KIND = {"F": "F", "R": "B", "FL": "L", "FR": "R", "RL": "l", "RR": "r"}


def radius_at(speed: int, r150: float = tp.RADIUS_CM,
              k: float = UNDERSTEER_PER_100RPM) -> float:
    """Arc radius at V<speed> (x10 rpm); 0 = the profile's own 150 rpm."""
    v = 15 if speed == 0 else speed
    return r150 * (1.0 + k * (v - 15) / 10.0)


@dataclass(frozen=True)
class Move:
    """A Seg with protocol 10's speed (x10 rpm, 0 = protocol 9) and carry."""
    seg: Seg
    speed: int = 0
    carry: bool = False

    def token(self) -> str:
        return (self.seg.token() + (f"V{self.speed}" if self.speed else "")
                + ("C" if self.carry else ""))

    @property
    def arc(self) -> bool:
        return self.seg.kind in "LRlr"

    @property
    def forward(self) -> bool:
        return self.seg.kind in "FLR"


def tokens(moves: Sequence[Move]) -> str:
    return ",".join(m.token() for m in moves)


def parse_token(tok: str) -> Move:
    m = _TOKEN_RE.match(tok.strip())
    if not m:
        raise ValueError(f"not a movement token: {tok!r}")
    op, n, v, c = m.group(1).upper(), m.group(2), m.group(3), m.group(4)
    if len(n) > 1 and n[0] == "0":
        raise ValueError(f"leading zero in {tok!r}")
    if v is not None and (v[0] == "0"):
        raise ValueError(f"bad speed in {tok!r}")
    return Move(Seg(_KIND[op], int(n)), int(v) if v else 0, bool(c))


def check_token(tok: str) -> Tuple[bool, str]:
    try:
        m = parse_token(tok)
    except ValueError as exc:
        return False, str(exc)
    if m.carry and not m.speed:
        return False, f"'{tok}': C needs a V"
    if m.seg.value == 0 and (m.seg.kind != "F" or m.speed):
        return False, f"'{tok}': a zero argument is only F0, with no suffix"
    if m.speed:
        lo, hi = V_ARC if m.arc else V_STRAIGHT
        if not lo <= m.speed <= hi:
            return False, f"'{tok}': V{m.speed} outside V{lo}..V{hi}"
    return True, ""


def check_line(toks: Sequence[str]) -> Tuple[bool, str]:
    """The firmware's Cmd_ParseLine rules for movement lines (protocol 10)."""
    if not toks:
        return False, "empty line"
    if len(toks) > MAX_PRIMITIVES:
        return False, f"{len(toks)} primitives, over {MAX_PRIMITIVES}"
    if len((",".join(toks) + "\n").encode("ascii")) > LINE_MAX_BYTES:
        return False, f"over {LINE_MAX_BYTES} bytes"
    moves = []
    for t in toks:
        ok, why = check_token(t)
        if not ok:
            return False, why
        moves.append(parse_token(t))
    for i, m in enumerate(moves):
        if not m.carry:
            continue
        if i + 1 >= len(moves):
            return False, "C on the last token: a line ends stopped"
        nxt = moves[i + 1]
        if not nxt.speed:
            return False, f"'{toks[i + 1]}' follows a C and has no V"
        if nxt.forward != m.forward:
            return False, "C into a change of direction"
    return True, ""


def exit_speed(moves: Sequence[Move], i: int) -> int:
    """The hand-over speed the firmware fills in for a C token (x10 rpm)."""
    m = moves[i]
    if not m.carry:
        return 0
    return min(m.speed, moves[i + 1].speed)


# ═════════════════════════════════════════════════════════════════════════════
# Where a line goes
# ═════════════════════════════════════════════════════════════════════════════

def _kappa(m: Move, r_at: Callable[[int], float]) -> float:
    """Signed path curvature, 1/cm, CCW positive per cm of signed travel."""
    if not m.arc:
        return 0.0
    k = 1.0 / r_at(m.speed)
    return k if m.seg.kind in "Ll" else -k


def fly_path(p: Pose, moves: Sequence[Move], r_at: Callable[[int], float] = radius_at,
             slew_s: float = STEER_SLEW_S, step: float = 1.0,
             ends: Optional[List[int]] = None) -> List[Pose]:
    """
    Every pose along a protocol 10 line, `step` cm apart. A move that starts
    from rest (the first one, or after a stop) is exactly its ideal shape. A
    move that starts on the fly begins with a spiral: curvature changing
    linearly from the last move's to its own over the distance the steering
    swing takes at the hand-over speed. Arcs end on HEADING (the gyro), at the
    commanded total - less, for a C arc, half the yaw rate times the swing it
    lets go early by; straights end on distance from their launch and hold the
    commanded heading (the firmware's carry-over). With moves all at rest this
    is task2mn_park.run() exactly. `ends`, if given, collects the index in
    the returned list at which each move ends.
    """
    out = [p]
    x, y, th = p.x, p.y, p.th
    cmd = p.th
    prev_k = 0.0
    k_full = 1.0 / tp.RADIUS_CM
    for i, m in enumerate(moves):
        sgn = 1.0 if m.forward else -1.0
        k = _kappa(m, r_at)
        cmd_end = cmd + math.radians(m.seg.turn_deg)
        flying = i > 0 and moves[i - 1].carry
        v_in = (exit_speed(moves, i - 1) if flying else 0) * 10 * CM_S_PER_RPM
        s_t = (v_in * slew_s * abs(k - prev_k) / k_full) if flying else 0.0
        k0 = prev_k if flying else k
        v = (m.speed or 15) * 10 * CM_S_PER_RPM
        lead = 0.5 * v * abs(k) * slew_s if (m.arc and m.carry) else 0.0
        turn = 1.0 if m.seg.turn_deg > 0 else -1.0
        s = 0.0
        guard = 0
        while True:
            guard += 1
            if guard > 20000:
                raise RuntimeError("fly_path ran away")
            if m.arc:
                togo = (cmd_end - th) * turn
                if togo <= lead + 1e-9:
                    break
            elif s >= m.seg.value - 1e-9:
                break
            ds = step
            if not m.arc:
                ds = min(step, m.seg.value - s)
            if s < s_t:
                kk = k0 + (k - k0) * min(1.0, (s + ds / 2) / s_t)
            elif m.arc:
                kk = k
            else:
                # Heading hold back onto the commanded heading.
                err = math.atan2(math.sin(cmd_end - th), math.cos(cmd_end - th))
                kk = max(-k_full, min(k_full, err / HOLD_CM)) * sgn
            dth = kk * ds * sgn
            if m.arc:
                togo = (cmd_end - th) * turn
                if dth * turn > togo - lead and abs(kk) > 1e-9:
                    ds = max(0.0, (togo - lead) / abs(kk))
                    dth = kk * ds * sgn
            x += sgn * ds * math.cos(th + dth / 2)
            y += sgn * ds * math.sin(th + dth / 2)
            th += dth
            s += ds
            out.append(Pose(x, y, th))
            if ds <= 1e-9:
                break
        prev_k = k if m.arc else 0.0
        cmd = cmd_end
        if ends is not None:
            ends.append(len(out) - 1)
    return out


# ═════════════════════════════════════════════════════════════════════════════
# How long a line takes
# ═════════════════════════════════════════════════════════════════════════════

# One speed model for protocol 9 and 10 lines alike, from motion.h: straights
# ramp at 4 rpm/tick up and 6 down (fast mode), stop through a 12 cm tail at
# 40 rpm; arcs run at their speed, a stop slowing to 100 rpm for the last
# 20 deg. ARC_SLIP scales arc speed so a protocol 9 FR90 costs what
# task2mn_park.motion_cost says (its 30.9 cm/s estimate). Starting from rest
# costs the steering settle, stopping the brake settle: together
# task2mn_park's 0.5 s per-primitive overhead. A protocol 9 straight in fast
# mode is a V20 that stops; a protocol 9 arc a V15 that stops.
ALIGN_S = 0.2
BRAKE_S = tp.PRIMITIVE_OVERHEAD_S - ALIGN_S
ACCEL_CM_S2 = 4 * 100 * CM_S_PER_RPM
DECEL_CM_S2 = 6 * 100 * CM_S_PER_RPM
TAIL_CM = 12.0
APPROACH_CM_S = 40 * CM_S_PER_RPM
_ARC_STOP_S = (math.radians(70) * tp.RADIUS_CM / (150 * CM_S_PER_RPM)
               + math.radians(20) * tp.RADIUS_CM / (100 * CM_S_PER_RPM))
ARC_SLIP = _ARC_STOP_S / (math.radians(90) * tp.RADIUS_CM / tp.ARC_CM_S)


def _straight_s(d: float, v0: float, vc: float, ve: float) -> float:
    """Trapezoid from v0 to cruise vc and down to ve over d, cm/s; seconds."""
    vc = max(vc, v0, ve)
    up = max(0.0, (vc * vc - v0 * v0) / (2 * ACCEL_CM_S2))
    dn = max(0.0, (vc * vc - ve * ve) / (2 * DECEL_CM_S2))
    if up + dn > d:
        # Never reaches cruise: peak where the two ramps meet.
        vp2 = ((2 * ACCEL_CM_S2 * DECEL_CM_S2 * d + DECEL_CM_S2 * v0 * v0
                + ACCEL_CM_S2 * ve * ve) / (ACCEL_CM_S2 + DECEL_CM_S2))
        vp = math.sqrt(max(vp2, v0 * v0, ve * ve))
        return (max(0.0, vp - v0) / ACCEL_CM_S2 + max(0.0, vp - ve) / DECEL_CM_S2)
    return ((vc - v0) / ACCEL_CM_S2 + (vc - ve) / DECEL_CM_S2
            + (d - up - dn) / vc)


def line_seconds(moves: Sequence[Move], fast: bool = True,
                 done_cm: Optional[float] = None) -> float:
    """Seconds for a line. Protocol 9 moves (speed 0) are priced as the
    firmware runs them: straights as fast mode (or 100 rpm with fast False),
    arcs at the profile's 150 rpm. done_cm cuts the line short (an RST),
    priced to where it was cut plus a brake."""
    total = 0.0
    v_in = 0.0
    travelled = 0.0
    for i, m in enumerate(moves):
        flying = i > 0 and moves[i - 1].carry
        if not flying:
            total += ALIGN_S
            v_in = 0.0
        length = (math.radians(m.seg.value) * tp.RADIUS_CM if m.arc
                  else float(m.seg.value))
        cut = done_cm is not None and travelled + length > done_cm
        if cut:
            length = max(0.0, done_cm - travelled)
        travelled += length
        if m.arc:
            v = (m.speed or 15) * 10 * CM_S_PER_RPM * ARC_SLIP
            if m.carry or cut:
                total += length / v
            else:
                app = min(length, math.radians(20) * tp.RADIUS_CM)
                total += ((length - app) / v
                          + app / (100 * CM_S_PER_RPM * ARC_SLIP))
            v_out = v
        else:
            vc = (m.speed * 10 if m.speed else (200 if fast else 100)) * CM_S_PER_RPM
            if m.carry:
                ve = exit_speed(moves, i) * 10 * CM_S_PER_RPM
                total += _straight_s(length, v_in, vc, ve)
                v_out = ve
            elif cut:
                total += _straight_s(length, v_in, vc, vc)
                v_out = 0.0
            else:
                tail = min(TAIL_CM, length)
                total += (_straight_s(length - tail, v_in, vc, APPROACH_CM_S)
                          + tail / APPROACH_CM_S)
                v_out = 0.0
        if cut:
            return total + BRAKE_S + 0.4
        if not m.carry:
            total += BRAKE_S
        v_in = v_out
    return total


# ═════════════════════════════════════════════════════════════════════════════
# On the robot
# ═════════════════════════════════════════════════════════════════════════════

def enable() -> None:
    """Teach communications.stm the protocol 10 tokens, ?SEG and the longer
    line (it predates them), on top of task2mn_link's protocol 9 commands.
    Idempotent."""
    import task2mn_link
    from communications import stm as stm_module

    task2mn_link.enable()
    stm_module.QUERY_TAGS.update({"SEG"})
    stm_module.MAX_LINE_BYTES = max(stm_module.MAX_LINE_BYTES, LINE_MAX_BYTES)
    original = stm_module.validate_token
    if getattr(original, "_gordon", False):
        return

    def validate_token(token):
        tok = str(token).strip()
        m = _TOKEN_RE.match(tok)
        if m and (m.group(3) is not None or m.group(4)):
            return check_token(tok)
        return original(token)

    validate_token._gordon = True
    stm_module.validate_token = validate_token


def read_seg(stm, timeout: float = 0.5) -> Optional[Dict[str, int]]:
    """?SEG: which primitive of the running line is executing."""
    fields = stm.query_fields("?SEG", timeout)
    try:
        n, left, state = (int(f) for f in fields[:3])
    except (TypeError, ValueError):
        return None
    return {"index": n, "left": left, "state": state}


def trim_landing(p: Pose, moves: List[Move], land_y: float,
                 r_at: Callable[[int], float] = radius_at, span: int = 6) -> List[Move]:
    """
    Continuing from one arc straight into the opposite one swings the
    steering through twice its deflection on the move, and the spiral that
    leaves shifts everything after it sideways (4-5 cm through the kick of a
    lane change at V15). For a line that must END on a lane - y = land_y,
    heading along x - grow or shrink both arcs of its last such pair by the
    same whole degrees (so the heading is unchanged) until the flying path
    lands there. Moves come back unchanged if there is no such pair.
    """
    pair = None
    for i in range(len(moves) - 1):
        a, b = moves[i], moves[i + 1]
        if (a.carry and a.arc and b.arc
                and (a.seg.turn_deg > 0) != (b.seg.turn_deg > 0)):
            pair = i
    if pair is None:
        return list(moves)
    a, b = moves[pair], moves[pair + 1]
    best = None
    for d in range(-span, span + 1):
        if min(a.seg.value, b.seg.value) + d < tp.MIN_ARC_DEG:
            continue
        mm = list(moves)
        mm[pair] = Move(Seg(a.seg.kind, a.seg.value + d), a.speed, a.carry)
        mm[pair + 1] = Move(Seg(b.seg.kind, b.seg.value + d), b.speed, b.carry)
        err = abs(fly_path(p, mm, r_at, step=2.0)[-1].y - land_y)
        if best is None or err < best[0]:
            best = (err, mm)
    return best[1]


def speedify(p: Pose, segs: Sequence[Seg], boxes, need: float,
             arc_v: int = 15, straight_v: int = 20, last_v: Optional[int] = None,
             r_at: Callable[[int], float] = radius_at,
             land_y: Optional[float] = None) -> List[Move]:
    """
    A planned line as protocol 10 moves: arcs at arc_v, straights at
    straight_v (the last one at last_v if given), every hand-over continued
    except into a change of direction and at the end. The FLYING path is then
    checked against `boxes`: where it comes under `need`, the hand-over into
    that stretch stops instead, and the check repeats. All stops is the
    planned geometry itself; if even that fails `need`, the protocol 9 line
    comes back unchanged. land_y: see trim_landing().
    """
    n = len(segs)
    if n == 0:
        return []
    speeds = []
    for i, s in enumerate(segs):
        if s.kind in "LRlr":
            speeds.append(arc_v)
        else:
            speeds.append(last_v if (last_v and i == n - 1) else straight_v)
    carry = [i < n - 1 and (segs[i].kind in "FLR") == (segs[i + 1].kind in "FLR")
             for i in range(n)]
    for _ in range(n + 1):
        moves = [Move(s, v, c) for s, v, c in zip(segs, speeds, carry)]
        if land_y is not None:
            moves = trim_landing(p, moves, land_y, r_at)
        ends: List[int] = []
        path = fly_path(p, moves, r_at, ends=ends)
        bad = None
        for idx in range(0, len(path), 2):
            q = path[idx]
            if min((tp.clearance(q, b) for b in boxes), default=1e9) < need:
                bad = idx
                break
        if bad is None:
            return moves
        j = next((k for k, e in enumerate(ends) if e >= bad), n - 1)
        # Stop the latest hand-over at or before the move that came too close.
        k = j - 1
        while k >= 0 and not carry[k]:
            k -= 1
        if k < 0:
            break
        carry[k] = False
    return [Move(s) for s in segs]


def end_pose(p: Pose, moves: Sequence[Move], r_at: Callable[[int], float] = radius_at
             ) -> Pose:
    return fly_path(p, moves, r_at)[-1]
