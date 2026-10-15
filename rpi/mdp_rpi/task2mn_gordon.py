"""
task2mn_gordon.py - Task 2 version 2 ("gordon"): Mahan's task2mn route, faster.

Same course model, robot model, obstacle-1 approach and parking as the
task2mn files - it imports them and changes none of them. What it changes is
everything between the first arrow and the bay mouth, where the 2026-09-14
05:46 floor run spent ~25 of its 32 s: scans at precision speed with an RST
stop per corner, a handoff search run while standing still, and a return in
separate lines.

    python3 task2mn_gordon.py                           # on the robot, as task2mn.py
    python3 task2mn_gordon_sim.py 90 90 60 left right   # laptop simulator

THE ROUTE (world frame of task2mn_park.py: origin the bay mouth, +x out)

  1. Obstacle 1: task2mn_route.obstacle1(), unchanged - approach and arrow 1.
  2. ONE line: kick + cruise. The kick ends on a wider bypass lane than
     task2mn's (rear axle |y| = 28 cm, 13.6 cm body gap to obstacle 1, not 6).
     While it runs the Pi polls ?IRW (pose and both IRs in one tick) and ?US:
       - the inner IR on obstacle 1 gives the outbound lateral fix;
       - the ultrasound gives obstacle 2's face x. Only its x: the beam is a
         cone, and on the floor it heard a 30 cm obstacle's corner from this
         lane as well as a 60 cm obstacle's face (both ~93 cm). No echo at
         all is still taken as narrow (the old narrow scan, kept for that);
       - vision reads arrow 2 once obstacle 1 is behind the lens.
     The Pi RSTs the cruise where the face scan starts - the same for both
     arrows. Arrow 2 is read THERE, stopped, the camera ~45 cm from obstacle 2
     (its range is 30-50 cm) - before the scan turns it away.
  3. ONE measuring line, the same for both arrows: turn in toward the
     centreline onto obstacle 2's front face and run along it with the side
     IR; RST on its falling edge, which is that end (and so the width,
     obstacle 2 being centred). An IR that never sees a face means obstacle 2
     is narrower than where the IR started - enough to plan with. With the
     width measured the walls (at least 50 cm past the ends) are known too:
     no single path clears both a 45 cm half-width and the walls of a 15 cm
     one, which is why this line is not optional.
       opposite arrows  carry on along the face, round the measured end;
       same arrows      reverse back along the face and swing out with a
                        reverse arc onto a lane outside the other end, then
                        round it (a forward turn there would need 45 cm past
                        the end - the walls only promise 50).
  4. ONE line round obstacle 2 and back to the bay mouth: the loop (an
     F/arc family searched against the swept footprint), then task2mn_park's
     obstacle-1 pass lane and line B appended. The obstacle-1 IR fix is taken
     on the move; only if it disagrees with odometry by more than
     T2G_RETURN_FIX_TOL_CM before the S starts does the Pi RST and re-plan
     line B from the fix (task2mn_park.ParkRunner._finish).
  5. At the mouth: ParkRunner._park_from_mouth(), unchanged - side-fence
     centring and the ultrasound-checked final F at precision speed.

PROTOCOL 10 (firmware with ?VER >= 10, PROTOCOL.md section 13): the same
lines, sent as moves with their own speed that roll into each other without
stopping (task2mn_gordon_link.speedify) - arcs at TIGHT's calibrated V15,
straights at V20 - checked for clearance on the path they actually fly,
hand-over spirals included. Only the measuring RSTs, the bay mouth and the
parking F stop. Older firmware gets the plain tokens.

Anything gordon cannot plan hands over to task2mn's own around_obstacle2()
and ParkRunner from wherever the robot stopped.

Every movement is checked against the swept footprint before it is sent,
against the worst case still possible: obstacle 2 at its widest bound AND the
walls of its narrowest. Target T2G_PLAN_CLEARANCE_CM (6); a plan below
T2G_MIN_CLEARANCE_CM (4.5) is never sent. TIGHT arcs, frozen calibration.
"""

import concurrent.futures
import logging
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import task2mn_gordon_link as gl
import task2mn_link as link
import task2mn_park as tp
import task2mn_route as route
from task2mn_park import Box, Course, Pose, Seg

_env = tp._env_f

# ── Tunables ─────────────────────────────────────────────────────────────────
# Body gap past obstacle 1 on the bypass lane: rear axle |y| = 28 cm. A US
# miss from here caps obstacle 2's half-width at 29, which the narrow scan
# lane clears; a hit floors it at 26, which one more split can resolve. (26
# was tried: its 24..37 split leaves no same-side loop clear of the walls.)
# An obstacle just wider than the lane can be missed after drift; the narrow
# scan then meets its end inside the IR's 10 cm and takes it from there.
BYPASS_GAP_CM = _env("T2G_BYPASS_GAP_CM", 13.6)
BYPASS_LANE_CM = tp.OBS1_SIZE_CM / 2.0 + tp.HALF_WIDTH_CM + BYPASS_GAP_CM
# The US is a ray on the robot's centreline, as task2mn models it. A hit
# within US_HIT_TOL_CM of the face's end can be an echo off its corner, so a
# hit bounds the half-width from below by the lane |y| less that. A real beam
# is wider than the ray, so a miss is the stronger statement: it bounds the
# half-width from above by the lane |y| plus US_MISS_TOL_CM.
US_HIT_TOL_CM = _env("T2G_US_HIT_TOL_CM", 2.0)
US_MISS_TOL_CM = _env("T2G_US_MISS_TOL_CM", 1.0)
# Narrow scan lane |y|: the inner IR reads an end face at half-width 15..29 as
# 10.75..24.75 cm - in range, short of the 30-50 cm empty-floor plateau - and
# the lane change out to it keeps ~5.6 cm off the walls of a 30 cm obstacle.
NARROW_LANE_CM = _env("T2G_NARROW_LANE_CM", 45.0)
NARROW_IR_MAX_CM = _env("T2G_NARROW_IR_MAX_CM", 28.0)
# Running along the front face: the IR this far off it (task2mn's figure).
FACE_GAP_CM = route.FACE_GAP_CM
# An end measured by IR is mirrored about the drift-corrected centreline for
# the far end; this much extra is allowed for the drift since.
END_MARGIN_CM = _env("T2G_END_MARGIN_CM", 3.0)
# ...plus this much per metre driven since the last lateral fix (obstacle 1
# on the way out): the mirrored end moves with odometry drift, the measured
# one does not. The sweep's +-2 cm/m drift touched a narrow obstacle 2 after
# ~2.4 m with the flat 3 cm alone.
DRIFT_ALLOW_CM_PER_M = _env("T2G_DRIFT_ALLOW_CM_PER_M", 1.5)


# The same idea for the loop after a face scan, on both sides of the bound
# (obstacle wider AND walls nearer). Two thirds of the rate: the scan has just
# measured the near end, and the full allowance on both sides leaves some
# ordinary layouts no loop at all.
LOOP_DRIFT_CM_PER_M = _env("T2G_LOOP_DRIFT_CM_PER_M", 1.0)


def far_end_margin(path_cm: float) -> float:
    return END_MARGIN_CM + DRIFT_ALLOW_CM_PER_M * max(0.0, path_cm) / 100.0
PLAN_CLEAR_CM = _env("T2G_PLAN_CLEARANCE_CM", tp.CLEARANCE_CM)
MIN_CLEAR_CM = max(tp.CLEARANCE_FLOOR_CM, _env("T2G_MIN_CLEARANCE_CM", 4.5))
# Distance run after RST (protocol 9 brakes and holds), used to time an RST so
# the robot stops before a planned start rather than after it.
RST_COAST_FAST_CM = _env("T2G_RST_COAST_FAST_CM", 8.0)
RST_COAST_NORMAL_CM = _env("T2G_RST_COAST_NORMAL_CM", 4.0)
# Stop this far short of the latest start a branch allows.
DEADLINE_MARGIN_CM = _env("T2G_DEADLINE_MARGIN_CM", 3.0)
# Until the background thread has answered: a branch is assumed to start this
# far before obstacle 2's face (the earliest seen over the 108-layout sweep,
# rounded out).
DEFAULT_WIDE_LEAD_CM = _env("T2G_DEFAULT_WIDE_LEAD_CM", 100.0)
# RST and re-plan line B when the moving obstacle-1 fix is further off.
RETURN_FIX_TOL_CM = _env("T2G_RETURN_FIX_TOL_CM", 2.5)
# Wall-clock cap on one search; a fully checked plan found by then is kept.
PLAN_BUDGET_S = _env("T2G_PLAN_BUDGET_S", 2.0)
# task2mn approaches obstacle 1 in two straights: the bulk, then the last
# T2_APPROACH_LEAD_CM (10) after re-reading ?US. Its 6 cm bypass lane needs
# that precision; gordon's 13.6 cm lane does not, and the kick is planned from
# the measured pose anyway - so the approach is one straight (the ?US re-read
# and its sanity checks stay). One primitive, ~1.2 s, saved.
APPROACH_LEAD_CM = _env("T2G_APPROACH_LEAD_CM", 0.0)
# Allow task2mn_park's direct diagonal home (no obstacle-1 IR fix; the mouth
# centring still measures the side fences) when it is the cheaper return.
DIRECT_RETURN = os.getenv("T2G_DIRECT_RETURN", "1") != "0"
# Hand over to task2mn's own obstacle-2 route when gordon cannot plan. OFF by
# default: that route reads width from ultrasound echoes, and with the real
# (wide) beam it touched walls in the simulator. Off, gordon stops instead.
FALLBACK = os.getenv("T2G_FALLBACK", "0") != "0"
# Heading off the lane within which ultrasound samples classify obstacle 2.
US_MAX_HEADING_DEG = 3.0
POLL_S = _env("T2G_POLL_S", 0.0)

# Protocol 10 speeds, x10 rpm (task2mn_gordon_link, PROTOCOL.md section 13).
# Arcs stay at TIGHT's own 150 rpm until the radius at other speeds has been
# floor-chorded; straights cruise at fast mode's 200; the straight a
# measuring line scans on runs at SCAN_V so the RST that ends it lands close.
ARC_V = int(_env("T2G_ARC_V", 15))
STRAIGHT_V = int(_env("T2G_STRAIGHT_V", 20))
SCAN_V = int(_env("T2G_SCAN_V", 15))

HALF_W2_MAX = route.W2_RANGE[1] / 2.0
HALF_W2_MIN = route.W2_RANGE[0] / 2.0
GAP12_MIN = route.GAP12_RANGE[0]
GAP12_MAX = route.GAP12_RANGE[1]


# ═════════════════════════════════════════════════════════════════════════════
# Geometry
# ═════════════════════════════════════════════════════════════════════════════

def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _other(turn: str) -> str:
    return "R" if turn == "L" else "L"


def _arc_to(p: Pose, target: float, turn: str, r: float
            ) -> Tuple[List[Seg], Pose]:
    """A forward arc turning `turn` from p's heading onto `target`. Under
    MIN_ARC_DEG (the chassis would not move) or already just past: none."""
    d = (target - p.th) if turn == "L" else (p.th - target)
    deg = int(round(math.degrees(d % (2 * math.pi))))
    if deg < tp.MIN_ARC_DEG or deg > 350:
        return [], p
    seg = Seg(turn, deg)
    return [seg], tp.run(p, [seg], r, step=1e6)[-1]


def _straight(p: Pose, d: float) -> Tuple[List[Seg], Pose]:
    n = int(round(d))
    if n == 0:
        return [], p
    return [Seg("F" if n > 0 else "B", abs(n))], tp.straight(p, n)


def compact(segs: Sequence[Seg]) -> List[Seg]:
    """Merge neighbouring straights, and same-way arcs up to 180 deg: one
    primitive and one settle instead of two."""
    out: List[Seg] = []
    for s in segs:
        if s.value <= 0:
            continue
        if out and out[-1].kind == s.kind and (
                s.kind in ("F", "B") or out[-1].value + s.value <= 180):
            out[-1] = Seg(s.kind, out[-1].value + s.value)
        else:
            out.append(s)
    return out


def tokens(segs: Sequence[Seg]) -> str:
    return ",".join(s.token() for s in segs)


def fits_one_line(segs: Sequence[Seg]) -> bool:
    """PROTOCOL.md section 2: 16 primitives, 128 bytes."""
    return (len(segs) <= 16
            and len((tokens(segs) + "\n").encode("ascii")) <= 128)


@dataclass
class Obs2:
    """What is known of obstacle 2: half-width bounds, not one width. Loops
    must clear the widest still possible AND the walls of the narrowest."""
    x2: float                 # front (bay-side) face x
    h_lo: float
    h_hi: float
    c: float = 0.0            # centre y
    measured: bool = False    # an end was measured by IR

    def course_hi(self, x1: float) -> Course:
        return Course(x1, self.x2, 2 * self.h_hi, obs2_y=self.c,
                      wall_y=self.h_hi + tp.WALL_CLEARANCE_CM)

    def boxes(self, x1: float) -> List[Box]:
        hi = self.course_hi(x1)
        lo = Course(x1, self.x2, 2 * self.h_lo, obs2_y=self.c,
                    wall_y=self.h_lo + tp.WALL_CLEARANCE_CM)
        return [hi.obs1(), hi.obs2()] + hi.fence() + lo.wall_boxes()


def _clear(p: Pose, segs: Sequence[Seg], boxes, need: float, r: float
           ) -> Optional[float]:
    """Closest approach along segs, or None the moment it is under need."""
    if not segs:
        return float("inf")
    return route._path_clear(p, segs, r, boxes, need, step=2.0)


def _lanes(info: Obs2, need: float, prefer_max: Optional[float] = None
           ) -> List[float]:
    """Rear-axle |y - c| of lanes running past obstacle 2's end: clear of its
    widest bound and of the walls of its narrowest, middle first."""
    lo = info.h_hi + tp.HALF_WIDTH_CM + need
    hi = (info.h_lo + tp.WALL_CLEARANCE_CM - tp.HALF_WIDTH_CM - need
          if tp.ASSUME_WALLS else lo + 40.0)
    if lo > hi:
        return [round((lo + hi) / 2.0, 1)]
    mid = (lo + hi) / 2.0
    out = [mid]
    if prefer_max is not None and lo <= prefer_max < mid:
        out.insert(0, prefer_max)
    out += [x for x in (lo + 2.0, mid + 4.0, lo + 6.0) if lo <= x <= hi]
    seen, lanes = set(), []
    for x in out:
        k = round(x, 1)
        if k not in seen:
            seen.add(k)
            lanes.append(k)
    return lanes


def _home_ir_max(sigma: float) -> float:
    """Largest home lane |y| whose obstacle-1 IR fix needs no lane change
    (heading home on side -sigma)."""
    side = tp.ir_side(-sigma * 10.0)
    return tp.OBS1_SIZE_CM / 2 + tp.IR_SIDE_OFFSET_CM + tp.IR_MAX_GAP_CM[side]


@dataclass
class Loop:
    family: str
    segs: List[Seg]
    end: Pose
    clearance: float
    start_x: float            # where its first arc begins
    seconds: float
    to_mouth: bool = False    # ends at the bay mouth (direct diagonal home)


def _home_tail(q: Pose, sigma: float, info: Obs2, Lh: float, turn: str,
               r: float) -> Optional[Tuple[List[Seg], Pose]]:
    """From heading -sigma*90 behind obstacle 2: the straight along its back
    and the 90 home, ending on |y - c| = Lh heading home."""
    back = sigma * (q.y - (info.c - sigma * Lh + sigma * r))
    if back < -0.5:
        return None
    b, t = _straight(q, back)
    a, t = _arc_to(t, math.pi, turn, r)
    return b + a, t


def _direct_tail(q: Pose, sigma: float, turn: str, r: float
                 ) -> List[Tuple[List[Seg], Pose]]:
    """From heading -sigma*90 behind obstacle 2: F along the back, ONE arc
    of 90 + th round the far end and onto a diagonal toward the bay, the
    diagonal, and th back - square on the bay centreline at task2mn_park's
    mouth-check pose, as its plan_direct_return ends. Shallowest first."""
    check_x = tp.IR_FROM_AXLE_CM - tp.CENTRE_CHECK_IN_CM
    other = _other(turn)
    out = []
    for th in range(10, 64, 3):
        a1 = Seg(turn, 90 + th)
        e1 = tp.run(Pose(0.0, 0.0, q.th), [a1], r, step=1e6)[-1]
        a3 = Seg(other, th)
        e3 = tp.run(Pose(0.0, 0.0, e1.th), [a3], r, step=1e6)[-1]
        ux, uy = math.cos(e1.th), math.sin(e1.th)
        d = (check_x - 2.0 - q.x - e1.x - e3.x) / ux
        if d < 1.0:
            continue
        b = sigma * (q.y + e1.y + e3.y + d * uy)
        if b < -0.5:
            continue
        for dd in (0, 1, -1):
            segs = (_straight(q, b)[0] + [a1] + [Seg("F", int(round(d)) + dd)]
                    + [a3])
            end = tp.run(q, segs, r, step=1e6)[-1]
            if check_x - 4.0 <= end.x <= check_x and abs(end.y) <= 0.75:
                out.append((segs, end))
                break
    return out


def _tails(q: Pose, sigma: float, info: Obs2, homes: Sequence[float],
           turn: str, boxes, need: float, r: float, direct: bool = True):
    """Every checked way home from behind obstacle 2: (segs, end, clearance,
    to_mouth). The direct diagonal skips the obstacle-1 fix, so it keeps
    END_MARGIN_CM more from everything."""
    found = []
    no_back = [b for b in boxes if b.name != "back fence"]
    for Lh in homes:
        got = _home_tail(q, sigma, info, Lh, turn, r)
        if got is None:
            continue
        c = _clear(q, got[0], no_back, need, r)
        if c is not None:
            found.append((got[0], got[1], c, False))
    if DIRECT_RETURN and direct:
        for segs, end in _direct_tail(q, sigma, turn, r):
            c = _clear(q, segs, no_back, need + END_MARGIN_CM, r)
            if c is not None:
                found.append((segs, end, c, True))
                break
    return found


def _loops(family: str, p: Pose, prefix: List[Seg], q: Pose, pre: float,
           start_x: float, sigma: float, info: Obs2, homes, turn: str,
           boxes, need: float, r: float, direct: bool = True) -> List[Loop]:
    out = []
    for tail, end, c, mouth in _tails(q, sigma, info, homes, turn, boxes,
                                      need, r, direct):
        segs = compact(prefix + tail)
        out.append(Loop(family, segs, end, min(pre, c), start_x,
                        tp.motion_cost(segs, r), mouth))
    return out


def _loop_outer(p: Pose, sigma: float, info: Obs2, L: float, homes,
                lead: float, boxes, need: float, r: float) -> List[Loop]:
    """[lead] [lane change to the pass lane] F [90 behind] + a way home. The
    straight before the turn behind is the shortest that keeps `need`."""
    segs, q = _straight(p, lead)
    dy = info.c + sigma * L - q.y
    if abs(dy) >= 1.5:
        t1 = "L" if dy > 0 else "R"
        if abs(dy) <= 2 * r:
            th1, diag = math.acos(max(-1.0, 1.0 - abs(dy) / (2 * r))), 0.0
        else:
            th1, diag = math.pi / 2, abs(dy) - 2 * r
        a, q = _arc_to(q, math.copysign(th1, dy), t1, r)
        segs += a
        s, q = _straight(q, diag)
        segs += s
        a, q = _arc_to(q, 0.0, _other(t1), r)
        segs += a
    first = next((i for i, s in enumerate(segs) if s.kind in "LR"), None)
    start_x = (tp.run(p, segs[:first], r, step=1e6)[-1].x
               if first is not None else None)
    pre = _clear(p, segs, boxes, need, r)
    if pre is None:
        return []
    turn = "R" if sigma > 0 else "L"
    face_back = info.x2 + tp.OBS2_DEPTH_CM
    x_b = max(q.x, face_back - 14.0)
    while x_b <= face_back + 30.0:
        mid, t = _straight(q, x_b - q.x)
        a3, t = _arc_to(t, -sigma * math.pi / 2, turn, r)
        if not a3:
            return []
        c2 = _clear(q, mid + a3, boxes, need, r)
        if c2 is not None:
            loops = _loops("outer", p, segs + mid + a3, t, min(pre, c2),
                           x_b if start_x is None else start_x, sigma, info,
                           homes, turn, boxes, need, r)
            if loops:
                return loops
        x_b += 2.0
    return []


def _lane_clear_x(p: Pose, boxes, need: float, r: float,
                  limit: float = 300.0) -> float:
    """How far straight on from p stays `need` clear (x reached)."""
    d = 0.0
    while d < limit:
        q = tp.straight(p, d + 2.0)
        if min(tp.clearance(q, b) for b in boxes) < need:
            break
        d += 2.0
    return tp.straight(p, d).x


def _loop_eight(p: Pose, sigma: float, info: Obs2, th1_deg: int, cy: float,
                cx: float, homes, boxes, need: float, r: float,
                allow_reverse: bool, lane_clear_x: Optional[float] = None,
                direct: bool = True) -> List[Loop]:
    """task2mn's same-side 'outer eight': [lead] [arc out th1] F [arc in
    th1 + 90, centred on (cx, c + sigma*cy)] + a way home. Not straightening
    onto a lane first keeps the front corner's swing toward the wall least."""
    th1 = math.radians(th1_deg)
    y0 = sigma * (p.y - info.c)
    d = (cy - y0 - r + 2 * r * math.cos(th1)) / math.sin(th1)
    if d < -0.5:
        return []
    lead = cx - p.x - 2 * r * math.sin(th1) - d * math.cos(th1)
    if lead < -0.5 and not allow_reverse:
        return []
    out = "L" if sigma > 0 else "R"
    inn = _other(out)
    lead_segs, q0 = _straight(p, lead)
    start_x = q0.x
    if lane_clear_x is not None and lead >= 0 and start_x > lane_clear_x:
        return []
    a, q = _arc_to(q0, sigma * th1, out, r)
    segs = list(a)
    s, q = _straight(q, d)
    segs += s
    a, q = _arc_to(q, -sigma * math.pi / 2, inn, r)
    if not a:
        return []
    segs += a
    if lane_clear_x is not None and lead >= 0:
        pre = _clear(q0, segs, boxes, need, r)
    else:
        pre = _clear(p, lead_segs + segs, boxes, need, r)
    if pre is None:
        return []
    return _loops("eight", p, lead_segs + segs, q, pre, start_x, sigma, info,
                  homes, inn, boxes, need, r, direct)


def _loop_front(p: Pose, sigma: float, info: Obs2, gap: float, homes,
                boxes, need: float, r: float,
                allow_reverse: bool = False) -> List[Loop]:
    """[lead or reverse] [90 onto the front face, toward sigma] F along it
    [180 round its end] + a way home. Needs the end measured. Starting on
    the face already (heading sigma*90) it skips the first 90."""
    t1 = "L" if sigma > 0 else "R"
    if abs(_wrap(p.th - sigma * math.pi / 2)) < math.radians(20):
        segs, q, start_x = [], p, p.x
    else:
        lead = info.x2 - gap - tp.IR_SIDE_OFFSET_CM - r - p.x
        segs, q = _straight(p, lead)
        start_x = q.x
        a, q = _arc_to(q, sigma * math.pi / 2, t1, r)
        if not a:
            return []
        segs += a
    pre = _clear(p, segs, boxes, need, r)
    if pre is None:
        return []
    turn = _other(t1)
    y_rel = sigma * (q.y - info.c)
    # Along the face to where the 180 round its end can start. Already past
    # that (a scan that ran on beside a narrow obstacle) - back up to it: a
    # 180 from further out swings the far side toward the wall.
    f = info.h_hi - 16.0 - y_rel
    if not allow_reverse:
        f = max(0.0, f)
    wall_lo = info.h_lo + tp.WALL_CLEARANCE_CM
    while y_rel + f + r + tp.HALF_WIDTH_CM <= wall_lo:
        mid, t = _straight(q, f)
        a2, t = _arc_to(t, -sigma * math.pi / 2, turn, r)
        if not a2 or a2[0].value < 150:
            return []
        c2 = _clear(q, mid + a2, boxes, need, r)
        if c2 is not None:
            loops = _loops("front", p, segs + mid + a2, t, min(pre, c2),
                           start_x, sigma, info, homes, turn, boxes, need, r)
            if loops:
                return loops
        f += 2.0
    return []


def _loop_reverse_out(p: Pose, sigma: float, info: Obs2, homes, boxes,
                      need: float, r: float) -> List[Loop]:
    """Same arrows, from the face scan's stop (heading -sigma*90, in front of
    obstacle 2, having measured its -sigma end): reverse back along the face
    toward sigma, then a reverse arc that swings the tail out and leaves the
    robot heading +x on a lane outside obstacle 2's sigma end - and from
    there the 'outer' loop: past the end, 90 behind, home. Reversing out
    keeps everything inside the 50 cm the brief leaves to the walls; a
    forward turn from here would need the robot 45 cm past the end first."""
    if abs(_wrap(p.th + sigma * math.pi / 2)) > math.radians(20):
        return []
    # Reverse arc that ends heading +x: steer so the body rotates toward 0.
    arc_kind = "r" if sigma > 0 else "l"
    out: List[Loop] = []
    for lane in _lanes(info, need):
        # The reverse arc moves sigma*r further out: reverse until it lands
        # on the lane.
        y_r = info.c + sigma * (lane - r)
        d = sigma * (y_r - p.y)
        if d < -0.5:
            continue
        pre, q = _straight(Pose(p.x, p.y, p.th), -d)
        arc = Seg(arc_kind, 90)
        q = tp.run(q, [arc], r, step=1e6)[-1]
        pre = pre + [arc]
        c1 = _clear(p, pre, boxes, need, r)
        if c1 is None:
            continue
        loops = _loop_outer(q, sigma, info, lane, homes, 0.0, boxes, need, r)
        for lp in loops:
            segs = compact(pre + lp.segs)
            out.append(Loop("reverse_out", segs, lp.end, min(c1, lp.clearance),
                            p.x, tp.motion_cost(segs, r), lp.to_mouth))
        if out:
            return out
    return out


@dataclass
class GoPlan:
    side2: float
    loop: Loop
    ret: Optional[tp.ReturnPlan]
    line_b: List[Seg]
    clearance: float
    seconds: float
    info: Obs2

    @property
    def start_x(self) -> float:
        return self.loop.start_x

    def segments(self) -> List[Seg]:
        line_a = self.ret.line_a if self.ret is not None else []
        return compact(self.loop.segs + line_a + self.line_b)

    def lines(self) -> List[List[Seg]]:
        """Round and home as ONE firmware line when it fits, else two."""
        whole = self.segments()
        if fits_one_line(whole):
            return [whole]
        line_a = self.ret.line_a if self.ret is not None else []
        tail = compact(line_a + self.line_b)
        return [list(self.loop.segs)] + ([tail] if tail else [])

    def summary(self) -> str:
        return (f"{self.loop.family} loop to the "
                f"{'left' if self.side2 > 0 else 'right'}, "
                f"{'direct diagonal' if self.ret is None else 'obstacle-1 lane'} "
                f"home: {tokens(self.segments())} (clearance "
                f"{self.clearance:.1f} cm, ~{self.seconds:.1f} s)")


def _mouth_line_b(final: tp.FinalPlan) -> List[Seg]:
    """Line B as ParkRunner._finish drives it: the S, then on until both
    side IRs are just inside the mouth."""
    check_x = tp.IR_FROM_AXLE_CM - tp.CENTRE_CHECK_IN_CM
    line_b = list(final.segs)
    if final.end.x > check_x + 1.0:
        line_b += tp._f(final.end.x - check_x)
    return line_b


def _with_return(loop: Loop, sigma: float, x1: float, info: Obs2, boxes,
                 r: float) -> Optional[GoPlan]:
    """The loop with the drive back to the bay mouth: nothing more for a
    direct-diagonal loop, else task2mn_park's obstacle-1 pass lane (its IR
    fix taken on the move) and line B."""
    if loop.to_mouth:
        return GoPlan(sigma, loop, None, [], loop.clearance, loop.seconds, info)
    # A home lane: the obstacle-1 lane with its fix, or the direct diagonal.
    home = Pose(loop.end.x, loop.end.y, tp.HEADING_HOME)
    course = info.course_hi(x1)
    no_back = [b for b in boxes if b.name != "back fence"]
    best = None
    if DIRECT_RETURN:
        direct = tp.plan_direct_return(home, course, r)
        if direct is not None:
            # plan_direct_return already kept 8 cm off this course's boxes;
            # this adds the walls of the narrowest width still possible.
            c = _clear(home, direct.segs, no_back, PLAN_CLEAR_CM, r)
            if c is not None:
                best = GoPlan(sigma, loop, None, list(direct.segs),
                              min(loop.clearance, c),
                              tp.motion_cost(compact(loop.segs + direct.segs), r),
                              info)
    for min_clear in (tp.CLEARANCE_CM, MIN_CLEAR_CM):
        try:
            ret = tp.plan_return(home, course, r, min_clear=min_clear)
        except tp.NoPlan:
            continue
        line_b = _mouth_line_b(ret.final)
        rest = ret.line_a + line_b
        c = _clear(home, rest, no_back, tp.CLEARANCE_FLOOR_CM, r)
        if c is None:
            continue
        plan = GoPlan(sigma, loop, ret, line_b, min(loop.clearance, c),
                      tp.motion_cost(compact(loop.segs + rest), r), info)
        if best is None or plan.seconds < best.seconds:
            best = plan
        break
    return best


def plan_go_around(p: Pose, x1: float, info: Obs2, side2: float,
                   r: float = tp.RADIUS_CM, latest: bool = False,
                   allow_reverse: bool = False,
                   budget_s: float = PLAN_BUDGET_S,
                   families: Sequence[str] = ("outer", "eight", "front")
                   ) -> Optional[GoPlan]:
    """
    The cheapest checked way from p round obstacle 2 to the side2 side and
    back to the bay mouth. latest=True instead returns the loop whose first
    arc starts latest (no return attached): a decision deadline.
    """
    deadline = time.monotonic() + budget_s
    boxes = info.boxes(x1)
    sigma = side2
    for need in (PLAN_CLEAR_CM, MIN_CLEAR_CM):
        homes = _lanes(info, need, prefer_max=_home_ir_max(sigma))[:2]
        if latest:
            homes = homes[:1]
        loops: List[Loop] = []
        if "outer" in families:
            leads = [0.0] + [float(x) for x in range(4, 61, 4)]
            if allow_reverse:
                leads += [float(-x) for x in range(4, 61, 4)]
            for L in _lanes(info, need):
                for lead in leads:
                    got = _loop_outer(p, sigma, info, L, homes, lead, boxes, need, r)
                    if got:
                        loops += got
                        break
        if "eight" in families and abs(_wrap(p.th)) < math.radians(15):
            lane_x = _lane_clear_x(p, boxes, need, r)
            for th1 in (30, 45, 60):
                done = False
                for k in range(0, 13):
                    cy = info.h_hi - 9.0 - 0.5 * k
                    th = math.radians(th1)
                    y0 = sigma * (p.y - info.c)
                    d0 = (cy - y0 - r + 2 * r * math.cos(th)) / math.sin(th)
                    # No lead straight: the arc out starts where the robot is.
                    cx0 = p.x + 2 * r * math.sin(th) + d0 * math.cos(th) - info.x2
                    offsets = [5, 2, 8, 11, -1]
                    if -1.0 <= cx0 <= 11.0 and not latest:
                        offsets.insert(0, cx0)
                    for cxo in offsets:
                        if time.monotonic() > deadline and loops:
                            break
                        got = _loop_eight(p, sigma, info, th1, cy, info.x2 + cxo,
                                          homes, boxes, need, r, allow_reverse,
                                          lane_x, direct=not latest)
                        if got:
                            loops += got
                            done = not latest
                            break
                    if done:
                        break
        if "reverse_out" in families:
            loops += _loop_reverse_out(p, sigma, info, homes, boxes, need, r)
        if "front" in families and info.measured:
            for gap in (FACE_GAP_CM, FACE_GAP_CM - 4.0):
                got = _loop_front(p, sigma, info, gap, homes, boxes, need, r,
                                  allow_reverse)
                loops += [lp for lp in got
                          if allow_reverse or lp.segs[0].kind != "B"]
        if not loops:
            continue
        if latest:
            return GoPlan(sigma, max(loops, key=lambda l: l.start_x), None, [],
                          0.0, 0.0, info)
        best = None
        for lp in sorted(loops, key=lambda l: l.seconds):
            if best is not None and lp.seconds >= best.seconds:
                break
            plan = _with_return(lp, sigma, x1, info, boxes, r)
            if plan is not None and (best is None or plan.seconds < best.seconds):
                best = plan
        if best is not None:
            return best
    return None


# ── measuring lines ───────────────────────────────────────────────────────────

def narrow_info(x1: float, h_max: Optional[float] = None) -> Obs2:
    """Worst narrow obstacle 2 before the scan: closest, widest a miss allows
    (h_max, from where the US actually looked), with the walls of the
    narrowest."""
    return Obs2(x1 + tp.OBS1_SIZE_CM + GAP12_MIN, HALF_W2_MIN,
                h_max if h_max is not None else BYPASS_LANE_CM + US_MISS_TOL_CM)


def narrow_deadline(p: Pose, x1: float, side1: float, r: float = tp.RADIUS_CM,
                    need: float = MIN_CLEAR_CM) -> Optional[float]:
    """Latest rear-axle x on the bypass lane from which the lane change out
    to the narrow scan lane still clears the worst narrow obstacle 2."""
    boxes = narrow_info(x1).boxes(x1)
    best = None
    for lead in range(0, 121, 2):
        q = tp.straight(Pose(p.x, p.y, 0.0), lead)
        segs = tp.lane_change(0.0, side1 * NARROW_LANE_CM - q.y, r) + [Seg("F", 30)]
        if _clear(q, segs, boxes, need, r) is not None:
            best = q.x
        elif best is not None:
            break
    return best


def face_line(p: Pose, x1: float, info: Obs2, sigma: float,
              r: float = tp.RADIUS_CM) -> Optional[Tuple[List[Seg], float]]:
    """Opposite arrows, wide obstacle: [lead or reverse] 90 onto the front
    face toward sigma, then along it until the IR is well past the widest
    end. Returns (segs, x of the 90's start)."""
    lead = info.x2 - FACE_GAP_CM - tp.IR_SIDE_OFFSET_CM - r - p.x
    if -6.0 <= lead <= 4.0:
        lead = 0.0      # a few cm on the face gap beats a primitive (16..28 cm)
    segs, q = _straight(p, lead)
    start = q.x
    a, q = _arc_to(q, sigma * math.pi / 2, "L" if sigma > 0 else "R", r)
    segs += a
    run_to = info.c + sigma * (info.h_hi + 8.0 - tp.IR_FROM_AXLE_CM)
    s, q = _straight(q, sigma * (run_to - q.y))
    segs += s
    boxes = info.boxes(x1)
    if _clear(p, segs, boxes, MIN_CLEAR_CM, r) is None:
        return None
    return compact(segs), start


def face_deadline(info: Obs2, r: float = tp.RADIUS_CM) -> float:
    return info.x2 - FACE_GAP_CM - tp.IR_SIDE_OFFSET_CM - r


def _ray_box(ox, oy, dx, dy, b: Box) -> Optional[float]:
    t0, t1 = 0.0, float("inf")
    for o, d, lo, hi in ((ox, dx, b.x0, b.x1), (oy, dy, b.y0, b.y1)):
        if abs(d) < 1e-12:
            if not lo <= o <= hi:
                return None
            continue
        a, c = (lo - o) / d, (hi - o) / d
        if a > c:
            a, c = c, a
        t0, t1 = max(t0, a), min(t1, c)
        if t0 > t1:
            return None
    return t0


def _ir_expect(p: Pose, side: str, boxes) -> Optional[float]:
    """What a side IR should read at pose p among boxes, cm (None: nothing
    within 80) - the same geometry task2mn_sim ray-casts."""
    s = tp.straight(p, tp.IR_FROM_AXLE_CM)
    k = 1.0 if side == "left" else -1.0
    ox = s.x - k * tp.IR_SIDE_OFFSET_CM * math.sin(p.th)
    oy = s.y + k * tp.IR_SIDE_OFFSET_CM * math.cos(p.th)
    th = p.th + k * math.pi / 2
    hits = [d for d in (_ray_box(ox, oy, math.cos(th), math.sin(th), b)
                        for b in boxes) if d is not None]
    d = min(hits) if hits else None
    return d if d is not None and d <= 80.0 else None


# ═════════════════════════════════════════════════════════════════════════════
# Executor
# ═════════════════════════════════════════════════════════════════════════════

class GordonRoute(route.Task2):
    """task2mn_route.Task2 with gordon's lines from the first kick to the
    bay mouth. obstacle1(), the corner helpers and parking are task2mn's."""

    def __init__(self, stm, *args,
                 arrow2_peek: Optional[Callable[[], Optional[str]]] = None,
                 executor=None, **kwargs):
        super().__init__(stm, *args, **kwargs)
        # Non-blocking stage-2 arrow (the confirmed vote); defaults to the
        # callback task2mn installs for vision.
        self.arrow2_peek = arrow2_peek
        self._executor = executor
        # Decisions, for the log and for the simulator's clock.
        self.events: List[Tuple[str, str]] = []
        self.go_plan: Optional[GoPlan] = None
        # The round-and-home plan started on the background thread by a
        # measuring line, for the pose it expects to stop at.
        self._go_job = None
        self._fix_x = 0.0
        # Firmware protocol 10 (per-move speed, continuous lines): None until
        # ?VER has been asked; the simulator sets it.
        self.p10: Optional[bool] = None
        self._narrow_drift_path = 0.0
        self._narrow_h_max: Optional[float] = None

    # ── helpers ───────────────────────────────────────────────────────────────

    def _pose(self) -> Pose:
        p = self.frame.pose()
        if p is None:
            raise RuntimeError("No ?WPOSE.")
        return p

    def _report(self, phase: str) -> None:
        status = getattr(self, "status", None)
        if callable(status):
            status(phase)

    def _peek2(self) -> Optional[float]:
        fn = self.arrow2_peek or self.early_arrow_fn
        if fn is None or not self._stage2_view_open:
            return None
        answer = fn()
        if not isinstance(answer, str):
            return None
        answer = answer.strip().lower()
        return {"l": 1.0, "left": 1.0, "r": -1.0, "right": -1.0}.get(answer)

    def _arrow2(self) -> float:
        """The stage-2 arrow; blocks (logged as a wait) only if not yet in."""
        if self._peek2() is None:
            self.events.append(("wait_arrow", "stopped for the stage-2 vote"))
            logging.info("Stage-2 arrow not confirmed yet; waiting at the stop.")
        return route._ask_arrow(self.input_fn, "Obstacle 2")

    def _submit(self, fn, *args, **kwargs):
        if self._executor is None:
            self._executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=1, thread_name_prefix="gordon-plan")
        return self._executor.submit(fn, *args, **kwargs)

    def _start_go(self, predicted: Pose, x1: float, info: Obs2, side2: float,
                  families: Sequence[str], allow_reverse: bool) -> None:
        """Plan round and home in the background while the robot brakes."""
        fut = self._submit(plan_go_around, predicted, x1, info, side2, self.r,
                           allow_reverse=allow_reverse, families=families)
        self._go_job = (fut, predicted, info, side2)

    def _plan_go(self, p: Pose, x1: float, info: Obs2, side2: float,
                 families: Sequence[str], allow_reverse: bool) -> Optional[GoPlan]:
        """The background plan if it was made for this stop (within 3 cm and
        3 deg) and still clears from the measured pose; else plan now."""
        job, self._go_job = self._go_job, None
        if job is not None:
            fut, predicted, j_info, j_side = job
            plan = fut.result()
            c, sn = math.cos(predicted.th), math.sin(predicted.th)
            along = (p.x - predicted.x) * c + (p.y - predicted.y) * sn
            across = -(p.x - predicted.x) * sn + (p.y - predicted.y) * c
            square = abs(_wrap(p.th - predicted.th)) <= math.radians(3)
            if (plan is not None and square and abs(across) <= 2.0
                    and abs(along) > 3.0 and plan.loop.segs
                    and plan.loop.segs[0].kind in ("F", "B")):
                # Stopped further on (or short) along the same line: the plan's
                # leading straight takes it up, the rest is unchanged.
                first = plan.loop.segs[0]
                n = (first.value if first.kind == "F" else -first.value) - along
                lead, _ = _straight(Pose(0, 0, 0.0), n)
                plan.loop.segs = compact(lead + plan.loop.segs[1:])
                along = 0.0
            close = square and math.hypot(along, across) <= 3.0
            if (plan is not None and close and j_info is info and j_side == side2
                    and _clear(p, plan.segments(), info.boxes(x1),
                               MIN_CLEAR_CM, self.r) is not None):
                logging.info("Using the plan made on the move (stop %.1f cm "
                             "from where it was planned).",
                             math.hypot(p.x - predicted.x, p.y - predicted.y))
                return plan
            logging.info("Background plan (for a stop at %s, %s) not usable from "
                         "%s; planning now.", predicted,
                         "found" if plan is not None else "none found", p)
        self.events.append(("plan_stopped", "round-and-home planned at rest"))
        return plan_go_around(p, x1, info, side2, self.r,
                              allow_reverse=allow_reverse, families=families)

    @staticmethod
    def _mirrored(info: Obs2, path_cm: float) -> Obs2:
        """The narrow scan measured the near end; the other side's loop
        rounds the mirrored far end FIRST, after the reverse - allow for the
        drift since the obstacle-1 fix there (only there: elsewhere the extra
        width pushes the loop at the walls instead)."""
        if info.h_hi - info.h_lo > 2 * END_MARGIN_CM:
            return info                         # already allowed for
        h = info.h_lo + 1.0
        return Obs2(info.x2, info.h_lo, h + far_end_margin(path_cm), info.c,
                    measured=True)

    def _coast(self) -> float:
        return RST_COAST_FAST_CM if self.fast else RST_COAST_NORMAL_CM

    def _irw(self):
        """(pose, left, right, left_near, right_near) from one ?IRW tick on
        protocol 9; ?WPOSE + ?IR otherwise (no saturation flags)."""
        if self.ir_latch and self.frame.origin is not None:
            w = link.read_irw(self.stm)
            if w is None:
                return None
            q = self.frame.to_world(w["wpose"])
            self._last_poll = q
            return (q, w["left_cm"], w["right_cm"],
                    w["left_cm"] is None and link.near_saturated(w["left_raw"]),
                    w["right_cm"] is None and link.near_saturated(w["right_raw"]))
        q = self.frame.pose()
        if q is None:
            return None
        self._last_poll = q
        return q, self._ir("left"), self._ir("right"), False, False

    # --protocol9 on the full entry point: plain tokens whatever ?VER says.
    _force_p9 = False
    _last_poll: Optional[Pose] = None

    def _detect_p10(self) -> bool:
        if self.p10 is None and self._force_p9:
            self.p10 = False
        if self.p10 is None:
            try:
                fields = self.stm.query_fields("?VER", 0.5)
                self.p10 = fields is not None and int(fields[-1]) >= gl.PROTOCOL
            except Exception:
                self.p10 = False
            if self.p10:
                try:
                    gl.enable()
                except ImportError:
                    pass
            logging.info("Firmware protocol 10 (continuous lines): %s.",
                         "yes" if self.p10 else "no - one stop per primitive")
        return self.p10

    def _moves(self, p: Pose, segs: Sequence[Seg], boxes, need: float,
               last_v: Optional[int] = None, land: bool = False) -> list:
        """The line to send: protocol 10 moves that keep `need` on their
        flying path, or the plain segments on older firmware. land=True: the
        line ends on a lane (or the bay centreline) and must arrive on it as
        planned, hand-over spirals included."""
        if not segs or not self._detect_p10():
            return list(segs)
        land_y = tp.run(p, segs, self.r, step=1e6)[-1].y if land else None
        return gl.speedify(p, segs, [b for b in boxes if b.name != "back fence"],
                           need, ARC_V, STRAIGHT_V, last_v, land_y=land_y)

    @staticmethod
    def _text(items) -> str:
        return (gl.tokens(items) if items and isinstance(items[0], gl.Move)
                else tokens(items))

    def _halt(self, now: bool, reason: str) -> bool:
        """A run_polling poll's answer: RST now (and say why) or carry on."""
        if not now:
            return False
        self.events.append(("rst", reason))
        logging.info("RST: %s.", reason)
        self.stm.abort()
        return True

    def _drive(self, segs: Sequence[Seg], poll, what: str) -> Optional[str]:
        """Send one line, polling; 'OK' or 'STOPPED' (at rest), else None.
        segs may be Segs or protocol 10 Moves."""
        self._last_poll = None
        reply = tp.run_polling(self.stm, self._text(segs), poll)
        if reply == "STOPPED":
            p = self._wait_for_stopped_pose()
            if p is None:
                return None
            self.th = p.th
            if self._last_poll is not None:
                # The run-on actually seen, to check T2G_RST_COAST_*_CM by:
                # from the pose of the poll that sent RST to where it settled.
                logging.info("RST coast: %.1f cm (from the last poll at %s; "
                             "the %s allowance is %.1f cm).",
                             math.hypot(p.x - self._last_poll.x,
                                        p.y - self._last_poll.y),
                             self._last_poll, "fast" if self.fast else "normal",
                             self._coast())
            return reply
        if reply is None or not reply.startswith("OK"):
            logging.error("STM answered %r to the %s line %s.", reply, what,
                          self._text(segs))
            return None
        self._track([m.seg if isinstance(m, gl.Move) else m for m in segs])
        return reply

    # ── the run ───────────────────────────────────────────────────────────────

    def run(self) -> int:
        if self.startup and not self._startup():
            return 2
        if self.wait_for_go:
            self.input_fn("Robot in the bay, square, on the centreline. "
                          "Press Enter to GO. ")
            if self.startup:
                from communications import cal_profile
                if not cal_profile.zero_heading(self.stm):
                    return 2
        if not self.frame.anchor():
            logging.error("No ?WPOSE - flash firmware with protocol >= 6.")
            return 2
        t0 = time.monotonic()
        try:
            hit = self.obstacle1()
            if hit is None:
                return 1
            x1, side1 = hit
            if not self.gordon(x1, side1):
                return 1
        except KeyboardInterrupt:
            logging.warning("Interrupted - RST.")
            self.stm.abort()
            return 2
        except RuntimeError as exc:
            logging.error("%s", exc)
            return 1
        finally:
            if self._executor is not None:
                self._executor.shutdown(wait=False)
        logging.info("Gordon route finished in %.1f s; caller must verify "
                     "parking.", time.monotonic() - t0)
        return 0

    def gordon(self, x1: float, side1: float) -> bool:
        self._report("BYPASS_1")
        out = self.bypass(x1, side1)
        if out is None:
            return False
        if out["class"] not in ("wide", "narrow"):
            return self._fallback(x1, side1, out.get("x2"), None)
        if out["class"] == "narrow":
            info = self.narrow_scan(x1, side1)
            if info is None:
                return False
            side2 = self._arrow2()
            self._report("ROUND_2")
            # Same side: on past the end and behind it. Other side: back up
            # and run along the front face - the end is measured either way.
            same = side2 == side1
            if not same:
                info = self._mirrored(info, self._narrow_drift_path)
            plan = self._plan_go(self._pose(), x1, info, side2,
                                 ("outer",) if same else ("front",), not same)
        else:
            # Read arrow 2 HERE, stopped at the face scan's start: the camera
            # faces obstacle 2 from ~45 cm, 28 cm off its centre - in the
            # camera's 30-50 cm range (floor, 2026-10-15). The scan turns it
            # away, so the vote has to be in first; it also lets the way
            # round be planned on the move while the scan runs.
            side2 = self._arrow2()
            self._report("ROUND_2")
            info = self.face_scan(x1, out["info"], side1)
            if info is None:
                return self._fallback(x1, side1, out["info"].x2, side2)
            plan = self._plan_go(self._pose(), x1, info, side2,
                                 self._after_scan(side1, side2), True)
        if plan is None:
            # Every family, reversing allowed, before giving up the loop.
            plan = plan_go_around(self._pose(), x1, info, side2, self.r,
                                  allow_reverse=True, budget_s=2 * PLAN_BUDGET_S)
        if plan is None:
            logging.warning("No gordon loop from %s; handing over to task2mn's "
                            "obstacle-2 route.", self._pose())
            return self._fallback(x1, side1, info.x2, side2, info)
        logging.info("Gordon: %s", plan.summary())
        self.go_plan = plan
        self._report("RETURN_AND_PARK")
        return self.drive_home(plan, x1)

    # ── 2. kick + cruise ──────────────────────────────────────────────────────

    def obstacle1(self):
        old, tp.APPROACH_LEAD_CM = tp.APPROACH_LEAD_CM, APPROACH_LEAD_CM
        try:
            return super().obstacle1()
        finally:
            tp.APPROACH_LEAD_CM = old

    @staticmethod
    def _bypass_lanes() -> List[float]:
        # The wide lane first; task2mn's 6 cm lane only when obstacle 1 sits so
        # close to the bay that nothing wider can be reached. obstacle1() keeps
        # task2mn's lane for its standoff: that stops the approach ~25 cm
        # sooner than the wide lane would, which is the stretch of lane the
        # ultrasound needs to classify obstacle 2 before the narrow decision.
        base = tp.OBS1_SIZE_CM / 2 + tp.HALF_WIDTH_CM
        return [base + g for g in (BYPASS_GAP_CM, 10.0, route.OBS1_PASS_GAP_CM)]

    def _kick(self, p: Pose, x1: float, side1: float):
        course = Course(x1, x1 + tp.OBS1_SIZE_CM + 60.0, route.CONSERVATIVE_W2,
                        wall_y=route.CONSERVATIVE_WALL_Y)
        leads = list(range(0, 6)) + list(range(-1, -route.OBS1_RECOVERY_REVERSE_CM - 1, -1))
        best, best_c = None, -1.0
        for lane in self._bypass_lanes():
            # Two arcs first: no straight in the kick, one primitive less.
            shapes = (tp.lane_change(0.0, side1 * lane - p.y, self.r),
                      route.kick(side1 * lane - p.y, self.r))
            for lead in leads:
                for shape in shapes:
                    cand = route._straight(lead) + shape
                    c = self._check(p, cand, course)
                    if c >= route.OBS1_TURN_MIN_CM:
                        return cand, lane, c
                    if c > best_c:
                        best, best_c = (cand, lane), c
        if best is not None and best_c >= route.OBS1_TURN_FALLBACK_CM:
            return best[0], best[1], best_c
        return None

    def bypass(self, x1: float, side1: float) -> Optional[dict]:
        """Kick onto the bypass lane and cruise, polling the US, the inner IR
        and vision; RST at the latest point the next line can start from."""
        p = self._pose()
        got = self._kick(p, x1, side1)
        if got is None:
            logging.error("No clear kick past obstacle 1.")
            return None
        kick_segs, lane, kick_c = got
        self._fix_x = x1          # the obstacle-1 lateral fix happens here
        end = tp.run(p, kick_segs, self.r, step=1e6)[-1]
        # Never cruise far enough to touch the closest obstacle 2 possible.
        x_far = x1 + tp.OBS1_SIZE_CM + GAP12_MIN - tp.FRONT_FROM_AXLE_CM - 8.0
        cruise = int(math.floor(x_far - end.x))
        segs = list(kick_segs) + ([Seg("F", cruise)] if cruise >= 1 else [])
        k_course = Course(x1, x1 + tp.OBS1_SIZE_CM + GAP12_MIN, route.CONSERVATIVE_W2,
                          wall_y=route.CONSERVATIVE_WALL_Y)
        segs = self._moves(p, segs, k_course.boxes(),
                           max(MIN_CLEAR_CM, min(kick_c, route.OBS1_TURN_MIN_CM) - 1.0),
                           land=True)
        n_dead = narrow_deadline(end, x1, side1, self.r)
        if n_dead is None:
            n_dead = end.x
        # The narrow scan must be square on its lane before the IR can reach
        # the closest face possible: sightings mid-arc read the FRONT face
        # obliquely (seen at a 60 cm gap), not the end.
        widen = tp.lane_change(0.0, side1 * (NARROW_LANE_CM - lane), self.r)
        widen_dx = tp.run(Pose(0.0, 0.0, 0.0), widen, self.r, step=1e6)[-1].x
        n_dead = min(n_dead, x1 + tp.OBS1_SIZE_CM + GAP12_MIN - widen_dx
                     - tp.IR_FROM_AXLE_CM - 6.0)
        logging.info("Bypass: %s (lane %.1f, kick clearance %.1f cm); narrow "
                     "decision by x=%.1f.", self._text(segs), lane, kick_c, n_dead)
        room = n_dead - DEADLINE_MARGIN_CM - RST_COAST_FAST_CM - (end.x - 1.0)
        if room < 5.0:
            # The ultrasound only reads once square on the lane (end.x): the
            # RST allowance eats the lane it has to read obstacle 2 on.
            logging.warning("Only %.1f cm of lane to classify obstacle 2 on: "
                            "the RST coast allowance (T2G_RST_COAST_FAST_CM="
                            "%.1f) is too large - check the 'RST coast' log "
                            "lines for the real figure.", max(room, 0.0),
                            RST_COAST_FAST_CM)
        inner = "right" if side1 > 0 else "left"
        st = {"us": [], "miss": 0, "class": None, "x2": None, "us_y": None,
              "hits": [], "fix_done": False, "future": None, "branches": None,
              "reason": None, "info": None}
        k = -side1

        def poll() -> bool:
            if POLL_S:
                self.sleep_fn(POLL_S)
            got = self._irw()
            if got is None:
                return False
            q, left, right, _, _ = got
            self._notify_obstacle2_view(q, x1)
            # Obstacle-1 lateral fix (task2mn's _view_line, on the move).
            g = left if inner == "left" else right
            if (g is not None and not st["fix_done"]
                    and abs(math.sin(q.th)) < math.sin(math.radians(5))
                    and 10 <= g <= tp.IR_MAX_GAP_CM[inner]):
                c, s = math.cos(q.th), math.sin(q.th)
                off = tp.IR_SIDE_OFFSET_CM + g
                lead_ir = tp.IR_FROM_AXLE_CM - tp.IR_LAG_CM
                beam_x = q.x + lead_ir * c - off * k * s
                if x1 + 1 <= beam_x <= x1 + tp.OBS1_SIZE_CM - 1:
                    face_y = side1 * tp.OBS1_SIZE_CM / 2
                    st["hits"].append(face_y - lead_ir * s - off * k * c - q.y)
            if (not st["fix_done"] and q.x + tp.IR_FROM_AXLE_CM
                    > x1 + tp.OBS1_SIZE_CM + 6):
                st["fix_done"] = True
                self._apply_obs1_fix(st["hits"])
                q = self.frame.pose() or q
            # Obstacle 2 from the ultrasound, once square on the lane.
            if (st["class"] is None and q.x >= end.x - 1.0
                    and abs(_wrap(q.th)) < math.radians(US_MAX_HEADING_DEG)):
                self._classify(st, q, x1, lane)
            if st["class"] == "wide" and st["info"] is None:
                st["info"] = self._wide_info(st)
            # When to stop.
            reach = q.x + self._coast()
            if st["class"] in (None, "narrow"):
                return self._halt(reach >= n_dead - DEADLINE_MARGIN_CM,
                                  "narrow decision point" if st["class"]
                                  else "obstacle 2 not classified yet")
            if st["class"] != "wide":
                return self._halt(True, st["reason"])
            # Both arrows start the same way - the face scan toward the
            # centre - so the arrow is not needed yet: stop where that starts.
            return self._halt(reach >= face_deadline(st["info"], self.r)
                              - DEADLINE_MARGIN_CM, "at the face scan's start")

        self._set_fast(True)
        reply = self._drive(segs, poll, "bypass")
        if reply is None:
            return None
        if not st["fix_done"]:
            st["fix_done"] = True
            self._apply_obs1_fix(st["hits"])
        stop = self._pose()
        self.th = stop.th
        if st["class"] == "wide":
            if st["info"] is None:
                st["info"] = self._wide_info(st)
            self.obs2_us_y = st["us_y"]
            return {"class": "wide", "stop": stop, "info": st["info"]}
        if st["class"] == "narrow":
            return {"class": "narrow", "stop": stop}
        logging.warning("Obstacle 2 not classified on the lane (%s).",
                        st["reason"] or "no stable echo")
        return {"class": "fallback", "stop": stop, "x2": st["x2"]}

    def _classify(self, st: dict, q: Pose, x1: float, lane: float) -> None:
        u = self._us()
        front = tp.straight(q, tp.FRONT_FROM_AXLE_CM)
        hit_x = None if u is None else front.x + u * math.cos(q.th)
        gap12 = None if hit_x is None else hit_x - (x1 + tp.OBS1_SIZE_CM)
        if gap12 is not None and GAP12_MIN - 5 <= gap12 <= GAP12_MAX:
            st["miss"] = 0
            st["us"].append((hit_x, front.y + u * math.sin(q.th)))
            if len(st["us"]) >= 2 and abs(st["us"][-1][0] - st["us"][-2][0]) <= 2.0:
                st["class"] = "wide"
                st["x2"] = min(st["us"][-1][0], st["us"][-2][0])
                st["us_y"] = st["us"][-1][1]
                logging.info("Obstacle 2 face at x=%.1f from the lane (US ray "
                             "at y=%.1f): WIDE.", st["x2"], st["us_y"])
        elif gap12 is not None and gap12 < GAP12_MIN - 5:
            st["class"] = "odd"
            st["reason"] = "US echo only %.1f cm past obstacle 1" % gap12
        else:
            st["miss"] += 1
            if st["miss"] >= 3:
                st["class"] = "narrow"
                # The ray's own y, not the planned lane's: a miss bounds the
                # half-width by where the US actually looked.
                self._narrow_h_max = max(lane, abs(q.y)) + US_MISS_TOL_CM
                logging.info("No US echo from y=%.1f: obstacle 2 is NARROW "
                             "(half-width under %.0f).", q.y, self._narrow_h_max)

    @staticmethod
    def _wide_info(st: dict) -> Obs2:
        # The echo gives the face's x, not its width: the beam is a cone and
        # hears a 30 cm obstacle's corner from the lane as well as a 90 cm
        # obstacle's face. Any width is still possible until the IR measures it.
        return Obs2(st["x2"], HALF_W2_MIN, HALF_W2_MAX)

    def _apply_obs1_fix(self, shifts: List[float]) -> None:
        """Median of the sightings that agree, as task2mn's _view_line."""
        if len(shifts) < 2:
            return
        mid = sorted(shifts)[len(shifts) // 2]
        agree = [h for h in shifts if abs(h - mid) <= 1.5]
        if len(agree) * 2 < len(shifts) or len(agree) < 2:
            logging.warning("Ignoring inconsistent obstacle-1 IR lateral fix: %s.",
                            [round(s, 1) for s in shifts])
            return
        corr = sorted(agree)[len(agree) // 2]
        if 0.5 <= abs(corr) <= route.Y_RESYNC_MAX_CM:
            self.frame.y_shift += corr
            logging.info("Obstacle-1 IR lateral fix on the move: y %+.1f cm "
                         "(%d sightings).", corr, len(agree))

    # ── 3. the measuring line ─────────────────────────────────────────────────

    def narrow_scan(self, x1: float, side1: float) -> Optional[Obs2]:
        """Lane change out to NARROW_LANE_CM and find the near end face with
        the inner IR: the firmware latch when the IR is already past
        obstacle 1 (it would latch obstacle 1 otherwise), else ?IRW polling."""
        p = self._pose()
        widen = tp.lane_change(0.0, side1 * NARROW_LANE_CM - p.y, self.r)
        lc_end = tp.run(p, widen, self.r, step=1e6)[-1]
        far_x = (x1 + tp.OBS1_SIZE_CM + GAP12_MAX + tp.OBS2_DEPTH_CM + 6
                 - tp.IR_FROM_AXLE_CM)
        segs = compact(widen + [Seg("F", int(math.floor(far_x - lc_end.x)))])
        if _clear(p, widen, narrow_info(x1, self._narrow_h_max).boxes(x1),
                  MIN_CLEAR_CM, self.r) is None:
            logging.error("Lane change to the narrow scan lane is not clear.")
            return None
        segs = self._moves(p, segs, narrow_info(x1, self._narrow_h_max).boxes(x1),
                           MIN_CLEAR_CM, SCAN_V,
                           land=True)
        inner = "right" if side1 > 0 else "left"
        lo, hi = 10, int(NARROW_IR_MAX_CM)
        use_latch = (self.ir_latch
                     and p.x + tp.IR_FROM_AXLE_CM > x1 + tp.OBS1_SIZE_CM + 3)
        st = {"hits": [], "latched": None, "close": False, "seen": None,
              "stop_x": None, "use_latch": use_latch, "info": None,
              "drift_path": 0.0}

        def seen(q: Pose, gap: float, now: Pose) -> None:
            st["seen"] = (q, gap)
            x2 = q.x + tp.IR_FROM_AXLE_CM - tp.IR_LAG_CM
            h = abs(q.y - side1 * (tp.IR_SIDE_OFFSET_CM + gap))
            info = Obs2(x2, max(HALF_W2_MIN - 2, h - 1.0), h + END_MARGIN_CM,
                        0.0, measured=True)
            st["drift_path"] = (now.x - self._fix_x) + 60.0 + 2 * h + 60.0
            v = Pose(q.x, q.y, 0.0)
            loops = _loop_outer(v, side1, info, abs(q.y), [abs(q.y)], 0.0,
                                info.boxes(x1), PLAN_CLEAR_CM, self.r)
            st["stop_x"] = (min(l.start_x for l in loops) - 1.0 if loops
                            else q.x)
            st["info"] = info
            side2 = self._peek2()
            if side2 is not None:
                same = side2 == side1
                if not same:
                    info = st["info"] = self._mirrored(info, st["drift_path"])
                self._start_go(Pose(max(st["stop_x"], now.x + self._coast()), q.y,
                                    0.0), x1, info,
                               side2, ("outer",) if same else ("front",), not same)

        def poll() -> bool:
            if POLL_S:
                self.sleep_fn(POLL_S)
            got = self._irw()
            if got is None:
                return False
            q, left, right, ln, rn = got
            self._notify_obstacle2_view(q, x1)
            if st["stop_x"] is not None:
                return self._halt(q.x + self._coast() >= st["stop_x"],
                                  "past the narrow end, at the turn behind it")
            past1 = (q.x + tp.IR_FROM_AXLE_CM - tp.IR_LAG_CM
                     > x1 + tp.OBS1_SIZE_CM + 5)
            g, near = (left, ln) if inner == "left" else (right, rn)
            if past1 and near and st["seen"] is None:
                # Inside the IR's 10 cm floor: the end is nearer than the
                # window - obstacle 2 a little wider than the miss allowed
                # (a ray at the lane's edge). It is still an end face, and the
                # lane still clears it: take it as one at 9 cm.
                logging.warning("Inner IR inside 10 cm: end face closer than "
                                "its window - taking it at 9 cm.")
                seen(q, 9.0, q)
                return False
            square = abs(_wrap(q.th)) < math.radians(5)
            if st["use_latch"]:
                e = link.read_edge(self.stm)
                if e is not None and e["state"] == "latched":
                    w = self.frame.to_world(e["wpose"])
                    if abs(_wrap(w.th)) < math.radians(5):
                        st["latched"] = e
                        seen(w, float(e["cm"]), q)
                    else:
                        # Latched mid-arc: the front face at an angle. The
                        # latch cannot be re-armed moving; poll from here.
                        logging.warning("IR latch fired %.0f deg off the lane - "
                                        "ignored; polling ?IRW instead.",
                                        math.degrees(_wrap(w.th)))
                        st["use_latch"] = False
            elif square and past1 and g is not None and lo <= g <= hi:
                st["hits"].append((q, g))
                if (len(st["hits"]) >= 2
                        and abs(st["hits"][-1][1] - st["hits"][-2][1]) <= 3.0):
                    seen(*st["hits"][-2], q)
            else:
                st["hits"].clear()
            return False

        if use_latch and not link.set_edge(self.stm, inner, lo, hi):
            logging.warning("!IRE refused - polling the narrow scan instead.")
            use_latch = st["use_latch"] = False
        logging.info("Narrow scan: %s, %s IR %s.", self._text(segs), inner,
                     "edge latch" if use_latch else "?IRW polling")
        reply = self._drive(segs, poll, "narrow scan")
        if use_latch:
            link.set_edge(self.stm, None)
        if reply is None:
            return None
        if st["close"]:
            logging.error("Inner IR inside 10 cm in the narrow lane: obstacle 2 "
                          "is wider than a US miss allows. Stopped.")
            return None
        if st["seen"] is None:
            logging.error("Narrow scan ended without seeing obstacle 2's end.")
            return None
        q, gap = st["seen"]
        info = st["info"]
        near = q.y - side1 * (tp.IR_SIDE_OFFSET_CM + gap)
        logging.info("Narrow obstacle 2: face x=%.1f, near end y=%.1f (%s IR "
                     "%.0f cm).", info.x2, near, inner, gap)
        self.obs2_near_y = near
        self._narrow_drift_path = st["drift_path"]
        return info

    @staticmethod
    def _after_scan(side1: float, side2: float) -> Tuple[str, ...]:
        # The scan ran toward -side1. Opposite arrows carry on that way round
        # the measured end; same arrows back out to the side1 end.
        return ("front", "outer") if side2 == -side1 else ("reverse_out",)

    def face_scan(self, x1: float, info: Obs2, side1: float) -> Optional[Obs2]:
        """
        Onto obstacle 2's front face, turning in toward the centreline (away
        from side1), and along it at scan speed with the side IR on the face:
        RST on its falling edge, which is the end - the width, obstacle 2
        being centred. If the IR is already past the end when the scan starts
        it sees no face at all: obstacle 2 is narrower than that, and the
        bound is enough to plan with. Same for both arrows.
        """
        sigma = -side1
        p = self._pose()
        got = face_line(p, x1, info, sigma, self.r)
        if got is None:
            logging.error("No clear line onto obstacle 2's front face from %s.", p)
            return None
        segs, _ = got
        segs = self._moves(p, segs, info.boxes(x1), MIN_CLEAR_CM, SCAN_V)
        ir = "right" if sigma > 0 else "left"
        st = {"on": [], "off": 0, "end": None, "close": False, "info": None,
              "start": None, "narrow": False}

        def face_info(st) -> Obs2:
            # Odometry has drifted since the obstacle-1 fix, by up to
            # DRIFT_ALLOW_CM_PER_M over what is driven until the loop's far
            # side: on both sides of the bound - the obstacle may reach
            # further AND the walls may be nearer. (Widening only the
            # obstacle pushed loops into the walls.)
            drift = LOOP_DRIFT_CM_PER_M * max(
                0.0, info.x2 - self._fix_x + 2 * info.h_hi) / 100.0
            if st["narrow"]:
                # Never on the face: its end is short of where the IR began.
                h = abs(st["start"] - info.c) + 2.0
                return Obs2(info.x2, HALF_W2_MIN - drift,
                            h + END_MARGIN_CM + drift, info.c, measured=True)
            xs = sorted(x for x, _ in st["on"])
            h = abs(st["end"] - info.c)
            return Obs2(xs[len(xs) // 2], max(info.h_lo, h - 1.0) - drift,
                        h + END_MARGIN_CM + drift, info.c, measured=True)

        def plan_ahead(q: Pose) -> None:
            side2 = self._peek2()
            if side2 is not None:
                self._start_go(tp.straight(q, self._coast()), x1, st["info"],
                               side2, self._after_scan(side1, side2), True)

        def poll() -> bool:
            if POLL_S:
                self.sleep_fn(POLL_S)
            got = self._irw()
            if got is None:
                return False
            q, left, right, ln, rn = got
            if abs(_wrap(q.th - sigma * math.pi / 2)) > math.radians(5):
                return False
            g, near = (left, ln) if ir == "left" else (right, rn)
            if near:
                st["close"] = True
                return self._halt(True, "face IR inside 10 cm")
            expect = info.x2 - q.x - tp.IR_SIDE_OFFSET_CM
            ir_y = q.y + sigma * (tp.IR_FROM_AXLE_CM - tp.IR_LAG_CM)
            if st["start"] is None:
                st["start"] = ir_y
            if g is not None and abs(g - expect) <= 8.0:
                st["on"].append((q.x + tp.IR_SIDE_OFFSET_CM + g, ir_y))
                st["off"] = 0
                return False
            if len(st["on"]) >= 2:
                st["off"] += 1
                if st["off"] == 1:
                    st["end"] = ir_y
                if st["off"] >= 2:
                    st["info"] = face_info(st)
                    plan_ahead(q)
                    return self._halt(True, "front face ended")
            elif sigma * (ir_y - st["start"]) > 6.0 and (
                    not st["on"] or sigma * (ir_y - st["on"][-1][1]) > 4.0):
                # Nothing on the face - or one stray reading at its very end
                # while the arc was still finishing: narrow either way.
                if st["on"]:
                    st["start"] = st["on"][-1][1]
                st["narrow"] = True
                st["info"] = face_info(st)
                plan_ahead(q)
                return self._halt(True, "no front face: obstacle 2 is narrow")
            return False

        logging.info("Front face scan toward the %s: %s, %s IR.",
                     "left" if sigma > 0 else "right", self._text(segs), ir)
        reply = self._drive(segs, poll, "front face")
        if reply is None or st["close"]:
            return None
        if st["narrow"]:
            measured = st["info"]
            logging.info("No front face from y=%.1f: obstacle 2 is narrow "
                         "(half-width under %.0f); face x=%.1f from the US.",
                         st["start"], measured.h_hi, measured.x2)
            return measured
        if st["end"] is None or len(st["on"]) < 2:
            logging.error("Front face scan saw no end (%d face readings).",
                          len(st["on"]))
            return None
        measured = st["info"] or face_info(st)
        logging.info("Obstacle 2 front face at x=%.1f (IR; US said %.1f); its "
                     "%s end at y=%.1f.", measured.x2, info.x2,
                     "left" if sigma > 0 else "right", st["end"])
        return measured

    # ── 4. round obstacle 2 and home, one line ────────────────────────────────

    def drive_home(self, plan: GoPlan, x1: float) -> bool:
        info = plan.info
        course = Course(x1, info.x2, 2 * info.h_hi, obs2_y=info.c,
                        wall_y=info.h_hi + tp.WALL_CLEARANCE_CM)
        runner = tp.ParkRunner(self.stm, self.frame, course)
        runner.use_irw = self.ir_latch
        self._set_fast(True)
        lines = plan.lines()
        if self._detect_p10():
            # A hand-over may cost at most 0.5 cm of what the plan kept: the
            # robot's own spiral is a model until PROTOCOL.md 13.4 measures it.
            need = max(tp.CLEARANCE_FLOOR_CM, min(plan.clearance, PLAN_CLEAR_CM) - 0.5)
            q = self._pose()
            sent = []
            for i, segs in enumerate(lines):
                moves = self._moves(q, segs, info.boxes(x1), need,
                                    land=i == len(lines) - 1)
                sent.append(moves)
                q = gl.end_pose(q, moves) if moves and isinstance(moves[0], gl.Move) \
                    else tp.run(q, segs, self.r, step=1e6)[-1]
            lines = sent
            logging.info("Protocol 10 home line(s): %s; expected at the mouth "
                         "at %s.", " | ".join(self._text(l) for l in lines), q)
        self._expect_line(self._pose(), lines, info.boxes(x1))
        ret = plan.ret
        if ret is None:
            for segs in lines:
                if self._drive(segs, self._safety_poll, "home") in (None, "STOPPED"):
                    return False
            return runner._park_from_mouth()
        home_x = plan.loop.end.x
        samples: List[Tuple[float, float]] = []
        st = {"fixed": False, "replan": None, "unsafe": None}

        def poll() -> bool:
            if POLL_S:
                self.sleep_fn(POLL_S)
            got = self._irw()
            if got is None:
                return False
            q, left, right, ln, rn = got
            why = self._unexpected_near(q, ln, rn)
            if why is not None:
                st["unsafe"] = why
                return self._halt(True, st["unsafe"])
            homeward = math.cos(q.th) < -math.cos(math.radians(5))
            if not homeward or q.x > home_x + 1 or st["fixed"]:
                return False
            g = left if ret.side == "left" else right
            if g is not None:
                samples.append((q.x, g))
            if q.x < x1 + tp.IR_FROM_AXLE_CM - 2:
                st["fixed"] = True
                fix = tp.ir_fix(samples, ret, course)
                if fix is None:
                    logging.warning("Obstacle-1 IR fix on the move: %d samples, "
                                    "no fix.", len(samples))
                    return False
                err = fix.y - q.y
                logging.info("Obstacle-1 IR fix on the move: gap %.1f cm -> y "
                             "%.1f (odometry %.1f), x shift %s.", fix.gap_cm,
                             fix.y, q.y, "n/a" if fix.x_shift is None
                             else "%+.1f" % fix.x_shift)
                if abs(err) > RETURN_FIX_TOL_CM and q.x > ret.x_stop + 2.0:
                    st["replan"] = (err, fix.x_shift)
                    return self._halt(True, "obstacle-1 fix %+.1f cm off plan" % err)
                if 0.5 <= abs(err) <= route.Y_RESYNC_MAX_CM:
                    self.frame.y_shift += err
                if fix.x_shift is not None:
                    self.frame.x_shift += fix.x_shift
            return False

        for i, segs in enumerate(lines):
            last = i == len(lines) - 1
            reply = self._drive(segs, poll if last else (lambda: False), "home")
            if reply is None:
                return False
            if reply == "STOPPED":
                if st["unsafe"]:
                    logging.error("Stopped: %s.", st["unsafe"])
                    return False
                err, x_shift = st["replan"]
                # Straight on, heading home, since the samples: the lateral
                # error at the stop is the error at the fix.
                self.frame.y_shift += err
                if x_shift is not None:
                    self.frame.x_shift += x_shift
                here = self._pose()
                logging.info("Re-planning line B from the IR fix: %s.", here)
                return runner._finish(Pose(here.x, here.y, tp.HEADING_HOME))
        return runner._park_from_mouth()

    def _safety_poll(self) -> bool:
        if POLL_S:
            self.sleep_fn(POLL_S)
        got = self._irw()
        if got is None:
            return False
        q, _, _, ln, rn = got
        why = self._unexpected_near(q, ln, rn)
        return self._halt(why is not None, why or "")

    def _expect_line(self, p: Pose, lines, boxes) -> None:
        """What each side IR should read along the lines about to be sent:
        the path they fly (planner model), every 2 cm."""
        out = []
        q = p
        for items in lines:
            if items and isinstance(items[0], gl.Move):
                poses = gl.fly_path(q, items, step=2.0)
            else:
                poses = tp.run(q, items, self.r, step=2.0)
            for pq in poses:
                out.append((pq.x, pq.y, _ir_expect(pq, "left", boxes),
                            _ir_expect(pq, "right", boxes)))
            q = poses[-1]
        self._expected = out

    def _unexpected_near(self, q: Pose, left_near: bool, right_near: bool
                         ) -> Optional[str]:
        """A side IR inside its 10 cm floor where the plan says that side is
        clear: something is nearer than planned. Where the plan itself passes
        that close (a 5 cm pass beside obstacle 1 reads under 10 on the side
        IR) it is not a reason to stop."""
        if q.x <= 5.0 or not (left_near or right_near):
            return None
        exp = getattr(self, "_expected", None)
        if exp:
            k = min(range(len(exp)),
                    key=lambda i: (exp[i][0] - q.x) ** 2 + (exp[i][1] - q.y) ** 2)
            _, _, el, er = exp[k]
            # Under 10 on the IR is a body gap under ~6 cm. The plan may pass
            # that close (a 5 cm pass beside obstacle 1) and the real path can
            # run a few cm off the planned one: stop only where the plan
            # expected that side to be well clear (IR 18 cm, body gap ~14).
            if ((not left_near or (el is not None and el < 18.0))
                    and (not right_near or (er is not None and er < 18.0))):
                return None
        return "a side IR inside 10 cm, not expected there, at %s" % (q,)

    # ── fallback ──────────────────────────────────────────────────────────────

    def _fallback(self, x1: float, side1: float, x2: Optional[float],
                  side2: Optional[float], info: Optional[Obs2] = None) -> bool:
        """task2mn's own obstacle-2 route and return, from the stop - only
        with T2G_FALLBACK=1 (see FALLBACK)."""
        if not FALLBACK:
            logging.error("Gordon has no checked way on from here; stopping "
                          "(T2G_FALLBACK=1 would hand over to task2mn's route).")
            self.events.append(("stopped", "no plan, no fallback"))
            return False
        self.events.append(("fallback", "task2mn around_obstacle2"))
        if x2 is None:
            x2 = self.find_narrow(x1, side1)
            if x2 is None:
                return False
            course = Course(x1, x2, 2 * route.NARROW_HALF_MAX,
                            wall_y=route.CONSERVATIVE_WALL_Y)
        else:
            w2 = route.CONSERVATIVE_W2 if info is None else 2 * info.h_hi
            course = Course(x1, x2, w2, wall_y=route.CONSERVATIVE_WALL_Y)
        if side2 is None:
            side2 = self.photo_obstacle2(course, side1)
        measured = self.around_obstacle2(course, side1, side2)
        if measured is None:
            return False
        runner = tp.ParkRunner(self.stm, self.frame, measured)
        runner.use_irw = self.ir_latch
        return runner.run()


# ═════════════════════════════════════════════════════════════════════════════
# On the robot: task2mn.py's Android / camera / PC wiring, gordon's route
# ═════════════════════════════════════════════════════════════════════════════

def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Task 2 version 2 (gordon).")
    ap.add_argument("--manual", action="store_true",
                    help="floor test without Android or the PC: start on Enter, "
                         "type each arrow (left/right) when asked")
    ap.add_argument("--protocol9", action="store_true",
                    help="send plain protocol 9 tokens even if the firmware "
                         "answers ?VER 10 (one stop per primitive)")
    ap.add_argument("--verbose", action="store_true",
                    help="log every ?IR / ?WPOSE / ?IRW poll too")
    a = ap.parse_args()
    if a.manual:
        return _manual(a.protocol9, a.verbose)
    if a.protocol9:
        GordonRoute._force_p9 = True
    import task2mn
    base = task2mn.Task2

    class GordonTask2(base, GordonRoute):
        """task2mn.Task2 (vision, status, parking check) over GordonRoute.
        MRO: GordonTask2, task2mn.Task2, GordonRoute, task2mn_route.Task2."""

    task2mn.Task2 = GordonTask2
    try:
        return task2mn.main()
    finally:
        task2mn.Task2 = base


class _QuietPolling(route._QuietPolling):
    """task2mn's filter drops the ?IR / ?WPOSE polls; gordon also polls
    ?IRW, ?IRE and ?SEG ~50 times a second (their sends already match ?IR)."""

    def filter(self, record):
        msg = record.getMessage()
        return super().filter(record) and not msg.startswith(
            ("STM ← stm: IRW,", "STM ← stm: IRE,", "STM → stm: ?SEG",
             "STM ← stm: SEG,"))


def _manual(protocol9: bool, verbose: bool) -> int:
    """GordonRoute on its own, as task2mn_route.py's main() runs task2mn's
    route: connect, restore the Task 2 calibration (T2T_CAL_PROFILE), fast
    mode, then Enter to go and typed arrows."""
    if not verbose:
        for h in logging.getLogger().handlers:
            h.addFilter(_QuietPolling())
    from communications.stm import STM
    stm = STM()
    try:
        task = GordonRoute(stm)
        if protocol9:
            task.p10 = False
        return task.run()
    finally:
        try:
            stm.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
