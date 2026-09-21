"""Closed-form Dubins shortest-path solver (LSL, RSR, LSR, RSL, RLR, LRL).

Angles in radians, 0 = +x axis, increasing CCW. A path is returned as
(mode, (l1, l2, l3), length) where l1/l3 are turn angles in radians and
l2 is the straight length in the same units as x/y.
"""

import math
from typing import Optional, Tuple

TWO_PI = 2 * math.pi


def _mod2pi(theta: float) -> float:
    return theta % TWO_PI


def _lsl(alpha, beta, d) -> Optional[Tuple[float, float, float]]:
    ca, sa, cb, sb = math.cos(alpha), math.sin(alpha), math.cos(beta), math.sin(beta)
    p_sq = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (sa - sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(cb - ca, d + sa - sb)
    t = _mod2pi(-alpha + tmp)
    q = _mod2pi(beta - tmp)
    return t, p, q


def _rsr(alpha, beta, d) -> Optional[Tuple[float, float, float]]:
    ca, sa, cb, sb = math.cos(alpha), math.sin(alpha), math.cos(beta), math.sin(beta)
    p_sq = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (sb - sa)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(ca - cb, d - sa + sb)
    t = _mod2pi(alpha - tmp)
    q = _mod2pi(-beta + tmp)
    return t, p, q


def _lsr(alpha, beta, d) -> Optional[Tuple[float, float, float]]:
    ca, sa, cb, sb = math.cos(alpha), math.sin(alpha), math.cos(beta), math.sin(beta)
    p_sq = -2 + d * d + 2 * math.cos(alpha - beta) + 2 * d * (sa + sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    t = _mod2pi(-alpha + tmp)
    q = _mod2pi(-_mod2pi(beta) + tmp)
    return t, p, q


def _rsl(alpha, beta, d) -> Optional[Tuple[float, float, float]]:
    ca, sa, cb, sb = math.cos(alpha), math.sin(alpha), math.cos(beta), math.sin(beta)
    p_sq = d * d - 2 + 2 * math.cos(alpha - beta) - 2 * d * (sa + sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    t = _mod2pi(alpha - tmp)
    q = _mod2pi(beta - tmp)
    return t, p, q


def _rlr(alpha, beta, d) -> Optional[Tuple[float, float, float]]:
    ca, sa, cb, sb = math.cos(alpha), math.sin(alpha), math.cos(beta), math.sin(beta)
    tmp = (6.0 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) > 1:
        return None
    p = _mod2pi(TWO_PI - math.acos(tmp))
    t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
    q = _mod2pi(alpha - beta - t + p)
    return t, p, q


def _lrl(alpha, beta, d) -> Optional[Tuple[float, float, float]]:
    ca, sa, cb, sb = math.cos(alpha), math.sin(alpha), math.cos(beta), math.sin(beta)
    tmp = (6.0 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) > 1:
        return None
    p = _mod2pi(TWO_PI - math.acos(tmp))
    t = _mod2pi(-alpha + math.atan2(-ca + cb, d + sa - sb) + p / 2.0)
    q = _mod2pi(beta - alpha - t + p)
    return t, p, q


_SOLVERS = {
    "LSL": (_lsl, "LSL"),
    "RSR": (_rsr, "RSR"),
    "LSR": (_lsr, "LSR"),
    "RSL": (_rsl, "RSL"),
    "RLR": (_rlr, "RLR"),
    "LRL": (_lrl, "LRL"),
}


def shortest_path(
    x1: float, y1: float, t1: float,
    x2: float, y2: float, t2: float,
    r: float,
) -> Tuple[str, Tuple[float, float, float], float]:
    dx, dy = x2 - x1, y2 - y1
    d = math.hypot(dx, dy) / r
    theta = math.atan2(dy, dx)
    alpha = _mod2pi(t1 - theta)
    beta = _mod2pi(t2 - theta)

    best_mode = None
    best_lengths = None
    best_len = math.inf

    for name, (fn, mode) in _SOLVERS.items():
        result = fn(alpha, beta, d)
        if result is None:
            continue
        t, p, q = result
        is_ccc = mode in ("RLR", "LRL")
        length = r * (t + p + q) if is_ccc else r * (t + q) + r * p
        if length < best_len:
            best_len = length
            best_mode = mode
            straight = r * p if not is_ccc else 0.0
            best_lengths = (t, straight, q) if not is_ccc else (t, p, q)

    if best_mode is None:
        raise ValueError("no Dubins path found")

    return best_mode, best_lengths, best_len


def simulate(
    x1: float, y1: float, t1: float,
    mode: str, lengths: Tuple[float, float, float], r: float,
) -> Tuple[float, float, float]:
    """Forward-simulate a path — used for testing shortest_path()."""
    x, y, t = x1, y1, t1
    for seg_type, seg_len in zip(mode, lengths):
        if seg_type == "L":
            x += r * (math.sin(t + seg_len) - math.sin(t))
            y += r * (math.cos(t) - math.cos(t + seg_len))
            t += seg_len
        elif seg_type == "R":
            x += r * (math.sin(t) - math.sin(t - seg_len))
            y += r * (math.cos(t - seg_len) - math.cos(t))
            t -= seg_len
        elif seg_type == "S":
            x += seg_len * math.cos(t)
            y += seg_len * math.sin(t)
        t = math.atan2(math.sin(t), math.cos(t))
    return x, y, t
