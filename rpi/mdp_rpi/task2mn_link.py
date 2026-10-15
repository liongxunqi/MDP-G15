"""
task2mn_link.py - protocol 9 side-IR commands for the task2mn files.

Firmware protocol 9 (stm/ on the maha-task2 branch) adds three commands,
IR SNAPSHOT AND EDGE LATCH in the firmware's rpilink.h:

    ?IRW    IRW,<l_cm>,<r_cm>,<l_raw>,<r_raw>,<x_mm>,<y_mm>,<hdg_x10>,<ms>
            both side IRs, their raw counts and the ?WPOSE pose, one tick
    !IRE<n> arm the edge latch, n = side*10000 + lo*100 + hi (0 disarms)
    ?IRE    IRE,<state>,<side>,<cm>,<raw>,<x_mm>,<y_mm>,<hdg_x10>,<ms>

The shared communications/stm.py on the Pi may predate them: its token check
refuses !IRE and its reply router drops IRW/IRE lines as noise. Rather than
edit that shared file, enable() teaches it the three tokens at run time. It
is idempotent and harmless on a client that already knows them.

Every helper takes anything with send_line / wait_reply / query_fields - the
raw STM or task2mn.Motion.
"""

import logging
import re
from typing import Optional

PROTOCOL = 9
NO_READING = 0xFFFF
_IRE_RE = re.compile(r"^!IRE(0|[12]\d{4})$", re.IGNORECASE)


def enable() -> None:
    """Make communications.stm accept !IRE and route IRW / IRE replies."""
    from communications import stm as stm_module

    stm_module.QUERY_TAGS.update({"IRW", "IRE"})
    original = stm_module.validate_token
    if getattr(original, "_task2mn", False):
        return

    def validate_token(token):
        if _IRE_RE.match(str(token).strip()):
            return True, ""
        return original(token)

    validate_token._task2mn = True
    stm_module.validate_token = validate_token


def _ints(fields, n: int, tag: str):
    if fields is None:
        # A query that timed out - routine while a line is being taken in.
        logging.debug("STM: no %s reply.", tag)
        return None
    if len(fields) < n:
        logging.error("STM: malformed or missing %s reply %s.", tag, fields)
        return None
    try:
        return [int(f) for f in fields[:n]]
    except ValueError:
        logging.error("STM: non-integer field in %s reply %s.", tag, fields)
        return None


def _wpose(v) -> dict:
    # Same keys as STM.read_wpose(), so WorldFrame can convert it.
    return {"x_mm": v[0], "y_mm": v[1], "heading_deg": v[2] / 10.0}


def read_irw(stm, timeout: float = 0.5) -> Optional[dict]:
    """
    ?IRW. Distances in cm or None. Raw counts tell a None apart: high counts
    are closer than the IR's 10 cm minimum (saturated), low are nothing in
    range - see near_saturated().
    """
    v = _ints(stm.query_fields("?IRW", timeout), 8, "IRW")
    if v is None:
        return None
    return {"left_cm": None if v[0] == NO_READING else v[0],
            "right_cm": None if v[1] == NO_READING else v[1],
            "left_raw": v[2], "right_raw": v[3],
            "wpose": _wpose(v[4:7]), "ms": v[7]}


def edge_token(side: Optional[str], lo_cm: int = 0, hi_cm: int = 0) -> str:
    if side is None:
        return "!IRE0"
    code = {"left": 1, "right": 2}.get(side)
    lo, hi = int(round(lo_cm)), int(round(hi_cm))
    if code is None or not (10 <= lo <= hi <= 80):
        raise ValueError(f"IR edge window {side} {lo}..{hi} cm is not valid")
    return f"!IRE{code * 10000 + lo * 100 + hi}"


def set_edge(stm, side: Optional[str], lo_cm: float = 0,
             hi_cm: float = 0) -> bool:
    """Arm (or disarm, side=None) the latch. Idle only: plain OK reply."""
    if not stm.send_line([edge_token(side, lo_cm, hi_cm)]):
        return False
    reply = stm.wait_reply()
    return reply is not None and reply.strip().upper() == "OK"


def read_edge(stm, timeout: float = 0.5) -> Optional[dict]:
    """?IRE: state 'off' / 'armed' / 'latched'; when latched the reading and
    the ?WPOSE pose of the first tick of the confirming run."""
    v = _ints(stm.query_fields("?IRE", timeout), 8, "IRE")
    if v is None:
        return None
    return {"state": {0: "off", 1: "armed", 2: "latched"}.get(v[0], "?"),
            "side": {1: "left", 2: "right"}.get(v[1]),
            "cm": v[2], "raw": v[3], "wpose": _wpose(v[4:7]), "ms": v[7]}


# Raw ADC counts above which a side IR with no reading is closer than its
# 10 cm floor rather than seeing nothing. ir.h's fits put 10 cm at 2.75 V
# (3410 counts) on the left and 2.54 V (3150) on the right; "nothing in
# range" is under 0.30 V (370). Only consulted when the cm reading is None.
NEAR_SATURATED_RAW = 3000


def near_saturated(raw: int) -> bool:
    return raw >= NEAR_SATURATED_RAW
