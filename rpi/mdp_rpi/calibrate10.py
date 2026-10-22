"""
calibrate10.py  —  protocol 10 calibration: per move, per speed, approved by you
─────────────────────────────────────────────────────────────────────────────────
calibrate.py treats the robot as symmetric: one trim, one braking model, one
steering pulse per side. This robot is not symmetric - one motor pulls less,
one turn swings wider, forward and reverse differ, and all of it moves with
speed. Protocol 10 firmware holds a calibration TABLE instead (PROTOCOL.md §13,
caltab.h): each move type F, R, FR, FL, RR, RL has its own values at up to
three speeds, interpolated in between, plus hardware values (wheel size ratio,
each motor's own feedforward line and deadband).

This tool fills that table from the robot's OWN sensors. After every move the
firmware reports what happened (?MV): counts and rpm and duty per wheel, the
steering it needed, how fast it was turning when it braked and how far it
coasted, where it stopped. A tape is only asked for what nothing on board can
know - absolute distance, and the radius of each turn once.

Nothing changes unless you approve it. Every step ends with a list:

    #  value          now     ->  new     why
    1  G.wheel        -       ->  10031   right wheel covers +0.31% more ground per count
    2  F@100.trim     +44     ->  +38     steering held -7.5 µs on average (4 runs, sd 1.1)
    3  FR@150.decel   517.0   ->  498.2   stops +0.6° past its aim (4 runs)

    Enter = apply all, n = none, numbers = only those (e.g. "1 3"),
    e 2 40 = change #2 to 40 first.

Approved values go to the robot at once (so the next round drives on them) and
into cal_profiles/<name>.json. Each step then runs again to confirm, up to
three rounds, and stops early when nothing is left to fix.

    python3 calibrate10.py guided arena --from tr18        # all steps
    python3 calibrate10.py straights arena --speeds 80,150,220
    python3 calibrate10.py arcs arena --speeds 100,150 --target 29.1 --tape
    python3 calibrate10.py arcs arena --moves FR,RR --target match
    python3 calibrate10.py gyro arena
    python3 calibrate10.py verify arena --speed 150
    python3 calibrate10.py run arena FR90V15 --repeat 3     # one move, full report
    python3 calibrate10.py set arena FR@150.steer=610 G.wheel=10030
    python3 calibrate10.py set arena FR@150.rpm=0           # drop that anchor
    python3 calibrate10.py show arena
    python3 calibrate10.py restore arena

Every move of every step is logged to cal_profiles/<name>_runs.csv - the full
?MV report, the tape you entered, and the round - so a session can be looked at
again without re-running it.

THE STEPS
─────────
  straights  F and R at each speed, back and forth on one strip of floor
             (needs `cm` + 30 cm clear). From the reports:
               G.wheel       right/left ground per count, from the count
                             ratio on held-straight runs
               G.ff* G.db*   each motor's duty-vs-rpm line, per direction -
                             a weak or dragging motor shows here
               F/R@rpm.trim  the steering each direction needed at each speed
               F/R@rpm.coast how far it rolled after the brake
               F/R@rpm.dist  (with --tape) true / odometry distance
  gyro       four FR90 and four FL90 against an edge (the one thing the gyro
             cannot see about itself)
  arcs       FR+RR and FL+RL pairs at each speed - each pair comes back to the
             start. From the reports:
               <arc>@rpm.decel/.lag  braking, fitted across speeds
               <arc>@rpm.steer       (with --target) steering for the radius
             Radius comes from the odometry chord, corrected by one tape
             chord per move and speed (--tape); without a tape it uses the
             correction saved from an earlier session, if any.
  verify     a square at your speeds; reports every stop and the heading.

Speeds are axle-centre rpm in tens: straights 40..250, arcs 60..170 (the V
suffix, PROTOCOL.md §13). Tasks only get the table when they drive at those
speeds - an anchor at 150 is used by anything near 150 and held flat beyond.
"""

import argparse
import csv
import json
import logging
import math
import os
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                    datefmt="%H:%M:%S")

import calibrate as c9  # noqa: E402  (shared prompts, gyro check, connect)
from communications import cal_profile  # noqa: E402
from communications.stm import (CAL_TABLE_ANCHORS, CAL_TABLE_GLOBALS,  # noqa: E402
                                 CAL_TABLE_MOVES, STM, TABLE_MIN_PROTOCOL,
                                 V_ARC, V_STRAIGHT, k_field_ok)

ARC_MOVES = ("FR", "FL", "RR", "RL")
SERVO_CENTER_US = 1474          # motors.h

STRAIGHT_SPEEDS = (100, 180)    # rpm
ARC_SPEEDS = (100, 150)
STRAIGHT_CM = 100
ARC_DEG = 90
RUNS = 2
MAX_ROUNDS = 3

# What "nothing left to fix" means.
TRIM_OK_US = 4.0                # mean steering correction
TRIM_GAIN = 0.8                 # of the measured correction, per round
COAST_OK_MM = 2.0
DIST_OK = 0.002                 # 0.2 %
WHEEL_OK = 5                    # x10000, i.e. 0.05 %
STOP_OK_DEG = 0.4               # mean arc stop error worth refitting for
DECEL_OK_PCT = 4.0
RADIUS_TOL_CM = 0.5
RADIUS_SLOPE_CM_PER_US = -0.04  # wider steer -> smaller radius; secant replaces it
RADIUS_MAX_STEP_US = 60
FF_MIN_SPREAD_RPM = 40          # speeds this far apart before a line is fitted
FF_MIN_SAMPLES = 20             # cruise ticks a run needs to count
HOLD_MAX_ERR_DEG = 3.0          # straights that strayed more say nothing of wheels
# Firmware with MOTION_ARC_TAPER brakes arcs from a crawl (~25 rpm), so the
# coast after the brake is a fraction of a degree - too small to fit a
# braking model from. Below this mean coast the braking is not refitted;
# the stop error is reported instead.
MIN_FIT_COAST_DEG = 0.5
# A tape chord more than this fraction away from the odometry (or from this
# move's other tapes) is asked about before it is used: the good ones on
# this robot sit within ~4 %, and one misread corrupts every radius of that
# move.
TAPE_SUSPECT = 0.08

# Protocol 9 values a move uses when the table has none (motion.h).
P9_COAST_MM = 8.0


# ── Small helpers ─────────────────────────────────────────────────────────────

def mean(v):
    v = list(v)
    return sum(v) / len(v) if v else None


def sd(v):
    v = list(v)
    if len(v) < 2:
        return 0.0
    m = mean(v)
    return (sum((x - m) ** 2 for x in v) / (len(v) - 1)) ** 0.5


def fmt(v, nd=1, sign=False):
    if v is None:
        return "-"
    return f"{v:+.{nd}f}" if sign else f"{v:.{nd}f}"


def parse_speeds(text: str, arc: bool) -> List[int]:
    lo, hi = V_ARC if arc else V_STRAIGHT
    out = []
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        rpm = int(part)
        if rpm % 10 or not (lo * 10 <= rpm <= hi * 10):
            raise ValueError(f"speed {rpm}: must be a multiple of 10 in "
                             f"{lo * 10}..{hi * 10} rpm for "
                             f"{'arcs' if arc else 'straights'}")
        if rpm not in out:
            out.append(rpm)
    if not out:
        raise ValueError("no speeds given")
    if len(out) > CAL_TABLE_ANCHORS:
        raise ValueError(f"at most {CAL_TABLE_ANCHORS} speeds - the firmware "
                         f"holds {CAL_TABLE_ANCHORS} anchors per move")
    return out


def token(move: str, amount: int, rpm: int) -> str:
    return f"{move}{amount}V{rpm // 10}"


# ── The table as the profile stores it ───────────────────────────────────────
#
# profile["table"] = {"FR": [{"rpm": 150, "steer": 610, ...}, ...],
#                     "F": [...], "G": {"wheel": 10031, ...}}
# Keys in proposals: (move, rpm, field) - move "G" and rpm None for globals.

def table(profile: dict) -> dict:
    return profile.setdefault("table", {})


def get_value(tab: dict, move: str, rpm: Optional[int], field: str):
    if move == "G":
        return (tab.get("G") or {}).get(field) or None
    for a in tab.get(move) or []:
        if a.get("rpm") == rpm:
            return a.get(field)
    return None


def effective(tab: dict, move: str, rpm: float, field: str):
    """What the firmware will use for `field` on `move` at `rpm`: the
    piecewise-linear interpolation CalTab_Lookup() does, None if unset."""
    pts = sorted((a["rpm"], a[field]) for a in tab.get(move) or []
                 if a.get(field) is not None and a.get("rpm"))
    if not pts:
        return None
    if rpm <= pts[0][0]:
        return float(pts[0][1])
    if rpm >= pts[-1][0]:
        return float(pts[-1][1])
    for (r0, v0), (r1, v1) in zip(pts, pts[1:]):
        if rpm <= r1:
            return v0 + (rpm - r0) / (r1 - r0) * (v1 - v0)
    return float(pts[-1][1])


def set_value(tab: dict, move: str, rpm: Optional[int], field: str, value) -> None:
    """Store one value; rpm 0 for field 'rpm' drops that anchor. Raises
    ValueError if the firmware would refuse it."""
    if move == "G":
        ok, why = k_field_ok(None, field, int(value))
        if not ok:
            raise ValueError(why)
        g = tab.setdefault("G", {})
        if int(value) == 0:
            g.pop(field, None)
        else:
            g[field] = int(value)
        return
    if move not in CAL_TABLE_MOVES:
        raise ValueError(f"unknown move {move!r}")
    anchors = tab.setdefault(move, [])
    if field == "rpm":
        if int(value) == 0:
            tab[move] = [a for a in anchors if a.get("rpm") != rpm]
            return
        raise ValueError("use <move>@<rpm>.<field>=... - the rpm IS the anchor")
    ok, why = k_field_ok(move, field, int(value))
    if not ok:
        raise ValueError(why)
    for a in anchors:
        if a.get("rpm") == rpm:
            a[field] = int(value)
            return
    if len(anchors) >= CAL_TABLE_ANCHORS:
        raise ValueError(f"{move} already has {CAL_TABLE_ANCHORS} speeds "
                         f"({[a['rpm'] for a in anchors]}); drop one first "
                         f"({move}@<rpm>.rpm=0)")
    ok, why = k_field_ok(move, "rpm", int(rpm))
    if not ok:
        raise ValueError(why)
    anchors.append({"rpm": int(rpm), field: int(value)})
    anchors.sort(key=lambda a: a["rpm"])


def key_name(key) -> str:
    move, rpm, field = key
    return f"G.{field}" if move == "G" else f"{move}@{rpm}.{field}"


def parse_key(text: str):
    """'FR@150.steer' / 'G.wheel' -> (move, rpm, field)."""
    text = text.strip()
    if text.upper().startswith("G."):
        return ("G", None, text[2:].lower())
    head, _, field = text.partition(".")
    move, _, rpm = head.partition("@")
    if not field or not rpm:
        raise ValueError(f"{text!r}: write MOVE@RPM.FIELD or G.FIELD")
    return (move.upper(), int(rpm), field.lower())


# ── Proposals and approval ────────────────────────────────────────────────────

class Proposal:
    def __init__(self, key, old, new, why: str, scale: float = 1.0):
        self.key = key
        self.old = old
        self.new = int(round(new))
        self.why = why
        self.scale = scale          # display divisor (x10 fields shown as units)

    def shown(self, v):
        if v is None:
            return "-"
        if self.scale != 1.0:
            return f"{v / self.scale:.1f}"
        return f"{int(round(v)):+d}" if self.key[2] == "trim" else f"{int(round(v))}"


def print_proposals(props: List[Proposal]) -> None:
    print()
    print("    #  value                  now  ->  new      why")
    for i, p in enumerate(props, 1):
        print(f"   {i:2d}  {key_name(p.key):<18} {p.shown(p.old):>7}  ->  "
              f"{p.shown(p.new):<8} {p.why}")


def approve(props: List[Proposal], ask=None) -> List[Proposal]:
    """Ask which proposals to apply. Returns the accepted ones (edited)."""
    ask = ask or c9.ask
    if not props:
        return []
    while True:
        print_proposals(props)
        raw = ask("Enter = apply all, n = none, numbers = only those, "
                  "'e N VALUE' = edit #N:").strip().lower()
        if raw == "":
            return props
        if raw in ("n", "none", "no"):
            return []
        parts = raw.split()
        if parts[0] == "e":
            try:
                i, val = int(parts[1]), float(parts[2])
                p = props[i - 1]
                p.new = int(round(val * p.scale))
                p.why = "edited by you"
            except (IndexError, ValueError):
                print("    e.g. 'e 2 40' sets #2 to 40 (in the units shown).")
            continue
        try:
            picks = sorted({int(x) for x in parts})
            if all(1 <= i <= len(props) for i in picks):
                return [props[i - 1] for i in picks]
        except ValueError:
            pass
        print("    Enter, n, numbers like '1 3', or 'e 2 40'.")


def apply_props(stm: Optional[STM], profile: dict, name: str,
                props: List[Proposal]) -> bool:
    """Into the profile, onto the robot (whole table, read back), saved."""
    tab = table(profile)
    for p in props:
        try:
            set_value(tab, p.key[0], p.key[1], p.key[2], p.new)
        except ValueError as exc:
            logging.error(f"{key_name(p.key)}={p.new} not applied: {exc}")
            return False
    if stm is not None and not cal_profile.push_table(stm, tab):
        return False
    cal_profile.save(name, profile)
    logging.info(f"Applied {len(props)} value(s); saved {cal_profile.profile_path(name)}.")
    return True


# ── Talking to the robot ──────────────────────────────────────────────────────

class RunLog:
    """Every move's ?MV report, one CSV row each, appended per profile."""

    FIELDS = ["time", "step", "round", "token", "reply", "tape_cm"] + list(STM.MV_FIELDS)

    def __init__(self, name: str):
        os.makedirs(cal_profile.PROFILE_DIR, exist_ok=True)
        self.path = os.path.join(cal_profile.PROFILE_DIR, f"{name}_runs.csv")

    def write(self, step: str, rnd: int, tok: str, reply: str, mv: dict,
              tape_cm=None) -> None:
        # Opened per row: a session cut short with Ctrl-C keeps every row.
        new = not os.path.exists(self.path)
        row = {"time": datetime.now().isoformat(timespec="seconds"),
               "step": step, "round": rnd, "token": tok, "reply": reply,
               "tape_cm": "" if tape_cm is None else tape_cm}
        row.update(mv)
        with open(self.path, "a", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=self.FIELDS, extrasaction="ignore")
            if new:
                w.writeheader()
            w.writerow(row)

    def close(self):
        pass


def describe(mv: dict) -> str:
    """One line per move: what the report says."""
    if mv["move"] in ("F", "R"):
        return (f"enc A {mv['enc_a']:+d} B {mv['enc_b']:+d} "
                f"(A/B {abs(mv['enc_a']) / max(1, abs(mv['enc_b'])):.4f})  "
                f"rpm {mv['rpm_a']:.0f}/{mv['rpm_b']:.0f}  "
                f"duty {mv['duty_a']}/{mv['duty_b']}  "
                f"steer {mv['corr']:+.1f}µs  worst {mv['max_err']:.1f}°  "
                f"coast {mv['dist'] - mv['dist_brake']:.1f}mm  "
                f"drift {mv['y']:+.1f}mm")
    return (f"aim {abs(mv['aim']):.1f}° stop {abs(mv['turned']) - abs(mv['aim']):+.2f}°  "
            f"brake at {mv['brake_rate']:.0f}°/s coast "
            f"{abs(mv['turned']) - mv['brake_ang']:.1f}°  "
            f"chord {math.hypot(mv['x'], mv['y']) / 10:.1f}cm  "
            f"rpm {mv['rpm_a']:.0f}/{mv['rpm_b']:.0f}  "
            f"duty {mv['duty_a']}/{mv['duty_b']}  servo {mv['servo']}")


def run_move(stm: STM, tok: str, log: Optional[RunLog], step: str, rnd: int,
             tape_prompt: Optional[str] = None) -> Optional[dict]:
    """One movement line; the report; optionally a tape. None on failure."""
    before = stm.read_mv()
    if not stm.send_line([tok]):
        return None
    reply = stm.wait_reply()
    if reply is None:
        logging.error(f"{tok}: no reply - lost link.")
        return None
    mv = stm.read_mv()
    if mv is None:
        return None
    if before is not None and mv["seq"] == before["seq"]:
        logging.error(f"{tok}: the move report did not change - the move never "
                      "launched.")
        return None
    reply = reply.strip()
    tape = None
    print(f"    {tok:<10} {reply:<14} {describe(mv)}")
    if tape_prompt:
        tape = c9.ask_float(tape_prompt)
    mv["tape_cm"] = tape
    if log is not None:
        log.write(step, rnd, tok, reply, mv, tape)
    if reply.upper() != "OK":
        logging.error(f"{tok}: {reply} - the robot is not where it should be. "
                      "Stopping this step.")
        return None
    return mv


def connect(stm: STM, profile: dict) -> bool:
    if not c9.connect_and_check(stm):
        return False
    proto = cal_profile.firmware_protocol(stm)
    if proto is None or proto < TABLE_MIN_PROTOCOL:
        logging.error(f"Firmware is protocol {proto}; this tool needs "
                      f"{TABLE_MIN_PROTOCOL}. Flash the current build.")
        return False
    arc = cal_profile.arc_of(profile)
    if not c9.select_arc(stm, arc):
        return False
    if not cal_profile.push(stm, profile, proto):
        return False
    return True


def load_or_start(stm: Optional[STM], name: str, base: Optional[str]) -> Optional[dict]:
    """The profile to work on: NAME if saved, else a copy of BASE, else the
    robot's current protocol 9 values with an empty table."""
    if os.path.exists(cal_profile.profile_path(name)):
        p = cal_profile.load(name)
        if p is not None and base:
            logging.warning(f"{name!r} exists - --from {base} ignored.")
        return p
    if base:
        p = cal_profile.load(base)
        if p is None:
            return None
        p = json.loads(json.dumps(p))
        p.pop("verified", None)
        p["note"] = f"from {base}; " + p.get("note", "")
        p["checks"] = {}
        logging.info(f"Starting {name!r} from {base!r}.")
        return p
    if stm is None:
        logging.error(f"No profile {name!r}; give --from <profile>.")
        return None
    cur = c9.current_values(stm)
    if cur is None:
        return None
    logging.info(f"Starting {name!r} from the robot's current values.")
    p = {k: cur[k] for k in ("decel_x10", "lag_ms_x10", "trim_us",
                              "gyro_x10000", "steer_left_us", "steer_right_us")}
    p.update({"arc_profile": cal_profile.ARC_PROFILE, "checks": {}, "table": {},
              "note": ""})
    return p


# ── Analysis: straights ──────────────────────────────────────────────────────

def fit_line(points):
    """Least squares y = a + b x. None if the x spread is too small."""
    xs = [p[0] for p in points]
    if len(points) < 2 or max(xs) - min(xs) < FF_MIN_SPREAD_RPM:
        return None
    mx, my = mean(xs), mean(p[1] for p in points)
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in points)
    b = sxy / sxx
    return my - b * mx, b


def analyse_straights(runs: List[dict], tab: dict, profile: dict):
    """
    runs: ?MV reports (with "tape_cm") of F and R moves.
    Returns (summary rows, proposals, findings).
    """
    props: List[Proposal] = []
    notes: List[str] = []
    rows = []
    groups: Dict[Tuple[str, int], List[dict]] = {}
    for mv in runs:
        groups.setdefault((mv["move"], mv["rpm"]), []).append(mv)

    # Wheel ratio: right/left ground per count = A counts / B counts on any
    # run the steering held straight.
    ratios = [abs(mv["enc_a"]) / abs(mv["enc_b"]) for mv in runs
              if mv["max_err"] <= HOLD_MAX_ERR_DEG and mv["enc_b"]
              and abs(mv["enc_a"]) > 1000]
    w_now = get_value(tab, "G", None, "wheel")
    if ratios:
        w = mean(ratios) * 10000
        if abs(w - (w_now or 10000)) >= WHEEL_OK:
            side = "right" if w > 10000 else "left"
            props.append(Proposal(("G", None, "wheel"), w_now, w,
                                  f"{side} wheel covers {abs(w - 10000) / 100:.2f}% "
                                  f"more ground per count ({len(ratios)} runs, "
                                  f"sd {sd(ratios) * 100:.3f}%)"))

    for (move, rpm), g in sorted(groups.items()):
        corr = [mv["corr"] for mv in g]
        coast = [mv["dist"] - mv["dist_brake"] for mv in g]
        coast_now = effective(tab, move, rpm, "coast")
        coast_now_mm = coast_now / 10.0 if coast_now is not None else P9_COAST_MM
        # Odometry past the aim: dist - (target + the coast allowed for).
        over = [mv["dist"] - (mv["target"] + coast_now_mm) for mv in g]
        tapes = [(mv["tape_cm"] * 10.0) / mv["dist"] for mv in g
                 if mv.get("tape_cm") and mv["dist"] > 0]
        row = {"move": move, "rpm": rpm, "runs": len(g),
               "corr": mean(corr), "corr_sd": sd(corr),
               "max_err": max(mv["max_err"] for mv in g),
               "coast": mean(coast), "over": mean(over), "over_sd": sd(over),
               "drift": mean(mv["y"] for mv in g),
               "rpm_a": mean(mv["rpm_a"] for mv in g),
               "rpm_b": mean(mv["rpm_b"] for mv in g),
               "duty_a": mean(mv["duty_a"] for mv in g),
               "duty_b": mean(mv["duty_b"] for mv in g),
               "tape_ratio": mean(tapes) if tapes else None,
               "trim": g[-1]["trim"]}
        rows.append(row)

        # "now" is always the value the move drove on, table or not.
        if abs(row["corr"]) > TRIM_OK_US:
            new = row["trim"] + TRIM_GAIN * row["corr"]
            props.append(Proposal((move, rpm, "trim"), row["trim"], new,
                                  f"steering held {row['corr']:+.1f} µs on average "
                                  f"({len(g)} runs, sd {row['corr_sd']:.1f})"))
        if abs(row["over"]) > COAST_OK_MM:
            new = max(0.0, min(40.0, coast_now_mm + row["over"])) * 10
            props.append(Proposal((move, rpm, "coast"), coast_now_mm * 10, new,
                                  f"stops {row['over']:+.1f} mm past its mark by "
                                  f"odometry (sd {row['over_sd']:.1f})", scale=10.0))
        if tapes:
            d_now = effective(tab, move, rpm, "dist") or 10000
            d = mean(tapes) * 10000
            if abs(d - d_now) / 10000 > DIST_OK:
                props.append(Proposal((move, rpm, "dist"), d_now, d,
                                      f"tape/odometry {mean(tapes):.4f} "
                                      f"({len(tapes)} taped, sd {sd(tapes) * 100:.2f}%)"))

    # Motor lines: per wheel and direction, duty against rpm across speeds.
    lines = {}
    for wheel in ("a", "b"):
        for move, d in (("F", "f"), ("R", "r")):
            pts = [(mv[f"rpm_{wheel}"], mv[f"duty_{wheel}"]) for mv in runs
                   if mv["move"] == move and mv["n"] >= FF_MIN_SAMPLES]
            fit = fit_line(pts)
            if fit is not None and fit[1] > 0:
                lines[(wheel, d)] = (fit, pts)
    for wheel in ("a", "b"):
        fits = [lines[(wheel, d)] for d in ("f", "r") if (wheel, d) in lines]
        if not fits:
            continue
        db = mean(f[0][0] for f in fits)
        db_ok = 300 <= db <= 900
        db_use = db if db_ok else (get_value(tab, "G", None, f"db{wheel}") or 600)
        if db_ok:
            old = get_value(tab, "G", None, f"db{wheel}")
            if old is None or abs(old - db) >= 10:
                props.append(Proposal(("G", None, f"db{wheel}"), old, db,
                                      f"wheel {wheel.upper()} starts turning at "
                                      f"~{db:.0f} duty"))
        for d in ("f", "r"):
            if (wheel, d) not in lines:
                continue
            (_a, _b), pts = lines[(wheel, d)]
            rm = mean(p[0] for p in pts)
            dm = mean(p[1] for p in pts)
            if dm - db_use <= 1:
                continue
            ff = rm * (1000 - db_use) / (dm - db_use)
            old = get_value(tab, "G", None, f"ff{wheel}{d}")
            if 150 <= ff <= 600 and (old is None or abs(old - ff) >= 5):
                props.append(Proposal(("G", None, f"ff{wheel}{d}"), old, ff,
                                      f"wheel {wheel.upper()} "
                                      f"{'forward' if d == 'f' else 'reverse'}: "
                                      f"{dm:.0f} duty for {rm:.0f} rpm"))

    # Which motor works harder - the thing to know about this robot.
    for move in ("F", "R"):
        g = [r for r in rows if r["move"] == move and r["duty_a"] and r["duty_b"]]
        if g:
            ratio = mean(r["duty_b"] / r["duty_a"] for r in g)
            if abs(ratio - 1) > 0.03:
                harder = "B (right)" if ratio > 1 else "A (left)"
                notes.append(f"{move}: wheel {harder} needs {abs(ratio - 1) * 100:.0f}% "
                             "more duty for the same speed - it is the weaker or "
                             "draggier one.")
    return rows, props, notes


def print_straight_rows(rows) -> None:
    print("\n    move  rpm runs  steer µs (sd)  worst°  coast mm  past mark mm  "
          "drift mm  rpm A/B      duty A/B   tape/odo")
    for r in rows:
        print(f"    {r['move']:<4} {r['rpm']:>4} {r['runs']:>4}  "
              f"{r['corr']:+6.1f} ({r['corr_sd']:.1f})  {r['max_err']:5.1f}  "
              f"{r['coast']:7.1f}  {r['over']:+8.1f} ({r['over_sd']:.1f})  "
              f"{r['drift']:+7.1f}  {r['rpm_a']:5.0f}/{r['rpm_b']:<5.0f} "
              f"{r['duty_a']:4.0f}/{r['duty_b']:<4.0f}  "
              f"{fmt(r['tape_ratio'], 4)}")


# ── Analysis: arcs ───────────────────────────────────────────────────────────

def arc_radius_cm(mv: dict, factor: float = 1.0) -> Optional[float]:
    t = math.radians(abs(mv["turned"]))
    if t < math.radians(20):
        return None
    chord = math.hypot(mv["x"], mv["y"]) / 10.0 * factor
    return chord / (2.0 * math.sin(t / 2.0))


def fit_brakes(samples, lag_now_s: float):
    """
    samples: [(w deg/s at brake, coast deg)]. Model coast = w²/(2a) + w·lag.
    Returns (decel dps², lag s, fitted_jointly). With one speed's worth of
    rates the two cannot be told apart, so lag is held at lag_now_s.
    """
    pts = [(w, c) for w, c in samples if w > 5 and c > 0.2]
    if not pts:
        return None
    ws = [w for w, _ in pts]
    if len(pts) >= 3 and max(ws) / min(ws) >= 1.3:
        # least squares on c = p w² + q w
        s44 = sum(w ** 4 for w in ws)
        s33 = sum(w ** 3 for w in ws)
        s22 = sum(w ** 2 for w in ws)
        b2 = sum(c * w * w for w, c in pts)
        b1 = sum(c * w for w, c in pts)
        det = s44 * s22 - s33 * s33
        if det > 0:
            p = (b2 * s22 - b1 * s33) / det
            q = (s44 * b1 - s33 * b2) / det
            if p > 0 and 0 <= q <= 0.25:
                return 1.0 / (2 * p), q, True
    q = lag_now_s
    ps = [(c - q * w) / (w * w) for w, c in pts if c - q * w > 0]
    if not ps:
        return None
    return 1.0 / (2 * mean(ps)), q, False


def analyse_arcs(runs: List[dict], tab: dict, profile: dict,
                 target: Optional[str], factors: Dict[str, float],
                 history: Dict[tuple, list]):
    """
    runs: ?MV reports of arcs. factors: tape/odometry chord per move.
    target: None/"keep", "match", or a radius in cm as a string.
    history: (move, rpm) -> [(steer µs, radius cm)] from earlier rounds,
    for the steering slope; this round's points are appended.
    Returns (rows, proposals, notes).
    """
    props: List[Proposal] = []
    notes: List[str] = []
    rows = []
    groups: Dict[Tuple[str, int], List[dict]] = {}
    for mv in runs:
        groups.setdefault((mv["move"], mv["rpm"]), []).append(mv)

    lag_global = profile.get("lag_ms_x10", 0) / 10000.0
    decel_global = profile.get("decel_x10", 6360) / 10.0

    # Braking, per move across its speeds.
    fits = {}
    for move in ARC_MOVES:
        g = [mv for mv in runs if mv["move"] == move]
        if not g:
            continue
        lag_now = effective(tab, move, g[0]["rpm"], "lag")
        lag_now = lag_now / 10000.0 if lag_now is not None else lag_global
        fit = fit_brakes([(mv["brake_rate"], abs(mv["turned"]) - mv["brake_ang"])
                          for mv in g], lag_now)
        if fit is not None:
            fits[move] = fit

    for (move, rpm), g in sorted(groups.items()):
        stops = [abs(mv["turned"]) - abs(mv["aim"]) for mv in g]
        f = factors.get(move)
        radii = [r for r in (arc_radius_cm(mv, f or 1.0) for mv in g) if r]
        steer = abs(g[-1]["servo"] - SERVO_CENTER_US)
        row = {"move": move, "rpm": rpm, "runs": len(g),
               "stop": mean(stops), "stop_sd": sd(stops),
               "w": mean(mv["brake_rate"] for mv in g),
               "radius": mean(radii) if radii else None,
               "radius_sd": sd(radii) if radii else None,
               "radius_src": "tape-corrected" if f else "odometry only",
               "steer": steer,
               "rpm_a": mean(mv["rpm_a"] for mv in g),
               "rpm_b": mean(mv["rpm_b"] for mv in g),
               "duty_a": mean(mv["duty_a"] for mv in g),
               "duty_b": mean(mv["duty_b"] for mv in g)}
        rows.append(row)
        if row["radius"] is not None:
            history.setdefault((move, rpm), []).append((steer, row["radius"]))

        # Braking: per anchor, from this anchor's coasts with the move's lag.
        fit = fits.get(move)
        coasts = [abs(mv["turned"]) - mv["brake_ang"] for mv in g]
        if mean(coasts) < MIN_FIT_COAST_DEG:
            fit = None
            notes.append(f"{move}@{rpm}: coasts only {mean(coasts):.2f}° after the "
                         f"brake (tapered arc) - braking not refitted; stops "
                         f"{row['stop']:+.2f}° from its aim.")
        if fit is not None and abs(row["stop"]) <= STOP_OK_DEG:
            # Stops are already on target. The fit has more freedom than
            # these few coasts can pin down (decel and lag trade against
            # each other), so refitting here only swaps one good pair for
            # another round after round. Leave it.
            fit = None
        if fit is not None:
            _decel, lag_s, joint = fit
            samples = [(mv["brake_rate"], abs(mv["turned"]) - mv["brake_ang"]) for mv in g]
            per = [w * w / (2 * (c - lag_s * w)) for w, c in samples
                   if w > 5 and c - lag_s * w > 0.2]
            d_now = effective(tab, move, rpm, "decel")
            d_now = d_now / 10.0 if d_now is not None else decel_global
            if per:
                d_new = mean(per)
                if abs(d_new - d_now) / d_now * 100 > DECEL_OK_PCT:
                    props.append(Proposal((move, rpm, "decel"), d_now * 10,
                                          d_new * 10,
                                          f"stops {row['stop']:+.2f}° from its aim "
                                          f"(sd {row['stop_sd']:.2f}); coasts imply "
                                          f"{d_new:.0f} dps²", scale=10.0))
            if joint:
                l_now = effective(tab, move, rpm, "lag")
                l_now = l_now / 10.0 if l_now is not None else lag_global * 1000
                if abs(lag_s * 1000 - l_now) >= 2.0:
                    props.append(Proposal((move, rpm, "lag"), l_now * 10,
                                          lag_s * 10000,
                                          f"brake lag fitted across speeds: "
                                          f"{lag_s * 1000:.1f} ms", scale=10.0))

    # Steering for the radius.
    if target and target != "keep":
        for row in rows:
            if row["radius"] is None:
                continue
            if target == "match":
                slow = [r for r in rows if r["move"] == row["move"]
                        and r["radius"] is not None]
                want = min(slow, key=lambda r: r["rpm"])["radius"]
                if row is min(slow, key=lambda r: r["rpm"]):
                    continue
            else:
                want = float(target)
            err = row["radius"] - want
            if abs(err) <= RADIUS_TOL_CM:
                continue
            pts = history.get((row["move"], row["rpm"]), [])
            slope = RADIUS_SLOPE_CM_PER_US
            distinct = sorted({s for s, _ in pts})
            if len(distinct) >= 2:
                (s0, r0), (s1, r1) = pts[-2], pts[-1]
                if s1 != s0 and (r1 - r0) / (s1 - s0) < -0.005:
                    slope = (r1 - r0) / (s1 - s0)
            step = max(-RADIUS_MAX_STEP_US, min(RADIUS_MAX_STEP_US, err / -slope))
            lo, hi = (300, 626) if row["move"] in ("FR", "RR") else (300, 574)
            new = max(lo, min(hi, row["steer"] + step))
            if int(round(new)) == row["steer"] and row["steer"] in (lo, hi):
                notes.append(f"{row['move']}@{row['rpm']}: radius {row['radius']:.1f} "
                             f"cm cannot reach {want:.1f} - steering is at its "
                             f"{hi if row['steer'] == hi else lo} µs limit. Plan "
                             "this turn with its own radius.")
            if int(round(new)) != row["steer"]:
                props.append(Proposal((row["move"], row["rpm"], "steer"),
                                      row["steer"], new,
                                      f"radius {row['radius']:.1f} cm, want "
                                      f"{want:.1f} ({row['radius_src']})"))

    for move in ARC_MOVES:
        g = [r for r in rows if r["move"] == move and r["radius"]]
        if len(g) >= 2:
            lo, hi = min(g, key=lambda r: r["rpm"]), max(g, key=lambda r: r["rpm"])
            notes.append(f"{move}: radius {lo['radius']:.1f} cm at {lo['rpm']} rpm, "
                         f"{hi['radius']:.1f} cm at {hi['rpm']} rpm "
                         f"({(hi['radius'] - lo['radius']):+.1f} cm with speed).")
    for fwd, rev in (("FR", "RR"), ("FL", "RL")):
        for rpm in sorted({r["rpm"] for r in rows}):
            a = next((r for r in rows if r["move"] == fwd and r["rpm"] == rpm), None)
            b = next((r for r in rows if r["move"] == rev and r["rpm"] == rpm), None)
            if a and b and a["radius"] and b["radius"] and \
                    abs(a["radius"] - b["radius"]) > 1.0:
                notes.append(f"{fwd}/{rev} at {rpm} rpm trace {a['radius']:.1f} / "
                             f"{b['radius']:.1f} cm - the table steers each its own.")
    return rows, props, notes


def print_arc_rows(rows) -> None:
    print("\n    move  rpm runs  stop° (sd)     brake°/s  radius cm (sd)        "
          "steer  rpm A/B     duty A/B")
    for r in rows:
        print(f"    {r['move']:<4} {r['rpm']:>4} {r['runs']:>4}  "
              f"{r['stop']:+.2f} ({r['stop_sd']:.2f})  {r['w']:8.0f}  "
              f"{fmt(r['radius'])} ({fmt(r['radius_sd'], 2)}) "
              f"{'' if r['radius_src'] == 'tape-corrected' else '*'}"
              f"{'':<6} {r['steer']:5d}  {r['rpm_a']:4.0f}/{r['rpm_b']:<4.0f}  "
              f"{r['duty_a']:4.0f}/{r['duty_b']:<4.0f}")
    if any(r["radius_src"] != "tape-corrected" for r in rows):
        print("    * odometry chord only - it reads long on arcs (the rear "
              "wheels scrub). Use --tape once.")


# ── Steps ─────────────────────────────────────────────────────────────────────

def step_straights(stm: STM, profile: dict, name: str, log: RunLog,
                   speeds: List[int], runs: int, cm: int, tape: bool) -> bool:
    print("\n  ── STRAIGHTS ───────────────────────────────────────────────────")
    print(f"  F{cm} then R{cm} at {', '.join(map(str, speeds))} rpm, {runs} each.")
    print(f"  Needs {cm + 30} cm clear ahead; it comes back by itself.")
    if tape:
        print("  Mark the floor under the rear axle before each forward run;")
        print("  after each move type how far the axle moved, in cm.")
    allruns: List[dict] = []
    for rnd in range(1, MAX_ROUNDS + 1):
        if not c9.position_and_zero(
                stm, f"Round {rnd}: robot on its start mark, straight along the strip."):
            return False
        batch = []
        for rpm in speeds:
            for _ in range(runs):
                for move in ("F", "R"):
                    mv = run_move(stm, token(move, cm, rpm), log, "straights", rnd,
                                  f"tape {move}: cm moved (Enter = skip):" if tape else None)
                    if mv is None:
                        return False
                    batch.append(mv)
        if not c9.gyro_ok(stm):
            return False
        allruns += batch
        rows, props, notes = analyse_straights(batch, table(profile), profile)
        print_straight_rows(rows)
        for n in notes:
            print(f"    · {n}")
        profile.setdefault("checks", {}).setdefault("straights", []).append(
            {"round": rnd, "rows": rows})
        if not props:
            print("\n  Nothing left to fix in the straights.")
            cal_profile.save(name, profile)
            return True
        accepted = approve(props)
        if not accepted:
            cal_profile.save(name, profile)
            return True
        if not apply_props(stm, profile, name, accepted):
            return False
        if rnd < MAX_ROUNDS and c9.ask("Run again to confirm? [Y/n]").lower() == "n":
            return True
    return True


def step_arcs(stm: STM, profile: dict, name: str, log: RunLog,
              moves: List[str], speeds: List[int], runs: int, deg: int,
              tape: bool, target: Optional[str]) -> bool:
    print("\n  ── ARCS ────────────────────────────────────────────────────────")
    pairs = []
    for fwd, rev in (("FR", "RR"), ("FL", "RL")):
        if fwd in moves or rev in moves:
            pairs.append((fwd, rev))
    print(f"  {', '.join(f'{a}+{b}' for a, b in pairs)} {deg}° at "
          f"{', '.join(map(str, speeds))} rpm, {runs} each. Each pair returns "
          "to the start.")
    print("  Needs about 70 x 70 cm clear on the side it turns to.")
    checks = profile.setdefault("checks", {})
    # Tape/odometry chord per move: this session's tapes when there are any,
    # else what an earlier session saved.
    factors: Dict[str, float] = dict(checks.get("arc_chord_factor", {}))
    taped_k: Dict[str, List[float]] = {}
    history: Dict[tuple, list] = {}
    if tape:
        print("  Tape: mark under the rear axle before each arc, then the straight")
        print("  distance between the marks in cm. Asked on the first arc of each")
        print("  move and speed; the rest use its odometry correction.")
    for rnd in range(1, MAX_ROUNDS + 1):
        if not c9.position_and_zero(stm, f"Round {rnd}: robot on its start mark."):
            return False
        batch = []
        taped = set()
        for fwd, rev in pairs:
            for rpm in speeds:
                for _ in range(runs):
                    for move in (fwd, rev):
                        ask_tape = tape and rnd == 1 and (move, rpm) not in taped
                        mv = run_move(stm, token(move, deg, rpm), log, "arcs", rnd,
                                      f"tape {move}: chord cm (Enter = skip):"
                                      if ask_tape else None)
                        if mv is None:
                            return False
                        if ask_tape and mv.get("tape_cm"):
                            taped.add((move, rpm))
                            odo = math.hypot(mv["x"], mv["y"]) / 10.0
                            if odo > 5:
                                k = mv["tape_cm"] / odo
                                others = taped_k.get(move) or [factors.get(move, 1.0)]
                                use = True
                                if (abs(k - 1.0) > TAPE_SUSPECT or
                                        abs(k - mean(others)) > TAPE_SUSPECT):
                                    logging.warning(
                                        f"    {move}: tape {mv['tape_cm']} cm against "
                                        f"odometry {odo:.1f} cm (x{k:.3f}) - far "
                                        "from the others; probably a misread.")
                                    use = c9.ask("Use this tape anyway? [y/N]").lower() == "y"
                                    if not use:
                                        taped.discard((move, rpm))   # asks again next run
                                if use:
                                    taped_k.setdefault(move, []).append(k)
                                    factors[move] = mean(taped_k[move])
                                    logging.info(f"    {move}: tape/odometry chord {k:.4f}")
                        if move in moves:
                            batch.append(mv)
        if not c9.gyro_ok(stm):
            return False
        checks["arc_chord_factor"] = {m: round(k, 5) for m, k in factors.items()}
        rows, props, notes = analyse_arcs(batch, table(profile), profile,
                                          target, factors, history)
        print_arc_rows(rows)
        for n in notes:
            print(f"    · {n}")
        checks.setdefault("arcs", []).append({"round": rnd, "rows": rows})
        if not props:
            print("\n  Nothing left to fix in the arcs.")
            cal_profile.save(name, profile)
            return True
        accepted = approve(props)
        if not accepted:
            cal_profile.save(name, profile)
            return True
        if not apply_props(stm, profile, name, accepted):
            return False
        if rnd < MAX_ROUNDS and c9.ask("Run again to confirm? [Y/n]").lower() == "n":
            return True
    return True


def step_gyro(stm: STM, profile: dict, name: str) -> bool:
    print("\n  ── GYRO SCALE ──────────────────────────────────────────────────")
    print("  Side flush against a straight edge; four quarter turns bring it")
    print("  back beside it. Angle from the edge: + past it, - short of it;")
    print("  degrees, or offset/span in mm along its side (e.g. +6/200).")
    cal = c9.current_values(stm)
    if cal is None:
        return False
    scale = cal["gyro_x10000"] / 10000.0
    ratios = []
    for tok, count in c9.GYRO_LOOPS:
        if not c9.position_and_zero(stm, f"Robot flush with the edge for {count} x {tok}."):
            return False
        for _ in range(count):
            if not c9.send_move(stm, tok):
                return False
        hdg = stm.read_heading()
        if hdg is None:
            return False
        seen = c9.ask_angle(f"{tok} x{count}: angle from the edge (+ past, - short):")
        ratios.append((360.0 * count / 4.0 + seen) / abs(hdg["actual"]))
    new = int(round(scale * mean(ratios) * 10000))
    asym = abs(ratios[0] - ratios[-1]) * 100
    print(f"\n    right loop ×{ratios[0]:.4f}, left loop ×{ratios[-1]:.4f} "
          f"(differ {asym:.2f}%{' - re-measure, a scale error is the same both ways' if asym > 0.5 else ''})")
    if abs(new - cal["gyro_x10000"]) < 3:
        print("    Gyro scale is already right.")
        return True
    print(f"\n     1  gyro scale  ×{scale:.4f}  ->  ×{new / 10000:.4f}   "
          "full circles against the edge")
    if c9.ask("Apply? [Y/n]").lower() == "n":
        return True
    if not stm.set_cal(gyro_x10000=new):
        return False
    profile["gyro_x10000"] = new
    profile.setdefault("checks", {})["gyro"] = {"ratios": [round(r, 5) for r in ratios],
                                                "from": cal["gyro_x10000"], "to": new}
    cal_profile.save(name, profile)
    return True


def step_verify(stm: STM, profile: dict, name: str, log: RunLog,
                straight_rpm: int, arc_rpm: int, side_cm: int = 50) -> bool:
    print("\n  ── VERIFY ──────────────────────────────────────────────────────")
    print(f"  A {side_cm} cm square, right turns, straights at {straight_rpm} "
          f"and arcs at {arc_rpm} rpm. Mark the start under the rear axle.")
    if not c9.position_and_zero(stm, "Robot on the start mark."):
        return False
    stops = []
    for _ in range(4):
        for tok in (token("F", side_cm, straight_rpm), token("FR", 90, arc_rpm)):
            mv = run_move(stm, tok, log, "verify", 1)
            if mv is None:
                return False
            if mv["move"] in ARC_MOVES:
                stops.append(abs(mv["turned"]) - abs(mv["aim"]))
    hdg = stm.read_heading()
    gap = c9.ask_float("Distance from the start mark, cm (Enter = skip):")
    worst = max(abs(s) for s in stops)
    print(f"\n    arc stops {', '.join(f'{s:+.2f}' for s in stops)}°  "
          f"(worst {worst:.2f}°)  heading error {hdg['error']:+.2f}°"
          + (f"  gap {gap:.1f} cm" if gap is not None else ""))
    profile.setdefault("checks", {})["verify"] = {
        "straight_rpm": straight_rpm, "arc_rpm": arc_rpm, "stops": stops,
        "heading_err": hdg["error"], "gap_cm": gap,
        "taken": datetime.now().isoformat(timespec="seconds")}
    if c9.ask("Mark this profile verified? [y/N]").lower() == "y":
        profile["verified"] = True
    cal_profile.save(name, profile)
    return True


# ── Commands ─────────────────────────────────────────────────────────────────

def show(profile: dict) -> None:
    tab = table(profile)
    print(f"\n  {profile.get('name')}  (taken {profile.get('taken', '?')}; "
          f"{'verified' if profile.get('verified') else 'not verified'})")
    print(f"  protocol 9 values: decel {profile['decel_x10'] / 10:.1f} dps², "
          f"lag {profile['lag_ms_x10'] / 10:.1f} ms, trim {profile['trim_us']:+d} µs, "
          f"gyro ×{profile.get('gyro_x10000', 10000) / 10000:.4f}, steer L "
          f"{profile.get('steer_left_us')} / R {profile.get('steer_right_us')} µs")
    g = tab.get("G") or {}
    print("  hardware: " + (", ".join(f"{k}={g[k]}" for k in CAL_TABLE_GLOBALS
                                     if g.get(k)) or "none (compiled constants)"))
    for move in CAL_TABLE_MOVES:
        for a in tab.get(move) or []:
            vals = ", ".join(f"{k}={v}" for k, v in a.items() if k != "rpm")
            print(f"  {move:<2} @ {a['rpm']:>3} rpm: {vals}")
    if not any(tab.get(m) for m in CAL_TABLE_MOVES):
        print("  no per-move values - every move drives on the protocol 9 values.")


def cmd_set(profile: dict, name: str, assigns: List[str]) -> bool:
    tab = table(profile)
    for item in assigns:
        key_text, _, val = item.partition("=")
        try:
            key = parse_key(key_text)
            if key[2] == "gyro":
                profile["gyro_x10000"] = int(val)
            else:
                set_value(tab, key[0], key[1], key[2], int(val))
        except ValueError as exc:
            logging.error(f"{item}: {exc}")
            return False
        logging.info(f"{key_name(key)} = {val}")
    try:
        cal_profile.table_entries(tab)
    except ValueError as exc:
        logging.error(f"Table would not restore: {exc}")
        return False
    cal_profile.save(name, profile)
    show(profile)
    return True


def cmd_run(stm: STM, log: RunLog, toks: List[str], repeat: int) -> bool:
    reports = []
    for i in range(repeat):
        for tok in toks:
            mv = run_move(stm, tok, log, "run", i + 1)
            if mv is None:
                return False
            reports.append(mv)
    if repeat > 1:
        for tok in toks:
            g = [mv for mv, t in zip(reports, toks * repeat) if t == tok]
            if g[0]["move"] in ARC_MOVES:
                s = [abs(mv["turned"]) - abs(mv["aim"]) for mv in g]
                print(f"    {tok}: stop {mean(s):+.2f}° sd {sd(s):.2f} over {len(g)}")
            else:
                c = [mv["corr"] for mv in g]
                print(f"    {tok}: steer {mean(c):+.1f} µs sd {sd(c):.1f}, drift "
                      f"{mean(mv['y'] for mv in g):+.1f} mm over {len(g)}")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    def common(p, steps=True):
        p.add_argument("name")
        p.add_argument("--from", dest="base", default=None,
                       help="start a new profile from this one")
        p.add_argument("--note", default=None)
        if steps:
            p.add_argument("--runs", type=int, default=RUNS)
            p.add_argument("--tape", action="store_true",
                           help="ask for tape measurements (distance / chord)")

    g = sub.add_parser("guided", help="straights, gyro, arcs, verify")
    common(g)
    g.add_argument("--straight-speeds", default=",".join(map(str, STRAIGHT_SPEEDS)))
    g.add_argument("--arc-speeds", default=",".join(map(str, ARC_SPEEDS)))
    g.add_argument("--target", default="keep",
                   help="arc radius: keep | match | <cm>")
    g.add_argument("--skip", default="", help="comma list of steps to skip")

    s = sub.add_parser("straights", help="wheels, motors, trim, coast, distance")
    common(s)
    s.add_argument("--speeds", default=",".join(map(str, STRAIGHT_SPEEDS)))
    s.add_argument("--cm", type=int, default=STRAIGHT_CM)

    a = sub.add_parser("arcs", help="braking and steering per arc and speed")
    common(a)
    a.add_argument("--speeds", default=",".join(map(str, ARC_SPEEDS)))
    a.add_argument("--moves", default="FR,FL,RR,RL")
    a.add_argument("--deg", type=int, default=ARC_DEG)
    a.add_argument("--target", default="keep", help="keep | match | <cm>")

    gy = sub.add_parser("gyro", help="gyro scale against an edge")
    common(gy, steps=False)

    v = sub.add_parser("verify", help="a square at your speeds")
    common(v, steps=False)
    v.add_argument("--speed", type=int, default=150, help="straight rpm")
    v.add_argument("--arc-speed", type=int, default=150)
    v.add_argument("--side", type=int, default=50)

    r = sub.add_parser("run", help="run moves and print their reports")
    common(r, steps=False)
    r.add_argument("tokens", nargs="+")
    r.add_argument("--repeat", type=int, default=1)
    r.add_argument("--no-restore", action="store_true",
                   help="drive on whatever the robot holds now")

    st = sub.add_parser("set", help="set values by hand: FR@150.steer=610 G.wheel=10030")
    st.add_argument("name")
    st.add_argument("assign", nargs="+")
    sub.add_parser("list", help="saved profiles")
    sh = sub.add_parser("show", help="print a profile's table")
    sh.add_argument("name")
    rs = sub.add_parser("restore", help="send a profile to the robot")
    rs.add_argument("name")

    args = ap.parse_args()
    if args.cmd is None:
        ap.print_help()
        return 1

    if args.cmd == "list":
        for n in cal_profile.list_profiles():
            print(n)
        return 0
    if args.cmd == "show":
        p = cal_profile.load(args.name)
        if p is None:
            return 1
        show(p)
        return 0
    if args.cmd == "set":
        p = cal_profile.load(args.name)
        if p is None:
            return 1
        return 0 if cmd_set(p, args.name, args.assign) else 1

    stm = STM()
    try:
        if args.cmd == "restore":
            p = cal_profile.load(args.name)
            if p is None or not connect(stm, p):
                return 1
            logging.info(f"{args.name!r} is on the robot.")
            return 0

        if args.cmd == "run" and args.no_restore:
            if not c9.connect_and_check(stm):
                return 1
            log = RunLog(args.name)
            return 0 if cmd_run(stm, log, args.tokens, args.repeat) else 1

        # Steps need a profile: NAME, or a new one from --from / the robot.
        if not os.path.exists(cal_profile.profile_path(args.name)) and not args.base:
            if not c9.connect_and_check(stm):
                return 1
            p = load_or_start(stm, args.name, None)
        else:
            p = load_or_start(None, args.name, args.base)
        if p is None:
            return 1
        if args.note is not None:
            p["note"] = args.note
        p.setdefault("table", {})
        p.setdefault("checks", {})
        p["taken"] = datetime.now().isoformat(timespec="seconds")
        if not connect(stm, p):
            return 1
        cal_profile.save(args.name, p)
        log = RunLog(args.name)
        logging.info(f"Every move is logged to {log.path}.")

        ok = True
        if args.cmd == "run":
            ok = cmd_run(stm, log, args.tokens, args.repeat)
        elif args.cmd == "straights":
            ok = step_straights(stm, p, args.name, log, parse_speeds(args.speeds, False),
                                args.runs, args.cm, args.tape)
        elif args.cmd == "arcs":
            moves = [m.strip().upper() for m in args.moves.split(",") if m.strip()]
            bad = [m for m in moves if m not in ARC_MOVES]
            if bad:
                logging.error(f"Unknown arc(s) {bad}; use {ARC_MOVES}.")
                return 1
            ok = step_arcs(stm, p, args.name, log, moves,
                           parse_speeds(args.speeds, True), args.runs, args.deg,
                           args.tape, args.target)
        elif args.cmd == "gyro":
            ok = step_gyro(stm, p, args.name)
        elif args.cmd == "verify":
            ok = step_verify(stm, p, args.name, log, args.speed, args.arc_speed,
                             args.side)
        elif args.cmd == "guided":
            skip = {x.strip().lower() for x in args.skip.split(",") if x.strip()}
            ss = parse_speeds(args.straight_speeds, False)
            as_ = parse_speeds(args.arc_speeds, True)
            if ok and "straights" not in skip:
                ok = step_straights(stm, p, args.name, log, ss, args.runs,
                                    STRAIGHT_CM, args.tape)
            if ok and "gyro" not in skip:
                ok = step_gyro(stm, p, args.name)
            if ok and "arcs" not in skip:
                ok = step_arcs(stm, p, args.name, log, list(ARC_MOVES), as_,
                               args.runs, ARC_DEG, args.tape, args.target)
            if ok and "verify" not in skip:
                ok = step_verify(stm, p, args.name, log, max(ss), max(as_))
        log.close()
        show(p)
        if ok:
            print(f"\n  Saved {cal_profile.profile_path(args.name)}. Tasks restore it "
                  f"with CAL_PROFILE={args.name} in .env.")
        return 0 if ok else 1
    except KeyboardInterrupt:
        stm.abort()
        print("\n  Aborted - robot stopped. Values approved so far are saved.")
        return 1
    finally:
        try:
            stm.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
