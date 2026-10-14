"""
communications/cal_profile.py — saved calibration, and getting a robot into it
──────────────────────────────────────────────────────────────────────────────
One place for three things every script needs to agree on:

  * where calibration profiles live and what is in them
  * prepare_for_task(): the fixed startup every task runs before it moves
  * HeadingLog: one line per move of commanded vs actual heading, so a run's
    turning precision can be read afterwards instead of guessed at

WHY TASKS REFUSE TO START WITHOUT A PROFILE
───────────────────────────────────────────
Protocol 4 firmware powers up with learning OFF and the compiled seeds in
place. Those seeds were measured on another day, on another floor, with the
compiled gyro scale — so a task that just started driving would run on numbers
nobody checked here, and nothing during the run would fix them. Failing loudly
at startup costs one command (`calibrate.py restore <name>` is not even
needed — the task restores it); failing quietly costs the run.

Set CAL_PROFILE in .env to the profile for the arena you are on. CAL_REQUIRED=0
lets a task run on the seeds anyway, for bench work only.
"""

import csv
import json
import logging
import os
import time
from datetime import datetime
from typing import Optional

from communications.stm import (FAST_MIN_PROTOCOL, FROZEN_CAL_MIN_PROTOCOL,
                                 STEER_CAL_MIN_PROTOCOL, STM)

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROFILE_DIR = os.path.join(_HERE, "cal_profiles")
HEADING_LOG_DIR = os.path.join(_HERE, "heading_logs")

# The arc profiles that are calibrated and driven; the firmware is locked to
# these two (MOTION_PROFILE_LOCK). SAFE is TIGHT's steering at 120 rpm instead
# of 150. A calibration only fits the profile it was taken on, so every saved
# calibration records its own - and everything saved before SAFE existed
# already says 0, TIGHT.
ARC_TIGHT = 0
ARC_SAFE = 3
ARC_NAMES = {ARC_TIGHT: "TIGHT", ARC_SAFE: "SAFE"}
ARC_BY_NAME = {v.lower(): k for k, v in ARC_NAMES.items()}

# Default when nothing names a calibration.
ARC_PROFILE = ARC_TIGHT

# What the last prepare_for_task() selected, for the task to report and plan
# against.
selected_arc = ARC_PROFILE

# A still robot whose gyro reads more than this is on a bad bias - measured at
# power-on while it was being handled - and every turn will be off by this
# much per second. Firmware with the re-baseline repairs it after 3 s still.
GYRO_REST_MAX_DPS = 1.0

# A profile saved before the gyro scale existed restores as 1.0.
DEFAULT_GYRO_X10000 = 10000

# A profile saved before per-side steering existed restores both sides to the
# firmware's symmetric TIGHT deflection: 575 asked, 574 after the left side's
# servo limit (motion.c arc_steer_us()).
DEFAULT_STEER_US = 574


# ── Store ─────────────────────────────────────────────────────────────────────

def profile_path(name: str) -> str:
    return os.path.join(PROFILE_DIR, f"{name}.json")


def list_profiles():
    if not os.path.isdir(PROFILE_DIR):
        return []
    return sorted(f[:-5] for f in os.listdir(PROFILE_DIR) if f.endswith(".json"))


def load(name: str) -> Optional[dict]:
    path = profile_path(name)
    if not os.path.exists(path):
        logging.error(f"No calibration profile named {name!r} in {PROFILE_DIR}. "
                      f"Saved profiles: {list_profiles() or 'none'}.")
        return None
    with open(path, encoding="utf-8") as fh:
        p = json.load(fh)
    p.setdefault("gyro_x10000", DEFAULT_GYRO_X10000)
    p.setdefault("steer_left_us", DEFAULT_STEER_US)
    p.setdefault("steer_right_us", DEFAULT_STEER_US)
    p.setdefault("arc_profile", ARC_TIGHT)
    if arc_of(p) not in ARC_NAMES:
        logging.error(f"Profile {name!r} was taken on arc profile "
                      f"{p['arc_profile']}, which is not driven any more "
                      f"(only {ARC_NAMES}).")
        return None
    return p


def arc_of(p: dict) -> int:
    """The arc profile a saved calibration was taken on."""
    return int(p.get("arc_profile", ARC_TIGHT))


def save(name: str, profile: dict) -> str:
    os.makedirs(PROFILE_DIR, exist_ok=True)
    profile = dict(profile)
    profile["name"] = name
    profile.setdefault("arc_profile", ARC_PROFILE)
    profile.setdefault("taken", datetime.now().isoformat(timespec="seconds"))
    path = profile_path(name)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(profile, fh, indent=2)
    return path


def values_of(p: dict) -> tuple:
    """The six numbers the firmware holds, in ?CAL order (learn left out)."""
    return (int(p["decel_x10"]), int(p["lag_ms_x10"]), int(p["trim_us"]),
            int(p.get("gyro_x10000", DEFAULT_GYRO_X10000)),
            int(p.get("steer_left_us", DEFAULT_STEER_US)),
            int(p.get("steer_right_us", DEFAULT_STEER_US)))


# ── Talking to the robot ──────────────────────────────────────────────────────

def firmware_protocol(stm: STM) -> Optional[int]:
    ver = stm.query("?VER")
    if ver is None:
        logging.error("No answer to ?VER — the link is dead, or the board is on "
                      "USB Port 1 (download only). See PROTOCOL.md §1.")
        return None
    fields = ver.split(",")
    try:
        return int(fields[2])
    except (IndexError, ValueError):
        logging.error(f"Could not read a protocol version out of {ver!r}.")
        return None


def push(stm: STM, p: dict, proto: Optional[int] = None) -> bool:
    """
    Send a profile's values and read them back. Robot must be idle.

    Per-side steering needs protocol 5. On a protocol 4 board it is skipped -
    unless the profile actually differs from the symmetric default, in which
    case restoring the rest would quietly drive turns the profile was not
    measured on, so it refuses instead.
    """
    if proto is None:
        proto = firmware_protocol(stm)
        if proto is None:
            return False

    decel, lag, trim, gyro, steer_l, steer_r = values_of(p)
    with_steer = proto >= STEER_CAL_MIN_PROTOCOL
    if not with_steer and (steer_l, steer_r) != (DEFAULT_STEER_US, DEFAULT_STEER_US):
        logging.error(
            f"This profile sets per-side steering (left {steer_l}, right "
            f"{steer_r} µs) but the firmware is protocol {proto}; it needs "
            f"{STEER_CAL_MIN_PROTOCOL}. Flash the current build.")
        return False

    if not stm.set_cal(decel_x10=decel, lag_ms_x10=lag, trim_us=trim,
                       gyro_x10000=gyro,
                       steer_left_us=steer_l if with_steer else None,
                       steer_right_us=steer_r if with_steer else None):
        logging.error("Calibration restore failed — see above.")
        return False
    back = stm.read_cal_full()
    if back is None:
        return False
    want = (decel, lag, trim, gyro)
    got = (back["decel_x10"], back["lag_ms_x10"], back["trim_us"],
           back["gyro_x10000"])
    if with_steer:
        want += (steer_l, steer_r)
        got += (back["steer_left_us"], back["steer_right_us"])
    if got != want:
        logging.error(f"Read-back mismatch: firmware holds {got}, "
                      f"expected {want}.")
        return False
    return True


def gyro_in_use(stm: STM, what: str = "") -> bool:
    """
    ?IMU: is heading still coming from the gyro?

    If the gyro ever goes quiet for 100 ms the firmware falls back to ENCODER
    heading until power-off - and with the rear wheels scrubbing through every
    arc, that is degrees out per turn. Nothing else on the wire says so; the
    robot keeps driving and answering OK.
    """
    imu = stm.read_imu()
    if imu is None:
        logging.error("?IMU did not answer — cannot confirm the gyro is in use.")
        return False
    if not imu["ready"]:
        logging.error(
            f"The gyro has dropped out ({imu['stalls']} stall(s)) and the robot "
            f"is on ENCODER heading until power-off{what}. Power-cycle with the "
            "robot still on the floor - it measures the gyro bias at boot.")
        return False
    if abs(imu["rate_dps"]) > GYRO_REST_MAX_DPS:
        logging.error(
            f"The gyro reads {imu['rate_dps']:+.1f}°/s with the robot still"
            f"{what}. Its bias is wrong - it was probably moved during the "
            "2.5 s after power-on. Leave it untouched for 5 s and retry; if it "
            "persists, power-cycle with the robot still on the floor.")
        return False
    return True


def zero_heading(stm: STM) -> bool:
    """
    !ZERO: make the robot's current pose heading 0, commanded and actual alike,
    so the carry-over has nothing left to correct. Do this whenever the robot
    has been moved by hand - the gyro saw that rotation, and without a zero the
    next move would steer it straight back.

    Refused while a move is running: !ZERO is answered immediately even
    mid-move, and zeroing the odometry under a turn in flight would end it on
    the wrong angle. Reads ?HDG back rather than trusting the OK.
    """
    busy = stm.is_busy()
    if busy is None:
        logging.error("?STAT did not answer — not zeroing blind.")
        return False
    if busy:
        logging.error("Robot is moving — not zeroing mid-move. Wait for it to stop.")
        return False

    if not stm.zero_odometry():
        logging.error("!ZERO refused.")
        return False

    h = stm.read_heading()
    if h is None:
        return False
    if abs(h["commanded"]) > 0.05 or abs(h["actual"]) > 0.2:
        logging.error(f"Heading did not zero (cmd {h['commanded']:+.1f}°, "
                      f"act {h['actual']:+.1f}°) — was the robot still moving?")
        return False
    logging.info("Heading zeroed: this pose is now heading 0.")
    return True


def prepare_for_task(stm: STM, name: Optional[str] = None,
                     required: Optional[bool] = None,
                     allowed_arcs: Optional[set] = None,
                     fast: bool = False) -> bool:
    """
    The fixed startup every task runs, in this order:

        1. protocol >= 4          otherwise none of the below exists
        2. !LEARN0                nothing changes the calibration mid-run
        3. !PROF<n>               the arc profile the calibration was taken
                                  on - TIGHT or SAFE; TIGHT if none is named
        4. restore the profile    and read it back
        5. !ZERO                  this pose is heading 0 for the carry-over
        6. ?IMU                   heading is coming from the gyro, not the
                                  encoder fallback

    allowed_arcs: the arc profiles this task can drive. A task whose geometry
    is TIGHT's radius refuses a SAFE calibration instead of driving it.

    fast: Task 2's fast mode (!FAST1). Every other task passes False, which
    sends !FAST0 - so a Task 2 run earlier this power-on cannot leave the
    robot in fast mode for the next task.
    The profile selected is left in `selected_arc`.

    Must run before any movement line is outstanding: every step replies a
    plain OK on the movement path.
    """
    global selected_arc
    name = name if name is not None else os.getenv("CAL_PROFILE", "").strip()
    if required is None:
        required = os.getenv("CAL_REQUIRED", "1").strip() != "0"

    proto = firmware_protocol(stm)
    if proto is None:
        return False
    if proto < FROZEN_CAL_MIN_PROTOCOL:
        logging.error(
            f"Firmware is protocol {proto}; frozen calibration and heading "
            f"carry-over need {FROZEN_CAL_MIN_PROTOCOL}. Flash the current "
            "build — on this one the robot would keep learning through the run."
        )
        return False

    p = None
    if name:
        p = load(name)
        if p is None:
            return False
    arc = arc_of(p) if p is not None else ARC_PROFILE
    if allowed_arcs is not None and arc not in allowed_arcs:
        logging.error(
            f"Calibration {name!r} was taken on {ARC_NAMES[arc]}, but this task "
            f"only drives {', '.join(ARC_NAMES[a] for a in sorted(allowed_arcs))}.")
        return False

    if not stm.set_learning(False):
        logging.error("!LEARN0 refused — cannot guarantee a frozen calibration.")
        return False

    if proto >= FAST_MIN_PROTOCOL:
        if not stm.set_fast(fast):
            logging.error(f"!FAST{int(fast)} refused.")
            return False
    elif fast:
        logging.warning(f"Firmware is protocol {proto}: no fast mode (needs "
                        f"{FAST_MIN_PROTOCOL}). Running at normal speed.")

    if not stm.set_profile(arc):
        logging.error(f"!PROF{arc} refused — the robot is not on {ARC_NAMES[arc]}."
                      + (" Flash firmware that has the SAFE profile."
                         if arc == ARC_SAFE else ""))
        return False
    selected_arc = arc

    if p is not None:
        if not p.get("verified", False):
            logging.warning(
                f"Profile {name!r} never passed the square check "
                "(calibrate.py verify). Running on it anyway.")
        if not push(stm, p, proto):
            return False
        logging.info(
            f"Calibration {name!r} ({ARC_NAMES[arc]}) restored and verified: "
            f"decel {p['decel_x10']/10:.1f} dps², lag {p['lag_ms_x10']/10:.1f} ms, "
            f"trim {p['trim_us']:+d} µs, gyro ×{values_of(p)[3]/10000:.4f}, "
            f"steer L {values_of(p)[4]} / R {values_of(p)[5]} µs "
            f"(taken {p.get('taken', '?')}; {p.get('note', '')})")
    elif required:
        logging.error(
            "No calibration profile selected. Set CAL_PROFILE=<name> in .env "
            f"(saved: {list_profiles() or 'none — run calibrate.py guided'}), "
            "or CAL_REQUIRED=0 to run on the compiled seeds for bench work.")
        return False
    else:
        logging.warning("CAL_REQUIRED=0 — running on the compiled seeds, "
                        "uncalibrated. Bench use only.")

    # Last, immediately before the task's first move: wherever the robot was
    # placed by hand is heading 0 for the whole run.
    if not zero_heading(stm):
        return False

    back = stm.read_cal_full()
    if back is None or back.get("learn") != 0:
        logging.error(f"Learning is not confirmed off (CAL reply {back}).")
        return False

    # Every turn ends on the gyro's angle. A run started on the encoder
    # fallback is degrees out per turn from the first move.
    return gyro_in_use(stm, " - not starting")


# ── Per-move heading log ─────────────────────────────────────────────────────

class HeadingLog:
    """
    After each move, one CSV row of ?HDG: what was commanded, what the gyro
    says happened, and how precisely the last arc stopped. The file is the
    record of a run's turning precision - read it before touching calibration.
    """

    FIELDS = ["time", "label", "commanded", "actual", "error",
              "last_aim", "last_turned", "last_stop_err", "clamped", "gyro"]

    def __init__(self, stm: STM, task: str):
        self.stm = stm
        os.makedirs(HEADING_LOG_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(HEADING_LOG_DIR, f"{task}_{stamp}.csv")
        self._fh = open(self.path, "w", newline="", encoding="utf-8")
        self._csv = csv.DictWriter(self._fh, fieldnames=self.FIELDS)
        self._csv.writeheader()
        self._clamped = 0
        # Set once the gyro is seen gone; the fallback lasts until power-off,
        # so there is no coming back from it within a run.
        self.gyro_lost = False
        logging.info(f"Heading log: {self.path}")

    def _check_gyro(self, label: str) -> str:
        """'ok', 'LOST' or '?' for the CSV; logs the drop-out once."""
        imu = self.stm.read_imu()
        if imu is None:
            return "?"
        if imu["ready"]:
            return "ok"
        if not self.gyro_lost:
            self.gyro_lost = True
            logging.error(
                f"GYRO LOST by {label}: the robot is on ENCODER heading for the "
                "rest of this power-on, so every turn from here can be degrees "
                "out and the heading error below is no longer measured by the "
                "gyro. Power-cycle with the robot still before the next run.")
        return "LOST"

    def record(self, label: str) -> Optional[dict]:
        h = self.stm.read_heading()
        if h is None:
            return None
        row = {"time": f"{time.time():.2f}", "label": label}
        row.update({k: (f"{v:+.1f}" if isinstance(v, float) else v)
                    for k, v in h.items()})
        row["gyro"] = self._check_gyro(label)
        self._csv.writerow(row)
        self._fh.flush()
        logging.info(
            f"HDG after {label}: error {h['error']:+.1f}° "
            f"(cmd {h['commanded']:+.1f}, act {h['actual']:+.1f}); "
            f"last arc stopped {h['last_stop_err']:+.1f}° from its aim")
        if h["clamped"] > self._clamped:
            logging.warning(
                f"Heading error exceeded the 5° carry cap after {label} — the "
                "robot was bumped, slipped, or a move was cut short.")
            self._clamped = h["clamped"]
        return h

    def close(self) -> None:
        try:
            self._fh.close()
        except OSError:
            pass
