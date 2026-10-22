"""
calibrate.py  —  Calibrate the robot on the floor, check it by eye, save it
────────────────────────────────────────────────────────────────────────────
Protocol 4 firmware does NOT learn during a task. Learning is off at power-on
and every task restores a saved profile and freezes it (see
communications/cal_profile.py). So the calibration a task drives on is exactly
what this script measured — which makes this the one place precision is won.

    python3 calibrate.py guided arena --note "arena floor, full battery"
    python3 calibrate.py verify arena        # square test before a run

  One step at a time, each repeatable, each saved into the profile without
  touching the others (a missing profile starts from the robot's values):

    python3 calibrate.py straight arena      # step 1: trim
    python3 calibrate.py gyro arena          # step 2: gyro scale
    python3 calibrate.py radius arena        # step 3: match left/right to 29.1 cm (180° arcs)
    python3 calibrate.py radius arena --target auto   # ...or find the radius
    python3 calibrate.py radius arena_safe --fast      # SAFE: tape each turn once
    python3 calibrate.py brakes arena        # step 4: braking (also: new battery)
    python3 calibrate.py restore arena
    python3 calibrate.py list
    python3 calibrate.py show arena
    python3 calibrate.py zero                # heading 0 here, after moving it by hand
    python3 calibrate.py drift               # is the gyro drifting at rest?
    python3 calibrate.py reverse             # are the reverse signs right?
    python3 calibrate.py encoders            # do both wheels count 1560 a turn?
    python3 calibrate.py irsides             # is the right IR on the right?
    python3 calibrate.py encoders --reverse  # ...turning backwards

  Two arc profiles are driven, each with its own calibration: TIGHT (the
  default) and SAFE (TIGHT's steering at 120 rpm). A saved calibration
  remembers which it was taken on, and restore / verify / tasks select it:

    python3 calibrate.py guided arena_safe --arc safe   # radius found, not assumed
    python3 calibrate.py radius arena_safe              # steps keep the saved profile's arc

Then set CAL_PROFILE=arena in .env; task1.py and task_a5.py restore it.

THE GUIDED PROCEDURE, AND WHY EACH STEP NEEDS YOUR EYES
───────────────────────────────────────────────────────
  1. STRAIGHT   F100 along a tape line, learning the steering trim, until the
                trim stops moving AND you measure under 1 cm of drift. Also
                reports the distance scale against your tape.
  2. GYRO       Four FR90 (a full circle) then four FL90, starting flush with a
                reference edge. You measure how far past or short of the edge
                it ended. The gyro cannot see its own scale error - it measures
                its own turns - so this is the only step that can find it.
  3. RADIUS    FR180 then RR180 back along the same circle, then FL180/RL180,
                from marks under the rear axle; you measure how far the axle
                moved. Over a half circle that is the DIAMETER, so tape error
                is halved against a 90° chord and one run per direction is
                enough. Needs about 70 cm x 45 cm per side. The planner uses
                reverse arcs too.
                The left and right steering are adjusted until both match the
                29.1 cm the planner and the A5 orbit assume. The same pulse
                either side of centre does not give the same wheel angle, so
                without this a left turn and a right turn land the robot in
                different places even when both stop at exactly 90 degrees.
  4. BRAKES     Four arcs (FR, RR, FL, RL) with learning on; each one's RAW
                decel measurement is averaged, pushed back and frozen, with
                lag fixed at 0. Then four more with learning off to show how
                precisely each one stops.
  5. SQUARE     F50 + FR90 four times with learning off. You measure how far
                from the start mark it ends and its final angle. Only a profile
                that passes this is marked verified.

Order matters: the braking model is in gyro degrees, so it has to be learned
AFTER the gyro scale is fixed - and after the radius, because a different
steering angle turns at a different rate and coasts a different amount.

ALWAYS RE-ZERO AFTER YOU TOUCH THE ROBOT
────────────────────────────────────────
The firmware now carries heading error from one move into the next. If you
pick the robot up and straighten it by hand, the gyro sees that rotation and
the next move would "correct" it straight back. Every prompt that asks you to
position the robot sends !ZERO afterwards for exactly this reason. Tasks do
the same at startup.
"""

import argparse
import logging
import math
import os
import sys
import time
from datetime import datetime
from typing import List, Optional

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                    datefmt="%H:%M:%S")

from communications import cal_profile
from communications.stm import (CAL_LIMITS, ENC_MIN_PROTOCOL, FAST_MIN_PROTOCOL,
                                 FROZEN_CAL_MIN_PROTOCOL, RAW_DECEL_MIN_PROTOCOL,
                                 STEER_CAL_MIN_PROTOCOL, STM, WPOSE_MIN_PROTOCOL)

# Step 1. F<n> is centimetres.
STRAIGHT_CM = 100
STRAIGHT_MAX_RUNS = 2
TRIM_SETTLED_US = 2          # trim moving less than this per run = converged
DRIFT_OK_CM = 1.0            # sideways error at 1 m you are prepared to accept
DISTANCE_OK_PCT = 1.0
# One run going this much further than the others' median is not noise: the
# speed loop holds the two wheels' COUNT rates equal, so an encoder that drops
# counts makes its wheel turn faster - the robot covers extra ground and the
# trim soaks up the pull. Nothing on board can see it; the tape can.
DISTANCE_OUTLIER_CM = 2.0

# Step 2. Four quarter turns make a full circle and bring the robot back to
# where it started, so the reference edge is right there to measure against.
GYRO_LOOPS = [("FR90", 4), ("FL90", 4)]
GYRO_ASYMMETRY_WARN_PCT = 0.5

# Step 3. The radius both sides are matched to: what the planner
# (pc_side/stm_tokens.py TURN_RADIUS_MM) and A5 (TIGHT_RADIUS_CM) assume.
RADIUS_TARGET_CM = 29.1
RADIUS_TOL_CM = 0.5
# A half circle: the chord is the diameter, so a tape error of e cm is e/2 cm
# of radius (e/1.41 over a quarter circle), and the start and stop transients
# are a smaller share of the arc. One pair per side per round is then as good
# as two quarter-circle pairs were.
RADIUS_ARC_DEG = 180
RADIUS_RUNS = 1              # forward+reverse pairs measured per side per round
# Tape chord against the odometry chord (?WPOSE). The odometry runs long on an
# arc - the rear wheels scrub - so it is a sanity check for a misread tape or
# a slipping run, never a value. Warn past the larger of these.
WPOSE_CHORD_WARN_CM = 3.0
WPOSE_CHORD_WARN_PCT = 10.0
# radius --fast: each kind of turn (FR, RR, FL, RL) is taped once and the
# odometry checked against it; after that the robot measures its own chord.
# An arc that stopped further than this from its aim is taped regardless -
# a wheel surge throws the odometry off and early braking is its signature.
FAST_STOP_ERR_DEG = 2.0
# Tape and odometry further apart than this on the check arc and that kind
# stays on the tape. The good arcs on SAFE agreed to about 1%.
FAST_MAX_CORRECTION = 0.03
RADIUS_MAX_ROUNDS = 4
RADIUS_MAX_STEP_US = 60      # never move one side's steering further per round
# Forward and reverse share one steering pulse per side, so steering cannot
# pull them together. Past this they are worth knowing about: the planner
# uses one radius for both, and a retrace home lands off by the difference.
RADIUS_FWD_REV_WARN_CM = 1.0

# Step 4. Alternating forward/reverse so the heading returns and the robot
# stays on one patch of floor. Needs about 1.5 m x 1.5 m clear.
# Protocol 6 reports each arc's own measurement, and with lag fixed one arc is
# a complete, self-consistent answer - so one per direction is enough. The
# filtered value older firmware reports carries the prior for several arcs,
# so it keeps the longer sequence.
BRAKE_LEARN_SEQ = ["FR90", "RR90", "FL90", "RL90"]
BRAKE_LEARN_SEQ_OLD = BRAKE_LEARN_SEQ * 2
BRAKE_CHECK_SEQ = ["FR90", "RR90", "FL90", "RL90"]
STOP_ERR_OK_DEG = 0.5

# Step 5.
SQUARE_SIDE_CM = 50
SQUARE_GAP_OK_CM = 3.0
SQUARE_ANGLE_OK_DEG = 1.0
SQUARE_HDG_OK_DEG = 0.5


# ── Small helpers ─────────────────────────────────────────────────────────────

def _mean(vals) -> float:
    return sum(vals) / float(len(vals))


def _stdev(vals) -> float:
    if len(vals) < 2:
        return 0.0
    m = _mean(vals)
    return (sum((v - m) ** 2 for v in vals) / (len(vals) - 1)) ** 0.5


def ask(prompt: str) -> str:
    return input(f"  {prompt} ").strip()


def ask_float(prompt: str, allow_blank: bool = True) -> Optional[float]:
    while True:
        raw = ask(prompt)
        if raw == "" and allow_blank:
            return None
        try:
            return float(raw)
        except ValueError:
            print("    A number, please (or Enter to skip).")


def ask_angle(prompt: str) -> float:
    """
    Degrees, or an offset measured along an edge: '12/200' means the robot's
    side is 12 mm further out at one end than the other over 200 mm, which is
    atan(12/200) = 3.4 degrees. Easier to measure well than a protractor.
    Sign is the caller's convention; for offsets put the sign on the first
    number.
    """
    while True:
        raw = ask(prompt)
        try:
            if "/" in raw:
                off, span = (float(x) for x in raw.split("/", 1))
                return math.degrees(math.atan2(off, span))
            return float(raw)
        except ValueError:
            print("    Degrees (e.g. -1.5) or offset/span in mm (e.g. -12/200).")


def pause(msg: str) -> None:
    input(f"\n  {msg}\n  Enter when ready, Ctrl-C to abort... ")


# Set by connect_and_check() when the firmware has ?ENC: every arc then logs
# how far each wheel turned.
_wheel_counts = False

# One wheel turning less than this share of the other in an arc is not the
# inner/outer split (SAFE ~0.58, TIGHT ~0.39) - that wheel stalled or lost drive.
WHEEL_STALL_RATIO = 0.25


def log_wheels(token: str, before, after) -> Optional[str]:
    """
    One line of per-wheel counts for an arc. Returns the stalled wheel's name
    if one barely turned while the other drove, else None.
    """
    da, db = after[0] - before[0], after[1] - before[1]
    big = max(abs(da), abs(db))
    logging.info(f"    wheels: A (left) {da:+d}, B (right) {db:+d} counts")
    if big < 200:                       # did not really drive at all
        return None
    for name, mine in (("A (LEFT)", da), ("B (RIGHT)", db)):
        if abs(mine) < WHEEL_STALL_RATIO * big:
            logging.error(
                f"{token}: wheel {name} turned {abs(mine)} counts while the "
                f"other turned {big}. It stalled or lost drive mid-arc and the "
                "robot pivoted round it. The encoders count fine by hand, so "
                "check that motor's power: its connector, its wires, the motor "
                "driver output, and the battery connector.")
            return name
    return None


def send_move(stm: STM, token: str) -> bool:
    """One movement line, wait for its reply. Anything but OK stops the run."""
    arc = _wheel_counts and token[:2].upper() in ("FR", "FL", "RR", "RL")
    before = stm.read_enc() if arc else None
    if not stm.send_line([token]):
        return False
    reply = stm.wait_reply()
    if reply is None:
        logging.error(f"{token}: no reply — lost link.")
        return False
    if before is not None:
        after = stm.read_enc()
        if after is not None:
            log_wheels(token, before, after)
    if reply.strip().upper() == "OK":
        return True
    logging.error(f"{token}: {reply.strip()}. The robot is not where you think "
                  "it is — stopping. Check for a stalled wheel before trusting "
                  "anything measured after this.")
    return False


def position_and_zero(stm: STM, msg: str) -> bool:
    """Ask for the robot to be placed, then make that pose heading 0."""
    pause(msg)
    return cal_profile.zero_heading(stm)


def gyro_ok(stm: STM) -> bool:
    """
    Is the heading still coming from the gyro? If the gyro ever went quiet
    for 100 ms the firmware switched to encoder heading until power-off, and
    every angle measured after that is wrong by degrees - silently.
    """
    return cal_profile.gyro_in_use(
        stm, "; nothing measured since is valid - redo this step after the "
             "power-cycle")


def connect_and_check(stm: STM) -> bool:
    try:
        stm.connect()
    except Exception as exc:
        logging.error(f"Could not open the serial port: {exc}. Check SERIAL_PORT "
                      "in .env; USB Port 2 is the RPi link.")
        return False
    proto = cal_profile.firmware_protocol(stm)
    if proto is None:
        return False
    if proto < FROZEN_CAL_MIN_PROTOCOL:
        logging.error(f"Firmware is protocol {proto}; this tool needs "
                      f"{FROZEN_CAL_MIN_PROTOCOL}. Flash the current build.")
        return False
    if not stm.set_learning(False) or not stm.set_profile(cal_profile.ARC_PROFILE):
        logging.error("Could not put the robot in a known state (!LEARN0, !PROF0).")
        return False
    # Calibrate at normal speed: a Task 2 run may have left fast mode on.
    if proto >= FAST_MIN_PROTOCOL and not stm.set_fast(False):
        logging.error("Could not switch fast mode off (!FAST0).")
        return False
    global _wheel_counts
    _wheel_counts = proto >= ENC_MIN_PROTOCOL
    return gyro_ok(stm)


def select_arc(stm: STM, arc: int) -> bool:
    """!PROF for the arc profile a calibration belongs to. Robot must be idle."""
    if stm.set_profile(arc):
        logging.info(f"Arc profile {arc} ({cal_profile.ARC_NAMES[arc]}).")
        return True
    logging.error(f"!PROF{arc} refused — cannot select {cal_profile.ARC_NAMES[arc]}."
                  + (" Flash firmware that has the SAFE profile."
                     if arc == cal_profile.ARC_SAFE else ""))
    return False


def arc_from_args(args) -> Optional[int]:
    """--arc tight|safe as a profile number, None if not given."""
    name = getattr(args, "arc", None)
    return None if name is None else cal_profile.ARC_BY_NAME[name]


def default_target(arc: int) -> Optional[float]:
    """
    TIGHT is matched to the radius the planner already uses. SAFE's radius is
    not known yet - slower turns tighter - so it is found, not assumed:
    matching SAFE to 29.1 would widen the steering and throw the gain away.
    """
    return RADIUS_TARGET_CM if arc == cal_profile.ARC_TIGHT else None


def current_values(stm: STM) -> Optional[dict]:
    back = stm.read_cal_full()
    if back is None:
        logging.error("?CAL did not answer.")
    return back


def show_values(label: str, v: dict) -> None:
    logging.info(f"{label}: decel {v['decel_x10']/10:.1f} dps²  "
                 f"lag {v['lag_ms_x10']/10:.1f} ms  trim {v['trim_us']:+d} µs  "
                 f"gyro ×{v['gyro_x10000']/10000:.4f}")


# ── Step 1: straightness ─────────────────────────────────────────────────────

def step_straight(stm: STM, result: dict) -> bool:
    print("\n  ── STEP 1: STRAIGHT LINE ──────────────────────────────────────")
    print(f"  Lay a straight tape line of at least {STRAIGHT_CM + 30} cm. Put the")
    print("  robot on it, centred and pointing along it, and mark the rear axle.")
    print(f"  Each run drives F{STRAIGHT_CM} with trim learning on, then reverses")
    print("  back with it off. At the far end, measure the sideways offset from the line of")
    print("  one point on the robot's centreline - the front centre is fine, the")
    print("  same point every run - and how far it actually went.")

    if not stm.set_learning(True):
        return False

    drifts: List[float] = []
    distances: List[float] = []
    prev_trim = current_values(stm)["trim_us"]
    converged = False

    try:
        for run in range(1, STRAIGHT_MAX_RUNS + 1):
            if not position_and_zero(stm, f"Run {run}: robot on the line at the start mark."):
                return False
            if not send_move(stm, f"F{STRAIGHT_CM}"):
                return False

            trim = current_values(stm)["trim_us"]
            drift = ask_float("Sideways offset from the line at the end, cm "
                              "(+ left, - right, Enter to skip):")
            dist = ask_float(f"Distance actually travelled, cm (Enter to skip):")
            if drift is not None:
                drifts.append(drift)
            if dist is not None:
                distances.append(dist)

            moved = trim - prev_trim
            logging.info(f"  run {run}: trim {trim:+d} µs ({moved:+d})"
                         + (f", drift {drift:+.1f} cm" if drift is not None else ""))
            prev_trim = trim

            # The return trip does NOT teach the trim. Reversing loads the
            # linkage slack the other way and the front wheels trail, so the
            # centre that holds a reverse line is not the forward one - and it
            # starts with the forward run's leftover heading to steer out on
            # top. Learning on both, each run pulled the trim one way going
            # out and the other way coming back, and it never settled.
            if not stm.set_learning(False):
                return False
            if not send_move(stm, f"R{STRAIGHT_CM}"):
                return False
            if not stm.set_learning(True):
                return False

            drift_ok = drift is None or abs(drift) <= DRIFT_OK_CM
            if run >= 2 and abs(moved) <= TRIM_SETTLED_US and drift_ok:
                converged = True
                break
    finally:
        stm.set_learning(False)

    if not converged:
        logging.warning("Trim did not settle, or drift stayed over "
                        f"{DRIFT_OK_CM} cm, within {STRAIGHT_MAX_RUNS} runs.")
    if drifts and abs(drifts[-1]) > DRIFT_OK_CM and converged is False:
        logging.warning(
            "If the trim HAS stopped moving but the drift has not, the trim is "
            "not the cause: the heading loop holds the gyro heading, so drift "
            "means the robot launched crooked or the gyro is drifting. Check "
            "the start alignment and ?IMU before going further.")

    why = ("The robot stops on encoder counts, so an occasional long run is "
           "an encoder losing counts - check both encoder connectors, then run "
           "`calibrate.py encoders`. A trim learned through it is suspect.")
    if len(distances) >= 3:
        med = sorted(distances)[len(distances) // 2]
        odd = [(i + 1, d) for i, d in enumerate(distances)
               if abs(d - med) > DISTANCE_OUTLIER_CM]
        if odd:
            logging.warning(
                "Run(s) " + ", ".join(f"{i} ({d:.1f} cm)" for i, d in odd)
                + f" went more than {DISTANCE_OUTLIER_CM} cm off the others' "
                f"{med:.1f} cm. " + why)
    elif len(distances) == 2 and abs(distances[0] - distances[1]) > DISTANCE_OUTLIER_CM:
        # Two runs cannot say which one is odd, only that they disagree.
        logging.warning(
            f"The two runs went {distances[0]:.1f} and {distances[1]:.1f} cm - "
            f"more than {DISTANCE_OUTLIER_CM} cm apart. " + why)

    dist_note = None
    if distances:
        err_pct = (_mean(distances) - STRAIGHT_CM) / STRAIGHT_CM * 100.0
        dist_note = round(err_pct, 2)
        logging.info(f"  distance: mean {_mean(distances):.1f} cm for {STRAIGHT_CM} "
                     f"commanded ({err_pct:+.1f}%)")
        if abs(err_pct) > DISTANCE_OK_PCT:
            logging.warning(
                f"Distance is {err_pct:+.1f}% off. That is WHEEL_DIAMETER_MM in "
                "stm/PeripheralDrivers/Inc/odom.h (compile time), not something "
                f"calibration can fix. Multiply it by {STRAIGHT_CM/_mean(distances):.4f} "
                "if the tape is right.")

    result["checks"]["straight"] = {
        "converged": converged,
        "drift_cm": drifts,
        "distance_err_pct": dist_note,
        "trim_us": prev_trim,
    }
    return True


# ── Step 2: gyro scale ───────────────────────────────────────────────────────

def step_gyro(stm: STM, result: dict) -> bool:
    print("\n  ── STEP 2: GYRO SCALE ─────────────────────────────────────────")
    print("  Put the robot's side flush against a straight reference edge (a")
    print("  tape line or a ruler taped down). Four quarter turns make a full")
    print("  circle and bring it back beside the edge. Clear about 1 m x 1 m.")
    print("  Then measure how far it is rotated from the edge:")
    print("    + if it turned PAST the edge (too far), - if it stopped SHORT.")
    print("  Degrees, or offset/span in mm along its side, e.g. +6/200.")

    cal = current_values(stm)
    scale = cal["gyro_x10000"] / 10000.0
    ratios = []

    for token, count in GYRO_LOOPS:
        if not position_and_zero(stm, f"Robot flush with the edge for {count} x {token}."):
            return False
        for _ in range(count):
            if not send_move(stm, token):
                return False
        hdg = stm.read_heading()
        if hdg is None:
            return False
        gyro_deg = abs(hdg["actual"])
        seen = ask_angle(f"{token} x{count}: angle from the edge, degrees "
                         "(+ past, - short):")
        real_deg = 360.0 * count / 4.0 + seen
        ratio = real_deg / gyro_deg
        ratios.append(ratio)
        logging.info(f"  {token} x{count}: gyro {gyro_deg:.1f}°, floor "
                     f"{real_deg:.1f}° → gyro reads ×{1/ratio:.4f} of the truth")

    ratio = _mean(ratios)
    asym = abs(ratios[0] - ratios[1]) / ratio * 100.0 if len(ratios) > 1 else 0.0
    if asym > GYRO_ASYMMETRY_WARN_PCT:
        logging.warning(
            f"Right and left loops disagree by {asym:.2f}%. A gyro scale error "
            "is the same both ways, so a difference means something else - "
            "one direction slipping, or the measurement. Re-measure before "
            "trusting this.")

    new_scale = scale * ratio
    new_x10000 = int(round(new_scale * 10000))
    logging.info(f"  gyro scale ×{scale:.4f} → ×{new_scale:.4f}")
    if not stm.set_cal(gyro_x10000=new_x10000):
        logging.error("!CALG refused — scale outside 0.90..1.10? That is not a "
                      "scale error; re-measure.")
        return False

    result["checks"]["gyro"] = {"ratios": [round(r, 5) for r in ratios],
                                "asymmetry_pct": round(asym, 3),
                                "from": scale, "to": round(new_scale, 5)}
    return True


# ── Step 3: radius, per side ─────────────────────────────────────────────────

def _chord_radius(stm: STM, token: str, prompt: str,
                  wpose: bool = False, fast: Optional[dict] = None) -> Optional[float]:
    """
    Drive one arc and turn the measured chord into a radius. For a turn of
    theta the chord is 2 R sin(theta/2), so R = chord / (2 sin(theta/2)) -
    using the angle the gyro says it actually turned, not the angle asked
    for. Near 180 the sine is flat, so a degree of stop error costs nothing.

    fast: the --fast odometry map, {turn kind: tape/odometry factor}, or None
    for tape every arc. A kind not in it yet is taped once and its factor
    learned (see learn_odometry); after that its chord comes from ?WPOSE
    times the factor - unless the arc looks wrong, when it is taped again.
    """
    kind = token[:2]
    before = stm.read_wpose() if (wpose or fast is not None) else None
    if not send_move(stm, token):
        return None
    hdg = stm.read_heading()
    if hdg is None:
        return None
    after = stm.read_wpose() if before is not None else None
    turned = abs(hdg["last_turned"])
    stop_err = abs(hdg["last_turned"] - hdg["last_aim"])
    odo = None
    if after is not None:
        odo = math.hypot(after["x_mm"] - before["x_mm"],
                         after["y_mm"] - before["y_mm"]) / 10.0

    k = fast.get(kind) if fast is not None else None
    if k and odo is not None and stop_err <= FAST_STOP_ERR_DEG:
        chord = odo * k
        r = chord / (2.0 * math.sin(math.radians(turned) / 2.0))
        logging.info(f"  {token}: turned {turned:.1f}°, chord {chord:.1f} cm "
                     f"from odometry (×{k:.3f}) → radius {r:.1f} cm")
        return r

    if k:
        # The robot was to measure this one, so nobody marked the floor -
        # and this arc cannot be trusted to the odometry. Nothing to fall
        # back on: stop rather than feed a bad chord into the steering.
        why = ("no odometry reading" if odo is None else
               f"it stopped {stop_err:.1f}° from its aim, so a wheel likely "
               "slipped or surged")
        logging.error(f"{token}: {why}. Not using this arc - re-run the step.")
        return None

    chord = ask_float(prompt, allow_blank=False)
    r = chord / (2.0 * math.sin(math.radians(turned) / 2.0))
    logging.info(f"  {token}: turned {turned:.1f}°, chord {chord:.1f} cm "
                 f"→ radius {r:.1f} cm")
    if after is not None:
        check_chord(token, chord, before, after)
        if fast is not None and kind not in fast:
            fast[kind] = learn_odometry(kind, chord, odo, stop_err)
    return r


def learn_odometry(kind: str, tape_cm: float, odo_cm: float,
                   stop_err: float) -> Optional[float]:
    """
    The tape/odometry factor for one kind of turn, from its first taped arc.
    None = the odometry is not trusted for this kind; it stays on the tape.

    Per kind, because the odometry's bias is not the same each way: forward
    and reverse, left and right each scrub differently (63.0 tape against
    62.5 odometry on one FR180, 64.0 against 64.8 on the RR180 after it).
    """
    if stop_err > FAST_STOP_ERR_DEG or odo_cm <= 1.0:
        logging.warning(f"{kind}: this arc was not clean enough to check the "
                        "odometry against - it stays on the tape.")
        return None
    k = tape_cm / odo_cm
    if abs(k - 1.0) > FAST_MAX_CORRECTION:
        logging.warning(
            f"{kind}: tape and odometry differ by {(k - 1.0) * 100:+.1f}% - too "
            "much to trust it. This kind stays on the tape; if every kind does "
            "this, the encoders or the wheel size need looking at.")
        return None
    logging.info(f"  {kind}: odometry checked against the tape (×{k:.3f}) - "
                 "the rest of these are measured by the robot.")
    return k


def check_chord(token: str, tape_cm: float, before: dict, after: dict) -> float:
    """
    Compare the tape chord with the odometry's. Warns, changes nothing.
    Returns the odometry chord in cm.
    """
    odo = math.hypot(after["x_mm"] - before["x_mm"],
                     after["y_mm"] - before["y_mm"]) / 10.0
    gap = tape_cm - odo
    logging.info(f"    odometry chord {odo:.1f} cm (tape {gap:+.1f} cm from it)")
    if abs(gap) > max(WPOSE_CHORD_WARN_CM, tape_cm * WPOSE_CHORD_WARN_PCT / 100.0):
        logging.warning(
            f"{token}: the tape says {tape_cm:.1f} cm, the odometry {odo:.1f} cm. "
            "Re-check the tape reading - mark under the axle CENTRE, straight "
            "across. If it is right, a wheel slipped on this arc: re-run the "
            "round rather than let it steer the result.")
    return odo


def measure_side(stm: STM, side: str, runs: int,
                 wpose: bool = False, fast: Optional[dict] = None) -> Optional[dict]:
    """
    One side's radius, forward AND reverse, from the floor. Each pair is a
    forward arc out (FR180) and the reverse arc on the same steering back
    (RR180). Reverse with the same wheel angle runs back along the same
    circle, so the robot only ever uses one half circle of floor and ends
    near where it started - no repositioning between pairs.

    Two marks per pair: A under the rear axle before, B after the forward
    arc. Forward radius from A->axle, reverse radius from B->axle. The gap
    to A at the end is the retrace error, optional.
    """
    fwd, rev = f"F{side}{RADIUS_ARC_DEG}", f"R{side}{RADIUS_ARC_DEG}"
    home = []
    out = {"fwd": [], "rev": [], "home": home}
    way = "right" if side == "R" else "left"
    if not position_and_zero(stm, f"{fwd}/{rev}: robot with about 45 cm clear "
                             f"ahead and 70 cm to the {way}."):
        return None
    def robot_measures(token: str) -> bool:
        return fast is not None and bool(fast.get(token[:2]))

    for i in range(1, runs + 1):
        # No mark needed when the robot measures the chord itself. If it then
        # asks for the tape after all, the arc went wrong anyway - re-run.
        if robot_measures(fwd):
            logging.info(f"  {fwd} {i}/{runs}: driving - the robot measures this one.")
        else:
            pause(f"{fwd} {i}/{runs}: mark A on the floor under the CENTRE of "
                  "the rear axle.")
        r = _chord_radius(stm, fwd, "Straight-line distance from mark A to the "
                                    "rear axle centre now, cm:", wpose, fast)
        if r is None:
            return None
        out["fwd"].append(r)

        if robot_measures(rev):
            logging.info(f"  {rev} {i}/{runs}: driving back - the robot measures this one.")
        else:
            pause(f"{rev} {i}/{runs}: mark B under the rear axle centre. It "
                  "will reverse back towards A.")
        r = _chord_radius(stm, rev, "Straight-line distance from mark B to the "
                                    "rear axle centre now, cm:", wpose, fast)
        if r is None:
            return None
        out["rev"].append(r)

        if fast is None:
            gap = ask_float("How far is the axle centre from mark A now? cm "
                            "(Enter to skip):")
            if gap is not None:
                home.append(gap)
    return out


def steer_for_radius(points, target_cm: float) -> float:
    """
    The deflection that should give target_cm, from this side's (steer µs,
    radius cm) measurements so far.

    One point: assume R is inversely proportional to the deflection. That
    is the geometry with no tyre slip, and it is a fair first guess.

    Two or more: draw the line through the last two and read the target off
    it (a secant step). This uses how the robot ACTUALLY responded - with the
    rear wheels scrubbing, a 60 µs change can move the radius a fraction of
    what the geometry predicts, and stepping on the model then crawls toward
    the target for rounds. A slope that says more steering widens the turn,
    or that is too flat to trust, is measurement noise; fall back to the
    model rather than follow it.
    """
    s2, r2 = points[-1]
    model = s2 * r2 / target_cm
    if len(points) < 2:
        return model
    s1, r1 = points[-2]
    if s1 == s2:
        return model
    slope = (r2 - r1) / (s2 - s1)          # cm per µs, negative when sane
    if slope >= -1e-3:
        return model
    return s2 + (target_cm - r2) / slope


def step_radius(stm: STM, result: dict,
                target_cm: Optional[float] = RADIUS_TARGET_CM,
                fast: bool = False) -> bool:
    """
    target_cm None = find the radius instead of assuming one: the first round
    runs both sides at full lock, and the wider of the two - the tightest
    circle both sides can trace - becomes the target the other is matched to.

    fast: tape each kind of turn once, then let the odometry measure the rest
    (see _chord_radius). SAFE only - on TIGHT the rear scrub is too heavy for
    the odometry to be trusted at this precision.
    """
    auto = target_cm is None
    print("\n  ── STEP 3: TURN RADIUS, PER SIDE ──────────────────────────────")
    if auto:
        print("  Finds the tightest radius both sides can turn: round 1 at full")
        print("  lock, then the tighter side is widened to match the wider one,")
        print("  forward and reverse.")
    else:
        print(f"  Matches the left and right turn radius to {target_cm} cm, forward")
        print("  and reverse.")
    print(f"  Each side turns forward {RADIUS_ARC_DEG} then reverses back along the")
    print("  same circle, so it needs about 70 cm x 45 cm. Before each arc, mark")
    print("  the floor under the centre of the rear axle; after it, measure")
    print("  straight from that mark to the axle centre - across a half circle")
    print("  that is the diameter. Measure to the millimetre: it is the only")
    print("  number the whole step rests on.")

    proto = cal_profile.firmware_protocol(stm)
    if proto is None or proto < STEER_CAL_MIN_PROTOCOL:
        logging.error(f"Per-side steering needs protocol {STEER_CAL_MIN_PROTOCOL} "
                      "firmware. Flash the current build.")
        return False

    odo_map = None
    if fast:
        if cal_profile.arc_of(result) != cal_profile.ARC_SAFE:
            logging.warning("--fast is for SAFE only: TIGHT's rear wheels scrub "
                            "too much for the odometry. Taping every arc.")
        elif proto < WPOSE_MIN_PROTOCOL:
            logging.warning(f"--fast needs protocol {WPOSE_MIN_PROTOCOL} "
                            "firmware (?WPOSE). Taping every arc.")
        else:
            odo_map = {}
            print("  --fast: each kind of turn is taped ONCE; after that the robot")
            print("  measures its own chord and only asks when an arc looks wrong.")

    cal = current_values(stm)
    if cal is None:
        return False
    steer = {"L": cal["steer_left_us"], "R": cal["steer_right_us"]}
    limit = {"L": CAL_LIMITS["SL"][1], "R": CAL_LIMITS["SR"][1]}
    floor_us = CAL_LIMITS["SL"][0]
    history = []
    points = {"L": [], "R": []}          # (steer µs, radius cm) per round
    if auto:
        steer = dict(limit)
        if not stm.set_cal(steer_left_us=steer["L"], steer_right_us=steer["R"]):
            return False

    for rnd in range(1, RADIUS_MAX_ROUNDS + 1):
        logging.info(f"Round {rnd}: steering left {steer['L']} µs, right {steer['R']} µs")
        # One pulse steers a side both ways, so the side is tuned on the
        # mean of its forward and reverse radius.
        got, row = {}, {"steer_l": steer["L"], "steer_r": steer["R"]}
        for side in "RL":
            m = measure_side(stm, side, RADIUS_RUNS,
                             wpose=proto >= WPOSE_MIN_PROTOCOL, fast=odo_map)
            if m is None:
                return False
            f, r = _mean(m["fwd"]), _mean(m["rev"])
            got[side] = (f + r) / 2.0
            points[side].append((steer[side], got[side]))
            s = side.lower()
            row[f"radius_{s}_cm"] = round(got[side], 2)
            row[f"radius_{s}f_cm"] = round(f, 2)
            row[f"radius_{s}r_cm"] = round(r, 2)
            if m["home"]:
                row[f"home_{s}_cm"] = round(max(m["home"]), 1)
            if abs(f - r) > RADIUS_FWD_REV_WARN_CM:
                logging.warning(
                    f"{'Right' if side == 'R' else 'Left'} side: forward "
                    f"{f:.1f} cm, reverse {r:.1f} cm. Steering cannot separate "
                    "them - check the linkage for play and the tyres for scrub.")
        if auto and rnd == 1:
            target_cm = round(max(got.values()), 1)
            logging.info(f"  Full lock: left {got['L']:.1f} cm, right {got['R']:.1f} cm "
                         f"→ target {target_cm} cm, the tightest both can turn.")
        history.append(row)
        logging.info(f"  radius: left {got['L']:.1f} cm (fwd {row['radius_lf_cm']}, "
                     f"rev {row['radius_lr_cm']}), right {got['R']:.1f} cm "
                     f"(fwd {row['radius_rf_cm']}, rev {row['radius_rr_cm']}) "
                     f"(target {target_cm} ± {RADIUS_TOL_CM})")

        if all(abs(got[s] - target_cm) <= RADIUS_TOL_CM for s in "LR"):
            break
        if rnd == RADIUS_MAX_ROUNDS:
            logging.warning(f"Not within ±{RADIUS_TOL_CM} cm after {rnd} rounds — "
                            "keeping the last values. Check the linkage for play.")
            break

        # Smaller wheel angle, wider circle. Round 1 scales on the geometry;
        # after that each side steps along its own measured response.
        capped = []
        for s_ in "LR":
            if abs(got[s_] - target_cm) <= RADIUS_TOL_CM:
                continue
            want = steer_for_radius(points[s_], target_cm)
            step = max(-RADIUS_MAX_STEP_US, min(RADIUS_MAX_STEP_US, want - steer[s_]))
            new = int(round(steer[s_] + step))
            new = max(floor_us, min(limit[s_], new))
            if new == steer[s_]:
                capped.append(s_)
            steer[s_] = new

        if capped:
            names = {"L": "left", "R": "right"}
            side = capped[0]
            logging.warning(
                f"The {names[side]} side is at its servo limit ({steer[side]} µs) "
                f"and still turns {got[side]:.1f} cm wide of {target_cm}.")
            ans = ask(f"Match the other side to {got[side]:.1f} cm instead? "
                      "The planner and A5 radius must then be changed to match. [y/N]")
            if ans.lower().startswith("y"):
                target_cm = round(got[side], 1)
            else:
                logging.warning("Keeping the target; this side cannot reach it.")

        if not stm.set_cal(steer_left_us=steer["L"], steer_right_us=steer["R"]):
            return False

    result["steer_left_us"] = steer["L"]
    result["steer_right_us"] = steer["R"]
    result["checks"]["radius"] = {"target_cm": target_cm, "rounds": history}
    if odo_map is not None:
        result["checks"]["radius"]["odometry_factor"] = {
            k: (round(v, 4) if v else None) for k, v in odo_map.items()}
    arc = cal_profile.arc_of(result)
    if arc != cal_profile.ARC_TIGHT:
        logging.warning(
            f"{cal_profile.ARC_NAMES[arc]} turn radius is {target_cm} cm. Put "
            f"{int(round(target_cm * 10))} in TURN_RADIUS_MM[PROFILE_"
            f"{cal_profile.ARC_NAMES[arc]}] in algo/stm_tokens.py and in "
            f"radius_mm of the {cal_profile.ARC_NAMES[arc]} row in motion.c.")
    elif abs(target_cm - RADIUS_TARGET_CM) > 1e-6:
        logging.warning(
            f"Turn radius is now {target_cm} cm, not {RADIUS_TARGET_CM}. Update "
            f"TURN_RADIUS_MM[PROFILE_TIGHT] in algo/stm_tokens.py to "
            f"{int(round(target_cm * 10))} and the A5 radius in task_a5.py to "
            f"{target_cm}, then re-check A5_ORBIT.")
    return True


# ── Step 4: brakes ───────────────────────────────────────────────────────────

def run_arcs(stm: STM, seq: List[str], sample_cal: bool):
    """Drive arcs one line each; return per-arc ?CAL and ?HDG readings."""
    cal_samples, stop_errs = [], []
    for i, token in enumerate(seq, 1):
        if not send_move(stm, token):
            return None
        hdg = stm.read_heading()
        if hdg is None:
            return None
        stop_errs.append((token, hdg["last_stop_err"]))
        line = f"  arc {i}/{len(seq)} {token:5s} stopped {hdg['last_stop_err']:+.1f}° from aim"
        if sample_cal:
            cal = current_values(stm)
            if cal is None:
                return None
            cal_samples.append(cal)
            line += (f"   decel {cal['decel_x10']/10:7.1f}  lag "
                     f"{cal['lag_ms_x10']/10:5.1f} ms")
        logging.info(line)
    return cal_samples, stop_errs


def summarise_stops(stop_errs) -> dict:
    by_dir = {}
    for token, err in stop_errs:
        by_dir.setdefault(token[:2], []).append(err)
    worst = max(abs(e) for _, e in stop_errs)
    logging.info("  per direction mean stop error: " + ", ".join(
        f"{d} {_mean(v):+.2f}°" for d, v in sorted(by_dir.items())))
    logging.info(f"  worst single arc: {worst:.2f}° (target ≤ {STOP_ERR_OK_DEG}°)")
    return {"worst_deg": round(worst, 2),
            "by_direction": {d: round(_mean(v), 2) for d, v in by_dir.items()},
            "all": [[t, round(e, 2)] for t, e in stop_errs]}


def step_brakes(stm: STM, result: dict) -> bool:
    proto = cal_profile.firmware_protocol(stm)
    if proto is None:
        return False
    raw = proto >= RAW_DECEL_MIN_PROTOCOL
    learn_seq = BRAKE_LEARN_SEQ if raw else BRAKE_LEARN_SEQ_OLD

    print("\n  ── STEP 4: BRAKES ─────────────────────────────────────────────")
    print(f"  {len(learn_seq)} arcs with learning on, then the average is")
    print(f"  frozen and {len(BRAKE_CHECK_SEQ)} more arcs show how precisely they stop.")
    print("  Needs about 1.5 m x 1.5 m clear.")

    # Protocol 6 learns decel only, from the coast less w*lag: with lag held
    # at a constant, each arc's decel is the one that would have stopped it
    # exactly. Zero is the simplest constant, and the one profiles record.
    if raw and not stm.set_cal(lag_ms_x10=0):
        return False

    if not position_and_zero(stm, "Robot in the middle of the clear area."):
        return False

    if not stm.set_learning(True):
        return False
    try:
        out = run_arcs(stm, learn_seq, sample_cal=True)
    finally:
        stm.set_learning(False)
    if out is None:
        return False
    samples, _ = out

    if raw:
        # Each arc's own measurement. Averaging the filtered value instead
        # leaves about a third of the OLD decel in the answer after eight
        # arcs, because each update only moves a quarter of the way.
        by_dir = {}
        for tok, s in zip(learn_seq, samples):
            if s.get("last_decel_x10"):
                by_dir.setdefault(tok[:2], []).append(s["last_decel_x10"])
        decel = [v for vs in by_dir.values() for v in vs]
        if len(decel) < len(learn_seq) - 1:
            logging.error(
                f"Only {len(decel)} of {len(learn_seq)} arcs gave a braking "
                "measurement; the rest were rejected as out of range. A wheel "
                "slipped or the robot was touched - re-run the step.")
            return False
        mean_decel, mean_lag = int(round(_mean(decel))), 0
        lag = [0]
        logging.info("  raw decel per direction: " + ", ".join(
            f"{d} {_mean(v)/10:.0f}" for d, v in sorted(by_dir.items())))
        logging.info(f"  decel mean {mean_decel/10:.1f} dps² (sd "
                     f"{_stdev(decel)/10:.1f}, {len(decel)} arcs), lag fixed "
                     "at 0 — freezing")
    else:
        # The learner scatters around the answer rather than converging to
        # it, so the mean of all the samples is the estimate.
        logging.warning(f"Firmware is protocol {proto}: it reports no raw "
                        "measurement, so the filtered value is averaged. Flash "
                        f"protocol {RAW_DECEL_MIN_PROTOCOL} for a one-pass result.")
        decel = [s["decel_x10"] for s in samples]
        lag = [s["lag_ms_x10"] for s in samples]
        mean_decel, mean_lag = int(round(_mean(decel))), int(round(_mean(lag)))
        logging.info(f"  decel mean {mean_decel/10:.1f} dps² (sd {_stdev(decel)/10:.1f}), "
                     f"lag mean {mean_lag/10:.1f} ms (sd {_stdev(lag)/10:.1f}) — freezing")
    if not stm.set_cal(decel_x10=mean_decel, lag_ms_x10=mean_lag):
        return False

    out = run_arcs(stm, BRAKE_CHECK_SEQ, sample_cal=False)
    if out is None:
        return False
    check = summarise_stops(out[1])

    result["spread"] = {"decel_x10": {"sd": round(_stdev(decel), 1),
                                      "min": min(decel), "max": max(decel)},
                        "lag_ms_x10": {"sd": round(_stdev(lag), 1),
                                       "min": min(lag), "max": max(lag)}}
    result["checks"]["brakes"] = check
    if check["worst_deg"] > STOP_ERR_OK_DEG:
        logging.warning(
            f"An arc stopped {check['worst_deg']:.2f}° from its aim with learning "
            "off. The carry-over corrects it on the next move, but single-turn "
            "precision is limited by how repeatably the robot stops - check the "
            "battery and the floor before re-running.")
    return True


# ── Step 5: square ───────────────────────────────────────────────────────────

def step_square(stm: STM, result: dict, side_cm: int = SQUARE_SIDE_CM) -> bool:
    print("\n  ── STEP 5: SQUARE ─────────────────────────────────────────────")
    print(f"  F{side_cm} + FR90, four times, learning off. It should come back to")
    print("  where it started, facing the same way. Mark the rear axle and lay")
    print("  a reference edge along the robot's side before it goes.")
    print(f"  Clear about {side_cm + 70} cm x {side_cm + 70} cm, to the right.")

    if not position_and_zero(stm, "Robot at the start mark, side against the edge."):
        return False

    stops = []
    for _ in range(4):
        for token in (f"F{side_cm}", "FR90"):
            if not send_move(stm, token):
                return False
        hdg = stm.read_heading()
        if hdg is None:
            return False
        stops.append(("FR", hdg["last_stop_err"]))
        logging.info(f"  corner {len(stops)}: stopped {hdg['last_stop_err']:+.1f}° "
                     f"from aim, heading error now {hdg['error']:+.1f}°")

    final = stm.read_heading()
    gap = ask_float("Distance between the start mark and where the rear axle "
                    "ended, cm:", allow_blank=False)
    angle = ask_angle("Final angle from the reference edge, degrees "
                      "(or offset/span mm):")

    worst = max(abs(e) for _, e in stops)
    passed = (worst <= STOP_ERR_OK_DEG and abs(final["error"]) <= SQUARE_HDG_OK_DEG
              and abs(angle) <= SQUARE_ANGLE_OK_DEG and abs(gap) <= SQUARE_GAP_OK_CM)

    logging.info(f"  worst corner {worst:.2f}° (≤{STOP_ERR_OK_DEG}), gyro heading "
                 f"error {final['error']:+.2f}° (≤{SQUARE_HDG_OK_DEG}), floor angle "
                 f"{angle:+.2f}° (≤{SQUARE_ANGLE_OK_DEG}), gap {gap:.1f} cm "
                 f"(≤{SQUARE_GAP_OK_CM}) → {'PASS' if passed else 'FAIL'}")
    if abs(final["error"]) <= SQUARE_HDG_OK_DEG < abs(angle):
        logging.warning(
            "The gyro says square but the floor says not. That is the gyro "
            "scale - re-run `calibrate.py gyro`, then radius and brakes.")

    result["checks"]["square"] = {
        "side_cm": side_cm, "worst_corner_deg": round(worst, 2),
        "gyro_heading_err_deg": round(final["error"], 2),
        "floor_angle_deg": round(angle, 2), "gap_cm": gap, "passed": passed,
        "when": datetime.now().isoformat(timespec="seconds"),
    }
    result["verified"] = passed
    return True


# ── Diagnostic: gyro drift at rest ───────────────────────────────────────────

# A healthy, bias-corrected gyro at rest: well under 0.3° in 30 s.
DRIFT_OK_DPS = 0.01
DRIFT_SECONDS = 30


def drift_rate(stm: STM, seconds: int) -> Optional[float]:
    """Zero the heading, leave the robot alone, return the gyro's drift in °/s."""
    if not cal_profile.zero_heading(stm):
        return None
    t0 = time.monotonic()
    for i in range(1, seconds + 1):
        time.sleep(max(0.0, t0 + i - time.monotonic()))
        hdg = stm.read_heading()
        if hdg is None:
            return None
        if i % 10 == 0 or i == seconds:
            logging.info(f"  {i:3d} s: heading {hdg['actual']:+.1f}°")
    return hdg["actual"] / (time.monotonic() - t0)


def gyro_drift(stm: STM, seconds: int = DRIFT_SECONDS) -> bool:
    """
    The robot is not moving, so every degree the gyro reports is error. Run
    twice: once after the robot has stood still, once straight after it was
    handled - the firmware tracks bias whenever the wheels are still, and a
    robot lifted or turned by hand has still wheels too.
    """
    print("\n  ── GYRO DRIFT AT REST ─────────────────────────────────────────")
    print(f"  Two {seconds} s readings with nothing moving. Do not touch the robot")
    print("  or the table while it counts.")

    pause("1/2: robot on the floor and left alone for 30 s already. Enter to start.")
    still = drift_rate(stm, seconds)
    if still is None:
        return False
    pause("2/2: pick the robot up, turn it about 90° and back, put it down, and "
          "press Enter AT ONCE.")
    handled = drift_rate(stm, seconds)
    if handled is None:
        return False

    for label, r in (("left alone", still), ("just handled", handled)):
        logging.info(f"  {label:12s}: {r:+.4f} °/s = {r * 30:+.1f}° per 30 s "
                     f"→ {'OK' if abs(r) <= DRIFT_OK_DPS else 'DRIFTING'}")
    if abs(handled) > max(3 * abs(still), DRIFT_OK_DPS):
        logging.warning(
            "Handling the robot corrupts the gyro bias: the zero-rate tracker "
            "learns the hand rotation as bias, then takes tens of seconds to "
            "unlearn it. Every calibration step starts by positioning the "
            "robot by hand, so this lands in the measurements. Needs the "
            "firmware fix in imu.c (IMU_TrackBias rate gate).")
    elif abs(still) > DRIFT_OK_DPS:
        logging.warning("Drifting even when left alone: the power-on bias was "
                        "measured while the robot moved, or the part is warming "
                        "up. Power-cycle with the robot still, wait a minute, re-run.")
    return True


# ── Diagnostic: reverse signs ────────────────────────────────────────────────
#
# Reversing flips two signs in the firmware, and neither had been checked on
# the robot (motion.c still says "VERIFY THIS ON THE ROBOT"):
#
#   arcs       which way the body rotates for RR / RL (Motion_DriveArc)
#   straights  which way heading hold steers when backing up (odom.c flips the
#              correction for negative rpm)
#
# Forward is known good, so every reverse check is run next to its forward
# twin: if the forward one fails too, the test is wrong, not the reverse.

REV_ARC_DEG = 45
# FR/RL turn the body clockwise, FL/RR anticlockwise; heading is CCW positive.
REV_ARC_EXPECT = {"FR": -1, "RR": +1, "FL": +1, "RL": -1}
REV_ARC_TOL_DEG = 10.0

# Long enough for the cross-track check to separate: its lean is 0.08 deg per
# mm, so an offset decays (or, sign wrong, grows) by e over about 70 cm.
REV_STRAIGHT_CM = 100
# A hand twist inside this window. Under 2 the before/after difference is
# lost in the stop error; past MOTION_CARRY_MAX_DEG (5) the firmware caps the
# correction, which still shows the sign but spends more floor.
REV_TWIST_MIN_DEG = 2.0
REV_TWIST_MAX_DEG = 8.0
REV_TWIST_TRIES = 3
# Firmware heading carry-over cap, MOTION_CARRY_MAX_DEG in motion.h. The
# straight holds the twist error up to this, and its line runs along it.
REV_CARRY_MAX_DEG = 5.0
# Cross-track verdict, offset from the line at the end of the straight, mm.
# Under the heading-hold deadband (0.3 deg = ~4 mm of lean) nothing corrects,
# so below REV_CROSS_OK_MM both outcomes look alike.
REV_CROSS_OK_MM = 6.0
REV_CROSS_FAIL_MM = 12.0


def _move_reply(stm: STM, token: str) -> Optional[str]:
    """One move, its reply as-is. FAIL,WRONGWAY is a result here, not an error."""
    if not stm.send_line([token]):
        return None
    reply = stm.wait_reply()
    if reply is None:
        logging.error(f"{token}: no reply — lost link.")
        return None
    return reply.strip().upper()


def _arc_sign_check(stm: STM, side: str) -> Optional[dict]:
    """F{side}45 out, R{side}45 back. Each must reply OK and turn the right way."""
    way = "right" if side == "R" else "left"
    if not position_and_zero(stm, f"ARCS {way.upper()}: robot with about 40 cm "
                             f"clear ahead and to the {way}, and behind it."):
        return None
    out = {}
    for drive in ("F", "R"):
        key = drive + side
        token = f"{key}{REV_ARC_DEG}"
        # The actual heading before and after, not ?HDG's last_turned: that is
        # only written when an arc completes, so after a FAIL it still holds
        # the PREVIOUS arc and reported a wrong-way turn as a full right one.
        start = stm.read_heading()
        if start is None:
            return None
        reply = _move_reply(stm, token)
        if reply is None:
            return None
        hdg = stm.read_heading()
        if hdg is None:
            return None
        turned = hdg["actual"] - start["actual"]
        want = REV_ARC_EXPECT[key]
        if reply == "OK" and turned * want > 0 \
                and abs(abs(turned) - REV_ARC_DEG) <= REV_ARC_TOL_DEG:
            verdict = "PASS"
        else:
            verdict = "FAIL"
        logging.info(f"  {token}: {reply}, turned {turned:+.1f}° "
                     f"(expected {'+' if want > 0 else '-'}{REV_ARC_DEG}) → {verdict}")
        out[key] = {"reply": reply, "turned": turned, "pass": verdict == "PASS"}
        if reply != "OK":
            # The firmware re-synced to where the robot really is; zero so the
            # next arc does not inherit a 20 degree wrong-way excursion.
            if not position_and_zero(stm, "Put the robot back where it started."):
                return None
    return out


def _twisted_heading_error(stm: STM, nose: str) -> Optional[float]:
    """Zero, have the robot twisted by hand, return the error it now sees."""
    for _ in range(REV_TWIST_TRIES):
        if not cal_profile.zero_heading(stm):
            return None
        pause(f"Twist the robot about 5° NOSE {nose.upper()} by hand, in place, "
              "and let go. Do not straighten it.")
        hdg = stm.read_heading()
        if hdg is None:
            return None
        err = hdg["error"]
        if REV_TWIST_MIN_DEG <= abs(err) <= REV_TWIST_MAX_DEG:
            return err
        print(f"    Gyro saw {-err:+.1f}° — want {REV_TWIST_MIN_DEG:.0f} to "
              f"{REV_TWIST_MAX_DEG:.0f}°. Put it back straight and try again.")
    logging.error("Could not get a usable twist.")
    return None


def _straight_sign_check(stm: STM, drive: str, nose: str) -> Optional[dict]:
    """
    Twist the robot off its zeroed heading, then drive straight. Heading hold
    aims back at the zeroed heading, so the robot must ROTATE TOWARD it. With
    the correction sign wrong it rotates further away instead.

    The verdict is on the direction of rotation, not on where it finished: on
    the floor a 5 degree kick overshoots - F100 went -5.0 -> +4.5, a 9.5
    degree swing the right way - and judging by the final error called that
    a failure.
    """
    token = f"{drive}{REV_STRAIGHT_CM}"
    before = _twisted_heading_error(stm, nose)
    if before is None:
        return None
    reply = _move_reply(stm, token)
    if reply is None:
        return None
    hdg = stm.read_heading()
    if hdg is None:
        return None
    after = hdg["error"]
    # Commanded stays 0 on a straight, so the change in error is the rotation;
    # it has the sign of `before` when the robot turned back toward the line.
    turned = before - after
    if reply == "OK" and turned * before > 0 and abs(turned) >= 0.5 * abs(before):
        verdict = "PASS"            # rotated back toward the line
    elif turned * before < 0 and abs(turned) > 1.0:
        verdict = "FAIL"            # rotated further away: sign is wrong
    else:
        verdict = "UNCLEAR"
    overshoot = verdict == "PASS" and after * before < 0 \
        and abs(after) > 0.5 * abs(before)
    cross = _cross_offset_mm(stm, before)
    if cross is None:
        return None
    logging.info(f"  {token}, nose {nose}: heading error {before:+.1f}° → "
                 f"{after:+.1f}°, turned {turned:+.1f}° ({reply}) → {verdict}"
                 + (" (overshot)" if overshoot else "")
                 + f"; {cross:+.0f} mm off the line")
    return {"before": before, "after": after, "reply": reply, "verdict": verdict,
            "overshoot": overshoot, "cross_mm": cross}


def _cross_offset_mm(stm: STM, held_err: float) -> Optional[float]:
    """
    Where the straight finished relative to the line it was steering for -
    the same quantity odom.c drives to zero, from the same dead reckoning.

    ?POSE is in the frame of the move's start, and the line runs through the
    start along the heading the move held: the twist error, capped like the
    firmware caps it.
    """
    fields = stm.query_fields("?POSE")
    if fields is None or len(fields) < 2:
        logging.error(f"?POSE did not answer properly: {fields}")
        return None
    try:
        x, y = float(fields[0]), float(fields[1])
    except ValueError:
        logging.error(f"Non-numeric ?POSE reply: {fields}")
        return None
    held = max(-REV_CARRY_MAX_DEG, min(REV_CARRY_MAX_DEG, held_err))
    hr = math.radians(held)
    return y * math.cos(hr) - x * math.sin(hr)


def _cross_verdict(fwd_mm: float, rev_mm: float) -> str:
    """
    Reverse against forward from the same twist. Correct signs mirror each
    other; a wrong reverse cross-track sign pushes away from the line and
    ends several times further out than forward did.
    """
    if abs(rev_mm) <= max(REV_CROSS_OK_MM, 1.5 * abs(fwd_mm)):
        return "PASS"
    if abs(rev_mm) >= max(REV_CROSS_FAIL_MM, 3.0 * abs(fwd_mm)):
        return "FAIL"
    return "UNCLEAR"


def reverse_check(stm: STM) -> bool:
    """
    Deliberate test of the two reverse sign conventions. Diagnostic only:
    nothing is saved to a profile. Returns True only if every reverse check
    passes and its forward control does too.
    """
    print("\n  ── REVERSE SIGN CHECK ─────────────────────────────────────────")
    print(f"  Part 1: F{{R,L}}{REV_ARC_DEG} out and R{{R,L}}{REV_ARC_DEG} back. A wrong reverse")
    print("  arc sign makes the firmware abort it as FAIL,WRONGWAY.")
    print(f"  Part 2: twist the robot ~5° by hand, then F{REV_STRAIGHT_CM} / "
          f"R{REV_STRAIGHT_CM}. Heading")
    print("  hold must steer back - the error shrinks. With the reverse sign")
    print("  wrong it steers further off and the error grows.")
    print("  The same runs check cross-track: the twist pushes the robot off its")
    print("  line before the heading recovers, and the reverse run must come")
    print("  back onto the line about as well as the forward one does.")
    print("  Learning is off throughout; nothing is saved.")

    arcs = {}
    for side in ("R", "L"):
        got = _arc_sign_check(stm, side)
        if got is None:
            return False
        arcs.update(got)

    pause(f"STRAIGHTS: about {REV_STRAIGHT_CM + 20} cm clear ahead and behind. "
          "The robot runs forward then back, twice.")
    straights = {}
    for nose in ("left", "right"):
        for drive in ("F", "R"):
            got = _straight_sign_check(stm, drive, nose)
            if got is None:
                return False
            straights[(drive, nose)] = got

    fwd_ok = all(arcs[k]["pass"] for k in ("FR", "FL")) and \
        all(straights[(d, n)]["verdict"] == "PASS"
            for (d, n) in straights if d == "F")
    rev_arc_ok = all(arcs[k]["pass"] for k in ("RR", "RL"))
    rev_str = [straights[(d, n)]["verdict"] for (d, n) in straights if d == "R"]
    # Cross-track only means something once heading hold steers the right way.
    rev_cross = [_cross_verdict(straights[("F", n)]["cross_mm"],
                                straights[("R", n)]["cross_mm"])
                 if straights[("R", n)]["verdict"] == "PASS" else "N/A"
                 for n in ("left", "right")]

    print("\n  ── RESULT ──")
    if not fwd_ok:
        logging.error("A FORWARD control failed, and forward is known good. Do not "
                      "trust the reverse results: check the IMU (?STAT imu_ok), "
                      "the battery, and that the twist was not undone by hand.")
        return False
    logging.info(f"  Reverse arcs:      {'OK' if rev_arc_ok else 'WRONG SIGN'}")
    logging.info(f"  Reverse straights: {', '.join(rev_str)}")
    logging.info(f"  Reverse cross-track: {', '.join(rev_cross)}")
    if not rev_arc_ok:
        logging.error("Reverse arc sign is wrong: flip the reverse cases of the "
                      "body-rotation sign in Motion_DriveArc (motion.c), NOT "
                      "MOTION_ARC_SIGN - that would break the forward turns.")
    if "FAIL" in rev_str:
        logging.error("Reverse heading hold steers the wrong way: the "
                      "'if (s_holdRpm < 0) correction = -correction' flip in "
                      "odom.c is backwards for this robot.")
    elif "UNCLEAR" in rev_str:
        logging.warning("Reverse straight neither corrected nor diverged clearly. "
                        "Re-run with a bigger twist (4-5°).")
    if "FAIL" in rev_cross:
        logging.error("Reverse cross-track steers away from the line: the "
                      "'if (s_holdRpm < 0) cross = -cross' flip in odom.c is "
                      "backwards for this robot.")
    elif "UNCLEAR" in rev_cross:
        logging.warning("Reverse cross-track inconclusive - either the twist did not "
                        "push the robot far enough off its line (re-run with 4-5°), "
                        "or the heading weaving swamped it. No runaway, so not a "
                        "wrong sign.")
    if any(s["overshoot"] for s in straights.values()):
        logging.warning("Heading hold swung past straight after the twist (marked "
                        "'overshot'): the sign is right but the loop is lively. "
                        "Real runs start ~1° off, not 5°; if straights weave, lower "
                        "HEADING_KP_US or CROSS_KP_DEG_PER_MM in odom.h.")
    return rev_arc_ok and all(v == "PASS" for v in rev_str + rev_cross)


# ── Profile bookkeeping ──────────────────────────────────────────────────────

def snapshot(stm: STM, result: dict) -> bool:
    v = current_values(stm)
    if v is None:
        return False
    for k in ("decel_x10", "lag_ms_x10", "trim_us", "gyro_x10000",
              "steer_left_us", "steer_right_us"):
        if v.get(k) is not None:
            result[k] = v[k]
    return True


# ── Diagnostic: encoders by hand ─────────────────────────────────────────────
#
# Driving cannot test an encoder: the speed loop holds both wheels' COUNT rates
# equal, so a wheel whose encoder drops counts is simply driven faster until it
# matches, and every onboard number agrees with itself. With the motors off and
# a wheel turned by hand there is no loop to hide behind.

ENC_COUNTS_PER_REV = 1560     # ODOM_COUNTS_PER_REV_A/B in odom.h
ENC_TURNS = 5
ENC_OK_PCT = 1.0              # a hand count is good to a few degrees in 5 turns


def encoder_check(stm: STM, turns: int = ENC_TURNS, reverse: bool = False) -> bool:
    """
    reverse: turn the wheels BACKWARDS and expect negative counts. Forward
    alone misses an encoder that only misbehaves one way round - and both of
    the left-turn faults so far were in reverse.
    """
    way = "BACKWARD" if reverse else "FORWARD"
    sign = -1 if reverse else 1
    proto = cal_profile.firmware_protocol(stm)
    if proto is None:
        return False
    if proto < ENC_MIN_PROTOCOL:
        logging.error(f"?ENC needs protocol {ENC_MIN_PROTOCOL} firmware; this is "
                      f"{proto}. Flash the current build.")
        return False
    if stm.is_busy():
        logging.error("Robot is moving — wait for it to stop.")
        return False

    print("\n  ── ENCODERS BY HAND ───────────────────────────────────────────")
    print("  Lift the robot so both rear wheels are clear of the floor. Nothing")
    print(f"  will drive. Put a mark on each tyre. For each wheel, turn it {way}")
    print(f"  exactly {turns} full turns by hand, at a steady pace, then press Enter.")
    print("  If a wheel is suspect, wiggle its encoder cable while turning it.")

    want = sign * turns * ENC_COUNTS_PER_REV
    ok = True
    for idx, name in ((0, "A, LEFT rear"), (1, "B, RIGHT rear")):
        start = stm.read_enc()
        if start is None:
            return False
        pause(f"Wheel {name}: turn it {turns} turns {way.lower()} now.")
        end = stm.read_enc()
        if end is None:
            return False
        got = end[idx] - start[idx]
        other = end[1 - idx] - start[1 - idx]
        err = (got - want) / abs(want) * 100.0 * sign
        verdict = "OK"
        if got * sign < 0:
            verdict = "COUNTS BACKWARDS"
        elif abs(err) > ENC_OK_PCT:
            verdict = "WRONG COUNT"
        logging.info(f"  wheel {name}: {got:+d} counts for {turns} turns "
                     f"(want {want:+d}, {err:+.1f}%) → {verdict}"
                     + (f"; the other wheel moved {other:+d}" if other else ""))
        if verdict == "COUNTS BACKWARDS":
            logging.error(f"Wheel {name} counts backwards: ENCODER_"
                          f"{'AB'[idx]}_INVERT in encoders.h is wrong.")
        elif verdict == "WRONG COUNT":
            logging.error(
                f"Wheel {name} is {err:+.1f}% off. Short = dropping counts: "
                "reseat its encoder connector and check the cable and solder "
                "joints. Long on both wheels = check the turn count, then "
                "ODOM_COUNTS_PER_REV in odom.h.")
        ok = ok and verdict == "OK"
    return ok


# ── Diagnostic: which IR is which, and what it reads with nothing there ──────
#
# ?IR answers left, right. If the two are swapped, every side-IR measurement
# looks the wrong way - and with nothing in range the IR does not go blank, it
# reads its background (seen: 29-75 cm), which looks like a real distance.

IR_TEST_BOX_CM = 15
IR_TEST_SAMPLES = 12


def _ir_pair(stm: STM) -> Optional[tuple]:
    """Median left and right ?IR over a moment; None for a blank side."""
    left, right = [], []
    for _ in range(IR_TEST_SAMPLES):
        fields = stm.query_fields("?IR")
        if fields and len(fields) >= 2:
            for vals, f in ((left, fields[0]), (right, fields[1])):
                try:
                    v = int(f)
                except ValueError:
                    continue
                vals.append(None if v == 0xFFFF else v)
        time.sleep(0.05)
    if not left or not right:
        logging.error("?IR did not answer.")
        return None

    def med(vals):
        nums = sorted(v for v in vals if v is not None)
        return nums[len(nums) // 2] if len(nums) * 2 > len(vals) else None
    return med(left), med(right)


def ir_side_check(stm: STM) -> bool:
    """Open floor, then a box on the robot's right, then its left."""
    print("\n  ── IR SIDES ───────────────────────────────────────────────────")
    print("  Three readings. Keep everything else at least 1 m from the robot's")
    print("  sides. 'Box' = any flat board or box, held upright and square to")
    print(f"  the side of the robot, {IR_TEST_BOX_CM} cm from it.")

    shown = lambda v: "blank" if v is None else f"{v} cm"
    pause("1/3: NOTHING beside the robot on either side.")
    base = _ir_pair(stm)
    if base is None:
        return False
    logging.info(f"  open floor:  left IR {shown(base[0])}, right IR {shown(base[1])}")
    pause(f"2/3: box {IR_TEST_BOX_CM} cm from the robot's RIGHT side.")
    on_r = _ir_pair(stm)
    if on_r is None:
        return False
    logging.info(f"  box right:   left IR {shown(on_r[0])}, right IR {shown(on_r[1])}")
    pause(f"3/3: box {IR_TEST_BOX_CM} cm from the robot's LEFT side.")
    on_l = _ir_pair(stm)
    if on_l is None:
        return False
    logging.info(f"  box left:    left IR {shown(on_l[0])}, right IR {shown(on_l[1])}")

    def near(v):
        return v is not None and abs(v - IR_TEST_BOX_CM) <= 6

    ok = True
    for name, idx, box_here, box_there in (("right", 1, on_r, on_l),
                                           ("left", 0, on_l, on_r)):
        if near(box_here[idx]):
            logging.info(f"  {name} IR: sees the box on the {name} - correct.")
        elif near(box_there[idx]):
            logging.error(f"  {name} IR: answered for the box on the OTHER side - "
                          "left and right are SWAPPED (ir.c / the ADC channel "
                          "order, or the plugs).")
            ok = False
        else:
            logging.error(f"  {name} IR: never read ~{IR_TEST_BOX_CM} cm - faulty, "
                          "unplugged, or aimed away from the side.")
            ok = False
    for name, v in (("left", base[0]), ("right", base[1])):
        if v is not None:
            logging.warning(f"  {name} IR reads {v} cm with NOTHING there. Readings "
                            f"around {v} cm cannot be trusted as a real distance; "
                            f"set T2T_IR_FACE_MAX_CM / T2_IR_MAX_GAP_"
                            f"{name.upper()}_CM well below it.")
    return ok


STEP_ORDER = ("straight", "gyro", "radius", "brakes")

# What re-running a step leaves stale. The trim only enters the straight-line
# heading loop (odom.c) - arcs set the servo from centre +/- the per-side
# deflection without it - so a new trim touches nothing else. The radius
# chord uses the gyro's angle and braking is learned in gyro degrees, so a
# new gyro scale stales both; new steering changes how an arc coasts.
STALE_AFTER = {"straight": (), "gyro": ("radius", "brakes"),
               "radius": ("brakes",), "brakes": ()}


def run_one_step(stm: STM, name: str, step: str, note: Optional[str] = None,
                 target_cm="default", arc: Optional[int] = None,
                 fast: bool = False) -> bool:
    """
    One guided step on its own, saved into profile `name` without touching
    the other steps' results. A missing profile is started from the robot's
    current values, so a profile can be built one step at a time.

    arc: the arc profile to calibrate on. An existing profile keeps its own,
    and asking for a different one is refused - a SAFE step folded into a
    TIGHT calibration would fit neither. None = the profile's, or TIGHT.
    target_cm: "default" = default_target() for the arc.
    """
    if os.path.exists(cal_profile.profile_path(name)):
        p = cal_profile.load(name)
        if p is None:
            return False
        have = cal_profile.arc_of(p)
        if arc is not None and arc != have:
            logging.error(
                f"{name!r} is a {cal_profile.ARC_NAMES[have]} calibration; it "
                f"cannot take a {cal_profile.ARC_NAMES[arc]} step. Use another "
                "name for that profile.")
            return False
        arc = have
        if not select_arc(stm, arc) or not cal_profile.push(stm, p):
            return False
    else:
        arc = cal_profile.ARC_PROFILE if arc is None else arc
        logging.info(f"No profile {name!r} yet — starting it as "
                     f"{cal_profile.ARC_NAMES[arc]} from the robot's current values.")
        if not select_arc(stm, arc):
            return False
        p = {"note": "", "verified": False, "arc_profile": arc}
    if target_cm == "default":
        target_cm = default_target(arc)
    steps = {"straight": step_straight, "gyro": step_gyro,
             "radius": lambda s, r: step_radius(s, r, target_cm=target_cm,
                                                fast=fast),
             "brakes": step_brakes}
    p.setdefault("checks", {})
    if note is not None:
        p["note"] = note

    # gyro_ok() after the step: a gyro that dropped out mid-step leaves
    # numbers measured on encoder heading, which must not be saved.
    if not steps[step](stm, p) or not gyro_ok(stm) or not snapshot(stm, p):
        return False
    p["verified"] = False
    p["taken"] = datetime.now().isoformat(timespec="seconds")
    cal_profile.save(name, p)

    stale = STALE_AFTER[step]
    logging.info(f"Saved {step} into {name!r}. Repeat it until it looks right, "
                 "then " + (f"re-run {', '.join(stale)}, then " if stale else "")
                 + f"`verify {name}`.")
    report(dict(p, name=name))
    return True


def report(p: dict) -> None:
    print()
    print(f"  profile  : {p.get('name')}   ({'VERIFIED' if p.get('verified') else 'not verified'})"
          f"   arc {cal_profile.ARC_NAMES.get(cal_profile.arc_of(p), '?')}")
    if p.get("note"):
        print(f"  note     : {p['note']}")
    print(f"  taken    : {p.get('taken')}")
    print(f"  decel    : {p['decel_x10']/10:8.1f} dps²")
    print(f"  lag      : {p['lag_ms_x10']/10:8.1f} ms")
    print(f"  trim     : {p['trim_us']:8d} µs")
    print(f"  gyro     : ×{cal_profile.values_of(p)[3]/10000:.4f}")
    print(f"  steering : left {cal_profile.values_of(p)[4]} µs, "
          f"right {cal_profile.values_of(p)[5]} µs")
    st = p.get("checks", {}).get("straight")
    if st:
        drift = f"{st['drift_cm'][-1]:+} cm" if st.get("drift_cm") else "not measured"
        print(f"  straight : last drift {drift}, trim "
              f"{'settled' if st['converged'] else 'NOT settled'}")
    gy = p.get("checks", {}).get("gyro")
    if gy:
        print(f"  gyro chk : ×{gy['from']:.4f} → ×{gy['to']:.4f}, "
              f"left/right disagree {gy['asymmetry_pct']}%")
    rad = p.get("checks", {}).get("radius")
    if rad and rad.get("rounds"):
        last = rad["rounds"][-1]
        print(f"  radius   : left {last['radius_l_cm']} cm, right "
              f"{last['radius_r_cm']} cm (target {rad['target_cm']})")
        if "radius_lr_cm" in last:
            print(f"             fwd/rev  left {last['radius_lf_cm']}/"
                  f"{last['radius_lr_cm']}, right {last['radius_rf_cm']}/"
                  f"{last['radius_rr_cm']} cm")
    sq = p.get("checks", {}).get("square")
    if sq:
        print(f"  square   : gap {sq['gap_cm']} cm, floor angle "
              f"{sq['floor_angle_deg']:+}°, worst corner {sq['worst_corner_deg']}° "
              f"— {'PASS' if sq['passed'] else 'FAIL'} ({sq['when']})")
    print()


# ── Entry point ──────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description="Calibrate, check by eye, and save.")
    sub = ap.add_subparsers(dest="cmd")
    sub.required = True

    g = sub.add_parser("guided", help="full on-floor calibration, steps 1-4")
    g.add_argument("name")
    g.add_argument("--note", default="", help="floor, battery, load")
    g.add_argument("--from", dest="start_from", default=None,
                   help="start from this saved profile instead of the robot's values")
    g.add_argument("--skip", default="",
                   help="comma list of steps to skip: straight,gyro,radius,brakes,square")
    arc_help = ("arc profile to calibrate on: tight (default) or safe. Each "
                "keeps its own saved calibration")
    g.add_argument("--arc", choices=sorted(cal_profile.ARC_BY_NAME), default=None,
                   help=arc_help)

    for step, help_ in (("straight", "step 1 only: steering trim, keep the rest"),
                        ("gyro", "step 2 only: gyro scale, keep the rest"),
                        ("radius", "step 3 only: re-match left/right turn radius"),
                        ("brakes", "step 4 only: braking, keep trim, gyro, radius")):
        one = sub.add_parser(step, help=help_)
        one.add_argument("name")
        one.add_argument("--note", default=None)
        one.add_argument("--arc", choices=sorted(cal_profile.ARC_BY_NAME),
                         default=None,
                         help=arc_help + "; an existing profile keeps its own")
        if step == "radius":
            one.add_argument("--target", default=None,
                             help="radius both sides are matched to, cm "
                                  f"(default {RADIUS_TARGET_CM} on TIGHT, auto "
                                  "on SAFE), or 'auto' to find the tightest "
                                  "both sides can turn")
            one.add_argument("--fast", action="store_true",
                             help="SAFE only: tape each kind of turn once, "
                                  "then the robot measures its own chords")

    v = sub.add_parser("verify", help="restore a profile and run the square test")
    v.add_argument("name")
    v.add_argument("--side", type=int, default=SQUARE_SIDE_CM)

    dr = sub.add_parser("drift", help="diagnostic: gyro drift at rest, before and "
                                      "after handling the robot")
    dr.add_argument("--seconds", type=int, default=DRIFT_SECONDS)

    sub.add_parser("irsides", help="diagnostic: which side IR is which, and "
                                   "what each reads with nothing there")
    en = sub.add_parser("encoders", help="diagnostic: robot lifted, turn each "
                                         "wheel by hand; do both encoders count "
                                         "1560 a turn? Saves nothing")
    en.add_argument("--turns", type=int, default=ENC_TURNS)
    en.add_argument("--reverse", action="store_true",
                    help="turn the wheels backwards instead")

    rv = sub.add_parser("reverse", help="diagnostic: are the reverse arc and "
                                        "reverse heading-hold signs right? "
                                        "Saves nothing")
    rv.add_argument("--arc", choices=sorted(cal_profile.ARC_BY_NAME), default=None,
                    help="arc profile to drive it on (default tight)")

    sub.add_parser("zero", help="make the current pose heading 0 (after moving "
                               "the robot by hand). Tasks do this at startup")

    r = sub.add_parser("restore", help="send a saved profile to the robot, frozen")
    r.add_argument("name")

    sub.add_parser("list", help="list saved profiles")
    s = sub.add_parser("show", help="print a saved profile")
    s.add_argument("name")

    args = ap.parse_args()

    if args.cmd == "list":
        names = cal_profile.list_profiles()
        if not names:
            print("No profiles saved yet.")
        for n in names:
            p = cal_profile.load(n)
            if p:
                print(f"  {n:20s} {'✓' if p.get('verified') else ' '} "
                      f"{cal_profile.ARC_NAMES[cal_profile.arc_of(p)]:5s}  "
                      f"decel {p['decel_x10']/10:7.1f}  lag {p['lag_ms_x10']/10:5.1f}  "
                      f"trim {p['trim_us']:+3d}  gyro ×{cal_profile.values_of(p)[3]/10000:.4f}  "
                      f"steer L{cal_profile.values_of(p)[4]}/R{cal_profile.values_of(p)[5]}"
                      f"   {p.get('note', '')}")
        return 0

    if args.cmd == "show":
        p = cal_profile.load(args.name)
        if p is None:
            return 1
        report(p)
        return 0

    stm = STM()
    try:
        if args.cmd == "zero":
            # Deliberately NOT connect_and_check(): zeroing must not change the
            # learning switch or the profile as a side effect.
            try:
                stm.connect()
            except Exception as exc:
                logging.error(f"Could not open the serial port: {exc}")
                return 1
            proto = cal_profile.firmware_protocol(stm)
            if proto is None or proto < FROZEN_CAL_MIN_PROTOCOL:
                logging.error("Needs protocol 4 firmware (?HDG). Flash the current build.")
                return 1
            before = stm.read_heading()
            if before is not None:
                logging.info(f"Before: commanded {before['commanded']:+.1f}°, "
                             f"actual {before['actual']:+.1f}°, "
                             f"error {before['error']:+.1f}°")
            return 0 if cal_profile.zero_heading(stm) else 1

        if not connect_and_check(stm):
            return 1

        if args.cmd == "drift":
            return 0 if gyro_drift(stm, args.seconds) else 1

        if args.cmd == "irsides":
            return 0 if ir_side_check(stm) else 1

        if args.cmd == "encoders":
            return 0 if encoder_check(stm, args.turns, args.reverse) else 1

        if args.cmd == "reverse":
            arc = arc_from_args(args)
            if arc is not None and not select_arc(stm, arc):
                return 1
            return 0 if reverse_check(stm) else 1

        if args.cmd == "restore":
            p = cal_profile.load(args.name)
            if p is None or not select_arc(stm, cal_profile.arc_of(p)) \
                    or not cal_profile.push(stm, p):
                return 1
            logging.info(f"Restored {args.name!r} "
                         f"({cal_profile.ARC_NAMES[cal_profile.arc_of(p)]}), "
                         "learning off.")
            return 0

        if args.cmd == "verify":
            p = cal_profile.load(args.name)
            if p is None or not select_arc(stm, cal_profile.arc_of(p)) \
                    or not cal_profile.push(stm, p):
                return 1
            p.setdefault("checks", {})
            if not step_square(stm, p, side_cm=args.side) or not gyro_ok(stm):
                return 1
            cal_profile.save(args.name, p)
            report(p)
            return 0 if p["verified"] else 1

        if args.cmd in STEP_ORDER:
            target = getattr(args, "target", None)
            try:
                target_cm = ("default" if target is None
                             else None if target.lower() == "auto"
                             else float(target))
            except ValueError:
                logging.error(f"--target {target!r}: a number of cm, or 'auto'.")
                return 1
            ok = run_one_step(stm, args.name, args.cmd, note=args.note,
                              target_cm=target_cm, arc=arc_from_args(args),
                              fast=getattr(args, "fast", False))
            return 0 if ok else 1

        # guided
        skip = {x.strip() for x in args.skip.split(",") if x.strip()}
        arc = arc_from_args(args)
        arc = cal_profile.ARC_PROFILE if arc is None else arc
        result = {"note": args.note, "checks": {}, "verified": False,
                  "arc_profile": arc,
                  "taken": datetime.now().isoformat(timespec="seconds")}
        if not select_arc(stm, arc):
            return 1
        if args.start_from:
            start = cal_profile.load(args.start_from)
            if start is None or not cal_profile.push(stm, start):
                return 1
            if cal_profile.arc_of(start) != arc:
                logging.warning(
                    f"Starting values come from a "
                    f"{cal_profile.ARC_NAMES[cal_profile.arc_of(start)]} "
                    f"calibration; this one is {cal_profile.ARC_NAMES[arc]}. "
                    "Fine as a starting point - every step re-measures.")
        show_values("Starting from", current_values(stm))

        target_cm = default_target(arc)
        for name, step in (("straight", step_straight), ("gyro", step_gyro),
                           ("radius", lambda s, r: step_radius(s, r, target_cm=target_cm)),
                           ("brakes", step_brakes)):
            if name in skip:
                logging.info(f"Skipping step {name}.")
                continue
            if not step(stm, result) or not gyro_ok(stm):
                return 1

        if not snapshot(stm, result):
            return 1
        path = cal_profile.save(args.name, result)
        logging.info(f"Saved to {path} (not yet verified).")

        if "square" not in skip:
            if not step_square(stm, result) or not gyro_ok(stm):
                return 1
            cal_profile.save(args.name, result)

        result["name"] = args.name
        report(result)
        print(f"  Use it: set CAL_PROFILE={args.name} in .env. Tasks restore it,")
        print("  freeze learning and zero the heading at startup.\n")
        return 0 if result.get("verified") else 1

    except KeyboardInterrupt:
        print()
        logging.warning("Aborting — RST, learning off.")
        stm.abort()
        stm.set_learning(False)
        return 130
    finally:
        stm.disconnect()


if __name__ == "__main__":
    sys.exit(main())
