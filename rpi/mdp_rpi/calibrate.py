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
    python3 calibrate.py radius arena        # step 3: match left/right to 29.1 cm
    python3 calibrate.py radius arena --target auto   # ...or find the radius
    python3 calibrate.py brakes arena        # step 4: braking (also: new battery)
    python3 calibrate.py restore arena
    python3 calibrate.py list
    python3 calibrate.py show arena
    python3 calibrate.py zero                # heading 0 here, after moving it by hand
    python3 calibrate.py drift               # is the gyro drifting at rest?

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
  3. RADIUS    FR90 then RR90 back along the same circle, then FL90/RL90, from
                marks under the rear axle; you measure how far the axle moved.
                That chord is the turn radius, per side, forward and reverse,
                in about 50 cm x 50 cm. The planner uses reverse arcs too.
                The left and right steering are adjusted until both match the
                29.1 cm the planner and the A5 orbit assume. The same pulse
                either side of centre does not give the same wheel angle, so
                without this a left turn and a right turn land the robot in
                different places even when both stop at exactly 90 degrees.
  4. BRAKES     Eight arcs with learning on, then the AVERAGE decel and lag are
                pushed back and frozen. Then four more arcs with learning off
                to show how precisely each one stops.
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
from communications.stm import (CAL_LIMITS, FROZEN_CAL_MIN_PROTOCOL,
                                 STEER_CAL_MIN_PROTOCOL, STM)

# Step 1. F<n> is centimetres.
STRAIGHT_CM = 100
STRAIGHT_MAX_RUNS = 6
TRIM_SETTLED_US = 2          # trim moving less than this per run = converged
DRIFT_OK_CM = 1.0            # sideways error at 1 m you are prepared to accept
DISTANCE_OK_PCT = 1.0

# Step 2. Four quarter turns make a full circle and bring the robot back to
# where it started, so the reference edge is right there to measure against.
GYRO_LOOPS = [("FR90", 4), ("FL90", 4)]
GYRO_ASYMMETRY_WARN_PCT = 0.5

# Step 3. The radius both sides are matched to: what the planner
# (pc_side/stm_tokens.py TURN_RADIUS_MM) and A5 (TIGHT_RADIUS_CM) assume.
RADIUS_TARGET_CM = 29.1
RADIUS_TOL_CM = 0.5
RADIUS_RUNS = 2              # forward+reverse pairs measured per side per round
RADIUS_MAX_ROUNDS = 4
RADIUS_MAX_STEP_US = 60      # never move one side's steering further per round
# Forward and reverse share one steering pulse per side, so steering cannot
# pull them together. Past this they are worth knowing about: the planner
# uses one radius for both, and a retrace home lands off by the difference.
RADIUS_FWD_REV_WARN_CM = 1.0

# Step 4. Alternating forward/reverse so the heading returns and the robot
# stays on one patch of floor. Needs about 1.5 m x 1.5 m clear.
BRAKE_LEARN_SEQ = ["FR90", "RR90", "FL90", "RL90"] * 2
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


def send_move(stm: STM, token: str) -> bool:
    """One movement line, wait for its reply. Anything but OK stops the run."""
    if not stm.send_line([token]):
        return False
    reply = stm.wait_reply()
    if reply is None:
        logging.error(f"{token}: no reply — lost link.")
        return False
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
    return True


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
    print("  back. At the far end, measure the sideways offset from the line of")
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

            if not send_move(stm, f"R{STRAIGHT_CM}"):
                return False
            # The reverse also feeds the trim; read it so the next run's
            # "moved" compares like with like.
            prev_trim = current_values(stm)["trim_us"]

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

def _chord_radius(stm: STM, token: str, prompt: str) -> Optional[float]:
    """
    Drive one arc and turn the measured chord into a radius. For a turn of
    theta the chord is 2 R sin(theta/2), so R = chord / (2 sin(theta/2)) -
    using the angle the gyro says it actually turned, not the 90 asked for.
    """
    if not send_move(stm, token):
        return None
    hdg = stm.read_heading()
    if hdg is None:
        return None
    turned = abs(hdg["last_turned"])
    chord = ask_float(prompt, allow_blank=False)
    r = chord / (2.0 * math.sin(math.radians(turned) / 2.0))
    logging.info(f"  {token}: turned {turned:.1f}°, chord {chord:.1f} cm "
                 f"→ radius {r:.1f} cm")
    return r


def measure_side(stm: STM, side: str, runs: int) -> Optional[dict]:
    """
    One side's radius, forward AND reverse, from the floor. Each pair is a
    forward arc out (FR90) and the reverse arc on the same steering back
    (RR90). Reverse with the same wheel angle runs back along the same
    circle, so the robot only ever uses one quarter circle of floor and ends
    near where it started - no repositioning between pairs.

    Two marks per pair: A under the rear axle before, B after the forward
    arc. Forward radius from A->axle, reverse radius from B->axle. The gap
    to A at the end is the retrace error, optional.
    """
    fwd, rev, home = f"F{side}90", f"R{side}90", []
    out = {"fwd": [], "rev": [], "home": home}
    way = "right" if side == "R" else "left"
    if not position_and_zero(stm, f"{fwd}/{rev}: robot with about 50 cm clear "
                             f"ahead and to the {way}."):
        return None
    for i in range(1, runs + 1):
        pause(f"{fwd} {i}/{runs}: mark A on the floor under the CENTRE of the "
              "rear axle.")
        r = _chord_radius(stm, fwd, "Straight-line distance from mark A to the "
                                    "rear axle centre now, cm:")
        if r is None:
            return None
        out["fwd"].append(r)

        pause(f"{rev} {i}/{runs}: mark B under the rear axle centre. It will "
              "reverse back towards A.")
        r = _chord_radius(stm, rev, "Straight-line distance from mark B to the "
                                    "rear axle centre now, cm:")
        if r is None:
            return None
        out["rev"].append(r)

        gap = ask_float("How far is the axle centre from mark A now? cm "
                        "(Enter to skip):")
        if gap is not None:
            home.append(gap)
    return out


def step_radius(stm: STM, result: dict,
                target_cm: Optional[float] = RADIUS_TARGET_CM) -> bool:
    """
    target_cm None = find the radius instead of assuming one: the first round
    runs both sides at full lock, and the wider of the two - the tightest
    circle both sides can trace - becomes the target the other is matched to.
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
    print("  Each side turns forward 90 then reverses back along the")
    print("  same circle, so it needs about 50 cm x 50 cm. Before each arc, mark")
    print("  the floor under the centre of the rear axle; after it, measure")
    print("  straight from that mark to the axle centre. A tape across the gap")
    print("  is enough - it is a straight line.")

    proto = cal_profile.firmware_protocol(stm)
    if proto is None or proto < STEER_CAL_MIN_PROTOCOL:
        logging.error(f"Per-side steering needs protocol {STEER_CAL_MIN_PROTOCOL} "
                      "firmware. Flash the current build.")
        return False

    cal = current_values(stm)
    if cal is None:
        return False
    steer = {"L": cal["steer_left_us"], "R": cal["steer_right_us"]}
    limit = {"L": CAL_LIMITS["SL"][1], "R": CAL_LIMITS["SR"][1]}
    floor_us = CAL_LIMITS["SL"][0]
    history = []
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
            m = measure_side(stm, side, RADIUS_RUNS)
            if m is None:
                return False
            f, r = _mean(m["fwd"]), _mean(m["rev"])
            got[side] = (f + r) / 2.0
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

        # Smaller wheel angle, wider circle: R is roughly inversely
        # proportional to the deflection over this range, so scale each
        # side by measured/target and let the next round correct the rest.
        capped = []
        for s_ in "LR":
            if abs(got[s_] - target_cm) <= RADIUS_TOL_CM:
                continue
            want = steer[s_] * got[s_] / target_cm
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
    if abs(target_cm - RADIUS_TARGET_CM) > 1e-6:
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
    print("\n  ── STEP 4: BRAKES ─────────────────────────────────────────────")
    print(f"  {len(BRAKE_LEARN_SEQ)} arcs with learning on, then the averages are")
    print(f"  frozen and {len(BRAKE_CHECK_SEQ)} more arcs show how precisely they stop.")
    print("  Needs about 1.5 m x 1.5 m clear.")

    if not position_and_zero(stm, "Robot in the middle of the clear area."):
        return False

    if not stm.set_learning(True):
        return False
    try:
        out = run_arcs(stm, BRAKE_LEARN_SEQ, sample_cal=True)
    finally:
        stm.set_learning(False)
    if out is None:
        return False
    samples, _ = out

    # The learner scatters around the answer rather than converging to it
    # (decel ~3%, lag ~26% spread), so the last reading is just the last
    # sample. The mean of all of them is the estimate.
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


STEP_ORDER = ("straight", "gyro", "radius", "brakes")

# What re-running a step leaves stale. The trim only enters the straight-line
# heading loop (odom.c) - arcs set the servo from centre +/- the per-side
# deflection without it - so a new trim touches nothing else. The radius
# chord uses the gyro's angle and braking is learned in gyro degrees, so a
# new gyro scale stales both; new steering changes how an arc coasts.
STALE_AFTER = {"straight": (), "gyro": ("radius", "brakes"),
               "radius": ("brakes",), "brakes": ()}


def run_one_step(stm: STM, name: str, step: str, note: Optional[str] = None,
                 target_cm: Optional[float] = RADIUS_TARGET_CM) -> bool:
    """
    One guided step on its own, saved into profile `name` without touching
    the other steps' results. A missing profile is started from the robot's
    current values, so a profile can be built one step at a time.
    """
    steps = {"straight": step_straight, "gyro": step_gyro,
             "radius": lambda s, r: step_radius(s, r, target_cm=target_cm),
             "brakes": step_brakes}

    if os.path.exists(cal_profile.profile_path(name)):
        p = cal_profile.load(name)
        if p is None or not cal_profile.push(stm, p):
            return False
    else:
        logging.info(f"No profile {name!r} yet — starting it from the robot's "
                     "current values.")
        p = {"note": "", "verified": False}
    p.setdefault("checks", {})
    if note is not None:
        p["note"] = note

    if not steps[step](stm, p) or not snapshot(stm, p):
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
    print(f"  profile  : {p.get('name')}   ({'VERIFIED' if p.get('verified') else 'not verified'})")
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

    for step, help_ in (("straight", "step 1 only: steering trim, keep the rest"),
                        ("gyro", "step 2 only: gyro scale, keep the rest"),
                        ("radius", "step 3 only: re-match left/right turn radius"),
                        ("brakes", "step 4 only: braking, keep trim, gyro, radius")):
        one = sub.add_parser(step, help=help_)
        one.add_argument("name")
        one.add_argument("--note", default=None)
        if step == "radius":
            one.add_argument("--target", default=str(RADIUS_TARGET_CM),
                             help="radius both sides are matched to, cm "
                                  f"(default {RADIUS_TARGET_CM}), or 'auto' to "
                                  "find the tightest both sides can turn")

    v = sub.add_parser("verify", help="restore a profile and run the square test")
    v.add_argument("name")
    v.add_argument("--side", type=int, default=SQUARE_SIDE_CM)

    dr = sub.add_parser("drift", help="diagnostic: gyro drift at rest, before and "
                                      "after handling the robot")
    dr.add_argument("--seconds", type=int, default=DRIFT_SECONDS)

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

        if args.cmd == "restore":
            p = cal_profile.load(args.name)
            if p is None or not cal_profile.push(stm, p):
                return 1
            logging.info(f"Restored {args.name!r}, learning off.")
            return 0

        if args.cmd == "verify":
            p = cal_profile.load(args.name)
            if p is None or not cal_profile.push(stm, p):
                return 1
            p.setdefault("checks", {})
            if not step_square(stm, p, side_cm=args.side):
                return 1
            cal_profile.save(args.name, p)
            report(p)
            return 0 if p["verified"] else 1

        if args.cmd in STEP_ORDER:
            target = getattr(args, "target", str(RADIUS_TARGET_CM))
            try:
                target_cm = None if target.lower() == "auto" else float(target)
            except ValueError:
                logging.error(f"--target {target!r}: a number of cm, or 'auto'.")
                return 1
            ok = run_one_step(stm, args.name, args.cmd, note=args.note,
                              target_cm=target_cm)
            return 0 if ok else 1

        # guided
        skip = {x.strip() for x in args.skip.split(",") if x.strip()}
        result = {"note": args.note, "checks": {}, "verified": False,
                  "taken": datetime.now().isoformat(timespec="seconds")}
        if args.start_from:
            start = cal_profile.load(args.start_from)
            if start is None or not cal_profile.push(stm, start):
                return 1
        show_values("Starting from", current_values(stm))

        for name, step in (("straight", step_straight), ("gyro", step_gyro),
                           ("radius", step_radius), ("brakes", step_brakes)):
            if name in skip:
                logging.info(f"Skipping step {name}.")
                continue
            if not step(stm, result):
                return 1

        if not snapshot(stm, result):
            return 1
        path = cal_profile.save(args.name, result)
        logging.info(f"Saved to {path} (not yet verified).")

        if "square" not in skip:
            if not step_square(stm, result):
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
