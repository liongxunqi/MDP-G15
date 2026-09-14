"""
calibrate.py  —  Run the PROTOCOL.md §7 calibration sequence and save the result
────────────────────────────────────────────────────────────────────────────────
The firmware learns three things while it drives — arc deceleration, brake
engagement lag and steering trim — and loses all three at power-off. A cold
robot spends its first three or four arcs converging, which is the difference
between a 9 degree error and a 1 degree one on the first turn of a session.

This runs the sequence, averages the readings, checks the robot against itself,
and writes the answer to a named profile you can restore in one command.

    python3 calibrate.py run arena --note "smooth floor, full battery, no load"
    python3 calibrate.py restore arena
    python3 calibrate.py list
    python3 calibrate.py show arena

Profiles live in cal_profiles/<name>.json.

WHY A NAMED PROFILE PER ENVIRONMENT
───────────────────────────────────
The numbers do not transfer. They are a property of the surface, the battery
charge and the weight on the chassis, not of the robot. A profile taken on
the lab bench will be wrong in the arena, and one taken on a full battery will
be wrong at the end of a run. Name them after the conditions, not the day.

WHY THE MEAN AND NOT THE LAST READING
─────────────────────────────────────
The learner does not converge to a point — it scatters around one. Measured
over six arcs: decel held a spread of about 3% and lag about 26%, with no
trend in either. Taking the final reading therefore bakes in whichever sample
happened to be last, and the noisiest sample is as likely as any other. The
mean of several arcs is the estimate; the spread tells you how much to trust
it, which is why both are reported and stored.

THE FIRST ARC IS BURN-IN
────────────────────────
Arc 1 is one update away from the compile-time seed and still carries it, so
it is discarded by default. --keep-first includes it.
"""

import argparse
import json
import logging
import os
import sys
from datetime import datetime
from typing import List, Optional, Tuple

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                    datefmt="%H:%M:%S")

from communications.stm import STM, CAL_LIMITS, PROTOCOL_VERSION

PROFILE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cal_profiles")


def _mean(vals) -> float:
    """
    Arithmetic mean, computed here rather than with statistics.fmean().

    fmean() is Python 3.8+, and the Pi runs older than that. This is the only
    thing that needed it, so a two-line helper is cheaper than a version
    requirement on a robot that is awkward to upgrade mid-project.
    """
    return sum(vals) / float(len(vals))


def _stdev(vals) -> float:
    """Sample standard deviation. Zero for a single sample rather than raising."""
    if len(vals) < 2:
        return 0.0
    m = _mean(vals)
    return (sum((v - m) ** 2 for v in vals) / (len(vals) - 1)) ** 0.5

# PROTOCOL.md §7. NOTE the straight is F50, not the F600 the document prints:
# F<n> is CENTIMETRES (§4), so F600 is six metres and does not fit a 2.0m arena.
# What the trim learner actually needs is 50 samples at the 10ms heading tick,
# i.e. half a second of straight driving, which F50 covers.
STRAIGHT_CM = 50
ARC_DEG = 90

# Alternating so the robot returns to its start heading and stays on the same
# patch of floor. At TIGHT (291mm radius) each 90 degree turn eats roughly
# 291mm forward AND 291mm sideways, so this needs about 1.5m x 1m clear.
ARC_SEQUENCE = ["FR", "RR", "FR", "RR", "FR", "RR"]


# ── Wire helpers ──────────────────────────────────────────────────────────────

def send_move(stm: STM, token: str) -> bool:
    """
    Send one movement token and wait for its reply. Returns False on anything
    that is not OK.

    PROTOCOL.md §3: FAIL,* means the robot is not where you think it is. There
    is no point averaging readings taken after a stalled wheel, so this stops
    the run rather than carrying on and producing a confident wrong answer.
    """
    if not stm.send_line([token]):
        return False

    reply = stm.wait_reply()
    if reply is None:
        logging.error(f"{token}: no reply — lost link (PROTOCOL.md §11).")
        return False

    verdict = reply.strip().upper()
    if verdict == "OK":
        return True

    if verdict.startswith("FAIL"):
        logging.error(
            f"{token}: {verdict}. The robot is not where you think it is — "
            "stopping. Check for a stalled wheel or a dead encoder before "
            "trusting anything measured after this point."
        )
    else:
        logging.error(f"{token}: unexpected reply {reply!r}.")
    return False


def check_version(stm: STM) -> bool:
    ver = stm.query("?VER")
    if ver is None:
        logging.error(
            "No answer to ?VER. The link may be dead, or the board may be on "
            "USB Port 1 (UART1, download only — the firmware never transmits "
            "there). See PROTOCOL.md §1."
        )
        return False

    logging.info(f"STM: {ver}")
    fields = ver.split(",")
    proto = fields[2].strip() if len(fields) >= 3 else ""
    if proto != str(PROTOCOL_VERSION):
        logging.error(
            f"Firmware reports protocol {proto!r}, this tool needs "
            f"{PROTOCOL_VERSION}. Calibration read/write does not exist before "
            "protocol 2 — flash a v2 build first."
        )
        return False
    return True


# ── The run ───────────────────────────────────────────────────────────────────

def run(stm: STM, arcs: int, profile: int, keep_first: bool
        ) -> Optional[Tuple[dict, List[Tuple[int, int, int]]]]:
    """Drive the sequence, sampling ?CAL after every arc."""
    if not check_version(stm):
        return None

    before = stm.read_cal()
    if before is None:
        logging.error("?CAL did not answer — cannot calibrate.")
        return None
    logging.info(f"Starting values: decel={before[0]/10:.1f} dps², "
                 f"lag={before[1]/10:.1f} ms, trim={before[2]} µs")

    # Known state. RST or a previous run may have left the queue part-drained
    # and the steering off centre.
    logging.info("Squaring up: S, !ZERO, !PROF%d", profile)
    if not send_move(stm, "S"):
        return None
    if not stm.zero_odometry():
        logging.error("!ZERO refused.")
        return None
    if not stm.set_profile(profile):
        logging.error(f"!PROF{profile} refused.")
        return None

    # The straight feeds the trim learner its 50 samples.
    logging.info(f"Straight: F{STRAIGHT_CM} (trim needs ≥0.5s of driving)")
    if not send_move(stm, f"F{STRAIGHT_CM}"):
        return None

    samples: List[Tuple[int, int, int]] = []
    for i in range(arcs):
        token = f"{ARC_SEQUENCE[i % len(ARC_SEQUENCE)]}{ARC_DEG}"
        logging.info(f"Arc {i + 1}/{arcs}: {token}")
        if not send_move(stm, token):
            return None

        reading = stm.read_cal()
        if reading is None:
            logging.error("?CAL did not answer mid-run.")
            return None

        samples.append(reading)
        logging.info(f"  decel={reading[0]/10:7.1f} dps²  "
                     f"lag={reading[1]/10:5.1f} ms  trim={reading[2]:+d} µs")

    used = samples if keep_first else samples[1:]
    if not used:
        logging.error("No samples left after discarding burn-in — run more arcs.")
        return None

    def summarise(idx: int):
        vals = [s[idx] for s in used]
        return _mean(vals), _stdev(vals), min(vals), max(vals)

    d_mean, d_sd, d_lo, d_hi = summarise(0)
    l_mean, l_sd, l_lo, l_hi = summarise(1)
    t_mean, t_sd, t_lo, t_hi = summarise(2)

    result = {
        "decel_x10": int(round(d_mean)),
        "lag_ms_x10": int(round(l_mean)),
        "trim_us": int(round(t_mean)),
        "spread": {
            "decel_x10": {"sd": round(d_sd, 1), "min": d_lo, "max": d_hi},
            "lag_ms_x10": {"sd": round(l_sd, 1), "min": l_lo, "max": l_hi},
            "trim_us": {"sd": round(t_sd, 1), "min": t_lo, "max": t_hi},
        },
        "arcs_driven": arcs,
        "arcs_averaged": len(used),
        "arc_profile": profile,
        "samples": [list(s) for s in samples],
    }
    return result, samples


def health_checks(stm: STM, result: dict) -> List[str]:
    """
    The two checks from PROTOCOL.md §7 that stop a converged-looking CAL from
    covering for a mechanical fault. If either fails the answer is a spanner,
    not a number — so they are reported loudly rather than folded into the
    saved profile as if they were just more data.
    """
    warnings: List[str] = []

    # 1. Trim near zero. SERVO_CENTER_US carries the measured steering centre,
    #    so trim has nothing left to absorb. One walking away from zero means
    #    the centre itself moved.
    trim = result["trim_us"]
    lo, hi, _, _ = CAL_LIMITS["T"]
    if abs(trim) > 0.5 * hi:
        warnings.append(
            f"Trim is {trim} µs, over half the ±{hi} limit. The steering centre "
            "itself has probably moved — check the linkage rather than trusting "
            "this number."
        )
    elif abs(trim) > 20:
        warnings.append(
            f"Trim is {trim} µs. Not alarming, but it should hover within a few "
            "µs of zero; worth watching across sessions."
        )

    # 2. Cross-check on the CLEAN profile. The encoder-derived angle shares no
    #    hardware with the gyro, so agreement is real evidence. Only meaningful
    #    on profile 1 — the others scrub the tyres, inflating the encoder arc.
    logging.info("Cross-check: !PROF1 then one arc")
    if not stm.set_profile(1):
        warnings.append("Could not select !PROF1 — cross-check skipped.")
        return warnings
    if not send_move(stm, f"FR{ARC_DEG}"):
        warnings.append("Cross-check arc failed — cross-check skipped.")
        return warnings

    fields = stm.query_fields("?XCHK")
    if fields is None or len(fields) < 4:
        warnings.append("?XCHK did not answer — cross-check skipped.")
        return warnings

    enc, gyro, err, ok = (int(f) for f in fields[:4])
    result["xchk"] = {"enc_x10": enc, "gyro_x10": gyro, "err_pct": err, "ok": ok}
    logging.info(f"  encoder={enc/10:.1f}°  gyro={gyro/10:.1f}°  "
                 f"err={err}%  ok={ok}")

    # The gyro field is signed (a right turn reads negative); the firmware
    # compares magnitudes, so opposite signs here are expected, not a fault.
    overshoot = abs(gyro) / 10.0 - ARC_DEG
    logging.info(f"  commanded {ARC_DEG}°, body turned {abs(gyro)/10:.1f}° "
                 f"({overshoot:+.1f}°)")

    if not ok:
        warnings.append(
            f"?XCHK FAILED at {err}%. The encoder and the gyro disagree, which "
            "means one of them is lying. Do not trust this profile — find the "
            "mechanical cause first."
        )
    return warnings


# ── Profile store ─────────────────────────────────────────────────────────────

def save(name: str, result: dict, note: str) -> str:
    os.makedirs(PROFILE_DIR, exist_ok=True)
    path = os.path.join(PROFILE_DIR, f"{name}.json")
    result = dict(result)
    result["name"] = name
    result["note"] = note
    result["taken"] = datetime.now().isoformat(timespec="seconds")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2)
    return path


def load(name: str) -> Optional[dict]:
    path = os.path.join(PROFILE_DIR, f"{name}.json")
    if not os.path.exists(path):
        logging.error(f"No profile named {name!r} in {PROFILE_DIR}.")
        return None
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def restore_commands(p: dict) -> List[str]:
    return [f"!CALD{p['decel_x10']}", f"!CALL{p['lag_ms_x10']}",
            f"!CALT{p['trim_us']}"]


def report(p: dict) -> None:
    print()
    print(f"  profile : {p.get('name')}")
    if p.get("note"):
        print(f"  note    : {p['note']}")
    print(f"  taken   : {p.get('taken')}")
    print(f"  arcs    : {p.get('arcs_averaged')} averaged "
          f"of {p.get('arcs_driven')} driven")
    print()
    sp = p.get("spread", {})
    print(f"  decel   : {p['decel_x10']/10:8.1f} dps²   "
          f"(sd {sp.get('decel_x10', {}).get('sd', '?')})")
    print(f"  lag     : {p['lag_ms_x10']/10:8.1f} ms     "
          f"(sd {sp.get('lag_ms_x10', {}).get('sd', '?')})")
    print(f"  trim    : {p['trim_us']:8d} µs     "
          f"(sd {sp.get('trim_us', {}).get('sd', '?')})")
    if "xchk" in p:
        x = p["xchk"]
        print(f"  xchk    : {x['err_pct']}% "
              f"{'OK' if x['ok'] else 'FAILED'}")
    print()
    print("  Restore with:   python3 calibrate.py restore "
          f"{p.get('name')}")
    print("  Or by hand:     " + "  ".join(restore_commands(p)))
    print()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Run and store the PROTOCOL.md §7 calibration sequence.")
    sub = ap.add_subparsers(dest="cmd")
    sub.required = True

    r = sub.add_parser("run", help="drive the sequence and save a profile")
    r.add_argument("name", help="profile name, e.g. arena-full-battery")
    r.add_argument("--note", default="",
                   help="surface, battery and weight this was taken on")
    r.add_argument("--arcs", type=int, default=6,
                   help="arcs to drive (default 6)")
    r.add_argument("--profile", type=int, default=0, choices=(0, 1, 2),
                   help="arc profile to calibrate for (default 0 TIGHT)")
    r.add_argument("--keep-first", action="store_true",
                   help="include arc 1 in the average (it is burn-in)")
    r.add_argument("--no-xchk", action="store_true",
                   help="skip the cross-check arc")

    s = sub.add_parser("restore", help="send a saved profile to the firmware")
    s.add_argument("name")

    sub.add_parser("list", help="list saved profiles")

    sh = sub.add_parser("show", help="print a saved profile")
    sh.add_argument("name")

    args = ap.parse_args()

    if args.cmd == "list":
        if not os.path.isdir(PROFILE_DIR):
            print("No profiles saved yet.")
            return 0
        names = sorted(f[:-5] for f in os.listdir(PROFILE_DIR)
                       if f.endswith(".json"))
        if not names:
            print("No profiles saved yet.")
            return 0
        for n in names:
            p = load(n)
            if p:
                print(f"  {n:24s}  decel {p['decel_x10']/10:7.1f}  "
                      f"lag {p['lag_ms_x10']/10:5.1f}  trim {p['trim_us']:+d}"
                      f"   {p.get('note', '')}")
        return 0

    if args.cmd == "show":
        p = load(args.name)
        if p is None:
            return 1
        report(p)
        return 0

    stm = STM()
    try:
        stm.connect()
    except Exception as exc:
        logging.error(f"Could not open the serial port: {exc}")
        logging.error("Check SERIAL_PORT in .env, and note PROTOCOL.md §1: USB "
                      "Port 2 is the RPi link. Port 1 is for download only.")
        return 1

    try:
        if args.cmd == "restore":
            p = load(args.name)
            if p is None:
                return 1
            if not check_version(stm):
                return 1
            logging.info(f"Restoring {args.name!r}: "
                         f"{' '.join(restore_commands(p))}")
            # set_cal() sends each value on its own line and retries a RESEND
            # that means BUSY. It must not run while a move is outstanding.
            if not stm.set_cal(decel_x10=p["decel_x10"],
                               lag_ms_x10=p["lag_ms_x10"],
                               trim_us=p["trim_us"]):
                logging.error("Restore failed — see above.")
                return 1
            # Read back rather than trusting three OKs. An OK says the command
            # was accepted, not that the value is what you meant.
            back = stm.read_cal()
            if back != (p["decel_x10"], p["lag_ms_x10"], p["trim_us"]):
                logging.error(f"Read-back mismatch: firmware holds {back}, "
                              f"expected {(p['decel_x10'], p['lag_ms_x10'], p['trim_us'])}.")
                return 1
            logging.info(f"Restored and verified: CAL,{back[0]},{back[1]},{back[2]}")
            return 0

        print()
        print(f"  About to drive {args.arcs} arcs plus a "
              f"{STRAIGHT_CM}cm straight.")
        print("  Clear roughly 1.5m x 1m of floor. Ctrl-C aborts (sends RST).")
        print()
        input("  Enter to start, Ctrl-C to cancel... ")

        out = run(stm, args.arcs, args.profile, args.keep_first)
        if out is None:
            return 1
        result, _ = out

        warnings = [] if args.no_xchk else health_checks(stm, result)

        path = save(args.name, result, args.note)
        result["name"] = args.name
        logging.info(f"Saved to {path}")
        report(result)

        if warnings:
            print("  WARNINGS — calibration must never hide a fault:")
            for w in warnings:
                print(f"    * {w}")
            print()
            return 1
        return 0

    except KeyboardInterrupt:
        print()
        logging.warning("Aborting — sending RST.")
        stm.abort()
        return 130
    finally:
        stm.disconnect()


if __name__ == "__main__":
    sys.exit(main())
