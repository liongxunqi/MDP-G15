"""
test_stm.py  —  Interactive STM command sender for debugging
─────────────────────────────────────────────────────────────
Run on the RPi with the STM32 physically connected:

    python3 test_stm.py            # interactive prompt
    python3 test_stm.py --bringup  # walk PROTOCOL.md §10 stages 3-5 (no motion)

Type whole lines the way the firmware expects them:

    F50                 forward 50 cm (odometry only — IGNORES the ultrasound)
    FU30                forward until the ultrasound reads 30 cm  -> OK
    F150,FU20           travel fast, then close the last bit precisely
    FR90,F20,S          arc right 90, forward 20, stop  -> ONE OK at the end
    ?US                 query front distance            -> "US,42", no OK
    !PROF1              select the CLEAN arc profile    -> OK
    RST                 emergency abort                 -> NO reply, never waits

    ?VER                identity + protocol version     -> "VER,MDPG15-STM32,3"
    ?CAL                learned decel, lag, trim        -> "CAL,1234,180,-3"
    !CALT-3             restore steering trim, us       -> OK  (signed!)

    q                   quit

FU<n> is protocol 3 and behaves unlike anything else here (§4.1). It is not a
distance cap: it closes whatever gap it measures, so FU30 with the wall 180 cm
out drives 150 cm. It needs something to see — nothing in range answers
FAIL,NOECHO and the robot does not move. And it is slow on purpose, standing
still for 400 ms between passes: about 1.2 s for a short approach, 4 s from a
metre, 8.6 s worst case. Do not mistake that silence for a dead link.

Calibration (PROTOCOL.md §7) is a script you send, not a firmware mode. The
sequence that converges it:

    !PROF1 / F600 / FR90 / RR90 / FR90 / RR90, with ?CAL after each —
    stop when decel and lag stop moving.

A setter answering RESEND while the robot is moving means BUSY, not malformed:
wait for ?STAT to report busy 0 and send the same bytes again.

Why this waits instead of sleeping
──────────────────────────────────
The reply comes when the MOVE FINISHES, which is seconds, not milliseconds.
An earlier version of this tool slept 0.3s and then did a non-blocking read, so
every real movement command looked like it had failed. It now blocks on
wait_reply() with the §11 read timeout (20s, comfortably above the firmware's
15s watchdog).
"""

import argparse
import logging
import sys

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

from communications.stm import STM

DEFAULT_ARC_PROFILE = 1


def send_and_report(stm: STM, line: str) -> None:
    """Send one line and print whatever comes back, honouring the RST exception."""
    tokens = [t for t in line.replace(" ", ",").split(",") if t]

    # PROTOCOL.md §4: RST is the sole command that replies nothing at all.
    if len(tokens) == 1 and tokens[0].upper() == "RST":
        stm.abort()
        print("  RST sent — no reply expected (PROTOCOL.md §4).")
        return

    # PROTOCOL.md §5: queries bypass the movement queue; the reply IS the data.
    if len(tokens) == 1 and tokens[0].startswith("?"):
        reply = stm.query(tokens[0])
        print(f"  {reply}" if reply else "  (no reply — query timed out)")
        return

    if not stm.send_line(tokens):
        return  # validation already logged the reason; nothing was sent

    reply = stm.wait_reply()
    if reply is None:
        print("  (no reply — lost link? see PROTOCOL.md §11)")
        return

    print(f"  {reply}")
    if reply.upper().startswith("FAIL"):
        # NOECHO is the exception to the usual advice. FU refuses to drive at
        # something it cannot see, so nothing moved and the pose is intact —
        # telling the operator to re-plan would send them after the wrong
        # problem entirely.
        if reply.strip().upper().endswith("NOECHO"):
            print("  ^ FU had no usable ultrasound reading. NOTHING MOVED, the")
            print("    pose is unchanged, and the rest of the line was dropped.")
            print("    Check the wiring and that something is actually in front;")
            print("    ?US should answer with a distance, not 65535.")
        else:
            print("  ^ the robot is NOT where you think it is — stop and re-plan.")


def bringup(stm: STM) -> None:
    """
    PROTOCOL.md §10 stages 3-5, plus stage 9. No motion at all — these prove
    the link and the sensor. If they pass and a later F10 fails, the problem is
    motion, not the link.

    Stage 9 is here rather than left to the motion stages because it is the one
    thing worth knowing BEFORE trusting FU with the robot: FU navigates on what
    the ultrasound tells it, and a sensor that answers 65535 will simply refuse
    to move. Cheaper to find out standing still.
    """
    print("\n-- Stage 3: send S (expect OK, nothing moves) --")
    send_and_report(stm, "S")

    print("\n-- Stage 4: send XYZ (expect RESEND, nothing moves) --")
    # Deliberately bypasses the local validator: the point of this stage is to
    # prove the FIRMWARE rejects garbage, so the garbage has to reach the wire.
    stm.send("XYZ")
    reply = stm.wait_reply()
    print(f"  {reply}" if reply else "  (no reply)")

    print("\n-- Stage 5: send ?VER (expect VER,<name>,<proto>) --")
    send_and_report(stm, "?VER")

    print("\n-- Stage 9: send ?US, box ~50 cm ahead (expect US,~50) --")
    print("   65535 means NO READING — fix that before sending any FU.")
    send_and_report(stm, "?US")

    print("\nStages 3-5 and 9 done. If they passed, the link and the sensor are good.")
    print("Next, by hand and with the floor clear:")
    print("  stage 6   F10                  moves ~10 cm, OK AFTER it stops")
    print("  stage 7   FR90,F20,S           exactly ONE OK, at the end")
    print("  stage 10  FU20, box ~1 m out   OK after ~4 s, then ?US reads 18-22")
    print("  stage 11  FU20, sensor unplugged   FAIL,NOECHO, nothing moves\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive STM32 command sender.")
    parser.add_argument(
        "--bringup",
        action="store_true",
        help="run PROTOCOL.md §10 stages 3-5 and 9 (no motion), then exit",
    )
    parser.add_argument(
        "--profile",
        type=int,
        choices=(0, 1, 2),
        default=DEFAULT_ARC_PROFILE,
        help="select the STM arc profile on startup (default: 1 CLEAN)",
    )
    args = parser.parse_args()

    stm = STM()
    try:
        stm.connect()
    except Exception as exc:
        logging.error(f"Could not open the serial port: {exc}")
        logging.error(
            "Check SERIAL_PORT in .env. Note PROTOCOL.md §1: USB Port 2 is the "
            "RPi link (USART3). USB Port 1 is UART1, for code download only — "
            "the firmware never transmits there."
        )
        sys.exit(1)

    if not stm.set_profile(args.profile):
        logging.error("Could not select STM arc profile %d.", args.profile)
        stm.disconnect()
        sys.exit(1)
    logging.info("STM arc profile set to %d.", args.profile)

    if args.bringup:
        bringup(stm)
        stm.disconnect()
        return

    print("\nSTM connected. Enter a command line (q to quit).")
    print("Examples: F50 | FU30 | F150,FU20 | FR90,F20,S | ?US | ?CAL | RST")
    print("FU waits while it settles between passes — up to ~8.6 s. That is normal.\n")

    try:
        while True:
            try:
                line = input("CMD> ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line.lower() == "q":
                break
            send_and_report(stm, line)
    except KeyboardInterrupt:
        print()
    finally:
        stm.disconnect()
        logging.info("Done.")


if __name__ == "__main__":
    main()
