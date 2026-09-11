"""
test_stm.py  —  Interactive STM command sender for debugging
─────────────────────────────────────────────────────────────
Run on the RPi with the STM32 physically connected:

    python3 test_stm.py            # interactive prompt
    python3 test_stm.py --bringup  # walk PROTOCOL.md §9 stages 3-5 (no motion)

Type whole lines the way the firmware expects them:

    F50                 forward 50 cm
    FR90,F20,S          arc right 90, forward 20, stop  -> ONE OK at the end
    ?US                 query front distance            -> "US,42", no OK
    !PROF1              select the CLEAN arc profile    -> OK
    RST                 emergency abort                 -> NO reply, never waits

    q                   quit

Why this waits instead of sleeping
──────────────────────────────────
The reply comes when the MOVE FINISHES, which is seconds, not milliseconds.
An earlier version of this tool slept 0.3s and then did a non-blocking read, so
every real movement command looked like it had failed. It now blocks on
wait_reply() with the §10 read timeout (20s, comfortably above the firmware's
15s watchdog).
"""

import argparse
import logging
import sys

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

from communications.stm import STM


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
        print("  (no reply — lost link? see PROTOCOL.md §10)")
        return

    print(f"  {reply}")
    if reply.upper().startswith("FAIL"):
        print("  ^ the robot is NOT where you think it is — stop and re-plan.")


def bringup(stm: STM) -> None:
    """
    PROTOCOL.md §9 stages 3-5. No motion at all — these prove the link.
    If these pass and a later F10 fails, the problem is motion, not the link.
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

    print("\nStages 3-5 done. If all three passed, the link is good.")
    print("Next, by hand: F10 (stage 6), then FR90,F20,S (stage 7 — exactly ONE OK).\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive STM32 command sender.")
    parser.add_argument(
        "--bringup",
        action="store_true",
        help="run PROTOCOL.md §9 stages 3-5 (no motion), then exit",
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

    if args.bringup:
        bringup(stm)
        stm.disconnect()
        return

    print("\nSTM connected. Enter a command line (q to quit).")
    print("Examples: F50 | FR90,F20,S | ?US | !PROF1 | RST\n")

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
