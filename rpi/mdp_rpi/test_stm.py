"""
test_stm.py  —  Interactive STM command sender for debugging
─────────────────────────────────────────────────────────────
Run on the RPi when the STM32 is physically connected:
    python3 test_stm.py

Type 4-character commands (e.g. W050, D100) and press Enter.
Type 'q' to quit.
"""

import logging
logging.basicConfig(level=logging.INFO)

from communications.stm import STM

def main():
    stm = STM()
    stm.connect()
    logging.info("STM connected. Enter 4-char commands (q to quit):")

    while True:
        cmd = input("CMD> ").strip()
        if cmd.lower() == "q":
            break
        if len(cmd) != 4:
            print("Commands must be exactly 4 characters (e.g. W050, D100, A100, S020)")
            continue
        stm.send_command(cmd)
        # Brief wait for reply
        import time; time.sleep(0.3)
        reply = stm.receive()
        if reply:
            print(f"STM reply: {reply}")

    stm.disconnect()
    logging.info("Done.")

if __name__ == "__main__":
    main()
