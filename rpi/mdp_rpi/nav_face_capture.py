"""
nav_face_capture.py  —  Pi side of the STM's NAV mode (OLED mode 10)
─────────────────────────────────────────────────────────────────────
Run on the RPi with the STM32 connected, then on the robot long-press to
mode 10 and short-press to start:

    python3 nav_face_capture.py

This is the one exchange on the link where the STM asks and the Pi answers:

    STM → RPi :  SNAP,<n>\\n       robot is 10 cm from a face, take photo n
    RPi → STM :  !SNAPOK<n>\\n     photo n is taken — the STM replies OK

The STM holds still until the ack arrives, re-asks every 2 s, and gives up
after 20 s (OLED shows NO PI ACK). So:

  * Take ONE photo per id. A repeated SNAP,<n> is the STM re-asking while we
    were still busy capturing, not a request for a second photo — answer it
    with the ack again and nothing else.
  * Do NOT ack a failed capture. Silence makes the STM re-ask, which is a free
    retry; an ack would send the robot on to the next face with no photo.

Photos are saved to captured_images/face_<n>_<time>.jpg.

Standalone on purpose: it opens the serial port itself, so do not run it at
the same time as task1.py or test_stm.py.
"""

import logging
import os
import re
import time

import serial
from dotenv import load_dotenv

from image_capture.camera import Camera

load_dotenv()

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s - %(message)s")

_SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyACM0")
_BAUD_RATE = int(os.getenv("BAUD_RATE", "115200"))

SAVE_DIR = "captured_images"

_SNAP_RE = re.compile(r"^SNAP,(\d+)$", re.IGNORECASE)


def send_ack(port: serial.Serial, snap_id: int) -> None:
    port.write(f"!SNAPOK{snap_id}\n".encode("utf-8"))
    logging.info("→ stm: !SNAPOK%d", snap_id)


def save_photo(image_bytes: bytes, snap_id: int) -> str:
    os.makedirs(SAVE_DIR, exist_ok=True)
    path = os.path.join(SAVE_DIR, f"face_{snap_id}_{time.strftime('%H%M%S')}.jpg")
    with open(path, "wb") as f:
        f.write(image_bytes)
    return path


def main() -> None:
    # Camera first: it needs ~2 s to warm up, and the STM's 20 s window starts
    # the moment it sends SNAP, not when this script gets round to it.
    camera = Camera()

    # Short timeout so Ctrl+C is responsive; an empty read just loops.
    port = serial.Serial(_SERIAL_PORT, _BAUD_RATE, timeout=0.5)
    logging.info("Listening on %s @ %d. Start NAV (mode 10) on the robot.",
                 _SERIAL_PORT, _BAUD_RATE)

    last_done = None

    try:
        while True:
            raw = port.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            m = _SNAP_RE.match(line)
            if not m:
                # Banner, run log, or the STM's OK to our ack.
                logging.info("stm: %s", line)
                continue

            snap_id = int(m.group(1))
            logging.info("← stm: SNAP,%d", snap_id)

            if snap_id == last_done:
                send_ack(port, snap_id)
                continue

            image_bytes = camera.capture_image()
            if not image_bytes:
                logging.error("Capture for SNAP,%d failed — not acking, the STM "
                              "will re-ask.", snap_id)
                continue

            logging.info("Saved %s", save_photo(image_bytes, snap_id))
            last_done = snap_id
            send_ack(port, snap_id)

    except KeyboardInterrupt:
        logging.info("Stopped by user.")
    finally:
        port.close()
        camera.stop_camera()


if __name__ == "__main__":
    main()
