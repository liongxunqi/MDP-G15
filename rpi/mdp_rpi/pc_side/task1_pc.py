"""
pc_side/task1_pc.py  —  PC algorithm + image-recognition server for Task 1
───────────────────────────────────────────────────────────────────────────
Run this on the PC BEFORE starting task1.py on the RPi.

What it does
─────────────
1. Connects to the RPi's TCP socket server.
2. Waits for OBSTACLES,<json>  →  runs pathfinding  →  sends back PATH,<json>.
3. Enters detection loop:
     RPi sends:  DETECT,<obstacle_id>\n
                 4-byte big-endian image size (struct.pack(">I", size))
                 raw JPEG bytes (exactly <size> bytes)
     PC saves the JPEG, runs YOLO, sends back:
                 OBJECT,<obstacle_id>,<confidence>,<class_id>\n
4. On STITCH,<n>  →  stitches the annotated images side-by-side and saves.

Setup
──────
pip install ultralytics opencv-python

Put your trained weights at:  pc_side/weights/best.pt
Put your detect.py at:        pc_side/image_recognition/detect.py

Edit RPI_IP below to match your RPi's actual IP (check with: hostname -I on RPi).
"""

import json
import logging
import os
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Optional

import cv2

# ── Config — edit these ────────────────────────────────────────────────────────
RPI_IP   = "192.168.20.1"   # RPi's IP on the hotspot — must match RPI_HOST in .env
RPI_PORT = 5000              # must match RPI_PORT in .env

RECEIVED_DIR    = "received_images"   # incoming JPEGs saved here
ANNOTATED_DIR   = "runs/predict"      # YOLO-annotated outputs saved here
STITCHED_OUTPUT = "stitched_result.jpg"

# ── Logging ────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [PC] %(levelname)s — %(message)s",
    datefmt="%H:%M:%S",
)

# ── Detection — uses your detect.py directly ──────────────────────────────────
# detect.py lives at pc_side/image_recognition/detect.py
# It exposes:  detect(image_path) -> (class_name, confidence)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from image_recognition.detect import detect as _detect


def run_detection(image_path: str):
    """
    Wrapper around your detect.py.
    Returns (class_name, confidence) or (None, None) if nothing detected.
    """
    return _detect(image_path)


# ── Pathfinding ────────────────────────────────────────────────────────────────
# Replace compute_path() with your actual algorithm when it's ready.
# Must return a dict with these keys:
#   "segments"     — list of lists of STM commands, one sublist per obstacle
#                    e.g. [ ["W050", "D100"], ["W030"], ["A100", "W020"] ]
#   "obstacle_ids" — obstacle IDs in the order they will be visited
#                    e.g. [2, 1, 3]
#   "dirs"         — optional list of robot direction dicts (can be [])

def compute_path(obstacles: list) -> dict:
    """
    STUB — replace with your pathfinding algorithm.
    Currently returns one dummy segment per obstacle so nothing crashes
    while you integrate the real algorithm.
    """
    logging.warning(
        "compute_path() is using the STUB — replace with your real algorithm."
    )
    sorted_obs = sorted(obstacles, key=lambda o: o.get("id", 0))
    return {
        "segments":     [["W000"] for _ in sorted_obs],
        "obstacle_ids": [o["id"] for o in sorted_obs],
        "dirs":         [],
    }


# ── Image stitching ────────────────────────────────────────────────────────────

def stitch_images(image_paths: list, output_path: str) -> None:
    """Concatenate a list of images horizontally and save."""
    imgs = [cv2.imread(p) for p in image_paths if os.path.exists(p)]
    if not imgs:
        logging.warning("Stitch: no valid images found — skipping.")
        return
    min_h = min(img.shape[0] for img in imgs)
    resized = [
        cv2.resize(img, (int(img.shape[1] * min_h / img.shape[0]), min_h))
        for img in imgs
    ]
    cv2.imwrite(output_path, cv2.hconcat(resized))
    logging.info(f"Stitch: saved → {output_path}.")


# ── Socket helpers ─────────────────────────────────────────────────────────────

class RPiConnection:
    """TCP client that connects to the RPi's socket server."""

    def __init__(self, ip: str, port: int):
        self.ip   = ip
        self.port = port
        self.sock: Optional[socket.socket] = None
        self._rx_buf = ""   # text line buffer

    def connect(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.port))
        logging.info(f"Connected to RPi at {self.ip}:{self.port}.")

    def send(self, message: str) -> None:
        """Send a newline-terminated UTF-8 text message to the RPi."""
        payload = (message.rstrip("\n") + "\n").encode("utf-8")
        self.sock.sendall(payload)
        logging.info(f"→ RPi: {message.strip()}")

    def receive_line(self) -> Optional[str]:
        """
        Blocking — accumulates data until a newline is found, then returns
        one complete line. Returns None if the connection is closed.
        """
        while "\n" not in self._rx_buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                return None
            self._rx_buf += chunk.decode("utf-8")
        line, self._rx_buf = self._rx_buf.split("\n", 1)
        msg = line.strip()
        logging.info(f"← RPi: {msg}")
        return msg

    def _recv_exact(self, num_bytes: int) -> bytes:
        """
        Read exactly num_bytes from the socket.
        Necessary because TCP is a stream — recv() can return fewer bytes
        than requested even if more are on the way.
        Raises ConnectionError if the connection closes mid-read.
        """
        data = b""
        while len(data) < num_bytes:
            chunk = self.sock.recv(num_bytes - len(data))
            if not chunk:
                raise ConnectionError(
                    f"Connection closed after {len(data)}/{num_bytes} bytes."
                )
            data += chunk
        return data

    def receive_image(self, save_path: str) -> bool:
        """
        Receive one image using the length-prefix protocol (matches pc.py send_image):
          1. Read 4 bytes → big-endian uint32 = image size in bytes.
          2. Read exactly that many bytes → raw JPEG data.
          3. Save to save_path.

        Any leftover bytes in the text buffer from the previous receive_line()
        call are used first — the RPi sends the header immediately after the
        DETECT text line with no gap between them.

        Returns True on success, False on any error.
        """
        os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
        try:
            # Drain any leftover bytes from the text receive buffer.
            # Use latin-1 so every byte value round-trips without corruption.
            leftover = self._rx_buf.encode("latin-1") if self._rx_buf else b""
            self._rx_buf = ""  # now in binary mode

            # ── Step 1: 4-byte size header ────────────────────────────────────
            header_buf = leftover
            while len(header_buf) < 4:
                chunk = self.sock.recv(4 - len(header_buf))
                if not chunk:
                    raise ConnectionError("Connection closed reading size header.")
                header_buf += chunk

            image_size = struct.unpack(">I", header_buf[:4])[0]
            # Bytes beyond the 4-byte header are the start of the image payload
            payload = header_buf[4:]
            logging.info(f"Receiving image: {image_size} bytes.")

            # ── Step 2: image payload ─────────────────────────────────────────
            remaining = image_size - len(payload)
            if remaining > 0:
                payload += self._recv_exact(remaining)

            # ── Step 3: save ──────────────────────────────────────────────────
            with open(save_path, "wb") as fh:
                fh.write(payload)
            logging.info(f"Image saved → {save_path}.")
            return True

        except Exception as exc:
            logging.error(f"receive_image failed: {exc}")
            return False

    def close(self) -> None:
        if self.sock:
            self.sock.close()
            self.sock = None


# ── Main loop ──────────────────────────────────────────────────────────────────

def main() -> None:
    # Run relative to this file so all relative paths work correctly
    os.chdir(Path(__file__).parent)
    os.makedirs(RECEIVED_DIR, exist_ok=True)

    rpi = RPiConnection(RPI_IP, RPI_PORT)
    rpi.connect()

    annotated_images = []

    logging.info("Waiting for messages from RPi…")

    while True:
        msg = rpi.receive_line()
        if msg is None:
            logging.info("RPi disconnected — exiting.")
            break

        # ── OBSTACLES → pathfinding → PATH ────────────────────────────────────
        if msg.startswith("OBSTACLES"):
            try:
                obstacles = json.loads(msg.split("OBSTACLES,", 1)[1])
            except (IndexError, json.JSONDecodeError) as exc:
                logging.error(f"Bad OBSTACLES message: {exc}")
                continue

            logging.info(f"{len(obstacles)} obstacle(s) received — computing path…")
            path = compute_path(obstacles)
            rpi.send("PATH," + json.dumps(path))

        # ── DETECT → receive image → YOLO → OBJECT ────────────────────────────
        elif msg.startswith("DETECT"):
            parts = msg.split(",")
            obstacle_id = parts[1].strip() if len(parts) > 1 else "0"

            # Save path — named by obstacle ID + timestamp so files don't collide
            filename = f"obstacle_{obstacle_id}_{int(time.time())}.jpg"
            save_path = os.path.join(RECEIVED_DIR, filename)

            # Receive the JPEG (4-byte length header + raw bytes)
            ok = rpi.receive_image(save_path)
            if not ok:
                logging.error(f"Image receive failed for obstacle {obstacle_id}.")
                rpi.send(f"OBJECT,{obstacle_id},0.0,NONE")
                continue

            # Run YOLO via your detect.py
            class_id, confidence = run_detection(save_path)

            if class_id is None:
                logging.warning(f"Nothing detected for obstacle {obstacle_id}.")
                rpi.send(f"OBJECT,{obstacle_id},0.0,NONE")
            else:
                logging.info(
                    f"Detected '{class_id}' (conf={confidence:.2f}) "
                    f"for obstacle {obstacle_id}."
                )
                rpi.send(f"OBJECT,{obstacle_id},{confidence:.4f},{class_id}")

                # Track for stitching
                annotated_path = os.path.join(
                    ANNOTATED_DIR, os.path.basename(save_path)
                )
                annotated_images.append(annotated_path)

        # ── STITCH → combine all result images ────────────────────────────────
        elif msg.startswith("STITCH"):
            logging.info("STITCH received — creating result image…")
            stitch_images(annotated_images, STITCHED_OUTPUT)
            annotated_images.clear()
            logging.info("Stitching done. Ready for next run (Ctrl-C to quit).")

        else:
            logging.warning(f"Unrecognised message from RPi: '{msg}'.")

    rpi.close()


if __name__ == "__main__":
    main()