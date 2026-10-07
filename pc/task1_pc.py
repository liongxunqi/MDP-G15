"""
pc/task1_pc.py  —  PC algorithm + image-recognition server for Task 1
──────────────────────────────────────────────────────────────────────
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

Put your trained weights at:  pc/weights/best.pt
Put your detect.py at:        pc/image_recognition/detect.py
The path planner lives in algo/; run_log.py in rpi/mdp_rpi/.

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
RPI_IP   = "192.168.15.15"   # RPi's IP on the hotspot — must match RPI_HOST in .env
RPI_PORT = 5000              # must match RPI_PORT in .env

RECEIVED_DIR    = "received_images"   # incoming JPEGs saved here
ANNOTATED_DIR   = "runs/predict"      # YOLO-annotated outputs saved here
STITCHED_OUTPUT = "stitched_result.jpg"

# ── Logging ────────────────────────────────────────────────────────────────────
# Shared with the RPi side, so it lives in rpi/mdp_rpi/. Logs go to pc/logs/.
PC_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(PC_DIR)
sys.path.insert(0, os.path.join(REPO_ROOT, "rpi", "mdp_rpi"))
import run_log  # noqa: E402

run_log.setup("task1_pc", "%(asctime)s [PC] %(levelname)s — %(message)s",
              log_dir=os.path.join(PC_DIR, "logs"))

# ── Detection — uses your detect.py directly ──────────────────────────────────
# detect.py lives at pc/image_recognition/detect.py
# It exposes:  detect(image_path) -> (class_name, confidence)
sys.path.insert(0, PC_DIR)
from image_recognition.detect import detect as _detect
from progress_monitor import PlanMonitor


def run_detection(image_path: str):
    """
    Wrapper around your detect.py.
    Returns (class_name, confidence) or (None, None) if nothing detected.
    """
    return _detect(image_path)


# ── Pathfinding ────────────────────────────────────────────────────────────────
# compute_path() returns:
#   "segments"          — list of STM token lines, e.g. [["FR90","F20","S"], ["F30","S"]]
#   "obstacle_ids"      — obstacle IDs in visit order
#   "segment_obstacles" — parallel to "segments": obstacle to photograph after
#                         that line, or None for pure travel
#   "dirs"              — robot pose per line, for the Android map

sys.path.insert(0, os.path.join(REPO_ROOT, "algo"))
from stm_tokens import PROFILE_NAMES, TURN_RADIUS_MM  # noqa: E402
from path_planner import plan_mission  # noqa: E402

# Must match whatever profile is actually running on the STM (?STAT).
ARC_PROFILE = int(os.getenv("STM_ARC_PROFILE", "0"))


def compute_path(obstacles: list) -> dict:
    logging.info(
        f"Planning against arc profile {ARC_PROFILE} "
        f"({PROFILE_NAMES.get(ARC_PROFILE, '?')}, radius "
        f"{TURN_RADIUS_MM.get(ARC_PROFILE, '?')}mm)."
    )
    return plan_mission(obstacles, arc_profile=ARC_PROFILE)


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

# The RPi gives up on a PATH reply after PATH_TIMEOUT_S (30 s, rpi/mdp_rpi/task1.py).
# A plan slower than this may arrive after the RPi has stopped listening for it.
RPI_PATH_TIMEOUT_S = float(os.getenv("PATH_TIMEOUT_S", "30.0"))


def _connection_lost_msg(doing: str, message, exc: BaseException) -> str:
    what = f" ({message.split(',', 1)[0]})" if message else ""
    return (
        f"Connection to the RPi was lost while {doing}{what}: {exc!r}. "
        "The RPi side closed or dropped the link - usually task1.py on the RPi "
        "exited or was restarted, or the hotspot blipped. Restart task1.py on "
        "the RPi first, then this script."
    )


class RPiConnection:
    """TCP client that connects to the RPi's socket server."""

    def __init__(self, ip: str, port: int):
        self.ip   = ip
        self.port = port
        self.sock: Optional[socket.socket] = None
        self._rx_buf = b""   # socket receive buffer

    def connect(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.port))
        logging.info(f"Connected to RPi at {self.ip}:{self.port}.")

    def send(self, message: str) -> bool:
        """Send a newline-terminated UTF-8 text message to the RPi.

        Returns False (and logs why) if the connection is gone, instead of
        raising: on Windows a peer that closed while the PC was busy shows up
        here or in receive_line() as ConnectionAbortedError [WinError 10053] /
        ConnectionResetError [WinError 10054], and a raw traceback hides what
        actually happened.
        """
        payload = (message.rstrip("\n") + "\n").encode("utf-8")
        try:
            self.sock.sendall(payload)
        except OSError as exc:
            logging.error(_connection_lost_msg("sending", message, exc))
            return False
        logging.info(f"→ RPi: {message.strip()}")
        return True

    def receive_line(self) -> Optional[str]:
        """
        Blocking — accumulates data until a newline is found, then returns
        one complete line. Returns None if the connection is closed.
        """
        while b"\n" not in self._rx_buf:
            try:
                chunk = self.sock.recv(4096)
            except OSError as exc:
                logging.error(_connection_lost_msg("waiting for a message", None, exc))
                return None
            if not chunk:
                return None
            self._rx_buf += chunk
        line, self._rx_buf = self._rx_buf.split(b"\n", 1)
        msg = line.decode("utf-8", errors="replace").strip()
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
        """Receive one image using the test_camera length-prefix protocol."""
        os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
        try:
            while len(self._rx_buf) < 4:
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise ConnectionError("Connection closed reading image size.")
                self._rx_buf += chunk

            image_size = struct.unpack(">I", self._rx_buf[:4])[0]
            self._rx_buf = self._rx_buf[4:]
            logging.info(f"Receiving image: {image_size} bytes.")

            while len(self._rx_buf) < image_size:
                chunk = self.sock.recv(image_size - len(self._rx_buf))
                if not chunk:
                    raise ConnectionError("Connection closed reading image bytes.")
                self._rx_buf += chunk
            image_bytes = self._rx_buf[:image_size]
            self._rx_buf = self._rx_buf[image_size:]

            with open(save_path, "wb") as fh:
                fh.write(image_bytes)
            logging.info(f"Image saved → {save_path} ({len(image_bytes)} bytes).")
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
    monitor = PlanMonitor(None)     # replaced by one built from each PATH we send

    logging.info("Waiting for messages from RPi…")

    while True:
        msg = rpi.receive_line()
        if msg is None:
            logging.info("RPi disconnected — exiting.")
            logging.info(monitor.summary())
            break

        # ── PROGRESS → compare with the plan; answer a feedback wait ──────────
        # One of these arrives after EVERY instruction (the RPi now sends each
        # token as its own STM line). With feedback on, the RPi stalls until we
        # answer, so this must stay quick and never raise.
        if msg.startswith("PROGRESS,"):
            reply = monitor.handle(msg.split(",", 1)[1])
            if reply is not None and not rpi.send(reply):
                break
            continue

        # ── OBSTACLES → pathfinding → PATH ────────────────────────────────────
        if msg.startswith("OBSTACLES"):
            try:
                obstacles = json.loads(msg.split("OBSTACLES,", 1)[1])
            except (IndexError, json.JSONDecodeError) as exc:
                logging.error(f"Bad OBSTACLES message: {exc}")
                continue

            logging.info(f"{len(obstacles)} obstacle(s) received — computing path…")
            t_plan = time.monotonic()
            try:
                path = compute_path(obstacles)
            except Exception:
                # One bad layout must not take the whole PC server down. Nothing
                # is sent, so the RPi's PATH watchdog reports STATUS,FAILED to
                # Android; fix the layout / planner and press Send Data again.
                logging.exception(
                    f"Path planning crashed on obstacles {obstacles} — no PATH sent."
                )
                continue
            took = time.monotonic() - t_plan
            if took > 0.8 * RPI_PATH_TIMEOUT_S:
                logging.warning(
                    f"Planning took {took:.0f}s; the RPi stops waiting for PATH after "
                    f"{RPI_PATH_TIMEOUT_S:.0f}s, so this plan may arrive too late."
                )
            monitor = PlanMonitor(path)
            if not rpi.send("PATH," + json.dumps(path)):
                break

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
                if not rpi.send(f"OBJECT,{obstacle_id},0.0,NONE"):
                    break
                continue

            # Run YOLO via your detect.py
            class_id, confidence = run_detection(save_path)

            if class_id is None:
                logging.warning(f"Nothing detected for obstacle {obstacle_id}.")
                if not rpi.send(f"OBJECT,{obstacle_id},0.0,NONE"):
                    break
            else:
                logging.info(
                    f"Detected '{class_id}' (conf={confidence:.2f}) "
                    f"for obstacle {obstacle_id}."
                )
                if not rpi.send(f"OBJECT,{obstacle_id},{confidence:.4f},{class_id}"):
                    break

                # Track for stitching
                annotated_path = os.path.join(
                    ANNOTATED_DIR, os.path.basename(save_path)
                )
                annotated_images.append(annotated_path)

        # ── STITCH → combine all result images ────────────────────────────────
        elif msg.startswith("STITCH"):
            logging.info("STITCH received — creating result image…")
            logging.info(monitor.summary())
            stitch_images(annotated_images, STITCHED_OUTPUT)
            annotated_images.clear()
            logging.info("Stitching done. Ready for next run (Ctrl-C to quit).")

        else:
            logging.warning(f"Unrecognised message from RPi: '{msg}'.")

    rpi.close()


if __name__ == "__main__":
    main()
