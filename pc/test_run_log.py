"""
pc/test_run_log.py  —  check run_log.py end-to-end, no robot needed
─────────────────────────────────────────────────────────────────────────
Runs the real task1_pc.main() against a fake RPi on localhost. YOLO is
stubbed out (and OpenCV too, if missing), so neither needs installing.

The fake RPi sends:
  1. OBSTACLES with one good obstacle and one that has no viewing pose  → ERROR
  2. a DETECT whose image never arrives                                  → ERROR
  3. a message the PC doesn't recognise                                  → WARNING
  then disconnects, so task1_pc exits normally.

    python3 test_run_log.py

Expected: a "Run finished … error(s), … warning(s)" summary at the end,
and a new task1_pc_<timestamp>.log in pc/logs/ containing it.
"""

import json
import os
import socket
import sys
import threading
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# Stub YOLO before task1_pc imports it.
fake_detect = types.ModuleType("image_recognition.detect")
fake_detect.detect = lambda image_path: (None, None)
sys.modules["image_recognition.detect"] = fake_detect
try:
    import cv2  # noqa: F401  (only used for stitching, which this test skips)
except ImportError:
    sys.modules["cv2"] = types.ModuleType("cv2")

import task1_pc  # noqa: E402  (calls run_log.setup on import)

OBSTACLES = [
    {"id": 1, "x": 10, "y": 10, "d": 0},   # open space — plannable
    {"id": 2, "x": 1, "y": 18, "d": 0},    # faces the top wall — no viewing pose
]


def fake_rpi(server: socket.socket) -> None:
    conn, _ = server.accept()
    with conn:
        conn.sendall(f"OBSTACLES,{json.dumps(OBSTACLES)}\n".encode())
        reply = b""
        while not reply.endswith(b"\n"):          # wait for PATH
            reply += conn.recv(65536)
        conn.sendall(b"HELLO_FROM_NOWHERE\n")       # unrecognised → WARNING
        conn.sendall(b"DETECT,1\n")                 # then hang up mid-image → ERROR


def main() -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    task1_pc.RPI_IP, task1_pc.RPI_PORT = server.getsockname()
    threading.Thread(target=fake_rpi, args=(server,), name="fake-rpi", daemon=True).start()
    task1_pc.main()
    server.close()


if __name__ == "__main__":
    main()
