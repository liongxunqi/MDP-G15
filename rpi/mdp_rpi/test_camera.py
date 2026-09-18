"""
test_camera.py  —  Capture image, send to PC, print detection result
─────────────────────────────────────────────────────────────────────
Run on RPi:
    python3 test_camera.py

Run on laptop first:
    python3 test_pc_server.py
"""

import logging
import socket
import struct
import os

logging.basicConfig(level=logging.INFO)

from image_capture.camera import Camera

PC_IP   = "192.168.15.26"   # replace with your laptop's IP on the hotspot
PC_PORT = 5001

def send_image(image_path: str, host: str, port: int) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        logging.info(f"Connected to PC at {host}:{port}")

        with open(image_path, "rb") as f:
            image_bytes = f.read()

        # 4-byte header + raw bytes (matches test_pc_server.py)
        s.sendall(struct.pack(">I", len(image_bytes)))
        s.sendall(image_bytes)
        logging.info(f"Sent {len(image_bytes)} bytes")

        result = s.recv(1024).decode("utf-8")
        logging.info(f"Detection result: {result}")
        return result

def main():
    logging.info("Initialising camera…")
    cam = Camera()
    logging.info("Capturing image…")
    path = cam.capture_image()
    if not path:
        logging.error("Capture failed.")
        cam.stop_camera()
        return

    logging.info(f"Image saved: {path}")
    result = send_image(path, PC_IP, PC_PORT)
    print(f"Detected: {result}")
    cam.stop_camera()

if __name__ == "__main__":
    main()