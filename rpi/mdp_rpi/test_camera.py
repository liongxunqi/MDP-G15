"""
test_camera.py  —  Verify picamera2 capture works on the RPi
──────────────────────────────────────────────────────────────
Run on the RPi only:
    python3 test_camera.py

Captures one image, prints the saved path.
No networking or STM needed.
"""

import logging
logging.basicConfig(level=logging.INFO)

from image_capture.camera import Camera

def main():
    logging.info("Initialising camera…")
    cam = Camera()
    logging.info("Camera ready. Capturing test image…")
    path = cam.capture_image()
    if path:
        logging.info(f"SUCCESS — image saved to: {path}")
    else:
        logging.error("FAILED — capture_image() returned None.")
    cam.stop_camera()

if __name__ == "__main__":
    main()
