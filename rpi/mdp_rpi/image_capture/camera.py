"""
image_capture/camera.py
───────────────────────
Wraps picamera2 for still-image capture.

Differences from Group 6's version:
  • No input() calls — camera starts immediately in auto-exposure mode.
    If you need manual exposure, call set_exposure() before capturing.
  • Camera is started once in __init__ and stays warm between captures
    (faster capture, avoids re-init overhead mid-run).
  • stop_camera() called once at the very end of Task 1.

Requires:  sudo apt install python3-picamera2
"""

import logging
import os
from datetime import datetime
from typing import Optional

from picamera2 import Picamera2


class Camera:

    def __init__(self, save_dir: str = "captured_images"):
        """
        Initialise and start the camera in still-capture configuration.
        The camera is kept warm (started) until stop_camera() is called.
        """
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)

        self.picam2 = Picamera2()

        # Still config at full sensor resolution — good for YOLO accuracy
        config = self.picam2.create_still_configuration(
            main={"size": (1920, 1080)}
        )
        self.picam2.configure(config)

        # Auto exposure / auto gain — works well indoors and outdoors
        # Call set_exposure() if you need to override
        self.picam2.start()
        logging.info("Camera: started in auto-exposure mode (1920×1080).")

    # ── Exposure helpers ──────────────────────────────────────────────────────

    def set_auto(self) -> None:
        """Switch back to full auto (default)."""
        self.picam2.set_controls({"AeEnable": True})
        logging.info("Camera: auto-exposure enabled.")

    def set_exposure(self, exposure_us: int, gain: float) -> None:
        """
        Manual override.
          exposure_us  — shutter time in microseconds
                         e.g. 10_000 (bright), 100_000 (dim), 500_000 (dark)
          gain         — analogue gain multiplier (1.0 = no boost, 2.0 = darker scenes)
        """
        self.picam2.set_controls({
            "AeEnable": False,
            "ExposureTime": exposure_us,
            "AnalogueGain": gain,
        })
        logging.info(f"Camera: manual exposure={exposure_us}µs  gain={gain}.")

    # ── Capture ───────────────────────────────────────────────────────────────

    def capture_image(self) -> Optional[str]:
        """
        Capture a JPEG, save it to save_dir with a timestamp filename.
        Returns the full file path, or None on failure.
        """
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"image_{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)

            self.picam2.capture_file(filepath)
            logging.info(f"Camera: captured → {filepath}")
            return filepath
        except Exception as exc:
            logging.error(f"Camera: capture failed — {exc}")
            return None

    # ── Teardown ──────────────────────────────────────────────────────────────

    def stop_camera(self) -> None:
        """Stop the camera. Call once at the end of the task."""
        try:
            self.picam2.stop()
            logging.info("Camera: stopped.")
        except Exception as exc:
            logging.warning(f"Camera: stop error — {exc}")
