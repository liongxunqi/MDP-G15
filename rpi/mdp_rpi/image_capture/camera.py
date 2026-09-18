"""
image_capture/camera.py
───────────────────────
Camera wrapper for Raspberry Pi OS Buster.

Uses the legacy picamera library instead of picamera2.

Requires:
    sudo apt install python3-picamera
"""

import logging
import os
import time
from datetime import datetime
from typing import Optional

from picamera import PiCamera


class Camera:

    def __init__(self, save_dir: str = "captured_images"):
        """
        Initialise the Raspberry Pi camera.

        The camera stays open between captures and should only be closed
        using stop_camera() when Task 1 is finished.
        """
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)

        self.camera = PiCamera()

        # Same resolution requested in the previous implementation
        self.camera.resolution = (1920, 1080)

        # Allow auto exposure / white balance to settle
        time.sleep(2)

        logging.info(
            "Camera: started in auto-exposure mode (1920x1080)."
        )

    # ── Exposure helpers ──────────────────────────────────────────────

    def set_auto(self) -> None:
        """Switch camera back to automatic exposure."""
        self.camera.exposure_mode = "auto"
        self.camera.awb_mode = "auto"

        logging.info("Camera: auto-exposure enabled.")

    def set_exposure(self, exposure_us: int, gain: float) -> None:
        """
        Manual exposure override.

        exposure_us:
            Shutter time in microseconds.

        gain:
            Analogue gain requested by the caller.

        Note:
            Legacy picamera does not provide the same direct analogue-gain
            control interface as picamera2. Exposure time can be controlled
            using shutter_speed.
        """

        self.camera.shutter_speed = exposure_us
        self.camera.exposure_mode = "off"

        logging.info(
            f"Camera: manual exposure={exposure_us}us "
            f"(requested gain={gain})."
        )

    # ── Capture ───────────────────────────────────────────────────────

    def capture_image(self) -> Optional[str]:
        """
        Capture a JPEG and save it with a timestamp filename.

        Returns the file path on success or None on failure.
        """
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"image_{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)

            self.camera.capture(filepath, format="jpeg")

            logging.info(f"Camera: captured -> {filepath}")

            return filepath

        except Exception as exc:
            logging.error(f"Camera: capture failed - {exc}")
            return None

    # ── Teardown ─────────────────────────────────────────────────────

    def stop_camera(self) -> None:
        """
        Close the camera.

        Call once when Task 1 has finished.
        """
        try:
            self.camera.close()
            logging.info("Camera: stopped.")

        except Exception as exc:
            logging.warning(f"Camera: stop error - {exc}")