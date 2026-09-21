"""
image_capture/camera.py
───────────────────────
Camera wrapper for Raspberry Pi OS Buster.

Uses the legacy picamera library.

The Raspberry Pi automatically controls:
    - shutter speed
    - ISO / analogue gain
    - exposure
    - white balance
    - metering

Nothing is saved on the Raspberry Pi.
The JPEG is captured directly into RAM.

Requires:
    sudo apt install python3-picamera
"""

import io
import logging
import time
from typing import Optional

from picamera import PiCamera


class Camera:

    def __init__(self):
        """
        Initialise the Raspberry Pi camera.

        The camera stays open during Task 1 so that automatic exposure
        and white balance do not need to restart for every image.
        """

        self.camera = None
        self._open_camera()

    # ── Camera lifecycle ─────────────────────────────────────────────

    def _open_camera(self) -> None:
        """Open and configure the legacy PiCamera device."""
        self.camera = PiCamera()

        # Full resolution sent to PC
        self.camera.resolution = (1920, 1080)

        # Reasonable frame rate while still allowing AE some flexibility
        self.camera.framerate = 15

        self.set_auto()

        logging.info(
            "Camera: warming up automatic exposure and white balance..."
        )

        # Initial camera warm-up
        time.sleep(2)

        logging.info(
            "Camera: ready in full automatic mode (1920x1080)."
        )

    def _reopen_camera(self) -> None:
        """Recover from a wedged MMAL capture by closing and reopening."""
        logging.warning("Camera: reopening camera after capture failure...")
        try:
            if self.camera is not None:
                self.camera.close()
        except Exception as exc:
            logging.warning("Camera: close during reopen failed - %s", exc)
        time.sleep(1)
        self._open_camera()

    # ── Automatic settings ───────────────────────────────────────────

    def set_auto(self) -> None:
        """
        Restore genuine automatic camera behaviour.
        """
        if self.camera is None:
            self._open_camera()

        # Camera chooses shutter speed automatically
        self.camera.shutter_speed = 0

        # Camera chooses ISO / analogue gain automatically
        self.camera.iso = 0

        # Genuine automatic exposure
        self.camera.exposure_mode = "auto"

        # Automatic white balance
        self.camera.awb_mode = "auto"

        # Let the camera evaluate the whole frame.
        #
        # Do NOT use spot here because our target may be dark
        # while the surrounding wall is bright.
        self.camera.meter_mode = "average"

        # Do not bias automatic exposure
        self.camera.exposure_compensation = 0

        # Neutral image processing
        self.camera.brightness = 50
        self.camera.contrast = 0
        self.camera.sharpness = 0
        self.camera.saturation = 0

        logging.info(
            "Camera: full automatic mode enabled."
        )

    # ── Compatibility helper ─────────────────────────────────────────

    def set_exposure(
        self,
        exposure_us: int,
        gain: float,
    ) -> None:
        """
        Optional manual exposure override.

        Kept only so existing code using this method does not break.

        Normal Task 1 captures should use automatic mode.
        """

        self.camera.shutter_speed = exposure_us
        self.camera.exposure_mode = "off"

        logging.info(
            "Camera: manual exposure=%sus "
            "(requested gain=%s).",
            exposure_us,
            gain,
        )

    # ── Automatic exposure settling ──────────────────────────────────

    def _settle_camera(self) -> None:
        """
        Give Raspberry Pi's automatic exposure and AWB algorithms
        a few frames to react to the current lighting.

        Do not capture throw-away frames here. On some Buster/MMAL camera
        stacks a video-port warm-up capture can hang for a full minute even
        though `raspistill` works. Passive settling keeps the sensor pipeline
        simple and leaves the actual still capture to the same path raspistill
        uses successfully.
        """

        logging.info(
            "Camera: allowing automatic settings to settle..."
        )

        time.sleep(1.0)

    # ── Debugging ────────────────────────────────────────────────────

    def _log_auto_settings(self) -> None:
        """
        Print what the Raspberry Pi automatically selected.
        """

        try:

            logging.info(
                "Camera auto settings: "
                "exposure=%sus, "
                "analog_gain=%s, "
                "digital_gain=%s, "
                "awb_gains=%s",
                self.camera.exposure_speed,
                self.camera.analog_gain,
                self.camera.digital_gain,
                self.camera.awb_gains,
            )

        except Exception as exc:

            logging.warning(
                "Camera: could not read auto settings - %s",
                exc,
            )

    # ── Capture ──────────────────────────────────────────────────────

    def capture_image(self) -> Optional[bytes]:
        """
        Capture one JPEG directly into memory.

        Nothing is written to disk on the Raspberry Pi.

        Returns:
            JPEG bytes on success

        or:
            None on failure
        """

        for attempt in range(1, 3):
            stream = io.BytesIO()
            try:

                # Always begin from clean automatic settings
                self.set_auto()

                # Let the camera inspect the current lighting first
                self._settle_camera()

                # See what values the Pi chose
                self._log_auto_settings()

                # Full-resolution JPEG stays entirely in RAM. This uses the
                # still port, matching the path that `raspistill` exercises.
                self.camera.capture(
                    stream,
                    format="jpeg",
                    use_video_port=False,
                )

                image_bytes = stream.getvalue()

                if not image_bytes:

                    logging.error(
                        "Camera: captured JPEG is empty."
                    )

                    return None

                logging.info(
                    "Camera: captured %d JPEG bytes in memory.",
                    len(image_bytes),
                )

                return image_bytes

            except Exception as exc:

                logging.error(
                    "Camera: capture attempt %d failed - %s",
                    attempt,
                    exc,
                )
                if attempt == 1:
                    self._reopen_camera()
                else:
                    return None

            finally:

                stream.close()

        return None

    # ── Teardown ─────────────────────────────────────────────────────

    def stop_camera(self) -> None:
        """
        Close the camera once Task 1/testing is completely finished.
        """

        try:

            if self.camera is not None:
                self.camera.close()
                self.camera = None

            logging.info(
                "Camera: stopped."
            )

        except Exception as exc:

            logging.warning(
                "Camera: stop error - %s",
                exc,
            )
