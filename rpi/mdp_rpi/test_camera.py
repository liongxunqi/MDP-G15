"""
test_camera.py
─────────────────────────────────────────────────────────────────────

Persistent camera test flow:

    Start Camera ONCE
        ↓
    Press ENTER
        ↓
    Capture JPEG into RAM
        ↓
    Send image to laptop
        ↓
    Laptop runs YOLO
        ↓
    Detection returned to RPi
        ↓
    If Android connected:
        TARGET,<obstacle_id>,<class_id>

    Otherwise:
        result is printed locally
        ↓
    Press ENTER again for next image

The camera remains open for the ENTIRE test session.
The image is NOT saved on the Raspberry Pi.

Press q + ENTER to quit.
"""

import logging
import socket
import struct
import time

from image_capture.camera import Camera
from communications.android import Android


# ── Logging ──────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s - %(message)s",
)

# ── Configuration ────────────────────────────────────────────────────

PC_IP = "192.168.15.26"
PC_PORT = 5001

# In actual Task 1 this comes from obstacle data.
TEST_OBSTACLE_ID = "1"

# Maximum time to wait for Android.
ANDROID_TIMEOUT = 15

# PC socket timeout so the test cannot hang forever.
PC_TIMEOUT = 15

# ── Send image to laptop ─────────────────────────────────────────────

def send_image(image_bytes: bytes) -> str:
    """
    Send JPEG bytes directly to the laptop.
    Protocol:
        4-byte big-endian image size
        +
        raw JPEG bytes
    Laptop returns the detection result.
    """
    with socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM,
    ) as sock:
        sock.settimeout(PC_TIMEOUT)
        logging.info(
            "Connecting to PC at %s:%s...",
            PC_IP,
            PC_PORT,
        )
        sock.connect(
            (PC_IP, PC_PORT)
        )
        logging.info(
            "Connected to PC."
        )
        # Send image size
        image_size = len(image_bytes)
        sock.sendall(
            struct.pack(
                ">I",
                image_size,
            )
        )

        # Send JPEG
        sock.sendall(
            image_bytes
        )

        logging.info(
            "Sent %d bytes.",
            image_size,
        )

        # Receive detection result
        result = (
            sock.recv(1024)
            .decode("utf-8")
            .strip()
        )

        logging.info(
            "Result received from PC: %s",
            result,
        )

        return result


# ── Parse laptop result ──────────────────────────────────────────────

def parse_result(result: str):
    """
    Supports both:
        G
    and:
        OBJECT,1,0.77,G
    Returns:
        obstacle_id
        class_id
        confidence
    """
    result = result.strip()
    parts = [
        part.strip()
        for part in result.split(",")
    ]
    # Full Task 1 response
    if (
        len(parts) >= 4
        and parts[0].upper() == "OBJECT"
    ):
        obstacle_id = parts[1]
        confidence_string = parts[2]
        class_id = parts[3]
        try:
            confidence = float(
                confidence_string
            )
        except ValueError:
            confidence = None

        return (
            obstacle_id,
            class_id,
            confidence,
        )

    # Simple PC test response:
    #
    # G
    # C
    # RIGHT_ARROW
    #
    return (
        TEST_OBSTACLE_ID,
        result,
        None,
    )

# ── Display result ───────────────────────────────────────────────────

def display_result(
    capture_number,
    obstacle_id,
    class_id,
    confidence,
):

    print()
    print(
        "──────── Detection Result ────────"
    )

    print(
        "Capture  : {}".format(
            capture_number
        )
    )

    print(
        "Obstacle : {}".format(
            obstacle_id
        )
    )

    print(
        "Class    : {}".format(
            class_id
        )
    )

    if confidence is not None:
        print(
            "Confidence: {:.3f}".format(
                confidence
            )
        )

    print(
        "──────────────────────────────────"
    )
    print()

# ── Main ─────────────────────────────────────────────────────────────

def main():

    android = None
    camera = None

    try:

        # =============================================================
        # 1. START ANDROID BLUETOOTH
        # =============================================================

        try:

            android = Android()
            android.start()

            logging.info(
                "Waiting up to %d seconds for Android...",
                ANDROID_TIMEOUT,
            )

            start_time = time.time()

            while not android.connected:

                elapsed = (
                    time.time()
                    - start_time
                )

                if elapsed >= ANDROID_TIMEOUT:

                    logging.info(
                        "No Android connection after %d seconds.",
                        ANDROID_TIMEOUT,
                    )

                    logging.info(
                        "Continuing in local test mode."
                    )

                    break

                time.sleep(0.2)

            if android.connected:

                logging.info(
                    "Android connected - "
                    "results will be forwarded."
                )

        except Exception as exc:

            logging.warning(
                "Android unavailable: %s",
                exc,
            )

            android = None


        # =============================================================
        # 2. INITIALISE CAMERA ONCE
        # =============================================================

        logging.info(
            "Initialising camera..."
        )

        camera = Camera()

        logging.info(
            "Camera initialised."
        )

        logging.info(
            "Camera will remain open for the entire test."
        )


        # =============================================================
        # 3. REPEATED CAPTURE LOOP
        # =============================================================

        capture_number = 0

        while True:

            print()
            command = input(
                "Press ENTER to capture "
                "(or q + ENTER to quit): "
            ).strip().lower()

            if command == "q":
                logging.info(
                    "Camera test finished by user."
                )
                break

            capture_number += 1

            print()

            logging.info(
                "Capture #%d - capturing image...",
                capture_number,
            )


            # =========================================================
            # 4. CAPTURE IMAGE INTO RAM
            # =========================================================

            try:

                image_bytes = (
                    camera.capture_image()
                )

            except Exception as exc:

                logging.exception(
                    "Capture #%d failed: %s",
                    capture_number,
                    exc,
                )

                logging.error(
                    "Camera test will remain running."
                )

                logging.error(
                    "Press ENTER to try another capture "
                    "or q to quit."
                )

                continue


            if not image_bytes:

                logging.error(
                    "Capture #%d returned no image.",
                    capture_number,
                )

                continue


            logging.info(
                "Image captured in RAM: %d bytes.",
                len(image_bytes),
            )


            # =========================================================
            # 5. SEND IMAGE TO LAPTOP
            # =========================================================

            try:

                result = send_image(
                    image_bytes
                )

            except ConnectionRefusedError:

                logging.error(
                    "Could not connect to PC."
                )

                logging.error(
                    "Make sure the PC detection "
                    "server is running."
                )

                continue

            except socket.timeout:

                logging.error(
                    "PC connection timed out."
                )

                continue

            except OSError as exc:

                logging.error(
                    "PC connection failed: %s",
                    exc,
                )

                continue


            if not result:

                logging.error(
                    "PC returned no detection result."
                )

                continue


            # =========================================================
            # 6. PARSE DETECTION
            # =========================================================

            (
                obstacle_id,
                class_id,
                confidence,
            ) = parse_result(
                result
            )


            # =========================================================
            # 7. DISPLAY RESULT ON RPI
            # =========================================================

            display_result(
                capture_number,
                obstacle_id,
                class_id,
                confidence,
            )


            # =========================================================
            # 8. SEND RESULT TO ANDROID
            # =========================================================

            if (
                android is not None
                and android.connected
            ):

                android_message = (
                    "TARGET,{0},{1}".format(
                        obstacle_id,
                        class_id,
                    )
                )

                try:

                    android.send(
                        android_message
                    )

                    logging.info(
                        "Sent to Android: %s",
                        android_message,
                    )

                except Exception as exc:

                    logging.warning(
                        "Could not send result "
                        "to Android: %s",
                        exc,
                    )

            else:

                logging.info(
                    "Android not connected."
                )

                logging.info(
                    "Detection shown locally instead."
                )


    # ── Ctrl+C ───────────────────────────────────────────────────────

    except KeyboardInterrupt:

        logging.info(
            "Interrupted by user."
        )


    # ── Unexpected error ─────────────────────────────────────────────

    except Exception as exc:

        logging.exception(
            "Test failed: %s",
            exc,
        )


    # ── Cleanup ──────────────────────────────────────────────────────

    finally:

        logging.info(
            "Cleaning up..."
        )

        if camera is not None:

            try:

                camera.stop_camera()

                logging.info(
                    "Camera stopped."
                )

            except Exception as exc:

                logging.warning(
                    "Error while stopping camera: %s",
                    exc,
                )


        if android is not None:

            try:

                android.stop()

                logging.info(
                    "Android stopped."
                )

            except Exception:

                pass

        logging.info(
            "Test complete."
        )


# ── Entry point ──────────────────────────────────────────────────────

if __name__ == "__main__":
    main()