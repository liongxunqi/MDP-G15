"""
task_a5.py — MDP checklist A.5: navigate around the obstacle

RPi movement/camera with PC-hosted detection. No Android or pathfinding.

    python3 task_a5.py
    python3 task_a5.py --dry-run
    python3 task_a5.py --faces 2

Loop: approach to standoff, look at the face in front, if it's not a valid
image orbit to the next face and repeat.
"""

import argparse
import logging
import os
import socket
import struct
import sys
from time import sleep

from dotenv import load_dotenv

load_dotenv()

from communications.stm import (
    FU_MAX_CM,
    FU_MIN_CM,
    FU_MIN_PROTOCOL,
    PROTOCOL_VERSION,
    US_BIAS_CM,
    STM,
)
from image_capture.camera import Camera

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [A5] %(levelname)s — %(message)s",
    datefmt="%H:%M:%S",
)

# bullseye is the "right obstacle, wrong face" marker, not a target. dot is filler.
NOT_A_TARGET = {"bullseye", "dot"}
LOOK_ERROR = object()

STANDOFF_CM = int(os.getenv("A5_STANDOFF_CM", "45"))
ORBIT_INSET_CM = int(os.getenv("A5_ORBIT_INSET_CM", "20"))
CAPTURE_SETTLE_S = float(os.getenv("A5_CAPTURE_SETTLE_S", "2.0"))
MAX_APPROACH_CM = int(os.getenv("A5_MAX_APPROACH_CM", "150"))
MAX_FACES = int(os.getenv("A5_MAX_FACES", "4"))
ARC_PROFILE = int(os.getenv("A5_ARC_PROFILE", "1"))
OBSTACLE_SIZE_CM = float(os.getenv("A5_OBSTACLE_SIZE_CM", "10"))
ORBIT_BACKUP_CM = int(os.getenv("A5_ORBIT_BACKUP_CM", "20"))
REAR_AXLE_TO_SENSOR_CM = float(os.getenv("A5_REAR_AXLE_TO_SENSOR_CM", "23"))
ORBIT_ADVANCE_CORRECTION_CM = int(os.getenv("A5_ORBIT_ADVANCE_CORRECTION_CM", "10"))
ORBIT_CLEARANCE_CM = int(os.getenv("A5_ORBIT_CLEARANCE_CM", "15"))
PC_IP = os.getenv("A5_PC_IP", "192.168.15.26")
PC_PORT = int(os.getenv("A5_PC_PORT", "5001"))
PC_TIMEOUT_S = float(os.getenv("A5_PC_TIMEOUT_S", "30.0"))

# The initial approach() has no idea how far the obstacle really is — a human
# placed the robot, could be a metre off — so it needs the loose ceiling
# above. After orbit(), the geometry says exactly how close the obstacle
# should be: roughly STANDOFF_CM out, give or take execution slop. A reading
# far outside that is the sensor looking at something else — the wall
# behind the obstacle, most likely — not the obstacle itself. Reject it
# instead of driving at whatever it sees; a loud stop beats a silent
# overshoot into an object that was never actually where FU thought it was.
POST_ORBIT_MAX_APPROACH_CM = int(os.getenv("A5_POST_ORBIT_MAX_APPROACH_CM",
                                            str(STANDOFF_CM + 50)))

# Profile 1 CLEAN has a measured 31.8cm radius. A5 requests the real geometric
# angle, then uses ?TURN to verify and correct the completed heading.
CLEAN_RADIUS_CM = 31.8
ORBIT_FR_DEG = int(os.getenv("A5_ORBIT_FR_DEG", "90"))
ORBIT_FL_DEG = int(os.getenv("A5_ORBIT_FL_DEG", "90"))
ORBIT_FR_COMMAND_OFFSET_DEG = int(
    os.getenv("A5_ORBIT_FR_COMMAND_OFFSET_DEG", "0")
)
ORBIT_FL_COMMAND_OFFSET_DEG = int(
    os.getenv("A5_ORBIT_FL_COMMAND_OFFSET_DEG", "0")
)
ORBIT_RR_COMMAND_OFFSET_DEG = int(
    os.getenv("A5_ORBIT_RR_COMMAND_OFFSET_DEG", "0")
)
ORBIT_RL_COMMAND_OFFSET_DEG = int(
    os.getenv("A5_ORBIT_RL_COMMAND_OFFSET_DEG", "0")
)
ORBIT_HEADING_TOLERANCE_DEG = float(
    os.getenv("A5_ORBIT_HEADING_TOLERANCE_DEG", "2")
)
ORBIT_MAX_CORRECTION_DEG = int(os.getenv("A5_ORBIT_MAX_CORRECTION_DEG", "10"))
ORBIT_MIN_CORRECTION_DEG = int(os.getenv("A5_ORBIT_MIN_CORRECTION_DEG", "5"))
ORBIT_MAX_CORRECTIONS = int(os.getenv("A5_ORBIT_MAX_CORRECTIONS", "1"))
ORBIT_CORRECTION_COAST_DEG = float(
    os.getenv("A5_ORBIT_CORRECTION_COAST_DEG", "3.5")
)
if ARC_PROFILE != 1:
    raise ValueError("Task A5 is configured only for arc profile 1 CLEAN.")
if not (0 < ORBIT_INSET_CM < STANDOFF_CM):
    raise ValueError(
        "A5_ORBIT_INSET_CM must be positive and smaller than A5_STANDOFF_CM."
    )

ORBIT_GEOMETRY_STANDOFF_CM = STANDOFF_CM - ORBIT_INSET_CM


def _profile_one_orbit() -> str:
    # FU measures from the front ultrasonic sensor, while arc displacement is
    # measured at the driven rear axle. Include that longitudinal offset when
    # placing the axle around the next face.
    radial_cm = (
        ORBIT_GEOMETRY_STANDOFF_CM
        + OBSTACLE_SIZE_CM / 2.0
        + REAR_AXLE_TO_SENSOR_CM
    )
    # Floor testing lands square but 10 cm before the next face's centreline.
    # Subtracting that measured shortfall from the middle reverse advances the
    # chassis around the obstacle without changing its radial standoff.
    middle_reverse_cm = round(
        3.0 * CLEAN_RADIUS_CM
        - radial_cm
        - ORBIT_BACKUP_CM
        - ORBIT_ADVANCE_CORRECTION_CM
    )
    # Move the middle and final arcs into an outer lane. The final reverse is
    # shortened by the same amount, so this adds chassis clearance around the
    # obstacle without changing the final camera standoff.
    outward_reverse_cm = round(
        radial_cm - CLEAN_RADIUS_CM - ORBIT_CLEARANCE_CM
    )
    if outward_reverse_cm <= 0:
        raise ValueError("Task A5 orbit geometry produced a non-positive outward reverse.")

    if middle_reverse_cm > 0:
        middle_token = f"R{middle_reverse_cm}"
    elif middle_reverse_cm < 0:
        middle_token = f"F{-middle_reverse_cm}"
    else:
        middle_token = None

    tokens = [
        f"R{ORBIT_BACKUP_CM}",
        f"FR{ORBIT_FR_DEG}",
    ]
    if ORBIT_CLEARANCE_CM > 0:
        tokens.append(f"F{ORBIT_CLEARANCE_CM}")
    tokens.append(f"FL{ORBIT_FL_DEG}")
    if middle_token:
        tokens.append(middle_token)
    tokens.extend((f"FL{ORBIT_FL_DEG}", f"R{outward_reverse_cm}"))
    return ",".join(tokens)


DERIVED_ORBIT_LINE = _profile_one_orbit()
ORBIT_LINE = os.getenv("A5_ORBIT", "").strip() or DERIVED_ORBIT_LINE
if ORBIT_LINE != DERIVED_ORBIT_LINE:
    logging.warning("A5_ORBIT overrides profile-1 geometry: derived %s, using %s.",
                    DERIVED_ORBIT_LINE, ORBIT_LINE)


class TaskA5:

    def __init__(self, dry_run: bool = False, max_faces: int = MAX_FACES):
        self.stm = STM()
        self.camera = None
        self.dry_run = dry_run
        self.max_faces = max_faces

    def _send(self, line: str) -> bool:
        if self.dry_run:
            logging.info(f"[dry-run] would send: {line}")
            return True

        if not self.stm.send_line(line):
            logging.error(f"Refused to send '{line}' — see the validation error above.")
            return False

        reply = self.stm.wait_reply()
        if reply is None:
            logging.error("No reply from the STM — treat the link as lost.")
            return False
        if reply.startswith("OK"):
            return True
        if reply.startswith("RESEND"):
            logging.error(f"STM rejected '{line}' as malformed (RESEND).")
            return False

        logging.error(f"STM reported {reply} on '{line}'. The pose is no longer "
                      f"trustworthy — stopping.")
        return False

    def _detect_on_pc(self, image_bytes: bytes):
        """Send a length-prefixed JPEG to test_pc_server.py."""
        with socket.create_connection((PC_IP, PC_PORT), timeout=PC_TIMEOUT_S) as sock:
            sock.settimeout(PC_TIMEOUT_S)
            sock.sendall(struct.pack(">I", len(image_bytes)))
            sock.sendall(image_bytes)
            response = sock.recv(1024).decode("utf-8").strip()

        if not response:
            raise ConnectionError("PC closed the connection without a result.")
        parts = [part.strip() for part in response.split(",")]
        if len(parts) < 4 or parts[0].upper() != "OBJECT":
            raise ValueError(f"Unexpected detector response: {response!r}")
        try:
            confidence = float(parts[2])
        except ValueError as exc:
            raise ValueError(f"Invalid detector confidence: {response!r}") from exc
        return parts[3], confidence

    def _us_cm(self):
        if self.dry_run:
            return STANDOFF_CM

        fields = self.stm.query_fields("?US")
        if not fields:
            return None
        try:
            value = int(fields[0])
        except ValueError:
            logging.warning(f"Unparseable ?US reply: {fields}")
            return None

        if value == 0xFFFF:
            return None
        return value

    def _stand_off(self, target_cm: int, max_approach_cm: int = MAX_APPROACH_CM) -> bool:
        wanted = int(round(target_cm + US_BIAS_CM))
        if not (FU_MIN_CM <= wanted <= FU_MAX_CM):
            logging.error(f"Standoff {target_cm} cm becomes FU{wanted}, outside "
                          f"{FU_MIN_CM}..{FU_MAX_CM}.")
            return False

        seen = self._us_cm()
        if seen is None:
            logging.error("Ultrasonic sees nothing ahead — cannot stand off. Is the "
                          "robot pointed at the obstacle?")
            return False
        if seen > max_approach_cm:
            logging.error(f"Nearest thing ahead is {seen} cm away, past the "
                          f"{max_approach_cm} cm limit. That is not the obstacle "
                          "— likely the wall behind it, or a wall to the side, "
                          "because the robot isn't aligned with the obstacle. "
                          "Refusing to drive at it.")
            return False

        logging.info(f"Ultrasonic reads {seen} cm — closing to {target_cm} cm "
                     f"(FU{wanted}).")
        return self._send(f"FU{wanted}")

    def approach(self) -> bool:
        logging.info("── Approaching the obstacle ──")
        return self._stand_off(STANDOFF_CM)

    def look(self, face: int):
        logging.info(f"── Looking at face {face} ──")

        # FU and the final reverse may leave the camera mount rocking even
        # after the STM has replied OK. With automatic exposure near 60 ms,
        # that vibration visibly smears the target and can defeat YOLO.
        logging.info("Letting the chassis settle for %.1fs before capture.",
                     CAPTURE_SETTLE_S)
        sleep(CAPTURE_SETTLE_S)

        image_bytes = self.camera.capture_image()
        if not image_bytes:
            logging.error("Capture failed — stopping instead of orbiting blind.")
            return LOOK_ERROR

        try:
            logging.info("Sending %d JPEG bytes to PC at %s:%d...",
                         len(image_bytes), PC_IP, PC_PORT)
            name, conf = self._detect_on_pc(image_bytes)
        except Exception as exc:
            logging.error(f"PC detection failed: {exc}")
            return LOOK_ERROR

        if name is None or name.upper() == "NONE":
            logging.info("Nothing detected on this face.")
            return None
        if name.lower() in NOT_A_TARGET:
            logging.info(f"Found '{name}' (conf={conf:.2f}) — that is the marker, "
                         f"not a target. Wrong face.")
            return None
        logging.info(f"Valid image: '{name}' at conf={conf:.2f}.")
        return name, conf

    def orbit(self, face: int) -> bool:
        logging.info(f"── Face {face} was not it — going around ──")

        # Run the geometric segment without injecting corrections between its
        # primitives. ?TURN is relative to one arc, so add the signed results
        # and correct the net heading once the complete segment has landed.
        expected_heading = 0.0
        actual_heading = 0.0
        for token in ORBIT_LINE.split(","):
            token = token.strip()
            if not token:
                continue
            if token.startswith(("FR", "FL", "RR", "RL")):
                measured = self._send_measured_arc(token)
                if measured is None:
                    return False
                expected_heading += self._arc_heading(token)
                actual_heading += measured
            elif not self._send(token):
                return False

        corrected_heading = self._correct_segment_heading(
            expected_heading, actual_heading
        )
        if corrected_heading is None:
            return False

        # The compact orbit lands at its smaller sensor radius. Back away by
        # the difference, then let FU trim forward odometry error at the next
        # face and restore the camera's viewing pose.
        if not self._send(f"R{ORBIT_INSET_CM}"):
            return False

        sleep(0.3)
        return self._stand_off(STANDOFF_CM, max_approach_cm=POST_ORBIT_MAX_APPROACH_CM)

    def _read_turn(self, token: str):
        fields = self.stm.query_fields("?TURN")
        if not fields:
            logging.error("No ?TURN reading after %s.", token)
            return None
        try:
            return int(fields[0]) / 10.0
        except ValueError:
            logging.error("Invalid ?TURN reading after %s: %s", token, fields)
            return None

    @staticmethod
    def _arc_heading(token: str) -> float:
        heading_sign = {"FR": -1.0, "FL": 1.0, "RR": 1.0, "RL": -1.0}
        return float(token[2:]) * heading_sign[token[:2]]

    def _send_measured_arc(self, token: str):
        arc_type = token[:2]
        requested_deg = float(token[2:])
        command_offsets = {
            "FR": ORBIT_FR_COMMAND_OFFSET_DEG,
            "FL": ORBIT_FL_COMMAND_OFFSET_DEG,
            "RR": ORBIT_RR_COMMAND_OFFSET_DEG,
            "RL": ORBIT_RL_COMMAND_OFFSET_DEG,
        }
        if arc_type not in command_offsets:
            logging.error("Unsupported A5 arc token: %s.", token)
            return None

        command_offset = command_offsets[arc_type]
        command_deg = int(round(requested_deg + command_offset))
        if command_deg <= 0:
            logging.error("A5 arc compensation produced invalid command for %s.", token)
            return None
        command = arc_type + str(command_deg)
        if command != token:
            logging.info("A5 turn compensation: %s target uses %s command.",
                         token, command)
        if not self._send(command):
            return None
        if self.dry_run:
            return self._arc_heading(token)

        measured_deg = self._read_turn(command)
        if measured_deg is None:
            return None
        logging.info("Orbit %s measured %+.1f°.", command, measured_deg)
        return measured_deg

    def _correct_segment_heading(
        self, expected_heading: float, actual_heading: float
    ):
        logging.info(
            "Orbit segment complete: heading %+.1f° "
            "(target %+.1f°, error %+.1f°).",
            actual_heading, expected_heading, expected_heading - actual_heading,
        )
        for attempt in range(1, ORBIT_MAX_CORRECTIONS + 1):
            error_deg = expected_heading - actual_heading
            if abs(error_deg) <= ORBIT_HEADING_TOLERANCE_DEG + 1e-6:
                return actual_heading

            # Final correction only. This chassis cannot pivot in place, so a
            # short forward arc is the least disruptive available adjustment.
            correction_deg = max(
                ORBIT_MIN_CORRECTION_DEG,
                int(round(abs(error_deg) - ORBIT_CORRECTION_COAST_DEG)),
            )
            correction_deg = min(correction_deg, ORBIT_MAX_CORRECTION_DEG)
            correction_type = "FL" if error_deg > 0.0 else "FR"
            correction = correction_type + str(correction_deg)
            logging.warning(
                "Final segment correction %d/%d: %s.",
                attempt, ORBIT_MAX_CORRECTIONS, correction,
            )
            if not self._send(correction):
                return None

            corrected_deg = self._read_turn(correction)
            if corrected_deg is None:
                return None
            previous_error = abs(error_deg)
            actual_heading += corrected_deg
            residual_deg = expected_heading - actual_heading
            logging.info(
                "Final heading after %s: %+.1f° "
                "(target %+.1f°, residual %+.1f°).",
                correction, actual_heading, expected_heading, residual_deg,
            )
            if abs(residual_deg) >= previous_error:
                logging.warning(
                    "%s did not improve the heading; avoiding correction oscillation.",
                    correction,
                )
                break

        residual_deg = expected_heading - actual_heading
        if abs(residual_deg) <= ORBIT_HEADING_TOLERANCE_DEG + 1e-6:
            return actual_heading
        logging.warning(
            "Heading remains %+.1f° from target after %d corrections; "
            "continuing as configured.",
            residual_deg, ORBIT_MAX_CORRECTIONS,
        )
        return actual_heading

    def _check_protocol(self) -> bool:
        fields = self.stm.query_fields("?VER")
        if not fields:
            logging.error("No answer to ?VER. The STM is not talking — check the "
                          "cable, the port in .env, and that the board is powered.")
            return False

        name = fields[0] if fields else "?"
        try:
            version = int(fields[1])
        except (IndexError, ValueError):
            logging.error(f"Could not read a protocol version out of ?VER reply: {fields}")
            return False

        logging.info(f"STM identifies as '{name}', protocol v{version}.")
        if version < FU_MIN_PROTOCOL:
            logging.error(f"This firmware speaks protocol v{version}, but FU — which "
                          f"the whole approach depends on — needs v{FU_MIN_PROTOCOL}. "
                          f"Flash the current firmware.")
            return False
        if version != PROTOCOL_VERSION:
            logging.warning(f"Pi expects v{PROTOCOL_VERSION}, STM speaks v{version}. "
                            f"Continuing, but they should match.")
        return True

    def run(self) -> int:
        if not self.dry_run:
            try:
                self.stm.connect()
            except Exception as exc:
                logging.error(f"Could not open the serial link: {exc}")
                return 2

            if not self._check_protocol():
                self.stm.disconnect()
                return 2

            # Select the profile used to derive ORBIT_LINE rather than trusting
            # whichever profile a previous process left active.
            if not self.stm.set_profile(ARC_PROFILE):
                logging.error(f"Could not set arc profile to {ARC_PROFILE} — refusing "
                              "to run ORBIT_LINE against an unknown radius.")
                self.stm.disconnect()
                return 2
            logging.info(f"STM arc profile set to {ARC_PROFILE} for A5.")
            logging.info(
                "A5 compact orbit (equivalent %d cm inset), returning to %d cm "
                "for images: %s",
                ORBIT_INSET_CM, STANDOFF_CM, ORBIT_LINE,
            )

        try:
            self.camera = Camera()
        except Exception as exc:
            logging.error(f"Camera would not start: {exc}")
            if not self.dry_run:
                self.stm.disconnect()
            return 2

        try:
            if not self.approach():
                return 2

            for face in range(1, self.max_faces + 1):
                hit = self.look(face)
                if hit is LOOK_ERROR:
                    return 2
                if hit is not None:
                    name, conf = hit
                    logging.info("=" * 58)
                    logging.info(f"A.5 COMPLETE — valid image '{name}' "
                                 f"(conf={conf:.2f}) found on face {face} of "
                                 f"{self.max_faces}.")
                    logging.info("Annotated image saved on the PC under runs/predict/.")
                    logging.info("=" * 58)
                    return 0

                if face == self.max_faces:
                    break
                if not self.orbit(face):
                    return 2

            logging.warning(f"Went around all {self.max_faces} faces without finding "
                            f"a valid image. Either the standoff framing is off or "
                            f"the orbit is not landing square — check the stills in "
                            f"captured_images/.")
            return 1

        except KeyboardInterrupt:
            logging.warning("Interrupted — aborting the robot.")
            if not self.dry_run:
                self.stm.abort()
            return 2
        finally:
            if self.camera is not None:
                self.camera.stop_camera()
            if not self.dry_run:
                self.stm.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MDP checklist A.5 — navigate around the obstacle (RPi only)."
    )
    parser.add_argument("--dry-run", action="store_true",
                        help="Do not move. Proves the camera and YOLO work and "
                             "prints the lines that would have been sent.")
    parser.add_argument("--faces", type=int, default=MAX_FACES,
                        help=f"Faces to try before giving up (default {MAX_FACES}).")
    args = parser.parse_args()

    return TaskA5(dry_run=args.dry_run, max_faces=args.faces).run()


if __name__ == "__main__":
    sys.exit(main())
