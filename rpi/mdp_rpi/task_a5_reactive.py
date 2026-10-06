"""
task_a5_reactive.py  —  MDP checklist A.5: navigate around the obstacle
───────────────────────────────────────────────────────────────
    "Demonstrate that your robot can navigate towards a given obstacle having
     a visual marker indicating obstacle. Your robot needs to navigate around
     the obstacle in search of face which has a valid image from the image
     list."

No Android, no pathfinding, no obstacle list. One obstacle and a reactive
loop — look, decide, move — so it gets its own entry point rather than a
special case threaded through the Task 1 state machine.

    RPi:  python3 task_a5_reactive.py
    PC:   cd pc_side && python3 task1_pc.py      ← start this FIRST

Detection runs on the PC, exactly as it does in Task 1
───────────────────────────────────────────────────────
Camera.capture_image() returns JPEG bytes and writes nothing to disk, and
pc.send_image() takes those bytes directly. The PC's task1_pc.py answers a
DETECT with an OBJECT whether or not it found anything, and it handles DETECT
independently of OBSTACLES/PATH — so A.5 can use the detection half of that
server and ignore the pathfinding half entirely.

That also means A.5 recognises through the SAME tuned pipeline as Task 1
(centre crop, imgsz 960, conf 0.25) instead of a second copy that drifts, and
nothing needs ultralytics or torch installed on the Pi.

The cost is that the PC must be up. If you want A.5 to run standalone one day,
the change is to install ultralytics on the Pi and swap look()'s two pc calls
for a local detect() — nothing else in this file moves.

Modes
──────
    --dry-run      no motion. Camera and PC still run, so this proves the
                   capture → DETECT → OBJECT chain without touching the floor.
    --no-detect    no camera, no PC. Every face reports "not it", so the robot
                   walks the full orbit — this is how you tune ORBIT_LINE with
                   a tape measure before you care about images.
    --faces N      give up after N faces (a block has four).

How it works
─────────────
1. APPROACH  — check something is actually in front (?US), then FU to a
               camera standoff. FU measures, drives, re-measures; it lands
               within about 2 cm and will correct BACKWARDS if it ends up too
               close, which is what makes step 3 repeatable.
2. LOOK      — capture a still, send it to the PC, read the class back.
               A bullseye is the MARKER, not a target: it means "right
               obstacle, wrong face". So is nothing at all.
3. ORBIT     — if the face was not a valid image, drive around to the next
               one and go back to step 2.
4. REPORT    — first valid image wins.

The geometry is the part you will tune
───────────────────────────────────────
Every turn is an arc of radius ~291 mm (TIGHT profile) — the chassis is
Ackermann and cannot turn on the spot. A 90 degree arc therefore eats ~291 mm
in BOTH axes, which is enormous next to a 10 cm obstacle. ORBIT_LINE below is
reasoned from that geometry (see the comment on it) but the real numbers
depend on your robot, so it is a single editable constant rather than logic
spread through the file.

What saves you: FU re-acquires the standoff at every face. Distance errors do
not accumulate — only the LATERAL alignment does, and that is the one number
worth a tape measure.
"""

import argparse
import logging
import os
import sys
from time import sleep
from typing import Optional, Tuple

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

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [A5] %(levelname)s — %(message)s",
    datefmt="%H:%M:%S",
)

# ── What counts as "a valid image from the image list" ────────────────────────
# Everything the model knows EXCEPT these. The bullseye is the marker that
# says "this is the obstacle" — finding it means you are at the right block
# and the wrong face, which is precisely the situation A.5 asks you to get out
# of. 'dot' is the non-symbol filler class and is not on the image list either.
# NONE is what task1_pc.py sends back when YOLO found nothing at all.
NOT_A_TARGET = {"bullseye", "dot", "none"}

# Below this, treat a detection as noise rather than a face. This sits ON TOP
# of the 0.25 floor already applied inside pc_side/image_recognition/detect.py.
# Deliberately low: a missed valid face costs a whole extra orbit leg, a false
# positive costs the run. Raise it if you see the model calling symbols out of
# thin air.
MIN_CONFIDENCE = float(os.getenv("A5_MIN_CONFIDENCE", "0.55"))

# ── Geometry ──────────────────────────────────────────────────────────────────
# Camera standoff. Far enough to frame the whole symbol, close enough that the
# symbol is big in the frame. 25 cm is a starting point — check what your lens
# actually sees before trusting it, and remember detect.py crops to the central
# 70% x 80% before inference.
STANDOFF_CM = int(os.getenv("A5_STANDOFF_CM", "25"))

# How far the ultrasonic may read and still count as "the obstacle is there".
# Past this we assume we are pointed at a wall or at nothing useful.
MAX_APPROACH_CM = int(os.getenv("A5_MAX_APPROACH_CM", "150"))

# Faces to try before giving up. A block has four; trying all four returns you
# to where you started, so 4 is the natural cap.
MAX_FACES = int(os.getenv("A5_MAX_FACES", "4"))

# Settle before FU's first stationary reading. FU opens by standing still to
# measure, and it deserves a robot that has stopped rocking.
SETTLE_S = float(os.getenv("A5_SETTLE_S", "0.3"))

# The arc profile ORBIT_LINE was traced for. 0 TIGHT (r=291mm) is the firmware
# default and nothing here sends !PROF, so this is a check rather than a
# setting: planning for one radius and driving another puts every turn wide,
# and across four arcs that is the difference between facing the next face and
# facing past it.
ARC_PROFILE = int(os.getenv("A5_ARC_PROFILE", "0"))
PROFILE_RADII_MM = {0: 291, 1: 318, 2: 306}

# ── The orbit: one face to the next ───────────────────────────────────────────
# Traced on paper for a 291 mm arc radius, standoff 25 cm, 10 cm block, with
# the robot starting square on to a face. Origin at the obstacle centre, robot
# starting at (0, -30) facing +y:
#
#   R20    back off to make room for the first arc  -> (0.0, -50.0) facing +y
#   FR90   swing out, now running alongside         -> (29.1, -20.9) facing +x
#   FL90   turn back to parallel                    -> (58.2,   8.2) facing +y
#   R37    straight reverse: cancels the northward  -> (58.2, -28.8) facing +y
#          drift the two arcs piled up
#   FL90   turn in to face the next face            -> (29.1,   0.3) facing -x
#
# Lands ~29 cm out from the obstacle centre, square on to the next face, which
# the FU that follows then trims to the real standoff.
#
# Sent as ONE line: five primitives, well inside the 16-primitive / 128-byte
# cap, and one round trip instead of five. We do not need the intermediate
# poses — if any of it fails we abort the run anyway.
#
# Straight reverse (R) is used rather than a reverse ARC (RR/RL) on purpose:
# reverse arcs are implemented in the firmware but have never been run on the
# floor, and the sign convention there is still marked "VERIFY THIS ON THE
# ROBOT". A tighter segmented version is possible once they are trusted.
ORBIT_LINE = os.getenv("A5_ORBIT", "R20,FR90,FL90,R37,FL90")


class PCLinkLost(RuntimeError):
    """
    The PC stopped answering, or answered something unreadable.

    Raised rather than folded into "no valid image on this face", because the
    two want opposite responses: a wrong face means orbit and look again, a
    dead PC means every remaining face would be looked at blind. The run ends.
    """


class TaskA5:
    """
    The A.5 run.

    The three peripherals are injected rather than constructed here so that
    test_task_a5_reactive.py can drive the whole sequence with fakes on a laptop. run()
    builds the real ones when they were not supplied.
    """

    def __init__(self, dry_run: bool = False, detect: bool = True,
                 max_faces: int = MAX_FACES, standoff_cm: int = STANDOFF_CM,
                 stm=None, camera=None, pc=None):
        self.dry_run = dry_run
        self.detect = detect
        self.max_faces = max_faces
        self.standoff_cm = standoff_cm

        self.stm = stm
        self.camera = camera
        self.pc = pc

        # Whether WE opened it, and therefore whether we should close it.
        self._owns_stm = stm is None
        self._owns_camera = camera is None
        self._owns_pc = pc is None

    # ── STM helpers ───────────────────────────────────────────────────────────

    def _send(self, line: str) -> bool:
        """
        Send one line and wait for the whole line to finish.

        Returns True only on OK. Every other outcome is fatal to the run: a
        FAIL means the robot is not where we think it is, and continuing to
        drive from a pose we have lost is how you put it into a wall.
        """
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
            # Not retried here. A RESEND is a PARSE error, so the same bytes
            # would earn the same answer; the line is wrong, not unlucky.
            logging.error(f"STM rejected '{line}' as malformed (RESEND).")
            return False

        logging.error(f"STM reported {reply} on '{line}'. The pose is no longer "
                      f"trustworthy — stopping.")
        return False

    def _us_cm(self) -> Optional[int]:
        """Front distance in cm, or None if the sensor has nothing in view."""
        if self.dry_run:
            return self.standoff_cm

        fields = self.stm.query_fields("?US")
        if not fields:
            return None
        try:
            value = int(fields[0])
        except ValueError:
            logging.warning(f"Unparseable ?US reply: {fields}")
            return None

        # 65535 is the firmware's "no echo" sentinel, not a 655 metre wall.
        if value == 0xFFFF:
            return None
        return value

    def _stand_off(self, target_cm: int) -> bool:
        """
        Put the robot target_cm from whatever is in front, using FU.

        FU is asked for target + US_BIAS_CM because the sensor reads about
        1.3 cm long and every correction pass reads through that same offset,
        so it cannot average out. Asking for 26 is how you end up at 25.
        """
        wanted = int(round(target_cm + US_BIAS_CM))
        if not (FU_MIN_CM <= wanted <= FU_MAX_CM):
            logging.error(f"Standoff {target_cm} cm becomes FU{wanted}, outside "
                          f"{FU_MIN_CM}..{FU_MAX_CM}.")
            return False

        # FU refuses to drive at something it cannot see and answers
        # FAIL,NOECHO. Checking first turns that into a clear message instead
        # of a failed run.
        seen = self._us_cm()
        if seen is None:
            logging.error("Ultrasonic sees nothing ahead — cannot stand off. Is the "
                          "robot pointed at the obstacle?")
            return False
        if seen > MAX_APPROACH_CM:
            logging.error(f"Nearest thing ahead is {seen} cm away, past the "
                          f"{MAX_APPROACH_CM} cm limit. That is not the obstacle.")
            return False

        logging.info(f"Ultrasonic reads {seen} cm — closing to {target_cm} cm "
                     f"(FU{wanted}).")
        return self._send(f"FU{wanted}")

    # ── The three steps ───────────────────────────────────────────────────────

    def approach(self) -> bool:
        """Drive at the obstacle and stop a camera's distance from it."""
        logging.info("── Approaching the obstacle ──")
        return self._stand_off(self.standoff_cm)

    def look(self, face: int) -> Optional[Tuple[str, float]]:
        """
        Photograph the face in front and decide whether it is a valid image.

        Returns (class_name, confidence) for a valid image, or None for
        "keep looking" — which covers a bullseye, a low-confidence guess, and
        nothing at all, because all three mean the same thing to A.5.
        """
        if not self.detect:
            logging.info(f"── Face {face}: detection disabled (--no-detect) ──")
            return None

        logging.info(f"── Looking at face {face} ──")

        image = self.camera.capture_image()
        if not image:
            logging.error("Capture failed — treating this face as no-image.")
            return None

        # The tag is echoed back in the OBJECT reply and is what task1_pc.py
        # names the saved JPEG after, so per-face tags give you four separately
        # named stills to put in the report.
        tag = f"A5F{face}"
        self.pc.send(f"DETECT,{tag}")
        self.pc.send_image(image)

        name, conf = self._await_object(tag)

        if name.lower() in NOT_A_TARGET:
            logging.info(f"Found '{name}' (conf={conf:.2f}) — that is the marker or "
                         f"nothing at all, not a target. Wrong face.")
            return None
        if conf < MIN_CONFIDENCE:
            logging.info(f"Found '{name}' but only at conf={conf:.2f}, below "
                         f"{MIN_CONFIDENCE}. Not trusting it.")
            return None

        logging.info(f"Valid image: '{name}' at conf={conf:.2f}.")
        return name, conf

    def _await_object(self, tag: str) -> Tuple[str, float]:
        """
        Block until the PC answers this DETECT, and return (class, confidence).

        Deliberately blocking with no timeout. task1_pc.py answers EVERY DETECT,
        sending OBJECT,<tag>,0.0,NONE when the receive failed or YOLO found
        nothing, so a silence here means the PC has died rather than that it is
        thinking — and pc.receive() already reports a dropped socket as None.
        Raises PCLinkLost on a dropped link or an unreadable reply.
        """
        while True:
            msg = self.pc.receive()
            if msg is None:
                raise PCLinkLost("PC link dropped while waiting for OBJECT.")

            if not msg.upper().startswith("OBJECT"):
                # A.5 never sends OBSTACLES, so nothing else should arrive. Say
                # so rather than swallowing it: an unexpected PATH here means
                # task1.py is running against the same server.
                logging.warning(f"Ignoring unexpected message from PC: '{msg}'.")
                continue

            parts = [p.strip() for p in msg.split(",")]
            if len(parts) < 4:
                raise PCLinkLost(f"Malformed OBJECT reply '{msg}' — expected "
                                 f"OBJECT,<id>,<confidence>,<class>.")

            if parts[1] != tag:
                logging.warning(f"OBJECT is tagged '{parts[1]}' but we asked about "
                                f"'{tag}' — using it anyway, one reply behind is "
                                f"worse than one reply late.")

            try:
                confidence = float(parts[2])
            except ValueError:
                raise PCLinkLost(f"Unparseable confidence in '{msg}'.")

            return parts[3], confidence

    def orbit(self, face: int) -> bool:
        """Drive around the obstacle to the next face, then re-acquire standoff."""
        logging.info(f"── Face {face} was not it — going around ──")

        if not self._send(ORBIT_LINE):
            return False

        # Let the chassis settle before the ultrasonic is asked to measure —
        # FU's first act is a stationary reading and it deserves a still robot.
        sleep(SETTLE_S)
        return self._stand_off(self.standoff_cm)

    # ── Startup ───────────────────────────────────────────────────────────────

    def _check_protocol(self) -> bool:
        fields = self.stm.query_fields("?VER")
        if not fields:
            logging.error("No answer to ?VER. The STM is not talking — check the "
                          "cable, the port in .env, and that the board is powered. "
                          "USB Port 1 is the download port and never transmits.")
            return False

        name = fields[0]
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

    def _check_profile(self) -> None:
        """
        Warn if the robot is not on the arc profile ORBIT_LINE was traced for.

        Not fatal — someone may have changed it on purpose — but it is the
        first thing to suspect when the orbit lands crooked, and finding it
        here costs one query instead of an afternoon.
        """
        fields = self.stm.query_fields("?STAT")
        if not fields or len(fields) < 4:
            logging.warning("No usable ?STAT reply — cannot confirm the arc profile.")
            return
        try:
            profile = int(fields[3])
        except ValueError:
            logging.warning(f"Unparseable profile in ?STAT reply: {fields}")
            return

        if profile != ARC_PROFILE:
            logging.warning(
                f"Robot is on arc profile {profile} "
                f"(r≈{PROFILE_RADII_MM.get(profile, '?')}mm) but A5_ORBIT was "
                f"traced for profile {ARC_PROFILE} "
                f"(r≈{PROFILE_RADII_MM.get(ARC_PROFILE, '?')}mm). Every arc will "
                f"land wide or tight by the difference. Re-trace A5_ORBIT or put "
                f"the robot back on profile {ARC_PROFILE}."
            )

    def _open(self) -> Optional[int]:
        """
        Bring up whatever this run actually needs. Returns an exit code on
        failure, or None when everything asked for is open.

        Imports are late and local: picamera2 only exists on the Pi, and
        --no-detect deliberately runs without a camera at all.
        """
        if not self.dry_run:
            if self.stm is None:
                self.stm = STM()
                try:
                    self.stm.connect()
                except Exception as exc:
                    logging.error(f"Could not open the serial link: {exc}")
                    return 2
            # Checked whoever opened the link: FU is the one thing this whole
            # task stands on, and finding out it is missing halfway through an
            # approach is finding out too late.
            if not self._check_protocol():
                return 2
            self._check_profile()

        if self.detect:
            if self.camera is None:
                try:
                    from image_capture.camera import Camera
                    self.camera = Camera()
                except Exception as exc:
                    logging.error(f"Camera would not start: {exc}")
                    return 2

            if self.pc is None:
                try:
                    from communications.pc import PC
                    self.pc = PC()
                    logging.info("Waiting for the PC — start pc_side/task1_pc.py now.")
                    self.pc.connect()
                # SystemExit is caught on purpose: pc.py calls sys.exit(1) when
                # the bind fails, and letting that through would report a
                # busy port as "went round without finding an image".
                except (Exception, SystemExit) as exc:
                    logging.error(f"Could not accept a PC connection: {exc}")
                    return 2

        return None

    def _close(self) -> None:
        if self.camera is not None and self._owns_camera:
            try:
                self.camera.stop_camera()
            except Exception as exc:
                logging.warning(f"Camera would not close cleanly: {exc}")
        if self.pc is not None and self._owns_pc:
            self.pc.disconnect()
        if self.stm is not None and self._owns_stm and not self.dry_run:
            self.stm.disconnect()

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self) -> int:
        """Returns a process exit code: 0 found it, 1 did not, 2 broke."""
        failed = self._open()
        if failed is not None:
            self._close()
            return failed

        try:
            if not self.approach():
                return 2

            for face in range(1, self.max_faces + 1):
                hit = self.look(face)
                if hit is not None:
                    name, conf = hit
                    logging.info("=" * 58)
                    logging.info(f"A.5 COMPLETE — valid image '{name}' "
                                 f"(conf={conf:.2f}) found on face {face} of "
                                 f"{self.max_faces}.")
                    logging.info("Annotated image is on the PC under "
                                 "pc_side/runs/predict/.")
                    logging.info("=" * 58)
                    return 0

                if face == self.max_faces:
                    break
                if not self.orbit(face):
                    return 2

            logging.warning(f"Went around all {self.max_faces} faces without finding "
                            f"a valid image. Either the standoff framing is off or "
                            f"the orbit is not landing square — check the stills the "
                            f"PC saved under pc_side/received_images/.")
            return 1

        except PCLinkLost as exc:
            logging.error(f"{exc} Stopping — looking at the remaining faces without "
                          f"a detector would just drive the robot in a circle.")
            return 2

        except KeyboardInterrupt:
            logging.warning("Interrupted — aborting the robot.")
            if not self.dry_run and self.stm is not None:
                self.stm.abort()
            return 2
        finally:
            self._close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MDP checklist A.5 — navigate around the obstacle."
    )
    parser.add_argument("--dry-run", action="store_true",
                        help="Do not move. Still captures and detects, so it proves "
                             "the camera and the PC link, and prints the lines that "
                             "would have been sent.")
    parser.add_argument("--no-detect", action="store_true",
                        help="Motion only: no camera, no PC. Every face reports "
                             "'not it', so the robot walks the whole orbit. Use this "
                             "to tune A5_ORBIT with a tape measure.")
    parser.add_argument("--faces", type=int, default=MAX_FACES,
                        help=f"Faces to try before giving up (default {MAX_FACES}).")
    parser.add_argument("--standoff", type=int, default=STANDOFF_CM,
                        help=f"Camera standoff in cm (default {STANDOFF_CM}).")
    args = parser.parse_args()

    if args.dry_run and args.no_detect:
        logging.error("--dry-run with --no-detect would do nothing at all: no motion "
                      "and no detection. Pick one.")
        return 2

    return TaskA5(
        dry_run=args.dry_run,
        detect=not args.no_detect,
        max_faces=args.faces,
        standoff_cm=args.standoff,
    ).run()


if __name__ == "__main__":
    sys.exit(main())
