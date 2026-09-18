"""
task_a5.py  —  MDP checklist A.5: navigate around the obstacle
───────────────────────────────────────────────────────────────
    "Demonstrate that your robot can navigate towards a given obstacle having
     a visual marker indicating obstacle. Your robot needs to navigate around
     the obstacle in search of face which has a valid image from the image
     list."

RPi ONLY. No PC, no Android, no pathfinding. Run it and it runs.

    python3 task_a5.py                 # full run
    python3 task_a5.py --dry-run       # no motion: proves link + camera + YOLO
    python3 task_a5.py --faces 2       # give up after 2 faces

Why this is not part of task1.py
─────────────────────────────────
task1.py is a planner-driven pipeline: Android supplies obstacles, the PC
returns a path, the Pi pumps segments. A.5 has none of that. It is one
obstacle and a reactive loop — look, decide, move — so it gets its own entry
point rather than a special case threaded through the Task 1 state machine.

How it works
────────────
1. APPROACH  — check something is actually in front (?US), then FU to a
               camera standoff. FU measures, drives, re-measures; it lands
               within about 2 cm and will correct BACKWARDS if it ends up too
               close, which is what makes step 3 repeatable.
2. LOOK      — capture a still, run YOLO on the Pi (vision.py).
               A bullseye is the MARKER, not a target: it means "right
               obstacle, wrong face". So is nothing at all.
3. ORBIT     — if the face was not a valid image, drive around to the next
               one and go back to step 2.
4. REPORT    — first valid image wins. Print it, keep the annotated JPEG.

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

# ── What counts as "a valid image from the image list" ────────────────────────
# Everything the model knows EXCEPT these. The bullseye is the marker that
# says "this is the obstacle" — finding it means you are at the right block
# and the wrong face, which is precisely the situation A.5 asks you to get out
# of. 'dot' is the non-symbol filler class and is not on the image list either.
NOT_A_TARGET = {"bullseye", "dot"}

# Below this, treat a detection as noise rather than a face. Deliberately low:
# a missed valid face costs a whole extra orbit leg, a false positive costs the
# run. Raise it if you see the model calling symbols out of thin air.
MIN_CONFIDENCE = float(os.getenv("A5_MIN_CONFIDENCE", "0.55"))

# ── Geometry ──────────────────────────────────────────────────────────────────
# Camera standoff. Far enough to frame the whole symbol, close enough that the
# symbol is big in the frame. 25 cm is a starting point — check what your lens
# actually sees before trusting it.
STANDOFF_CM = int(os.getenv("A5_STANDOFF_CM", "25"))

# How far the ultrasonic may read and still count as "the obstacle is there".
# Past this we assume we are pointed at a wall or at nothing useful.
MAX_APPROACH_CM = int(os.getenv("A5_MAX_APPROACH_CM", "150"))

# Faces to try before giving up. A block has four; trying all four returns you
# to where you started, so 4 is the natural cap.
MAX_FACES = int(os.getenv("A5_MAX_FACES", "4"))

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


class TaskA5:

    def __init__(self, dry_run: bool = False, max_faces: int = MAX_FACES):
        self.stm = STM()
        self.camera = None          # opened late: see run()
        self.dry_run = dry_run
        self.max_faces = max_faces

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

    def _us_cm(self):
        """Front distance in cm, or None if the sensor has nothing in view."""
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

        # 0xFFFF is the firmware's "no echo" sentinel, not a 655 metre wall.
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
        return self._stand_off(STANDOFF_CM)

    def look(self, face: int):
        """
        Photograph the face in front and decide whether it is a valid image.

        Returns (class_name, confidence) for a valid image, or None for
        "keep looking" — which covers a bullseye, a low-confidence guess, and
        nothing at all, because all three mean the same thing to A.5.
        """
        logging.info(f"── Looking at face {face} ──")

        path = self.camera.capture_image()
        if path is None:
            logging.error("Capture failed — treating this face as no-image.")
            return None

        import vision   # imported late: pulls in torch on first use
        try:
            name, conf = vision.detect(path)
        except Exception as exc:
            logging.error(f"Detection failed: {exc}")
            return None

        if name is None:
            logging.info("Nothing detected on this face.")
            return None
        if name in NOT_A_TARGET:
            logging.info(f"Found '{name}' (conf={conf:.2f}) — that is the marker, "
                         f"not a target. Wrong face.")
            return None
        if conf < MIN_CONFIDENCE:
            logging.info(f"Found '{name}' but only at conf={conf:.2f}, below "
                         f"{MIN_CONFIDENCE}. Not trusting it.")
            return None

        logging.info(f"Valid image: '{name}' at conf={conf:.2f}.")
        return name, conf

    def orbit(self, face: int) -> bool:
        """Drive around the obstacle to the next face, then re-acquire standoff."""
        logging.info(f"── Face {face} was not it — going around ──")

        if not self._send(ORBIT_LINE):
            return False

        # Let the chassis settle before the ultrasonic is asked to measure —
        # FU's first act is a stationary reading and it deserves a still robot.
        sleep(0.3)
        return self._stand_off(STANDOFF_CM)

    # ── Run ───────────────────────────────────────────────────────────────────

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
        """Returns a process exit code: 0 found it, 1 did not, 2 broke."""
        if not self.dry_run:
            try:
                self.stm.connect()
            except Exception as exc:
                logging.error(f"Could not open the serial link: {exc}")
                return 2

            if not self._check_protocol():
                self.stm.disconnect()
                return 2

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
                if hit is not None:
                    name, conf = hit
                    logging.info("=" * 58)
                    logging.info(f"A.5 COMPLETE — valid image '{name}' "
                                 f"(conf={conf:.2f}) found on face {face} of "
                                 f"{self.max_faces}.")
                    logging.info("Annotated image saved under runs/predict/.")
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
