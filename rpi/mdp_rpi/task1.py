"""
task1.py  —  RPi orchestrator for MDP Task 1 (Image Recognition)
─────────────────────────────────────────────────────────────────
Architecture
─────────────────────────────────────────────────────────────────
Three persistent threads, one per communication channel:

  android_thread  ←→  Android tablet  (Bluetooth RFCOMM)
  pc_thread       ←→  PC / algorithm server  (TCP socket)
  stm_thread      ←→  STM32 motor controller  (serial UART)

Shared state is protected with a threading.Lock (≡ pthread_mutex_t).
Cross-thread signalling uses threading.Event (≡ binary semaphore).

Task 1 message flow
────────────────────
1. Android sends OBSTACLE,<id>,<x>,<y>,<facing>  (one per obstacle)
2. 1s debounce fires  →  RPi sends OBSTACLES,<json> to PC
3. Android sends PATH or BEGIN  →  RPi (re-)sends OBSTACLES if needed
4. PC replies  PATH,<json>  →  RPi stores segments, sets path_ready
5. Android sends BEGIN  →  RPi sends first STM segment
6. STM replies OK  →  RPi sends DETECT,<obstacle_id> to PC
                    →  RPi captures image, sends filename + bytes to PC
7. PC replies  OBJECT,<obstacle_id>,<confidence>,<class_id>
                    →  RPi forwards  TARGET,<obstacle_id>,<class_id>  to Android
                    →  RPi sends next STM segment (if any left)
8. After last segment  →  RPi sends STITCH,<n> to PC

Thread-safety notes
────────────────────
  _idx_lock     guards  segments, segments_index, obstacle_order, directions
  image_done    Event: stm_thread produces (waits for result), pc_thread sets it
  path_ready    Event: android_thread waits on it; pc_thread sets it
"""

import json
import fcntl
import logging
import os
from threading import Event, Lock, Thread, Timer
from time import sleep

from dotenv import load_dotenv

load_dotenv()

from communications.android import Android
from communications.pc import PC
from communications.stm import (
    CAL_MIN_PROTOCOL,
    FU_MIN_PROTOCOL,
    PROTOCOL_VERSION,
    STM,
    validate_line,
)
from image_capture.camera import Camera

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(threadName)s] %(levelname)s — %(message)s",
    datefmt="%H:%M:%S",
)


class Task1:
    """Orchestrates the three-thread pipeline for Task 1."""

    # ── Init ──────────────────────────────────────────────────────────────────

    def __init__(self):
        # Communication objects
        self.android = Android()
        self.pc = PC()
        self.stm = STM()
        self.camera = Camera()

        # Thread handles
        self.android_thread: Thread | None = None
        self.pc_thread: Thread | None = None
        self.stm_thread: Thread | None = None

        # ── Obstacle / path state ─────────────────────────────────────────────
        self.obstacles: list = []           # list of obstacle dicts from Android
        self.segments: list = []            # movement segments from PC
        self.segments_index: int = 0        # which segment we are up to
        self.obstacle_order: list = []      # obstacle IDs in visit order
        self.directions: list = []          # direction info per segment (for Android map)
        self.direction_index: int = 0

        # Per-segment obstacle mapping. A segment is NOT 1:1 with an obstacle any
        # more: PROTOCOL.md §2 caps a line at 16 primitives / 128 bytes, so one
        # obstacle approach may be split across several segments, and some
        # segments are pure travel with no photo at the end. Entry is an
        # obstacle id, or None for "no detection after this segment".
        self.segment_obstacles: list = []

        self.started: bool = False          # True after Android sends BEGIN
        self.path_requested: bool = False   # True while OBSTACLES request is in-flight
        self.halted: bool = False           # True after FAIL,* or exhausted RESENDs

        # PROTOCOL.md §3: cap RESEND retries. A permanently malformed segment
        # retried forever is an infinite loop in which the robot never moves.
        self._resend_counts: dict = {}

        # ── Synchronisation primitives ────────────────────────────────────────
        # Mutex for all mutable index / segment state
        self._idx_lock = Lock()
        # Set by pc_thread when OBJECT arrives; waited on by stm_thread DETECT logic
        self.image_done = Event()
        # Set by pc_thread when PATH arrives; waited on by android_thread BEGIN logic
        self.path_ready = Event()
        # Debounce timer — send OBSTACLES to PC 1 s after last obstacle arrives
        self._debounce_lock = Lock()
        self._calc_timer: Timer | None = None

        # ── Tunable config from .env ──────────────────────────────────────────
        self._debounce_delay = float(os.getenv("DEBOUNCE_DELAY_S", "1.0"))
        self.segment_delay = float(os.getenv("SEGMENT_DELAY_S", "0.5"))
        self.detect_retries = int(os.getenv("DETECT_RETRY_COUNT", "1"))
        self.detect_retry_delay = float(os.getenv("DETECT_RETRY_DELAY_S", "0.2"))
        self.detect_timeout = float(os.getenv("DETECT_TIMEOUT_S", "0.5"))
        # PROTOCOL.md §3 recommends at most three retransmissions.
        self.max_resends = int(os.getenv("STM_MAX_RESENDS", "3"))
        # PROTOCOL.md §6: 0 TIGHT (r=291mm), 1 CLEAN (r=318mm), 2 SLOW (r=306mm).
        # The firmware's own default is TIGHT, and that choice belongs to the
        # firmware — this client does not override it. Unset (the default here)
        # means send no !PROF at all and run whatever the STM already selected.
        # Set STM_ARC_PROFILE only to deliberately ask for a different one.
        _profile_env = os.getenv("STM_ARC_PROFILE")
        self.arc_profile = int(_profile_env) if _profile_env not in (None, "") else None

        logging.info(
            f"Config — segment_delay={self.segment_delay}s  "
            f"detect_retries={self.detect_retries}  "
            f"detect_retry_delay={self.detect_retry_delay}s  "
            f"detect_timeout={self.detect_timeout}s  "
            f"debounce_delay={self._debounce_delay}s  "
            f"max_resends={self.max_resends}  "
            f"arc_profile={self.arc_profile}"
        )

    # ── Path calculation helpers ───────────────────────────────────────────────

    def _schedule_path_calculation(self) -> None:
        """
        Debounced trigger: wait _debounce_delay seconds after the LAST obstacle
        arrives before sending OBSTACLES to PC.  Any new obstacle resets the timer.
        This prevents hammering PC with partial obstacle lists.
        """
        with self._debounce_lock:
            if self._calc_timer is not None:
                self._calc_timer.cancel()
            self._calc_timer = Timer(self._debounce_delay, self._request_path_from_pc)
            self._calc_timer.daemon = True
            self._calc_timer.start()
        logging.info(f"Path calculation scheduled in {self._debounce_delay}s…")

    def _request_path_from_pc(self) -> bool:
        """Send current obstacle list to PC for pathfinding.  Idempotent."""
        if self.path_requested:
            logging.info("PATH already in-flight — skipping duplicate OBSTACLES send.")
            return False
        payload = "OBSTACLES," + json.dumps(self.obstacles) + "\n"
        self.pc.send(payload)
        self.path_requested = True
        logging.info(f"Sent OBSTACLES to PC ({len(self.obstacles)} obstacle(s)).")
        return True

    # ── STM segment helpers ────────────────────────────────────────────────────

    def _obstacle_for_segment(self, seg_index: int):
        """
        Which obstacle (if any) we should photograph after segment `seg_index`.

        Prefers the explicit per-segment map. Falls back to the old positional
        convention (segment i <-> obstacle_order[i]) so a PC still sending the
        original PATH shape keeps working.
        """
        if self.segment_obstacles:
            if 0 <= seg_index < len(self.segment_obstacles):
                value = self.segment_obstacles[seg_index]
                return None if value is None else str(value)
            return None
        if 0 <= seg_index < len(self.obstacle_order):
            return str(self.obstacle_order[seg_index])
        return None

    def _send_next_segment(self) -> bool:
        """
        Atomically read-and-advance segments_index, then send the segment to STM.
        Returns True if a segment was sent, False if we are past the end or the
        mission is halted.

        Uses stm.send_line(), which validates against PROTOCOL.md §2/§4 and
        sends nothing if the segment is malformed — catching it here costs
        nothing, catching it on the wire costs a RESEND round trip.
        """
        if self.halted:
            logging.warning("Segment pump is halted — not sending.")
            return False

        malformed = None

        # The index advances ONLY on a successful write. Advancing first and
        # rolling back on failure would race: three threads can reach this
        # (Android on BEGIN, PC on PATH, STM on OK/RESEND), and a rolled-back
        # index could silently skip a segment.
        with self._idx_lock:
            if self.segments_index < 0 or self.segments_index >= len(self.segments):
                return False
            seg = self.segments[self.segments_index]
            sent_index = self.segments_index

            ok, reason = validate_line(list(seg))
            if ok:
                if not self.stm.send_line(seg):
                    # Refused by the §2 in-flight guard: a previous line has not
                    # been answered yet. NOT a halt — the outstanding reply will
                    # drive the pump forward on its own. Dropping this attempt is
                    # exactly the right outcome.
                    logging.warning(
                        f"Segment {sent_index} not sent — previous line still "
                        "awaiting its reply (PROTOCOL.md §2). Ignoring this trigger."
                    )
                    return False
                self.segments_index += 1
            else:
                malformed = reason

        if malformed is not None:
            # A segment the firmware could never parse. Retrying it would just
            # burn the RESEND budget, so stop here instead.
            self._halt_mission(
                f"segment {sent_index} is malformed: {seg} — {malformed}"
            )
            return False

        logging.info(
            f"STM segment {self.segments_index}/{len(self.segments)} sent: {seg}"
        ) 
        # for individual segments when they start running
        try:
            self.android.send(f"STATUS,RUNNING,{self.segments_index},{len(self.segments)}")
        except OSError as exc:
            logging.warning(f"Could not notify Android of running status: {exc}")
        return True

    # ── Failure handling ───────────────────────────────────────────────────────

    def _halt_mission(self, reason: str) -> None:
        """
        Stop the segment pump and report.

        PROTOCOL.md §3: FAIL,* means "the robot is not where you think it is".
        Continuing to feed segments to a robot whose pose is unknown drives it
        into obstacles, so the pump stops here and waits for a human or a
        re-plan rather than pressing on.
        """
        if self.halted:
            return
        self.halted = True
        logging.error(f"MISSION HALTED — {reason}")

        # Where does the robot actually think it is? Queries bypass the movement
        # queue (§5) so this is safe even if a move is still running, and the
        # answer is exactly what a re-plan needs.
        pose = self.stm.query("?POSE")
        stat = self.stm.query("?STAT")
        logging.error(f"Halt diagnostics — {pose or 'POSE unavailable'} | {stat or 'STAT unavailable'}")

        try:
            self.android.send("STATUS,FAILED")
        except OSError as exc:
            logging.error(f"Could not notify Android of halt: {exc}")

    # ── Image detection with retry ────────────────────────────────────────────

    def _detect_and_send_image(self, obstacle_id: str) -> bool:
        """
        Full image capture + transfer + wait-for-result cycle with retry.

        Steps per attempt:
          1. Capture JPEG bytes from camera.
          2. Tell PC a DETECT is coming: DETECT,<id>
          3. Send a 4-byte size header then raw bytes.
          4. Wait up to detect_timeout seconds for pc_thread to set image_done.
          5. Retry up to detect_retries times if no response.

        Returns True if OBJECT was received, False after all attempts exhausted.
        """
        total_attempts = self.detect_retries + 1

        for attempt in range(1, total_attempts + 1):
            # ── Capture ──────────────────────────────────────────────────────
            image_bytes = self.camera.capture_image()
            if not image_bytes:
                logging.error(f"DETECT attempt {attempt}: camera capture failed — skipping.")
                continue

            # ── Signal + send ─────────────────────────────────────────────────
            self.image_done.clear()
            self.pc.send(f"DETECT,{obstacle_id}\n")
            logging.info(
                f"DETECT sent for obstacle {obstacle_id} "
                f"(attempt {attempt}/{total_attempts})."
            )

            # Send actual image bytes
            self.pc.send_image(image_bytes)

            # ── Wait for result ───────────────────────────────────────────────
            if self.image_done.wait(timeout=self.detect_timeout):
                logging.info(
                    f"OBJECT received for obstacle {obstacle_id} "
                    f"on attempt {attempt}."
                )
                return True

            logging.warning(
                f"No OBJECT for obstacle {obstacle_id} within "
                f"{self.detect_timeout}s (attempt {attempt}/{total_attempts})."
            )
            if attempt < total_attempts and self.detect_retry_delay > 0:
                sleep(self.detect_retry_delay)

        logging.warning(
            f"Gave up waiting for OBJECT for obstacle {obstacle_id} "
            f"after {total_attempts} attempt(s)."
        )
        return False

    # ══ Thread: Android receive ═══════════════════════════════════════════════

    def android_receive(self) -> None:
        """
        Runs in android_thread.
        Handles: OBSTACLE, CLEAR, PATH/ALG (send data), BEGIN.
        """
        while True:
            try:
                msg = self.android.receive()
                if not msg:
                    continue

                parts = msg.strip().split(",")
                tag = parts[0].strip().upper()

                if tag == "OBSTACLE" and len(parts) >= 5:
                    # ── New obstacle from Android ─────────────────────────────
                    facing_map = {
                        "N": 0, "NORTH": 0,
                        "E": 2, "EAST": 2,
                        "S": 4, "SOUTH": 4,
                        "W": 6, "WEST": 6,
                        "SKIP": 8,
                    }
                    offset = 1
                    if parts[1].strip().upper() == "UPSERT" and len(parts) >= 6:
                        offset = 2
                    elif parts[1].strip().upper() == "REMOVE":
                        obstacle_id = int(parts[2])
                        before = len(self.obstacles)
                        self.obstacles = [
                            ob for ob in self.obstacles if ob.get("id") != obstacle_id
                        ]
                        self.path_ready.clear()
                        self.path_requested = False
                        logging.info(
                            f"Android: removed obstacle {obstacle_id} "
                            f"({before - len(self.obstacles)} entr{'y' if before - len(self.obstacles) == 1 else 'ies'})."
                        )
                        continue

                    obstacle = {
                        "id":  int(parts[offset]),
                        "x":   int(parts[offset + 1]) / 10,
                        "y":   int(parts[offset + 2]) / 10,
                        "d":   facing_map.get(parts[offset + 3].strip().upper(), 0),
                    }
                    self.obstacles = [
                        ob for ob in self.obstacles if ob.get("id") != obstacle["id"]
                    ]
                    self.obstacles.append(obstacle)
                    self.path_ready.clear()
                    self.path_requested = False
                    logging.info(f"Android: added obstacle {obstacle}.")
                    # Debounce: send to PC 1 s after last obstacle
                    self._schedule_path_calculation()

                elif tag == "CLEAR":
                    # ── User cleared the map ──────────────────────────────────
                    with self._debounce_lock:
                        if self._calc_timer:
                            self._calc_timer.cancel()
                            self._calc_timer = None
                    self.obstacles.clear()
                    self.obstacle_order.clear()
                    self.path_ready.clear()
                    self.path_requested = False
                    logging.info("Android: obstacles cleared.")

                elif tag in ("PATH", "ALG") or msg.startswith("ALG|"):
                    # ── User pressed "Send Data" button ───────────────────────
                    if self.started and self.path_ready.is_set():
                        logging.info(
                            "Android: PATH request ignored — mission already running."
                        )
                    else:
                        with self._debounce_lock:
                            if self._calc_timer:
                                self._calc_timer.cancel()
                                self._calc_timer = None
                        self.path_requested = False   # force fresh request
                        self._request_path_from_pc()

                elif tag == "BEGIN":
                    # ── User pressed Start ────────────────────────────────────
                    if not self.started:
                        logging.info("Android: BEGIN received — mission starting.")
                        self.started = True

                    with self._idx_lock:
                        self.segments_index = 0
                        # A fresh BEGIN clears a previous halt: the operator has
                        # seen the failure and is restarting deliberately.
                        self.halted = False
                        self._resend_counts.clear()

                    if not self.path_ready.is_set():
                        logging.info(
                            "Android: BEGIN received but PATH not ready yet — "
                            "will send first segment once PATH arrives."
                        )
                        continue

                    with self._idx_lock:
                        if self.directions:
                            start_pose = self.directions[0]
                            try:
                                self.android.send(
                                    f"STATUS,START,{start_pose['x']},{start_pose['y']},{start_pose['dir']}"
                                )
                            except OSError as exc:
                                logging.warning(f"Could not notify Android of start position: {exc}")

                    if not self._send_next_segment():
                        logging.warning("Android: BEGIN received but no segments to send.")

                else:
                    logging.warning(f"Android: unrecognised message '{msg}' — ignoring.")

            except OSError as exc:
                logging.error(f"Android thread OSError: {exc}")
            except Exception as exc:
                logging.exception(f"Android thread error while handling message: {exc}")

    # ══ Thread: PC receive ════════════════════════════════════════════════════

    def pc_receive(self) -> None:
        """
        Runs in pc_thread.
        Handles: PATH (pathfinding result), OBJECT (detection result).
        """
        while True:
            try:
                msg = self.pc.receive()
                if not msg:
                    continue

                if msg.startswith("PATH"):
                    # ── PC sent back the computed path ────────────────────────
                    try:
                        payload = json.loads(msg.split("PATH,", 1)[1])
                    except (IndexError, json.JSONDecodeError) as exc:
                        logging.error(f"PC: malformed PATH message — {exc}")
                        continue

                    with self._idx_lock:
                        self.segments = payload.get("segments", [])
                        self.obstacle_order = payload.get("obstacle_ids", [])
                        # Optional per-segment map; see _obstacle_for_segment().
                        # Present when the planner had to split an approach
                        # across several lines to respect the §2 caps.
                        self.segment_obstacles = payload.get("segment_obstacles", [])
                        self.directions = payload.get("dirs", [])
                        self.direction_index = 0

                    self.path_requested = False
                    self.path_ready.set()
                    logging.info(
                        f"PC: PATH received — {len(self.segments)} segment(s), "
                        f"obstacle order: {self.obstacle_order}."
                    )

                    # If Android already sent BEGIN but PATH hadn't arrived yet,
                    # kick off the first segment now
                    if self.started and self.segments_index == 0:
                        with self._idx_lock:
                            if self.directions:
                                start_pose = self.directions[0]
                                try:
                                    self.android.send(
                                        f"STATUS,START,{start_pose['x']},{start_pose['y']},{start_pose['dir']}"
                                    )
                                except OSError as exc:
                                    logging.warning(f"Could not notify Android of start position: {exc}")
                        if not self._send_next_segment():
                            logging.warning("PC: PATH arrived but no segments to send.")

                elif msg.startswith("OBJECT"):
                    # ── PC replied with a detection result ────────────────────
                    # Expected format: OBJECT,<obstacle_id>,<confidence>,<class_id>
                    parts = msg.split(",")
                    if len(parts) < 4:
                        logging.error(f"PC: malformed OBJECT message '{msg}'.")
                        self.image_done.set()  # unblock stm_thread regardless
                        continue

                    _, obstacle_id, conf_str, class_id = parts[0], parts[1], parts[2], parts[3]
                    class_id = class_id.strip()

                    try:
                        confidence = float(conf_str)
                    except ValueError:
                        confidence = None

                    logging.info(
                        f"PC: OBJECT — obstacle={obstacle_id}  "
                        f"class={class_id}  conf={confidence}."
                    )

                    # Unblock stm_thread which is waiting for this
                    self.image_done.set()

                    # Forward result to Android
                    if confidence is not None:
                        self.android.send(f"TARGET,{obstacle_id},{class_id}")

                else:
                    logging.warning(f"PC: unrecognised message '{msg}' — ignoring.")

            except OSError as exc:
                logging.error(f"PC thread OSError: {exc}")

    # ══ Thread: STM receive ═══════════════════════════════════════════════════

    def stm_receive(self) -> None:
        """
        Runs in stm_thread.
        Handles: OK (command complete), RESEND (error → retransmit last).

        When STM says OK:
          1. Trigger image detection for the obstacle we just reached.
          2. Send the next movement segment (if any remain).
          3. When all segments done, tell PC to stitch.
        """
        while True:
            try:
                # Do not start a reply timeout while idle. A line may be sent by
                # Android between loop iterations; this check ensures the 20 s
                # deadline begins only after that specific line is in flight.
                if not self.stm.awaiting_reply:
                    sleep(0.02)
                    continue

                # It returns only line-level replies (OK / RESEND / FAIL,*);
                # query answers are routed separately by the STM reader thread,
                # so they can never be mistaken for a movement reply here.
                stm_msg = self.stm.wait_reply()
                if not stm_msg:
                    # PROTOCOL.md §11: past the 15s watchdog this is a lost
                    # link, not a slow move.
                    if self.started and not self.halted:
                        self._halt_mission("no reply from STM — link presumed lost")
                    continue

                reply = stm_msg.strip().upper()
                logging.info(f"STM received: '{stm_msg}'")

                if reply.startswith("FAIL"):
                    # ── Move did not complete (§3) ─────────────────────────────
                    # FAIL,TIMEOUT  → wheel stalled or encoder dead
                    # FAIL,WRONGWAY → arc rotated away from target, aborted
                    # FAIL,NOECHO   → FU<n> had no reading; NOTHING MOVED (§4.1)
                    detail = stm_msg.strip().split(",", 1)[1] if "," in stm_msg else "UNKNOWN"

                    if detail == "NOECHO":
                        # Worth its own message. The other two mean the pose is
                        # unknown; this one means the pose is exactly what it
                        # was, because FU refuses to drive at something it
                        # cannot see. The firmware dropped the rest of the line
                        # rather than run it from the wrong place, so the plan
                        # is still stale and the mission still stops — but the
                        # thing to go and look at is the ultrasound, not the
                        # wheels, and a recovery can start from the last known
                        # pose instead of re-localising.
                        self._halt_mission(
                            "STM reported FAIL,NOECHO — FU had no usable ultrasound "
                            "reading, so nothing moved and the rest of the line was "
                            "dropped (PROTOCOL.md §4.1). The pose is unchanged. "
                            "Check the sensor wiring and that something is actually "
                            "in front of the robot, then resend from here."
                        )
                        continue

                    self._halt_mission(
                        f"STM reported {stm_msg.strip()} — the move did not complete "
                        f"({detail}). Re-plan from the robot's actual pose."
                    )
                    continue

                if reply == "RESEND":
                    # ── Parse failure: nothing executed, safe to retransmit ────
                    with self._idx_lock:
                        if not self.segments:
                            logging.warning("STM RESEND but no segments loaded yet.")
                            continue
                        last_idx = max(self.segments_index - 1, 0)
                        seg = self.segments[last_idx]

                    count = self._resend_counts.get(last_idx, 0) + 1
                    self._resend_counts[last_idx] = count

                    if count > self.max_resends:
                        # PROTOCOL.md §3 — give up and report rather than loop.
                        self._halt_mission(
                            f"segment {last_idx} {seg} was RESENDed "
                            f"{self.max_resends} times and never parsed. The "
                            "tokens are wrong, not the transmission."
                        )
                        continue

                    logging.warning(
                        f"STM RESEND: retransmitting segment {last_idx} "
                        f"(attempt {count}/{self.max_resends}): {seg}."
                    )
                    self.stm.send_line(seg)

                elif reply == "OK":
                    with self._idx_lock:
                        just_finished = self.segments_index - 1
                        more_to_send = self.segments_index < len(self.segments)
                        # Cleared on success so a later genuine RESEND on this
                        # index starts counting from zero again.
                        self._resend_counts.pop(just_finished, None)

                    if self.halted:
                        logging.warning("OK received but mission is halted — ignoring.")
                        continue

                    # Update Android with robot's expected grid position
                    with self._idx_lock:
                        if 0 <= just_finished < len(self.directions):
                            robot_pose = self.directions[just_finished]
                        else:
                            robot_pose = None

                    if robot_pose:
                        self.android.send(
                            f"ROBOT,{robot_pose['x']},{robot_pose['y']},{robot_pose['dir']}"
                        )
                    
                    # ── Capture + detect for the obstacle we just reached ──────
                    obstacle_id = self._obstacle_for_segment(just_finished)
                    if obstacle_id is not None:
                        self._detect_and_send_image(obstacle_id)

                    # ── Send next movement segment (or finish) ─────────────────
                    if more_to_send:
                        if self.segment_delay > 0:
                            sleep(self.segment_delay)
                        if not self._send_next_segment():
                            logging.warning(
                                "STM OK: expected more segments but none available."
                            )
                    else:
                        # All done — tell PC to stitch the result images.
                        # Counts IMAGES, not segments: with §2 chunking one
                        # obstacle can span several segments, so len(segments)
                        # would over-count.
                        if self.segment_obstacles:
                            shots = sum(1 for o in self.segment_obstacles if o is not None)
                        else:
                            shots = len(self.obstacle_order) or len(self.segments)
                        self.pc.send(f"STITCH,{max(shots - 1, 0)}\n")
                        self.android.send("STATUS,DONE")
                        logging.info("All segments complete — STITCH sent to PC.")

                else:
                    # classify_reply() only routes OK / RESEND / FAIL,* here, so
                    # anything else means the firmware and PROTOCOL.md have
                    # drifted apart. Worth a warning, not a silent debug line.
                    logging.warning(
                        f"STM: unexpected line-level reply '{stm_msg}' — not one of "
                        "OK / RESEND / FAIL,* (PROTOCOL.md §3)."
                    )

            except OSError as exc:
                logging.error(f"STM thread OSError: {exc}")

    # ══ Entry point ═══════════════════════════════════════════════════════════

    def _stm_startup_check(self) -> None:
        """
        PROTOCOL.md §10 stage 5 — prove the link before trusting it with motion.

        ?VER costs one round trip and distinguishes "the firmware is alive and
        talking a protocol version we know" from "the port opened but nothing
        is listening", which otherwise only shows up as a mysteriously silent
        first move. It is also the only way to find out whether FU<n> exists
        before a segment containing one earns a RESEND.

        Runs BEFORE the segment pump starts, so sending !PROF here cannot
        collide with a movement OK (see STM.set_profile).
        """
        ver = self.stm.query("?VER")
        if ver is None:
            logging.warning(
                "STM: no answer to ?VER. The link may be dead, or the board may be "
                "on USB Port 1 (UART1, download only — the firmware never transmits "
                "there). See PROTOCOL.md §1."
            )
        else:
            logging.info(f"STM: {ver}")
            fields = ver.split(",")
            proto = fields[2].strip() if len(fields) >= 3 else ""
            # This client implements protocol 3. Every version is a pure
            # SUPERSET of the one before — the movement tokens are byte-for-byte
            # identical back to v1 — so an older firmware is a CAPABILITY gap,
            # not an incompatibility, and saying so is more useful than a flat
            # version-mismatch warning that would cry wolf on a board that runs
            # Task 1 perfectly well.
            #
            # The gap that can actually bite is FU<n>: on v1 or v2 it is still a
            # reserved token, so a segment containing one is a parse failure and
            # the WHOLE line RESENDs. That is a silent planning bug if nobody is
            # told, hence the explicit warning rather than an info line.
            try:
                proto_num = int(proto)
            except ValueError:
                proto_num = -1

            if proto_num < 0:
                logging.warning(
                    f"STM: could not read a protocol version out of {ver!r} — "
                    "re-read PROTOCOL.md §5."
                )
            elif proto_num < CAL_MIN_PROTOCOL:
                logging.info(
                    f"STM: firmware is protocol {proto_num}. Movement is "
                    "unaffected, but ?CAL and the !CAL* setters will RESEND "
                    "(PROTOCOL.md §7)."
                )
            elif proto_num < FU_MIN_PROTOCOL:
                logging.warning(
                    f"STM: firmware is protocol {proto_num}. Movement and "
                    "calibration are fine, but FU<n> is still RESERVED there — "
                    "any segment containing one will RESEND the entire line "
                    "(PROTOCOL.md §9). Plan with F0 or flash a v"
                    f"{FU_MIN_PROTOCOL} build."
                )
            elif proto_num > PROTOCOL_VERSION:
                logging.warning(
                    f"STM: firmware reports protocol {proto_num}, newer than "
                    f"the {PROTOCOL_VERSION} this client implements. Anything "
                    "it added is unused here — re-read PROTOCOL.md."
                )

        if self.arc_profile is None:
            # No !PROF sent: the firmware's selected profile stands. Ask what it
            # is rather than assume, so the planner's radius can be checked
            # against reality instead of against a guess.
            stat = self.stm.query("?STAT")
            if stat:
                logging.info(f"STM: {stat} (arc profile is the last field — §5).")
            logging.info(
                "STM: no !PROF sent — running the firmware's own profile "
                "(default TIGHT, radius 291mm)."
            )
        elif self.stm.set_profile(self.arc_profile):
            logging.info(f"STM: arc profile set to {self.arc_profile}.")
        else:
            logging.warning(
                f"STM: could not set arc profile {self.arc_profile} — the firmware "
                "keeps whatever it had. If the planner assumed a different radius, "
                "its turns will land short or long."
            )

    def start(self) -> None:
        """
        Connect all peripherals, start threads, and block until they exit.
        """
        logging.info("=" * 60)
        logging.info("Task 1 — starting up")
        logging.info("=" * 60)

        # ── Connections (order matters: BT can take time) ──────────────────
        logging.info("Connecting to STM32…")
        self.stm.connect()
        self._stm_startup_check()

        logging.info("Starting Bluetooth server (waiting for Android)…")
        self.android.start()          # non-blocking; background accept loop starts

        logging.info("Waiting for PC to connect on TCP…")
        self.pc.connect()             # blocks until PC connects

        logging.info("All connections established — launching threads.")

        # ── Thread creation ────────────────────────────────────────────────
        self.stm_thread = Thread(
            target=self.stm_receive, name="stm-thread", daemon=True
        )
        self.pc_thread = Thread(
            target=self.pc_receive, name="pc-thread", daemon=True
        )
        self.android_thread = Thread(
            target=self.android_receive, name="android-thread", daemon=True
        )

        self.stm_thread.start()
        self.pc_thread.start()
        self.android_thread.start()

        logging.info("All threads running.  Waiting for mission…")

        # Block main thread until all workers finish (they run forever unless
        # interrupted, so Ctrl-C or an unhandled exception will stop them)
        try:
            self.stm_thread.join()
            self.pc_thread.join()
            self.android_thread.join()
        except KeyboardInterrupt:
            logging.info("Interrupted — shutting down.")
        finally:
            self.camera.stop_camera()
            self.stm.disconnect()
            self.pc.disconnect()
            self.android.stop()
            logging.info("Task 1 shut down cleanly.")


# ── Run ────────────────────────────────────────────────────────────────────────

_LOCK_PATH = "/var/run/mdp-task1.lock"


def _acquire_task_lock():
    """Prevent concurrent Task 1 orchestrators without affecting test scripts."""
    os.makedirs(os.path.dirname(_LOCK_PATH), exist_ok=True)
    lock_file = open(_LOCK_PATH, "w")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.close()
        logging.error("Another Task 1 instance is already running — exiting.")
        return None

    lock_file.write(str(os.getpid()))
    lock_file.flush()
    return lock_file

if __name__ == "__main__":
    task_lock = _acquire_task_lock()
    if task_lock is not None:
        try:
            Task1().start()
        finally:
            fcntl.flock(task_lock, fcntl.LOCK_UN)
            task_lock.close()
