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
import logging
import os
from threading import Event, Lock, Thread, Timer
from time import sleep

from dotenv import load_dotenv

load_dotenv()

from communications.android import Android
from communications.pc import PC
from communications.stm import STM
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

        self.started: bool = False          # True after Android sends BEGIN
        self.path_requested: bool = False   # True while OBSTACLES request is in-flight

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

        logging.info(
            f"Config — segment_delay={self.segment_delay}s  "
            f"detect_retries={self.detect_retries}  "
            f"detect_retry_delay={self.detect_retry_delay}s  "
            f"detect_timeout={self.detect_timeout}s  "
            f"debounce_delay={self._debounce_delay}s"
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

    def _send_next_segment(self) -> bool:
        """
        Atomically read-and-advance segments_index, then send the segment to STM.
        Returns True if a segment was sent, False if we are past the end.
        """
        with self._idx_lock:
            if self.segments_index < 0 or self.segments_index >= len(self.segments):
                return False
            seg = self.segments[self.segments_index]
            self.segments_index += 1

        cmd = ",".join(seg) + "\n"
        self.stm.send(cmd)
        logging.info(
            f"STM segment {self.segments_index}/{len(self.segments)} sent: {seg}"
        )
        return True

    # ── Image detection with retry ────────────────────────────────────────────

    def _detect_and_send_image(self, obstacle_id: str) -> bool:
        """
        Full image capture + transfer + wait-for-result cycle with retry.

        Steps per attempt:
          1. Capture image from camera.
          2. Tell PC a DETECT is coming: DETECT,<id>
          3. Send image filename then raw bytes.
          4. Wait up to detect_timeout seconds for pc_thread to set image_done.
          5. Retry up to detect_retries times if no response.

        Returns True if OBJECT was received, False after all attempts exhausted.
        """
        total_attempts = self.detect_retries + 1

        for attempt in range(1, total_attempts + 1):
            # ── Capture ──────────────────────────────────────────────────────
            image_path = self.camera.capture_image()
            if not image_path:
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
            self.pc.send_image(image_path)

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
                    facing_map = {"NORTH": 0, "EAST": 2, "SOUTH": 4, "WEST": 6, "SKIP": 8}
                    obstacle = {
                        "id":  int(parts[1]),
                        "x":   int(parts[2]) / 10,
                        "y":   int(parts[3]) / 10,
                        "d":   facing_map.get(parts[4].strip().upper(), 0),
                    }
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

                    if not self.path_ready.is_set():
                        logging.info(
                            "Android: BEGIN received but PATH not ready yet — "
                            "will send first segment once PATH arrives."
                        )
                        continue

                    if not self._send_next_segment():
                        logging.warning("Android: BEGIN received but no segments to send.")

                else:
                    logging.warning(f"Android: unrecognised message '{msg}' — ignoring.")

            except OSError as exc:
                logging.error(f"Android thread OSError: {exc}")

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
                # wait_receive() blocks — this is fine because stm_thread is dedicated
                stm_msg = self.stm.wait_receive()
                if not stm_msg:
                    continue

                logging.info(f"STM received: '{stm_msg}'")

                if "RESEND" in stm_msg:
                    # ── STM detected an error — resend last segment ────────────
                    with self._idx_lock:
                        if not self.segments:
                            logging.warning("STM RESEND but no segments loaded yet.")
                            continue
                        last_idx = max(self.segments_index - 1, 0)
                        seg = self.segments[last_idx]
                    cmd = ",".join(seg) + "\n"
                    self.stm.send(cmd)
                    logging.info(f"STM RESEND: retransmitting segment {last_idx}: {seg}.")

                elif "OK" in stm_msg:
                    with self._idx_lock:
                        just_finished = self.segments_index - 1
                        more_to_send = self.segments_index < len(self.segments)

                    # ── Capture + detect for the obstacle we just reached ──────
                    if 0 <= just_finished < len(self.obstacle_order):
                        obstacle_id = str(self.obstacle_order[just_finished])
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
                        # All done — tell PC to stitch the result images
                        total = len(self.segments)
                        self.pc.send(f"STITCH,{total - 1}\n")
                        self.android.send("STATUS,DONE")
                        logging.info("All segments complete — STITCH sent to PC.")

                else:
                    # Some boards send intermediate status strings — log and ignore
                    logging.debug(f"STM: unhandled message '{stm_msg}'.")

            except OSError as exc:
                logging.error(f"STM thread OSError: {exc}")

    # ══ Entry point ═══════════════════════════════════════════════════════════

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

if __name__ == "__main__":
    task = Task1()
    task.start()
