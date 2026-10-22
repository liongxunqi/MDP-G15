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
5. Android sends BEGIN  →  RPi sends first STM instruction
6. Each OK → query WPOSE → ROBOT to Android and PROGRESS to PC
   Optional feedback wait → CONTINUE / REPLACE, or timeout → next instruction
   At a segment boundary → DETECT and image bytes to PC, if a photo is requested
7. PC replies  OBJECT,<obstacle_id>,<confidence>,<class_id>
                    →  RPi forwards  TARGET,<obstacle_id>,<class_id>  to Android
                    →  RPi begins the next segment (if any left)
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
from collections import Counter
from threading import Event, Lock, Thread, Timer
from time import monotonic, sleep
from typing import Optional

from dotenv import load_dotenv

load_dotenv()

from communications.android import Android
from communications import cal_profile
from communications.pc import PC
from communications.stm import (
    CAL_MIN_PROTOCOL,
    FU_MIN_PROTOCOL,
    PROTOCOL_VERSION,
    STM,
)
from image_capture.camera import Camera
from instruction_mission import InstructionMission, parse_start_pose
from feedback_control import FeedbackControl

import run_log

run_log.setup("task1", "%(asctime)s [%(threadName)s] %(levelname)s — %(message)s")


# ── Checklist A.5 ─────────────────────────────────────────────────────────────
# A.5 is Task 1 with ONE thing removed: you do not know which face the image is
# on. Task 1 is built entirely around knowing it — the operator taps the face in
# on Android, it arrives as `d`, and path_planner._viewing_pose() turns it into
# the pose to photograph from.
#
# So A.5 reduces to Task 1 exactly: declare the single obstacle FOUR times, once
# per face, and one unknown-face problem becomes four known-face problems. The
# planner then does the orbit for you — four viewing poses, a shortest tour
# between them, and one DETECT at each — instead of the robot dead-reckoning its
# way around a block.
#
# Nothing in the planner or the Android app changes. The fan-out happens here,
# on the way out to the PC, and is folded back on the way to Android.
A5_FACE_D = (0, 2, 4, 6)
A5_FACE_NAME = {0: "N", 2: "E", 4: "S", 6: "W"}

# What does NOT count as "a valid image from the image list". The bullseye is
# the MARKER, 'dot' is the filler class, and NONE is what task1_pc.py sends when
# YOLO found nothing. A5 uses this to keep checking faces. Normal Task 1 uses
# the PC detector's final selection directly, including a bullseye when no
# preferred image class was present.
A5_NOT_A_TARGET = {"bullseye", "dot", "none"}
TASK1_NOT_A_TARGET = {"dot", "none"}


class Task1:
    """Orchestrates the three-thread pipeline for Task 1."""

    # ── Init ──────────────────────────────────────────────────────────────────

    def __init__(self, a5_mode: bool = False):
        # Communication objects
        self.android = Android()
        self.pc = PC()
        self.stm = STM()
        self.camera = Camera()

        # Thread handles
        self.android_thread: Optional[Thread] = None
        self.pc_thread: Optional[Thread] = None
        self.stm_thread: Optional[Thread] = None

        # ── Obstacle / path state ─────────────────────────────────────────────
        self.obstacles: list = []           # list of obstacle dicts from Android
        self.segments: list = []            # movement segments from PC
        self.segments_index: int = 0        # which segment we are up to
        self.instruction_mission = None
        self._completed_photo_count = 0
        self.obstacle_order: list = []      # obstacle IDs in visit order
        self.directions: list = []          # direction info per segment (for Android map)
        self.start_pose: Optional[dict] = None # Android's 2x2 drawing anchor from PATH.start
        self.odometry_start: Optional[dict] = None # planner-provided ROBOT/PROGRESS origin
        self.direction_index: int = 0

        # Per-segment obstacle mapping. A segment is NOT 1:1 with an obstacle any
        # more: PROTOCOL.md §2 caps a line at 16 primitives / 128 bytes, so one
        # obstacle approach may be split across several segments, and some
        # segments are pure travel with no photo at the end. Entry is an
        # obstacle id, or None for "no detection after this segment".
        self.segment_obstacles: list = []
        # Planner-approved camera positions, keyed by obstacle id. Distance
        # retries never invent a pose that was not checked for arena bounds,
        # obstacle clearance and ultrasonic line of sight.
        self.photo_standoffs: dict = {}
        self.selected_standoffs: dict = {}
        self.ultrasonic_adjustments: dict = {}

        self.started: bool = False          # True after Android sends BEGIN
        self.path_requested: bool = False   # True while OBSTACLES request is in-flight
        self.halted: bool = False           # True after FAIL,* or exhausted RESENDs

        # ── A.5 state ─────────────────────────────────────────────────────────
        # Off unless this was constructed by task_a5.py. Everything it touches
        # is guarded, so Task 1 behaves exactly as it did.
        self.a5_mode: bool = a5_mode
        self._a5_base_id: Optional[int] = None    # the id Android actually knows
        self._a5_face_of: dict = {}            # fanned-out id (str) -> "N"/"E"/"S"/"W"
        self._a5_found: Optional[tuple] = None    # (face, class_id, confidence)

        # PROTOCOL.md §3: cap RESEND retries. A permanently malformed segment
        # retried forever is an infinite loop in which the robot never moves.
        self._resend_counts: dict = {}

        # ── Synchronisation primitives ────────────────────────────────────────
        # Mutex for all mutable index / segment state
        self._idx_lock = Lock()
        self.feedback_control = FeedbackControl(
            enabled=os.getenv("TASK1_FEEDBACK_WAIT", "0").strip().lower() in ("1", "true", "yes"),
            timeout=float(os.getenv("TASK1_FEEDBACK_TIMEOUT_S", "7.0")),
        )
        logging.info("PC feedback wait mode: %s (timeout %.1fs)",
                     self.feedback_control.enabled, self.feedback_control.timeout)
        # Set by pc_thread when OBJECT arrives; waited on by stm_thread DETECT logic
        self.image_done = Event()
        self._detection_lock = Lock()
        self._expected_detection_id = None
        self._last_detection = None
        self._last_capture_attempts = 0
        # Set by pc_thread when PATH arrives; waited on by android_thread BEGIN logic
        self.path_ready = Event()
        # Debounce timer — send OBSTACLES to PC 1 s after last obstacle arrives
        self._debounce_lock = Lock()
        self._calc_timer: Optional[Timer] = None
        # Watchdog on an outstanding OBSTACLES request. See _arm_path_watchdog().
        self._path_timer: Optional[Timer] = None

        # ── Tunable config from .env ──────────────────────────────────────────
        self._debounce_delay = float(os.getenv("DEBOUNCE_DELAY_S", "1.0"))
        # How long to wait for PATH before calling it a failure. Generous:
        # planning is an exhaustive tour search with an A* per edge, and a slow
        # laptop is not a broken one.
        self._path_timeout = float(os.getenv("PATH_TIMEOUT_S", "30.0"))
        self.segment_delay = float(os.getenv("SEGMENT_DELAY_S", "0.5"))
        self.detect_retries = int(os.getenv("DETECT_RETRY_COUNT", "1"))
        self.detect_retry_delay = float(os.getenv("DETECT_RETRY_DELAY_S", "0.2"))
        self.detect_timeout = float(os.getenv("DETECT_TIMEOUT_S", "30.0"))
        self.capture_settle = float(os.getenv("TASK1_CAPTURE_SETTLE_S", "2.0"))
        self.us_adjust_retries = max(
            0, int(os.getenv("TASK1_US_ADJUST_RETRIES", "3"))
        )
        self.us_adjust_tolerance_cm = max(
            0.0, float(os.getenv("TASK1_US_ADJUST_TOLERANCE_CM", "3.0"))
        )
        self.us_adjust_max_step_cm = max(
            1, int(os.getenv("TASK1_US_ADJUST_MAX_STEP_CM", "30"))
        )
        self.us_settle_s = max(
            0.0, float(os.getenv("TASK1_US_SETTLE_S", "0.2"))
        )
        self.ir_feedback_enabled = os.getenv(
            "TASK1_IR_FEEDBACK", "1"
        ).strip().lower() in ("1", "true", "yes")
        self.preflight_enabled = self.ir_feedback_enabled and self.feedback_control.enabled
        self.gyro_settle_enabled = os.getenv(
            "TASK1_GYRO_SETTLE", "1"
        ).strip().lower() in ("1", "true", "yes")
        self.gyro_settle_rate_dps = float(os.getenv(
            "TASK1_GYRO_SETTLE_RATE_DPS", "1.0"
        ))
        self.gyro_settle_samples = max(1, int(os.getenv(
            "TASK1_GYRO_SETTLE_SAMPLES", "3"
        )))
        self.gyro_settle_timeout = max(0.0, float(os.getenv(
            "TASK1_GYRO_SETTLE_TIMEOUT_S", "1.0"
        )))
        retry_text = os.getenv("TASK1_PHOTO_RETRY_STANDOFFS", "40,45,35,33,20")
        try:
            self.photo_retry_standoffs = [
                int(value.strip()) for value in retry_text.split(",") if value.strip()
            ]
        except ValueError:
            logging.warning(
                "Invalid TASK1_PHOTO_RETRY_STANDOFFS=%r; using 40,45,35,33,20.",
                retry_text,
            )
            self.photo_retry_standoffs = [40, 45, 35, 33, 20]
        self.photo_retry_limit = max(
            0, int(os.getenv("TASK1_PHOTO_RETRY_LIMIT", "5"))
        )
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
            f"capture_settle={self.capture_settle}s  "
            f"us_adjust={self.us_adjust_retries}x/"
            f"{self.us_adjust_tolerance_cm:.1f}cm/"
            f"max{self.us_adjust_max_step_cm}cm  "
            f"photo_retry_standoffs={self.photo_retry_standoffs}  "
            f"photo_retry_limit={self.photo_retry_limit}  "
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

    def _a5_expand(self, obstacles: list) -> list:
        """
        One obstacle in, four out — same cell, one per face.

        The face Android sent is DISCARDED on purpose. In A.5 the operator
        cannot know it; whatever they tapped is a guess, and acting on a guess
        would photograph one face and call the task done.

        Ids are derived as base*10 + d so the fold-back in pc_receive() is a
        division. Android is never told about them — it knows one obstacle and
        must keep knowing one obstacle, or ArenaReducer rejects the TARGET as
        referencing an obstacle that does not exist.
        """
        if len(obstacles) != 1:
            logging.error(
                f"A.5 expects exactly one obstacle, got {len(obstacles)}. Sending "
                f"them unchanged — this will behave like Task 1, not like A.5."
            )
            return obstacles

        base = obstacles[0]
        try:
            base_id = int(base["id"])
        except (KeyError, TypeError, ValueError):
            logging.error(f"A.5: obstacle has no usable integer id: {base!r}.")
            return obstacles

        self._a5_base_id = base_id
        self._a5_face_of = {}
        self._a5_found = None

        expanded = []
        for d in A5_FACE_D:
            face_id = base_id * 10 + d
            expanded.append({**base, "id": face_id, "d": d})
            self._a5_face_of[str(face_id)] = A5_FACE_NAME[d]

        logging.info(
            f"A.5: obstacle {base_id} at ({base.get('x')}, {base.get('y')}) "
            f"expanded to four faces {sorted(self._a5_face_of.values())} — the "
            f"face Android sent (d={base.get('d')}) is ignored, which is the "
            f"whole point of A.5."
        )
        return expanded

    def _a5_report(self, face_id: str, class_id: str, confidence: float) -> None:
        """
        Decide what a detection on one face means, and what Android hears.

        THREE OF THE FOUR REPLIES ARE NOT THE ANSWER, and that is the search
        working rather than anything going wrong. Task 1 forwards every result
        because every face it visits genuinely has an image; here three of them
        are bullseyes or blanks.

        Forwarding them would not merely be noise. Every fanned-out id folds
        back to the SAME obstacle, and ArenaReducer.applyTarget() overwrites
        targetId each time — so the last face visited would win, and a bullseye
        arriving after the real image would replace it on the screen the
        supervisor is watching. Only a valid image is forwarded, and only the
        first one.
        """
        face = self._a5_face_of.get(str(face_id).strip(), "?")
        name = class_id.strip()

        if name.lower() in A5_NOT_A_TARGET:
            logging.info(
                f"A.5 face {face}: '{name}' — the marker or nothing at all, not "
                f"a target. Wrong face, keep going."
            )
            return

        if self._a5_found is not None:
            prev_face, prev_name, _ = self._a5_found
            logging.info(
                f"A.5 face {face}: '{name}' (conf={confidence:.2f}) — already "
                f"found '{prev_name}' on face {prev_face}, not overwriting it. "
                f"The path was planned before the first photo, so the remaining "
                f"faces get visited either way."
            )
            return

        self._a5_found = (face, name, confidence)
        logging.info("=" * 58)
        logging.info(
            f"A.5 COMPLETE — valid image '{name}' (conf={confidence:.2f}) found "
            f"on the {face} face."
        )
        logging.info(
            f"The annotated still is on the PC under pc/runs/predict/, "
            f"named obstacle_{face_id}_*.jpg."
        )
        logging.info("=" * 58)

        # Folded back to the id Android actually knows about.
        target_id = self._a5_base_id if self._a5_base_id is not None else face_id
        self.android.send(f"TARGET,{target_id},{name}")

    def _request_path_from_pc(self) -> bool:
        """Send current obstacle list to PC for pathfinding.  Idempotent."""
        if self.path_requested:
            logging.info("PATH already in-flight — skipping duplicate OBSTACLES send.")
            return False

        outgoing = self._a5_expand(self.obstacles) if self.a5_mode else self.obstacles

        payload = "OBSTACLES," + json.dumps(outgoing) + "\n"
        self.pc.send(payload)
        self.path_requested = True
        self._arm_path_watchdog()
        logging.info(f"Sent OBSTACLES to PC ({len(outgoing)} obstacle(s)).")
        return True

    # ── PATH watchdog ─────────────────────────────────────────────────────────

    def _arm_path_watchdog(self) -> None:
        """
        Fail loudly if the PC never answers an OBSTACLES request.

        Nothing blocks on PATH — BEGIN checks path_ready and returns, and the
        arrival of PATH is what kicks off the first segment. So a planner that
        raises does not deadlock anything. It leaves path_requested True
        forever, which makes _request_path_from_pc() refuse to try again, and
        the robot stands still with nothing on screen to say why.

        That silence is the whole problem: a stalled plan and a robot waiting
        for BEGIN look identical from the outside.
        """
        self._cancel_path_watchdog()
        self._path_timer = Timer(self._path_timeout, self._on_path_timeout)
        self._path_timer.daemon = True
        self._path_timer.start()

    def _cancel_path_watchdog(self) -> None:
        if self._path_timer is not None:
            self._path_timer.cancel()
            self._path_timer = None

    def _on_path_timeout(self) -> None:
        if not self.path_requested:
            return          # PATH landed while the timer was already firing

        logging.error(
            f"No PATH from the PC within {self._path_timeout}s. The planner has "
            f"most likely raised — check the PC's console. Press Send Data "
            f"again to retry once it is back."
        )
        # Cleared so a retry is possible at all: _request_path_from_pc()
        # refuses to send while a request is in flight.
        self.path_requested = False
        try:
            self.android.send("STATUS,FAILED")
        except OSError as exc:
            logging.warning(f"Could not tell Android the planner failed: {exc}")

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

    def _read_ir_snapshot(self):
        """Read both calibrated IR ranges and their filtered ADC counts."""
        if not getattr(self, "ir_feedback_enabled", False):
            return {}
        ranges = self.stm.query_fields("?IR")
        raw = self.stm.query_fields("?IRR")
        snapshot = {
            "ir_left_cm": None,
            "ir_right_cm": None,
            "ir_left_raw": None,
            "ir_right_raw": None,
        }
        try:
            if ranges is not None and len(ranges) == 2:
                snapshot["ir_left_cm"], snapshot["ir_right_cm"] = (
                    int(value) for value in ranges
                )
            if raw is not None and len(raw) == 2:
                snapshot["ir_left_raw"], snapshot["ir_right_raw"] = (
                    int(value) for value in raw
                )
        except (TypeError, ValueError):
            logging.warning("Ignoring malformed STM IR/IRR response.")
        logging.info(
            "IR snapshot: L=%s cm/%s raw, R=%s cm/%s raw.",
            snapshot["ir_left_cm"], snapshot["ir_left_raw"],
            snapshot["ir_right_cm"], snapshot["ir_right_raw"],
        )
        return snapshot

    def _read_ultrasonic_cm(self):
        """Return one usable front-ultrasonic range, or None for no echo."""
        fields = self.stm.query_fields("?US")
        if fields is None or len(fields) != 1:
            logging.warning("STM returned no usable ?US response.")
            return None
        try:
            distance = int(fields[0])
        except (TypeError, ValueError):
            logging.warning("Ignoring malformed STM ?US response: %r", fields)
            return None
        if distance <= 0 or distance == 65535:
            logging.warning("Ultrasonic has no target echo (US,%s).", distance)
            return None
        return distance

    def _wait_for_gyro_settle(self, token):
        """After an arc OK, wait briefly until measured yaw rate is stationary."""
        if (not getattr(self, "gyro_settle_enabled", False) or
                not str(token).upper().startswith(("FR", "FL", "RR", "RL")) or
                self.gyro_settle_timeout <= 0):
            return
        deadline = monotonic() + self.gyro_settle_timeout
        stable = 0
        last_rate = None
        while monotonic() < deadline:
            fields = self.stm.query_fields("?IMU")
            try:
                ready = fields is not None and len(fields) >= 3 and int(fields[0]) == 1
                last_rate = abs(int(fields[2]) / 10.0) if ready else None
            except (TypeError, ValueError):
                ready, last_rate = False, None
            if ready and last_rate <= self.gyro_settle_rate_dps:
                stable += 1
                if stable >= self.gyro_settle_samples:
                    logging.info(
                        "Gyro settled after %s: %.1f dps (%d samples).",
                        token, last_rate, stable,
                    )
                    return
            else:
                stable = 0
            sleep(0.05)
        logging.warning(
            "Gyro did not provide %d stable sample(s) <= %.1f dps after %s "
            "within %.1fs (last=%s); continuing.",
            self.gyro_settle_samples, self.gyro_settle_rate_dps, token,
            self.gyro_settle_timeout, last_rate,
        )

    def _send_next_segment(self) -> bool:
        """Send the next primitive, retaining the planner's photo boundaries."""
        if self.halted:
            return False
        try:
            while True:
                with self._idx_lock:
                    if self.halted or self.feedback_control.waiting:
                        return False
                    if self.instruction_mission is None:
                        self.instruction_mission = InstructionMission(
                            self.segments, self.odometry_start
                        )
                    mission = self.instruction_mission
                    needs_preflight = (
                        getattr(self, "preflight_enabled", False) and
                        mission.segment_index not in mission.preflighted_segments
                    )
                if not needs_preflight:
                    break
                outcome = self._preflight_mission(mission)
                if outcome is None:
                    return False
                if outcome == "replace":
                    continue

            with self._idx_lock:
                if self.halted or self.feedback_control.waiting:
                    return False
                mission = self.instruction_mission
                if not mission.send_next(self.stm):
                    return False
                self.segments_index = mission.segment_index + 1
                logging.info("STM segment %d/%d instruction %d: %s",
                             self.segments_index, len(self.segments),
                             mission.instruction_index + 1, mission.token)
                if mission.instruction_index == 0:
                    self.android.send(
                        f"STATUS,RUNNING,{self.segments_index},{len(self.segments)}"
                    )
            return True
        except (ValueError, OSError) as exc:
            self._halt_mission(str(exc))
            return False

    def _preflight_mission(self, mission):
        """Give PC live pose/IR before the first instruction of a new route."""
        with self._idx_lock:
            if self.instruction_mission is not mission or mission.done:
                return None
            if mission.origin is None:
                mission.origin = mission.read_pose(self.stm)
            robot_message, pose = mission.report_pose(self.stm)
            remaining = [
                obstacle_id
                for i in range(mission.segment_index, len(mission.segments))
                for obstacle_id in [self._obstacle_for_segment(i)]
                if obstacle_id is not None
            ]
            progress = {
                "event": "instruction_preflight",
                "segment_index": mission.segment_index,
                "instruction_index": -1,
                "token": mission.token,
                "segment": list(mission.segments[mission.segment_index]),
                "segment_completed": False,
                "motion_plan_completed": False,
                "next_segment_index": mission.segment_index,
                "next_instruction_index": mission.instruction_index,
                "x_grid": pose["x_grid"],
                "y_grid": pose["y_grid"],
                "heading_deg": pose["heading_deg"],
                "remaining_photo_ids": remaining,
            }
            progress.update(self._read_ir_snapshot())
            pending = self.feedback_control.arm(progress)
            self.android.send(robot_message)
            self.pc.send("PROGRESS," + json.dumps(progress, separators=(",", ":")))

        decision = self._wait_for_feedback(pending, mission, None)
        if decision == "continue":
            with self._idx_lock:
                if self.instruction_mission is mission:
                    mission.preflighted_segments.add(mission.segment_index)
        return decision

    def _send_start_status(self) -> None:
        """Pass the planner's Android drawing anchor directly to Android."""
        with self._idx_lock:
            start = self.start_pose
        if start is None:
            self._halt_mission("Cannot start without planner PATH.start")
            return
        status = f"STATUS,START,{start['x']},{start['y']},{start['dir']}"
        try:
            self.android.send(status)
        except OSError as exc:
            logging.warning(f"Could not notify Android of start position: {exc}")

    def _validate_task1_mission(self, mission):
        """Reject legacy FU tokens at every Task 1 path boundary."""
        if (not getattr(self, "a5_mode", False) and
                any(token.startswith("FU")
                    for segment in mission.segments for token in segment)):
            raise ValueError(
                "Task 1 PATH may not contain FU; plan ordinary F/R motion and "
                "let the RPi verify the camera range with ?US"
            )
        return mission

    # ── Failure handling ───────────────────────────────────────────────────────

    def _apply_replacement(self, payload, mission, just_finished):
        """Called under _idx_lock; validate everything before swapping state."""
        candidate = self._validate_task1_mission(
            InstructionMission(payload.get("segments"), mission.start_pose)
        )
        mapping = payload.get("segment_obstacles")
        if not isinstance(mapping, list) or len(mapping) != len(candidate.segments):
            raise ValueError("REPLACE requires segment_obstacles parallel to segments")
        if any(value is not None and
               (type(value) not in (str, int) or not str(value).strip()) for value in mapping):
            raise ValueError("Replacement obstacle IDs must be nonempty strings, integers or null")
        first = mission.segment_index if just_finished is None else just_finished
        remaining = [self._obstacle_for_segment(i) for i in range(first, len(mission.segments))]
        if Counter(str(v) for v in mapping if v is not None) != Counter(v for v in remaining if v is not None):
            raise ValueError("Replacement must preserve every pending photo assignment")
        candidate.origin = mission.origin
        self.instruction_mission = candidate
        self.segments = candidate.segments
        self.segment_obstacles = list(mapping)
        self.obstacle_order = [str(v) for v in mapping if v is not None]
        self.directions = []
        self.direction_index = 0
        self.segments_index = 0
        self._resend_counts.clear()
        logging.info("Installed replacement: %d segments; odometry origin retained", len(self.segments))

    def _wait_for_feedback(self, pending, mission, just_finished):
        """Return continue/replace, or None if cancelled/invalid. Never hold the lock while waiting."""
        try:
            logging.info("Waiting for PC decision %s (%.1fs)",
                         pending["feedback_id"], self.feedback_control.timeout)
            action, payload = self.feedback_control.wait(pending)
            with self._idx_lock:
                if self.halted or self.instruction_mission is not mission:
                    return None
                logging.info("PC decision %s for feedback %s", action, pending["feedback_id"])
                if action == "CONTINUE":
                    return "continue"
                if action == "TIMEOUT":
                    logging.warning(
                        "No PC decision for feedback %s within %.1fs; continuing existing path",
                        pending["feedback_id"], self.feedback_control.timeout,
                    )
                    return "continue"
                if action == "REPLACE":
                    self._apply_replacement(payload, mission, just_finished)
                    return "replace"
                raise ValueError(str(payload.get("reason", "Feedback wait cancelled")))
        except ValueError as exc:
            if self.instruction_mission is mission:
                self._halt_mission(str(exc))
            return None

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
        self.feedback_control.cancel(reason)
        if self.feedback_control.enabled:
            # A remaining-route replacement cannot be replayed from the fixed start.
            self.path_ready.clear()
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

    @staticmethod
    def _valid_detection(result) -> bool:
        if not isinstance(result, dict):
            return False
        class_id = str(result.get("class_id", "")).strip().lower()
        confidence = result.get("confidence")
        return (class_id not in TASK1_NOT_A_TARGET and
                isinstance(confidence, (int, float)) and confidence > 0.0)

    def _detect_and_send_image(self, obstacle_id: str, standoff_cm=None):
        """
        Full image capture + transfer + wait-for-result cycle with retry.

        Steps per attempt:
          1. Capture JPEG bytes from camera.
          2. Tell PC a DETECT is coming: DETECT,<id>[,<standoff_cm>]
          3. Send a 4-byte size header then raw bytes.
          4. Wait up to detect_timeout seconds for pc_thread to set image_done.
          5. Retry up to detect_retries times if no response.

        Returns the OBJECT fields as a dict, or None after all attempts fail.
        An OBJECT with class NONE is returned normally so the caller can move
        to another safe camera distance and take a fresh frame.
        """
        total_attempts = self.detect_retries + 1

        for attempt in range(1, total_attempts + 1):
            # ── Capture ──────────────────────────────────────────────────────
            image_bytes = self.camera.capture_image()
            if not image_bytes:
                logging.error(f"DETECT attempt {attempt}: camera capture failed — skipping.")
                continue

            # ── Signal + send ─────────────────────────────────────────────────
            with self._detection_lock:
                self._expected_detection_id = str(obstacle_id)
                self._last_detection = None
                self.image_done.clear()
            header = f"DETECT,{obstacle_id}"
            if standoff_cm is not None:
                header += f",{int(standoff_cm)}"
            self.pc.send_image(image_bytes, header=header)
            self._last_capture_attempts += 1
            logging.info(
                f"DETECT sent for obstacle {obstacle_id} "
                f"(attempt {attempt}/{total_attempts})."
            )

            # ── Wait for result ───────────────────────────────────────────────
            if self.image_done.wait(timeout=self.detect_timeout):
                with self._detection_lock:
                    result = self._last_detection
                logging.info(
                    "OBJECT received for obstacle %s on attempt %d: %s (conf=%s).",
                    obstacle_id, attempt,
                    result.get("class_id") if result else "invalid",
                    result.get("confidence") if result else "?",
                )
                return result

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
        with self._detection_lock:
            self._expected_detection_id = None
        return None

    def _move_for_photo(self, mission, obstacle_id: str, standoff_cm: int) -> bool:
        """Reach a camera range with ?US plus bounded ordinary F/R commands.

        FU is deliberately not used by Task 1. A large disagreement is treated
        as a wrong/missing target echo rather than permission for a long blind
        drive. Every actual correction publishes WPOSE and the resulting range.
        """
        measured_us = None
        for correction in range(self.us_adjust_retries + 1):
            if self.us_settle_s > 0:
                sleep(self.us_settle_s)
            measured_us = self._read_ultrasonic_cm()
            if measured_us is None:
                logging.warning(
                    "Cannot range-correct obstacle %s to %d cm; keeping the "
                    "odometry-planned camera pose.", obstacle_id, standoff_cm,
                )
                return False

            error_cm = measured_us - standoff_cm
            if abs(error_cm) <= self.us_adjust_tolerance_cm:
                logging.info(
                    "Obstacle %s camera range is %d cm (target %d cm, tolerance %.1f cm).",
                    obstacle_id, measured_us, standoff_cm,
                    self.us_adjust_tolerance_cm,
                )
                return True
            if correction >= self.us_adjust_retries:
                logging.warning(
                    "Obstacle %s remains at %d cm after %d range correction(s); "
                    "target is %d cm. Continuing with the photograph.",
                    obstacle_id, measured_us, self.us_adjust_retries, standoff_cm,
                )
                return False

            distance_cm = int(round(abs(error_cm)))
            if distance_cm > self.us_adjust_max_step_cm:
                logging.warning(
                    "Ultrasonic range %d cm is %d cm from obstacle %s's %d cm "
                    "target, beyond the %d cm correction cap. This is probably "
                    "the wrong echo; refusing blind motion.",
                    measured_us, distance_cm, obstacle_id, standoff_cm,
                    self.us_adjust_max_step_cm,
                )
                return False
            token = f"F{distance_cm}" if error_cm > 0 else f"R{distance_cm}"

            reply_upper = None
            for resend in range(self.max_resends + 1):
                if not self.stm.send_line([token]):
                    self._halt_mission(f"Could not send camera adjustment {token}")
                    return False
                reply = self.stm.wait_reply()
                if reply is None:
                    self._halt_mission(f"No STM reply for camera adjustment {token}")
                    return False
                reply_upper = reply.strip().upper()
                logging.info(
                    "Camera correction %s received STM reply %s.", token, reply_upper,
                )
                if reply_upper != "RESEND":
                    break
                if resend >= self.max_resends:
                    self._halt_mission(f"Camera adjustment {token} exceeded RESEND limit")
                    return False
                logging.warning(
                    "Camera adjustment %s was RESENDed; retry %d/%d.",
                    token, resend + 1, self.max_resends,
                )
            if reply_upper != "OK":
                self._halt_mission(f"Camera adjustment {token} failed: {reply_upper}")
                return False

            try:
                robot_message, pose = mission.report_pose(self.stm)
                self.android.send(robot_message)
                report = {
                    "event": "photo_distance_adjustment",
                    "obstacle_id": str(obstacle_id),
                    "token": token,
                    "target_standoff_cm": standoff_cm,
                    "measured_us_cm_before": measured_us,
                    "correction_attempt": correction + 1,
                    "x_grid": pose["x_grid"],
                    "y_grid": pose["y_grid"],
                    "heading_deg": pose["heading_deg"],
                }
                self.pc.send("PHOTO_PROGRESS," + json.dumps(report, separators=(",", ":")))
            except (ValueError, OSError) as exc:
                self._halt_mission(str(exc))
                return False
        return False

    def _detect_with_distance_retry(self, obstacle_id: str, mission):
        """Retry an unclear photograph only at planner-approved standoffs."""
        self._last_capture_attempts = 0
        obstacle_key = str(obstacle_id)
        original = getattr(self, "selected_standoffs", {}).get(obstacle_key)
        us_adjustable = getattr(self, "ultrasonic_adjustments", {}).get(
            obstacle_key, False
        )
        if original is not None and us_adjustable:
            self._move_for_photo(mission, obstacle_id, original)
        if self.capture_settle > 0:
            logging.info(
                "Waiting %.1fs for chassis vibration to settle before capture.",
                self.capture_settle,
            )
            sleep(self.capture_settle)
        if original is None:
            result = self._detect_and_send_image(obstacle_id)
        else:
            result = self._detect_and_send_image(obstacle_id, original)
        if self.a5_mode or self._valid_detection(result):
            return result

        safe = getattr(self, "photo_standoffs", {}).get(obstacle_key, [])
        if original is None or not safe or not us_adjustable:
            logging.warning(
                "No planner-approved alternate camera distances for obstacle %s; "
                "keeping the original frame.", obstacle_id,
            )
            return result

        candidates = []
        for distance in self.photo_retry_standoffs:
            if distance in safe and distance != original and distance not in candidates:
                candidates.append(distance)
        candidates = candidates[:self.photo_retry_limit]
        if not candidates:
            logging.warning(
                "Obstacle %s has no safe configured retry standoff (selected=%s, safe=%s).",
                obstacle_id, original, safe,
            )
            return result

        logging.warning(
            "Detection for obstacle %s was missing/low-confidence at %d cm; "
            "trying safe camera distances %s.",
            obstacle_id, original, candidates,
        )
        moved = False
        for distance in candidates:
            if not self._move_for_photo(mission, obstacle_id, distance):
                break
            moved = True
            if self.capture_settle > 0:
                logging.info(
                    "Waiting %.1fs for camera to settle at %d cm.",
                    self.capture_settle, distance,
                )
                sleep(self.capture_settle)
            result = self._detect_and_send_image(obstacle_id, distance)
            if self._valid_detection(result):
                logging.info(
                    "Obstacle %s detected at retry standoff %d cm.",
                    obstacle_id, distance,
                )
                break

        if moved and not self.halted:
            logging.info(
                "Returning obstacle %s to its planned %d cm standoff before continuing.",
                obstacle_id, original,
            )
            self._move_for_photo(mission, obstacle_id, original)
        return result

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
                    with self._idx_lock:
                        if self.started and not self.halted:
                            logging.info("Ignoring duplicate BEGIN during an active mission.")
                            continue
                        logging.info("Android: BEGIN received — mission starting.")
                        self.started = True
                        self.segments_index = 0
                        self.instruction_mission = None
                        self.feedback_control.cancel("New BEGIN")
                        self._completed_photo_count = 0
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

                    self._send_start_status()

                    if not self._send_next_segment():
                        logging.warning("Android: BEGIN received but no segments to send.")

                elif tag in ("S", "STOP"):
                    # Android's stop button sends lowercase `s` on current builds.
                    # RST is the STM protocol's immediate brake-and-drop-queue
                    # command; a normal S token cannot be queued safely while a
                    # movement instruction is outstanding.
                    if self.started and not self.halted:
                        logging.warning("Android: emergency stop requested.")
                        self.stm.abort()
                        self._halt_mission("operator requested emergency stop")
                    else:
                        logging.info("Android: stop received while no mission is active.")

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
        Handles PATH, OBJECT and correlated CONTINUE / REPLACE decisions.
        """
        while True:
            try:
                msg = self.pc.receive()
                if not msg:
                    continue

                tag = msg.split(",", 1)[0]
                if tag in ("CONTINUE", "REPLACE"):
                    try:
                        payload = json.loads(msg.split(",", 1)[1])
                    except (IndexError, json.JSONDecodeError):
                        logging.warning("Malformed PC decision; still waiting for a valid response")
                        continue
                    if not self.feedback_control.submit(tag, payload):
                        logging.warning("Ignoring stale, duplicate or unsolicited PC decision")
                    continue

                if msg.startswith("PATH"):
                    # ── PC sent back the computed path ────────────────────────
                    try:
                        payload = json.loads(msg.split("PATH,", 1)[1])
                    except (IndexError, json.JSONDecodeError) as exc:
                        logging.error(f"PC: malformed PATH message — {exc}")
                        continue

                    with self._idx_lock:
                        if self.started and self.instruction_mission is not None and not self.halted:
                            logging.warning("Ignoring replacement PATH during execution.")
                            continue
                        try:
                            parse_start_pose(payload.get("start"))
                            odometry_start = parse_start_pose(payload.get("odometry_start"))
                            candidate = self._validate_task1_mission(
                                InstructionMission(
                                    payload.get("segments", []), odometry_start
                                )
                            )
                        except ValueError as exc:
                            logging.error("PC: invalid PATH — %s", exc)
                            continue
                        self.segments = candidate.segments
                        self.instruction_mission = candidate
                        self.obstacle_order = payload.get("obstacle_ids", [])
                        # Optional per-segment map; see _obstacle_for_segment().
                        # Present when the planner had to split an approach
                        # across several lines to respect the §2 caps.
                        self.segment_obstacles = payload.get("segment_obstacles", [])
                        raw_safe = payload.get("photo_standoffs", {})
                        raw_selected = payload.get("selected_standoffs", {})
                        raw_us_adjustments = payload.get("ultrasonic_adjustments", {})
                        if (not isinstance(raw_safe, dict) or
                                not isinstance(raw_selected, dict) or
                                not isinstance(raw_us_adjustments, dict)):
                            raise ValueError("PATH camera standoff metadata must be objects")
                        self.photo_standoffs = {
                            str(key): [int(value) for value in values]
                            for key, values in raw_safe.items()
                            if isinstance(values, list)
                        }
                        self.selected_standoffs = {
                            str(key): int(value) for key, value in raw_selected.items()
                        }
                        self.ultrasonic_adjustments = {
                            str(key): bool(value)
                            for key, value in raw_us_adjustments.items()
                        }
                        self.directions = payload.get("dirs", [])
                        # Keep the planner's wire values for STATUS,START.
                        self.start_pose = {
                            "x": payload["start"]["x"],
                            "y": payload["start"]["y"],
                            "dir": payload["start"]["dir"],
                        }
                        self.odometry_start = {
                            "x": payload["odometry_start"]["x"],
                            "y": payload["odometry_start"]["y"],
                            "dir": payload["odometry_start"]["dir"],
                        }
                        self.direction_index = 0

                    self.path_requested = False
                    self._cancel_path_watchdog()
                    self.path_ready.set()
                    logging.info(
                        f"PC: PATH received — {len(self.segments)} segment(s), "
                        f"obstacle order: {self.obstacle_order}; "
                        f"Android start: {self.start_pose}; "
                        f"odometry start: {self.odometry_start}."
                    )

                    # If Android already sent BEGIN but PATH hadn't arrived yet,
                    # kick off the first segment now
                    if self.started and self.segments_index == 0:
                        self._send_start_status()
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

                    accepted_result = False
                    with self._detection_lock:
                        expected = self._expected_detection_id
                        if expected is not None and str(obstacle_id) == expected:
                            self._last_detection = {
                                "obstacle_id": str(obstacle_id),
                                "confidence": confidence,
                                "class_id": class_id,
                            }
                            self._expected_detection_id = None
                            accepted_result = True
                            self.image_done.set()
                        else:
                            logging.warning(
                                "Ignoring stale OBJECT for obstacle %s; waiting for %s.",
                                obstacle_id, expected,
                            )

                    # Forward result to Android
                    if not accepted_result or confidence is None:
                        pass
                    elif self.a5_mode:
                        self._a5_report(obstacle_id, class_id, confidence)
                    elif class_id.lower() not in TASK1_NOT_A_TARGET:
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
          1. Read continuous odometry and update Android.
          2. At a segment boundary, capture its obstacle if requested.
          3. Send the next instruction, or finish and request stitching.
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

                if self.halted:
                    continue

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
                        mission = self.instruction_mission
                        if mission is None or not mission.pending:
                            logging.warning("STM RESEND without an outstanding instruction.")
                            continue
                        last_idx = mission.key
                        seg = [mission.token]

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
                    if not self.stm.send_line(seg):
                        self._halt_mission("Could not retransmit outstanding instruction")

                elif reply == "OK":
                    try:
                        with self._idx_lock:
                            active = self.instruction_mission
                            if active is None or not active.pending:
                                raise ValueError("Unexpected STM OK outside a mission")
                            completed_token = active.token
                        self._wait_for_gyro_settle(completed_token)

                        with self._idx_lock:
                            mission = self.instruction_mission
                            if mission is None:
                                raise ValueError("Unexpected STM OK outside a mission")
                            key = mission.key
                            robot_message, just_finished = mission.complete_instruction(self.stm)
                            self._resend_counts.pop(key, None)
                            more_to_send = not mission.done
                            # Publish before releasing the lock or starting another move.
                            self.android.send(robot_message)
                            # PROGRESS includes pose and execution context for PC.
                            pending = None
                            first_photo_segment = mission.segment_index if just_finished is None else just_finished
                            remaining_photo_ids = []
                            for i in range(first_photo_segment, len(mission.segments)):
                                obstacle_id = self._obstacle_for_segment(i)
                                if obstacle_id is not None:
                                    remaining_photo_ids.append(obstacle_id)
                            mission.progress["remaining_photo_ids"] = remaining_photo_ids
                            mission.progress.update(self._read_ir_snapshot())
                            finished_obstacle = (
                                self._obstacle_for_segment(just_finished)
                                if just_finished is not None else None
                            )
                            if (finished_obstacle is not None and
                                    getattr(self, "ultrasonic_adjustments", {}).get(
                                        str(finished_obstacle), False
                                    )):
                                mission.progress["ultrasonic_cm"] = self._read_ultrasonic_cm()
                            mission.progress["awaiting_decision"] = self.feedback_control.enabled
                            if self.feedback_control.enabled:
                                pending = self.feedback_control.arm(mission.progress)
                            self.pc.send("PROGRESS," + json.dumps(mission.progress, separators=(",", ":")))
                    except (ValueError, OSError) as exc:
                        self._halt_mission(str(exc))
                        continue

                    if pending is not None:
                        decision = self._wait_for_feedback(pending, mission, just_finished)
                        if decision is None:
                            continue
                        if decision == "replace":
                            if not self._send_next_segment():
                                self._halt_mission("Could not send replacement instruction")
                            continue

                    if just_finished is None:
                        if not self._send_next_segment():
                            self._halt_mission("Could not send next instruction")
                        continue
                    
                    # ── Capture + detect for the obstacle we just reached ──────
                    obstacle_id = self._obstacle_for_segment(just_finished)
                    if obstacle_id is not None:
                        self._detect_with_distance_retry(obstacle_id, mission)
                        if self.halted:
                            continue
                        self._completed_photo_count += max(
                            1, getattr(self, "_last_capture_attempts", 0)
                        )

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
                        shots = self._completed_photo_count
                        self.pc.send(f"STITCH,{shots}\n")
                        self.android.send("STATUS,DONE")
                        self.started = False

                        if self.a5_mode:
                            if self._a5_found is not None:
                                face, name, conf = self._a5_found
                                logging.info(
                                    f"A.5 finished: '{name}' (conf={conf:.2f}) on "
                                    f"the {face} face."
                                )
                            else:
                                logging.warning(
                                    "A.5 finished having seen all four faces "
                                    "without a valid image. Either the framing is "
                                    "off at this standoff or the obstacle was not "
                                    "where Android said — check the stills the PC "
                                    "saved under pc/received_images/."
                                )
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
        first move. Task 1 itself uses ?US plus F/R correction; A.5 still uses
        FU and therefore retains its protocol-version warning.

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
            # FU is no longer emitted by Task 1. A.5 still uses it, so only A.5
            # needs the protocol-v3 capability warning.
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
            elif self.a5_mode and proto_num < FU_MIN_PROTOCOL:
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
        if self.arc_profile not in (None, cal_profile.ARC_PROFILE):
            logging.error(
                "STM_ARC_PROFILE=%s conflicts with calibrated Task 1 profile %s.",
                self.arc_profile, cal_profile.ARC_PROFILE,
            )
            self.camera.stop_camera()
            self.stm.disconnect()
            return
        if not cal_profile.prepare_for_task(self.stm):
            logging.error("Task 1 calibration startup failed — refusing to move.")
            self.camera.stop_camera()
            self.stm.disconnect()
            return
        stat = self.stm.query("?STAT")
        if stat:
            logging.info("STM after calibration: %s", stat)

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
            self.halted = True
            self.feedback_control.cancel("Task 1 shutting down")
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
