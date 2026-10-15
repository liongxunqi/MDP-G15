"""Autonomous Task 2 entry point: Android BEGIN, TIGHT route, PC arrows, parking."""
import logging
import math
import os
import socket
import time
import uuid
from pathlib import Path
from threading import Event, Lock, Thread

from dotenv import load_dotenv

load_dotenv(Path(__file__).with_name(".env"))

import task2t as route
import task2t_park as geometry
from task2_vision import Vision


def android_status(phase):
    """Keep the existing Android STATUS grammar; details go to the text log."""
    stages = {"START": 0, "APPROACH_1": 1, "BYPASS_1": 2,
              "APPROACH_2": 3, "ROUND_2": 4, "RETURN_AND_PARK": 5}
    if phase in ("DONE", "FAILED"):
        return "STATUS," + phase
    if phase in stages:
        return "STATUS,RUNNING,%d,5" % stages[phase]
    if phase == "READY":
        return "STATUS,OK"
    return "TASK2," + phase


def task2_control_status(ready, running):
    """Answer a tablet readiness query without relying on the one-time READY log."""
    return "TASK2,RUNNING" if running else "TASK2,READY" if ready else "TASK2,NOT_READY"


class Motion:
    """One mission thread owns movement; abort is the only cross-thread command."""
    def __init__(self, stm, stop, report):
        self.raw, self.stop, self.report = stm, stop, report
        self.lock = Lock()
        self.supports_fast = True
        self.supports_v_speed = False
        self.fast_enabled = False

    def check(self):
        if self.stop.is_set():
            raise RuntimeError("Task 2 cancelled")

    def send_line(self, line):
        with self.lock:
            self.check()
            return self.raw.send_line(line)

    def abort(self):
        with self.lock:
            self.raw.abort()

    def receive(self):
        self.check()
        return self.raw.receive()

    def wait_reply(self, timeout=25.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            reply = self.receive()
            if reply is not None:
                # Route reads measured WPOSE at its next decision point.
                # A telemetry-only read here duplicated that serial query.
                return reply
            self.stop.wait(0.02)
        self.abort()
        raise RuntimeError("STM movement reply timed out")

    def query_fields(self, command, timeout=0.5):
        self.check()
        return self.raw.query_fields(command, timeout)

    def read_wpose(self, timeout=0.5):
        self.check()
        return self.raw.read_wpose(timeout)

    def read_irw(self, timeout=0.5):
        """Return protocol-10 IR and pose captured in the same control tick."""
        self.check()
        reader = getattr(self.raw, "read_irw", None)
        return reader(timeout) if reader is not None else None

    def set_ir_edge(self, side, lo_cm=0, hi_cm=0, timeout=None):
        """Arm/disarm the firmware's three-tick IR acquisition latch."""
        with self.lock:
            self.check()
            setter = getattr(self.raw, "set_ir_edge", None)
            return (setter(side, lo_cm, hi_cm, timeout)
                    if setter is not None else False)

    def read_ir_edge(self, timeout=0.5):
        self.check()
        reader = getattr(self.raw, "read_ir_edge", None)
        return reader(timeout) if reader is not None else None

    def set_fast(self, on, timeout=None):
        with self.lock:
            self.check()
            if not self.supports_fast:
                return not on  # normal mode already; no unsupported FAST0
            accepted = self.raw.set_fast(on, timeout)
            if accepted:
                self.fast_enabled = bool(on)
            return accepted

    def prepare_start(self):
        from communications import cal_profile
        with self.lock:
            self.check()
            logging.info("Android BEGIN: zeroing heading at the placed start pose.")
            return cal_profile.zero_heading(self.raw)


class Task2(route.Task2):
    """Task-specific route decisions; communication helpers remain shared."""
    def __init__(self, stm, vision, stop, status):
        super().__init__(stm, input_fn=self._arrow, sleep_fn=stop.wait,
                         startup=False, wait_for_go=False,
                         early_arrow_fn=getattr(vision, "peek_arrow", None),
                         lane_reached_fn=self._lane_reached,
                         arrow_pending_fn=getattr(vision, "arrow_pending", None))
        self.vision, self.stop, self.status = vision, stop, status
        self.obstacle = 0
        self.stage2_started = False
        # main() enabled this before BEGIN. The route must know so that its
        # first precision phase actually sends FAST0.
        self.fast = route.FAST

    def _stable_us(self, attempts=4, tolerance=2.0):
        return geometry.stable_us(self._us, attempts, tolerance)

    def _pose(self):
        pose = self.frame.pose()
        if pose is None:
            raise RuntimeError("No valid measured world pose")
        return pose

    def _arrow(self, prompt):
        # Obstacle 1 vision is opened only after the stationary 45 cm camera
        # checkpoint. Stage 2 is opened by the measured parallel-lane callback.
        if self.obstacle == 1:
            self.vision.begin(1)
        direction = self.vision.wait_arrow(float(os.getenv("T2_ARROW_TIMEOUT_S", "8")))
        center_reader = getattr(self.vision, "arrow_center_x", None)
        center_x = center_reader() if center_reader is not None else None
        if self.obstacle == 1 and center_x is not None:
            # The marker is centred on the obstacle face. Convert its image
            # offset into the shared course-axis offset used by collision
            # checks. Positive world y is camera-left; image x grows right.
            hfov = float(os.getenv("T2_CAMERA_HFOV_DEG", "62.0"))
            max_offset = float(os.getenv("T2_VISION_AXIS_MAX_CM", "15.0"))
            if 20.0 <= hfov <= 120.0 and 0.0 < max_offset <= 25.0:
                offset = -(2.0 * center_x - 1.0) * route.OBS1_CAMERA_CHECKPOINT_CM \
                    * math.tan(math.radians(hfov / 2.0))
                self.obs1_y = max(-max_offset, min(max_offset, offset))
                logging.info(
                    "YOLO arrow centre x=%.3f sets obstacle axis y=%.1f cm "
                    "(HFOV %.1f deg, bounded to +/-%.1f cm).",
                    center_x, self.obs1_y, hfov, max_offset)
        logging.info("Confirmed obstacle %s arrow: %s", self.obstacle, direction)
        self.status("ARROW,%s,%s" % (self.obstacle, direction.upper()))
        self.vision.idle()
        return direction

    def obstacle1(self):
        self.obstacle = 1
        self.status("APPROACH_1")
        return super().obstacle1()

    def pass_obstacle1(self, x1, side1):
        self.status("BYPASS_1")
        result = super().pass_obstacle1(x1, side1)
        return result

    def _lane_reached(self):
        """Open stage 2 only after the camera is aimed at obstacle 2."""
        if self.stage2_started:
            return
        self.obstacle = 2
        self.status("APPROACH_2")
        self.vision.begin(2)
        self.stage2_started = True
        logging.info("Camera aimed at obstacle 2; fresh stage-2 recognition is live.")

    def photo_obstacle2(self, course, side1):
        self.obstacle = 2
        self.status("APPROACH_2")
        return super().photo_obstacle2(course, side1)

    def around_obstacle2(self, course, side1, side2):
        self.status("ROUND_2")
        result = super().around_obstacle2(course, side1, side2)
        if result is not None:
            self.status("RETURN_AND_PARK")
        return result

    def _first_clear(self, pose, candidates, course, what):
        # Prefer the shortest estimated execution time at the normal margin.
        # A fallback never relaxes clearance below the configured positive floor.
        choices = [(self._check(pose, segs, course), geometry.motion_cost(segs, self.r), segs)
                   for segs in candidates]
        for margin in (geometry.CLEARANCE_CM, geometry.CLEARANCE_FLOOR_CM):
            safe = [item for item in choices if item[0] >= margin]
            if safe:
                clearance, cost, segs = min(safe, key=lambda item: item[1])
                logging.info("%s: estimated cost %.2f, clearance %.1f cm", what, cost, clearance)
                return segs
        logging.error("No collision-checked %s from %s", what, pose)
        return None

    def run(self):
        result = super().run()
        if result:
            return result
        pose = self.frame.pose()
        us = self._stable_us()
        left, right = self._ir("left"), self._ir("right")
        imu = self.stm.query_fields("?IMU")
        try:
            stationary = int(imu[0]) == 1 and abs(float(imu[2]) / 10) <= 1.0
        except (TypeError, ValueError, IndexError):
            stationary = False
        corners = geometry.footprint(pose) if pose is not None else []
        inside = bool(corners) and all(
            -geometry.BAY_DEPTH_CM < x < 0 and abs(y) < geometry.BAY_HALF_WIDTH_CM
            for x, y in corners)
        # Completion means fully parked, not exactly at the aiming point.
        # Use independent sensor clearance as well as the odometry footprint.
        aligned = (pose is not None and geometry.parking_heading_error(pose.th)
                   <= geometry.PARK_HEADING_TOL_DEG)
        us_at_target = (us is not None and math.isfinite(us)
                        and geometry.PARK_GAP_CM - geometry.PARK_GAP_TOL_CM <= us
                        <= geometry.PARK_GAP_CM + geometry.PARK_GAP_TOL_CM)
        # Side IRs are useful when both fences are visible, but the final
        # parking decision must still work when one sensor looks through the
        # open mouth. Any side reading that is present remains a hard guard.
        side_safe = all(v is None or (math.isfinite(v)
                                      and v >= geometry.PARK_SIDE_STOP_CM)
                        for v in (left, right))
        if (not inside and pose is not None and pose.x < -geometry.REAR_FROM_AXLE_CM
                and aligned and stationary and us_at_target and side_safe):
            # The back wall is physically impassable, but wheel odometry can
            # place the front bumper beyond it. Reconstruct x from that wall.
            # Prefer both side fences for y; if one IR looks through the open
            # mouth, retain the measured y only when the other side IR is
            # present and the reconstructed whole footprint is inside.
            wall_y = geometry.centre_offset(left, right)
            if wall_y is not None:
                wall_x = (-geometry.BAY_DEPTH_CM
                          - (geometry.FRONT_FROM_AXLE_CM + us) * math.cos(pose.th))
                wall_pose = geometry.Pose(wall_x, wall_y, pose.th)
                inside = all(
                    -geometry.BAY_DEPTH_CM < x < 0
                    and abs(y) < geometry.BAY_HALF_WIDTH_CM
                    for x, y in geometry.footprint(wall_pose))
                if inside:
                    logging.warning("Parking odometry x=%.1f cm disagrees with "
                                    "the confirmed back-wall and side-IR pose "
                                    "(x=%.1f, y=%.1f cm); using the measured "
                                    "bay position for completion.",
                                    pose.x, wall_x, wall_y)
        sensor_ok = aligned and us_at_target and side_safe
        logging.info("Parking verification: pose=%s inside=%s US=%s L=%s R=%s "
                     "stationary=%s aligned=%s sensor_clearance=%s",
                     pose, inside, us, left, right, stationary, aligned, sensor_ok)
        if not (inside and sensor_ok and stationary):
            logging.error("Parking rejected: inside=%s, sensor_clearance=%s, "
                          "aligned=%s, stationary=%s", inside, sensor_ok, aligned, stationary)
        return 0 if inside and sensor_ok and stationary else 1


def main():
    import run_log
    from communications.android import Android
    from communications.pc import PC
    from communications.stm import STM
    from communications import cal_profile
    from image_capture.camera import Camera

    # The imported bench engine installs basicConfig; use the shared run logger only.
    logging.getLogger().handlers.clear()
    run_log.setup("task2", "%(asctime)s [%(threadName)s] %(levelname)s - %(message)s")
    stop, go, ready = Event(), Event(), Event()
    android, pc, stm = Android(), PC(), STM()
    android_lock = Lock()
    vision = Vision(pc, stop, uuid.uuid4().hex)
    workers = []
    started = [None]
    def send_android(message):
        with android_lock:
            if android.connected:
                try:
                    android.send(message)
                except Exception:
                    # A tablet disconnect must not abort the autonomous mission.
                    # The tablet can query Task 2 readiness after reconnecting.
                    logging.warning("Could not send Android update %s", message, exc_info=True)
    def status(phase):
        elapsed = None if started[0] is None else time.monotonic() - started[0]
        logging.info("Task 2 phase: %s", phase)
        if elapsed is not None:
            logging.info("Task 2 competition elapsed: %.3fs (%s)", elapsed, phase)
        send_android(android_status(phase))
    def report():
        pose = stm.read_wpose(timeout=0.5)
        if pose:
            logging.info("Task 2 raw WPOSE: %s", pose)
    motion = Motion(stm, stop, report)
    def cancel(reason):
        logging.error("Task 2 stop: %s", reason)
        stop.set()
        motion.abort()
    def worker(name, function):
        def guarded():
            try:
                function()
            except (Exception, SystemExit):
                if not stop.is_set():
                    logging.exception("%s failed", name)
                    cancel(name + " failed")
        thread = Thread(target=guarded, name=name, daemon=True)
        workers.append(thread)
        thread.start()
    def android_loop():
        while not stop.is_set():
            message = android.receive()
            if message is None:
                stop.wait(0.05)
                continue
            command = message.strip().upper()
            if command in ("S", "STOP", "RST"):
                cancel("Android STOP")
            elif command == "TASK2,STATUS?":
                send_android(task2_control_status(ready.is_set(), go.is_set()))
            elif command == "BEGIN":
                if ready.is_set() and not go.is_set():
                    go.set()
                elif not ready.is_set():
                    status("NOT_READY")
    def camera_loop():
        camera = Camera()
        try:
            width = int(os.getenv("T2_CAMERA_WIDTH", "960"))
            height = int(os.getenv("T2_CAMERA_HEIGHT", "540"))
            if width < 320 or height < 240:
                raise ValueError("Task 2 camera resolution is too small")
            for jpeg in camera.continuous_images(
                    stop, resolution=(width, height),
                    exposure_us=int(os.getenv("T2_EXPOSURE_US", "5000"))):
                vision.publish(jpeg)
        finally:
            camera.stop_camera()
    def pc_loop():
        pc.connect()
        if stop.is_set():
            return
        pc.client_socket.settimeout(5)
        vision.serve()
    def watchdog():
        while not stop.wait(0.2):
            if started[0] is not None:
                if time.monotonic() - started[0] >= 175:
                    cancel("175 second mission deadline")
                elif time.monotonic() - vision.last_frame_time > 5:
                    cancel("Camera frame stream stalled")
    result = 1
    fast_active = False
    try:
        stm.connect()
        profile_name = os.getenv("T2T_CAL_PROFILE", "tr00")
        profile = cal_profile.load(profile_name)
        if profile is None or profile.get("arc_profile") != 0:
            raise RuntimeError("Task 2 requires a saved TIGHT/profile-0 calibration")
        if not profile.get("verified", False):
            logging.warning("Calibration %s is marked unverified; nominal route "
                            "clearance is not a hardware safety guarantee", profile_name)
        if not cal_profile.prepare_for_task(stm, name=profile_name, required=True):
            raise RuntimeError("Calibration startup verification failed")
        if stm.read_wpose() is None:
            raise RuntimeError("Firmware does not answer ?WPOSE")
        imu = stm.query_fields("?IMU")
        ir = stm.query_fields("?IR")
        us = stm.query_fields("?US")
        if not imu or imu[0] != "1" or not ir or len(ir) != 2 or not us:
            raise RuntimeError("STM IMU/IR/US startup queries failed")
        # Query the actual board: source on disk need not be the firmware that
        # is currently flashed. Protocol 10 adds atomic IR/pose sampling.
        from communications.stm import FAST_MIN_PROTOCOL, IRW_MIN_PROTOCOL
        proto = cal_profile.firmware_protocol(stm)
        motion.supports_fast = proto is not None and proto >= FAST_MIN_PROTOCOL
        motion.supports_v_speed = proto is not None and proto >= 10
        motion.supports_irw = proto is not None and proto >= IRW_MIN_PROTOCOL
        if not motion.supports_fast:
            route.FAST = False
            logging.info("Board protocol %s: using its existing normal mode; "
                         "FAST commands will not be sent.", proto)
        if route.FAST:
            if not motion.set_fast(True):
                raise RuntimeError("Board advertised fast mode but refused !FAST1")
            fast_active = True
            logging.info("Protocol-%s !FAST1 enabled: open straights use the "
                         "firmware's calibrated 200 rpm ramp; obstacle-edge "
                         "motion remains precision speed.", proto)
        logging.info("Atomic IR/pose sampling and firmware IR latch: %s.",
                     "enabled" if motion.supports_irw else "unavailable")
        logging.info("Obstacle-2 configured width cap %.1f cm; PDFs specify "
                     "minimum 30 cm and no numeric maximum.", route.W2_RANGE[1])
        if abs(geometry.RADIUS_CM - 30.0) > 0.01:
            logging.warning("Task 2 radius override: %.2f cm", geometry.RADIUS_CM)
        logging.info("Task 2 uses %s, profile 0, radius %.2f cm; fast mode %s",
                     profile_name, geometry.RADIUS_CM,
                     "enabled" if fast_active else "disabled")
        logging.info("Geometry (cm): body %.1f x %.1f, axle-to-US %.1f, "
                     "axle-to-camera %.1f, axle-to-IR %.1f; verify on the robot",
                     geometry.ROBOT_LENGTH_CM, geometry.ROBOT_WIDTH_CM,
                     geometry.FRONT_FROM_AXLE_CM, route.CAMERA_FROM_AXLE_CM,
                     geometry.IR_FROM_AXLE_CM)
        android.start()
        worker("t2-android", android_loop)
        worker("t2-camera", camera_loop)
        worker("t2-pc", pc_loop)
        worker("t2-watchdog", watchdog)
        startup_deadline = time.monotonic() + 120
        while not stop.is_set():
            if vision.ready and vision.latest is not None:
                ready.set()
                break
            if time.monotonic() >= startup_deadline:
                raise RuntimeError("Camera/PC did not become ready within 120 seconds")
            stop.wait(0.1)
        if ready.is_set() and not stop.is_set():
            status("READY")
            send_android("TASK2,READY")
        while not stop.is_set() and not go.wait(0.1):
            pass
        if not stop.is_set():
            started[0] = time.monotonic()
            status("START")
            if not motion.prepare_start():
                raise RuntimeError("Could not zero heading at BEGIN; no route movement sent")
            result = Task2(motion, vision, stop, status).run()
            status("DONE" if result == 0 else "FAILED")
    except KeyboardInterrupt:
        cancel("Keyboard interrupt")
    except Exception:
        logging.exception("Task 2 failed")
        status("FAILED")
    finally:
        stop.set()
        if result != 0:
            try:
                motion.abort()
            except Exception:
                logging.exception("Could not send emergency brake")
        else:
            # Every movement has completed and parking has been verified.
            # Firmware v8's RST path changes Motors_Brake() back to coast, so
            # an unconditional shutdown RST can make a parked robot roll.
            logging.info("Successful park verified; no shutdown RST sent.")
        if fast_active:
            try:
                if stm.set_fast(False):
                    logging.info("STM fast mode restored to normal for the next task")
                else:
                    logging.warning("STM refused !FAST0 during shutdown")
            except Exception:
                logging.exception("Could not disable STM fast mode during shutdown")
        if result != 0:
            try:
                status("FAILED")
            except Exception:
                logging.exception("Could not send failure status to Android")
        # Shutdown sockets wakes blocked network workers before joining them.
        for sock in (pc.client_socket, pc.server_socket):
            if sock is None:
                continue
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        pc.disconnect()
        android.stop()
        for thread in workers:
            thread.join(timeout=3)
            if thread.is_alive():
                logging.error("Worker did not exit within 3s: %s", thread.name)
        stm.disconnect()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
