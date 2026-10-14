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


class Motion:
    """One mission thread owns movement; abort is the only cross-thread command."""
    def __init__(self, stm, stop, report):
        self.raw, self.stop, self.report = stm, stop, report
        self.lock = Lock()

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
                self.report()
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

    def set_fast(self, on, timeout=None):
        with self.lock:
            self.check()
            return self.raw.set_fast(on, timeout)


class Task2(route.Task2):
    """Task-specific route decisions; communication helpers remain shared."""
    def __init__(self, stm, vision, stop, status):
        super().__init__(stm, input_fn=self._arrow, sleep_fn=stop.wait,
                         startup=False, wait_for_go=False,
                         early_arrow_fn=getattr(vision, "peek_arrow", None),
                         lane_reached_fn=self._lane_reached)
        self.vision, self.stop, self.status = vision, stop, status
        self.obstacle = 0
        self.stage2_started = False
        # main() has already enabled this on the firmware. Keeping the route's
        # state in sync lets its precision phase issue !FAST0 and !FAST1.
        self.fast = route.FAST

    def _pose(self):
        pose = self.frame.pose()
        if pose is None:
            raise RuntimeError("No valid measured world pose")
        return pose

    def _arrow(self, prompt):
        direction = self.vision.wait_arrow(float(os.getenv("T2_ARROW_TIMEOUT_S", "8")))
        logging.info("Confirmed obstacle %s arrow: %s", self.obstacle, direction)
        self.status("ARROW,%s,%s" % (self.obstacle, direction.upper()))
        self.vision.idle()
        return direction

    def obstacle1(self):
        self.obstacle = 1
        self.status("APPROACH_1")
        self.vision.begin(1)
        return super().obstacle1()

    def pass_obstacle1(self, x1, side1):
        self.status("BYPASS_1")
        result = super().pass_obstacle1(x1, side1)
        if result is not None and not self.stage2_started:
            self._lane_reached()
        return result

    def _lane_reached(self):
        """Start stage 2 as soon as the robot enters the bypass lane."""
        if self.stage2_started:
            return
        self.obstacle = 2
        self.status("APPROACH_2")
        self.vision.begin(2)
        self.stage2_started = True
        logging.info("Obstacle 1 bypass lane reached; stage-2 recognition is "
                     "live during the forward cruise.")

    def photo_obstacle2(self, course, side1):
        self.obstacle = 2
        self.status("APPROACH_2")
        if not self.stage2_started:
            self.vision.begin(2)
            self.stage2_started = True
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
        choices = [(self._check(pose, segs, course), geometry._cost(segs, self.r), segs)
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
        us = self._us()
        left, right = self._ir("left"), self._ir("right")
        offset = geometry.centre_offset(left, right)
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
        margin = 2.0
        aligned = pose is not None and math.cos(pose.th) <= -math.cos(math.radians(10))
        sensor_ok = False
        if (aligned and all(v is not None and math.isfinite(v) and v > 0
                            for v in (us, left, right)) and offset is not None):
            length = (geometry.ROBOT_LENGTH_CM * abs(math.cos(pose.th))
                      + geometry.ROBOT_WIDTH_CM * abs(math.sin(pose.th)))
            front_gap = us * abs(math.cos(pose.th))
            side_gap = min(left, right) - (geometry.HALF_WIDTH_CM
                                          - geometry.IR_SIDE_OFFSET_CM)
            sensor_ok = (front_gap >= margin and side_gap >= margin
                         and front_gap + length <= geometry.BAY_DEPTH_CM - margin)
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
                android.send(message)
    def status(phase):
        logging.info("Task 2 phase: %s", phase)
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
            elif command == "BEGIN":
                if ready.is_set() and not go.is_set():
                    go.set()
                elif not ready.is_set():
                    status("NOT_READY")
    def camera_loop():
        camera = Camera()
        try:
            for jpeg in camera.continuous_images(
                    stop, exposure_us=int(os.getenv("T2_EXPOSURE_US", "5000"))):
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
        profile_name = os.getenv("T2T_CAL_PROFILE", "tbricks")
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
        if route.FAST:
            if not stm.set_fast(True):
                raise RuntimeError(
                    "T2T_FAST=1 but STM refused !FAST1; flash fast-mode firmware"
                )
            fast_active = True
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
        while not stop.is_set() and not go.wait(0.1):
            pass
        if not stop.is_set():
            started[0] = time.monotonic()
            status("START")
            result = Task2(motion, vision, stop, status).run()
            status("DONE" if result == 0 else "FAILED")
    except KeyboardInterrupt:
        cancel("Keyboard interrupt")
    except Exception:
        logging.exception("Task 2 failed")
        status("FAILED")
    finally:
        stop.set()
        try:
            motion.abort()
        except Exception:
            logging.exception("Could not send emergency brake")
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
