"""
task2fw.py - Task 2 driven by the STM itself (firmware protocol 11).

The course - approach, swerve past obstacle 1, round obstacle 2, home and
park - runs on the STM (stm/PeripheralDrivers/Src/task2_mission.c, ported
from the MDP_WORKING_2 firmware that ran this course end to end). The Pi only:

  1. restores the Task 2 calibration (protocol 10 table) - every move the STM
     makes runs on it, and the course plans its turns on its radii;
  2. waits for Android BEGIN, then sends  !T2GO<seconds>;
  3. answers each  P,<n>  prompt with the camera's arrow:  !T2A<n><0|1>;
  4. logs the STM's  D,...  run log, and reports the result.

The STM never guesses an arrow: no answer in 20 s stops the run (FAIL,NOARROW).
The Pi's Vision does fall back to its last vote / T2MN_DEFAULT_ARROW after
T2_ARROW_TIMEOUT_S, as task2mn does, so in practice the STM always gets one.

    python3 task2fw.py             competition: Android BEGIN, PC arrows
    python3 task2fw.py --manual    floor test: Enter to start, type each arrow

Wire: stm/PROTOCOL.md section 14.
"""
import argparse
import logging
import os
import socket
import sys
import time
import uuid
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Lock, Thread

from dotenv import load_dotenv

load_dotenv(Path(__file__).with_name(".env"))

from communications.stm import STM  # noqa: E402

# Firmware with !T2GO.
T2FW_MIN_PROTOCOL = 11
# Mission seconds handed to the STM. It stops the course itself when they run
# out (FAIL,DEADLINE), independent of anything the Pi does.
SECONDS = int(os.getenv("T2FW_SECONDS", "170"))
# The calibration restored before the run. Task 2's own, else task2mn's.
CAL_NAME = os.getenv("T2FW_CAL_PROFILE", os.getenv("T2T_CAL_PROFILE", "tbricks"))
FAST = os.getenv("T2FW_FAST", os.getenv("T2T_FAST", "1")).strip() != "0"
ARROW_TIMEOUT_S = float(os.getenv("T2_ARROW_TIMEOUT_S", "8"))
GYRO_WAIT_S = float(os.getenv("T2MN_GYRO_WAIT_S", "4"))

# The STM's run-log steps that start each of task2mn's Android stages
# (task2mn.android_status). P,1 starts APPROACH_1.
STAGES = {"arrow1": "BYPASS_1", "swerved": "APPROACH_2",
          "arrow2": "ROUND_2", "far_end2": "RETURN_AND_PARK"}


class CourseSTM(STM):
    """STM whose P,<n> / D,... lines reach the course instead of being dropped
    as banner output. Everything else is the shared client, unchanged."""

    def __init__(self):
        super().__init__()
        self.course_lines: "Queue[str]" = Queue()

    def _dispatch(self, line: str) -> None:
        if line[:2].upper() in ("P,", "D,"):
            self.course_lines.put(line)
            return
        super()._dispatch(line)


def run_course(stm: CourseSTM, seconds: int, arrow_fn, stop: Event,
               status=lambda phase: None) -> str:
    """
    !T2GO<seconds>, then answer prompts until the STM's one reply. arrow_fn(n)
    returns "left" / "right" for obstacle n and may block; it runs on its own
    thread so the STM's prompts and log keep flowing. Returns the reply line
    (OK / FAIL,<why> / RESEND), or "TIMEOUT".
    """
    answers = {}
    asking = set()
    lock = Lock()

    def answer(n: int) -> None:
        try:
            direction = arrow_fn(n)
        except Exception:
            logging.exception("Arrow %d: recognition failed", n)
            return
        d = 1 if str(direction).lower().startswith("r") else 0
        with lock:
            answers[n] = d
        logging.info("Arrow %d: %s", n, "right" if d else "left")
        status("ARROW,%d,%s" % (n, "RIGHT" if d else "LEFT"))
        stm.send("!T2A%d%d" % (n, d))

    while not stm.course_lines.empty():
        stm.course_lines.get_nowait()
    while stm.receive() is not None:
        pass
    stm.send("!T2GO%d" % seconds)
    deadline = time.monotonic() + seconds + 15.0
    rst_sent = False

    while True:
        reply = stm.receive()
        if reply is not None:
            return reply
        if stop.is_set() and not rst_sent:
            # The STM brakes and answers the !T2GO line FAIL,STOP.
            stm.abort()
            rst_sent = True
        if time.monotonic() >= deadline:
            logging.error("No reply to !T2GO within %d s - RST", seconds + 15)
            stm.abort()
            return "TIMEOUT"
        try:
            line = stm.course_lines.get(timeout=0.05)
        except Empty:
            continue
        if line.startswith("D,"):
            logging.info("course %s", line)
            stage = STAGES.get(line.split(",")[1] if "," in line else "")
            if stage:
                status(stage)
            continue
        try:
            n = int(line.split(",")[1])
        except (IndexError, ValueError):
            continue
        if n not in (1, 2):
            continue
        with lock:
            known, busy = answers.get(n), n in asking
            if known is None and not busy:
                asking.add(n)
        if known is not None:
            # A repeated prompt: the answer was lost - send it again.
            stm.send("!T2A%d%d" % (n, known))
        elif not busy:
            logging.info("STM asks for arrow %d", n)
            if n == 1:
                status("APPROACH_1")
            Thread(target=answer, args=(n,), name="t2fw-arrow%d" % n, daemon=True).start()


def prepare(stm: CourseSTM) -> bool:
    """Calibration, fast mode, protocol and gyro checks. Nothing moves."""
    from communications import cal_profile

    proto = cal_profile.firmware_protocol(stm)
    if proto is None or proto < T2FW_MIN_PROTOCOL:
        logging.error("Firmware protocol %s has no !T2GO (needs %d) - flash the "
                      "current stm/ build.", proto, T2FW_MIN_PROTOCOL)
        return False
    if not cal_profile.prepare_for_task(stm, name=CAL_NAME, required=True, fast=FAST):
        logging.error("Calibration %r could not be restored - not starting.", CAL_NAME)
        return False
    logging.info("Task 2 (firmware course) on calibration %r, %s profile, fast %s",
                 CAL_NAME, cal_profile.ARC_NAMES.get(cal_profile.selected_arc, "?"),
                 "on" if FAST else "off")
    deadline = time.monotonic() + GYRO_WAIT_S
    imu = stm.query_fields("?IMU")
    while imu and imu[0] != "1" and time.monotonic() < deadline:
        time.sleep(0.25)
        imu = stm.query_fields("?IMU")
    if not imu or imu[0] != "1":
        logging.error("Gyro not ready (%s) - keep the robot still and retry.", imu)
        return False
    us = stm.query_fields("?US")
    logging.info("Start: ?US %s, ?IR %s", us, stm.query_fields("?IR"))
    return True


def finish(stm: CourseSTM) -> None:
    """Leave the robot in normal mode for the next task."""
    if FAST:
        try:
            stm.set_fast(False)
        except Exception:
            logging.exception("Could not send !FAST0")


def manual() -> int:
    """Floor test: no Android, no PC. Enter to start; type each arrow."""
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s - %(message)s")
    stm, stop = CourseSTM(), Event()
    stm.connect()
    try:
        if not prepare(stm):
            return 1
        input("Robot in the carpark, facing out. Enter to run the course... ")

        def typed(n: int) -> str:
            while True:
                a = input("Obstacle %d arrow [l/r]: " % n).strip().lower()
                if a[:1] in ("l", "r"):
                    return a

        t0 = time.monotonic()
        try:
            reply = run_course(stm, SECONDS, typed, stop)
        except KeyboardInterrupt:
            stop.set()
            stm.abort()
            reply = "FAIL,STOP"
        logging.info("Course reply %s after %.1f s", reply, time.monotonic() - t0)
        return 0 if reply.strip().upper() == "OK" else 1
    finally:
        finish(stm)
        stm.disconnect()


def competition() -> int:
    """Android BEGIN, camera + PC arrows - task2mn's harness around the
    firmware course."""
    try:
        import run_log
        run_log.setup("task2fw", "%(asctime)s [%(threadName)s] %(levelname)s - %(message)s")
    except ImportError:
        logging.basicConfig(level=logging.INFO,
                            format="%(asctime)s [%(threadName)s] %(levelname)s - %(message)s")
    from communications.android import Android
    from communications.pc import PC
    from image_capture.camera import Camera
    from task2mn import android_status, task2_control_status
    from task2mn_vision import Vision

    stop, go, ready = Event(), Event(), Event()
    android, pc, stm = Android(), PC(), CourseSTM()
    android_lock = Lock()
    vision = Vision(pc, stop, uuid.uuid4().hex)
    workers = []
    final = [None]

    def send_android(message):
        with android_lock:
            if android.connected:
                try:
                    android.send(message)
                except Exception:
                    logging.warning("Could not send Android update %s", message, exc_info=True)

    def status(phase):
        if phase in ("DONE", "FAILED"):
            if final[0] is not None:
                return
            final[0] = phase
        logging.info("Task 2 phase: %s", phase)
        send_android(android_status(phase))

    def worker(name, fn):
        def guarded():
            try:
                fn()
            except Exception:
                if not stop.is_set():
                    logging.exception("%s failed", name)
                    stop.set()
        t = Thread(target=guarded, name=name, daemon=True)
        workers.append(t)
        t.start()

    def android_loop():
        while not stop.is_set():
            message = android.receive()
            if message is None:
                stop.wait(0.05)
                continue
            command = message.strip().upper()
            if command in ("S", "STOP", "RST"):
                logging.error("Android STOP")
                stop.set()
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
            for jpeg in camera.continuous_images(
                    stop, exposure_us=int(os.getenv("T2_EXPOSURE_US", "5000"))):
                vision.publish(jpeg)
        finally:
            camera.stop_camera()

    def pc_loop():
        pc.connect()
        if stop.is_set():
            return
        pc.client_socket.settimeout(float(os.getenv("T2MN_PC_TIMEOUT_S", "15")))
        try:
            vision.serve()
        except Exception as exc:
            if not vision.ready or stop.is_set():
                raise
            vision.fail(str(exc))

    def arrow(n: int) -> str:
        vision.begin(n)
        try:
            return vision.wait_arrow(ARROW_TIMEOUT_S)
        finally:
            vision.idle()

    result = 1
    try:
        stm.connect()
        if not prepare(stm):
            raise RuntimeError("STM startup checks failed")
        android.start()
        worker("t2-android", android_loop)
        worker("t2-camera", camera_loop)
        worker("t2-pc", pc_loop)
        startup = time.monotonic() + 120
        while not stop.is_set() and not (vision.ready and vision.latest is not None):
            if time.monotonic() >= startup:
                raise RuntimeError("Camera/PC not ready within 120 s")
            stop.wait(0.1)
        if not stop.is_set():
            ready.set()
            status("READY")
            send_android("TASK2,READY")
        while not stop.is_set() and not go.wait(0.1):
            pass
        if not stop.is_set():
            t0 = time.monotonic()
            status("START")
            reply = run_course(stm, SECONDS, arrow, stop, status)
            logging.info("Course reply %s after %.1f s", reply, time.monotonic() - t0)
            result = 0 if reply.strip().upper() == "OK" else 1
            status("DONE" if result == 0 else "FAILED")
    except KeyboardInterrupt:
        stop.set()
        stm.abort()
    except Exception:
        logging.exception("Task 2 (firmware course) failed")
        status("FAILED")
    finally:
        stop.set()
        finish(stm)
        for sock in (pc.client_socket, pc.server_socket):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
        pc.disconnect()
        android.stop()
        for t in workers:
            t.join(timeout=3)
        stm.disconnect()
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--manual", action="store_true",
                    help="floor test: Enter to start, type each arrow")
    a = ap.parse_args()
    return manual() if a.manual else competition()


if __name__ == "__main__":
    sys.exit(main())
