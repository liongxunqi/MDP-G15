"""
pc/test_pc_resilience.py  —  task1_pc must survive a dying link or a bad plan
──────────────────────────────────────────────────────────────────────────────
Runs the real task1_pc.main() against a fake RPi on localhost. No robot, no
YOLO (stubbed), no OpenCV needed.

    python3 test_pc_resilience.py

Two cases, each must finish with PASS:

  1. LINK DIES MID-PLAN
     The RPi sends OBSTACLES, then hard-closes (TCP RST) while the PC is still
     planning - what happens when task1.py on the RPi is restarted or the
     hotspot drops during a long plan. On Windows the PC used to die with
     "ConnectionAbortedError [WinError 10053]" / "[WinError 10054]". Now it
     must log ONE clear error and return from main() normally.

  2. PLANNER RAISES ON ONE LAYOUT
     The first OBSTACLES makes compute_path() raise. The PC must log the
     traceback, send nothing for that request (the RPi's PATH watchdog then
     reports STATUS,FAILED), and STILL answer the second OBSTACLES message.
"""

import json
import os
import socket
import struct
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

fake_detect = types.ModuleType("image_recognition.detect")
fake_detect.detect = lambda image_path: (None, None)
sys.modules["image_recognition.detect"] = fake_detect
try:
    import cv2  # noqa: F401
except ImportError:
    sys.modules["cv2"] = types.ModuleType("cv2")

import task1_pc  # noqa: E402

GOOD = [{"id": 1, "x": 10, "y": 10, "d": 0}]
_real_compute_path = task1_pc.compute_path


def _serve(handler):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    task1_pc.RPI_IP, task1_pc.RPI_PORT = server.getsockname()
    threading.Thread(target=handler, args=(server,), daemon=True).start()
    return server


def case_link_dies_mid_plan() -> bool:
    def rpi(server):
        conn, _ = server.accept()
        conn.sendall(f"OBSTACLES,{json.dumps(GOOD)}\n".encode())
        time.sleep(0.05)
        # SO_LINGER (on, 0): close() sends RST instead of a polite FIN.
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
        conn.close()

    def slow(obstacles):                # keep the PC busy until the RST lands
        time.sleep(0.5)
        return _real_compute_path(obstacles)

    task1_pc.compute_path = slow
    server = _serve(rpi)
    try:
        task1_pc.main()                 # must RETURN, not raise
    except Exception as exc:            # noqa: BLE001
        print(f"  main() raised {exc!r}")
        return False
    finally:
        task1_pc.compute_path = _real_compute_path
        server.close()
    return True


def case_planner_raises_once() -> bool:
    replies = []

    def rpi(server):
        conn, _ = server.accept()
        with conn:
            conn.sendall(f"OBSTACLES,{json.dumps(GOOD)}\n".encode())   # 1st: planner crashes
            time.sleep(0.5)                                            # let it finish crashing
            conn.sendall(f"OBSTACLES,{json.dumps(GOOD)}\n".encode())   # 2nd: must still be answered
            conn.settimeout(10)
            buf = b""
            while b"\n" not in buf:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
            replies.append(buf.decode().strip())

    calls = {"n": 0}

    def flaky(obstacles):
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("simulated planner bug")
        return _real_compute_path(obstacles)

    task1_pc.compute_path = flaky
    server = _serve(rpi)
    try:
        task1_pc.main()
    except Exception as exc:            # noqa: BLE001
        print(f"  main() raised {exc!r}")
        return False
    finally:
        task1_pc.compute_path = _real_compute_path
        server.close()

    ok = calls["n"] == 2 and len(replies) == 1 and replies[0].startswith("PATH,")
    if not ok:
        print(f"  planner calls={calls['n']}, replies={replies}")
    return ok


def main() -> int:
    results = {
        "link dies mid-plan": case_link_dies_mid_plan(),
        "planner raises once": case_planner_raises_once(),
    }
    print()
    for name, ok in results.items():
        print(f"{'PASS' if ok else 'FAIL'}  {name}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
