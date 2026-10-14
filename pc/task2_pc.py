"""Task 2 arrow service. Run on the PC; the Pi runs task2.py."""
import argparse
import json
import logging
import os
import socket
import struct
import sys
import time
from functools import partial
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def exact(stream, size):
    data = bytearray()
    while len(data) < size:
        chunk = stream.read(size - len(data))
        if not chunk:
            raise ConnectionError("Image transfer ended early")
        data.extend(chunk)
    return bytes(data)


def discard_frame(directory, path):
    """Remove transient raw and annotated evidence for one camera frame."""
    for candidate in (path, directory / "annotated" / path.name):
        try:
            candidate.unlink()
        except FileNotFoundError:
            pass


def serve(sock, detector, directory, mapping, stitch):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    saved, frames = {}, {}
    with sock.makefile("rb") as stream:
        hello = stream.readline(4096).decode().strip()
        if not hello.startswith("TASK2,"):
            raise ValueError("Expected Task 2 handshake")
        run_id = json.loads(hello.split(",", 1)[1])["run_id"]
        def send(line):
            sock.sendall((line + "\n").encode())
        send("READY,TASK2")
        while True:
            line = stream.readline(4096)
            if not line:
                return
            verb, payload = line.decode().strip().split(",", 1)
            request = json.loads(payload)
            if request.get("run_id") != run_id or request.get("stage") not in (1, 2):
                raise ValueError("Invalid run/stage")
            frame = request["frame_id"]
            if not isinstance(frame, int) or frame < 1:
                raise ValueError("Invalid frame ID")
            if verb == "FRAME":
                size = struct.unpack(">I", exact(stream, 4))[0]
                if not 0 < size <= 8 * 1024 * 1024:
                    raise ValueError("Invalid JPEG size")
                path = directory / ("stage%d_frame%d.jpg" % (request["stage"], frame))
                path.write_bytes(exact(stream, size))
                label, confidence = detector(path, allowed_classes=set(mapping),
                                             output_dir=directory / "annotated",
                                             selection_policy=("nearest" if request["stage"] == 1
                                                               else "unique_arrow"))
                direction = mapping.get(str(label).lower())
                reply = dict(request, direction=direction, confidence=float(confidence or 0))
                if direction is None:
                    discard_frame(directory, path)
                else:
                    frames[frame] = (request["stage"], path, direction)
                    while len(frames) > 16:
                        _, old_path, _ = frames.pop(next(iter(frames)))
                        if directory / "annotated" / old_path.name not in saved.values():
                            discard_frame(directory, old_path)
                send("ARROW," + json.dumps(reply))
                logging.info("Stage %s frame %s: %s confidence %.3f", request["stage"],
                             frame, direction, confidence or 0)
            elif verb == "ACCEPT":
                if frame not in frames:
                    raise ValueError("No retained arrow evidence for acceptance")
                stage, path, direction = frames[frame]
                if stage != request["stage"] or direction is None:
                    raise ValueError("No valid arrow evidence for acceptance")
                for old_frame, (old_stage, old_path, _) in list(frames.items()):
                    if old_stage == stage and old_frame != frame:
                        discard_frame(directory, old_path)
                        del frames[old_frame]
                saved[stage] = directory / "annotated" / path.name
                stitch(saved, directory / "stitched_task2.jpg")
                send("SAVED," + str(frame))
            else:
                raise ValueError("Unknown Task 2 message")


def stitch_images(saved, output):
    import cv2
    import numpy as np
    canvas = np.zeros((540, 1920, 3), dtype=np.uint8)
    for stage, path in saved.items():
        image = cv2.imread(str(path))
        if image is None:
            raise RuntimeError("Missing annotated evidence: %s" % path)
        scale = min(960 / image.shape[1], 540 / image.shape[0])
        tile = cv2.resize(image, (int(image.shape[1] * scale), int(image.shape[0] * scale)))
        x = (stage - 1) * 960
        canvas[:tile.shape[0], x:x + tile.shape[1]] = tile
    temporary = output.with_name("stitched_task2.tmp.jpg")
    if not cv2.imwrite(str(temporary), canvas):
        raise RuntimeError("Could not save stitched evidence")
    os.replace(str(temporary), str(output))


def next_run_directory():
    """Return a fresh evidence directory for one Pi connection."""
    stem = "task2_" + time.strftime("%Y%m%d_%H%M%S")
    directory = ROOT / "runs" / stem
    suffix = 2
    while directory.exists():
        directory = ROOT / "runs" / (stem + "_%d" % suffix)
        suffix += 1
    return directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.15.15")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--left-label", default="left_arrow")
    parser.add_argument("--right-label", default="right_arrow")
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT.parent / "rpi" / "mdp_rpi"))
    import run_log
    run_log.setup("task2_pc", "%(asctime)s [T2 PC] %(levelname)s - %(message)s",
                  log_dir=str(ROOT / "logs"))
    from image_recognition.detect import detect, model
    names = model.names.values() if isinstance(model.names, dict) else model.names
    labels = {str(value).lower() for value in names}
    mapping = {args.left_label.lower(): "left", args.right_label.lower(): "right"}
    if len(mapping) != 2 or not set(mapping) <= labels:
        raise ValueError("Set --left-label/--right-label to the model's actual arrow labels: %s" % sorted(labels))
    # Pay model warm-up cost before allowing Android BEGIN.
    import numpy as np
    model.predict(source=np.zeros((540, 960, 3), dtype=np.uint8), verbose=False)
    logging.info("Arrow mapping %s", mapping)
    while True:
        sock = None
        waiting_since = time.monotonic()
        waiting_logged = False
        while sock is None:
            try:
                sock = socket.create_connection((args.host, args.port), timeout=5)
            except OSError:
                if not waiting_logged:
                    logging.info("Waiting for Pi %s:%s (further retries are quiet)",
                                 args.host, args.port)
                    waiting_logged = True
                time.sleep(1)

        directory = next_run_directory()
        logging.info("Connected to Pi after %.1fs; run evidence: %s",
                     time.monotonic() - waiting_since, directory)
        try:
            with sock:
                # Android may start much later; idle time is not a failed connection.
                sock.settimeout(None)
                serve(sock, partial(detect, target_confidence=0.70),
                      directory, mapping, stitch_images)
            logging.info("Pi session ended; waiting for the next Task 2 run.")
        except KeyboardInterrupt:
            raise
        except (ConnectionError, OSError, ValueError, json.JSONDecodeError):
            logging.exception("Task 2 Pi session failed; waiting for a new run.")
        time.sleep(0.5)


if __name__ == "__main__":
    main()
