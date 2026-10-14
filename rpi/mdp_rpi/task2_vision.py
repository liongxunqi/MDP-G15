"""Task 2 latest-frame recognition; exactly one worker owns the PC socket."""
import json
import logging
import math
import time
from threading import Condition


class Vision:
    def __init__(self, pc, stop, run_id, max_age=2.5, confidence=0.70):
        self.pc, self.stop, self.run_id = pc, stop, run_id
        self.max_age, self.confidence = max_age, confidence
        self.condition = Condition()
        self.latest = None
        self.stage = 0
        self.epoch = 0
        self.stage_started = 0
        self.result = None
        self.candidate_time = None
        self.ready = False
        self.last_frame_time = time.monotonic()

    def publish(self, jpeg):
        with self.condition:
            self.last_frame_time = time.monotonic()
            number = 1 if self.latest is None else self.latest[0] + 1
            self.latest = (number, self.last_frame_time, jpeg)
            self.condition.notify_all()

    def begin(self, stage):
        with self.condition:
            self.stage = stage
            self.epoch += 1
            self.stage_started = time.monotonic()
            self.result = None
            self.candidate_time = None
            self.condition.notify_all()

    def wait_arrow(self, timeout=8.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.result is None and not self.stop.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("No confirmed arrow at safe decision point")
                self.condition.wait(min(remaining, 0.1))
            if self.stop.is_set():
                raise RuntimeError("Task 2 cancelled")
            return self.result

    def peek_arrow(self):
        """Return a confirmed arrow without waiting or consuming it."""
        with self.condition:
            return self.result

    def arrow_pending(self):
        """A fresh first vote is awaiting confirmation in the current stage."""
        with self.condition:
            return (self.candidate_time is not None and self.stage != 0
                    and time.monotonic() - self.candidate_time <= self.max_age)

    def idle(self):
        self.begin(0)

    def serve(self):
        self.pc.send("TASK2," + json.dumps({"run_id": self.run_id}))
        if self.pc.receive() != "READY,TASK2":
            raise RuntimeError("PC did not acknowledge the Task 2 protocol")
        with self.condition:
            self.ready = True
            self.condition.notify_all()
        seen, vote, count, vote_time, previous_epoch = -1, None, 0, 0, -1
        while not self.stop.is_set():
            with self.condition:
                self.condition.wait_for(lambda: self.stop.is_set() or (
                    self.stage and self.result is None and self.latest is not None
                    and self.latest[0] != seen), timeout=0.2)
                if self.stop.is_set():
                    return
                if not self.stage or self.result is not None or self.latest is None:
                    continue
                frame, captured, jpeg = self.latest
                if frame == seen:
                    continue
                seen = frame
                if captured < self.stage_started:
                    continue
                stage, epoch = self.stage, self.epoch
            seen = frame
            if time.monotonic() - captured > self.max_age:
                continue
            if epoch != previous_epoch:
                vote, count = None, 0
                previous_epoch = epoch
            request = {"run_id": self.run_id, "stage": stage, "frame_id": frame}
            self.pc.send_image(jpeg, header="FRAME," + json.dumps(request))
            message = self.pc.receive()
            if not message or not message.startswith("ARROW,"):
                raise RuntimeError("PC recognition connection failed")
            reply = json.loads(message.split(",", 1)[1])
            if any(reply.get(key) != value for key, value in request.items()):
                raise RuntimeError("PC returned a mismatched frame/stage identifier")
            now = time.monotonic()
            direction = reply.get("direction")
            conf = float(reply.get("confidence", 0))
            valid = (direction in ("left", "right") and math.isfinite(conf)
                     and self.confidence <= conf <= 1 and now - captured <= self.max_age)
            with self.condition:
                if epoch != self.epoch:
                    continue
                if not valid:
                    vote, count = None, 0
                    self.candidate_time = None
                    continue
                count = count + 1 if direction == vote and now - vote_time <= self.max_age else 1
                vote, vote_time = direction, now
                self.candidate_time = now
                logging.info("Arrow stage %s frame %s: %s %.3f (%s/2)",
                             stage, frame, direction, conf, count)
            if count >= 2:
                # Network I/O must not hold the camera publication lock.
                self.pc.send("ACCEPT," + json.dumps(request))
                if self.pc.receive() != "SAVED," + str(frame):
                    raise RuntimeError("PC did not confirm arrow evidence was saved")
                with self.condition:
                    if epoch != self.epoch or self.stop.is_set():
                        continue
                    self.result = direction
                    self.condition.notify_all()
