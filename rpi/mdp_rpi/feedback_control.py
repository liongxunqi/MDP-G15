"""One-shot, correlated PC decisions at completed-instruction boundaries."""

import math
import time
from threading import Event, Lock
from uuid import uuid4


class FeedbackControl:
    def __init__(self, enabled=False, timeout=30.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("TASK1_FEEDBACK_TIMEOUT_S must be finite and positive")
        self.enabled = enabled
        self.timeout = timeout
        self._lock = Lock()
        self._pending = None

    @property
    def waiting(self):
        with self._lock:
            return self._pending is not None

    def arm(self, progress):
        with self._lock:
            if self._pending is not None:
                raise ValueError("Already waiting for a PC decision")
            pending = {
                "feedback_id": uuid4().hex,
                "segment_index": progress["segment_index"],
                "instruction_index": progress["instruction_index"],
                "deadline": time.monotonic() + self.timeout,
                "event": Event(),
                "response": None,
            }
            self._pending = pending
            progress.update({k: pending[k] for k in
                             ("feedback_id", "segment_index", "instruction_index")})
            progress["awaiting_decision"] = True
            progress["decision_timeout_s"] = self.timeout
            return pending

    def submit(self, action, payload):
        with self._lock:
            pending = self._pending
            if (action not in ("CONTINUE", "REPLACE") or
                    not isinstance(payload, dict) or pending is None or
                    payload.get("feedback_id") != pending["feedback_id"] or
                    pending["response"] is not None or
                    time.monotonic() >= pending["deadline"]):
                return False
            for key in ("segment_index", "instruction_index"):
                if type(payload.get(key)) is not int or payload[key] != pending[key]:
                    action, payload = "INVALID", {"reason": "Decision indexes do not match feedback"}
                    break
            pending["response"] = (action, payload)
            pending["event"].set()
            return True

    def wait(self, pending):
        pending["event"].wait(max(0.0, pending["deadline"] - time.monotonic()))
        with self._lock:
            if self._pending is pending:
                self._pending = None
            response = pending["response"]
        return ("TIMEOUT", {}) if response is None else response

    def cancel(self, reason):
        with self._lock:
            if self._pending is not None:
                self._pending["response"] = ("CANCEL", {"reason": reason})
                self._pending["event"].set()
                self._pending = None
