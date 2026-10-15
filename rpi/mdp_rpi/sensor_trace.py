"""
sensor_trace.py - what the side IRs said, when, and what the Pi decided.

Every face-scan poll writes one CSV row: the IR pair as the STM reported it,
the pose paired with it, how long each query took, how long since the last
poll, the speed, and the edge-vote state. Open it in a spreadsheet next to
the run video to see whether the robot is reacting late (look at poll_dt_ms
and the *_rtt_ms columns) or reacting to the wrong thing (look at ir_* and
decision).

    T2_SENSOR_TRACE=0      turn it off
    logs/sensor_trace_<YYYYmmdd_HHMMSS>.csv

At the end of the run, summary() logs median / 95th-percentile timings.
"""

import csv
import logging
import os
import time
from datetime import datetime
from typing import Dict, List, Optional

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

COLUMNS = (
    "t_ms", "phase", "side", "ir_left", "ir_right", "ir_used", "us",
    "x", "y", "th_deg", "speed_cm_s", "poll_dt_ms", "ir_rtt_ms",
    "us_rtt_ms", "pose_rtt_ms", "ir_to_pose_ms", "baseline", "edge_votes",
    "seen", "decision",
)


def _pct(values: List[float], q: float) -> float:
    s = sorted(values)
    return s[min(len(s) - 1, int(round(q * (len(s) - 1))))]


class SensorTrace:
    def __init__(self, enabled: Optional[bool] = None,
                 log_dir: Optional[str] = None):
        if enabled is None:
            enabled = os.getenv("T2_SENSOR_TRACE", "1").strip() != "0"
        self.enabled = enabled
        self.t0 = time.monotonic()
        self.path: Optional[str] = None
        self._file = None
        self._writer = None
        self._timings: Dict[str, List[float]] = {}
        self._log_dir = log_dir or LOG_DIR

    def _open(self) -> None:
        try:
            os.makedirs(self._log_dir, exist_ok=True)
            self.path = os.path.join(
                self._log_dir,
                f"sensor_trace_{datetime.now():%Y%m%d_%H%M%S}.csv")
            self._file = open(self.path, "w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._file, COLUMNS,
                                          extrasaction="ignore")
            self._writer.writeheader()
            logging.info("Sensor trace: %s", self.path)
        except OSError as exc:
            logging.warning("Sensor trace disabled: %s", exc)
            self.enabled = False

    def now_ms(self) -> float:
        return (time.monotonic() - self.t0) * 1000.0

    def row(self, **fields) -> None:
        if not self.enabled:
            return
        if self._writer is None:
            self._open()
            if not self.enabled:
                return
        fields.setdefault("t_ms", self.now_ms())
        for key in ("poll_dt_ms", "ir_rtt_ms", "us_rtt_ms", "pose_rtt_ms",
                    "ir_to_pose_ms"):
            v = fields.get(key)
            if isinstance(v, (int, float)):
                self._timings.setdefault(key, []).append(float(v))
        self._writer.writerow({k: (f"{v:.1f}" if isinstance(v, float) else v)
                               for k, v in fields.items()})
        # Flushed per row: a crash or Ctrl-C mid-run must not lose the trace.
        self._file.flush()

    def summary(self) -> None:
        for key, values in self._timings.items():
            if values:
                logging.info("Sensor timing %s: n=%d median %.1f ms, p95 %.1f ms,"
                             " max %.1f ms.", key, len(values), _pct(values, 0.5),
                             _pct(values, 0.95), max(values))
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
