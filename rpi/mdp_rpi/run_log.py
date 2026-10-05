"""
run_log.py  —  per-run log file + end-of-run problem summary
─────────────────────────────────────────────────────────────
Call setup() once, at the top of an entry point, in place of
logging.basicConfig(). Every run then:

  • prints to the console exactly as before
  • writes the full log to  logs/<name>_<YYYYmmdd_HHMMSS>.log
  • logs uncaught exceptions (main thread AND worker threads) with traceback
  • on exit, prints and saves a summary of every WARNING / ERROR / CRITICAL

Only the newest KEEP_RUNS logs per name are kept.
"""

import atexit
import logging
import os
import re
import sys
import threading
import time
from datetime import datetime

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
KEEP_RUNS = 30


class _ProblemCollector(logging.Handler):
    """Remembers every WARNING-or-worse record for the end-of-run summary."""

    def __init__(self):
        super().__init__(logging.WARNING)
        self.records = []

    def emit(self, record):
        self.records.append(record)


def setup(name: str, fmt: str, datefmt: str = "%H:%M:%S") -> str:
    """Configure root logging for this run. Returns the log file path."""
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, f"{name}_{datetime.now():%Y%m%d_%H%M%S}.log")

    console = logging.StreamHandler()
    console.setFormatter(logging.Formatter(fmt, datefmt))
    # Full date in the file: logs outlive the day they were written.
    file = logging.FileHandler(path, encoding="utf-8")
    file.setFormatter(logging.Formatter(fmt, "%Y-%m-%d %H:%M:%S"))
    collector = _ProblemCollector()

    root = logging.getLogger()
    root.setLevel(logging.INFO)
    for handler in (console, file, collector):
        root.addHandler(handler)

    _install_exception_hooks()
    # Registered after `import logging`, so it runs before logging.shutdown()
    # closes the file handler (atexit is LIFO).
    atexit.register(_summarise, collector, path, time.monotonic())
    _prune(name)

    logging.info(f"Logging this run to {path}")
    return path


def _install_exception_hooks():
    def main_hook(exc_type, exc, tb):
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc, tb)
            return
        logging.critical("Uncaught exception — program crashed", exc_info=(exc_type, exc, tb))

    def thread_hook(args):
        if args.exc_type is SystemExit:
            return
        name = args.thread.name if args.thread else "?"
        logging.critical(
            f"Uncaught exception in thread '{name}' — that thread has died",
            exc_info=(args.exc_type, args.exc_value, args.exc_traceback),
        )

    sys.excepthook = main_hook
    threading.excepthook = thread_hook


def _summarise(collector, path, started):
    elapsed = time.monotonic() - started
    records = collector.records
    errors = sum(r.levelno >= logging.ERROR for r in records)
    warnings = len(records) - errors

    lines = [f"Run finished after {elapsed:.0f}s — {errors} error(s), {warnings} warning(s)."]
    for r in records:
        stamp = datetime.fromtimestamp(r.created).strftime("%H:%M:%S")
        first_line = r.getMessage().splitlines()[0] if r.getMessage() else ""
        lines.append(f"  {stamp} {r.levelname:<8} [{r.threadName}] {first_line}")
    lines.append(f"Full log: {path}")

    bar = "=" * 60
    logging.info("\n".join([bar, *lines, bar]))


def _prune(name):
    try:
        # Match the timestamp too, so "task1" never prunes "task1_pc" logs.
        pattern = re.compile(rf"{re.escape(name)}_\d{{8}}_\d{{6}}\.log")
        runs = sorted(f for f in os.listdir(LOG_DIR) if pattern.fullmatch(f))
        for old in runs[:-KEEP_RUNS]:
            os.remove(os.path.join(LOG_DIR, old))
    except OSError as exc:
        logging.warning(f"Could not prune old logs: {exc}")
