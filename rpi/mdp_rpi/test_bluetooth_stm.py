"""
test_bluetooth_stm.py  —  RPi manual control bridge for MDP
─────────────────────────────────────────────────────────
Listens for button presses from the Android tablet over Bluetooth and
forwards them as STM movement tokens (PROTOCOL.md §4).

Android wire string → STM token mapping
─────────────────────────────────────────
  f   → F20      forward 20 cm
  r   → R20      reverse 20 cm
  tl  → FL90     arc forward-left  90°
  tr  → FR90     arc forward-right 90°
  fl  → FL45     arc forward-left  45°
  fr  → FR45     arc forward-right 45°
  bl  → RL45     arc reverse-left  45°
  br  → RR45     arc reverse-right 45°
  s   → S        stop (brake + recentre steering)

Adjust the distances / angles in MANUAL_MOVE_CM and MANUAL_TURN_DEG to
match what your team decides during calibration — the STM token values
are the only thing that needs changing, the logic stays the same.

Thread model
─────────────
One thread per channel (same pattern as task1.py):
  android_thread  ←→  Android tablet  (Bluetooth RFCOMM)
  stm_thread      ←→  STM32           (serial UART)

stm_thread blocks on wait_reply() so that we never send a second command
before the first one is answered — PROTOCOL.md §2 forbids it and the
firmware silently queues them which throws the reply stream out of step.
"""

import logging
import os
from threading import Event, Thread
from time import sleep

from dotenv import load_dotenv

load_dotenv()

from communications.android import Android
from communications.stm import STM, validate_line

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(threadName)s] %(levelname)s — %(message)s",
    datefmt="%H:%M:%S",
)

# ── Tunable movement distances/angles ─────────────────────────────────────────
# Change these to calibrate; the mapping logic below does not need to move.
MANUAL_MOVE_CM   = int(os.getenv("MANUAL_MOVE_CM",   "20"))   # F / R distance
MANUAL_TURN_DEG  = int(os.getenv("MANUAL_TURN_DEG",  "90"))   # tl / tr arc degrees
MANUAL_DIAG_DEG  = int(os.getenv("MANUAL_DIAG_DEG",  "45"))   # fl / fr / bl / br arc degrees

# ── Android wire string → STM token list ──────────────────────────────────────
# Each entry is a list because send_line() takes a list of tokens (one line).
# If you ever need a compound move on one button (e.g. F10,FR45) just add
# more tokens to the list — the firmware executes them in order on one reply.
def _build_command_map() -> dict:
    return {
        "f":  [f"F{MANUAL_MOVE_CM}"],
        "r":  [f"R{MANUAL_MOVE_CM}"],
        "tl": [f"FL{MANUAL_TURN_DEG}"],
        "tr": [f"FR{MANUAL_TURN_DEG}"],
        "fl": [f"FL{MANUAL_DIAG_DEG}"],
        "fr": [f"FR{MANUAL_DIAG_DEG}"],
        "bl": [f"RL{MANUAL_DIAG_DEG}"],
        "br": [f"RR{MANUAL_DIAG_DEG}"],
        "s":  ["S"],
    }


class ManualControl:
    """Bridges Android button presses to STM movement commands."""

    def __init__(self):
        self.android = Android()
        self.stm = STM()

        self.android_thread: Thread | None = None
        self.stm_thread: Thread | None = None

        # Pending command queue — android_thread writes, stm_thread reads.
        # We use a simple list + Event rather than queue.Queue so the stm_thread
        # can drain it itself and apply the §2 one-outstanding-line rule.
        self._pending: list[list[str]] = []
        self._pending_event = Event()
        self._busy = False          # True while STM is executing a line

        self.command_map = _build_command_map()

    # ── Android receive thread ─────────────────────────────────────────────────

    def android_receive(self) -> None:
        while self.android.client_socket is None:
            sleep(0.1)
        while True:
            try:
                while self.android.client_socket is None:
                    sleep(0.1)
                raw = self.android.client_socket.recv(1024)
                if not raw:
                    continue

                self._rx_buffer = getattr(self, "_rx_buffer", "")
                self._rx_buffer += raw.decode("utf-8")

                if "\n" in self._rx_buffer:
                    line, self._rx_buffer = self._rx_buffer.split("\n", 1)
                else:
                    line, self._rx_buffer = self._rx_buffer, ""

                msg = line.strip()
                if not msg:
                    continue

                logging.info(f"Android ← tablet: {msg}")
                cmd = msg.lower()
                tokens = self.command_map.get(cmd)

                if tokens is None:
                    logging.warning(f"Android: unknown manual command '{msg}' — ignoring.")
                    continue

                ok, reason = validate_line(list(tokens))
                if not ok:
                    logging.error(f"Android: token list {tokens} invalid — {reason}.")
                    continue

                if cmd == "s":
                    self._pending.clear()
                self._pending.append(tokens)
                self._pending_event.set()
                logging.info(f"Android: queued {tokens} for '{cmd}'.")

            except OSError as exc:
                if "timed out" in str(exc):
                    continue #no button has been pressed so dont throw error message
                logging.error(f"Android thread OSError: {exc}")

    # ── STM receive / dispatch thread ──────────────────────────────────────────

    def stm_receive(self) -> None:
        """
        Runs in stm_thread.
        Waits for pending commands from android_thread, sends them one at a
        time, and waits for the STM reply before sending the next — §2 rule.
        """
        while True:
            try:
                # Block until android_thread signals a new command
                self._pending_event.wait()
                self._pending_event.clear()

                while self._pending:
                    tokens = self._pending.pop(0)

                    sent = self.stm.send_line(tokens)
                    if not sent:
                        # §2 in-flight guard fired — should not happen here
                        # because we wait for each reply before sending the next,
                        # but guard defensively.
                        logging.warning(
                            f"STM: send_line({tokens}) rejected — previous line "
                            "still in flight. Dropping this command."
                        )
                        continue

                    logging.info(f"STM ← {tokens}")

                    # Block until STM replies (OK / RESEND / FAIL,*)
                    reply = self.stm.wait_reply()
                    if not reply:
                        logging.warning("STM: no reply received — link may be lost.")
                        continue

                    reply_upper = reply.strip().upper()
                    logging.info(f"STM → '{reply_upper}'")

                    if reply_upper == "OK":
                        try:
                            self.android.send("STATUS,OK")
                        except OSError as exc:
                            logging.warning(f"Could not notify Android of OK: {exc}")

                    elif reply_upper == "RESEND":
                        # Parse failure — nothing moved, safe to retransmit once.
                        logging.warning(f"STM: RESEND for {tokens} — retransmitting once.")
                        resent = self.stm.send_line(tokens)
                        if resent:
                            retry_reply = self.stm.wait_reply()
                            if retry_reply and retry_reply.strip().upper() == "OK":
                                logging.info("STM: RESEND retry OK.")
                                try:
                                    self.android.send("STATUS,OK")
                                except OSError:
                                    pass
                            else:
                                logging.error(
                                    f"STM: RESEND retry failed — reply: {retry_reply}. "
                                    "Check token format."
                                )
                                try:
                                    self.android.send("STATUS,FAILED")
                                except OSError:
                                    pass

                    elif reply_upper.startswith("FAIL"):
                        detail = reply.strip().split(",", 1)[1] if "," in reply else "UNKNOWN"
                        logging.error(f"STM: FAIL,{detail} — move did not complete.")
                        try:
                            self.android.send(f"STATUS,FAILED")
                        except OSError as exc:
                            logging.warning(f"Could not notify Android of FAIL: {exc}")

                    else:
                        logging.warning(f"STM: unexpected reply '{reply}' — ignoring.")

            except OSError as exc:
                logging.error(f"STM thread OSError: {exc}")

    # ── Entry point ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Connect peripherals, start threads, and block until they exit."""
        logging.info("=" * 60)
        logging.info("Manual Control — starting up")
        logging.info("=" * 60)
        logging.info(
            f"Move distances: forward/reverse={MANUAL_MOVE_CM}cm  "
            f"turn={MANUAL_TURN_DEG}°  diagonal={MANUAL_DIAG_DEG}°"
        )

        logging.info("Connecting to STM32…")
        self.stm.connect()

        logging.info("Starting Bluetooth server (waiting for Android)…")
        self.android.start()

        logging.info("All connections established — launching threads.")

        self.android_thread = Thread(
            target=self.android_receive, name="android-thread", daemon=True
        )
        self.stm_thread = Thread(
            target=self.stm_receive, name="stm-thread", daemon=True
        )

        self.android_thread.start()
        self.stm_thread.start()

        logging.info("Manual control ready. Waiting for button presses…")

        try:
            self.android_thread.join()
            self.stm_thread.join()
        except KeyboardInterrupt:
            logging.info("Interrupted — shutting down.")
        finally:
            self.stm.disconnect()
            self.android.stop()
            logging.info("Manual control shut down cleanly.")


# ── Run ────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    mc = ManualControl()
    mc.start()