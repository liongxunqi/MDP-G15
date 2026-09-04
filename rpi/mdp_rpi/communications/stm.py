"""
communications/stm.py
─────────────────────
Serial UART connection to the STM32 microcontroller.

Command format (4 characters, no spaces):
  W050  →  move forward 50 cm
  S050  →  move backward 50 cm
  D100  →  turn right (angle unit)
  A100  →  turn left  (angle unit)

The STM replies with:
  "OK"     →  command completed successfully, ready for next
  "RESEND" →  checksum/error, please resend the last command

receive() is non-blocking — returns None immediately if no bytes waiting.
wait_receive() blocks until bytes arrive (used in the stm_receive loop which
runs in its own thread so blocking is fine there).
"""

import logging
import os
from typing import Optional

import serial
from dotenv import load_dotenv

load_dotenv()

_SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyACM0")
_BAUD_RATE = int(os.getenv("BAUD_RATE", "115200"))


class STM:

    def __init__(self):
        self.serial: Optional[serial.Serial] = None

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Open the serial port. Raises if the port is unavailable."""
        self.serial = serial.Serial(_SERIAL_PORT, _BAUD_RATE, timeout=1)
        logging.info(f"STM: connected on {_SERIAL_PORT} @ {_BAUD_RATE} baud.")

    def disconnect(self) -> None:
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
            logging.info("STM: disconnected.")

    # ── Sending ───────────────────────────────────────────────────────────────

    def send(self, message: str) -> None:
        """Send a raw string to the STM32 (UTF-8 encoded)."""
        if not self.serial:
            logging.error("STM: send called but not connected.")
            return
        self.serial.write(message.encode("utf-8"))
        logging.info(f"STM → stm: {message.strip()}")

    def send_command(self, command: str) -> None:
        """
        Send a validated 4-character STM command.
        Appends a newline so the STM firmware can detect end-of-command.
        """
        if len(command) != 4:
            logging.error(f"STM: invalid command length '{command}' — must be exactly 4 chars.")
            return
        self.send(command + "\n")

    # ── Receiving ─────────────────────────────────────────────────────────────

    def receive(self) -> Optional[str]:
        """
        Non-blocking read.
        Returns whatever bytes are in the input buffer right now, decoded as
        UTF-8.  Returns None if the buffer is empty.
        """
        if not self.serial or not self.serial.is_open:
            return None
        if self.serial.in_waiting > 0:
            raw = self.serial.read_all()
            return raw.decode("utf-8", errors="replace").strip()
        return None

    def wait_receive(self) -> Optional[str]:
        """
        Blocking read — spins until at least one byte arrives.
        Meant to be called from the dedicated stm_thread only.
        Uses readline() so it waits for a full newline-terminated response.
        """
        if not self.serial:
            return None
        while True:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode("utf-8", errors="replace").strip()
                if line:
                    logging.info(f"STM ← stm: {line}")
                    return line
