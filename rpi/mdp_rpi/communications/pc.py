"""
communications/pc.py
────────────────────
TCP socket server — the RPi listens and the PC connects to it.

Why RPi is the server:
  The RPi has a fixed IP on the hotspot network so the PC can always find it.
  The PC's IP can change depending on which laptop connects.

Image transfer protocol (send_image):
  1. RPi sends a 4-byte big-endian image size.
  2. RPi sends exactly that many raw JPEG bytes.
"""

import logging
import os
import socket
import struct
import sys
from typing import Optional

from dotenv import load_dotenv

load_dotenv()

_HOST = os.getenv("RPI_HOST", "0.0.0.0")
_PORT = int(os.getenv("RPI_PORT", "5000"))


class PC:
    """TCP server for RPi ↔ PC communication."""

    def __init__(self):
        self.host = _HOST
        self.port = _PORT
        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self._rx_buffer: str = ""  # line-buffer for text messages

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Bind, listen, and block until the PC connects."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server_socket.bind((self.host, self.port))
        except socket.error as exc:
            logging.error(f"PC: bind failed on {self.host}:{self.port} — {exc}")
            self.server_socket.close()
            sys.exit(1)

        logging.info(f"PC: listening on {self.host}:{self.port} — waiting for PC to connect…")
        self.server_socket.listen(1)
        try:
            self.client_socket, addr = self.server_socket.accept()
            logging.info(f"PC: connected from {addr}.")
        except socket.error as exc:
            logging.error(f"PC: accept failed — {exc}")
            self.server_socket.close()
            sys.exit(1)

    def disconnect(self) -> None:
        for sock in (self.client_socket, self.server_socket):
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
        self.client_socket = None
        self.server_socket = None
        logging.info("PC: disconnected.")

    # ── Text messaging ────────────────────────────────────────────────────────

    def send(self, message: str) -> None:
        """Send a UTF-8 text message to the PC (newline-terminated)."""
        if not self.client_socket:
            logging.warning("PC: send called but not connected — skipping.")
            return
        payload = (message.rstrip("\n") + "\n").encode("utf-8")
        try:
            self.client_socket.sendall(payload)
            logging.info(f"PC → pc: {message.strip()}")
        except socket.error as exc:
            logging.error(f"PC: send error — {exc}")
            self.disconnect()

    def receive(self) -> Optional[str]:
        """
        Blocking receive — returns one complete newline-terminated message.
        Blocks until a full line arrives or the connection drops.
        Returns None on connection failure.
        """
        if not self.client_socket:
            return None
        try:
            while "\n" not in self._rx_buffer:
                chunk = self.client_socket.recv(4096)
                if not chunk:
                    logging.info("PC: connection closed by remote.")
                    self.disconnect()
                    return None
                self._rx_buffer += chunk.decode("utf-8")

            line, self._rx_buffer = self._rx_buffer.split("\n", 1)
            msg = line.strip()
            logging.info(f"PC ← pc: {msg}")
            return msg
        except socket.error as exc:
            logging.error(f"PC: receive error — {exc}")
            self.disconnect()
            return None

    # ── Image transfer ────────────────────────────────────────────────────────

    def send_image(self, image_or_path) -> None:
        """
        Transfer a JPEG to the PC using the test_camera length-prefix protocol.

        `image_or_path` may be in-memory JPEG bytes from Camera.capture_image()
        or a file path for older tests.
        """
        if not self.client_socket:
            logging.warning("PC: send_image called but not connected — skipping.")
            return

        try:
            if isinstance(image_or_path, (bytes, bytearray)):
                image_bytes = bytes(image_or_path)
            else:
                with open(os.fspath(image_or_path), "rb") as fh:
                    image_bytes = fh.read()

            self.client_socket.sendall(struct.pack(">I", len(image_bytes)))
            self.client_socket.sendall(image_bytes)
            logging.info(f"PC: image sent successfully ({len(image_bytes)} bytes).")
        except FileNotFoundError:
            logging.error(f"PC: image file not found — {image_or_path}")
        except socket.error as exc:
            logging.error(f"PC: error sending image — {exc}")
            self.disconnect()
