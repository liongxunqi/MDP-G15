"""
communications/android.py
─────────────────────────
Bluetooth RFCOMM SPP server that the Android tablet connects to.

Key behaviours:
  • Binds on channel 1 and advertises the Serial Port Profile so Android
    can find it without knowing the channel number upfront.
  • A background _accept_loop thread keeps listening so that if the tablet
    drops the connection the RPi is immediately ready for a reconnect —
    you never need to restart the script.
  • receive() is non-blocking (returns None on timeout / no data yet) so the
    android_receive loop in task1.py can spin without deadlocking other threads.
  • send() raises on failure so the caller knows the tablet is gone.
"""

import os
import errno
import fcntl
import logging
import sys
from socket import timeout as SocketTimeout
from threading import Event, Thread
from time import sleep
from typing import Optional

from bluetooth import (
    RFCOMM,
    SERIAL_PORT_CLASS,
    SERIAL_PORT_PROFILE,
    BluetoothError,
    BluetoothSocket,
    advertise_service,
)
from dotenv import load_dotenv

load_dotenv()

# ── Process-level lock so only one instance of the script runs at a time ──────
_LOCK_PATH = "/var/run/mdp-task1.lock"
os.makedirs(os.path.dirname(_LOCK_PATH), exist_ok=True)
_lock_fd = open(_LOCK_PATH, "w")
try:
    fcntl.flock(_lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    _lock_fd.write(str(os.getpid()))
    _lock_fd.flush()
except BlockingIOError:
    print("Another instance of this script is already running — exiting.")
    sys.exit(0)


class Android:
    """RFCOMM SPP server with auto-reconnect."""

    def __init__(self):
        self.server_socket: Optional[BluetoothSocket] = None
        self.client_socket: Optional[BluetoothSocket] = None
        self.connected: bool = False

        # Internal receive buffer — handles partial / multi-message packets
        self._rx_buffer: str = ""

        self._stop_event = Event()
        self._accept_thread: Optional[Thread] = None

        # BT config
        self.uuid = "00001101-0000-1000-8000-00805F9B34FB"  # Standard SPP UUID
        self.host = ""          # "" → any local adapter
        self.port = int(os.getenv("BT_PORT", "1"))

        # How long recv() blocks before returning None (keeps CPU free)
        self._recv_timeout_s = 1.0
        self._accept_backoff_s = 0.5

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the BT server and launch the background accept loop."""
        self._start_server()
        if self._accept_thread and self._accept_thread.is_alive():
            return
        self._stop_event.clear()
        self._accept_thread = Thread(target=self._accept_loop, daemon=True, name="bt-accept")
        self._accept_thread.start()
        logging.info("Android: background accept loop started.")

    def stop(self) -> None:
        """Shut everything down cleanly."""
        self._stop_event.set()
        self._close_client()
        self._stop_server()

    # ── Internal server management ────────────────────────────────────────────

    def _start_server(self) -> None:
        if self.server_socket:
            return  # already listening

        try:
            os.system("sudo hciconfig hci0 piscan")  # make adapter discoverable
            self.server_socket = BluetoothSocket(RFCOMM)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            advertise_service(
                self.server_socket,
                "MDPRPi",
                service_id=self.uuid,
                service_classes=[self.uuid, SERIAL_PORT_CLASS],
                profiles=[SERIAL_PORT_PROFILE],
            )
            logging.info(f"Android: SPP server advertised on RFCOMM channel {self.port}.")
        except Exception as exc:
            logging.error(f"Android: failed to start server — {exc}")
            self._stop_server()
            raise

    def _stop_server(self) -> None:
        self._close_client()
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass
            self.server_socket = None

    def _accept_loop(self) -> None:
        """Runs in a daemon thread; re-accepts whenever the tablet disconnects."""
        while not self._stop_event.is_set():
            if self.connected:
                sleep(0.1)
                continue
            try:
                if not self.server_socket:
                    self._start_server()
                logging.info("Android: waiting for tablet to connect…")
                client_sock, info = self.server_socket.accept()
                client_sock.settimeout(self._recv_timeout_s)
                self.client_socket = client_sock
                self.connected = True
                logging.info(f"Android: tablet connected from {info}.")
                # Tell the tablet we're alive
                try:
                    self.send("STATUS,CONNECTED TO RPI")
                except Exception:
                    pass
            except Exception as exc:
                if self._stop_event.is_set():
                    break
                logging.warning(f"Android: accept failed ({exc}) — retrying…")
                sleep(self._accept_backoff_s)

    def _close_client(self) -> None:
        if self.client_socket:
            try:
                self.client_socket.close()
            except Exception:
                pass
        self.client_socket = None
        self.connected = False

    # ── Public API ────────────────────────────────────────────────────────────

    def send(self, message: str) -> None:
        """Send a newline-terminated message to the tablet."""
        if not self.connected or not self.client_socket:
            raise ConnectionError("Android: not connected.")
        payload = (message.rstrip("\n") + "\n").encode("utf-8")
        try:
            self.client_socket.send(payload)
            logging.info(f"Android → tablet: {message.strip()}")
        except (BluetoothError, OSError) as exc:
            logging.warning(f"Android: send failed ({exc}) — marking disconnected.")
            self._close_client()
            raise

    def receive(self) -> Optional[str]:
        """
        Non-blocking receive.
        Returns one complete newline-delimited message, or None if nothing
        is available yet (timeout, no connection, partial packet).
        """
        if not self.connected or not self.client_socket:
            return None

        try:
            # Return a buffered complete line immediately if available
            if "\n" in self._rx_buffer:
                line, self._rx_buffer = self._rx_buffer.split("\n", 1)
                msg = line.strip()
                if msg:
                    logging.info(f"Android ← tablet: {msg}")
                    return msg

            raw = self.client_socket.recv(1024)
            if not raw:
                logging.info("Android: tablet disconnected (0 bytes).")
                self._close_client()
                return None
            self._rx_buffer += raw.decode("utf-8")

            if "\n" in self._rx_buffer:
                line, self._rx_buffer = self._rx_buffer.split("\n", 1)
                msg = line.strip()
                if msg:
                    logging.info(f"Android ← tablet: {msg}")
                    return msg
            return None

        except SocketTimeout:
            return None  # normal — no data this tick

        except BluetoothError as exc:
            if exc.args and "timed out" in str(exc.args[0]).lower():
                return None
            logging.warning(f"Android: BluetoothError in receive ({exc}).")
            self._close_client()
            return None

        except OSError as exc:
            if getattr(exc, "errno", None) in (errno.ECONNRESET, errno.ENOTCONN, errno.EPIPE):
                logging.warning(f"Android: connection reset ({exc}).")
                self._close_client()
                return None
            logging.error(f"Android: unexpected OSError in receive: {exc}")
            self._close_client()
            return None

    def disconnect(self) -> None:
        """Disconnect the current client but keep the server socket open."""
        self._close_client()
        logging.info("Android: client disconnected (server still listening).")
