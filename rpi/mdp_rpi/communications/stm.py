"""
communications/stm.py
─────────────────────
Serial UART link to the STM32 motor controller.

Wire format (PROTOCOL.md §2 — that document is authoritative):

    RPi → STM :  <tok1>,<tok2>,...,<tokN>\\n
    STM → RPi :  <one reply line>\\n

  * Tokens are comma-separated, the line is newline-terminated.
  * EXACTLY ONE reply per LINE, never per token.
  * Parsing is all-or-nothing: one bad token and nothing on that line runs.
  * Max 128 bytes per line, max 16 primitives queued.

  ** Do not send the next line until the previous one has been answered. **
  A line arriving mid-execution is appended to the same queue and you get one
  reply for two lines — permanently one reply out of step, silently.

Movement tokens (PROTOCOL.md §4):
    F<n>   forward n cm          F0  forward until obstacle (ultrasonic, 15cm)
    R<n>   reverse n cm
    FR<n> / FL<n>   arc forward-right / forward-left, n degrees
    RR<n> / RL<n>   arc reverse-right / reverse-left, n degrees
    S      stop (brake + recentre steering) — replies OK
    RST    emergency abort — replies NOTHING AT ALL, never wait for it

  NOTE: S means STOP, not backward. Backward is R<n>. An earlier draft of this
  client used S050 for reverse; that token is rejected by the firmware.

Replies (PROTOCOL.md §3):
    OK              whole line completed
    RESEND          parse failure, nothing executed, safe to retransmit
    FAIL,TIMEOUT    watchdog fired mid-move — wheel stalled or encoder dead
    FAIL,WRONGWAY   arc rotated away from target and was aborted
    <TAG>,<fields>  answer to a query (§5)

  FAIL,* means "the robot is not where you think it is". Stop and re-plan.

Queries (§5) bypass the movement queue and are answered immediately, even
mid-move. The reply IS the data — there is no separate OK. Because of that
this class owns a single reader thread: it is the only thing that touches
serial.readline(). Movement replies go to a queue consumed by the caller's
segment pump; query replies are handed back to whichever thread asked.

CAUTION: config commands (§6, !PROF0/1/2 and !ZERO) reply plain "OK", which is
indistinguishable from a movement OK. They are therefore sent down the MOVEMENT
path, not the query path, and must not be issued while a move is outstanding.
"""

import logging
import os
import re
import threading
import time
from queue import Empty, Queue
from typing import List, Optional, Tuple

import serial
from dotenv import load_dotenv

load_dotenv()

_SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyACM0")
_BAUD_RATE = int(os.getenv("BAUD_RATE", "115200"))

# PROTOCOL.md §10: the firmware per-primitive watchdog is 15s. A read timeout
# must sit comfortably above that — 20s — and expiry means "lost link", not
# "slow move". The old value here was 1s, shorter than a single legitimate move.
_READ_TIMEOUT = float(os.getenv("STM_READ_TIMEOUT_S", "20.0"))

# PROTOCOL.md §9 stage 2: the firmware prints a banner and run reports on this
# port while no host is connected. The first valid command latches that off.
# Drain whatever is already buffered before we start believing what we read.
_BANNER_DRAIN_S = float(os.getenv("STM_BANNER_DRAIN_S", "1.0"))

# PROTOCOL.md §2
MAX_LINE_BYTES = 128
MAX_PRIMITIVES = 16

# PROTOCOL.md §5 — every tag the firmware can answer a query with.
QUERY_TAGS = {"US", "IR", "IRR", "POSE", "DIST", "TURN", "STAT", "IMU", "XCHK", "VER"}

_MOVE_RE = re.compile(r"^(FR|FL|RR|RL|F|R)(\d+)$", re.IGNORECASE)
_QUERY_RE = re.compile(r"^\?([A-Z]+)$", re.IGNORECASE)
_CONFIG_RE = re.compile(r"^!(PROF[012]|ZERO)$", re.IGNORECASE)

# PROTOCOL.md §8 — the parser returns a parse failure for these, so a line
# containing one gets RESEND. Matched explicitly so the validator can say *why*
# rather than just "unrecognised".
_RESERVED_RE = re.compile(r"^(FU|FIRO|FILO|FIR|FIL|SR|SL)(\d*)$", re.IGNORECASE)

# The superseded 4-character dialect (W050/S050/D100/A100). Worth naming
# explicitly: S050 used to mean reverse, and S now means STOP.
_LEGACY_RE = re.compile(r"^([WSDA])(\d{3})$", re.IGNORECASE)


# ── Token / line validation ───────────────────────────────────────────────────
# Catching a malformed token here costs nothing. Catching it on the wire costs a
# full RESEND round trip, and if the sender retransmits the same bad bytes it is
# an infinite loop in which the robot never moves.

def validate_token(token: str) -> Tuple[bool, str]:
    """Return (ok, reason). reason is '' when ok."""
    tok = token.strip()
    if not tok:
        return False, "empty token"

    upper = tok.upper()
    if upper in ("S", "RST"):
        return True, ""

    # Reserved is checked before the movement regex: SR/SL would otherwise be
    # rejected with a confusing message, and FIR/FIL look nothing like F<n>.
    if _RESERVED_RE.match(tok):
        return False, (
            f"'{tok}' is reserved but not implemented (PROTOCOL.md §8) — "
            "the firmware will RESEND the whole line"
        )

    m = _MOVE_RE.match(tok)
    if m:
        op, digits = m.group(1).upper(), m.group(2)
        value = int(digits)
        # PROTOCOL.md §4: a zero argument is valid ONLY for F, where it means
        # "forward until obstacle". R0 / FR0 / RL0 are parse failures.
        if value == 0 and op != "F":
            return False, f"zero argument is only valid for F (got '{tok}')"
        return True, ""

    if _QUERY_RE.match(tok):
        tag = upper.lstrip("?")
        if tag not in QUERY_TAGS:
            return False, f"unknown query '?{tag}' — known: {sorted(QUERY_TAGS)}"
        return True, ""

    if _CONFIG_RE.match(tok):
        return True, ""

    if _LEGACY_RE.match(tok):
        return False, (
            f"'{tok}' is the superseded 4-character protocol (W050/S050/D100/A100) — "
            "the firmware does not understand it. Use F<n>/R<n>/FR<n>/FL<n>/RR<n>/RL<n>/S"
        )

    return False, f"unrecognised token '{tok}'"


def validate_line(tokens: List[str]) -> Tuple[bool, str]:
    """Validate a whole line: every token, plus the §2 length and count caps."""
    if not tokens:
        return False, "empty line"

    if len(tokens) > MAX_PRIMITIVES:
        return False, (
            f"{len(tokens)} primitives exceeds the firmware queue cap of "
            f"{MAX_PRIMITIVES} (PROTOCOL.md §2) — split this segment"
        )

    for tok in tokens:
        ok, reason = validate_token(tok)
        if not ok:
            return False, reason

    # PROTOCOL.md §5: queries are single-token lines only. A query mixed into a
    # movement line is a parse failure.
    if len(tokens) > 1:
        for tok in tokens:
            if tok.strip().startswith("?"):
                return False, (
                    f"query '{tok.strip()}' must be sent on its own line "
                    "(PROTOCOL.md §5)"
                )

    encoded = len((",".join(t.strip() for t in tokens) + "\n").encode("utf-8"))
    if encoded > MAX_LINE_BYTES:
        return False, (
            f"line is {encoded} bytes, over the {MAX_LINE_BYTES}-byte cap "
            "(PROTOCOL.md §2) — split this segment"
        )

    return True, ""


def classify_reply(line: str) -> str:
    """'movement' | 'query' | 'other' — see PROTOCOL.md §3."""
    stripped = line.strip()
    if not stripped:
        return "other"
    upper = stripped.upper()
    if upper in ("OK", "RESEND"):
        return "movement"
    tag = upper.split(",", 1)[0].strip()
    if tag == "FAIL":
        return "movement"
    if tag in QUERY_TAGS:
        return "query"
    return "other"


class STM:

    def __init__(self):
        self.serial: Optional[serial.Serial] = None

        # Replies that answer a sent LINE: OK / RESEND / FAIL,*
        self._movement_replies: "Queue[str]" = Queue()

        # One outstanding query at a time: (expected_tag, mailbox).
        self._query_lock = threading.Lock()
        self._pending_query: Optional[Tuple[str, "Queue[str]"]] = None

        self._reader_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        # Serialises writes so two threads cannot interleave bytes on the wire.
        self._write_lock = threading.Lock()

        # PROTOCOL.md §2, the rule that matters most: exactly one movement line
        # may be outstanding at a time. A second line arriving while the first
        # is still executing is appended to the same firmware queue, and when
        # that queue drains you get ONE reply for TWO lines — the sender is then
        # permanently one reply out of step, silently.
        #
        # This is enforced here rather than in the caller because several
        # threads can reach send_line(): the Android thread on BEGIN, the PC
        # thread when PATH arrives, and the STM thread on OK/RESEND. A guard in
        # any one of them would not cover the others.
        #
        # Queries are deliberately exempt — they bypass the firmware queue (§5).
        self._inflight = False
        self._inflight_lock = threading.Lock()

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Open the port, drain the startup banner, start the reader thread."""
        self.serial = serial.Serial(_SERIAL_PORT, _BAUD_RATE, timeout=_READ_TIMEOUT)
        logging.info(
            f"STM: connected on {_SERIAL_PORT} @ {_BAUD_RATE} baud "
            f"(read timeout {_READ_TIMEOUT}s)."
        )
        self._drain_banner()

        self._stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="stm-reader", daemon=True
        )
        self._reader_thread.start()

    def _drain_banner(self) -> None:
        """
        PROTOCOL.md §9 stage 2 — the firmware prints a banner and run reports
        until the first valid command latches them off. Anything already in the
        buffer at connect time is not a reply to us, so discard it rather than
        let the first wait_reply() mistake it for one.
        """
        if not self.serial:
            return
        deadline = time.monotonic() + _BANNER_DRAIN_S
        discarded: List[str] = []
        while time.monotonic() < deadline:
            if self.serial.in_waiting > 0:
                raw = self.serial.read(self.serial.in_waiting)
                text = raw.decode("utf-8", errors="replace")
                discarded.extend(t for t in text.splitlines() if t.strip())
            else:
                time.sleep(0.05)
        self.serial.reset_input_buffer()
        if discarded:
            logging.info(f"STM: discarded {len(discarded)} startup banner line(s):")
            for line in discarded[:10]:
                logging.info(f"STM   banner| {line.strip()}")
        else:
            logging.info(
                "STM: no startup banner seen. If the link looks dead later, note "
                "that the banner only prints while no host is connected and latches "
                "off after the first valid command (PROTOCOL.md §9 stage 2)."
            )

    def disconnect(self) -> None:
        self._stop.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        self._reader_thread = None
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
            logging.info("STM: disconnected.")

    # ── Reader thread — the ONLY caller of serial.readline() ──────────────────

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                if not self.serial or not self.serial.is_open:
                    break
                raw = self.serial.readline()
            except (serial.SerialException, OSError) as exc:
                if not self._stop.is_set():
                    logging.error(f"STM reader: serial error — {exc}")
                break

            if not raw:
                continue  # read timeout, nothing arrived

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            logging.info(f"STM ← stm: {line}")
            self._dispatch(line)

    def _dispatch(self, line: str) -> None:
        kind = classify_reply(line)

        if kind == "movement":
            self._movement_replies.put(line)
            return

        if kind == "query":
            tag = line.split(",", 1)[0].strip().upper()
            pending = self._pending_query
            if pending and pending[0] == tag:
                pending[1].put(line)
            else:
                logging.warning(
                    f"STM: unsolicited query reply '{line}' (nothing was waiting "
                    f"for tag {tag}) — discarding."
                )
            return

        logging.info(f"STM: unclassified line '{line}' — treating as banner/debug output.")

    # ── Sending ───────────────────────────────────────────────────────────────

    def _write_line(self, line: str) -> None:
        if not self.serial:
            logging.error("STM: write attempted but not connected.")
            return
        payload = line if line.endswith("\n") else line + "\n"
        with self._write_lock:
            self.serial.write(payload.encode("utf-8"))
        logging.info(f"STM → stm: {payload.strip()}")

    def send(self, message: str) -> None:
        """Raw escape hatch — sends bytes verbatim with no validation."""
        self._write_line(message)

    def send_command(self, command: str) -> bool:
        """
        Send a single validated token as its own line.
        Returns False (and sends nothing) if the token is malformed.
        """
        return self.send_line([command])

    def send_line(self, tokens) -> bool:
        """
        Send one validated line of movement tokens.

        `tokens` may be a list (["FR90", "F20", "S"]) or a pre-joined string
        ("FR90,F20,S"). Returns False and sends NOTHING if validation fails — a
        malformed line would only earn a RESEND anyway.

        Does NOT wait for the reply. Callers must wait_reply() before sending
        the next line (PROTOCOL.md §2).
        """
        if isinstance(tokens, str):
            tokens = [t for t in tokens.strip().split(",") if t.strip()]

        ok, reason = validate_line(list(tokens))
        if not ok:
            logging.error(f"STM: refusing to send invalid line {tokens} — {reason}")
            return False

        with self._inflight_lock:
            if self._inflight:
                # PROTOCOL.md §2 — see the note in __init__. Dropping the line is
                # always better than desyncing the reply stream, because a desync
                # is silent and unrecoverable while a dropped line is loud.
                logging.error(
                    f"STM: refusing to send {tokens} — a line is still awaiting its "
                    "reply. Sending now would merge both lines into one firmware "
                    "queue and return a single OK for two lines (PROTOCOL.md §2)."
                )
                return False
            self._write_line(",".join(t.strip() for t in tokens))
            self._inflight = True
        return True

    def abort(self) -> None:
        """
        Emergency abort (PROTOCOL.md §4). RST drops the firmware queue and brakes
        immediately, and is the ONE command that replies nothing at all — so this
        never waits for a reply.
        """
        self._write_line("RST")
        # RST drops the firmware queue, so the outstanding line will never be
        # answered. Release the guard or nothing could ever be sent again.
        self._clear_inflight()
        logging.warning("STM: RST sent (emergency abort) — no reply expected.")

    # ── Receiving ─────────────────────────────────────────────────────────────

    def wait_reply(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Block until the STM answers the line we sent: OK / RESEND / FAIL,*.
        Returns None on timeout, which per §10 means a lost link rather than a
        slow move — the firmware watchdog would have fired at 15s.
        """
        effective = timeout if timeout is not None else _READ_TIMEOUT
        try:
            reply = self._movement_replies.get(timeout=effective)
        except Empty:
            logging.error(
                f"STM: no reply within {effective}s — treat as lost link "
                "(PROTOCOL.md §10)."
            )
            # Released on timeout too: the line is never going to be answered,
            # and holding the guard would block recovery for good.
            self._clear_inflight()
            return None
        self._clear_inflight()
        return reply

    def _clear_inflight(self) -> None:
        with self._inflight_lock:
            self._inflight = False

    # Back-compat alias: the segment pump used to call wait_receive().
    def wait_receive(self, timeout: Optional[float] = None) -> Optional[str]:
        return self.wait_reply(timeout)

    def receive(self) -> Optional[str]:
        """Non-blocking pop of a movement reply, or None if none pending."""
        try:
            reply = self._movement_replies.get_nowait()
        except Empty:
            return None
        self._clear_inflight()
        return reply

    # ── Queries (PROTOCOL.md §5) ──────────────────────────────────────────────

    def query(self, command: str, timeout: float = 2.0) -> Optional[str]:
        """
        Send a query and return its reply line, e.g. query("?US") -> "US,42".

        Safe to call from any thread at any time, including mid-move: queries
        bypass the firmware's movement queue and never disturb it. The reply IS
        the data — there is no separate OK, so this does not consume one.

        Returns None on timeout or if the token is malformed.
        """
        tok = command.strip()
        if not tok.startswith("?"):
            tok = "?" + tok
        ok, reason = validate_token(tok)
        if not ok:
            logging.error(f"STM: invalid query '{command}' — {reason}")
            return None

        tag = tok.upper().lstrip("?")
        mailbox: "Queue[str]" = Queue(maxsize=1)

        with self._query_lock:
            self._pending_query = (tag, mailbox)
            try:
                self._write_line(tok)
                return mailbox.get(timeout=timeout)
            except Empty:
                logging.warning(f"STM: query '{tok}' timed out after {timeout}s.")
                return None
            finally:
                self._pending_query = None

    def query_fields(self, command: str, timeout: float = 2.0) -> Optional[List[str]]:
        """query() split into fields with the tag dropped. '?IR' -> ['12', '15']."""
        reply = self.query(command, timeout)
        if reply is None:
            return None
        return [f.strip() for f in reply.split(",")[1:]]

    # ── Config (PROTOCOL.md §6) ───────────────────────────────────────────────

    def set_profile(self, profile: int, timeout: Optional[float] = None) -> bool:
        """
        Select the arc profile: 0 TIGHT (r=291mm), 1 CLEAN (r=318mm),
        2 SLOW (r=306mm). Persists until changed or reset; default is TIGHT.

        Replies plain OK, which is indistinguishable from a movement OK — so
        this MUST NOT be called while a movement line is outstanding.
        """
        if profile not in (0, 1, 2):
            logging.error(f"STM: invalid profile {profile} — must be 0, 1 or 2.")
            return False
        if not self.send_line([f"!PROF{profile}"]):
            return False
        reply = self.wait_reply(timeout)
        return reply is not None and reply.strip().upper() == "OK"

    def zero_odometry(self, timeout: Optional[float] = None) -> bool:
        """Zero odometry and heading (!ZERO). Same OK-collision caveat as above."""
        if not self.send_line(["!ZERO"]):
            return False
        reply = self.wait_reply(timeout)
        return reply is not None and reply.strip().upper() == "OK"
