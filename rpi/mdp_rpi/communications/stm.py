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
    FU<n>  forward until the ultrasound reads n cm — n is YOURS to pick, 5..200
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
    FAIL,NOECHO     FU<n> had no usable reading — THE ROBOT DID NOT MOVE (v3)
    <TAG>,<fields>  answer to a query (§5)

  FAIL,* means "the robot is not where you think it is". Stop and re-plan.

  NOECHO is the one worth telling apart: nothing moved, so the pose is still
  whatever it was before the line, and the rest of that line was dropped by the
  firmware rather than run from the wrong place. The fix is the sensor, not the
  odometry.

Queries (§5) bypass the movement queue and are answered immediately, even
mid-move. The reply IS the data — there is no separate OK. Because of that
this class owns a single reader thread: it is the only thing that touches
serial.readline(). Movement replies go to a queue consumed by the caller's
segment pump; query replies are handed back to whichever thread asked.

CAUTION: config commands (§6, !PROF0/1/2 and !ZERO) and the calibration setters
(§7, !CALD/!CALL/!CALT) reply plain "OK", which is indistinguishable from a
movement OK. They are therefore sent down the MOVEMENT path, not the query path,
and must not be issued while a move is outstanding.

Calibration (§7, protocol 2):
    ?CAL            read the decel, brake lag and steering trim learned this
                    power-on — all three are RAM-only and lost at power-off
    !CALD<n>        arc deceleration, dps² ×10
    !CALL<n>        brake lag, milliseconds ×10
    !CALT<n>        steering trim, µs — the ONE signed argument in the protocol

  RESEND from a setter means BUSY, not malformed: the firmware refuses one that
  would land mid-arc because Motion_Tick() is reading those numbers. That is the
  single exception to the §3 rule that RESEND means "your bytes were bad", and
  _send_cal_token() tells the two apart with ?STAT rather than guessing.

Sensor-terminated movement (§4.1, protocol 3):
    FU<n>           drive until the front ultrasound reads n cm

  NOT a trip-wire like F0. The firmware stands still to MEASURE the gap — a
  rolling reading is ~120 ms stale, which is ~4 cm at cruise and a different
  4 cm every run — then drives it on odometry and re-measures, up to four
  passes, each paying a 400 ms stationary settle. Lands within ±2 cm of n.

  Two consequences for this client. A single FU line can take 8.6 s worst
  case, which is why _READ_TIMEOUT is sized off the LINE and not off one move.
  And FU is precise, not accurate: the sensor reads ~1.3 cm long and every
  pass reads through that same offset, so FU20 settles ~18.7 cm out. Correct
  for it here (US_BIAS_CM) rather than in the firmware, where it needs a
  reflash to change.

Every version so far has been a pure SUPERSET of the one before — movement
tokens are byte-for-byte identical all the way back to v1. Older firmware is
therefore a capability question, not a compatibility one:

    v1   everything here except ?CAL / !CAL* (RESEND) and FU (RESEND)
    v2   everything except FU — still reserved there, so the whole line RESENDs
    v3   everything

Check with ?VER at startup and degrade deliberately. Do not treat an older
firmware as an error it is not.
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

# PROTOCOL.md §11: the firmware per-primitive watchdog is 15s. A read timeout
# must sit comfortably above that — 20s — and expiry means "lost link", not
# "slow move". The old value here was 1s, shorter than a single legitimate move.
#
# Size it off the LINE, not off one primitive. FU<n> (§4.1) is several moves
# plus up to four 400 ms settles under a single reply, and the measured worst
# case — FU5 from 2 m — is 8.6 s. A sender that gave up early would abandon a
# line the firmware is still working on, and every reply after that would land
# against the wrong command.
_READ_TIMEOUT = float(os.getenv("STM_READ_TIMEOUT_S", "20.0"))

# PROTOCOL.md §10 stage 2: the firmware prints a banner and run reports on this
# port while no host is connected. The first valid command latches that off.
# Drain whatever is already buffered before we start believing what we read.
_BANNER_DRAIN_S = float(os.getenv("STM_BANNER_DRAIN_S", "1.0"))

# PROTOCOL.md §2
MAX_LINE_BYTES = 128
MAX_PRIMITIVES = 16

# PROTOCOL.md §4.1 — bounds on the FU<n> standoff, cm. Out of range is a PARSE
# FAILURE, not a clamp, so one bad value costs the whole line a RESEND.
#
# The floor is not the sensor's 2 cm limit; it is where the approach still has
# room to correct itself. The ceiling is about trusting the reading rather than
# reaching it — past ~2 m the beam has spread wide enough that the nearest
# thing it hears is often not the thing the planner meant.
FU_MIN_CM = 5
FU_MAX_CM = 200

# What the firmware calls "arrived" (§4.1). Anything tighter would have the
# robot chasing the sensor's own quantisation noise.
FU_TOL_CM = 2

# PROTOCOL.md §4.1 — this HC-SR04 reads about this much LONG, measured against
# a tape over 10-100 cm. FU removes the scatter but not the offset, because
# every pass reads through it, so FU20 settles ~18.7 cm from the obstacle.
# Kept on this side because it is a property of the sensor on the day and
# changing it must not need a reflash. Re-measure before relying on it; see
# stm_tokens.fwd_until(compensate=True) for the correction.
US_BIAS_CM = 1.3

# Minimum firmware protocol version each optional feature needs (§7, §4.1).
# Checked against ?VER so a capability gap is reported as one, rather than
# discovered as a mysterious RESEND halfway through a run.
CAL_MIN_PROTOCOL = 2
FU_MIN_PROTOCOL = 3

# PROTOCOL.md §5 — every tag the firmware can answer a query with.
# CAL is protocol 2. It must appear here twice over: once so validate_token()
# will let "?CAL" be sent at all, and once so classify_reply() routes the
# "CAL,..." answer to the waiting thread instead of discarding it as banner
# noise. Missing from either place and the query fails silently.
QUERY_TAGS = {"US", "IR", "IRR", "POSE", "DIST", "TURN", "STAT", "IMU", "XCHK",
              "VER", "CAL"}

# This client implements protocol 3. Compared against the ?VER reply at startup.
PROTOCOL_VERSION = 3

# FU before F, exactly as the firmware's own parser orders its prefixes: "fu"
# would otherwise be eaten by "f" and FU20 read as F with an argument of
# "U20", which does not parse at all.
_MOVE_RE = re.compile(r"^(FR|FL|FU|RR|RL|F|R)(\d+)$", re.IGNORECASE)
_QUERY_RE = re.compile(r"^\?([A-Z]+)$", re.IGNORECASE)
_CONFIG_RE = re.compile(r"^!(PROF[012]|ZERO)$", re.IGNORECASE)

# PROTOCOL.md §7 — the three calibration setters, protocol 2. !CALT is the ONLY
# token anywhere in this protocol that may carry a negative argument: every
# movement argument is a magnitude with its direction in the opcode, but a
# steering trim has no opcode to carry its sign.
_CAL_SET_RE = re.compile(r"^!CAL([DLT])(-?\d+)$", re.IGNORECASE)

# Accepted ranges, read out of the FIRMWARE rather than the prose in §7 — they
# disagree, and the firmware is the thing that will actually refuse you.
# Motion_SetArcDecel() tests `<=` and `>=`, so BOTH decel endpoints are
# exclusive: §7 says "50-3000 dps²", which reads inclusive and is not.
#
# Out of range is refused, not clamped (§7 rule 3) — a value the firmware's own
# learner would discard is a sender bug, and clamping would hide it behind OK.
CAL_LIMITS = {
    "D": (501, 29999, "dps² ×10", "50 < decel < 3000 dps²"),
    "L": (0, 2500, "ms ×10", "0 ≤ lag ≤ 250 ms"),
    "T": (-80, 80, "µs, signed", "±80 µs"),
}

# PROTOCOL.md §9 — the parser returns a parse failure for these, so a line
# containing one gets RESEND. Matched explicitly so the validator can say *why*
# rather than just "unrecognised".
# FU is NOT in this list any more — it is implemented as of protocol 3 (§4.1)
# and is matched by _MOVE_RE above. Leaving it here would have this client
# refuse to send a token the firmware now understands, which is precisely the
# failure this validator exists to prevent, only inverted.
_RESERVED_RE = re.compile(r"^(FIRO|FILO|FIR|FIL|SR|SL)(\d*)$", re.IGNORECASE)

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
            f"'{tok}' is reserved but not implemented (PROTOCOL.md §9) — "
            "the firmware will RESEND the whole line"
        )

    m = _MOVE_RE.match(tok)
    if m:
        op, digits = m.group(1).upper(), m.group(2)
        value = int(digits)

        # PROTOCOL.md §4.1: FU's argument is a standoff to HOLD, not a distance
        # to travel, and it is the one movement argument with a meaningful
        # floor as well as a ceiling. Tested before the zero rule below so
        # "FU0" is told the real reason rather than a misleading one.
        if op == "FU":
            if not (FU_MIN_CM <= value <= FU_MAX_CM):
                return False, (
                    f"'{tok}': {value} cm is outside {FU_MIN_CM}..{FU_MAX_CM} "
                    "(PROTOCOL.md §4.1). Below the floor the approach has no "
                    "room left to correct itself; above it the beam has spread "
                    "too wide to trust what it hears — send F<n> for the bulk "
                    "of the travel and FU<n> for the last stretch. Out of "
                    "range is a parse failure, not a clamp"
                )
            return True, ""

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

    # PROTOCOL.md §7 — calibration setters, checked before the legacy pattern so
    # a range failure says so rather than being mistaken for something else.
    m = _CAL_SET_RE.match(tok)
    if m:
        kind, digits = m.group(1).upper(), m.group(2)
        value = int(digits)
        lo, hi, unit, prose = CAL_LIMITS[kind]
        if kind != "T" and value < 0:
            return False, (
                f"'{tok}': only !CALT may be negative (PROTOCOL.md §7) — the "
                "firmware parses the other two as unsigned and will RESEND"
            )
        if not (lo <= value <= hi):
            return False, (
                f"'{tok}': {value} is outside {lo}..{hi} ({unit}, i.e. {prose}) — "
                "the firmware refuses out-of-range values rather than clamping "
                "(PROTOCOL.md §7 rule 3)"
            )
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
        PROTOCOL.md §10 stage 2 — the firmware prints a banner and run reports
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
                "off after the first valid command (PROTOCOL.md §10 stage 2)."
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
        Returns None on timeout, which per §11 means a lost link rather than a
        slow move — the firmware watchdog would have fired at 15s.
        """
        effective = timeout if timeout is not None else _READ_TIMEOUT
        try:
            reply = self._movement_replies.get(timeout=effective)
        except Empty:
            logging.error(
                f"STM: no reply within {effective}s — treat as lost link "
                "(PROTOCOL.md §11)."
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

    def is_busy(self, timeout: float = 2.0) -> Optional[bool]:
        """
        The `busy` field of ?STAT: True while the LINE you sent is unfinished.

        The meaning changed in protocol 3. It used to track the motion layer
        alone, which was indistinguishable from the line for ordinary movement
        but wrong for FU<n>: that command spends most of its life deliberately
        stopped, settling between passes, and the old field read 0 in those
        windows. A caller polling this to decide the robot had arrived would
        have carried on mid-approach.

        For watching, not for sequencing — the reply to the line is still the
        only real completion signal. Use wait_reply() for that.

        Returns None if the query timed out or the reply was malformed —
        deliberately tri-state, because "I could not find out" and "it is idle"
        must not collapse into the same answer for the caller below.
        """
        fields = self.query_fields("?STAT", timeout)
        if fields is None or len(fields) < 2:
            return None
        return fields[1].strip() == "1"

    # ── Calibration (PROTOCOL.md §7) ──────────────────────────────────────────

    def read_cal(self, timeout: float = 2.0) -> Optional[Tuple[int, int, int]]:
        """
        ?CAL — everything the firmware has learned this power-on, as
        (decel_dps2_x10, lag_ms_x10, trim_us). Returns None on timeout.

        Safe to call mid-move like any other query: it only reads.

        All three live in RAM and are lost at power-off, so a cold robot spends
        its first three or four arcs converging. Store what this returns against
        the surface, battery and weight it was taken on, and push it back with
        set_cal() after the next power-on to skip the warm-up.
        """
        fields = self.query_fields("?CAL", timeout)
        if fields is None:
            return None
        if len(fields) < 3:
            logging.error(f"STM: malformed CAL reply — expected 3 fields, got {fields}.")
            return None
        try:
            return int(fields[0]), int(fields[1]), int(fields[2])
        except ValueError:
            logging.error(f"STM: non-integer field in CAL reply {fields}.")
            return None

    def _send_cal_token(self, tok: str, retries: int, retry_delay: float) -> bool:
        """
        Send one !CAL* token, retrying while the firmware is busy.

        RESEND from a calibration setter does NOT mean "malformed", and this is
        the one place in the protocol where the §3 advice is the wrong response.
        Motion_Tick() reads the braking model on every tick of a turn, so the
        firmware refuses a setter that would land mid-arc — RESEND there means
        BUSY, and the fix is to wait for idle and send the same bytes again,
        not to give up after three tries and report a bad command.

        The two causes are told apart with ?STAT: a RESEND while idle is a
        genuine out-of-range rejection (§7 rule 3) and retrying cannot help.
        """
        for attempt in range(1, retries + 1):
            if not self.send_line([tok]):
                return False  # local validation already logged why; nothing sent

            reply = self.wait_reply()
            if reply is None:
                logging.error(f"STM: no reply to {tok} — lost link (PROTOCOL.md §11).")
                return False

            if reply.strip().upper() == "OK":
                return True

            busy = self.is_busy()
            if busy is False:
                logging.error(
                    f"STM: {tok} refused with RESEND while idle — the value is out "
                    "of range, not a timing collision. The firmware refuses rather "
                    "than clamping (PROTOCOL.md §7 rule 3); retrying will not help."
                )
                return False

            reason = "busy" if busy else "busy state unknown"
            logging.info(
                f"STM: {tok} refused ({reason}) — attempt {attempt}/{retries}, "
                f"waiting {retry_delay}s for idle."
            )
            time.sleep(retry_delay)

        logging.error(
            f"STM: {tok} still refused after {retries} attempts — the robot never "
            "went idle. Calibration not restored."
        )
        return False

    def set_cal(self, decel_x10: Optional[int] = None,
                lag_ms_x10: Optional[int] = None,
                trim_us: Optional[int] = None,
                retries: int = 3, retry_delay: float = 0.2) -> bool:
        """
        Restore saved calibration (PROTOCOL.md §7). Any argument left None is
        not sent. Returns True only if every value asked for was accepted.

        Each value goes as its OWN line — the firmware carries one argument per
        command, which is why there are three setters rather than one.

        These reply plain "OK", indistinguishable from a movement OK, so like
        set_profile() they travel the MOVEMENT path and MUST NOT be issued while
        a movement line is outstanding. Call this during startup, before the
        segment pump begins.
        """
        wanted = [
            ("D", decel_x10, "!CALD"),
            ("L", lag_ms_x10, "!CALL"),
            ("T", trim_us, "!CALT"),
        ]
        pending = [(k, v, p) for k, v, p in wanted if v is not None]
        if not pending:
            logging.warning("STM: set_cal() called with nothing to set.")
            return False

        all_ok = True
        for kind, value, prefix in pending:
            lo, hi, unit, prose = CAL_LIMITS[kind]
            # Checked here as well as in validate_token so the failure names the
            # VALUE the caller passed, not the token string it got mangled into.
            if not (lo <= int(value) <= hi):
                logging.error(
                    f"STM: {prefix} value {value} is outside {lo}..{hi} "
                    f"({unit}, i.e. {prose}) — not sending (PROTOCOL.md §7)."
                )
                all_ok = False
                continue
            if not self._send_cal_token(f"{prefix}{int(value)}", retries, retry_delay):
                all_ok = False

        return all_ok
