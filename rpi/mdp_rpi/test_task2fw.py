"""task2fw.run_course against a fake STM that speaks protocol 11's course
lines (stm/PROTOCOL.md section 13). No serial port, camera or robot.

    python3 -m unittest test_task2fw -v
"""
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from threading import Event

sys.path.insert(0, str(Path(__file__).resolve().parent))

# The laptop may have neither pyserial nor python-dotenv.
try:
    import dotenv  # noqa: F401
except ImportError:
    dotenv = types.ModuleType("dotenv")
    dotenv.load_dotenv = lambda *a, **k: None
    sys.modules["dotenv"] = dotenv
if "serial" not in sys.modules:
    try:
        import serial  # noqa: F401
    except ImportError:
        serial = types.ModuleType("serial")
        serial.Serial = object
        serial.SerialException = Exception
        sys.modules["serial"] = serial

import task2fw  # noqa: E402


class FakeSTM(task2fw.CourseSTM):
    """Firmware side of one course: script(stm, sent) runs on its own thread
    once !T2GO arrives and feeds lines back through _dispatch, as the reader
    thread would."""

    def __init__(self, script):
        super().__init__()
        self.sent = []
        self.script = script

    def _write_line(self, line):
        line = line.strip()
        self.sent.append(line)
        if line.upper().startswith("!T2GO"):
            threading.Thread(target=self.script, args=(self,), daemon=True).start()

    def line(self, text):
        self._dispatch(text)

    def wait_for(self, token, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(s.upper() == token.upper() for s in self.sent):
                return True
            time.sleep(0.01)
        return False


class RunCourse(unittest.TestCase):
    def test_answers_each_prompt_once_and_finishes(self):
        def script(stm):
            stm.line("D,start,0,0,0,0")
            stm.line("P,1")
            assert stm.wait_for("!T2A11")
            stm.line("D,arrow1,1,89,0,0")
            stm.line("P,2")
            stm.line("P,2")          # the repeat at the stop: no second stage
            assert stm.wait_for("!T2A20")
            stm.line("D,parked,0,24,0,180")
            stm.line("OK")

        calls, phases = [], []
        stm = FakeSTM(script)

        def arrow(n):
            calls.append(n)
            time.sleep(0.05)
            return "right" if n == 1 else "left"

        reply = task2fw.run_course(stm, 170, arrow, Event(), phases.append)
        self.assertEqual(reply, "OK")
        self.assertEqual(stm.sent[0], "!T2GO170")
        self.assertEqual(sorted(calls), [1, 2], "one recognition per obstacle")
        self.assertIn("ARROW,1,RIGHT", phases)
        self.assertIn("BYPASS_1", phases)

    def test_lost_answer_is_resent_on_the_next_prompt(self):
        def script(stm):
            stm.line("P,1")
            assert stm.wait_for("!T2A10")
            stm.line("P,1")          # the STM did not hear it
            deadline = time.monotonic() + 2
            while sum(s == "!T2A10" for s in stm.sent) < 2 and time.monotonic() < deadline:
                time.sleep(0.01)
            stm.line("FAIL,NOARROW")

        stm = FakeSTM(script)
        reply = task2fw.run_course(stm, 170, lambda n: "left", Event())
        self.assertEqual(reply, "FAIL,NOARROW")
        self.assertEqual(stm.sent.count("!T2A10"), 2)

    def test_stop_sends_rst_and_waits_for_the_course_reply(self):
        stop = Event()

        def script(stm):
            stm.line("P,1")
            stop.set()
            assert stm.wait_for("RST")
            stm.line("FAIL,STOP")

        stm = FakeSTM(script)
        reply = task2fw.run_course(stm, 170, lambda n: time.sleep(5) or "left", stop)
        self.assertEqual(reply, "FAIL,STOP")
        self.assertIn("RST", stm.sent)


if __name__ == "__main__":
    unittest.main()
