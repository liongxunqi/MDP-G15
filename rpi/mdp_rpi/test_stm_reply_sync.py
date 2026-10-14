"""Offline regressions for the movement-completion/RST race seen in Task 2."""
import importlib.util
import unittest
from pathlib import Path
from threading import Event, Thread
from unittest.mock import Mock, patch


spec = importlib.util.spec_from_file_location(
    "stm_reply_sync_test", Path(__file__).parent / "communications" / "stm.py")
module = importlib.util.module_from_spec(spec)
with patch.dict("sys.modules", {"serial": Mock()}):
    spec.loader.exec_module(module)


class ResetReplySync(unittest.TestCase):
    def setUp(self):
        self.stm = module.STM()
        self.writes = []
        self.on_ver = lambda: self.stm._dispatch("VER,STM32,4")
        self.stm.serial = Mock()
        self.stm.serial.write.side_effect = self.write

    def write(self, payload):
        line = payload.decode().strip()
        self.writes.append(line)
        if line == "?VER":
            self.on_ver()

    def test_normal_motion_has_no_extra_query_and_keeps_single_line_guard(self):
        self.assertTrue(self.stm.send_line("F33"))
        self.assertFalse(self.stm.send_line("FR45,F4,FL45"))
        self.stm._dispatch("OK")
        self.assertEqual(self.stm.wait_reply(0.01), "OK")
        self.assertTrue(self.stm.send_line("FR45,F4,FL45"))
        self.assertEqual(self.writes, ["F33", "FR45,F4,FL45"])

    def test_queued_ok_before_rst_cannot_complete_lane_change(self):
        self.stm.send_line("F33")
        self.stm._dispatch("OK")
        self.stm.abort()
        self.assertIsNone(self.stm.receive())
        self.assertTrue(self.stm.send_line("FR45,F4,FL45"))
        self.assertEqual(self.writes, ["F33", "RST", "?VER", "FR45,F4,FL45"])
        self.assertIsNone(self.stm.receive())
        self.assertTrue(self.stm.awaiting_reply)
        self.assertFalse(self.stm.send_line("F52"))
        self.stm._dispatch("OK")
        self.assertEqual(self.stm.receive(), "OK")
        self.assertTrue(self.stm.send_line("F52"))

    def test_in_transit_replies_discarded_until_ordered_query_returns(self):
        self.stm.send_line("F33")
        self.stm.abort()

        def replies():
            for reply in ("OK", "RESEND", "FAIL,TIMEOUT"):
                self.stm._dispatch(reply)
            self.stm._dispatch("VER,STM32,4")

        self.on_ver = replies
        self.assertTrue(self.stm.send_line("FR45,F4,FL45"))
        self.assertIsNone(self.stm.receive())
        self.stm._dispatch("OK")
        self.assertEqual(self.stm.wait_reply(0.01), "OK")

    def test_sync_timeout_does_not_send_motion_and_can_retry(self):
        self.stm.abort()
        self.on_ver = lambda: None
        original_query = self.stm.query
        with patch.object(self.stm, "query", side_effect=lambda cmd: original_query(cmd, 0.01)):
            self.assertFalse(self.stm.send_line("FR45"))
        self.stm._dispatch("OK")
        self.assertIsNone(self.stm.receive())
        self.assertEqual(self.writes, ["RST", "?VER", "?VER"])
        self.on_ver = lambda: self.stm._dispatch("VER,STM32,4")
        self.assertTrue(self.stm.send_line("FR45"))
        self.assertEqual(self.writes[-2:], ["?VER", "FR45"])

    def test_rejected_first_post_reset_query_retries_without_replaying_motion(self):
        self.stm.abort()
        replies = iter(["RESEND", "VER,STM32,8"])
        self.on_ver = lambda: self.stm._dispatch(next(replies))
        original_query = self.stm.query
        with patch.object(self.stm, "query",
                          side_effect=lambda cmd: original_query(cmd, 0.01)):
            self.assertTrue(self.stm.send_line("!FAST0"))
        self.assertEqual(self.writes, ["RST", "?VER", "?VER", "!FAST0"])
        self.assertIsNone(self.stm.receive())
        self.stm._dispatch("OK")
        self.assertEqual(self.stm.receive(), "OK")

    def test_new_abort_during_sync_cannot_be_undone_by_old_query(self):
        self.stm.abort()
        query_sent = Event()
        self.on_ver = query_sent.set
        result = []
        thread = Thread(target=lambda: result.append(self.stm.send_line("FR45")))
        thread.start()
        try:
            self.assertTrue(query_sent.wait(1))
            self.stm.abort()
            self.stm._dispatch("VER,STM32,4")
        finally:
            thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result, [False])
        self.assertNotIn("FR45", self.writes)
        self.on_ver = lambda: self.stm._dispatch("VER,STM32,4")
        self.assertTrue(self.stm.send_line("FR45"))
        self.assertEqual(self.writes[-2:], ["?VER", "FR45"])

    def test_config_reply_after_abort_is_not_discarded(self):
        self.stm.abort()
        self.assertTrue(self.stm.send_line("!FAST0"))
        self.stm._dispatch("OK")
        self.assertEqual(self.stm.wait_reply(0.01), "OK")

    def test_real_task2_poll_callback_completion_race(self):
        from task2 import Motion
        from task2t_park import run_polling

        motion = Motion(self.stm, Event(), lambda: None)

        def arrow_confirmed():
            # The F33 OK arrives while the arrow callback reads WPOSE.
            self.stm._dispatch("OK")
            motion.abort()
            return True

        with patch("task2t_park.time.sleep"):
            self.assertEqual(run_polling(motion, "F33", arrow_confirmed), "STOPPED")
        self.assertTrue(motion.send_line("FR45,F4,FL45"))
        self.assertIsNone(motion.receive())
        self.assertFalse(motion.send_line("F52"))
        self.stm._dispatch("OK")
        self.assertEqual(motion.wait_reply(0.1), "OK")
        self.assertTrue(motion.send_line("F52"))


if __name__ == "__main__":
    unittest.main()
