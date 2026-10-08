"""Offline wait/replace protocol tests; no motors or network connections."""

import json
import sys
import unittest
from pathlib import Path
from threading import Event, Lock, Thread
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_instruction_mission import EndLoop, FakeSTM, START, Task1
from feedback_control import FeedbackControl
from instruction_mission import InstructionMission


def echo(progress):
    return {k: progress[k] for k in ("feedback_id", "segment_index", "instruction_index")}


class FeedbackControlTests(unittest.TestCase):
    def setUp(self):
        self.control = FeedbackControl(True, 1)
        self.progress = {"segment_index": 0, "instruction_index": 1}
        self.pending = self.control.arm(self.progress)

    def test_matching_continue_consumed_once(self):
        payload = echo(self.progress)
        self.assertTrue(self.control.submit("CONTINUE", payload))
        self.assertFalse(self.control.submit("STOP", payload))
        self.assertEqual(self.control.wait(self.pending), ("CONTINUE", payload))
        self.assertFalse(self.control.waiting)
        self.assertFalse(self.control.submit("CONTINUE", payload))

    def test_stale_id_cannot_satisfy_new_wait_even_with_same_indexes(self):
        stale = echo(self.progress)
        self.control.submit("CONTINUE", stale)
        self.control.wait(self.pending)
        new = self.control.arm(self.progress)
        self.assertNotEqual(stale["feedback_id"], self.progress["feedback_id"])
        self.assertFalse(self.control.submit("REPLACE", stale))
        self.control.submit("CONTINUE", echo(self.progress))
        self.assertEqual(self.control.wait(new)[0], "CONTINUE")

    def test_wrong_indexes_are_terminal_for_matching_id(self):
        payload = echo(self.progress)
        payload["instruction_index"] = 2
        self.assertTrue(self.control.submit("CONTINUE", payload))
        self.assertEqual(self.control.wait(self.pending)[0], "INVALID")

    def test_timeout_falls_back_and_late_response_is_stale(self):
        with patch("feedback_control.time.monotonic", return_value=self.pending["deadline"] + 1):
            self.assertFalse(self.control.submit("CONTINUE", echo(self.progress)))
            self.assertEqual(self.control.wait(self.pending), ("TIMEOUT", {}))
        self.assertFalse(self.control.waiting)

    def test_cancel_wakes_waiter_without_clearing_new_wait(self):
        self.control.cancel("Restart")
        new = self.control.arm(self.progress)
        self.assertEqual(self.control.wait(self.pending)[0], "CANCEL")
        self.assertTrue(self.control.waiting)
        self.control.submit("CONTINUE", echo(self.progress))
        self.control.wait(new)

    def test_invalid_timeout_rejected(self):
        for timeout in (0, -1, float("nan"), float("inf")):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                FeedbackControl(True, timeout)


def bare_task(segments=None, mapping=None):
    task = Task1.__new__(Task1)
    task.segments = segments or [["F10", "S"], ["F10", "S"]]
    task.start_pose = dict(START)
    task.odometry_start = dict(START)
    task.segment_obstacles = mapping if mapping is not None else ["7", "8"]
    task.obstacle_order = [v for v in task.segment_obstacles if v is not None]
    task.segments_index = 0
    task.instruction_mission = None
    task._completed_photo_count = 0
    task.feedback_control = FeedbackControl(True, 1)
    task._idx_lock = Lock()
    task.path_ready = Event()
    task.path_ready.set()
    task._resend_counts = {}
    task.max_resends = 3
    task.segment_delay = 0
    task.capture_settle = 0
    task.halted = False
    task.started = True
    task.a5_mode = False
    task.stm = FakeSTM([["100", "0", "0"]] + [["225", "-75", "-26"]] * 20, ["OK"] * 20)
    task.stm.query = Mock(return_value=None)
    task.android = Mock()
    task.pc = Mock()
    task._last_capture_attempts = 0
    task._detect_and_send_image = Mock(return_value=True)
    task._detect_with_distance_retry = Mock(
        side_effect=lambda obstacle_id, mission: task._detect_and_send_image(obstacle_id)
    )
    return task


def run_task(task):
    assert task._send_next_segment()
    with patch("task1.sleep", side_effect=EndLoop):
        try:
            task.stm_receive()
        except EndLoop:
            pass


class TaskFeedbackTests(unittest.TestCase):
    def responder(self, task, first_action="CONTINUE", replacement=None):
        reports = []

        def send(message):
            if not message.startswith("PROGRESS,"):
                return
            progress = json.loads(message.split(",", 1)[1])
            reports.append(progress)
            # No next move can be issued while the decision is pending.
            self.assertTrue(task.feedback_control.waiting)
            self.assertFalse(task.stm.awaiting_reply)
            action = first_action if len(reports) == 1 else "CONTINUE"
            payload = echo(progress)
            if action == "REPLACE":
                payload.update(replacement)
            self.assertTrue(task.feedback_control.submit(action, payload))

        task.pc.send.side_effect = send
        return reports

    def test_continue_runs_original_and_photos(self):
        task = bare_task()
        reports = self.responder(task)
        run_task(task)
        self.assertFalse(task.halted)
        self.assertEqual(len(reports), 4)
        self.assertEqual(task._detect_and_send_image.call_count, 2)
        self.assertEqual(reports[1]["remaining_photo_ids"], ["7", "8"])
        self.assertEqual(reports[2]["remaining_photo_ids"], ["8"])
        task.pc.send.assert_called_with("STITCH,2\n")
        task.android.send.assert_called_with("STATUS,DONE")

    def test_replace_mid_segment_preserves_origin_and_photo_count(self):
        task = bare_task()
        reports = self.responder(task, "REPLACE", {
            "segments": [["R1", "S"], ["S"]], "segment_obstacles": ["7", "8"],
        })
        run_task(task)
        self.assertEqual([e for e in task.stm.events if isinstance(e, tuple)],
                         [("F10",), ("R1",), ("S",), ("S",)])
        self.assertEqual(task.instruction_mission.origin, (100, 0, 0))
        self.assertEqual([p["x_grid"] for p in reports], [2.75] * 4)
        self.assertEqual([p["y_grid"] for p in reports], [3.25] * 4)
        self.assertEqual([p["instruction_index"] for p in reports], [0, 0, 1, 0])
        self.assertEqual(len({p["feedback_id"] for p in reports}), 4)
        task.pc.send.assert_called_with("STITCH,2\n")

    def test_replacement_at_photo_boundary_defers_photo(self):
        task = bare_task([["S"], ["S"]])
        reports = self.responder(task, "REPLACE", {
            "segments": [["F1", "S"], ["S"]], "segment_obstacles": ["7", "8"],
        })
        counts = []
        task._detect_and_send_image.side_effect = lambda oid: counts.append((oid, len(reports)))
        run_task(task)
        self.assertEqual(counts, [("7", 3), ("8", 4)])

    def test_replacement_retains_prior_photo_count_but_not_prior_photo_target(self):
        task = bare_task([["S"], ["S"]])
        reports = []

        def send(message):
            if not message.startswith("PROGRESS,"):
                return
            progress = json.loads(message.split(",", 1)[1])
            reports.append(progress)
            payload = echo(progress)
            action = "CONTINUE"
            if len(reports) == 2:
                self.assertEqual(task._completed_photo_count, 1)
                self.assertEqual(progress["remaining_photo_ids"], ["8"])
                action = "REPLACE"
                payload.update(segments=[["S"]], segment_obstacles=["8"])
            task.feedback_control.submit(action, payload)

        task.pc.send.side_effect = send
        run_task(task)
        self.assertFalse(task.halted)
        self.assertEqual([c.args[0] for c in task._detect_and_send_image.call_args_list], ["7", "8"])
        self.assertEqual(task._completed_photo_count, 2)
        task.pc.send.assert_called_with("STITCH,2\n")

    def test_invalid_replacements_halt_without_sending_any_more_tokens(self):
        for replacement in [
            {"segments": [["RST"]], "segment_obstacles": ["7"]},
            {"segments": [["S"]], "segment_obstacles": ["7"]},
            {"segments": [["S"], ["S"]], "segment_obstacles": ["7", "7"]},
            {"segments": [["S"], ["S"]], "segment_obstacles": ["7", "99"]},
            {"segments": [["S"]]},
            {"segments": [], "segment_obstacles": []},
        ]:
            with self.subTest(replacement=replacement):
                task = bare_task()
                original = task.segments
                self.responder(task, "REPLACE", replacement)
                run_task(task)
                self.assertTrue(task.halted)
                self.assertIs(task.segments, original)
                self.assertEqual([e for e in task.stm.events if isinstance(e, tuple)], [("F10",)])
                task._detect_and_send_image.assert_not_called()

    def test_no_pc_reply_times_out_then_completes_existing_route(self):
        task = bare_task()
        task.feedback_control.timeout = .001
        run_task(task)
        self.assertFalse(task.halted)
        self.assertTrue(task.path_ready.is_set())
        self.assertEqual([e for e in task.stm.events if isinstance(e, tuple)],
                         [("F10",), ("S",), ("F10",), ("S",)])
        self.assertEqual(task._detect_and_send_image.call_count, 2)
        task.android.send.assert_called_with("STATUS,DONE")

    def test_stop_is_not_a_supported_decision(self):
        task = bare_task()
        progress = {"segment_index": 0, "instruction_index": 0}
        task.feedback_control.arm(progress)
        self.assertFalse(task.feedback_control.submit("STOP", echo(progress)))
        self.assertTrue(task.feedback_control.waiting)
        task.feedback_control.cancel("test cleanup")

    def test_send_guard_refuses_movement_while_waiting(self):
        task = bare_task()
        task.feedback_control.arm({"segment_index": 0, "instruction_index": 0})
        self.assertFalse(task._send_next_segment())
        self.assertEqual(task.stm.events, [])

    def test_receive_thread_parses_continue_while_execution_waits(self):
        task = bare_task()
        progress = {"segment_index": 0, "instruction_index": 1}
        pending = task.feedback_control.arm(progress)
        task.pc.receive.side_effect = ["CONTINUE,not-json",
                                      "CONTINUE," + json.dumps(echo(progress)), EndLoop()]
        with self.assertRaises(EndLoop):
            task.pc_receive()
        self.assertEqual(task.feedback_control.wait(pending)[0], "CONTINUE")

    def test_task1_forwards_detector_selection_to_android(self):
        task = bare_task()
        task.image_done = Event()
        task._detection_lock = Lock()
        task._expected_detection_id = "7"
        task._last_detection = None
        task.pc.receive.side_effect = ["OBJECT,7,0.99,bullseye", EndLoop()]
        with self.assertRaises(EndLoop):
            task.pc_receive()
        self.assertTrue(task.image_done.is_set())
        task.android.send.assert_called_once_with("TARGET,7,bullseye")

        task.android.reset_mock()
        task.image_done.clear()
        task._expected_detection_id = "7"
        task.pc.receive.side_effect = ["OBJECT,7,0.91,H", EndLoop()]
        with self.assertRaises(EndLoop):
            task.pc_receive()
        self.assertTrue(task.image_done.is_set())
        task.android.send.assert_called_once_with("TARGET,7,H")

        for class_id in ("dot", "NONE"):
            task.android.reset_mock()
            task.image_done.clear()
            task._expected_detection_id = "7"
            task.pc.receive.side_effect = [
                "OBJECT,7,0.0," + class_id,
                EndLoop(),
            ]
            with self.assertRaises(EndLoop):
                task.pc_receive()
            self.assertTrue(task.image_done.is_set())
            task.android.send.assert_not_called()

    def test_blocking_wait_does_not_hold_index_lock(self):
        task = bare_task()
        task.instruction_mission = InstructionMission(task.segments, task.start_pose)
        progress = {"segment_index": 0, "instruction_index": 0}
        pending = task.feedback_control.arm(progress)
        entered = Event()
        result = []

        def wait():
            entered.set()
            result.append(task._wait_for_feedback(pending, task.instruction_mission, None))

        thread = Thread(target=wait, daemon=True)
        thread.start()
        self.assertTrue(entered.wait(1))
        acquired = task._idx_lock.acquire(timeout=.2)
        if acquired:
            task._idx_lock.release()
        task.feedback_control.submit("CONTINUE", echo(progress))
        thread.join(2)
        self.assertTrue(acquired)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result, ["continue"])


if __name__ == "__main__":
    unittest.main()
