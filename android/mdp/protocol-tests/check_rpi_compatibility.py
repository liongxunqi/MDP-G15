"""Exercise Android compatibility against origin/protocol-v3-fu without checking it out.

No hardware imports, RPi edits or RPi bytecode files. Run with python3 from anywhere.
The batch strings are also asserted in ObstacleSyncTest / ArenaViewModelTest.
"""
import ast
import logging
import subprocess
from pathlib import Path
from threading import Event, Lock
from types import SimpleNamespace
import unittest


class Finished(BaseException):
    pass


ROOT = Path(__file__).resolve().parents[3]
REFERENCE = "origin/protocol-v3-fu"


def reference_method(name):
    source = f"{REFERENCE}:rpi/mdp_rpi/task1.py"
    # Fail if the reference is unavailable; never silently test main instead.
    text = subprocess.check_output(["git", "show", source], cwd=ROOT, text=True)
    tree = ast.parse(text)
    task = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Task1")
    method = next(node for node in task.body if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = {"logging": logging}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
    return namespace[name]


def finite_receive(messages):
    messages = iter(messages)
    def receive():
        try:
            return next(messages)
        except StopIteration:
            raise Finished()
    return receive


class RpiCompatibilityTests(unittest.TestCase):
    def test_complete_batches_replace_map_without_duplicates_or_unit_drift(self):
        state = SimpleNamespace(obstacles=[], obstacle_order=[], path_ready=Event(), path_requested=False,
                                _debounce_lock=Lock(), _calc_timer=None, _schedule_path_calculation=lambda: None)
        handler = reference_method("android_receive")
        batches = [
            ("CLEAR\nOBSTACLE,1,30,40,NORTH\nOBSTACLE,2,190,0,SKIP", [dict(id=1, x=3, y=4, d=0), dict(id=2, x=19, y=0, d=8)]),
            ("CLEAR\nOBSTACLE,2,80,90,WEST", [dict(id=2, x=8, y=9, d=6)]),
            ("CLEAR\nOBSTACLE,2,80,90,WEST", [dict(id=2, x=8, y=9, d=6)]),
            ("CLEAR", []),
        ]
        for batch, expected in batches:
            state.android = SimpleNamespace(receive=finite_receive(batch.splitlines()))
            with self.assertRaises(Finished):
                handler(state)
            self.assertEqual(expected, state.obstacles)

    def test_begin_is_accepted_and_clears_v3_halt_state(self):
        ready = Event()
        ready.set()
        sent = []
        state = SimpleNamespace(android=SimpleNamespace(receive=finite_receive(["BEGIN"])),
                                started=False, halted=True, _idx_lock=Lock(), segments_index=9,
                                _resend_counts={0: 2}, path_ready=ready,
                                _send_next_segment=lambda: sent.append("segment") or True)
        with self.assertRaises(Finished):
            reference_method("android_receive")(state)
        self.assertTrue(state.started)
        self.assertFalse(state.halted)
        self.assertEqual({}, state._resend_counts)
        self.assertEqual(["segment"], sent)

    def test_dpad_and_stop_are_not_implemented_by_v3_android_receiver(self):
        commands = ["f", "r", "tl", "tr", "s", "fl", "fr", "bl", "br"]
        state = SimpleNamespace(android=SimpleNamespace(receive=finite_receive(commands)))
        with self.assertLogs(level="WARNING") as logs, self.assertRaises(Finished):
            reference_method("android_receive")(state)
        self.assertEqual(len(commands), len(logs.output))
        self.assertTrue(all("unrecognised message" in line for line in logs.output))

    def test_v3_emits_expected_robot_pose_only_when_planner_supplies_directions(self):
        for directions, expected in (([dict(x=7, y=2, dir="W")], ["ROBOT,7,2,W", "STATUS,DONE"]),
                                     ([], ["STATUS,DONE"])):
            sent = []
            state = SimpleNamespace(stm=SimpleNamespace(wait_reply=finite_receive(["OK"])),
                                    android=SimpleNamespace(send=sent.append),
                                    pc=SimpleNamespace(send=lambda _: None),
                                    _idx_lock=Lock(), segments_index=1, segments=[["F10"]],
                                    _resend_counts={}, halted=False, directions=directions,
                                    _obstacle_for_segment=lambda _: None,
                                    segment_obstacles=[], obstacle_order=[])
            with self.assertRaises(Finished):
                reference_method("stm_receive")(state)
            self.assertEqual(expected, sent)


if __name__ == "__main__":
    commit = subprocess.check_output(["git", "rev-parse", "--short", REFERENCE], cwd=ROOT, text=True).strip()
    print(f"RPi reference: {REFERENCE} ({commit})", flush=True)
    unittest.main()
