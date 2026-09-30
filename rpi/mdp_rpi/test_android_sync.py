"""
test_android_sync.py — the Android state-snapshot / SYNC mechanism in task1.py.

task1.py imports `bluetooth` (PyBluez, Linux-only) and `fcntl` (POSIX-only) at
module level, so it cannot be imported on every dev machine. This extracts just
the methods under test from the file's own AST and execs them as free
functions against a lightweight stand-in for `self` — the same technique
protocol-tests/check_rpi_compatibility.py uses against a frozen git ref, here
used against the live file instead, since these are our own methods, not a
historical reference to stay compatible with.

Run with: python3 test_android_sync.py
"""
import ast
import logging
import threading
import unittest
from pathlib import Path
from threading import Lock
from types import SimpleNamespace

TASK1_PATH = Path(__file__).resolve().parent / "task1.py"


def _load_methods(*names: str) -> dict:
    """Extract named methods from Task1 in task1.py, exec'd as free functions taking `self`."""
    tree = ast.parse(TASK1_PATH.read_text(encoding="utf-8"))
    task = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Task1")
    wanted = set(names)
    found = {n.name: n for n in task.body if isinstance(n, ast.FunctionDef) and n.name in wanted}
    missing = wanted - found.keys()
    assert not missing, f"Task1 in task1.py has no method(s): {missing}"
    namespace = {"logging": logging}
    module = ast.Module(body=list(found.values()), type_ignores=[])
    exec(compile(module, str(TASK1_PATH), "exec"), namespace)
    return {name: namespace[name] for name in names}


class FakeAndroid:
    """Records every line that actually reached the (fake) socket; can fail on demand."""

    def __init__(self):
        self.sent: list = []
        self.fail_next = False

    def send(self, line: str) -> None:
        if self.fail_next:
            self.fail_next = False
            raise OSError("simulated Bluetooth link drop")
        self.sent.append(line)


class AndroidSyncTests(unittest.TestCase):
    def setUp(self):
        methods = _load_methods("_send_to_android", "_reset_android_snapshot", "_sync_android")
        self.send = methods["_send_to_android"]
        self.reset = methods["_reset_android_snapshot"]
        self.sync = methods["_sync_android"]

    def make_self(self, android: FakeAndroid | None = None) -> SimpleNamespace:
        """
        A stand-in for `self` carrying only the attributes these methods touch.
        _sync_android's body calls `self._send_to_android(...)`, so the extracted
        function is bound onto the namespace under that name too.
        """
        state = SimpleNamespace(
            android=android or FakeAndroid(),
            _snapshot_lock=Lock(),
            _target_snapshot={},
            _robot_snapshot=None,
            _status_snapshot=None,
        )
        state._send_to_android = lambda line: self.send(state, line)
        return state

    def test_target_sent_while_disconnected_is_recorded_and_resynced(self):
        """State must be recorded even though the write itself fails."""
        state = self.make_self()
        state.android.fail_next = True
        ok = self.send(state, "TARGET,3,circle")
        self.assertFalse(ok)
        self.assertEqual({"3": "TARGET,3,circle"}, state._target_snapshot)

        state.android.sent.clear()
        self.sync(state)
        self.assertEqual(["TARGET,3,circle"], state.android.sent)

    def test_newer_target_for_same_obstacle_replaces_the_old_one(self):
        state = self.make_self()
        self.send(state, "TARGET,3,bullseye")
        self.send(state, "TARGET,3,circle")
        self.assertEqual({"3": "TARGET,3,circle"}, state._target_snapshot)

        state.android.sent.clear()
        self.sync(state)
        self.assertEqual(["TARGET,3,circle"], state.android.sent)

    def test_only_latest_robot_is_kept_and_sent(self):
        state = self.make_self()
        self.send(state, "ROBOT,1,1,N")
        self.send(state, "ROBOT,2,1,N")
        self.assertEqual("ROBOT,2,1,N", state._robot_snapshot)

        state.android.sent.clear()
        self.sync(state)
        self.assertEqual(["ROBOT,2,1,N"], state.android.sent)

    def test_reset_clears_everything(self):
        state = self.make_self()
        self.send(state, "TARGET,3,circle")
        self.send(state, "ROBOT,1,1,N")
        self.send(state, "STATUS,RUNNING,1,4")

        self.reset(state)
        self.assertEqual({}, state._target_snapshot)
        self.assertIsNone(state._robot_snapshot)
        self.assertIsNone(state._status_snapshot)

        state.android.sent.clear()
        self.sync(state)
        self.assertEqual([], state.android.sent)

    def test_sync_sends_all_targets_then_robot_then_status(self):
        state = self.make_self()
        self.send(state, "TARGET,1,circle")
        self.send(state, "TARGET,2,square")
        self.send(state, "ROBOT,4,4,N")
        self.send(state, "STATUS,RUNNING,2,5")

        state.android.sent.clear()
        self.sync(state)
        self.assertEqual(
            ["TARGET,1,circle", "TARGET,2,square", "ROBOT,4,4,N", "STATUS,RUNNING,2,5"],
            state.android.sent,
        )

    def test_a_failed_sync_send_does_not_stop_the_rest(self):
        """One dead write mid-SYNC must not swallow the robot/status that follow it."""
        state = self.make_self()
        self.send(state, "TARGET,1,circle")
        self.send(state, "ROBOT,4,4,N")
        state.android.sent.clear()
        state.android.fail_next = True  # kills only the first resend (the TARGET)

        self.sync(state)
        self.assertEqual(["ROBOT,4,4,N"], state.android.sent)
        # The snapshot itself is untouched by the failed resend.
        self.assertEqual({"1": "TARGET,1,circle"}, state._target_snapshot)

    def test_concurrent_sends_from_multiple_threads_never_corrupt_a_line(self):
        state = self.make_self()
        errors: list = []

        def worker(i: int) -> None:
            try:
                for j in range(200):
                    self.send(state, f"TARGET,{i},class{j}")
            except Exception as exc:  # would otherwise fail silently in a thread
                errors.append(exc)

        threads = [threading.Thread(target=worker, args=(i,)) for i in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.assertEqual([], errors)
        # Every line actually written must be whole — never a torn fragment of
        # two different threads' sends interleaved on the fake socket.
        for line in state.android.sent:
            self.assertRegex(line, r"^TARGET,\d+,class\d+$")
        # One (the latest) snapshot entry survives per obstacle id.
        self.assertEqual(8, len(state._target_snapshot))
        for i in range(8):
            self.assertRegex(state._target_snapshot[str(i)], rf"^TARGET,{i},class\d+$")


if __name__ == "__main__":
    unittest.main()
