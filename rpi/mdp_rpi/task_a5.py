"""
task_a5.py  —  MDP checklist A.5: navigate around the obstacle
───────────────────────────────────────────────────────────────
    "Demonstrate that your robot can navigate towards a given obstacle having
     a visual marker indicating obstacle. Your robot needs to navigate around
     the obstacle in search of face which has a valid image from the image
     list."

    PC:      cd pc_side && python3 task1_pc.py     ← start first
    RPi:     python3 task_a5.py
    Android: place ONE obstacle, Send Data, Begin  ← exactly as Task 1

A.5 IS TASK 1 WITH ONE THING REMOVED
─────────────────────────────────────
You do not know which face the image is on. That is the only difference, and
it is the thing Task 1 is built around knowing: the operator taps the face in
on Android, it arrives as `d`, and path_planner._viewing_pose() turns it into
the pose to photograph from.

So A.5 reduces to Task 1 exactly. Declare the obstacle FOUR times, once per
face, and one unknown-face problem becomes four known-face problems. The
planner does the rest — four viewing poses 30 cm off each face, a shortest
tour between them, a `FU30` to measure the final standoff, and one DETECT at
each. The orbit is planned and collision-checked rather than dead-reckoned.

This file is a thin entry point. It runs the Task 1 orchestrator with A.5 mode
on; all three of the changes that mode makes live in task1.py, guarded, so
Task 1 itself behaves exactly as it did:

    1. FAN OUT  — one obstacle becomes four on the way to the PC, ids
                  base*10 + d. Android is never told; it knows one obstacle
                  and must keep knowing one, or ArenaReducer rejects the
                  TARGET as referencing an obstacle that does not exist.
    2. FOLD BACK— the winning TARGET is reported under the id Android knows.
    3. FILTER   — three of the four replies are bullseyes or blanks. That is
                  the search working, not a fault, and forwarding them would
                  overwrite the real answer on the tablet, because every
                  fanned id folds back to the same obstacle and
                  ArenaReducer.applyTarget() is last-write-wins.

WHAT THIS NEEDS THAT THE OLD VERSION DID NOT
─────────────────────────────────────────────
Bluetooth paired and an obstacle placed on the tablet. The standalone reactive
version is still here as task_a5_reactive.py: it needs only the Pi, the STM
and a laptop, and it is the one to reach for when Bluetooth will not pair on
the day. It dead-reckons its orbit, which is why this is the primary.

WHAT IT DOES NOT DO
───────────────────
Stop early. The path is planned in full before the first photo, so all four
faces get visited even when the image is on the first one. The answer is
latched on the first valid image and later faces cannot overwrite it, so this
costs time rather than correctness.
"""

import argparse
import logging
import sys

import fcntl

from task1 import Task1, _acquire_task_lock


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MDP checklist A.5 — navigate around the obstacle. "
                    "Runs the Task 1 pipeline with one obstacle expanded to "
                    "all four of its faces."
    )
    parser.parse_args()

    logging.info("=" * 60)
    logging.info("Checklist A.5 — one obstacle, four faces")
    logging.info("Place exactly ONE obstacle on Android. The face you tap is")
    logging.info("ignored: not knowing it is what A.5 is testing.")
    logging.info("=" * 60)

    # Shares Task 1's lock on purpose. Two orchestrators driving one STM would
    # interleave lines and desync the reply stream — PROTOCOL.md §2 — and that
    # is just as true when one of them is called A.5.
    task_lock = _acquire_task_lock()
    if task_lock is None:
        return 1

    try:
        Task1(a5_mode=True).start()
    finally:
        fcntl.flock(task_lock, fcntl.LOCK_UN)
        task_lock.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
