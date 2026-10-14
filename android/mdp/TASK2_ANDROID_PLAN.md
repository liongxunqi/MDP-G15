# Task 2 Android integration plan

## Requirements and current branch

The official `Rules for the Task 1 and Task 2.pdf` (pages 5–8) says obstacle
positions and parking gaps are unknown during Task 2 preparation. They must not
be measured or entered. After supervisor approval, Android Start begins one
autonomous run. The robot reads each arrow, passes the indicated side, returns
fully into the original carpark, and stops within three minutes. A tiled view
of raw camera frames with arrow bounding boxes is required on Android **or PC**.
The competition allows one retry shared across both tasks. This is a judging
rule, not an Android counter: local physical tests must remain repeatable.

This checkout is `task2-working`. The matching Pi entry point is
`rpi/mdp_rpi/task2.py`; its PC arrow service is `pc/task2_pc.py`. Task 1 and
the old D-pad remain present for the Android demonstration. Never run the Task 1
and Task 2 Pi programs on the same Bluetooth/PC sockets simultaneously.

## Implemented Android flow

1. Select Task 2 in Arena. Saved Task 1 obstacles, target faces, and recognized
   labels remain in local state, but Task 2 shows an empty map and sends no map
   commands to the Task 2 Pi handler. Placement and Task 1 `PATH` are unavailable.
2. Android sends `TASK2,STATUS?` on Task 2 selection, Bluetooth connection, and
   local New attempt. The Pi answers `TASK2,READY`, `TASK2,NOT_READY`, or
   `TASK2,RUNNING`; it also announces READY when PC and camera become ready.
   Generic `STATUS,OK` is never treated as Task 2 readiness.
3. Start requires Bluetooth, no pending manual move or
   placement, a setup phase, and explicit Task 2 READY. It queues `BEGIN` only
   once. A definite Bluetooth queue rejection returns to setup. A possibly
   partial send or disconnected active run remains uncertain and is not replayed.
   The existing Bluetooth newline toggle remains available in both tasks; the
   shared Pi receiver accepts commands with or without a trailing newline.
   The Pi still terminates its replies with newline; the toggle only changes
   Android-to-Pi commands.
4. A Pi rejection `TASK2,NOT_READY` after BEGIN returns the app to setup; wait
   for a new READY before trying again. RUNNING/DONE/FAILED and arrow reports
   appear in status/history, while raw TX/RX stays in Logs. Task 2 arrow reports
   do not change saved Task 1 target labels.
5. New attempt can be used repeatedly in local tests. It retains the Task 1
   map/results, clears Task 2 run state, and asks the Pi for fresh readiness.
   The Task 2 Pi and PC programs each serve one mission and exit; restart both
   before another physical run. In competition, obtain supervisor approval and
   respect the shared retry rule before pressing Start again.

## Exact Bluetooth messages and where they appear

| Direction | Message | Android behavior |
| --- | --- | --- |
| Android → Pi | No `CLEAR` or `OBSTACLE` in Task 2 | The Pi discovers obstacles live; saved Task 1 map stays local. |
| Android → Pi | `TASK2,STATUS?` | Query readiness after connect, selection, or New attempt. |
| Pi → Android | `STATUS,CONNECTED TO RPI` | Bluetooth status; **not** mission readiness. |
| Pi → Android | `STATUS,OK` | Existing generic status; **not** proof Task 2 is ready. |
| Pi → Android | `TASK2,NOT_READY` | Start disabled. If BEGIN was rejected, setup is restored. |
| Pi → Android | `TASK2,READY` | Start enabled once setup guards pass. |
| Android → Pi | `BEGIN` | One mission request after supervisor approval. |
| Pi → Android | `STATUS,RUNNING,0,5` | Start accepted; active run, step 0 of 5. |
| Pi → Android | `STATUS,RUNNING,1,5` through `5,5` | Approach 1, bypass 1, approach 2, round 2, return and park. Some stages may repeat. |
| Pi → Android | `TASK2,ARROW,1,LEFT/RIGHT` and `TASK2,ARROW,2,LEFT/RIGHT` | Show recognized direction; keep Task 1 image IDs intact. |
| Pi → Android | `TASK2,RUNNING` | Response to a readiness query when a run is already active; Start disabled. |
| Pi → Android | `STATUS,DONE` | Terminal success; save/show PC evidence and prepare local retry if needed. |
| Pi → Android | `STATUS,FAILED` | Terminal failure or startup failure. Pi may send it twice during shutdown. |
| Android → Pi | `s` | Emergency stop request; verify stationary robot and terminal status before retry. |

The Logs tab records timestamped QUEUED, TX, RX, CONNECTION, and ERROR entries
(last 500). `QUEUED` means accepted by the local Bluetooth queue; `TX` means
written to the socket, not acknowledged by the Pi. The Pi terminal and
`rpi/mdp_rpi/logs/task2_<timestamp>.log` show calibration, raw WPOSE, stage
transitions, arrow decisions, route selection, parking validation, and failures.
`pc/logs/task2_pc_<timestamp>.log` and
`pc/runs/task2_<timestamp>/stitched_task2.jpg` hold annotated evidence; open
that JPEG in one PC window after the run. The existing `stitched_live.html` is
for Task 1 and does not display Task 2 evidence.

## Rehearsal and contingencies

1. Start the Pi Task 2 program and PC Task 2 service from their documented
   directories. Check the calibration/profile/STM queries and model labels.
   The Pi sends READY only when it has a fresh camera frame and PC vision is
   ready. Confirm the tablet shows Task 2 ready before supervisor Start.
2. If READY never arrives, check PC connection/model warm-up/camera, Pi log,
   Bluetooth connection, the selected framing mode, and RX/TX Logs. Reconnect
   Android to trigger a fresh query. Do not substitute generic `STATUS,OK`.
3. If BEGIN was not queued, recheck Bluetooth and try after READY. If BEGIN was
   queued but progress never arrives, inspect Pi log before any retry: the
   socket may have dropped after queueing. If a run disconnects, Android keeps
   the outcome uncertain and never replays BEGIN on reconnect. It asks the Pi
   for status again. To recover for a local test, physically verify that the
   previous run has ended and the robot is stationary, then use New attempt.
   A fresh READY response is required before Start is enabled again; a RUNNING
   response returns the display to unknown and keeps Start disabled.
4. If `TASK2,NOT_READY` follows BEGIN, wait for the next READY. If `FAILED`
   arrives, check Pi and PC logs and restart their one-run programs before a
   local New attempt. After `s`, physically verify the robot has stopped.
5. Rehearse both arrow directions, same/opposite combinations, obstacle length
   and gap changes, camera/model loss, Bluetooth loss, restart after completion,
   and evidence viewing. Check full parking and automatic stop inside three
   minutes. No in-run tablet touches are allowed in competition beyond the
   supervisor-approved Start.

## Verification and remaining limits

- Focused Task 2/Task 1 Android unit tests and the app/arena Compose
  instrumentation suites passed on the Galaxy Tab A7 Lite emulator (25 app
  tests, 16 arena tests). Repeat on the physical tablet before competition.
- Run `python3 -B -m unittest test_task2_integration -q` from `rpi/mdp_rpi`
  in an environment with the Pi/PC Python dependencies. Bench-test the actual
  Bluetooth and camera path; local Python syntax/unit checks cannot establish
  hardware timing or route correctness.
- The two stale fractional-pose test expectations on this branch were aligned
  with the current pose model: reported telemetry is rounded only for the
  legacy grid adapter and retained even when outside the arena. Operator moves
  still use footprint validation. Both complete Android unit suites now pass.
- The Pi Python integration suite could not run in the current local Python
  environment because `python-dotenv` is missing. The edited Python files pass
  syntax checks and six extracted status-protocol assertions pass; run the
  integration suite in the team's Pi/PC environment before hardware trials.
- `tbricks` calibration is marked unverified in the Pi README. Verify actual
  clearance and stop behavior before a judged run.
- Acknowledged BEGIN, STOP, and run/attempt IDs would remove the last outcome
  ambiguity after a lost connection. They require an agreed protocol change
  across Android and Pi; the current implementation errs on disabling replay.
- In delimiter-free mode the legacy Pi receiver treats a Bluetooth `recv()`
  chunk as one command. RFCOMM can split or combine writes, so this mode needs
  an actual tablet-to-Pi bench test for `TASK2,STATUS?`, `BEGIN`, and `s`.
  Task 2 sends no map batch before its readiness query, reducing adjacent
  writes. Fully removing this risk requires agreed command framing on both
  ends; the app does not force a framing mode without hardware evidence.
- Android sends a blank heartbeat line in both framing modes. The Pi ignores
  it, and the socket write allows dead-link detection even when commands are
  sent without trailing newlines.
- The Task 2 Pi logs WPOSE but does not transmit a `ROBOT,x,y,heading` update to
  Android. Live Task 2 pose on the map would need an agreed carpark-to-grid
  transform. Task 1 pose display remains available for the Android demo.
