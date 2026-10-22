# Task 2 version 2: `task2mn_gordon`, with firmware protocol 10

A faster Task 2, built as one process across firmware and Pi. No task2mn file
was changed. Nothing has been flashed or run on the robot yet.

1. **Firmware protocol 10** (`stm/`, PROTOCOL.md §13). Each movement token
   can carry its own speed (`V<n>`) and roll into the next without braking
   (`C`). A whole route round obstacle 2 and back to the bay mouth runs as one
   continuous line, stopping only where precision matters. Old tokens behave
   exactly as before.
2. **Pi shim** `task2mn_gordon_link.py`. It holds:
   - the token rules (a Python mirror of the firmware parser);
   - a model of where a continuous line really goes (the spiral while the
     steering swings, plus speed-dependent radius);
   - a speed model for timing;
   - `enable()`, which teaches the shared `stm.py` the new commands.
3. **The route** `task2mn_gordon.py`. On firmware that answers `?VER` 10 it
   sends continuous lines. On older firmware it sends plain tokens: the same
   route, one stop per primitive.

## What the robot does

1. **Obstacle 1:** task2mn's approach and arrow-1 read, unchanged except that
   the approach is now one straight.
2. **Kick and cruise, one line.**
   - **Moves:** two arcs out to a side lane at |y| = 28 cm (13.6 cm clear of
     obstacle 1), then a fast straight.
   - **On the move:** the side IR on obstacle 1 corrects the lateral position,
     and the ultrasound measures how far away obstacle 2's face is.
   - **Stop:** an RST about 57 cm before the face, where the face scan
     starts.
3. **Arrow 2 is read there, stopped.** The camera is about 45 cm from the
   board and 28 cm to the side, within its 30–50 cm range (checked on the
   floor). It takes shots until the vote is confirmed.
4. **Face scan, one line, the same for either arrow.**
   - **Moves:** turns in toward the centre onto obstacle 2's front face and
     drives along it with the side IR.
   - **Width:** the IR's falling edge is the end, which gives the width. If
     the IR never sees a face, the obstacle is narrow, and the bound that
     gives is enough.
   - **Planning:** the way round is planned in the background during the
     scan.
5. **Round obstacle 2 and home, one line.**
   - **Opposite arrows:** carries on along the face, does a 180° round the
     measured end, runs along the back, then home.
   - **Same arrows:** reverses back along the face, swings out with a reverse
     arc onto a lane outside the other end, then round and home. A forward
     turn there would need 45 cm past the end, and the walls only promise 50.
   - **The way home:** a direct diagonal to the bay mouth, or the obstacle-1
     lane with its IR fix taken on the move, whichever is quicker.
   - **Guards on the move:** it RSTs and replans only if the fix is more than
     2.5 cm off. A side IR reading under 10 cm where the plan expected that
     side clear also stops it.
6. **Park:** task2mn's precision parking, unchanged: side-fence centring,
   then the ultrasound-checked final F.

If gordon has no checked plan it **stops**. Handing over to task2mn's route
is available with `T2G_FALLBACK=1`, but is off by default; see "The
ultrasound finding" below.

## The ultrasound finding (floor, 2026-10-15)

From the side lane, 28 cm off the centreline, the ultrasound read **93 cm for
both a 30 cm and a 60 cm board**. Its beam is a cone: it hears the narrow
board's corner at a slant almost exactly as far as the wide board's face. So
ultrasound gives obstacle 2's distance (to within about 1 cm) but **not its
width**.

- Gordon no longer uses ultrasound for width. The face scan above does it with
  the side IR.
- The simulator now models the cone, with only surfaces facing the sensor, or
  corners, giving an echo. It reproduces the floor reading: 93.9 and 93.0 cm.
- task2mn reads width from ultrasound echoes too. Under the same model it
  parks only 66 of 108 layouts and touches something in 12. That is why
  handing over to it is off by default. It is worth telling the team.

## Results (simulator, wide-beam ultrasound, 108 layouts: d1, d2 in {60, 90, 150}, w2 in {30, 60, 90}, 4 arrow pairs)

| | task2mn | gordon, protocol 9 | **gordon, protocol 10** |
|---|---|---|---|
| Parked within 4 cm, clearance ≥ 3.5 cm | 66/108 | **108/108** | **108/108** |
| Runs that touched anything | 12 | 0 | 0 |
| Smallest clearance | touches | 5.4 cm | 5.0 cm |
| Time, speed model, median | 39.5 s | 34.5 s | **24.8 s** |
| Saved, on the 84 layouts task2mn finishes | | 4.5 s | **14.6 s** |
| Stops per run | 19.0 | 16.5 | **6.0** |
| Movement lines per run | 13.6 | 5.0 | 5.0 |
| With +2 / -2 cm/m drift: parked | 49 / 48 | 93 / 92 | 83 / 87 |
| With drift: touched | 18 / 18 | 0 / 0 | 0 / 0 |

- **Under 15 s:** no layout gets there; the fastest protocol 10 run is
  19.5 s. That leaves the obstacle-1 approach, the arrow-2 read at its stop,
  arcs at 150 rpm, and the precise stops (scan, bay mouth, parking).
- **Same arrows, wide obstacle, protocol 9:** gordon can be about 0.5 s
  slower than task2mn, because measuring the width then reversing out costs
  that. task2mn saves it by passing a wall at the 3.5 cm floor.

**The speed model** (`task2mn_gordon_link.line_seconds`) prices protocol 9
and protocol 10 lines the same way, from motion.h:
- straights ramp up and down and cruise at their speed, and a stop ends in a
  12 cm tail;
- arcs run at their speed, scaled so that a protocol 9 FR90 costs exactly
  `task2mn_park.motion_cost`;
- every start from rest and every stop costs its settle.

Added on top, for every route: the stage-2 vote (38 cm of driving after
obstacle 1 leaves the camera view, the floor log's 1.1 s) and any background
plan not ready in time. For calibration, the model puts task2mn at 39.5 s;
its 2026-09-14 floor run took 32.2 s.

## Calibration before trusting protocol 10 (PROTOCOL.md §13.4)

Arcs stay at V15 (TIGHT's calibrated 150 rpm) until the radius at other
speeds is measured. All of protocol 10's gain above comes from not stopping.
On the floor, in this order:

1. **Steering swing time:** `MOTION_STEER_SLEW_S` in firmware and
   `T2G_STEER_SLEW_S` in `.env`, kept equal. Check: `?WPOSE`,
   `FR90V15C,F5V15`, `?WPOSE` turns −90 ±1°.
2. **Hand-over landing points:** mark the floor and compare against
   `fly_path`.
3. **Stops from speed:** `F100V20` / `F100V25`, within ±1 cm.
4. **RST coast:** set `T2G_RST_COAST_FAST_CM` (8 cm assumed).
5. **Side-IR lag at 200 rpm:** `T2_IR_LAG_CM`.
6. **Optional:** radius at V6–V17, then faster arcs (`T2G_ARC_V`).

## Settings (`.env`)

The existing task2-working settings are enough. Gordon's own, all with
working defaults:

| Setting | Default | What it is |
|---|---|---|
| `T2G_RST_COAST_FAST_CM` | 8 | distance it runs on after RST at speed |
| `T2G_STEER_SLEW_S` | 0.08 | steering swing time; must equal `MOTION_STEER_SLEW_S` |
| `T2G_ARC_V` / `T2G_STRAIGHT_V` / `T2G_SCAN_V` | 15 / 20 / 15 | protocol 10 speeds, ×10 rpm |
| `T2G_FALLBACK` | 0 | 1 hands over to task2mn's route when gordon has no plan |
| `T2G_LOOP_DRIFT_CM_PER_M` | 1.0 | drift allowance on the loop round obstacle 2 |

Also needed: `T2_CAMERA_FROM_AXLE_CM=12.5`, the camera's position for the
stage-2 view.

## Files

| File | Role |
|---|---|
| `task2mn_gordon.py` | The route. `main()` is the full run with Android, camera and PC; `--manual` is a floor test where you press Enter and type the arrows; `--protocol9` forces plain tokens. |
| `task2mn_gordon_link.py` | Protocol 10 for the Pi. |
| `task2mn_gordon_console.py` | `test_stm.py`'s prompt, taught `!IRE`, `?IRW`, `?SEG` and `V`/`C` tokens. |
| `task2mn_gordon_sim.py` | Simulator: task2mn, gordon and gordon10 side by side, with a wide-beam ultrasound and a "true" robot whose steering swing and understeer differ from the planner's. |
| `test_task2mn_gordon.py` | 29 tests. |
| `stm/…` | Protocol 10 firmware; builds with no warnings. Not flashed. |

**Tests,** in a scratch checkout of origin/task2-working: 251 passed, 490
subtests (gordon plus task2mn's four suites).

## Not verified / open

- **Nothing has run on the robot.** The new pieces most likely to need
  floor tuning:
  - the face scan's IR on the face at scan speed;
  - the same-arrow reverse-out;
  - the protocol 10 hand-overs.
- **Drift:** with ±2 cm/m drift, 16–25 runs stop safely rather than finish.
  Mostly these are long courses where the walls sit right at the 50 cm
  minimum.
- **Models:** the vision delay, the planner delay, the hand-over model and
  the speed model are all models.

## Per-layout time (speed model, no drift)

"fails" / "touches" is task2mn under the wide-beam ultrasound.

| d1 | d2 | w2 | arrows | task2mn s | gordon (p9) s | gordon (p10) s | p10 stops | p10 clear cm | p10 park y |
|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|
| 60 | 60 | 30 | LL | 35.1 | 34.0 | 22.2 | 6 | 8.1 | +1.7 |
| 60 | 60 | 30 | LR | fails | 29.7 | 20.5 | 6 | 7.8 | -0.7 |
| 60 | 60 | 30 | RL | fails | 29.7 | 20.5 | 6 | 7.8 | +0.7 |
| 60 | 60 | 30 | RR | 35.1 | 34.0 | 22.2 | 6 | 8.1 | -1.7 |
| 60 | 60 | 60 | LL | 34.4 | 34.4 | 22.4 | 6 | 5.2 | +2.6 |
| 60 | 60 | 60 | LR | 36.9 | 29.0 | 20.2 | 6 | 7.3 | +0.3 |
| 60 | 60 | 60 | RL | 36.9 | 29.0 | 20.2 | 6 | 7.3 | -0.3 |
| 60 | 60 | 60 | RR | 34.4 | 34.4 | 22.4 | 6 | 5.2 | -2.6 |
| 60 | 60 | 90 | LL | 37.1 | 34.2 | 23.7 | 6 | 6.6 | +2.3 |
| 60 | 60 | 90 | LR | 38.2 | 29.5 | 21.8 | 6 | 8.3 | +0.4 |
| 60 | 60 | 90 | RL | 38.2 | 29.5 | 21.8 | 6 | 8.3 | -0.4 |
| 60 | 60 | 90 | RR | 37.1 | 34.2 | 23.7 | 6 | 6.6 | -2.3 |
| 60 | 90 | 30 | LL | touches | 36.0 | 23.5 | 6 | 8.5 | +2.0 |
| 60 | 90 | 30 | LR | fails | 31.7 | 22.1 | 6 | 10.3 | -0.7 |
| 60 | 90 | 30 | RL | fails | 31.7 | 22.1 | 6 | 10.3 | +0.7 |
| 60 | 90 | 30 | RR | touches | 36.0 | 23.5 | 6 | 8.5 | -2.0 |
| 60 | 90 | 60 | LL | 36.6 | 36.4 | 24.6 | 6 | 9.0 | +0.9 |
| 60 | 90 | 60 | LR | 35.2 | 31.0 | 22.6 | 6 | 8.9 | -1.1 |
| 60 | 90 | 60 | RL | 35.2 | 31.0 | 22.6 | 6 | 8.9 | +1.1 |
| 60 | 90 | 60 | RR | 36.6 | 36.4 | 24.6 | 6 | 9.0 | -0.9 |
| 60 | 90 | 90 | LL | 40.3 | 38.2 | 25.3 | 6 | 5.0 | +2.6 |
| 60 | 90 | 90 | LR | 37.8 | 33.4 | 22.6 | 5 | 6.1 | -2.5 |
| 60 | 90 | 90 | RL | 37.8 | 33.4 | 22.6 | 5 | 6.1 | +2.5 |
| 60 | 90 | 90 | RR | 40.3 | 38.2 | 25.3 | 6 | 5.0 | -2.6 |
| 60 | 150 | 30 | LL | touches | 38.3 | 25.3 | 6 | 8.2 | +2.0 |
| 60 | 150 | 30 | LR | 37.8 | 34.0 | 23.6 | 6 | 8.9 | -0.7 |
| 60 | 150 | 30 | RL | 37.8 | 34.0 | 23.6 | 6 | 8.9 | +0.7 |
| 60 | 150 | 30 | RR | touches | 38.3 | 25.3 | 6 | 8.2 | -2.0 |
| 60 | 150 | 60 | LL | 38.3 | 38.8 | 26.3 | 6 | 9.0 | +0.9 |
| 60 | 150 | 60 | LR | 38.6 | 34.1 | 24.3 | 6 | 10.9 | -2.4 |
| 60 | 150 | 60 | RL | 38.6 | 34.1 | 24.3 | 6 | 10.9 | +2.4 |
| 60 | 150 | 60 | RR | 38.3 | 38.8 | 26.3 | 6 | 9.0 | -0.9 |
| 60 | 150 | 90 | LL | 42.0 | 42.6 | 29.8 | 8 | 5.1 | +0.1 |
| 60 | 150 | 90 | LR | 42.9 | 37.4 | 28.7 | 8 | 9.3 | -3.2 |
| 60 | 150 | 90 | RL | 42.9 | 37.4 | 28.7 | 8 | 9.3 | +3.2 |
| 60 | 150 | 90 | RR | 42.0 | 42.6 | 29.8 | 8 | 5.1 | -0.1 |
| 90 | 60 | 30 | LL | 36.8 | 31.2 | 22.0 | 6 | 5.3 | +2.6 |
| 90 | 60 | 30 | LR | fails | 26.2 | 19.5 | 5 | 6.1 | -0.0 |
| 90 | 60 | 30 | RL | fails | 26.2 | 19.5 | 5 | 6.1 | +0.0 |
| 90 | 60 | 30 | RR | 36.8 | 31.2 | 22.0 | 6 | 5.3 | -2.6 |
| 90 | 60 | 60 | LL | 36.9 | 33.6 | 23.0 | 6 | 10.7 | +2.4 |
| 90 | 60 | 60 | LR | 36.4 | 28.2 | 20.1 | 5 | 9.1 | +0.1 |
| 90 | 60 | 60 | RL | 36.4 | 28.2 | 20.1 | 5 | 9.1 | -0.1 |
| 90 | 60 | 60 | RR | 36.9 | 33.6 | 23.0 | 6 | 10.7 | -2.4 |
| 90 | 60 | 90 | LL | 39.6 | 35.1 | 24.5 | 6 | 10.4 | +2.2 |
| 90 | 60 | 90 | LR | 38.9 | 30.4 | 21.7 | 5 | 6.5 | -2.3 |
| 90 | 60 | 90 | RL | 38.9 | 30.4 | 21.7 | 5 | 6.5 | +2.3 |
| 90 | 60 | 90 | RR | 39.6 | 35.1 | 24.5 | 6 | 10.4 | -2.2 |
| 90 | 90 | 30 | LL | touches | 36.4 | 24.8 | 6 | 8.3 | +1.3 |
| 90 | 90 | 30 | LR | fails | 32.0 | 23.1 | 6 | 10.0 | -0.9 |
| 90 | 90 | 30 | RL | fails | 32.0 | 23.1 | 6 | 10.0 | +0.9 |
| 90 | 90 | 30 | RR | touches | 36.4 | 24.8 | 6 | 8.3 | -1.3 |
| 90 | 90 | 60 | LL | 39.0 | 34.1 | 24.4 | 6 | 6.5 | +2.6 |
| 90 | 90 | 60 | LR | 34.6 | 28.6 | 22.2 | 6 | 9.1 | +0.1 |
| 90 | 90 | 60 | RL | 34.6 | 28.6 | 22.2 | 6 | 9.1 | -0.1 |
| 90 | 90 | 60 | RR | 39.0 | 34.1 | 24.4 | 6 | 6.5 | -2.6 |
| 90 | 90 | 90 | LL | 41.7 | 37.1 | 26.0 | 6 | 10.5 | +2.2 |
| 90 | 90 | 90 | LR | 38.0 | 32.3 | 23.1 | 5 | 8.5 | -2.4 |
| 90 | 90 | 90 | RL | 38.0 | 32.3 | 23.1 | 5 | 8.5 | +2.4 |
| 90 | 90 | 90 | RR | 41.7 | 37.1 | 26.0 | 6 | 10.5 | -2.2 |
| 90 | 150 | 30 | LL | touches | 38.7 | 26.6 | 6 | 9.2 | +1.3 |
| 90 | 150 | 30 | LR | 39.5 | 34.4 | 24.9 | 6 | 10.0 | -0.9 |
| 90 | 150 | 30 | RL | 39.5 | 34.4 | 24.9 | 6 | 10.0 | +0.9 |
| 90 | 150 | 30 | RR | touches | 38.7 | 26.6 | 6 | 9.2 | -1.3 |
| 90 | 150 | 60 | LL | 40.8 | 40.0 | 28.0 | 6 | 9.1 | +1.7 |
| 90 | 150 | 60 | LR | 41.1 | 35.3 | 25.9 | 6 | 10.9 | -1.2 |
| 90 | 150 | 60 | RL | 41.1 | 35.3 | 25.9 | 6 | 10.9 | +1.2 |
| 90 | 150 | 60 | RR | 40.8 | 40.0 | 28.0 | 6 | 9.1 | -1.7 |
| 90 | 150 | 90 | LL | 43.4 | 39.6 | 27.6 | 6 | 5.5 | +2.4 |
| 90 | 150 | 90 | LR | 40.0 | 34.6 | 27.4 | 8 | 8.2 | -0.0 |
| 90 | 150 | 90 | RL | 40.0 | 34.6 | 27.4 | 8 | 8.2 | +0.0 |
| 90 | 150 | 90 | RR | 43.4 | 39.6 | 27.6 | 6 | 5.5 | -2.4 |
| 150 | 60 | 30 | LL | 38.5 | 36.2 | 25.4 | 6 | 8.4 | +1.4 |
| 150 | 60 | 30 | LR | fails | 31.2 | 23.0 | 5 | 6.1 | -0.9 |
| 150 | 60 | 30 | RL | fails | 31.2 | 23.0 | 5 | 6.1 | +0.9 |
| 150 | 60 | 30 | RR | 38.5 | 36.2 | 25.4 | 6 | 8.4 | -1.4 |
| 150 | 60 | 60 | LL | 38.6 | 35.1 | 24.7 | 6 | 10.7 | +2.5 |
| 150 | 60 | 60 | LR | 38.8 | 28.4 | 21.9 | 5 | 9.1 | +0.0 |
| 150 | 60 | 60 | RL | 38.8 | 28.4 | 21.9 | 5 | 9.1 | -0.0 |
| 150 | 60 | 60 | RR | 38.6 | 35.1 | 24.7 | 6 | 10.7 | -2.5 |
| 150 | 60 | 90 | LL | 41.4 | 35.3 | 26.1 | 6 | 10.4 | +2.3 |
| 150 | 60 | 90 | LR | 41.5 | 30.5 | 23.3 | 5 | 6.5 | -2.4 |
| 150 | 60 | 90 | RL | 41.5 | 30.5 | 23.3 | 5 | 6.5 | +2.4 |
| 150 | 60 | 90 | RR | 41.4 | 35.3 | 26.1 | 6 | 10.4 | -2.3 |
| 150 | 90 | 30 | LL | touches | 38.1 | 26.6 | 6 | 9.2 | +1.3 |
| 150 | 90 | 30 | LR | fails | 33.7 | 24.9 | 6 | 10.0 | -0.9 |
| 150 | 90 | 30 | RL | fails | 33.7 | 24.9 | 6 | 10.0 | +0.9 |
| 150 | 90 | 30 | RR | touches | 38.1 | 26.6 | 6 | 9.2 | -1.3 |
| 150 | 90 | 60 | LL | 40.8 | 37.7 | 26.0 | 6 | 11.2 | +2.3 |
| 150 | 90 | 60 | LR | 39.5 | 32.2 | 26.0 | 6 | 9.1 | -1.4 |
| 150 | 90 | 60 | RL | 39.5 | 32.2 | 26.0 | 6 | 9.1 | +1.4 |
| 150 | 90 | 60 | RR | 40.8 | 37.7 | 26.0 | 6 | 11.2 | -2.3 |
| 150 | 90 | 90 | LL | 43.4 | 38.9 | 27.6 | 6 | 10.5 | +2.4 |
| 150 | 90 | 90 | LR | 39.5 | 34.1 | 24.8 | 5 | 8.5 | -2.5 |
| 150 | 90 | 90 | RL | 39.5 | 34.1 | 24.8 | 5 | 8.5 | +2.5 |
| 150 | 90 | 90 | RR | 43.4 | 38.9 | 27.6 | 6 | 10.5 | -2.4 |
| 150 | 150 | 30 | LL | touches | 40.5 | 28.3 | 6 | 9.2 | +1.3 |
| 150 | 150 | 30 | LR | 41.2 | 36.1 | 26.6 | 6 | 10.0 | -0.9 |
| 150 | 150 | 30 | RL | 41.2 | 36.1 | 26.6 | 6 | 10.0 | +0.9 |
| 150 | 150 | 30 | RR | touches | 40.5 | 28.3 | 6 | 9.2 | -1.3 |
| 150 | 150 | 60 | LL | 42.5 | 41.8 | 29.8 | 6 | 9.1 | +1.7 |
| 150 | 150 | 60 | LR | 42.8 | 37.1 | 27.6 | 6 | 10.9 | -1.3 |
| 150 | 150 | 60 | RL | 42.8 | 37.1 | 27.6 | 6 | 10.9 | +1.3 |
| 150 | 150 | 60 | RR | 42.5 | 41.8 | 29.8 | 6 | 9.1 | -1.7 |
| 150 | 150 | 90 | LL | 45.1 | 44.5 | 29.2 | 6 | 10.5 | +2.4 |
| 150 | 150 | 90 | LR | 46.2 | 39.5 | 31.7 | 8 | 9.1 | -3.4 |
| 150 | 150 | 90 | RL | 46.2 | 39.5 | 31.7 | 8 | 9.1 | +3.4 |
| 150 | 150 | 90 | RR | 45.1 | 44.5 | 29.2 | 6 | 10.5 | -2.4 |
