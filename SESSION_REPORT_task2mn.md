# Session report: Task 2 (task2mn), 2026-10-15

What changed in this session in the STM32 firmware (`stm/`) and the Raspberry Pi
code (`rpi/mdp_rpi/`), why, how it was checked, and what is still open.

---

## 1. Where things stand

| Item | State |
|---|---|
| Firmware (`stm/`) | Committed as `8e99f98` on branch **`task2mn`** and pushed to `origin/task2mn`. Protocol version is now **9**. |
| Firmware build | Built with STM32CubeIDE 1.9.0's GCC 10.3: no warnings, no errors (`STM32X.elf`, about 78 KB of text). |
| Firmware on the robot | **Not flashed or floor-tested yet.** |
| Pi files (`rpi/mdp_rpi/`) | **Not committed.** Local only: the `task2mn*.py` files are new, and `communications/stm.py`, `communications/cal_profile.py` and `task2t.py` are modified. |
| Old `stm/STM32X/` folder | An untracked, stale copy of the project. Not committed; it is safe to delete. |

The `task2mn` branch was created from `maha-task2` (`85c349f`). The firmware diff
shows far more changed lines (about 4,500) than were edited: something other than
the edits rewrote whole lines in several files. Line endings were suspected but not
checked. The code itself builds.

---

## 2. Investigation that led to the changes

### 2.1 GitHub `task2-working` compared with the local folder
- `origin/task2-working` (7a583d6, 2026-10-15) is the team's latest Task 2. It is an
  integrated run: Android BEGIN, PC arrow recognition, the TIGHT route, and parking.
- Its `communications/stm.py` was rolled back to **protocol 5**. Compared with the
  local client it lost:
  - the gyro-drift startup check;
  - `!FAST0` being sent by every other task;
  - `FAIL,SHORT` and `?ENC` handling;
  - the leading-zero guard on movement tokens.
- It is still able to send `!FAST1`, and gates it on `?VER`.
- It plans every turn with a **30.0 cm** radius, from `T2T_RADIUS_CM=30.0` in its
  `.env`, with a fallback of 30.0 in `task2t_park.py`. The local `task2t_park.py`
  falls back to 29.1.

### 2.2 Review of GitHub's Task 2 (high-risk findings)
1. A PC socket timeout (5 s) or an error **aborts the run** (`task2_vision.serve` raises).
2. The camera watchdog aborts the run even after both arrows are known.
3. If no arrow is confirmed within 8 s, the run fails; there is no fallback.
4. Parking skips centring whenever a straight entry keeps 3.5 cm to the fence
   (heading tolerance 12°). In the simulator with drift, robots parked 14–16 cm off
   centre.
5. A failed post-park check sends RST to a robot that has already stopped.

### 2.3 Floor log, 2026-09-14 02:11 (GitHub version, before this session's edits)
- **Failure:** the run failed 12 s in. "Narrow-lane scan finished without a
  plausible obstacle-2 sighting". The left IR never read below 58 cm along the
  whole scan.
- **RST let the wheels roll:** the robot travelled 18.5 cm after RST, from about
  50 cm/s, because `abort_everything()` called `Motors_Coast()` straight after the
  brake.
- **Narrow decision from silence:** four `65535` (no echo) ultrasound readings
  were taken as proof that obstacle 2 is narrow.
- **Scan too fast to see the face:** the narrow scan polled the IR at fast speed.
  Obstacle 2's 10 cm end face passes in about 0.2 s, and stopping by RST made the
  at-rest confirmation impossible.
- **Two meanings of 65535:** the IR reports `65535` both for "closer than 10 cm"
  and for "nothing in range".

### 2.4 "STM doesn't connect"
- The most likely cause is the **gyro dropping out**. Once marked stale, the gyro
  never recovered until power-off, and GitHub's startup reports a not-ready gyro
  as "STM IMU/IR/US startup queries failed".
- Other link risks found:
  - the serial port name is fixed at `/dev/ttyACM0`;
  - the port is not opened exclusively;
  - a 2.5 s blocking IMU calibration runs at boot.

---

## 3. Firmware changes (`stm/`, committed `8e99f98`)

### 3.1 IMU I2C recovery and gyro re-arm
Files: `PeripheralDrivers/Src/imu.c`, `Inc/imu.h`, `Core/Src/main.c`

- **`i2c_bus_recover()`:**
  - Deinitialises I2C2 and drives PB10/PB11 as open-drain GPIO.
  - Clocks SCL up to 9 times until the gyro releases SDA, then sends a STOP.
  - Restores the AF4 pins, resets I2C2 through RCC (this clears the STM32F4 stuck
    BUSY erratum), and calls `HAL_I2C_Init` again.
- **`IMU_Init()`** recovers the bus before *every* bring-up attempt. A failed boot
  calibration no longer leaves the gyro dead.
- **Stale gyro (100 ms with no good read):**
  - `IMU_Tick` marks it unconfigured instead of dead for the rest of the power-on.
  - `IMU_Poll()` re-arms it once the robot has been still for 0.5 s with nothing
    queued (`IMU_REARM_STILL_TICKS`). It then recovers the bus, runs bring-up
    (retried every `IMU_REARM_PERIOD_MS` = 1000 ms), and needs 20 good reads.
  - If boot never measured a bias, it re-measures one without blocking (200
    samples, same sanity limit).
  - The heading is left untouched, so odometry carries straight across the gap.
- **New `IMU_NoteStill()`:** called every tick from `main.c` with "no move, no
  encoder change, queue empty".
- **New counter `IMU_GetRecoveryCount()`:** the stall counter is kept.
- **OLED mode 10:** when the gyro is not ready it now shows `st<stalls> rc<recoveries>`.
- **Boot log line:** now reads "IMU FAILED - retrying while still; check PB12, 1.8V rail".

### 3.2 Fast mode: speed ramp, 200 rpm and speed-scaled steering
Files: `Src/motion.c`, `Inc/motion.h`, `Src/odom.c`, `Inc/odom.h`

**Fast straights (`!FAST1` only):**
- Start at `MOTION_RAMP_START_RPM` = 60 and speed up by 4 rpm per tick to
  `MOTION_FAST_CRUISE_RPM` = **200** (was a step to 150).
- On the way in they slow by 6 rpm per tick to the 40 rpm approach speed.
- The slow-down starts when the remaining distance equals the slowing distance
  from the speed actually reached plus `MOTION_FAST_TAIL_MM` = 120 mm. From full
  cruise that is about 230 mm (`MOTION_FAST_APPROACH_MM`, now a computed value).
- The switch to slowing latches, so a lagging wheel cannot cause a surge.
- The final stop still happens from a steady 40 rpm, so `MOTION_BRAKE_MM` stays valid.

**Speed-scaled steering:**
- `HEADING_GAIN_REF_RPM` = 100. Above 100 rpm, both the heading gain
  (`HEADING_KP_US`) and the cross-track gain are multiplied by `100 / |rpm|`.
- Loop gain no longer rises with speed. The backlash push is not scaled.

**Unchanged:** normal mode (no ramp, the same steps) and arcs (150 rpm, existing
calibration).

**Simulated time per straight** (ideal tracking, so real runs will be slower):

| Distance | Before (150 rpm step) | After (200 rpm ramp) |
|---|---|---|
| 40 cm | 1.91 s | 1.49 s |
| 1 m | 3.07 s | 2.35 s |
| 1.5 m | 4.05 s | 3.10 s |

### 3.3 Protocol 9: `?IRW`, IR edge latch, braking RST
Files: `Inc/commands.h`, `Src/commands.c`, `Inc/rpilink.h`, `Src/rpilink.c`,
`Core/Src/main.c`, `PROTOCOL.md`

- **`?IRW`** returns `IRW,<l_cm>,<r_cm>,<l_raw>,<r_raw>,<x_mm>,<y_mm>,<hdg_x10>,<ms>`.
  - Both side IRs, their median-filtered raw counts, and the `?WPOSE` pose, all
    captured in the **same 10 ms tick** by `RpiLink_Tick()`.
  - `RpiLink_Tick()` runs right after `Odom_Update()`.
  - Raw counts separate the two meanings of `65535`: high counts mean closer than
    10 cm.
- **`!IRE<n>`** arms the edge latch. `n = side*10000 + lo*100 + hi`, so for example
  `!IRE11227` means left, 12–27 cm; `!IRE0` disarms.
  - The parser rejects a bad side or a window outside 10..80 cm.
  - The latch only watches while a move is running.
  - It needs 3 in-window ticks in a row (`RPILINK_IRE_CONFIRM_TICKS`) and records
    the pose of the first one. The move carries on.
- **`?IRE`** returns `IRE,<state>,<side>,<cm>,<raw>,<x_mm>,<y_mm>,<hdg_x10>,<ms>`.
  State 0 = off, 1 = armed, 2 = latched.
- **`RST` now holds the brake.** `abort_everything()` no longer calls
  `Motors_Coast()` after `Motion_Stop()`; the shorted bridges stay on until the
  next move.
- `CMD_PROTOCOL_VERSION` is now 9, and `Cmd_IsImmediate()`'s second range ends at
  `CMD_SET_IR_EDGE`.

---

## 4. Pi changes in the local `rpi/mdp_rpi/` (uncommitted)

### 4.1 `communications/stm.py` (protocol 9 client)
- `PROTOCOL_VERSION = 9` and `IRW_MIN_PROTOCOL = 9`.
- `QUERY_TAGS` now includes `IRW` and `IRE`, and `!IRE` tokens pass validation.
- New methods:
  - `read_irw()`: cm values are `None` when there is no reading; raw counts and
    pose are included.
  - `ir_edge_token()`, `set_ir_edge(side, lo, hi)` and `read_ir_edge()`.
- The `read_imu()` docstring now says the firmware re-arms a dropped gyro.

### 4.2 `communications/cal_profile.py`
- `gyro_in_use()` now waits up to `GYRO_REARM_WAIT_S` (4 s, set in `.env`) for the
  gyro to re-arm before refusing to start.
- Its error message still says "ENCODER heading", which tests rely on, and it
  points to OLED mode 10.
- Heading-log messages no longer say the fallback lasts until power-off.

### 4.3 `task2t.py`
Docstring only: fast mode now ramps up to 200 rpm.

---

## 5. Mahan's Task 2: the `task2mn` files (new, uncommitted)

Copied from GitHub **`origin/task2-working`**, so the team's `task2*` / `task2t*`
files stay untouched. They are meant to be dropped into that branch's
`rpi/mdp_rpi/` on the Pi, and they need its `run_log.py`, `communications/*`,
`image_capture/camera.py` and `cal_profiles/`.

| New file | Copied from | Role |
|---|---|---|
| `task2mn.py` | `task2.py` | Entry point: Android, PC and camera threads, startup, shutdown |
| `task2mn_route.py` | `task2t.py` | The TIGHT route |
| `task2mn_park.py` | `task2t_park.py` | Return and parking geometry, `ParkRunner` |
| `task2mn_vision.py` | `task2_vision.py` | Arrow recognition worker |
| `task2mn_link.py` | new | Protocol 9 helpers, plus `enable()`, which teaches the older shared `stm.py` the new commands at run time |
| `task2mn_sim.py` | `task2t_sim.py` | Laptop simulator, now with the IR edge latch and `?IRW` |
| `test_task2mn.py` | `test_task2t_park.py` | Route and parking tests |
| `test_task2mn_integration.py` | `test_task2_integration.py` | Entry-point integration tests |
| `test_task2mn_flow.py` | `test_task2_flow_regression.py` | Flow regression tests |
| `test_task2mn_fixes.py` | new | 14 tests for the changes below |

### 5.1 Fixes in the task2mn files
1. **Narrow obstacle 2 scan uses the IR edge latch** (`task2mn_route.find_narrow`;
   protocol 9 only, older firmware keeps the polled scan):
   - arms `!IRE` on the inner side and scans at **normal** speed;
   - polls `?IRE` and `?IRW` while driving;
   - on a latch, sends RST (which now brakes) and places the face at the latched
     pose;
   - stops safely if `?IRW` raw counts show the inner IR inside 10 cm, meaning
     obstacle 2 is wider than the lane allows;
   - widens the window 2 cm outward.
2. **Recognition no longer ends the run** (`task2mn_vision.py`, `task2mn.py`):
   - The PC timeout is `T2MN_PC_TIMEOUT_S` = 15 s, up from 5.
   - A failure after READY calls `vision.fail()` instead of cancelling.
   - `wait_arrow()` falls back to the last vote, then to `T2MN_DEFAULT_ARROW`
     (default `left`).
   - The camera watchdog only acts while an arrow is still being waited for.
3. **Startup waits for the gyro:** up to `T2MN_GYRO_WAIT_S` = 4 s, with its own
   error message instead of "IMU/IR/US queries failed".
4. **No RST after a finished route:** `route_finished` is set after the route
   completes, so a robot that only failed the parking check is left alone.
   `DONE`/`FAILED` reach the tablet only once.
5. **Parking centres unless genuinely centred** (`task2mn_park.py`):
   - `PARK_STRAIGHT_CLEARANCE_CM` = 10 (was 3.5) and the new `PARK_ALIGN_TOL_DEG`
     = 6 decide whether to run centring line C.
   - The final-F hard floor stays at 3.5 cm (`PARK_STRAIGHT_FLOOR_CM`).
   - New `centre_offset_measured()` works out the bay offset from both fences,
     from one fence alone, or from an IR closer than 10 cm (via `?IRW` raw).
     Before, the old code fell back to odometry in all these cases.
6. **`WorldFrame.to_world()`** converts any raw `?WPOSE`-frame sample, from `?IRW`
   or `?IRE`, to world cm.
7. **Quiet logging:** a `QuietPolling` filter drops per-query `?IR`, `?WPOSE`,
   `?IRW` and `?IRE` lines from the console and the log file.

### 5.2 Test results (in a scratch checkout of `origin/task2-working`)
- `test_task2mn.py` + `test_task2mn_integration.py` + `test_task2mn_flow.py`:
  **197 passed** (183 subtests).
  - One test changed with the new parking policy:
    `test_safe_offset_mouth_skips_wobbly_centring_line` became
    `test_far_off_centre_mouth_is_centred`.
  - A new test, `test_near_centred_mouth_skips_centring_line`, was added.
- `test_task2mn_fixes.py`: **14 passed**.
- Local `test_stm.py` + `test_calibration.py` + `test_task2t_park.py` (excluding
  EndToEnd): **97 passed**, using a stub `serial` module because pyserial is not
  installed on this laptop.

**Drift simulations (3 cm/m):**

| Case | GitHub version (y off centre at park) | task2mn (y off centre at park) |
|---|---|---|
| 90/82/60 right, +3 | −15.8 cm | +0.3 cm |
| 90/82/60 right, −3 | +16.2 cm | +0.1 cm |
| 60/60/90 left, +3 | −14.5 cm | +0.2 cm |
| 150/150/90 right, −3 | parked at +22.1 cm | refuses at the mouth (exit 1) |

---

## 6. Known issues still open
- **Contact with obstacle 2 under heavy drift:** some simulated routes touch or
  graze it (150/150/90 −3 cm/m: −1.0 cm; 60/60/90 +3: 0.3 cm). GitHub's version
  behaves identically, so this is in the route around obstacle 2, not in this
  session's changes.
- **IR fix on obstacle 1 sometimes gets one sample** at fast speed, so the return
  runs on odometry alone. It could be moved to `?IRW` or the latch.
- **`T2_IR_LAG_CM`** (2.0) is not measured. At 200 rpm the lag is closer to
  3–3.5 cm.
- **One turn ending more than 10° off still ends the run:** `FAIL,SHORT` gets no
  re-plan.
- **Simulator end-to-end tests are flaky:** results depend on poll timing.
- **Calibration:** `tr18` is unverified and its steering is asymmetric (L 425 /
  R 617 µs). The planner radius is 30 cm and has not been measured on the floor.
- **STM link hardening (not done):** stable `/dev/serial/by-id` path, exclusive
  port open, `?VER` retry.

---

## 7. How to run

**STM32CubeIDE:**
1. Use a new workspace outside the repo.
2. **Import → Existing Projects into Workspace → `MDP-G15-main\stm`**, with
   "Copy projects into workspace" left **unticked**.
3. Build.
4. Right-click the project → **Debug As / Run As → STM32 C/C++ Application**.

After flashing, `?VER` should answer `VER,MDPG15-STM32,9`.

**Pi** (in a `task2-working` checkout of `rpi/mdp_rpi/`, plus the 5 runtime
`task2mn` files: `task2mn.py`, `task2mn_route.py`, `task2mn_park.py`,
`task2mn_vision.py`, `task2mn_link.py`):

```bash
python3 task2mn.py
```

**Laptop:**

```bash
python3 task2_pc.py --host 192.168.15.15
```

**Optional `.env` settings:**
- `T2MN_DEFAULT_ARROW`
- `T2MN_PC_TIMEOUT_S`
- `T2MN_GYRO_WAIT_S`
- `T2MN_PARK_STRAIGHT_CLEARANCE_CM`
- `T2MN_PARK_ALIGN_TOL_DEG`
- `GYRO_REARM_WAIT_S` (local `cal_profile.py`)

**First floor checks:**
1. `?IRW` with a hand at different distances from each side IR.
2. `!IRE11227`, then `F50` past a box on the left, then `?IRE` (expect state 2).
3. `F100` with `!FAST1`, watching for weaving and checking the stop with `?DIST`.
4. RST mid-move: the robot should stop in a few cm, not about 18.
5. OLED mode 10: wiggle the I2C connector; `st`/`rc` should rise and the gyro
   should come back within about 1 s of standing still.

---

## 8. Fixes from the 2026-09-14 floor runs (05:46 success, 05:47 failure)

| Log | Problem | Fix (`task2mn_*`) |
|---|---|---|
| 05:47 failed | Face scan along obstacle 2 aborted the run on **one** 11 cm IR reading (median 13): 5.7 cm modelled body gap under the 6.5 cm guard. | `follow_face` stops on the median of the last 3 readings; one reading still stops below the 3.5 cm floor or at the IR's 10 cm minimum. |
| 05:46 success | Obstacle-1 IR fix `[-37.6, -2.5, -2.5, -1.6]` thrown away for one beam past the box edge. | `_view_line` drops readings >1.5 cm off the median and keeps the fix when most agree. |
| 05:46 success | Parked 6 cm off centre: "keeps 15.0 cm to back fence, so Line C is skipped". The back fence (always 15 cm, the planned stop) capped the clearance check. | Side fences only in that check, plus centring whenever the IRs say > `PARK_CENTRE_TOL_CM` (4 cm). No centring line fitting is no longer fatal if the straight entry is safe. |

**Aim at obstacle 2's centre** (`T2MN_AIM_OBS2_MAX_DEG`, **off** by default): ends the
stage-2 view with one arc pointing the camera at the arrow. Simulated over 108 layouts it
made 80 of them 20–97 cm longer (every route round obstacle 2 from the lane starts by
turning outward, so the arc is undone) and none shorter by more than 4 cm. When on and no
route fits from the aimed pose, only the arc is backed out before the full retrace.

Simulator, 108 layouts × 4 arrow pairs: 108/108 still park, paths unchanged. With 2 cm/m
drift (77 parked, the same 31 fail before and after): median offset 3.2 → 0.3 cm, worst
10.4 → 4.2 cm, more than 4 cm off 29 → 2.

New `.env` settings: `T2MN_PARK_CENTRE_TOL_CM` (4), `T2MN_AIM_OBS2_MAX_DEG` (0 = off),
`T2MN_AIM_OBS2_EXTRA_CM` (16).
