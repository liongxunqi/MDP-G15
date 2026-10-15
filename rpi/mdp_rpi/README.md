# MDP Task 1 — RPi Codebase

## File Structure

```
RPi/
├── .env                        ← all config lives here (IPs, ports, baud rate, timings)
├── task1.py                    ← main entry point, run this on the RPi
├── task_a5.py                  ← checklist A.5: runs task1 with one obstacle
│                                 expanded to all four of its faces
├── task_a5_reactive.py         ← A.5 fallback: no Android, no planner
├── test_task_a5.py             ← offline test for A.5 mode (17)
├── test_task_a5_reactive.py    ← offline test for the fallback (24)
├── test_connection.py          ← smoke test: RPi ↔ PC TCP link
├── test_camera.py              ← smoke test: picamera2 capture
├── test_stm.py                 ← smoke test: interactive STM command sender
├── requirements_rpi.txt
│
├── communications/
│   ├── android.py              ← Bluetooth RFCOMM server (auto-reconnect)
│   ├── pc.py                   ← TCP socket server + image transfer
│   └── stm.py                  ← Serial UART to STM32
│
├── image_capture/
│   └── camera.py               ← picamera2 wrapper (no blocking input() calls)
│
└── run_log.py                  ← per-run log files + end-of-run summary
                                  (also used by pc/task1_pc.py)
```

The PC side is not in this folder. It lives at the repo root:

```
pc/                             ← run these on the PC, not the RPi
├── task1_pc.py                 ← PC server: pathfinding + YOLO detection
├── image_recognition/detect.py
├── requirements_pc.txt
└── weights/best.pt             ← put your trained YOLO weights here
algo/                           ← the path planner task1_pc.py calls
├── path_planner.py, grid_search.py, stm_tokens.py
└── tests/                      ← unit tests + test_mission_sim.py
```

---

## One-time RPi Setup

```bash
# 1. Bluetooth
sudo apt install python3-bluetooth bluez
sudo systemctl enable bluetooth && sudo systemctl start bluetooth

# 2. Camera
sudo apt install python3-picamera2

# 3. Python packages
pip install -r requirements_rpi.txt

# 4. Edit .env — set your RPi's IP
nano .env
```

## One-time PC Setup

```bash
cd pc            # from the repo root
pip install -r requirements_pc.txt
# Copy your trained weights:
mkdir weights && cp /path/to/best.pt weights/
# Edit RPI_IP in task1_pc.py to match your RPi's IP
```

---

## Testing (step by step, no full hardware needed)

### Step 1 — Test TCP connection

On RPi:
```bash
python3 test_connection.py --mode server
```

On PC:
```bash
python3 test_connection.py --mode client --ip <RPi IP>
```

Expected: both sides print "Test passed ✓"

---

### Step 2 — Test camera (RPi only)

```bash
python3 test_camera.py
```

Expected: prints the saved image path in `captured_images/`

---

### Step 3 — Test STM (RPi only, STM physically connected)

```bash
python3 test_stm.py
```

Type `F10` and press Enter. The car should move forward ~10 cm, and `OK` arrives
**after it stops** — the reply comes when the move finishes, not when it starts.
Type `q` to quit.

To prove the link without moving the robot at all:

```bash
python3 test_stm.py --bringup
```

That runs stages 3–5 of the bring-up ladder in `PROTOCOL.md` §9 (`S` → `OK`,
`XYZ` → `RESEND`, `?VER` → `VER,...`). If those pass and `F10` then fails, the
problem is motion, not the link.

---

### Step 4 — Test YOLO detection (PC only, no RPi needed)

```python
# Run from pc/
from task1_pc import run_detection
class_id, conf = run_detection("path/to/test_image.jpg")
print(class_id, conf)
```

---

### Step 5 — Full Task 1 end-to-end

Order of startup matters:

1. **RPi:** `python3 task1.py`  ← **must start first.** The RPi is the TCP
   *server*; it binds, listens, and blocks on `accept()`
2. **PC:** `python3 pc/task1_pc.py` (from the repo root) — it *connects*, with no retry,
   so starting it first just gets connection refused
3. **Android:** connect via Bluetooth, send obstacles, press Send Data, press Begin

---

## Checklist A.5 — navigate around the obstacle

> *"Navigate towards a given obstacle having a visual marker… navigate around
> the obstacle in search of face which has a valid image from the image list."*

**A.5 is Task 1 with one thing removed: you do not know which face the image is
on.** That is the only difference, and it is the thing Task 1 is built around
knowing — the operator taps the face in on Android, it arrives as `d`, and
`path_planner._viewing_pose()` turns it into the pose to photograph from.

So A.5 reduces to Task 1 exactly. Declare the obstacle **four times, once per
face**, and one unknown-face problem becomes four known-face problems. The
planner does the orbit: four viewing poses 30 cm off each face, a shortest tour
between them, an `FU30` to measure the final standoff, and one `DETECT` at
each. Planned and collision-checked, rather than dead-reckoned.

### Running it

```bash
# 1. RPi — FIRST. It is the TCP server and blocks waiting for the PC.
python3 task_a5.py

# 2. PC — connects to the RPi. No retry, so it must come second.
python3 pc/task1_pc.py          # from the repo root

# 3. Android — place ONE obstacle, Send Data, Begin. Exactly as Task 1.
```

The face you tap on Android is **ignored**. Not knowing it is what A.5 tests,
so acting on a guess would photograph one face and call the task done.

### What A.5 mode changes

`task_a5.py` is a thin entry point; the three changes live in `task1.py` behind
`a5_mode`, so Task 1 itself is untouched.

| | |
|---|---|
| **Fan out** | One obstacle becomes four on the way to the PC, ids `base*10 + d`. Android is never told |
| **Fold back** | The winning `TARGET` is reported under the id Android actually knows |
| **Filter** | Only a valid image is forwarded, and only the first one |

The filter is not tidiness. Three of the four replies are bullseyes or blanks —
that is the search working — and every fanned id folds back to the *same*
obstacle, where `ArenaReducer.applyTarget()` is last-write-wins. Forward them
all and a bullseye arriving after the real image replaces it on the tablet,
while the Pi's own log still cheerfully reports the right answer.

### How much room it leaves

`OBSTACLE_SIZE_MM` is **assumed, not measured** — 100 mm from the course spec.
Nothing on the robot can measure a block, so that number is an input, not an
observation.

It matters in two places that behave completely differently:

| Use | Self-correcting? |
|---|---|
| Where the viewing pose goes | **Yes** — the approach ends on `FU30`, which reads the real gap and drives that, so the robot stops 30 cm from whatever is actually there |
| How much room the path leaves going past | **No** — open-loop, nothing watching |

`OBSTACLE_CLEARANCE_MM` (20 mm) exists so the second can be tuned without
moving the first. Before it was split out there was about **2 mm** of margin:
the virtual box keeps the robot's centre 220 mm from the block centre, and the
real robot's worst-case half-extent is ~148 mm at a corner, so the whole
allowance went on the robot's own corners. An obstacle wider than 10 cm was
clipped by the excess, and the first sign would be a stalled wheel and
`FAIL,TIMEOUT` fifteen seconds later.

**Sensors cannot cover this gap today.** The ultrasonic is front-facing and
during an orbit the block is to the *side*. The side IRs point the right way
but saturate below 10 cm — 5 cm and 8 cm both read ~9 cm (`ir.h`) — which is
exactly the range a clip happens in, and `FIR`/`FIL`, which would act on them,
are reserved and unimplemented. So the margin is bought up front rather than
sensed.

Clearance costs orbit radius one-for-one: +20 mm here is +20 mm of swing per
face. Nothing on a lone central block, worth re-checking before an
eight-obstacle arena.

### What it does not do

**Stop early.** The path is planned in full before the first photo, so all four
faces get visited even when the image is on the first one. The answer is
latched on the first valid image and later faces cannot overwrite it, so this
costs time, not correctness.

### The standalone fallback

`task_a5_reactive.py` is the earlier version: Pi + STM + PC detection, **no
Android and no planner**. It approaches on `FU`, and if the face is wrong it
drives a fixed five-token orbit (`A5_ORBIT` in `.env`) and looks again.

That orbit is dead-reckoned and has never been driven on a floor, which is why
it is the fallback rather than the primary — but it is the one to reach for
when Bluetooth will not pair on the day.

```bash
python3 task_a5_reactive.py --no-detect   # motion only: tune A5_ORBIT
python3 task_a5_reactive.py --dry-run     # no motion: prove camera + PC
```

### Offline tests

```bash
python3 test_task_a5.py                  # 17 tests — fan-out, fold-back, filter
python3 test_task_a5_reactive.py         # 24 tests — the standalone sequence
python3 ../../algo/tests/test_path_planner.py    # 16 tests — planner
python3 ../../algo/tests/test_mission_sim.py     # replays whole layouts
```

Neither needs a robot, a PC, Bluetooth or pyserial. Geometry is not covered by
either: the planner owns it in the primary path, and in the fallback it is what
`--no-detect` and a tape measure are for.

---

## Message Protocol Reference

### Android → RPi (Bluetooth)
| Message | Meaning |
|---|---|
| `OBSTACLE,<id>,<x*10>,<y*10>,<NORTH/EAST/SOUTH/WEST/SKIP>` | Add obstacle |
| `CLEAR` | Clear all obstacles |
| `PATH` or `ALG\|…` | Send Data button pressed — trigger pathfinding |
| `BEGIN` | Start button pressed — execute path |

### RPi → Android (Bluetooth)
| Message | Meaning |
|---|---|
| `STATUS,CONNECTED TO RPI` | Sent on Bluetooth connect |
| `TARGET,<obstacle_id>,<class_id>` | Detection result |
| `STATUS,DONE` | All obstacles visited |

### RPi → PC (TCP)
| Message | Meaning |
|---|---|
| `OBSTACLES,<json>` | List of obstacles for pathfinding |
| `DETECT,<obstacle_id>` | About to send image for this obstacle |
| `<4-byte big-endian length>` | Size of the JPEG that follows, sent right after DETECT |
| `<raw JPEG bytes>` | Exactly that many bytes. No sentinel, no filename |
| `STITCH,<n>` | All done, stitch n+1 images |

### PC → RPi (TCP)
| Message | Meaning |
|---|---|
| `PATH,<json>` | Pathfinding result with segments and obstacle order |
| `OBJECT,<obstacle_id>,<confidence>,<class_id>` | Detection result |

### STM → RPi (Serial)
| Message | Meaning |
|---|---|
| `OK` | The whole **line** completed, ready for next |
| `RESEND` | Parse error — nothing executed, safe to retransmit (cap at 3) |
| `FAIL,TIMEOUT` | Watchdog fired mid-move: wheel stalled or encoder dead |
| `FAIL,WRONGWAY` | Arc rotated away from target and was aborted |
| `FAIL,NOECHO` | `FU<n>` had no usable ultrasound reading — **nothing moved** |
| `<TAG>,<fields>` | Answer to a query, e.g. `US,42` — no separate `OK` |

`FAIL,*` means **the robot is not where you think it is** — stop and re-plan.

`FAIL,NOECHO` is the one exception, and it is worth knowing. `FU` refuses to
drive at something it cannot see, so the robot did **not** move and the pose is
still exactly what it was. The firmware drops the rest of that line rather than
run it from the wrong place, so the plan is still stale — but the thing to go
and look at is the ultrasound, not the wheels, and a retry can start from where
the robot is standing instead of re-localising.

### RPi → STM (Serial)

Variable-length comma-separated tokens, newline-terminated, **one reply per
line**: `FR90,F20,S\n` → one `OK` when the whole line finishes.

| Token | Meaning |
|---|---|
| `F<n>` | Forward n cm (`F0` = forward until obstacle, a fixed 15 cm trip-wire) |
| `FU<n>` | Forward until the ultrasound reads **n** cm — n is yours to pick, 5–200 |
| `R<n>` | Reverse n cm |
| `FR<n>` / `FL<n>` | Arc forward-right / forward-left, n degrees |
| `RR<n>` / `RL<n>` | Arc reverse-right / reverse-left |
| `S` | Stop — brake and recentre. **Not backward**; backward is `R<n>` |
| `RST` | Emergency abort — replies **nothing at all**, never wait for it |

Queries (`?US`, `?IR`, `?POSE`, `?STAT`, `?VER`, …) are single-token lines and
are answered immediately, even mid-move. Config is `!PROF0/1/2` and `!ZERO`.

> The superseded 4-character dialect (`W050`/`S050`/`D100`/`A100`) is rejected
> by the firmware. Note the trap: `S050` used to mean reverse, and `S` now means
> **stop**.

#### Standing off at a chosen distance — `FU<n>`

`F<n>` is pure odometry: `F50` drives 50 cm and **ignores the ultrasound
entirely**, obstacle or not. `F0` does watch it, but only as a trip-wire at a
fixed 15 cm — and it trips on a reading that is already about 120 ms old, which
at cruise is roughly 4 cm, and a *different* 4 cm each run. Measured overshoot:
3.9–5.3 cm.

`FU<n>` is the one to use when the standoff matters. It does not trip on the
sensor at all — it stands still to **measure**, drives the gap on **odometry**,
then re-measures and corrects, up to four passes. It lands within ±2 cm.

```
FU30            put me 30 cm off whatever is in front
F150,FU20       cover the distance fast, then close the last bit precisely
```

Three things that surprise people:

* **It is not a distance cap.** `FU30` with the wall 180 cm away drives 150 cm.
  It closes whatever gap it measures, anywhere in 5–200 cm. If you want "go at
  most 50 cm", that decision belongs here on the Pi — ask `?US` first.
* **It needs something to see.** Nothing in range is `FAIL,NOECHO` and the
  robot does not move. It will not creep forward hopefully.
* **It costs time.** Each pass pays a 400 ms stationary settle: roughly 1.2 s
  for a short approach, 4 s from a metre, 8.6 s worst case. Do not read a long
  silence as a dead link — the read timeout is 20 s for exactly this reason.

It is also **precise but not accurate**: the sensor reads about 1.3 cm long and
every pass reads through that same offset, so it cannot average out. `FU20`
settles around 18.7 cm from the obstacle, repeatably. Ask for `FU21`, or use
`algo/stm_tokens.fwd_until(20, compensate=True)` — and re-measure the offset on your
own sensor before trusting it.

Confirm with `?US`, not `?DIST`: `?DIST` reports only the last leg of the
approach, which after a correction pass is a couple of centimetres.

**Never send the next line until the previous one is answered.** A line arriving
mid-execution is appended to the same queue and you get one reply for two lines
— permanently one reply out of step, silently.

Full spec: `PROTOCOL.md` at the root of the firmware repo, which is
authoritative wherever it disagrees with this table.

---

## Tuning via .env (no code changes needed)

| Variable | Default | Effect |
|---|---|---|
| `SEGMENT_DELAY_S` | 0.5 | Pause between segments (give STM time to settle) |
| `DETECT_RETRY_COUNT` | 1 | Extra attempts if PC doesn't reply in time |
| `DETECT_RETRY_DELAY_S` | 0.2 | Wait between retries |
| `DETECT_TIMEOUT_S` | 30.0 | How long to wait for the PC's YOLO OBJECT reply per attempt |
| `TASK1_CAPTURE_SETTLE_S` | 2.0 | Chassis settling time before each Task 1 photograph |
| `DEBOUNCE_DELAY_S` | 1.0 | How long after last obstacle before auto-sending to PC |
| `PATH_TIMEOUT_S` | 30.0 | Give up waiting for the PC's `PATH` and report `STATUS,FAILED` |
| `TASK1_FEEDBACK_WAIT` | 0 | Set to 1 to require a PC decision after every instruction |
| `TASK1_FEEDBACK_TIMEOUT_S` | 30.0 | Positive finite seconds to wait for a matching decision |

A.5 only (`task_a5.py`):

| Variable | Default | Effect |
|---|---|---|
| `A5_STANDOFF_CM` | 25 | Camera standoff. Sent as `FU(standoff + 1.3)` |
| `A5_MAX_APPROACH_CM` | 150 | Further than this is a wall, not the obstacle |
| `A5_MAX_FACES` | 4 | A block has four; a fifth leg returns you to the first |
| `A5_MIN_CONFIDENCE` | 0.55 | Second gate, on top of the 0.25 floor in `detect.py` |
| `A5_SETTLE_S` | 0.3 | Let the chassis stop rocking before `FU` measures |
| `A5_ARC_PROFILE` | 0 | Profile `A5_ORBIT` was traced for. Checked against `?STAT`, never set |
| `A5_ORBIT` | `R20,FR90,FL90,R37,FL90` | One face to the next — **the thing you tune** |

## Task 1 PC Feedback Contract

The RPi and `pc/task1_pc.py` implement this handshake. With
`TASK1_FEEDBACK_WAIT=1`, the PC answers `CONTINUE` while measured pose remains
within its configured limits. Outside them it plans a collision-checked recovery
to the current segment's original endpoint, sends that correction plus the
unchanged later segments as `REPLACE`, and resets its expected-pose monitor to
the replacement. If recovery is unsafe, or no valid reply arrives within
`TASK1_FEEDBACK_TIMEOUT_S`, the RPi continues the existing path. The IP, port,
Android messages and image framing are unchanged.

After each STM `OK`, RPi queries `?WPOSE`, sends Android `ROBOT,x,y,heading`,
then sends PC one newline-terminated `PROGRESS,<json>` message. With wait mode
enabled, no next movement, photo capture or DONE is issued before the decision.
The PC receive thread remains available while the STM execution thread waits.

Example JSON (transmit on ONE line after `PROGRESS,`):

```json
{
  "event": "instruction_completed",
  "segment_index": 0,
  "instruction_index": 0,
  "token": "FR90",
  "segment": ["FR90", "F15", "S"],
  "segment_completed": false,
  "motion_plan_completed": false,
  "next_segment_index": 0,
  "next_instruction_index": 1,
  "x_grid": 2.75,
  "y_grid": 3.25,
  "heading_deg": 92.6,
  "remaining_photo_ids": ["7"],
  "awaiting_decision": true,
  "decision_timeout_s": 30.0,
  "feedback_id": "opaque-unique-value"
}
```

- Indexes are zero-based within the currently installed path. `token` has
  already completed. `segment` includes both executed and unexecuted tokens.
- Coordinates are fractional grid units: one unit = 100 mm; arena `[0,20)` on
  each axis. Heading is degrees, north=0, clockwise positive, `[0,360)`.
- `PATH.start` is only the bottom-left cell of Android's 2 x 2 robot drawing.
  The starting x/y/bearing for `ROBOT` and `PROGRESS` comes from the separate
  `PATH.odometry_start` object. The planner sends that origin explicitly; the
  RPi has no fixed `(2,2)` fallback. STM odometry measures the rear-axle
  midpoint, so the planner owns any required reference-point conversion.
- In telemetry-only mode `awaiting_decision` is false and no feedback ID is
  issued. Do not send decisions in that mode.
- Echo the feedback ID and completed indexes exactly. A fresh feedback ID is
  issued for EVERY wait, even after replacements and new runs. It is a request
  correlation ID, not an execution ID. Do not infer or reuse it.

The PC may send exactly ONE of these UTF-8 newline-terminated responses:

```text
CONTINUE,{"feedback_id":"opaque-unique-value","segment_index":0,"instruction_index":0}
REPLACE,{"feedback_id":"opaque-unique-value","segment_index":0,"instruction_index":0,"segments":[["F5","S"]],"segment_obstacles":["7"]}
```

### Decision Semantics

- `CONTINUE`: preserve the current path; perform any due photo, then send the
  next instruction. A final CONTINUE allows the final photo, STITCH and DONE.
- `REPLACE`: supply the ENTIRE remaining route from the reported measured pose,
  not a delta, not the old completed prefix. `segments` must be nonempty lists
  of valid STM movement tokens or `S`. Queries, resets and config commands are
  rejected. Existing STM line/token caps apply. `segment_obstacles` is required,
  parallel to `segments`, with null for travel or a string/integer obstacle ID.
  Non-null IDs must match `remaining_photo_ids` as a multiset (same counts;
  reordering is allowed). Omitted targets, duplicates and unknown IDs halt.
  RPi validates before swapping any path state, retains the original odometry
  origin and completed-photo count, resets path indexes to zero and immediately
  executes the first replacement instruction. Do not send a separate CONTINUE.
  `dirs` and `obstacle_ids` are not required or used for replacement.
- At a completed photo boundary, PROGRESS is sent BEFORE the photo. REPLACE
  defers that photo to its assignment in the replacement route. To photograph
  at the current position, a replacement segment `["S"]` with that obstacle ID
  is valid. Its completion will require a new decision. Completed photo
  assignments from earlier boundaries must not be reintroduced.
Missing replies, including a disconnected PC, cause the RPi to continue the
existing route after `TASK1_FEEDBACK_TIMEOUT_S`. `STOP` is not part of this
protocol and is ignored. Unknown IDs, duplicate/expired replies and malformed
JSON are ignored without extending the deadline. A matching ID with wrong
indexes or an invalid replacement still halts because executing an ambiguous or
malformed replacement is unsafe. Late decisions are stale and cannot alter the
route after fallback continuation. A feedback-mode validation halt invalidates
the old path: request a fresh PATH before restarting. A new BEGIN is an operator
restart from the configured physical start, not a resume from the last pose;
reposition/reset deliberately.

The RPi validates protocol and pending photo assignments, NOT collision safety
or physical reachability. The algorithm owns path geometry, non-cardinal
heading support and deciding whether a measured correction is appropriate.
Raw `PATH` messages remain rejected during active execution; use REPLACE.

Offline verification:

```bash
python3 -B -m unittest rpi/mdp_rpi/test_instruction_mission.py rpi/mdp_rpi/test_feedback_control.py rpi/mdp_rpi/test_task_a5.py -q
```

## Autonomous Task 2

Run `python3 task2.py` from `rpi/mdp_rpi` on the Pi and
`python3 task2_pc.py --host 192.168.15.15` from `pc` on the laptop.
Do not run Task 1 on the same sockets at the same time. Wait for the Pi's
READY log, then send the existing Android `BEGIN` command. The Android Task 2
screen now sends `TASK2,STATUS?` on connection/attempt preparation and enables
Start only after `TASK2,READY`. The Pi also announces `TASK2,READY` when its
camera and PC are ready; an early `BEGIN` is rejected with `TASK2,NOT_READY`.
Android `s`/`STOP` brakes
the robot. The main entry point is `task2.py`; `task2t.py` and
`task2t_park.py` supply Task 2 route/parking geometry. The previous manual
SAFE entry point is retained as `task2_safe.py` for bench comparison.

- Startup loads `T2T_CAL_PROFILE=tbricks`, selects TIGHT/profile 0, verifies
  calibration readback, zeros heading, and queries WPOSE, IMU, US and IR.
  The route uses `T2T_RADIUS_CM=30.0` and `T2T_FAST=1`. Startup sends
  `!FAST1` and refuses to run if the firmware does not acknowledge it, so the
  robot cannot silently fall back to normal speed. Precision measurement around
  obstacle 2 temporarily uses `!FAST0`; shutdown restores normal mode.
- One mission thread owns STM movement. Separate workers receive Android
  commands, continuously capture images, exchange PC recognition messages,
  and monitor camera freshness/the 175-second mission deadline.
- The camera keeps only its latest frame at 960x540 with a configurable
  5000-microsecond shutter. Capture and inference overlap approach movement;
  there is no fixed two-second per-photo wait. If an arrow is not confirmed
  at the decision point, movement stays stopped while fresh bullseye/blank
  frames arrive and waits for a valid arrow. `T2_ARROW_TIMEOUT_S` is renewed
  by every fresh frame and fails only when the camera pipeline becomes inactive
  safely. Continuous capture does not remove STM's own settling logic.
- The PC must confirm two fresh, matching arrow frames. Run, obstacle and
  frame identifiers reject stale replies. Accepted annotations are stitched
  into `pc/runs/task2_<timestamp>/stitched_task2.jpg` before turning is allowed.
  Use `--left-label`/`--right-label` if the model labels are not `left`/`right`.
- Android receives its existing valid status grammar; detailed Task 2 phases
  are text-log messages. No new Android start command or Task 1 planner is
  needed. Both programs write terminal and timestamped file logs.
- After obstacle 2 is recognized, skip the remaining photo positioning and
  read fresh WPOSE and plan the complete approach plus turn before moving.
  There is no separate F30 advance that can consume the turning space before
  a route is validated. Candidate joins are checked first to reject impossible
  routes quickly; accepted routes still undergo the complete swept-footprint
  check. Matching arrows try an all-forward
  outward lane shift, straight, and turn behind the obstacle; opposite arrows
  try a diagonal approach before the existing front-face route. Every accepted
  candidate retains the footprint/wall checks. Same-direction obstacle-2
  handoffs no longer use the reverse fallback: if no checked forward route
  exists, the manoeuvre fails without sending an unchecked movement.
  A forward front-face approach is also searched. When selected, the robot
  tracks the face in precision mode using side IR, requires two sightings
  and three misses to confirm its end, and checks front US on each poll.
  US at 20 cm or less, an invalid US reading, or missing pose stops this scan;
  none of these is accepted as an edge. The subsequent corner route is
  collision-checked and forward-only. This restriction applies to the
  same-direction obstacle-2 approach/corner, not the separate parking routine.
  Stage-2 vision gets up to 0.3 seconds to finish an in-flight lane frame
  before starting a photo manoeuvre. A confirmed arrow preserves that earlier
  pose. When no direct route fits, a parallel robot can try one fully checked
  outward sensing-lane shift. Two consistent US readings must still match
  obstacle 2's front before the new lateral hit extends its known face bound.
  Missing/inconsistent echoes stop recovery; they do not imply a narrower box.
  If IR has already located the near end, the planner first tries a simple
  forward straight and inward turn, without an unnecessary extra lane shift.
  The same-direction final turn home also excludes reverse distances.
  Late detections and uncertain intermediate obstacle lengths can still
  leave no checked forward route; these changes do not guarantee every layout.
  If the original candidates fail after vision interrupts a photo turn,
  headings up to 45 degrees off parallel also get a forward alignment-arc
  candidate. The alignment and remaining route are checked together before
  either is sent. This does not guarantee an all-forward route around an
  obstacle whose width is still unknown.
  Forward candidates across all approach/alignment alternatives are offered
  before reverse fallbacks. Duplicate candidates are removed and adjacent
  forward distances in a handoff are combined before full sweep validation,
  avoiding redundant straight stop/start commands without blending turns.
- `T2_HANDOFF_PLAN_BUDGET_S` defaults to 2 seconds. Handoff logs include search
  time, candidate count, selected commands, and clearance. Expiry does not
  authorize an unchecked movement; it uses an already fully checked candidate
  if available, otherwise reports failure. This is a planning
  budget, not a fixed sleep or a guarantee of Raspberry Pi execution speed.
- Obstacle-2 turns prefer `T2T_TURN_CLEARANCE_CM=8` cm body clearance;
  front-face IR distance now targets `T2T_FACE_GAP_CM=22` cm. After finding
  a valid handoff the search checks at most 16 more candidates for a wider
  option. Constrained routes can still use the existing 3.5 cm floor and log
  that fallback. These values do not change the calibrated turning radius.
- Face scans take two stationary IR samples before moving. If the movement
  finishes with a confirmed face but incomplete edge evidence, at most three
  further samples are taken at rest. These add 60 ms before a scan and at most
  90 ms afterwards, plus serial query time; no extra travel is commanded.
  The existing range, two-sighting/three-miss and wall-guard checks remain.
- Transfers around obstacle 2 use fast mode; side-IR edge acquisition stays
  at precision speed. The parking runner selects precision speed at the bay
  mouth. Success requires a stationary, home-facing robot fully inside the
  bay with valid sensor clearance (2 cm margin), not exact centring or a
  specific 12 cm front gap. The 12 cm gap remains the parking aiming point.
- When front US misses a narrow obstacle, the side-IR scan brakes at its first
  plausible sighting and requires two consistent readings at rest. Normal
  confirmation waits 0.10 seconds after braking plus two 0.03-second sample
  intervals, in addition to serial latency. It retains the first moving pose
  for the edge estimate. Failed confirmation resumes only the remaining scan,
  with at most three candidates and four confirmation queries per candidate;
  an unconfirmed reading is never accepted as the obstacle edge.

Bench validation is still required. `tbricks.json` is marked unverified and
its measured radii differ between directions. The imported route currently
models obstacle 2 widths of 30-90 cm, not every possible legal width. Camera,
US and IR mounting offsets must match the robot: `T2_CAMERA_FROM_AXLE_CM=12.5`
is a provisional camera offset, not the rear-axle-to-ultrasonic measurement.
Test video-port capture and lighting on the Pi before a floor run. Nominal
simulation, side IR and front US do not guarantee collision-free movement
with odometry drift or objects outside the sensors' fields of view.
The fault tests deliberately expect a safe failure for the tightest course
with a 2 cm radius error plus an 11 cm short ultrasonic reading: its IR
reading is indistinguishable from known empty-space output. Likewise, a
measured pose already below the 3.5 cm wall-clearance floor is not labelled
a safe parking start. Those are rejected cases, not successful completions.

Offline coordination and autonomous route tests (from `rpi/mdp_rpi`):

```bash
python3 -m unittest test_task2_integration -v
```
