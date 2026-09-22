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
└── pc_side/                    ← run these on the PC, not the RPi
    ├── task1_pc.py             ← PC server: pathfinding + YOLO detection
    ├── pc_config.py            ← PC-side config
    ├── requirements_pc.txt
    └── weights/
        └── best.pt             ← put your trained YOLO weights here
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
cd pc_side
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
# Run from pc_side/
from task1_pc import run_detection
class_id, conf = run_detection("path/to/test_image.jpg")
print(class_id, conf)
```

---

### Step 5 — Full Task 1 end-to-end

Order of startup matters:

1. **PC:** `cd pc_side && python3 task1_pc.py`  ← must start first, waits for RPi
2. **RPi:** `python3 task1.py`
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
# PC  — start first, same as Task 1
cd pc_side && python3 task1_pc.py

# RPi
python3 task_a5.py

# Android — place ONE obstacle, Send Data, Begin. Exactly as Task 1.
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
cd pc_side && python3 test_path_planner.py   # 10 tests — the tour search
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
`stm_tokens.fwd_until(20, compensate=True)` — and re-measure the offset on your
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
| `DETECT_TIMEOUT_S` | 0.5 | How long to wait for OBJECT reply per attempt |
| `DEBOUNCE_DELAY_S` | 1.0 | How long after last obstacle before auto-sending to PC |
| `PATH_TIMEOUT_S` | 30.0 | Give up waiting for the PC's `PATH` and report `STATUS,FAILED` |

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
