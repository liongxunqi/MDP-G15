# MDP Task 1 — RPi Codebase

## File Structure

```
RPi/
├── .env                        ← all config lives here (IPs, ports, baud rate, timings)
├── task1.py                    ← main entry point, run this on the RPi
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
| `<filename>\n` | Image filename (sent right after DETECT) |
| `<raw bytes>END_IMAGE` | JPEG data with sentinel |
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
