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

Type `W050` and press Enter. The car should move forward.
Type `q` to quit.

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
| `OK` | Command completed, ready for next |
| `RESEND` | Error, please retransmit last command |

### RPi → STM (Serial)
4-character commands: `W050` (forward 50), `S050` (back 50), `D100` (right), `A100` (left)

---

## Tuning via .env (no code changes needed)

| Variable | Default | Effect |
|---|---|---|
| `SEGMENT_DELAY_S` | 0.5 | Pause between segments (give STM time to settle) |
| `DETECT_RETRY_COUNT` | 1 | Extra attempts if PC doesn't reply in time |
| `DETECT_RETRY_DELAY_S` | 0.2 | Wait between retries |
| `DETECT_TIMEOUT_S` | 0.5 | How long to wait for OBJECT reply per attempt |
| `DEBOUNCE_DELAY_S` | 1.0 | How long after last obstacle before auto-sending to PC |
