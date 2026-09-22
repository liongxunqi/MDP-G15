# STM32 Robot Firmware — MDP Group 15

Firmware for the robot-hardware half of the MDP system. Runs on the WHEELTEC
STM32F407VET6 **C30D-V2.1** controller board (schematic rev 23.0).

**The chassis is Ackermann.** Two driven rear wheels, one steering servo on the
front axle. **It cannot turn on the spot.** Every turn is an arc. If you are on
the algorithm or RPi team, that one fact will shape more of your work than
anything else in this document — see [What other teams need to know](#what-other-teams-need-to-know).

---

## Status

| Checklist item | State |
|---|---|
| **A.3** Straight line, 80–120 cm, ±6%, no visible deviation | **Passing** — 0.0–0.3% error at 1000 and 1200 mm |
| **A.4** Rotation, 90–360° | **Passing** — within ±1° on the first turn of a cold session |
| **A.1** RPi ↔ STM over USB→Serial | STM side complete and unit-tested, **never tested on the wire** |
| **A.5** Navigate around obstacle | Firmware side complete — the navigation lives on the Pi in `rpi/mdp_rpi/task_a5.py` and drives `?US` + `FU<n>` + arcs. **Never run on the floor** |

Sensors: IR calibrated per channel, ultrasonic verified against a tape. Motors
C and D are not implemented.

---

## Quick start

### Build

STM32CubeIDE project. Import the folder and build — no external dependencies.
Compiles clean with `-Wall -Wextra`.

Toolchain used: GNU Arm 10.3-2021.10 (bundled with CubeIDE 1.9.0), hard FPU
(`fpv4-sp-d16`).

### Flash

Two routes:

- **ST-LINK/V2 over SWD** (in the kit) — Red 3.3 V, Black SWCLK, Blue GND,
  Yellow SWDIO. This is the easier path and supports debugging.
- **FlyMcu V0.2188 on USB Port 1** — set "Reset@DTR Low, ISP@RTS High", 115200.

### Power up — read this before you plug anything in

1. **12 V switch (SW4) OFF**, battery unplugged
2. Servo unplugged from J7 **if flashing over USB Port 1** — it draws high
   transient current when USB is inserted, and that is a documented way to kill
   the board
3. Flash
4. **Unplug USB from Port 1**
5. Reconnect servo, then switch SW4 on

**Never have USB in Port 1 and battery power on at the same time.**

Anything else you plug or unplug — sensors, jumpers — gets done with SW4 off.

### The 12 V trap

**The board must be on 12 V at SW4.** On USB alone the MCU, OLED and encoders
all work perfectly while the motors and servo are dead, because the 5V5 rail
that feeds them comes from a regulator on the 12 V input. It looks exactly like
a broken timer configuration. This has cost time twice.

---

## The three USB-C ports

They are not interchangeable.

| Port | Location | Wired to | Use |
|---|---|---|---|
| **1** | Under the OLED, nearest the potentiometer | UART1, PA9/PA10 | Code download (FlyMcu). **This firmware never transmits here.** |
| **2** | Next along | **UART3, PD8/PD9** | **The RPi link.** All firmware output appears here. |
| **3** | — | 5 V rail | Power supply to the RPi. No data. |

If you open a terminal and see nothing, you are almost certainly on Port 1.

---

## Button UI

One button (PE0). **Long press** (600 ms) cycles modes and aborts whatever is
moving. **Short press** acts within the current mode.

| Mode | Name | Short press does |
|---|---|---|
| 1 | **CALIB** | Drives one PROTOCOL.md §7 calibration cycle — a straight, then four arcs — and shows what it changed. Press again to stop it, or to run another |
| 2 | **DRIVE** | Runs the A.3 distance. Holds the result: target, odometry, error %, B/A encoder agreement |
| 3 | **SETDIST** | Steps target 800→1200 mm in 100 mm steps |
| 4 | **TURN** | Runs the A.4 turn. Holds the result: commanded, measured, error, radius, cross-check |
| 5 | **SETANGLE** | Steps angle 90/180/270/360, then direction (R/L, fwd/rev) |
| 6 | **PROFILE** | Cycles arc profiles. Shows learned decel and brake lag |
| 7 | **SERVO** | End-stop sweep, +25 µs per press. Auto-centres 2 s after the last press |
| 8 | **SENSE** | Live IR and ultrasonic distances, echo count, UART re-arm count |
| 9 | **IRCAL** | Median-filtered raw ADC counts, for fitting the IR curve |
| 10 | **IMU** | Gyro diagnostics: heading, rate, poll rate, stalls, peak raw |
| 11 | **CAL** | What `?CAL` would answer right now, in the integers the wire carries, plus a tally of what the `!CAL*` setters have done. Short press zeroes the tally |

**The board powers up in mode 2, not mode 1.** A stray press after power-on
should cost one straight run, not a five-move sequence nobody is standing next
to. Long-press once to reach CALIB.

Modes 8 and 9: short press also dumps a sample to USART3.

### Mode 1 — the calibration cycle

One press drives the `PROTOCOL.md` §7 sequence: an 800 mm straight, then four
90° arcs alternating forward and reverse on the same steering side, so the
robot retraces its own arc and stays on one patch of floor.

The straight is 800 mm because the trim learner discards the first 0.5 s as
launch transient and then wants 0.5 s of steady driving — about a second of
holding once the ramps are paid for. A shorter one teaches it nothing **and
says nothing**, which is why the screen prints `SKIP` rather than a trim that
merely did not move.

**It does not learn anything.** The learning already happens at the end of
every move inside `motion.c` — `Odom_LearnTrim()` after a straight, the decel
and lag update after an arc — and it does not care whether the move came from
this button or off the wire. The mode only drives §7's moves in §7's order.
That is why this is a sequencer and not a calibration routine, and why it was
about eighty lines rather than a rewrite.

```
1 CALIB TIGHT        <- the profile you are calibrating. It is NOT set here
done  run 2          <- or "step 3/5 arc", or "READY  run 0"
dec  12340   +40     <- value, then what THIS cycle changed it by
lag  1500    -10
trm   +12     +3     <- the delta that matters most, or SKIP
```

**One press is one cycle, not the whole convergence.** Press, read the right-
hand column, press again, and stop when it stops moving — that is what
converged means, and a delta answers it without you having to remember the
previous numbers. Press again while it is running to stop it; a long press
aborts and leaves the mode.

With the current learning rules that should take **two cycles, not ten**: the
decel and lag take the first arc's measurement outright, and the trim is
measured from the steering loop's own output rather than searched for. If
`trm`'s delta is still the same size on the fourth cycle as the first,
something is wrong — see *How the learning works* below.

Three things worth knowing before you use it:

- **It does not select a profile.** The learned values belong to whichever
  profile was active when they were learned, so set mode 6 first if TIGHT is
  not what you will drive.
- **The reverse legs are unverified.** `motion.c` still marks the reverse-arc
  sign convention "VERIFY THIS ON THE ROBOT". Watch the first cycle. If the
  reverse legs go the wrong way, flip `g_calArcFwd` in `main.c` to all `1U` —
  `FR90/FL90` also returns the heading, it just walks the robot forward in an
  S and needs more floor.
- **The host always wins.** A command line arriving mid-cycle cancels the
  cycle. See the amendment under §7 rule 1 in `PROTOCOL.md`.

Useful right now for the open steering-centre question: run four or five
cycles and watch `trm` on the bottom two lines. Settling within a few µs of
zero means `SERVO_CENTER_US` is right; settling at a consistent non-zero value
means it is off by about that much, at 7.5 µs per cm over a metre.

Mode 7 warning: the sweep pushes the steering past its normal limits to find
where the linkage binds. **Watch the wheels, not the numbers** — the moment
they stop moving is the mechanical stop, and further steps only stall the
servo.

Mode 11 is the one to sit on while the Pi runs `calibrate.py`. Four lines:

```
11 CAL wire
dec  12340          <- decel_x10, exactly as ?CAL sends it
lag 1500 trm +12    <- lag_ms_x10, trim in whole us
ok 3 bsy 0 rg 0     <- what the !CAL* setters did
short = zero
```

`bsy` and `rg` are the reason it exists. A setter answers `RESEND` both when
the value was out of range and when a move was in flight and the firmware
refused to change the braking model underneath it — identical on the wire, and
the host has to disambiguate with `?STAT`. Here they are separate columns.

Values are shown **scaled as the wire carries them**, not converted, so they
can be compared against the host's log without arithmetic. Mode 6 has the same
decel and lag in real units; mode 2 has the trim after a run.

---

## RPi integration

### Wire format

```
RPi → STM :  <tok1>,<tok2>,...,<tokN>\n
STM → RPi :  OK\n        after the ENTIRE line has executed
             RESEND\n    on parse failure, nothing executed
```

- **USART3, PD8/PD9, USB Port 2, 115200 8N1**, no flow control
- Tokens separated by **comma or space** — both accepted
- Line terminated by `\n`; a preceding `\r` is tolerated and discarded
- Input is **lowercased on receive**, so sender casing is irrelevant
- **One reply per LINE, never per token**
- Parsing is **all-or-nothing**: one bad token and nothing from that line is
  queued
- `RST` is the sole exception — it aborts immediately and **replies nothing**

Example: `FR90,F20,S\n` → arc right 90°, forward 20 cm, stop, then `OK\n`

### Opcodes

| Token | Arg | Meaning |
|---|---|---|
| `F{n}` | cm | Forward n cm |
| `F0` | — | Forward until an obstacle stops it (ultrasonic, stops at 15 cm) |
| `FU{n}` | cm | Forward until the ultrasound reads n cm, **n chosen by the sender**, 5–200 |
| `R{n}` | cm | Reverse n cm |
| `FR{n}` | degrees | Arc forward-right through n degrees |
| `FL{n}` | degrees | Arc forward-left |
| `RR{n}` | degrees | Arc reverse-right |
| `RL{n}` | degrees | Arc reverse-left |
| `S` | — | Stop: brake motors, recentre servo. Replies `OK` |
| `RST` | — | Emergency abort: drop queue, brake now. **Replies nothing** |

Limits: 128 bytes per line, 16 primitives queued.

Reserved for Task 2 and **not yet implemented** — the parser returns
`CMD_INVALID` for these, which means the whole line gets `RESEND`:
`FIR`, `FIL`, `FIRO`, `FILO`, `SR`, `SL`.

### `FU{n}` measures, `F0` trips

Both stop the robot in front of an obstacle and they are not
interchangeable.

`F0` brakes the moment the ultrasound reads under 15 cm. The reading that
trips it is **about 120 ms old** — 60 ms ping period, median of three — and at
cruise the robot covers 345 mm/s, so it sails roughly 4 cm past, by a
different amount every run depending where in the ping cycle it arrived.

`FU{n}` never terminates on the sensor. It stops, waits 400 ms for the median
filter to refill with stationary samples, reads the gap, hands `gap - n` to
the same closed loop that holds a straight run to ±6%, then re-measures and
corrects. Up to four passes, landing within ±2 cm of `n`.

The trade is time: budget about 1.2 s for a short approach and 4 s for a
metre, most of it spent deliberately stationary. Use `F{n}` for the bulk of a
long journey and `FU{n}` for the last stretch — past about 2 m the beam has
spread wide enough that the nearest echo is often not the obstacle you meant,
which is why the argument is capped at 200 cm.

If there is no usable reading when the command starts, `FU` replies
`FAIL,NOECHO` and **does not move**. It never creeps forward looking for an
echo: driving at an obstacle it cannot see is the one thing a distance-keeping
command must not do. That also drops the rest of the line, so
`FU20,FR90` cannot turn from a position the sender never reached.

Full behaviour table in `PROTOCOL.md` §4.1.

### The console goes quiet when you connect

USART3 is both the bench console and the command link — there is one port and
the protocol owns it.

With no host connected, the firmware prints a startup banner, run reports and
sensor dumps. **The first time a valid command line parses off the wire, all of
that latches off permanently** (until reset), so the host sees only `OK` and
`RESEND`.

On the bench this means: connect the RPi and your terminal output stops. That
is the mechanism working. Reset the board with the host quiet to get it back.

### Open questions for whoever owns the RPi side

Three things need agreeing before the first wire test:

1. **Primitive timeout — now urgent, not theoretical.** `commands.h` declares
   8 s; the firmware enforces 15 s. If the RPi gives up at 8 s while the STM is
   still working, the two desynchronise and the next reply lands against the
   wrong command. `FU<n>` makes this actually bite: it is several primitives
   plus settles, and the longest legal approach (`FU5` from 2 m) measures
   **8.6 s** — already past 8 s. The timeout has to be sized off the **line**,
   not off one move. Pick one number, make both sides read it, and make it 20 s.
2. **`RST` reply.** This firmware replies nothing, per the spec. Last year's
   firmware actually echoed `RST\r\n`. Confirm which the RPi expects.
3. **Timeout semantics.** On a stalled primitive this firmware replies `OK` so
   the host is never left hanging. Last year replied `RESEND`, asking for a
   retry. `OK` claims a move succeeded when it did not; `RESEND` overloads a
   token defined as "parse failure". Neither is obviously right.

### Sensor hooks

`commands.h` declares four hooks. Two are implemented in `ultrasonic.c`:

| Hook | Status |
|---|---|
| `Sensors_FrontDistanceCm()` | Implemented — median-filtered ultrasonic |
| `Sensors_ObstacleAhead(stop_cm)` | Implemented — **returns "obstacle" when there is no reading**, which is fail-safe but means `F0` refuses to move whenever the echo is lost |
| `Sensors_HeadingDeg()` | **Weak stub, returns 0.0** |
| `Sensors_HeadingValid()` | **Weak stub, returns 0** |

Strong definitions override the weak stubs at link time — no `#ifdef`, no
coordination needed.

---

## What other teams need to know

**The robot cannot turn on the spot.** Plan paths accordingly.

**Measured turn radii**, by the floor-chord method:

| Profile | diff_boost | Radius |
|---|---|---|
| TIGHT | 2.0 | **291 mm** |
| SLOW | 1.5 | 306 mm |
| CLEAN | 1.0 | **318 mm** |

TIGHT is the default. A 90° turn at 291 mm sweeps the robot roughly 291 mm
forward *and* 291 mm sideways — in a 2.0 m arena that is a seventh of the floor
per turn.

**Turns terminate on measured heading, not arc length.** Speed does not make
the angle wrong; it only changes how much floor the turn eats. The checklist's
warning that "if speed increases, the robot will take a larger turning angle"
applies to the path, not the angle.

**The θ/2 illusion.** On a circular arc the *nose* rotates by θ, but the
straight line from start to finish sits at θ/2 from the original heading.
Judging a 90° turn by where the robot ended up reads 45° every time. Judge the
nose.

**Distance during an arc is not honest.** The differential assist deliberately
scrubs the rear tyres, so encoder distance over-reads during a turn. The
*angle* is unaffected because the gyro measures the body directly.

Robot footprint is roughly 18.8 cm wide, 23 cm long (plate).

---

## Architecture

```
main.c          peripheral init, 100 Hz tick, OLED, 9-mode button UI
PeripheralDrivers/
  motors.c      PWM to two AT8236 bridges + steering servo
  encoders.c    quadrature capture, position and windowed speed
  pid.c         per-wheel PI speed control
  odom.c        pose integration, heading hold, cross-track, arc wheel split
  motion.c      non-blocking distance and arc primitives, arc profiles
  imu.c         ICM-20948 gyro: bring-up, bias, staleness detection
  ir.c          2x GP2Y0A21YK via ADC1 + circular DMA
  ultrasonic.c  HC-SR04 via TIM8 input capture
  commands.c    RPi wire protocol parser and queue (no hardware, host-testable)
  rpilink.c     USART3 line assembly and command executor
  oled.c        SSD1306 bit-bang driver
```

### Control tick — TIM6, 100 Hz, NVIC priority 6

**The order is not negotiable:**

```
Encoders_Update() → IR_Update() → Ultrasonic_Tick() → IMU_Tick()
                  → Motion_Tick() → Odom_Update() → PID_Update()
```

Sensors first, then control. `IMU_Tick()` must precede `Odom_Update()` or the
heading loop acts on a value one tick stale.

**Nothing blocking in the tick.** The ADC free-runs into memory over DMA; the
ultrasonic fires once every 60 ms with an 11 µs bounded busy-wait; the I²C read
lives in `IMU_Poll()` out in the main loop.

### Interrupt priorities

Grouping is `NVIC_PRIORITYGROUP_4` — all four bits preemption.

| IRQ | Priority | Note |
|---|---|---|
| TIM8_CC (echo capture) | 3 | Pre-empts the tick deliberately — 10 ms of capture delay is 1.7 m of error |
| USART3 | 5 | Pre-empts the tick, handles one byte, returns |
| TIM6 (control tick) | 6 | |
| DMA2_Stream0 (ADC) | 6 | Half- and full-transfer interrupts masked in `IR_Init()` — they fired ~21 000/s and jittered the tick |
| SysTick | 15 | |

---

## Hardware

### Clock

HSE 8 MHz crystal → PLL (M=8, N=336, P=2) → **168 MHz**.

| Bus | Divider | Peripheral | Timer clock |
|---|---|---|---|
| AHB | ÷1 | 168 MHz | — |
| APB1 | ÷4 | 42 MHz | 84 MHz (TIM2/3/4/6/12) |
| APB2 | ÷2 | 84 MHz | 168 MHz (TIM8/9, ADC1) |

**CubeMX 6.5.0 does not emit Voltage Scale 1 or `FLASH_LATENCY_5`.** Both are
hand-patched in `SystemClock_Config()` and are silently dropped on every
regeneration. Without them the part boots and then corrupts fetches under load.

### Pin map

| Function | Pin(s) | Peripheral | AF |
|---|---|---|---|
| Motor A (left rear) | PB9 / PB8 | TIM4_CH4 / CH3 | AF2 |
| Motor B (right rear) | PE5 / PE6 | TIM9_CH1 / CH2 | AF3 |
| Encoder A | PA15 / PB3 | TIM2_CH1/CH2 | AF1 |
| Encoder B | PB4 / PB5 | TIM3_CH1/CH2 | AF2 |
| Steering servo | PB15 | TIM12_CH2 | AF9 |
| Ultrasonic trigger | PB14 | GPIO out (J6) | — |
| Ultrasonic echo | PC7 | TIM8_CH2 (J2) | AF3 |
| IR left / right | PC0 / PC1 | ADC1_IN10 / IN11 | analog |
| IMU SCL / SDA | PB10 / PB11 | I2C2 | AF4 |
| IMU nCS | PB12 | GPIO out, **must be HIGH** | — |
| USART3 → RPi | PD8 / PD9 | USART3 | AF7 |
| OLED | PD11–PD14 | GPIO bit-bang | — |
| LED3 | PE8 | GPIO, active low | — |
| User button | PE0 | GPIO in, external pull-up | — |
| Debug | PA13 / PA14 | SWD | — |

**Not in the `.ioc`:** PE0, PB10, PB11, PB12 are configured by hand in
`main.c`. The `.ioc` is not a complete description of the board.

**Free for Motors C and D:** driver pins PE9/PE11 (Motor C) and PE13/PE14
(Motor D), both on TIM1. Encoder C is on PB6/PB7 = TIM4_CH1/CH2, which
**conflicts with Motor A's drive** — adding Motor C means moving Motor A to
TIM10_CH1 (PB8) and TIM11_CH1 (PB9), both AF3.

### Timer configuration

| Timer | PSC | ARR | Result |
|---|---|---|---|
| TIM4 (motor A) | 0 | 4199 | 20 kHz |
| TIM9 (motor B) | 1 | 4199 | 20 kHz (PSC 1 because TIM9 is APB2) |
| TIM12 (servo) | 83 | 19999 | 50 Hz, 1 µs/count — CCR *is* the pulse width |
| TIM6 (tick) | 8399 | 99 | 100 Hz |
| TIM8 (echo) | 167 | 65535 | 1 MHz, wraps at 65.5 ms |
| TIM2/TIM3 (encoders) | 0 | 65535 | full rate, input filter 10 |

ADC1: PCLK2/8 = 10.5 MHz, 480-cycle sampling, scan + continuous + circular DMA.
I²C2: 400 kHz. USART3: 115200 8N1.

### Board conditions specific to this equipment

- **Debug must be Serial Wire, not JTAG.** PA15, PB3 and PB4 are JTAG pins
  after reset and all three carry encoder signals. Set to JTAG and encoder A
  silently never counts.
- **The IMU runs at 1.8 V** behind an RS0102 level shifter fed by its own
  regulator. **PB12 is nCS and must be HIGH** — low leaves the part in SPI mode
  where it never acknowledges, which looks exactly like a dead sensor.
- **PC0/PC1 are FT pins but lose 5 V tolerance in analog mode** (DS8626 Table 7
  note 5), and are on the Table 47 list where allowed negative injected current
  is 0 mA. They are currently driven bare by 5 V sensors. See `ir.h` for the
  series-resistor mitigation if it ever needs addressing.
- **`main.c` is hand-written and largely outside USER CODE blocks.**
  Regenerating from CubeMX will destroy it. The I²C HAL was added by copying
  four files manually for exactly this reason.

---

## Calibration — measured values

**Everything here was measured on this robot. Do not replace with catalogue
figures.**

### How the learning works

Three values are learned while driving — arc decel, brake lag, steering trim —
and all three are RAM-only, so every power-on starts over. They used to be
fixed-gain integrators at ~0.25–0.3, which is about ten iterations each. They
are not any more, and the reason is worth keeping:

**Decel and lag take the first valid arc outright** (`MOTION_ARC_SEED_FIRST`).
What the old rule filtered *toward* was a constant measured on another day and
another floor; the first arc is a measurement of this one. There was no prior
worth defending, and the cold first run — the one A.3 and A.4 are graded on —
was always driven on numbers nobody had checked here. Later arcs still filter
at the old gains, which is where noise rejection belongs. A value restored with
`!CALD`/`!CALL` counts as seeded and is *not* overwritten by the next arc: that
one is a real prior.

For the lag specifically, `over / rate` **is** the lag error in seconds, not a
proxy for it — braking Δt late over-rotates by rate × Δt — so gain 1.0 is the
deadbeat correction and `MOTION_ARC_LAG_GAIN` was only ever damping.

**The trim is measured, not searched for** (`TRIM_LEARN_FROM_OUTPUT`). If the
robot is being held straight, the mean of what the steering loop *put out* is
the bias the centre is missing, already in servo µs:

```
s_servoUs = SERVO_CENTER_US + correction + s_headingTrim
```

so `trim += mean(correction)` is the answer rather than a fraction of it.
Damped to 0.7 with a 25 µs step clamp so one bumped or wheel-slipped run
cannot own the trim.

That change also gives the trim **somewhere to stop**, which is the part that
actually mattered. The old rule chased mean heading *error*, and the loop
ignores any error inside `HEADING_DEADBAND_DEG` — so a robot sitting steadily
at 0.2° contributed nothing the servo would act on and 0.8 µs of trim every
single run, forever. It was chasing something the controller had already ruled
close enough. Mean *correction* is zero exactly when the steering is doing
nothing, which is the real definition of a correct centre.

Both estimators are still accumulated every run, and both learners keep their
old rule behind a `#if`, so either can be A/B'd on the floor with a one-line
rebuild. Both also now discard the first 0.5 s of a straight
(`TRIM_WARMUP_TICKS`): the servo slams from centre at launch and that stretch
is transient, not evidence. Throwing it away is a better estimate from the same
run, which is cheaper than driving further.

A straight too short to clear `TRIM_WARMUP_TICKS + TRIM_MIN_SAMPLES` leaves
the trim untouched, and an untouched trim reads exactly like a converged one.
`Odom_TrimWasUpdated()` is how you tell them apart; mode 1 shows it as `SKIP`.
This is the one failure here that could quietly poison a saved calibration
profile, which is why it is visible rather than merely correct.

**None of this has been run on a robot.** Watch `trm`'s delta on mode 1: it
should shrink hard after the first cycle and sit near zero. If it oscillates —
same size, alternating sign — lower `TRIM_LEARN_GAIN`; the backlash push
inside `correction` is the thing most likely to cause it.

### Geometry

| Constant | Value | Source |
|---|---|---|
| `WHEEL_DIAMETER_MM` | 65.9 | Caliper across the tread **and** road test, agreeing |
| `WHEEL_BASE_MM` (track) | 127.0 | Measured |
| Wheelbase (front–rear axle) | ~170 | Measured, used for steer-angle maths |
| `ODOM_COUNTS_PER_REV_A/B` | 1560 | Ten-revolution hand test, both wheels |

**The wheel is sold as 2.4 inch (60.96 mm). That is the RIM.** The tread
measures 2.56 inch. Using the rim figure makes every distance 6.8% short.

### Encoders

`ENCODER_PPR 13`, `ENCODER_GEAR_RATIO 30` → 13 × 4 × 30 = 1560.
`ENCODER_A_INVERT 0`, `ENCODER_B_INVERT 1`. Ticked at a fixed 10 ms; RPM
averaged over 4 ticks (0.96 RPM resolution rather than 3.85).

> **The kit's motor datasheet PDF is for a different part.** It describes a
> JGB37-520 with an 11 PPR encoder. The issued motors are MG513P3012V. Using 11
> gives 1320 counts/rev and the robot stops 15% short of every commanded
> distance. Last year's firmware shipped with exactly that error.

### Motors

`MOTOR_A_INVERT 1`, `MOTOR_B_INVERT 1` — both verified on the stand.
`MOTOR_DEADBAND 600`, `MOTOR_A_MAX_RPM 378`, `MOTOR_B_MAX_RPM 362`. Usable
range roughly 55–360 RPM; below 55 the deadband clamp takes over and the PID
stops regulating.

### Steering

| Constant | Value |
|---|---|
| `SERVO_CENTER_US` | 1500 |
| `SERVO_MIN_US` / `SERVO_MAX_US` | 900 / 2100 |
| Mechanical stops | 850 / 2125 (swept) |
| `MOTION_ARC_STEER_US` | 575 |
| `SERVO_BACKLASH_US` | 25 (≈50 µs slack measured) |

Travel is **symmetric** about 1500 on purpose — an asymmetric pair makes one
turn direction clamp and come out wide, silently. Steer angle at 575 µs is
26.8°, near the practical Ackermann maximum; the linkage saturates hard, with
2.9× the pulse buying only 1.24× the angle.

### PID and heading

`KP 1.0`, `KI 2.0`, `KD 0` at 10 ms. Kd is deliberately zero — RPM quantises
and derivative amplifies exactly that.

`ODOM_HEADING_SOURCE = IMU`, `HEADING_SIGN -1`, `HEADING_KP_US 8.0`,
`HEADING_MAX_US 150`, `HEADING_DEADBAND_DEG 0.3`. Cross-track on,
`CROSS_KP_DEG_PER_MM 0.08`, `CROSS_MAX_DEG 8.0`.

> `HEADING_SIGN` is **−1 for the gyro and +1 for the encoder** — the two sources
> count in opposite directions. If `ODOM_HEADING_SOURCE` is ever switched back,
> flip this at the same time.

### IMU

`IMU_GYRO_FS_SEL 2` (±1000 dps, 32.8 LSB/dps), `IMU_GYRO_DLPFCFG 4` (23.9 Hz),
`IMU_Z_SIGN +1`, ODR 102 Hz.

**±250 dps clipped.** Vibration swinging several hundred dps around a small
mean clips harder on one side, rectifies, and under-reports the turn — a
commanded 90° read 87 while the robot physically turned about 156.
**196.6 Hz filter bandwidth was above Nyquist** for the 102 Hz output rate, so
vibration aliased into the measurement.

Confirmed on the floor: peak raw during normal turns is 4000–5000, about 14% of
full scale. The same signal at ±250 dps would sit at 55%.

### Motion

`MOTION_CRUISE_RPM 100`, `MOTION_APPROACH_RPM 40` over the last 150 mm,
`MOTION_BRAKE_MM 0`, `MOTION_ALIGN_TICKS 20`, `MOTION_TIMEOUT_TICKS 1500` (15 s).

Arcs: `MOTION_ARC_STEER_US 575`, adaptive braking on,
**`MOTION_ARC_DECEL_DPS2 636`**, **`MOTION_ARC_LAG_S 0.003`**.

> Those two seeds matter more than they look. Both quantities are learned live
> and **reset at power-off**. Seeded from a converged run, a cold first turn
> lands within 1°. With the old 294 seed a cold session ran errors of 9, 7, 3, 2
> before settling. **A.4 is one turn on a supervisor's word** — a cold robot
> hands them the first number, not the last.

### Arc profiles (`motion.c`)

```
  name     steer  rpm  appr  appr_deg  brake  radius  boost
  TIGHT     575   150  100     20.0      8.0   291.0   2.0
  CLEAN     575   150  100     20.0      8.0   318.0   1.0
  SLOW      575    70   70     15.0      3.0   306.0   1.5
```

`radius_mm` is a **measurement**, not a setting — changing `rpm`, `steer_us` or
`diff_boost` invalidates it. `brake_deg` is **dead** while
`MOTION_ARC_ADAPTIVE_BRAKE` is 1 (it is); the field is kept so the fallback
still compiles.

### IR sensors

**Per channel — the two units are not interchangeable.** The left reads about
20% more counts than the right at the same distance. Both were checked and sit
square at the same height, so that is the parts, not the mounting.

```
Left:   cm = 30.40 x V^-1.099
Right:  cm = 24.94 x V^-0.979
```

Measured with the matt black obstacle as the target, eleven points 5–80 cm.
`IR_DIVIDER_RATIO 1.0` — the sensors are wired bare, confirmed by inspection.

Valid span 10–80 cm. Below 10 cm the output **saturates** (5 cm and 8 cm both
map to about 9 cm), so readings there are refused. No fold-back was observed
above 5 cm. The right sensor is the weaker unit and stops reporting past about
65 cm.

**Obstacle thresholds in raw counts**, for when `FIR`/`FIL` get written — these
must be per channel too:

| Boundary | Left | Right |
|---|---|---|
| 25 cm | 1425 | 1296 |
| 30 cm | 1232 | 1062 |
| 40 cm | 990 | 815 |

### Ultrasonic

`US_US_PER_CM 58.3`, ping every 60 ms, echo timeout 50 ms, stale after 300 ms,
valid span 2–400 cm, median of 3.

Verified against a tape over 10–100 cm with ±2 µs repeatability. It reads
roughly 1.3 cm long, consistently — left uncorrected because a real sensor
offset could not be distinguished from a measurement-reference artefact, and
the bias is well inside what obstacle stopping needs. **Direction matters
though: reading long means the robot gets closer than it believes before
stopping.** With `RPILINK_F0_STOP_CM 15` it actually halts around 13.7 cm.

**The same bias lands on `FU{n}`, and it does not average out.** `FU` is more
*precise* than `F0` — it removes the scatter — but it is no more *accurate*,
because every pass reads through the same +1.3 cm offset. `FU20` settles about
18.7 cm from the obstacle, repeatably. If the RPi needs the true gap, either
ask for `FU21` or measure the offset on the day and correct once, on the Pi
side, where it can be changed without a reflash. Do not chase it by retuning
the approach: the approach is not what is wrong.

A 1 kΩ series / 2.2 kΩ shunt divider is fitted on the echo line.

---

## Design decisions worth preserving

**Turns terminate on measured heading, not arc length.** Arc length needs an
accurate radius, which depends on steering geometry, linkage slack, tyre scrub
and speed. Measuring the angle directly makes all of that irrelevant.

**Braking is adaptive.** `coast = ω²/2α + ω·t_lag`. The quadratic term is the
physical coast, the linear term the brake engagement latency. Both are learned
per run, so a profile change or a different floor is absorbed automatically.

**Adaptation is confined to what genuinely varies** — friction, battery,
weight. Geometry and sensor scale are measured once and frozen. Adaptation must
never be able to mask a calibration error.

**Differential assist.** With no mechanical differential, over-driving the outer
rear wheel adds a yaw moment and tightens the turn. Costs tyre scrub and makes
encoder distance dishonest during an arc — but not the angle, because the gyro
measures the body directly.

**Cross-check.** `arc_length ÷ radius` is a second angle estimate sharing no
hardware with the gyro. Reported as `XOK`/`XBAD` on the turn screen. Only
meaningful at `diff_boost 1.0` (the CLEAN profile) — above that the encoders
are inflated by scrub. **This exists because a gyro fault is self-consistent:**
every number on the display derives from the gyro, so they all agree with each
other while all being wrong. That has happened twice, and both times the only
thing that caught it was a tape measure on the floor.

**Heading hold uses the steering servo, not wheel trim.** On an Ackermann
chassis a wheel-speed difference cannot change heading — it only yaws the body
against the front tyres and scrubs them.

**No integral term in the heading loop.** It was tried and made straightness
*and* distance worse: it winds up against 50 µs of backlash while nothing
mechanical happens, then over-corrects, and a weaving path reads long.
The steering centre trim is learned **between** runs instead.

---

## Bench procedures

**Straight-line calibration.** Run at 1000 mm. `got` vs tape is wheel diameter
(`WHEEL_DIAMETER_MM × tape / got`); `got` vs target is coast (put the excess in
`MOTION_BRAKE_MM`). **Do the diameter first** — coast is measured in odometry
units, so calibrating it against a wrong scale bakes the error in twice.

**Encoder counts.** Mark the tyre, zero, roll exactly ten turns by hand, divide.
Ten turns because the start/end alignment error is fixed, so spreading it over
ten divides it by ten. **Turn slowly** — fast hand-rolling back-drives the 30:1
gearbox and can slip the hub, which looks exactly like a counting fault.

**Servo end stops.** Mode 7. Step up watching the **wheels**, not the numbers.

**Turn radius, without trusting the gyro.** Mark the floor under the rear axle,
run a 90, mark again, measure the chord. `R = chord / 1.414`. This is the only
measurement in the system that does not pass through the gyro.

**Gyro sanity.** Mode 10, robot flat on the floor, slide it through 180° both
slowly and fast. Both must read 180 — a fast reading that comes up short means
clipping. Rotate the whole robot on the ground; lifting and twisting mixes in
roll and pitch.

**IR calibration.** Mode 9, sensors mounted in their final positions, matt black
obstacle as the target (the arena obstacles are matt black, and a white card
gives a different answer). Record `IR_LeftFiltered()`, not the raw sample. Fit
`ln(cm)` against `ln(volts)` — the slope is B, the intercept is `ln(A)`.

---

## Faults found and fixed

Recorded because several were invisible on the display and cost real time.

| Fault | Symptom | Cause |
|---|---|---|
| Brake was a coast | Inconsistent stopping distance | `PID_Update()` overwrote `Motors_Brake()` in the same tick |
| Encoder sampled at half rate | Stale PID feedback, doubled odometry noise | 20 ms rate limit against a 10 ms loop |
| DMA interrupt storm | Control-tick jitter | ~21 000 IRQ/s at the same NVIC priority as TIM6 |
| Wheel hub slipping | Encoder B lost 10% of counts, 32% spread within a run | Grub screw — tightened |
| Rim vs tread diameter | 6.8% distance error | 2.4″ is the rim; tread is 2.56″ |
| Gyro clipping | Turns under-read by 1.8× | ±250 dps too sensitive for vibration |
| DLPF above Nyquist | Vibration aliased into the measurement | 196.6 Hz filter, 102 Hz output rate |
| Encoder fallback sign inverted | 90° turn ran to 369° | Fallback was CW-positive, gyro CCW-positive |
| Recentre during coast | Every stop nudged sideways | Servo swung while the robot was still moving |
| Heading integral | Straightness **and** distance worse | Wound up against backlash; weaving path reads long |
| Fault handlers left motors driving | Runaway on a hard fault | Added `KILL_MOTORS()` to five handlers |
| **UART could go permanently deaf** | Link stops hearing commands, everything else fine | HAL tears down RX on an overrun and calls a weak error callback that never re-arms |
| **Console collided with the protocol** | Would have corrupted the RPi's reply stream | Reports and banner shared USART3 with `OK`/`RESEND` |
| **IRCAL showed the raw sample** | Calibration screen too noisy to read | Displayed the instantaneous DMA value instead of the median |

**The recurring lesson:** every displayed number derives from the gyro, so a
gyro fault is self-consistent — the display agrees with itself throughout.
Twice the only thing that caught it was a tape measure on the floor. That is
why the cross-check exists.

---

## Known limitations

- **Learned steering trim is RAM-only** and resets at power-off. The arc decel
  and lag are now seeded, so only the trim starts cold.
- **Once the gyro goes stale it stays stale until reboot.** `IMU_Poll()` returns
  early when not ready. Deliberate — a flapping sensor switching heading
  sources mid-run is worse than a consistently degraded one. Watch `st` on
  mode 10.
- **Cross-track uses dead-reckoned `y_mm`.** It returns the robot to where it
  *believes* the line was, not to an absolute position.
- **`Sensors_ObstacleAhead()` treats "no reading" as "obstacle".** Fail-safe,
  but `F0` will refuse to move whenever the echo is lost.
- **Motors C and D are not implemented.**
- **The RPi link has never carried a byte.** The parser passes 36 host-side unit
  tests; the executor is untried on real hardware.
- **`Sensors_HeadingDeg()` / `Sensors_HeadingValid()` are weak stubs.**

---

## Outstanding work

**Before the first wire test**
- Agree the three protocol questions above with the RPi owner
- Then actually test it — that is the single largest unknown remaining

**Calibration**
- SLOW profile radius by floor chord (306 came from a different speed)
- Pick the `FIR`/`FIL` threshold distance and wire in the per-channel counts

**Software**
- Task 2 opcodes: `FIR`, `FIL`, `FIRO`, `FILO`, `SR`, `SL`
- Confirm the ultrasound's +1.3 cm bias with the sensor in its final mounting,
  then decide whether `FU` should correct for it or the RPi should
- Motors C and D
- Strong definitions for the two heading hooks, if the RPi ever wants heading

**Watch on the bench**
- `rx` on mode 8 should stay at **0**. Anything else means the link is glitching
  and being silently recovered — check the wiring before it bites in a run.
- `pk` on mode 10 should stay well under 32767. Near the rail means the gyro is
  clipping and every angle is under-read.
