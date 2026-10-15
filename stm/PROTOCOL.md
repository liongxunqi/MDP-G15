# RPi ↔ STM32 Protocol — MDP Group 15

**Status: firmware side IMPLEMENTED, awaiting RPi agreement.**

Movement commands are unchanged from the frozen interface and already in use.
Queries, config and the `FAIL,*` replies are implemented in firmware but have
**not** been agreed with the RPi owner or tested on the wire. Treat §5, §6, §7
and the `FAIL,*` rows of §3 as a proposal until that happens — the firmware can
change if the RPi side wants different names or fields.

**Protocol version: 5** — ask for it with `?VER`.

Version 5 adds `!CALSL` / `!CALSR`, a separate arc steering deflection for
each side (§7.1), and appends the two values in force to `?CAL`. Additive.

Version 4 freezes calibration during tasks (§7.1): learning is **off at
power-on** and runs only after `!LEARN1`; `!CALG` sets a gyro scale; heading
error now carries from one move into the next and `?HDG` reports it; `!PROF`
accepts only `0` (the robot is locked to TIGHT). `?CAL` gains two appended
fields. Movement tokens are unchanged, but a sender that relied on the robot
learning during a run must now ask for it.

Version 3 adds `FU<n>` (§4.1) and the `FAIL,NOECHO` reply. Movement is purely
additive; a sender that treats an unrecognised reply as fatal must learn
`NOECHO` before it sends its first `FU`.

Version 2 added `?CAL` and the three `!CAL*` setters (§7) and changed nothing
else. A sender written against version 1 keeps working untouched.

This document is authoritative. Where it disagrees with a docstring or a
comment anywhere else, this wins.

---

## 1. Link

| | |
|---|---|
| Port | **USART3, PD8/PD9** — board **USB Port 2**, the one labelled "to RPi" |
| Settings | **115200 8N1**, no flow control |
| Pi device | `/dev/ttyACM0` or `/dev/ttyUSB0` — verify, it depends on the driver |

USB Port 1 is UART1 and is for code download. **The firmware never transmits
there.** If a terminal shows nothing, this is almost always why.

---

## 2. Wire format

```
RPi → STM :  <tok1>,<tok2>,...,<tokN>\n
STM → RPi :  <one reply line>\n
```

- Tokens separated by **comma or space** — both accepted
- Line terminated by `\n`; a preceding `\r` is tolerated and discarded
- Input is **lowercased on receive**, so sender casing is irrelevant
- **Exactly one reply per LINE**, never per token
- Parsing is **all-or-nothing** — one bad token and nothing from that line runs
- Max **128 bytes** per line, max **16** primitives queued

### The rule that matters most

**Do not send the next line until the previous one has been answered.**

This is not style. A line arriving while the previous is still executing gets
appended to the same queue, and when the queue drains you get **one reply for
two lines**. The sender is then permanently one reply out of step, silently.

A line arriving while the previous is still *unconsumed* is dropped and
answered `RESEND`.

---

## 3. Replies

| Reply | Meaning |
|---|---|
| `OK` | The whole line completed normally |
| `RESEND` | Parse failure. **Nothing executed** — safe to retransmit verbatim |
| `FAIL,TIMEOUT` | Watchdog fired mid-move. Wheel stalled or encoder dead |
| `FAIL,WRONGWAY` | Arc rotated away from its target and was aborted |
| `FAIL,SHORT` | Arc finished more than 10° from its aim (braked early off a wheel surge, or stopped) - **v7** |
| `FAIL,NOECHO` | `FU<n>` had no usable ultrasound reading. **The robot did not move** |
| `<TAG>,<fields>` | Answer to a query — see §5 |

`RST` is the sole command that replies **nothing at all**. Do not wait for it.

> **`FAIL,*` is new.** Previously a timed-out move replied `OK`, making a
> stalled wheel indistinguishable from success. Senders must treat `FAIL,*` as
> "the robot is not where you think it is" — stop the sequence and re-plan.

**Cap your retries.** On `RESEND`, retransmit at most three times, then give up
and report. A permanently malformed segment retried forever is an infinite
loop, and the robot never moves while it happens.

---

## 4. Movement commands

Queued and executed in order. The reply comes when the **last** one finishes.

| Token | Arg | Meaning |
|---|---|---|
| `F<n>` | cm | Forward n cm |
| `F0` | — | Forward until an obstacle stops it (ultrasonic, 15 cm). Blunt — see §4.1 |
| `FU<n>` | cm | Forward until the ultrasound reads **n** cm. **n is yours to choose**, 5–200 |
| `R<n>` | cm | Reverse n cm |
| `FR<n>` | degrees | Arc forward-right |
| `FL<n>` | degrees | Arc forward-left |
| `RR<n>` | degrees | Arc reverse-right |
| `RL<n>` | degrees | Arc reverse-left |
| `S` | — | Stop: brake, recentre steering. Replies `OK` |
| `RST` | — | Emergency abort: drop queue, brake now and hold the brake until the next move (it used to release the wheels at once, so the robot rolled on). **No reply** |

Example: `FR90,F20,S\n` → arc right 90°, forward 20 cm, stop → one `OK`.

Arguments are unsigned decimal only. `F-5`, `F1x` and a bare `F` are all
parse failures, not clamped. A zero argument is valid **only** for `F`, where
it means "until obstacle".

### 4.1 `FU<n>` — stop at a distance *you* pick

`F0` is a trip-wire: drive, and brake the instant the ultrasound reads under
15 cm. Fine for "don't hit the wall", useless for "stand off at exactly 20 cm",
because **the reading that trips it is old**.

The driver pings every 60 ms and reports the **median of the last three**, so
the number the firmware sees describes where the robot was roughly 120 ms ago.
At cruise the robot covers 345 mm/s, so that lag is about **4 cm** — and a
different 4 cm every run, depending where in the ping cycle you arrived.
Measured in simulation against the real executor: a trip-wire stop overshoots
by 3.9–5.3 cm across 1–2 m approaches.

`FU<n>` doesn't trip on the sensor at all. It uses the sensor to **measure**
and the odometry to **move**:

1. stand still, let the median filter fill with stationary samples, read the gap
2. hand `gap − n` to the same closed loop that holds a straight run to ±6%
3. stop, re-measure, correct if still out — up to 4 passes

It finishes within **±2 cm** of `n`. Same simulation, same scenarios: ≤1 cm
error at every distance tested, including with ±2 cm of injected sensor noise.

**What it costs you: time.** Each pass pays a 400 ms stationary settle, so
budget roughly 1.2 s plus travel for a short approach and 4 s for a metre.

**Timing example** — `FU20` from 1 m away takes about 3.8 s. Do not treat a
long silence as a dead link; wait for the reply.

| Situation | What happens |
|---|---|
| No usable reading when the command starts | `FAIL,NOECHO`, **robot does not move** |
| Reading already within ±2 cm of `n` | `OK` immediately, no movement |
| Robot starts closer than `n` | Reverses to `n`, capped at 20 cm of backing up |
| Gap closes early mid-move (obstacle moves in) | Move is cut short, re-measured, corrected |
| Reading jitters wider than ±2 cm | Gives up after 4 passes and replies `OK` — ask `?US` for the truth |
| `FU` fails `NOECHO` mid-line | **Rest of the line is dropped**, one `FAIL,NOECHO` reply |

**`FU` is precise, not accurate — the difference matters here.** It removes the
*scatter*, not the sensor's *offset*. This HC-SR04 reads about **1.3 cm long**,
measured against a tape over 10–100 cm, and every pass of the approach reads
through that same offset, so it cannot average out. `FU20` settles roughly
**18.7 cm** from the obstacle, repeatably. Ask for `FU21`, or measure the
offset on the day and correct it on the Pi side where it can change without a
reflash.

**Confirm with `?US`, not `?DIST`.** `?DIST` reports the last *leg* of the
approach, which after a correction pass is a couple of centimetres, not the
total travelled. `?US` reports the gap actually achieved, which is the number
that says whether `FU` did its job.

**Don't use `FU` for the whole journey.** Past roughly 2 m the beam has spread
wide enough that the nearest thing it hears is often not the thing you meant.
Send `F<n>` for the bulk of the travel and `FU<n>` for the last stretch —
`F150,FU20` rather than `FU20` from across the arena. Values outside 5–200 cm
are a parse failure, not a clamp.

### Note the collision

`S` means **stop**. It does not mean "backward" — that is `R<n>`. An earlier
draft of the RPi client used `S050` for reverse; that token is rejected here.

---

## 5. Queries

**Single-token lines only.** `?US` alone. A query mixed into a movement line
is a parse failure.

Queries are answered **immediately**, even while a move is running — they
bypass the queue entirely and never disturb it. The reply **is** the data
line; there is no separate `OK`.

| Send | Reply | Fields |
|---|---|---|
| `?US` | `US,<cm>` | Front distance. `65535` = no reading |
| `?IR` | `IR,<left_cm>,<right_cm>` | `65535` = out of range |
| `?IRR` | `IRR,<left>,<right>` | Raw filtered counts, 0–4095 |
| `?IRW` | `IRW,<l_cm>,<r_cm>,<l_raw>,<r_raw>,<x_mm>,<y_mm>,<hdg_x10>,<ms>` | Both IRs, their raw counts and the `?WPOSE` pose from **one** control tick, plus its `HAL_GetTick()`. A `65535` with **high** raw counts is closer than 10 cm, not clear. **v9** |
| `?IRE` | `IRE,<state>,<side>,<cm>,<raw>,<x_mm>,<y_mm>,<hdg_x10>,<ms>` | IR edge latch: state 0 off, 1 armed, 2 latched; side 1 left, 2 right. Latched fields are the first tick of the confirming run, else 0. **v9** |
| `?POSE` | `POSE,<x_mm>,<y_mm>,<hdg_x10>` | Since the last `!ZERO` or move start |
| `?WPOSE` | `WPOSE,<x_mm>,<y_mm>,<hdg_x10>` | Continuous since boot; unaffected by move starts, FU passes or `!ZERO`. See §12 |
| `?DIST` | `DIST,<mm>` | Travelled in the current or last move |
| `?TURN` | `TURN,<deg_x10>` | Turned in the current or last arc, signed |
| `?STAT` | `STAT,<state>,<busy>,<imu_ok>,<profile>` | `busy` is the **line**, not the motor — see below |
| `?IMU` | `IMU,<ok>,<hdg_x10>,<rate_x10>,<stalls>,<peak>` | |
| `?XCHK` | `XCHK,<enc_x10>,<gyro_x10>,<err_pct>,<ok>` | Last arc's cross-check |
| `?VER` | `VER,<name>,<proto>` | Firmware identity and protocol version |
| `?CAL` | `CAL,<decel_x10>,<lag_ms_x10>,<trim_us>` | What the firmware has learned this power-on |

All values are **integers**. Anything needing a decimal is scaled ×10 — so
`hdg_x10` of `-453` means −45.3°. There are no floats on the wire.

**`?STAT` fields:** `state` is `0` IDLE, `1` ALIGN, `2` RUN, `3` BRAKE,
`4` DONE, `5` TIMEOUT. `busy` is 1 **while the line you sent is unfinished** —
not merely while the motors are turning. `imu_ok` is 1 while the gyro is
healthy. `profile` is the active arc profile: 0 TIGHT, 3 SAFE (a SAFE build
reports 3 after `!PROF0` too).

> `busy` changed meaning in version 3. It used to track the motion layer
> alone, which was indistinguishable from the line for ordinary movement. It
> is not for `FU<n>`: that command spends most of its life deliberately
> **stopped**, settling between passes, and the old field read 0 in those
> windows. A sender polling `?STAT` to decide the robot had arrived would have
> carried on mid-approach. The reply is still the real completion signal —
> `?STAT` is for watching, not for sequencing.

**`?IMU` is the health check.** `ok` 0 means the gyro has gone stale and
heading has fallen back to the encoder difference, which is much less
accurate. `stalls` counts how many times that has happened. The firmware
recovers the I2C bus and re-arms the gyro by itself once the robot has stood
still for ~0.5 s (~3 s if its bias was never measured), so `ok` 0 at rest is
usually temporary; one that stays 0 is a wiring or sensor fault. `peak` is the
largest raw sample seen — anything approaching **32767** means the gyro is
clipping and every angle is being under-reported. Normal is 4000–5000.

**`?XCHK` is the independent witness.** The encoder-derived angle shares no
hardware with the gyro, so a disagreement means one of them is lying. `ok` is
1 if they agree within tolerance. **Only meaningful on profile 1 (CLEAN)** —
the other profiles deliberately scrub the tyres, which inflates the encoder
arc. Reads all zeros when the check did not run.

---

## 6. Config

| Send | Effect | Reply |
|---|---|---|
| `!PROF0` | Arc profile **TIGHT** — radius 291 mm, fastest | `OK` |
| `!PROF1` | Arc profile CLEAN — radius 318 mm. **Locked out (v4)** | `RESEND` |
| `!PROF2` | Arc profile SLOW — radius 306 mm. **Locked out (v4)** | `RESEND` |
| `!PROF3` | Arc profile **SAFE** — TIGHT's steering at 120 rpm instead of 150. Radius not yet measured | `OK` (`RESEND` on firmware before SAFE) |
| `!ZERO` | Zero the odometry and heading, **and the heading carry-over** (§7.1) | `OK` |
| `!LEARN1` / `!LEARN0` | Switch the calibration learners on / off (§7.1) | `OK` |

**Locked to TIGHT and SAFE** (`MOTION_PROFILE_LOCK` in `motion.h`). The
braking model, steering and radius are one set of numbers that only fits the
profile it was measured on, so each of the two has its own saved calibration;
the RPi records which profile a calibration was taken on and selects it before
restoring. Turning the lock off restores CLEAN and SLOW.

**Send `!ZERO` after anything moves the robot by hand.** The gyro sees the
rotation, and without a zero the next move would steer it straight back.

---

## 7. Calibration

The firmware learns three things while it drives: arc deceleration, brake
engagement lag, and the steering centre trim. **All three live in RAM and are
lost at power-off**, so a cold robot has to converge them again every session
— the difference between a 9° error and a 1° one on the first turn.

How fast it converges changed. The decel and lag now take the **first** valid
arc of a power-on outright instead of filtering toward it from a compiled
constant, and the trim is measured from the steering loop's own output rather
than searched for. One arc and one straight now do what used to take about ten
of each. Restoring saved values with the setters below still beats both, and a
restored value is not overwritten by the next arc.

`?CAL` reads them. The setters write them back.

| Send | Effect | Reply |
|---|---|---|
| `?CAL` | Read all three | `CAL,<decel_x10>,<lag_ms_x10>,<trim_us>` |
| `!CALD<n>` | Arc deceleration, **dps² ×10** | `OK` / `RESEND` |
| `!CALL<n>` | Brake lag, **milliseconds ×10** | `OK` / `RESEND` |
| `!CALT<n>` | Steering trim, **µs, signed** | `OK` / `RESEND` |

`!CALT` is the only argument anywhere in this protocol that may be negative —
every movement argument is a magnitude with its direction in the opcode.

### The three rules

**1. The sequence lives on the RPi.** There is no "run calibration" command
and there will not be one. Calibration is a script you send, built from
ordinary primitives. Nothing the host can say makes the firmware drive itself,
so no wire command can leave a sequence running when a task starts.

> **Amended.** OLED **mode 1 (CALIB)** does drive this sequence, from the
> button. That is a bench tool with a human standing over it, not a protocol
> feature — there is still no token for it, and a sender cannot start it. It
> also cancels itself the moment a command line arrives, so the hazard this
> rule guards against (a firmware-driven sequence still running when a task
> starts) cannot happen: the task's first line ends the cycle.

**2. The setters are refused while a move is running.** `Motion_Tick()` reads
the braking model on every tick of a turn, so a restore landing mid-arc would
have the robot finish braking on numbers that changed underneath it. Reply is
`RESEND`; retry once `?STAT` reports `busy` 0.

**3. Out of range is refused, not clamped.** A value the learner itself would
throw out is a sender bug, and clamping would hide it behind an `OK`. Limits
are 50–3000 dps², 0–250 ms, and ±80 µs.

> Rules 2 and 3 both answer `RESEND`, and the wire cannot tell you which. The
> sender disambiguates with `?STAT`; on the robot, **OLED mode 11 (CAL)** keeps
> them as separate counters and shows the three live values in these same
> scaled integers. Worth watching during a calibration run — the console
> latches off the moment the host speaks, so there is nothing else to look at.

### A sequence that works

```
!PROF0            pick the profile the tasks will actually use
F80               a straight ≥ 800 mm — the trim discards the first 0.5 s as
                  launch transient, then wants 0.5 s of steady driving, so it
                  needs ~1.0 s of HOLDING. F<n> is CENTIMETRES (§4)
FR90              four arcs, alternating so the robot returns to its
RR90              start heading and stays on the same patch of floor
FR90
RR90
?CAL              after each — stop when decel and lag stop moving
```

Then store the `CAL` reply against the surface, battery and weight it was
taken on, and push it back with the three setters after the next power-on to
skip the warm-up entirely.

> **A short straight fails silently.** Below ~1.0 s of holding the firmware
> leaves the trim exactly as it was and says nothing, and a trim that did not
> move is indistinguishable over the wire from one that has converged — so a
> too-short run gets stored in a profile as though it had been measured. The
> firmware knows which it was, but `?CAL` has no field to carry it and adding
> one is a protocol bump; for now the robot reports it on **OLED mode 1**,
> which prints `SKIP` in place of the trim delta. Give the straight margin
> rather than relying on spotting it.

### Two health checks, free

**`trim` near zero.** `SERVO_CENTER_US` now carries the measured steering
centre, so the trim has nothing left to absorb and should hover within a few
µs of zero. A trim walking steadily away from zero means the centre itself has
moved — a mechanical change, not something to calibrate around.

**`?XCHK` OK on profile 1.** The encoder-derived angle shares no hardware with
the gyro. Run one `!PROF1` arc at the end of the sequence and check it.

> **Calibration must never be able to hide a fault.** Both checks exist so a
> converged-looking `CAL` reply cannot cover for a linkage that has come
> loose. If either fails, the answer is a spanner, not a number.

> **v4 note on `?XCHK`:** the cross-check only runs on a profile with no
> differential boost, and TIGHT has 2.0, so on a TIGHT-locked robot it reports
> 0. The gyro is checked against the floor instead — §7.1, step 2.

### 7.1 Frozen calibration and heading carry-over (protocol 4)

**Learning is off at power-on.** The arc decel/lag learner and the trim
learner run only between `!LEARN1` and `!LEARN0` (and during the OLED CALIB
cycle, which switches it on for itself). During a task the robot drives on
exactly what was restored — nothing a task does can change it.

| Send | Effect | Reply |
|---|---|---|
| `!CALG<n>` | Gyro scale, **×10000**, 9000..11000 (0.90..1.10). Same busy/range rules as §7 | `OK` / `RESEND` |
| `!CALSL<n>` / `!CALSR<n>` | Left / right arc deflection from centre, **µs**. Left 300..574, right 300..626 (servo limits). **v5** | `OK` / `RESEND` |
| `?CAL` | now `CAL,<decel_x10>,<lag_ms_x10>,<trim_us>,<gyro_x10000>,<learn>,<steer_left_us>,<steer_right_us>,<last_decel_x10>` (steering v5; `last_decel_x10` v6: the last arc's raw decel measurement, 0 if it gave none) | |
| `?HDG` | `HDG,<commanded_x10>,<actual_x10>,<last_aim_x10>,<last_turned_x10>,<capped>` | |
| `?ENC` | `ENC,<counts_a>,<counts_b>` cumulative per-wheel encoder counts since boot; only differences mean anything. **v7** | |
| `!IRE<n>` | Arm the IR edge latch: `n = side*10000 + lo*100 + hi` (side 1 left / 2 right, 10 ≤ lo ≤ hi ≤ 80 cm), e.g. `!IRE11227` = left, 12..27 cm. `!IRE0` disarms. While a primitive runs, the first 3 consecutive ticks in the window latch that side's reading and the pose of the first of them; the move carries on. Arming clears the old latch. Plain `OK` on the movement path - send it before the line it watches. **v9** | `OK` / `RESEND` |
| `!FAST1` / `!FAST0` | Fast mode on/off: straights ramp up to 200 rpm (not 100), ramp down to the usual 40 rpm approach, and every move settles 200 ms after braking (not 400); heading-hold gains scale down with speed above 100 rpm. Off at power-on; arcs unchanged. Plain `OK` on the movement path, like `!LEARN`. **v8** | `OK` |

**`?WPOSE`, the continuous pose** - the same increments as `?POSE`, never
reset. Frame, Task 1 usage and host test: §12.

**The gyro scale** corrects the datasheet sensitivity (32.8 LSB/dps is
nominal). A scale error is invisible to the robot — the arc termination, the
braking model and `?TURN` all read the same gyro — so it can only be measured
against the floor. `calibrate.py guided` does that with two full circles.

**Per-side steering (v5).** The same pulse either side of centre does not
give the same wheel angle, so left and right turns trace different circles
even though the gyro stops both at the right angle. Each side's deflection is
its own calibration value; `calibrate.py radius` measures each side's radius
with a floor chord and adjusts them until both match the planner's 29.1 cm.
Until something is restored, both sides use the symmetric 574 µs.

**Heading carry-over.** The firmware keeps two totals that survive between
moves: *commanded* (every arc angle asked for since `!ZERO`) and *actual*
(everything the gyro measured). Each move aims at the commanded total:

    arc       turns  asked + (commanded - actual)
    straight  holds  (commanded - actual)

So an `FR90` that stopped at 89.5° is followed by one that turns 90.5°, and a
straight after it steers the half degree out. Error never builds up past one
move's worth. The correction is capped at **5°**; beyond that it is a bump or
a slip, not braking error, and `?HDG`'s last field counts every capped move.
`RST` and a `FAIL,*` move reset commanded to actual.

From `?HDG`: `commanded - actual` is the error the next move will correct;
`last_turned - last_aim` is how precisely the last arc stopped.

**Task startup** (`communications/cal_profile.py: prepare_for_task`):
`!LEARN0`, `!PROF<n>` for the profile the calibration was taken on (0 TIGHT,
3 SAFE; older saves are TIGHT), restore the values from it and read
them back, then `!ZERO` at the start pose.

---

## 8. What the path planner must know

**The chassis is Ackermann. It cannot turn on the spot.** Every turn is an arc
with forward or reverse travel, and the swept area has to be planned for.

Measured turn radii, by floor chord:

| Profile | Radius |
|---|---|
| TIGHT (0) | **291 mm** |
| SLOW (2) | 306 mm |
| CLEAN (1) | **318 mm** |

A 90° turn at 291 mm moves the robot roughly 291 mm forward **and** 291 mm
sideways. In a 2.0 m arena that is a seventh of the floor per turn.

**Turns terminate on measured heading, not arc length.** Speed does not make
the angle wrong — it only changes how much floor the turn eats.

**Distance during an arc is not honest.** On profiles 0 and 2 the rear tyres
are deliberately scrubbed to tighten the turn, so `?DIST` over-reads during a
turn. The *angle* is unaffected, because the gyro measures the body directly.

Robot footprint is roughly **18.8 cm wide by 23 cm long**.

---

## 9. Reserved — not implemented

The parser returns a parse failure for these, so a line containing one gets
`RESEND`.

**Task 2 movement:** `FIR` / `FIL` forward until the right/left IR loses the
wall, `FIRO` / `FILO` forward until it gains one, `SR` / `SL` diagonal slide.

`FU<n>` is no longer reserved — it is implemented, see §4.1. The reserved
`us{d1},{d2}` reply was **not** adopted with it: one reply per line is the rule
everything else rests on, and `FU` can sit mid-line behind other primitives, so
a bespoke data reply would either arrive out of order or make one line produce
two replies. `FU` answers `OK` like any other movement, and `?US` gives the
number that reply would have carried.

**Streaming:** `!STREAM<hz>` would push sensor data unsolicited instead of
polling. Deliberately not implemented — with no flow control, unsolicited
traffic while the Pi is busy overruns its buffer and can fragment the next
reply. **Poll instead.** At 115200 a query round trip is about 12 bytes, so
even 50 Hz uses under 1% of the link. If a genuine need for streaming appears,
it will be opt-in and off by default.

---

## 10. Recommended bring-up order

Each stage isolates one failure. Do not skip ahead — a failure at stage 5 is
meaningless if stage 3 was never proven.

| Stage | Action | Pass |
|---|---|---|
| 1 | Plug in, check the device appears | New `/dev/tty*` on plug |
| 2 | Open port, press RESET on the STM | Startup banner arrives, readable |
| 3 | Send `S` | `OK`. Nothing moves |
| 4 | Send `XYZ` | `RESEND`. Nothing moves |
| 5 | Send `?VER` | `VER,...`. Proves queries |
| 6 | Send `F10`, clear floor | Moves ~10 cm, `OK` **after** it stops |
| 7 | Send `FR90,F20,S` | **Exactly one** `OK`, at the end |
| 8 | `F100`, then `RST` mid-move | Immediate brake, **no reply** |
| 9 | `?US` with a box ~50 cm ahead | `US,50`-ish. Proves the sensor before trusting `FU` |
| 10 | Send `FU20`, box ~1 m ahead | Approaches in stages, `OK` after ~4 s, `?US` reads 18-22 |
| 11 | Send `FU20` with the sensor unplugged | `FAIL,NOECHO`. **Nothing moves** |
| 12 | Loop a realistic sequence 20× | `rx` on OLED mode 8 stays **0** |

**Do stage 2 before anything else.** The firmware prints a banner and run
reports on this port while no host is connected; the first valid command
latches that off permanently so it cannot corrupt the reply stream. Reset with
the host quiet to get it back.

Stages 3–5 involve no motion at all. If they pass and stage 6 fails, the
problem is motion, not the link.

---

## 11. Timeouts

The firmware's per-primitive watchdog is **15 s**. A 360° arc on the slowest
profile is the longest legitimate move; anything beyond 15 s is a stall.

**`FU<n>` is the exception, and it breaks the old 8 s assumption.** It is
several primitives plus up to four 400 ms settles, so the watchdog covers each
*leg* while the **line** runs much longer. Worst case measured in simulation —
`FU5` from 2 m, the longest legal approach — is **8.6 s**, which is already
past the 8 s figure `commands.h` declares. A sender that gives up at 8 s will
abandon a line the firmware is still working on, and every reply after that
lands against the wrong command.

Size the read timeout off the **line**, not a single move: 20 s.

Senders should use a read timeout **comfortably above** that — 20 s — and treat
expiry as a lost link rather than a slow move. A sender that blocks forever
waiting for a reply cannot recover from a dropped connection.

## 12. Task 1 instruction feedback

Task 1 keeps the PC's segment/photo boundaries but sends each instruction as
its own line. For a segment `FR90,F20,S`, the wire exchange is:

```text
RPi -> STM: FR90
STM -> RPi: OK
RPi -> STM: ?WPOSE
STM -> RPi: WPOSE,<x_mm>,<y_mm>,<heading_degrees_times_10>
RPi -> Android: ROBOT,<grid_x>,<grid_y>,<heading_deg>
RPi -> STM: F20
... repeat for F20, then S ...
```

Only after the last instruction's OK and pose update does the RPi photograph
the segment's obstacle. RESEND retries only the outstanding instruction.
FAIL, missing OK, or missing pose halts execution before another instruction.
`OK` and `WPOSE` are separate replies, preserving the existing serial parser.

`?WPOSE` is an additive capability and requires updated STM firmware. It uses
encoder/gyro increments continuously across primitive and FU-pass resets;
`?POSE`, `?TURN`, and motion control keep their existing local semantics.
The boot frame has +x forward, +y left, and positive counterclockwise heading
with the gyro source. The RPi snapshots this frame before the first instruction
and anchors it to the `PATH.odometry_start` supplied by the PC planner. By
default this is the planner's calculated rear-axle start; it is not a fixed
`(0,0)` or `(2,2)`. The arena is 20 x 20 cells,
each 100 mm (10 cm), for a total of 2000 x 2000 mm. Android receives fractional
grid coordinates via position_mm / 100 (0 <= coordinate < 20), and the nearest
heading in degrees, where North is 0 and clockwise is positive. For example,
(275 mm, 325 mm) is `ROBOT,2.75,3.25,0` when facing North. Raw WPOSE values
stay in logs. Positions outside
the arena stop execution rather than being clamped to a misleading valid cell.
These coordinates are dead reckoning and can drift. They are not planner poses
or external localization. Keep the same physical pose reference when placing
the robot at the start; firmware integrates displacement at the rear axle.

The checked-in STM source is protocol 7 (v5, plus the raw decel field in `?CAL` in v6 and `?ENC` in v7) and includes `?WPOSE` alongside the
frozen calibration, per-side steering and gyro bias fix (§7.1). A protocol 5
board flashed before `?WPOSE` was added answers `RESEND`, and Task 1 stops
before its first movement - reflash from this tree. Task 1 probes WPOSE
before sending its first movement.

Host regression checks (no hardware):

```sh
cc -Istm/tests -Istm/PeripheralDrivers/Inc stm/tests/test_world_pose.c \
  stm/PeripheralDrivers/Src/odom.c stm/PeripheralDrivers/Src/commands.c \
  -lm -o /tmp/mdp-test-world-pose
/tmp/mdp-test-world-pose
python3 -B -m unittest rpi/mdp_rpi/test_instruction_mission.py -v
```

## 14. Task 2 on the STM (protocol 11)

The whole Task 2 course runs on the STM: approach obstacle 1, the S past it,
the turn round obstacle 2 on the side IR, home and into the carpark on the
ultrasonic. The Pi only starts it and answers the arrows. Ported from the
MDP_WORKING_2 firmware, whose car ran this course end to end
(`task2_mission.c`); the robot side is `task2fw.c`, the constants
`task2_params.h`, the Pi side `rpi/mdp_rpi/task2fw.py`.

**Every move the course makes is an ordinary `Motion_DriveDistanceV` /
`Motion_DriveArcV`**, so it runs on the calibration table (protocol 10)
exactly like `F` / `FR` / `FL` sent over the wire. Restore it first
(`cal_profile.prepare_for_task`). The course plans every turn on the table's
`radius` for FL / FR at the arc speed, or 30.0 cm if the table has none, and
does not re-measure it during the run.

| Direction | Line | Meaning |
|---|---|---|
| RPi → STM | `!T2GO<s>` | Run the course, s = mission seconds 1..180. **Alone on its line**, refused with `RESEND` while anything else is running. One reply when it ends |
| STM → RPi | `P,<n>` | Photograph obstacle n (1 or 2). Sent before the robot drives up to it, repeated every 1.5 s until answered |
| RPi → STM | `!T2A<n><d>` | Obstacle n's arrow, d `0` left / `1` right. **No reply.** Send it again if the prompt repeats |
| RPi → STM | `!T2Y<n><ddd>` | Optional: where that arrow sat sideways in the photo, ddd − 500 mm from the camera's centre line, left positive (`!T2Y1580` = 8 cm left). Before its `!T2A`. No reply |
| STM → RPi | `D,<step>,...` | The run log, one line per decision, ending `x,y,heading` in cm / degrees from the start pose |
| RPi → STM | `RST` | Abort: brake now. The `!T2GO` line then replies `FAIL,STOP` |

Replies to `!T2GO`:

| Reply | Meaning |
|---|---|
| `OK` | Parked |
| `FAIL,STOP` | `RST`, or a long press on the board |
| `FAIL,DEADLINE` | The `<s>` seconds ran out - the STM stops on its own, whatever the Pi is doing |
| `FAIL,NOARROW` | No `!T2A` for a prompted obstacle within 20 s. **The STM never guesses** |
| `FAIL,NOECHO` | No usable ultrasonic reading for an approach |
| `FAIL,NOEDGE` | Drove 200 cm along obstacle 2 without the side IR finding its end |
| `FAIL,GYRO` | The gyro was not ready, or went stale mid-run |
| `FAIL,TIMEOUT` / `WRONGWAY` / `SHORT` | A move failed, as for a movement line (§3) |

While the course runs, **queries are still answered** (`?US`, `?WPOSE`, ...);
`!` setters and movement lines get `RESEND` - nothing else moves the robot,
and nothing changes its calibration between two of the course's moves.

`!T2A` / `!T2Y` outside a run answer `RESEND`. Without `!T2GO` nothing else in
the protocol changes.

The arrow names its obstacle so a late repeat for obstacle 1 can never be
read as obstacle 2's.

### Side IRs: obstacle 2 and the carpark

The side IRs are used the way the RPi's Task 2 uses them (`task2mn_park.py`,
`task2mn_gordon.py`), with their constants in `task2_params.h`:

- **Per-side ranges.** A reading counts only inside 10..65 cm on the left and
  10..50 cm on the right (the right unit is the weaker one). A no-reading
  with a high raw count is "closer than 10 cm", never "clear".
- **Obstacle 2's end and width (Gordon's face scan).** The leg along
  obstacle 2's face ends where the side IR loses it; that point, moved back
  `T2_IR_LAG_CM` for the sensor's lag and forward to the lens, is obstacle
  2's near end (`D,end2_y`). Mirrored about obstacle 1's line it gives the
  far end and the width (`D,width2`; `D,walls` is where the arena walls must
  be at least). With the ends known, the leg behind obstacle 2 stops past
  the far end even if the IR never sees the back face (`D,back_capped`, no
  `FAIL,NOEDGE`), and the way home keeps clear of obstacle 2 as well as
  obstacle 1, driving on along the leg first if the turn home would clip
  its end (`D,home_clear2`).
- **Centring at the carpark mouth (`task2mn_park` `_park_from_mouth`).** At
  the entry pose the ultrasound finds the back wall (`D,bay` = the mouth's
  x). The robot drives in until the side IRs are 5 cm inside the mouth and
  reads its offset from the side walls: half the difference of the two,
  else from one wall, else from an IR inside its 10 cm floor (`D,bay_both`
  / `bay_left` / `bay_right` / `bay_near_l` / `bay_near_r`, offset ×10 mm).
  Off by more than 4 cm, or more than 5° crooked, it backs out straight if it
  needs the room (`D,centre_back`) and makes one lane change onto the
  centreline (`D,centre`), then reads again. `D,parked_off` is the offset it
  parked at.
- **Side-safety stop.** Every forward move into the carpark stops if either
  side IR reads under 10 cm (`D,side_stop`), and the robot centres from there.
- **No IR shortcut at obstacle 2.** The 90 away from obstacle 2 is always
  finished; the face scan, with the IR square on to the face, decides where
  the end is. The original cut the 90 short when the IR read clear part-way
  round (`T2_WATCH_CUT 1` brings it back): on the floor (2026-09-14 13:22)
  the right IR read the black face as clear 21° off square from ~14 cm, the
  course took obstacle 2 for short, and its turn back (`D,cut_blind`, a 161°
  right) drove through it. A face scan that starts already past the end
  drives `T2_PAST_RUN_CM` on before the 180 (the end may be right at the
  lens), and one whose lens is further from the face than the IR reaches
  (`D,past_unsure`) goes on past the widest obstacle 2 there can be (90 cm).

- **Obstacle 2's ends, both ways.** A face scan that starts already past the
  end backs up along the face until the IR sees it (`D,seek_back`,
  `ir_seen_back`), so the end is read rather than assumed; the leg behind
  obstacle 2 reads the far end the same way. Each end read updates the span
  (`D,width2`) and where the walls can be (`D,wall_lo` / `D,wall_hi`).
- **The arena walls**, which can stand as near as 50 cm past either end of
  obstacle 2. Until an end is read the wall is taken where the narrowest
  obstacle 2 would put it. The 180 round the end starts with the axle
  `T2_TURN_AXLE_SHORT_CM` short of the read end and moves up to 20 cm sooner
  if the wall needs it (`D,wall_180`); the turn home moves along the leg, up
  to 30 cm either way, to clear both obstacle 2's end and the wall
  (`D,home_shift`); every route home is checked against the walls.
  `D,wall_tight` means nothing fitted and the turn went ahead as planned.
- **The carpark from its fences.** Before moving, the side IRs place the
  carpark's centreline (`D,bay_y`, ×10 mm); on the way out the first side IR
  to lose its fence marks the mouth (`D,mouth`). The way home aims at that
  centreline, straightens up with the nose 6 cm outside the mouth, and keeps
  off the fences' ends.

### Before a floor run

The course logic was proven in the original car's simulator and, re-derived
for this robot, in `stm/tests/test_task2_mission.c`: 1677 of 1728 arena
layouts with the planned 30 cm radius off by up to ±2 cm (6 collisions, the
rest swinging further than 90 cm past obstacle 2's end - see below), all 16
starts 5-8 cm off the bay's centreline parked within 4 cm of it, and with the
robot at the back of the carpark, obstacle 1 34 cm past the mouth and gaps of
56-105 cm (`-DSTART_TAIL_CM=2`), 132 of 144 with no collision - the same with
an IR blind to black more than 15° off square (`-DIR_OBLIQUE_DEG=15`), which
the old shortcut drops to 54. Measure on this robot, then update
`task2_params.h` and re-run that test:

0. **The arena walls.** In the simulator the robot swings up to ~96 cm past
   obstacle 2's end (shortest obstacle 2, the same arrow twice, squared up
   off the line). If the walls are closer than that, it needs a tighter round
   for that case.

1. **The turn radius** on the arc profile and speed Task 2 uses (SAFE),
   per side - `calibrate.py radius`, stored as the table's FL / FR radius.
2. **How far the side IR sees obstacle 2** (it is black), square on: the
   course wants a reading of `T2_IR_SEEN_CM` (32) or less from ~32 cm. If it
   only sees to 28 cm the simulator completes 1497 of 1728 (`-DIR_SEE_CM=28`),
   at 20 cm under half - the most sensitive number on the robot.
3. **The carpark** (`T2_BAY_DEPTH_CM` 50, `T2_BAY_HALF_CM` 30) and the left -
   right IR bias with the robot centred in it by hand (`T2_IR_LR_BIAS_CM`).
4. **How far a deliberate stop rolls** after `Motion_Stop()` at
   `T2_EDGE_RPM` and at arc speed - the simulator's `EDGE_OVERRUN_CM` /
   `WATCH_OVERRUN_DEG` / `GUARD_OVERRUN_CM` / `SIDE_OVERRUN_CM`, and the IR
   lag (`T2_IR_LAG_CM`, 2 cm, not measured).
5. The car's real outline (`T2_NOSE_*`, `T2_HALF_WIDTH_CM`).

Host test (no hardware; any C compiler, the RPi included):

```sh
cc -std=c11 -O2 -Wall -Wextra -Istm/tests -Istm/PeripheralDrivers/Inc \
  stm/tests/test_task2_mission.c stm/PeripheralDrivers/Src/task2_mission.c \
  stm/PeripheralDrivers/Src/commands.c stm/PeripheralDrivers/Src/caltab.c \
  -lm -o /tmp/test_task2_mission && /tmp/test_task2_mission
python3 -B -m unittest rpi/mdp_rpi/test_task2fw.py -v
```
