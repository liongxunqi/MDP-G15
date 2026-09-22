# RPi ↔ STM32 Protocol — MDP Group 15

**Status: firmware side IMPLEMENTED, awaiting RPi agreement.**

Movement commands are unchanged from the frozen interface and already in use.
Queries, config and the `FAIL,*` replies are implemented in firmware but have
**not** been agreed with the RPi owner or tested on the wire. Treat §5, §6, §7
and the `FAIL,*` rows of §3 as a proposal until that happens — the firmware can
change if the RPi side wants different names or fields.

**Protocol version: 3** — ask for it with `?VER`.

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
| `RST` | — | Emergency abort: drop queue, brake now. **No reply** |

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
| `?POSE` | `POSE,<x_mm>,<y_mm>,<hdg_x10>` | Since the last `!ZERO` or move start |
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
healthy. `profile` is the active arc profile, 0–2.

> `busy` changed meaning in version 3. It used to track the motion layer
> alone, which was indistinguishable from the line for ordinary movement. It
> is not for `FU<n>`: that command spends most of its life deliberately
> **stopped**, settling between passes, and the old field read 0 in those
> windows. A sender polling `?STAT` to decide the robot had arrived would have
> carried on mid-approach. The reply is still the real completion signal —
> `?STAT` is for watching, not for sequencing.

**`?IMU` is the health check.** `ok` 0 means the gyro has gone stale and
heading has fallen back to the encoder difference, which is much less
accurate. `stalls` counts how many times that has happened. `peak` is the
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
| `!PROF1` | Arc profile **CLEAN** — radius 318 mm, no tyre scrub, cross-check valid | `OK` |
| `!PROF2` | Arc profile **SLOW** — radius 306 mm, gentlest | `OK` |
| `!ZERO` | Zero the odometry and heading | `OK` |

Profile persists until changed or reset. Default is **TIGHT**.

Pick **CLEAN** when the planner needs the robot to finish where it predicted —
it traces a truer circle. Pick **TIGHT** when floor space is the constraint.

---

## 7. Calibration

The firmware learns three things while it drives: arc deceleration, brake
engagement lag, and the steering centre trim. **All three live in RAM and are
lost at power-off**, so a cold robot spends its first three or four arcs
converging — the difference between a 9° error and a 1° one on the first turn
of a session.

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
ordinary primitives. The firmware never drives itself, so there is no mode
that could still be running when a task starts.

**2. The setters are refused while a move is running.** `Motion_Tick()` reads
the braking model on every tick of a turn, so a restore landing mid-arc would
have the robot finish braking on numbers that changed underneath it. Reply is
`RESEND`; retry once `?STAT` reports `busy` 0.

**3. Out of range is refused, not clamped.** A value the learner itself would
throw out is a sender bug, and clamping would hide it behind an `OK`. Limits
are 50–3000 dps², 0–250 ms, and ±80 µs.

### A sequence that works

```
!PROF0            pick the profile the tasks will actually use
F600              a straight ≥ 500 mm — the trim needs 50 samples to update
FR90              four arcs, alternating so the robot returns to its
RR90              start heading and stays on the same patch of floor
FR90
RR90
?CAL              after each — stop when decel and lag stop moving
```

Then store the `CAL` reply against the surface, battery and weight it was
taken on, and push it back with the three setters after the next power-on to
skip the warm-up entirely.

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
| 12 | Loop a realistic sequence 20× | `rx` on OLED mode 7 stays **0** |

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
