# Turning, Displacement Control and Return-to-Start

Analysis notes for the Ackermann turn problem, the RPi-side navigation
question, and the return-home strategy. Companion to `README.md`.

**Status of the numbers in this document:**

| Mark | Meaning |
|---|---|
| **[M]** | Measured on this robot, from `README.md`. Trust it. |
| **[D]** | Derived — geometry or simulation built on **[M]** values. Trust the shape, verify the magnitude. |
| **[C]** | Read from the code. Verifiable by inspection. |
| **[?]** | Unverified assumption. **Must be measured before it is relied on.** |

Nothing in this document has been run on the robot.

---

## 1. Can the turn radius be made tighter?

**No, not as a single arc.** The steering linkage is saturated.

### Steering is at the wall

| Pulse | Steer angle | Kinematic R |
|---|---|---|
| 575 µs (`MOTION_ARC_STEER_US`) | 26.8° **[M]** | 337 mm **[D]** |
| 600 µs (`SERVO_MAX_US`) | 27.0° **[D]** | 333 mm **[D]** |
| 625 µs (mechanical stop) | 27.3° **[D]** | 330 mm **[D]** |

Fitting the measured saturation (2.9× pulse buys 1.24× angle **[M]**) gives an
exponent of 0.20. Spending all remaining travel to the mechanical stop buys
**1.7% more steer angle** — at full stall torque against the HWZ020's plastic
gears. Not worth it.

For reference: R = 250 mm needs 34.2°, R = 200 mm needs 40.4° **[D]**. Neither
is in the linkage. The only route to a genuinely tighter arc is **changing the
steering linkage geometry** (shorter track-rod arm or longer servo horn), which
is a hardware change and risks the front tyres fouling the chassis.

### diff_boost has less headroom than it looks

`Odom_DriveArc()` clamps `ratio` at `ODOM_ARC_DIFF_MAX 0.50` **[C]**
([odom.c:401](PeripheralDrivers/Src/odom.c:401)):

```
boost 1.0 (CLEAN): ratio 0.218   inner wheel 0.78x
boost 1.5 (SLOW):  ratio 0.327   inner wheel 0.67x
boost 2.0 (TIGHT): ratio 0.436   inner wheel 0.56x
boost 2.29:        ratio 0.500   inner wheel 0.50x   <- CAP
boost 3.0:         ratio clamped to 0.500 - DOES NOTHING
```

Anything above **boost 2.29 is silently a no-op** at R = 291 **[D]**.

Extrapolating the two measured points (ratio 0.218 → 318 mm, ratio 0.436 →
291 mm **[M]**) linearly gives **~283 mm at the cap [D]** — about 3% for an
inner wheel now at 50 rpm during the approach phase, below the ~55 rpm floor
where the PID stops regulating. That is the freewheeling-inner-wheel skid
already documented in [motion.h:128](PeripheralDrivers/Inc/motion.h:128).

To spend it you would have to raise `approach_rpm` to ~120, costing stopping
repeatability.

> **Single-arc floor is ~283 mm, from 291 mm. Not worth the trade.**

---

## 2. Multi-segment turns: the real lever

The chassis cannot turn on the spot, but it can shuffle. Forward arc then
reverse arc, **both rotating the body the same way**, cancels most of the
translation.

### The pairing — easy to get wrong

`right` in `Motion_DriveArc()` is the **servo** direction, not the body's
**[C]** ([motion.c:293](PeripheralDrivers/Src/motion.c:293)).

```
forward + right  ->  body rotates RIGHT
reverse + right  ->  body rotates LEFT
reverse + left   ->  body rotates RIGHT
```

**To continue a rightward turn after `FR`, use `RL` — not `RR`.**
`README.md`'s opcode table ("RR = arc reverse-right") invites the mistake; the
comment at [motion.c:320](PeripheralDrivers/Src/motion.c:320) has it right.

### Segment count vs floor eaten

90° right, rear-axle reference, R = 291 mm. All **[D]**:

| Plan | Net displacement | Body swept box | Drive length |
|---|---|---|---|
| `FR90` | 412 mm | ~480 × 560 mm | 457 mm |
| `FR45,RL45` | 170 mm | ~450 × 450 mm | 457 mm |
| `FR30,RL30,FR30` | 110 mm | ~410 × 410 mm | 457 mm |
| `FR22,RL22,FR22,RL22` | 82 mm | ~400 × 400 mm | 457 mm |

**Drive length is identical in every row.** Same rotation, same radius — you
are not driving further, only over less floor. The cost is dead time, not
distance.

### Aiming the residual — the useful part

The split angle does not just shrink the residual, it **points** it. For
`FR{a},RL{90-a}`, all **[D]**:

| Plan | Net advance | Net lateral | Use when |
|---|---|---|---|
| `FR90` | +291 | 291 right | you have the room |
| `FR60,RL30` | **+213** | **0** | must not drift sideways |
| `FR45,RL45` | +121 | 121 left | minimum magnitude (170 mm) |
| `FR30,RL60` | **0** | **213 left** | must not advance |
| `FR15,RL45` | −140 | 271 left | need to back off while turning |

Counterintuitive but correct: **a right turn can leave you displaced left.**
The reverse segment drags you back and across.

### Four segments: 2D placement, and rotation in place

Three free parameters gives a reachable *region*. With a 15° floor on segment
size **[D]**:

```
advance  -73 .. +175 mm      lateral  -74 .. +172 mm
```

And net-zero is reachable:

> **`FR15,RL30,FR30,RL15` → 15 mm residual [D]**
> A turn on the spot, on a chassis that cannot turn on the spot.

This is the enabler for go-to-goal navigation: it decouples rotation from
translation, reducing the Ackermann problem to the differential-drive one. The
floor-space saving is the lesser benefit.

### Costs and risks

- **~0.6 s of dead time per extra segment [D]** — `MOTION_BRAKE_TICKS 40`
  (400 ms) plus align (200 ms) **[C]**.
- **The swept footprint barely shrinks** (480×560 → 400×400 **[D]**). The
  robot's own 188 × 230 body sweeping 90° sets a floor near 400 mm.
  Segmenting controls **where you end up**, not **how much space you need
  while getting there**. Obstacle 30 cm to the side mid-turn? Segmenting will
  not save you.
- **Robust to radius error**: 5% forward/reverse mismatch costs only 11 mm on
  `FR45,RL45`, 3 mm on the four-segment plan **[D]**. Errors partly cancel.
- **`MOTION_ALIGN_TICKS 20` is probably too short [C][?]**. It allows 100 ms
  for the servo to travel ~635 µs (≈63° of servo rotation) between segments. A
  typical hobby servo needs ~160 ms. Every segment after the first would launch
  mid-swing and run wide. **Visible on the bench without instrumentation —
  watch the wheels, not the numbers.**
- **Short segments never leave the approach phase.** `approach_deg 20` **[C]**
  means a 22° segment runs entirely at `approach_rpm`, not cruise. Slower turns
  tighter, so segment radius ≠ 291 **[?]**.

---

## 3. What the firmware actually knows — the critical finding

> **The STM keeps no state across primitives. There is no global pose and no
> absolute heading.**

`Odom_Reset()` is called from `motion_launch()` at the start of **every**
primitive **[C]** ([motion.c:228](PeripheralDrivers/Src/motion.c:228)), and
`Odom_Reset()` itself calls `IMU_ResetHeading()` **[C]**
([odom.c:148](PeripheralDrivers/Src/odom.c:148)).

So all three heading/pose references reset per move:

| Query | Meaning | Reset when | Scaling |
|---|---|---|---|
| `?POSE` x, y | displacement **within current primitive** | every primitive | mm |
| `?POSE` heading | wrapped ±180 | every primitive | ×10 |
| `?TURN` | unwrapped rotation | every primitive | ×10 |
| `?IMU` heading | unwrapped | **every primitive** | ×10 |

`IMU_ResetHeading()` also has a second caller at `main.c:944` (the mode-9
diagnostic button) **[C]**.

The design rationale is sound — heading hold, cross-track and arc termination
all depend on the per-move zeroing, and `imu.h` is explicit that gyros are
"excellent for seconds" and the firmware only ever needs seconds **[C]**. But
it means **any global pose must live on the RPi.**

### Consequences for the RPi

- **One primitive per line** if you want pose feedback. `FR45,RL45` returns one
  `OK` and the intermediate state is lost forever.
- **Queries are answered immediately, without touching the queue, and are safe
  to ask mid-move [C]** ([rpilink.c:418](PeripheralDrivers/Src/rpilink.c:418)).
  This is the enabler for reactive control.
- **A query must be alone on its line.** `F20,?US` is rejected **[C]**.
- **`?POSE` x/y is dishonest during an arc.** `diff_boost 2.0` scrubs the rear
  tyres so encoders over-read **[M]**. For arcs take the *angle* from `?TURN`
  and compute displacement **geometrically** from (R, θ). Or use CLEAN
  (boost 1.0) when the endpoint matters — which is why CLEAN exists
  ([motion.c:46](PeripheralDrivers/Src/motion.c:46)).

### Optional firmware fix: `?GPOSE`

Accumulate into a global pose immediately *before* zeroing in `Odom_Reset()` —
rotate current (x, y) into the global frame and add; accumulate
`s_headingTotal` into a `s_globalHeading`. Add a `?GPOSE` query. ~20 lines.

Touches nothing that matters: the per-move zeroing the control loops depend on
stays exactly as it is; this only adds an accumulator alongside. Buys a real
global pose *and* lets you queue multi-primitive lines again.

**Only needed for the direct-return strategy (§6). Not needed for retrace.**

---

## 4. Do not adapt for battery or friction on the RPi

Already handled, and RPi compensation would fight it.

- `s_arcDecel` and `s_arcLag` are learned live, per run, from the measured
  coast **[C]**.
- Steering trim is learned between runs **[C]**.
- Zero-rate gyro bias is re-tracked every time the robot stands still
  (`IMU_TrackBias()`, gated on no motion) **[C]** (`main.c:249`).

The documented design rule: *adaptation is confined to what genuinely varies —
friction, battery, weight; geometry and sensor scale are measured once and
frozen; adaptation must never mask a calibration error* **[M]**.

> **RPi owns geometry. STM owns execution.**

Two adaptive controllers in series with no visibility into each other makes a
bad turn undebuggable. What the RPi *should* read is `?XCHK` and `?STAT` — to
detect that execution went wrong, not to pre-compensate.

### Correcting the knob list

| Quantity | Controllable? |
|---|---|
| **θ (turn angle)** | Free, any degrees, gyro-terminated. **Primary knob.** |
| **R (radius)** | Three discrete profiles only (291 / 306 / 318), via `!PROFILE`. Not continuous. |
| **Floor displacement** | Derived — via the segmentation of §2. |
| **Arc length** | **Not a knob.** It is R × θ, a consequence. |

---

## 5. Return to start: retrace

**The recommended strategy.** Needs no global pose, no wall referencing, no
position fix — just a list on the RPi.

### Every primitive's inverse is the same steering, driven backwards

Verified: reverse with the *same* servo deflection sweeps the *same circle*
(identical ICC) **[D]**.

| Outbound | Inverse |
|---|---|
| `F{n}` | `R{n}` |
| `FR{n}` | `RR{n}` |
| `FL{n}` | `RL{n}` |

> **Not the same pairing as §2.** There, `FR`/`RL` *continues* a rotation.
> Here, `FR`/`RR` *undoes* one. Same opcodes, opposite purpose.

Push each executed primitive onto a stack; to go home, pop and swap the F/R
prefix. That is the whole algorithm.

### Why it beats navigating home

Simulated 6-primitive outbound leg and its retrace **[D]**:

```
outbound ends   +1437.8, -967.2 mm, heading -45 deg
after retrace        0.0,     0.0 mm, heading   0 deg
```

**Systematic turn bias cancels exactly:**

| Error injected | Residual at home | Heading error |
|---|---|---|
| every arc overshoots 1° | 0.0 mm | 0.00° |
| every arc overshoots 2° | 0.0 mm | 0.00° |
| every arc overshoots 3° | 0.0 mm | 0.00° |

The same bias applied in the opposite sense undoes itself. Much of the ±1° turn
error is systematic (seeded decel/lag, backlash resolution), so it evaporates.
Dead reckoning home instead multiplies that same bias by every leg length.

**And the path is known clear** — you just drove it. Satisfies "must not hit
any obstacle" by construction, with no avoidance logic on the return.

### The subtlety that decides whether it works

> **Replay the COMMANDED angle, not the measured one.**

If `FR90` actually turned 92°, send `RR90` — **not** `RR92`. Commanding 90 both
ways is what makes the bias cancel; commanding the measured value re-applies
the bias on top and leaves you 2° out.

**Exception:** sensor-terminated primitives. After `F0` you do not know how far
you went — read `?DIST` and push `R{that}`. Straight-line error is 0.3% **[M]**
and not a systematic angular bias, so replaying the measured distance is
correct there.

### The one error term that survives

Forward and reverse arcs having different radii **[?]**. It does not touch
heading at all — turns are gyro-terminated, so heading comes home at 0.00°
regardless — but it displaces you **[D]**:

| Reverse radius vs forward | Residual at home |
|---|---|
| +3% | 31 mm |
| +5% | 52 mm |
| +10% | 103 mm |

Even 10% mismatch lands 10 cm from start with the heading exactly right.

### Caveats

- **Reverse arcs have never been run on the floor.** `RR`/`RL` are implemented
  **[C]** ([rpilink.c:318](PeripheralDrivers/Src/rpilink.c:318)) and the sign
  logic is commented *"VERIFY THIS ON THE ROBOT"*
  ([motion.c:320](PeripheralDrivers/Src/motion.c:320)). The whole return plan
  rests on them. `MOTION_ARC_WRONGWAY_DEG 20` aborts a wrong-sign turn in about
  a second, so failure is safe — but find out on the bench.
- **Reversing is blind.** Ultrasonic is forward-only, IRs face sideways
  **[M]**. Acceptable over a path just driven in a static arena. Not acceptable
  if anything moves.

---

## 6. Direct return — possible, but not the time saving you want

### The timings

Representative outbound leg (2.09 m path, 6 primitives, ending 1.73 m from
home). All **[D]**, using measured speeds:

| Return strategy | Time | Saving |
|---|---|---|
| Retrace | 10.8 s | — |
| Direct, single wide arc to turn around | 8.6 s | 2.2 s (20%) |
| Direct, near-zero-displacement turn | 10.7 s | 0.1 s |
| Direct, chunked into 40 cm hops for safety | 15.7 s | **−4.8 s (slower)** |

Turning 169° to face home costs about what the retrace's extra distance costs.
And chunking to poll the ultrasonic — which you must, because **`F{n}` has no
obstacle abort; only `F0` sets `s_f0Active` [C]**
([rpilink.c:304](PeripheralDrivers/Src/rpilink.c:304)) — spends more than it
saves.

### Where the time actually is

```
baseline retrace                10.8 s
  dead time (0.6 s x 6)          3.6 s   33%
  straight approach taper        3.3 s   30%
  actual cruising                4.0 s   37%
```

**Two thirds of the run is the robot going slowly or standing still.**

| Change | Saving **[D]** |
|---|---|
| `MOTION_APPROACH_RPM` 40 → 70 | 1.4 s (13%) |
| `MOTION_BRAKE_TICKS` 40 → 20 | 1.2 s (11%) |
| `MOTION_CRUISE_RPM` 100 → 300 | 1.0 s (9%) |
| `MOTION_APPROACH_MM` 150 → 80 | 0.9 s (8%) |
| arc rpm 150 → 220 | 0.5 s (5%) |
| **all together** | **4.3 s (40%)** |

> **6.5 s instead of 10.8 s, with no new logic and no way to hit anything** —
> against 0.1–2.2 s for the direct return, which needs a pose estimate, a
> clear-lane search and a collision backstop.

`MOTION_CRUISE_RPM 100` sits in a usable range of 55–360 **[M]** — conservative
by a factor of three, yet it barely shows in the total because the approach
taper dominates. **`MOTION_APPROACH_RPM` is the better first move.**

### Free win regardless

**Queue the whole retrace in one line.** You already know the list, so you need
no per-primitive feedback. `RpiLink_Poll` starts the next primitive the moment
the previous finishes **[C]**, dropping every RPi round trip. Limits: 16
primitives, 128 bytes per line **[M]**.

### `MOTION_BRAKE_TICKS` deserves care

It is a hard 400 ms floor, ANDed with the recentre gate
([motion.c:570](PeripheralDrivers/Src/motion.c:570)) **[C]**. But the recentre
gate already waits for **both encoders to read zero** before it starts, then
150 ms for the servo. If the robot stops in 100 ms it is done at 250 ms and
then idles 150 ms waiting for the counter. Dropping to ~25 makes the
physically meaningful condition the binding one rather than a fixed timeout.

The other four trade against stopping accuracy, and A.3 currently passes at
0.0–0.3% **[M]**. **Change them one at a time with a tape measure.**

### When direct return *would* win

The saving scales with how convoluted the outbound path is. The simulated one
was only 1.2× the straight-line distance home. At **2× or more** the direct
return starts earning its complexity, and then it needs:

1. An approximate global pose → `?GPOSE` (§3)
2. Obstacle positions with margin for a clear lane (robot is 188 mm wide **[M]**)
3. Obstacle abort on `F{n}`, not just `F0` — roughly a two-line change, and the
   enabler for driving home fast without chunking

> **Measure your actual outbound path length against the straight-line distance
> home. That single ratio decides it, and you get it free from the first full
> run.**

---

## 7. Sensor reality for reactive avoidance

| Sensor | Facing | Valid span | Notes |
|---|---|---|---|
| Ultrasonic HC-SR04 | forward, one unit | 2–400 cm **[M]** | reads ~1.3 cm long (systematic, subtractable); `F0` nominal 15 cm stop is really ~13.7 cm **[M]** |
| IR GP2Y0A21YK ×2 | **one each side** **[C]** | 10–80 cm **[M]** | saturates below 10 cm and is refused; right unit quits past ~65 cm; thresholds are **per channel** (left reads ~20% higher) **[M]** |

The avoidance loop works: ultrasonic sees the obstacle ahead → arc out → side
IR tracks it going past → IR dropout (clears 80 cm) is the "turn back in"
trigger.

**The gap:** IR is blind below 10 cm — exactly when you are closest. Plan the
standoff around the IR floor, not around the 2 cm the ultrasonic can do.

**`Sensors_ObstacleAhead()` treats "no reading" as "obstacle"** **[M]** — fail
safe, but `F0` refuses to move whenever the echo is lost. A hazard for a long
uninterrupted drive home.

### Architectural fork for reactive control

- **RPi drives tight:** many short primitives with polling between. Works
  today, no firmware change. Costs ~600 ms dead time per primitive — a
  six-primitive avoidance burns ~3.6 s doing nothing.
- **Firmware owns the termination condition:** `FIR`, `FIL`, `FIRO`, `FILO` are
  already reserved in [commands.h:103](PeripheralDrivers/Inc/commands.h:103)
  and return `CMD_INVALID` **[C]**. They are precisely "arc until the IR says
  so". Implementing them puts the inner loop in the 100 Hz tick — no round
  trip, no dead time.

**Recommended:** the second. It is the same split that already works for arcs —
the RPi says *90° right*, the gyro decides *when*. Extend it: the RPi says
*arc out until you clear it*, the IR decides when.

---

## 8. Bench measurements needed

Ordered by what blocks the most downstream work.

| # | Measurement | Blocks | Method |
|---|---|---|---|
| 1 | **Reverse-arc radius**, floor chord | Retrace accuracy (§5) — on the critical path | Mark under rear axle, `RR90`, mark again, `R = chord / 1.414` |
| 2 | **Reverse arcs run at all**, correct sign | Everything in §5 | Bench `RR90` / `RL90`, watch for `WRONGWAY` abort |
| 3 | **Servo lock-to-lock time** vs the 100 ms align window | All multi-segment turns (§2) | Mode 6, or watch the wheels during `FR45,RL45` |
| 4 | **Short-segment radius** (30° arc, never leaves approach speed) | Segmented-turn predictions (§2) | Floor chord on a 30° arc |
| 5 | `FR45,RL45` net displacement vs the predicted 170 mm | Validates the whole §2 model | Chord method, compare against `FR90` |
| 6 | **Motion constants, one at a time** with a tape measure | 40% time saving (§6) | A.3 distance run after each change |
| 7 | **Side IR response** as an obstacle passes at arc speed | Reactive avoidance (§7) | Mode 7 live, drive an arc past a block |
| 8 | **Outbound path length ÷ straight-line distance home** | Decides retrace vs direct (§6) | Free from the first full run |

---

## 9. Proposed firmware changes

| Change | Size | Needed for | Priority |
|---|---|---|---|
| Raise `MOTION_ALIGN_TICKS`, or scale it with servo travel | tiny | Multi-segment turns (§2) | High if §2 is used |
| Tune motion constants (§6) | constants only | 40% time saving | **High — biggest win per effort** |
| Obstacle abort on `F{n}`, not just `F0` | ~2 lines | Safe fast return (§6) | Medium |
| `?GPOSE` global pose accumulator | ~20 lines | Direct return only (§3) | Low unless §6 ratio says otherwise |
| Composite turn primitive (firmware picks the split) | moderate | Convenience for §2 | Low |
| `FIR`/`FIL`/`FIRO`/`FILO` sensor-terminated arcs | moderate | Reactive avoidance without dead time (§7) | Medium |
| Reverse-arc radius bench mode (button UI) | small | Measurement #1 without the RPi link | Medium |

---

## 10. Summary of recommendations

1. **Do not chase a tighter single-arc radius.** 291 → 283 mm at best, and it
   costs inner-wheel regulation.
2. **Use segmented turns when the endpoint matters**, and remember they control
   *where you end up*, not the space you sweep. `FR15,RL30,FR30,RL15` gives
   rotation in place.
3. **Get the `FR`/`RL` vs `FR`/`RR` distinction right.** Continue vs undo.
4. **The STM is stateless across primitives.** Any global pose lives on the RPi.
5. **Return home by retracing.** Systematic error cancels exactly, the path is
   known clear, and it needs no pose estimate at all.
6. **For speed, tune the motion constants — not the return strategy.** 40% vs
   1–20%, at a fraction of the risk.
7. **Measure the reverse-arc radius first.** It is the one unknown the whole
   return plan rests on.

---

## Appendix: documentation fix for `README.md`

The opcode table currently reads:

| Token | Arg | Meaning |
|---|---|---|
| `RR{n}` | degrees | Arc reverse-right |
| `RL{n}` | degrees | Arc reverse-left |

"Reverse-right" reads as *body rotates right*, but the code means *servo steers
right*, and reverse + steer-right rotates the body **left**. Suggested wording:

| Token | Arg | Meaning |
|---|---|---|
| `RR{n}` | degrees | Reverse with steering RIGHT — body rotates **left** |
| `RL{n}` | degrees | Reverse with steering LEFT — body rotates **right** |
