#ifndef __MOTION_H
#define __MOTION_H

#include "stm32f4xx_hal.h"

/* For SERVO_CENTER_US / SERVO_MIN_US / SERVO_MAX_US, which the arc deflection
 * is checked against below. */
#include "motors.h"

/* ---------------------------------------------------------------------------
 * Motion primitives. Non-blocking state machine, ticked from TIM6 at 100 Hz.
 *
 * Checklist A.3: traverse a straight line, stop at a distance between 80 and
 * 120 cm specified by the supervisor, within +/-6% of target, with no visible
 * deviation from the line.
 *
 * Nothing here blocks. Motion_DriveDistance() returns immediately; the run
 * happens in the tick. Motion_IsBusy() is the boundary the command layer
 * polls to decide when to emit "OK".
 *
 * CALL ORDER in the TIM6 ISR:
 *      Encoders_Update();
 *      Motion_Tick();      <-- sets the speed for this tick
 *      Odom_Update();      <-- applies it, steers
 *      PID_Update();
 * ------------------------------------------------------------------------- */

/* Cruise speed, RPM. Same value Phase 3 was tuned at. */
#define MOTION_CRUISE_RPM       100

/* Approach speed for the last stretch. Slowing before the target cuts the
 * spread in stopping distance, which is what +/-6% actually depends on -
 * a fast stop is not inaccurate, it is inconsistent.
 *
 * This was equal to MOTION_CRUISE_RPM, which disabled the taper entirely:
 * want never differed from s_lastRpm, so the speed change was never issued
 * and every run braked from full cruise. 40 RPM is a starting point; raise
 * it if the approach crawls for too long. */
#define MOTION_APPROACH_RPM     40
#define MOTION_APPROACH_MM      150.0f

/* Coast after braking, mm. Subtracted from the target so the robot ends up
 * on the mark rather than past it. See the procedure in motion.c.
 *
 * MEASURED, not guessed: five 1000 mm A.3 runs read 1008, 1009, 1008, 1008,
 * 1008 by odometry - +0.8% with 1 mm of spread. That is coast past the stop
 * trigger rather than a scale error, because odometry keeps counting all the
 * way through the coast.
 *
 * Coast grows with speed and with battery voltage, so re-measure after any
 * change to MOTION_CRUISE_RPM or MOTION_APPROACH_RPM - both of which are
 * candidates for the run-time tuning in TURNING.md section 6. */
#define MOTION_BRAKE_MM         8.0f

/* Brake settling time before declaring the move finished, in 10 ms ticks. */
#define MOTION_BRAKE_TICKS      40U

/* Watchdog. A stalled wheel must not hang the primitive forever, or it hangs
 * the RPi link with no reply and nothing to explain it. */
#define MOTION_TIMEOUT_TICKS    1500U   /* 15 s */

/* ---------------------------------------------------------------------------
 * ARCS  (checklist A.4: rotate through an angle between 90 and 360 degrees)
 *
 * This chassis is Ackermann and cannot turn on the spot, so a "rotation" is
 * an arc. The turn is terminated on MEASURED HEADING from the gyro, not on
 * arc length.
 *
 * That is a real simplification over the arc-length approach. Arc length
 * needs the turn radius to be known accurately, and the radius depends on
 * steering geometry, linkage slack, tyre scrub and speed - the checklist even
 * warns that a faster robot turns wider. Measuring the angle directly makes
 * all of that irrelevant: the robot turns until the gyro says it has turned
 * far enough, whatever path it took to get there.
 *
 * The turn radius no longer sets the angle either. Each profile carries its
 * own measured radius_mm, used only by Odom_DriveArc() to split the inner and
 * outer wheel speeds so the tyres do not scrub, and as the denominator of the
 * encoder cross-check. A rough value costs a little scrub, not accuracy.
 *
 * MOTION_ARC_STEER_US is the deflection from centre used for every arc. Keep
 * it inside SERVO_MIN_US..SERVO_MAX_US or Servo_SetMicroseconds() clamps it
 * and every arc comes out wide.
 * ------------------------------------------------------------------------- */
/* Deflection from centre used for every arc.
 *
 * 575 us. Confirmed steering travel is 850 to 2125 about a centre of 1500,
 * and SERVO_MIN/MAX_US are set symmetrically at 900/2100. 575 sits 25 us
 * inside both, so neither direction clamps.
 *
 * DO NOT judge the resulting radius until the gyro fix is flashed. The R
 * shown on the turn screen is arc_length divided by the gyro's angle, so
 * while the gyro was clipping and under-reading by about 1.8x, every R was
 * inflated by the same factor. Nothing measured before that fix says anything
 * reliable about what the steering is doing. */
#define MOTION_ARC_STEER_US     575U

/* Slow down for the last part of the turn, degrees remaining. Same reasoning
 * as MOTION_APPROACH_MM on a straight run: a slow approach makes the stopping
 * point consistent, and consistency is what accuracy depends on. */
/* Speed for arcs, separate from MOTION_CRUISE_RPM.
 *
 * Slower turns tighter. At speed the front tyres slip sideways rather than
 * following exactly where they point, so the robot understeers and traces a
 * wider circle than the geometry says - which is precisely what the checklist
 * warns about: "if speed of the robot increases, the robot will take a larger
 * turning angle".
 *
 * Terminating on measured heading means this cannot make the ANGLE wrong. It
 * only changes how much floor the turn eats. Lower it if the robot swings too
 * far across the arena. */
/* Which way the body rotates for a given steering side. MEASURED.
 *
 * On this robot a right turn gives a NEGATIVE heading change - confirmed by
 * watching the robot physically turn right while hd counted down to -20.
 *
 * +1 = right turn is negative heading   <- this robot
 * -1 = right turn is positive heading
 *
 * If a commanded turn runs away, or aborts as TIMEOUT having gone the wrong
 * way, flip this. But check IMU_IsReady() first - see the note in odom.c
 * about the encoder fallback, which used to invert the sign all by itself. */
#define MOTION_ARC_SIGN         (+1)

/* Abort if the turn goes the WRONG WAY by this many degrees.
 *
 * With the sign inverted, "degrees still to go" only ever grows, so nothing
 * terminates the arc and it runs until the 15 s watchdog - which on a floor
 * is a long way. This catches it in about a second instead. */
#define MOTION_ARC_WRONGWAY_DEG 20.0f

/* Approach speed for arcs, SEPARATE from MOTION_APPROACH_RPM.
 *
 * A straight-line approach can be slow because both wheels run at the same
 * speed. An arc cannot: the inner wheel is driven at a fraction of the centre
 * speed, and with the profile's diff_boost at 2.0 that fraction is about 0.56.
 *
 * At the straight-line figure of 40 rpm the inner wheel is commanded at 22 -
 * below PID_MIN_RPM, so the PID gives up and outputs zero and the wheel
 * FREEWHEELS. For the last stretch of every turn the robot then has one wheel
 * driving and one coasting, which is a skid rather than an arc and shifts the
 * robot sideways the same way every time. The gyro still reports the angle
 * correctly, so the fault is invisible on the display and only shows up as
 * the robot not ending where it should.
 *
 * 100 rpm keeps the inner wheel at about 56, just above the deadband floor
 * where it still regulates. Raise it if the boost goes higher. */
/* ---------------------------------------------------------------------------
 * ADAPTIVE BRAKING
 *
 * Coast is not an arbitrary constant, it is physics:
 *
 *      coast_angle = w^2 / (2 * alpha)
 *
 * w is the yaw rate at the moment the brakes go on, and the gyro already
 * measures it every tick. alpha is the angular deceleration - one number that
 * describes how hard this robot stops on this floor.
 *
 * So rather than subtracting a fixed brake_deg and hoping, the tick predicts
 * the coast from the CURRENT rate and brakes when the remaining angle drops
 * below it. That adapts to speed immediately and for free: a fast profile
 * brakes earlier because w is larger, with nothing to retune.
 *
 * Then alpha itself is learned. After every turn the actual coast is known -
 * the angle at brake onset against the angle it finished at - so
 *
 *      alpha_measured = w_brake^2 / (2 * actual_coast)
 *
 * and that is blended slowly into the stored value. Carpet, a flat battery,
 * a heavier robot after the Pi goes on: all of it shows up as a different
 * alpha and all of it is absorbed within a few turns.
 *
 * WHY THIS MATTERS HERE: brake_deg 8 was measured at approach_rpm 40. Raising
 * that to 100 to stop the inner wheel freewheeling roughly doubled the coast,
 * and the profile constant was silently stale. Every coupled parameter has
 * that problem. A measured alpha does not.
 *
 * Set MOTION_ARC_ADAPTIVE_BRAKE to 0 to go back to the fixed brake_deg. */
#define MOTION_ARC_ADAPTIVE_BRAKE   1

/* Starting guess for angular deceleration, deg/s^2. Refined from the first
 * turn onwards, so only the first run or two use it directly.
 *
 * SEEDED FROM A CONVERGED RUN, and that matters more than it sounds.
 *
 * This was 294, derived from an early estimate of the coast. The learner
 * actually converges to 636 - the robot stops more than twice as hard as the
 * old seed assumed - and the cost of the bad seed was visible on the floor:
 * a cold session ran errors of 9, 7, 3, 2 degrees before settling at 1.
 *
 * A.4 is ONE turn on a supervisor's word. A cold robot gives them the 9, not
 * the 1. Seeding from the converged value is what makes the first turn of a
 * session behave like the fifth. Both learned values reset at power-off, so
 * this constant is the only thing standing between a cold start and that
 * four-run warm-up. */
#define MOTION_ARC_DECEL_DPS2       636.0f

/* How fast alpha is learned. 0.25 means a quarter of the way to the new
 * measurement each turn - converged in three or four, slow enough that one
 * odd run cannot throw it. */
#define MOTION_ARC_LEARN_GAIN       0.25f

/* 1 - the FIRST valid arc of a power-on replaces the compiled seed outright,
 *     and MOTION_ARC_LEARN_GAIN / MOTION_ARC_LAG_GAIN only damp the arcs
 *     after it. One arc to arrive instead of about ten.
 * 0 - every arc filters, including the first. The original behaviour.
 *
 * The seed is a number measured on another day and another floor. The first
 * arc is a measurement of THIS one, so filtering it toward the seed is
 * defending a prior that has nothing behind it - and it left the cold first
 * run, which is the run A.3 and A.4 are graded on, driving on constants
 * nobody had checked here.
 *
 * A value restored with !CALD / !CALL counts as seeded: that one IS a real
 * prior, and the next arc must not throw it away. See Motion_SetArcDecel(). */
#define MOTION_ARC_SEED_FIRST       1

/* Brake engagement lag, seconds. LEARNED, like alpha.
 *
 * The w^2/2a term describes the coast once the brakes are actually working.
 * It says nothing about the gap between the tick deciding to brake and the
 * bridges biting - and during that gap the robot carries on at full rate. So
 * the real model is
 *
 *      coast = w^2 / (2*alpha)   +   w * t_lag
 *              -------------         ---------
 *              physical coast        latency
 *
 * In principle the two terms are separable by braking at different rates.
 * In practice every arc brakes at about the same approach rate, so they are
 * NOT separable from these arcs - see MOTION_ARC_LEARN_LAG below, which now
 * holds the lag fixed and learns only alpha.
 *
 * Symptom of it missing: error settles to a small CONSTANT that more alpha
 * learning cannot remove, because alpha only scales the quadratic part. That
 * is exactly the steady -3 degrees seen at 65 deg/s, which is about 45 ms of
 * lag - four or five ticks. */
/* Seeded from the same converged run as MOTION_ARC_DECEL_DPS2 above: the
 * learner settles at 3 ms. Effectively zero - the bridges bite as soon as the
 * tick asks them to, and essentially the whole coast is physical rather than
 * latency. Kept as a seeded constant anyway so a cold start matches a warm
 * one, and because the term is what absorbs any residual constant error once
 * alpha has converged. */
#define MOTION_ARC_LAG_S            0.003f
#define MOTION_ARC_LAG_GAIN         0.30f
#define MOTION_ARC_LAG_MAX_S        0.25f

/* 0 - lag is a FIXED constant (whatever !CALL restored, 0 from calibrate.py)
 *     and only decel is learned, from the coast less w*lag. One arc then
 *     gives a self-consistent answer: the predicted lead equals the coast
 *     just seen.
 * 1 - learn lag from the residual overshoot as well. Every arc brakes at
 *     about the same rate, so one arc cannot tell the two terms apart; they
 *     traded degrees back and forth and neither settled. */
#define MOTION_ARC_LEARN_LAG        0

/* Control tick, seconds - how far the robot turns between two brake checks
 * is w * MOTION_TICK_S. */
#define MOTION_TICK_S               0.01f

/* Never brake later than this many degrees out, whatever the maths says.
 * Guards against a bad rate reading braking so late it sails past. */
#define MOTION_ARC_MIN_LEAD_DEG     2.0f

/* Sanity bounds on the learned alpha. Outside these the measurement is
 * rejected rather than believed - a turn cut short by hand, or a wheel
 * slipping, produces nonsense that would otherwise poison the estimate. */
#define MOTION_ARC_DECEL_MIN        50.0f
#define MOTION_ARC_DECEL_MAX        3000.0f

/* ---------------------------------------------------------------------------
 * INDEPENDENT ANGLE CROSS-CHECK
 *
 * Everything on the turn screen is derived from the gyro. R is arc divided by
 * gyro angle; alpha and lag are fitted to gyro angles. So a gyro fault is
 * SELF-CONSISTENT - every number agrees with every other number and the
 * display looks perfectly healthy. That has already happened twice, once from
 * clipping and once from a sign inversion, and both times the only way to
 * catch it was a tape measure on the floor.
 *
 * There is a second witness available for free. The encoders measure arc
 * LENGTH, and the radius is a calibrated constant, so
 *
 *      angle_from_encoders = arc_length / radius      (radians)
 *
 * shares no hardware with the gyro at all - different sensor, different bus,
 * different physics. If the two disagree by more than a sane margin, one of
 * them is broken and the robot should say so rather than finish the move
 * confidently.
 *
 * IMPORTANT: this is only meaningful with diff_boost at 1.0. Above that the
 * rear tyres are deliberately scrubbed, so the wheels turn further than the
 * ground travelled and the encoder arc is inflated by an unknown amount. The
 * check is therefore SKIPPED on any profile with boost above
 * MOTION_XCHECK_MAX_BOOST - use the CLEAN profile to run it.
 *
 * It is a diagnostic, not a safety interlock: it reports, it does not abort.
 * A disagreement means go and measure something with a tape, not that the
 * move was dangerous. */
#define MOTION_XCHECK_ENABLE      1
#define MOTION_XCHECK_MAX_BOOST   1.05f
#define MOTION_XCHECK_TOL_PCT     20.0f

/* Encoder-derived angle for the last arc, degrees. 0 if the check did not
 * run - wrong profile, or too short a move to mean anything. */
float   Motion_GetXCheckDeg(void);

/* Percentage the two witnesses disagreed by on the last arc. */
float   Motion_GetXCheckErrPct(void);

/* 1 if the last arc's two angle estimates disagreed beyond tolerance. */
uint8_t Motion_XCheckFailed(void);

/* Current learned value, for the display. */
float Motion_GetArcDecel(void);

/* The last arc's raw decel measurement, unfiltered; 0 if it gave none. */
float Motion_GetLastDecelMeas(void);

/* Restore learned braking values, for a sender that saved them from an
 * earlier session and wants to skip the three-or-four-arc warm-up. Return 0
 * and change nothing if the value is outside the range the learner itself
 * would accept. Callers must check the motion layer is idle first - these do
 * not, because the braking model is read live by a turn in flight. */
uint8_t Motion_SetArcDecel(float dps2);
uint8_t Motion_SetArcLag(float secs);
float Motion_GetArcLag(void);

/* Settling time for the steering before the wheels are allowed to turn, in
 * 10 ms ticks.
 *
 * Without this the motors start in the SAME TICK the servo is commanded to
 * its starting angle, so the robot begins moving while the steering is still
 * swinging. A hobby servo takes roughly 20-30 ms to cross the deflection a
 * heading correction can leave behind, and the linkage backlash adds more on
 * top. At cruise the robot covers about 7 mm in that time - with the front
 * wheels pointing somewhere they were never asked to point, while the heading
 * for the whole run is being latched.
 *
 * 200 ms is generous, unnoticeable, and paid only when the steering actually
 * has to move (see MOTION_ALIGN_SKIP_US). */
#define MOTION_ALIGN_TICKS      20U

/* If the servo is already within this many microseconds of where the move
 * wants it, skip the settle entirely. Back-to-back straight commands from the
 * RPi would otherwise each pay 200 ms for a servo that is not going to move. */
#define MOTION_ALIGN_SKIP_US    20U

/* ---------------------------------------------------------------------------
 * ARC PROFILES
 *
 * Sharpness and speed cannot be chosen independently of everything else, so a
 * profile carries the whole coupled set:
 *
 *   steer_us      sets the radius, together with diff_boost
 *   rpm           sets the time, and how far the robot coasts afterwards
 *   approach_rpm  must keep the INNER wheel above about 55 rpm, and the
 *                 inner wheel runs at (1 - ratio) of the centre speed, so a
 *                 bigger boost forces a HIGHER approach speed
 *   brake_deg     coast, which scales with rpm - MEASURE IT PER PROFILE
 *   radius_mm     what the robot actually drives - MEASURE IT PER PROFILE,
 *                 it only sets the inner/outer wheel split, not the angle
 *   diff_boost    over-differential, tightens the turn at the cost of scrub
 *
 * brake_deg and radius_mm are MEASUREMENTS, not settings. Changing steer_us,
 * rpm or diff_boost invalidates both, so recalibrate a profile after touching
 * anything else in it. The angle stays accurate throughout because the gyro
 * terminates the turn - only the path and the overshoot move.
 * ------------------------------------------------------------------------- */
typedef struct
{
    const char *name;          /* shown on the OLED, keep it short   */
    uint16_t    steer_us;      /* deflection from SERVO_CENTER_US    */
    int16_t     rpm;           /* cruise speed                       */
    int16_t     approach_rpm;  /* speed for the last approach_deg    */
    float       approach_deg;  /* how much of the turn is slowed     */
    float       brake_deg;     /* MEASURED coast                     */
    float       radius_mm;     /* MEASURED radius                    */
    float       diff_boost;    /* 1.0 = pure geometry, no assist     */
} ArcProfile_t;

/* ---------------------------------------------------------------------------
 * A/B SWITCHES FOR THE 2026-10 CALIBRATION FIXES
 *
 * Each is 1 for the new behaviour, 0 for exactly what was flashed before, so
 * they can be compared on the floor one at a time.
 *
 * MOTION_ARC_GEOMETRIC_SPLIT
 *   1  every arc splits the rear wheels by pure geometry, diff_boost 1.0,
 *      whatever the profile or the calibration table says. The steering then
 *      sets the radius on its own and the rear tyres follow it instead of
 *      scrubbing against it, so the radius stops moving with floor, battery
 *      and direction. It is also what lets the encoder-vs-gyro cross-check
 *      run (it is skipped above MOTION_XCHECK_MAX_BOOST).
 *      A table radius is still used if one is set, but radii measured with a
 *      boost are NOT valid here - re-measure (floor chord) per side and
 *      direction. With no table radius, MOTION_ARC_GEOM_RADIUS_MM is used.
 *   0  the profile's / table's diff_boost, as before.
 *
 * MOTION_ARC_GEOM_RADIUS_MM
 *   Wheel-split radius when the table has none. 318 is CLEAN's floor-chord
 *   measurement - the same 575 us steer at boost 1.0 - so it is the best
 *   number there is until TIGHT and SAFE are chorded at 1.0 themselves. */
#define MOTION_ARC_GEOMETRIC_SPLIT  1
#define MOTION_ARC_GEOM_RADIUS_MM   318.0f

/* MOTION_ARC_TAPER
 *   1  the arc's speed falls with the angle left -
 *          rpm = clamp(MOTION_ARC_TAPER_RPM_PER_DEG * togo,
 *                      MOTION_ARC_TAPER_MIN_RPM, cruise)
 *      - so the turn arrives at a crawl and the brake has almost nothing to
 *      coast (coast goes with speed squared: 25 rpm coasts ~1/16 of what
 *      100 rpm does). The predicted lead is still used, it is just tiny.
 *      Needs PID_LOW_DUTY 1, or the inner wheel cannot hold the floor speed.
 *   0  cruise, then the profile's approach_rpm for the last approach_deg,
 *      then brake on the predicted coast, as before.
 *
 * Speeds are axle-centre rpm, like every other arc speed. At the floor the
 *   outer wheel runs ~1.2x and the inner ~0.8x of it on a ~318 mm radius.
 * MOTION_ARC_TAPER_MIN_LEAD_DEG replaces MOTION_ARC_MIN_LEAD_DEG while the
 *   taper is on: 2 degrees of forced lead at a crawl would stop every turn
 *   short by most of it. */
#define MOTION_ARC_TAPER                1
#define MOTION_ARC_TAPER_RPM_PER_DEG    4.0f
#define MOTION_ARC_TAPER_MIN_RPM        25
#define MOTION_ARC_TAPER_MIN_LEAD_DEG   0.3f

/* MOTION_ALIGN_SKIP_FIX
 *   1  a move whose steering is already in place goes straight to its
 *      target pulse. The old code sent the servo SERVO_APPROACH_US below the
 *      target first and then skipped the wait that brings it back up, so it
 *      launched with the servo swinging away and back.
 *   0  as before. */
#define MOTION_ALIGN_SKIP_FIX           1

#define MOTION_ARC_PROFILE_COUNT  4

#define MOTION_PROFILE_TIGHT      0U
#define MOTION_PROFILE_SAFE       3U

/* Select the active profile. Out-of-range and locked-out values are ignored.
 * Returns 1 if the request was honoured (on a SAFE build, 0 selects SAFE and
 * still counts), 0 if it was ignored. */
uint8_t     Motion_SetArcProfile(uint8_t idx);
uint8_t     Motion_ArcProfileAllowed(uint8_t idx);  /* see PROFILE LOCK */
uint8_t     Motion_GetArcProfile(void);
const ArcProfile_t *Motion_GetArcProfileInfo(uint8_t idx);

/* The deflection actually used, after clamping to whatever the servo limits
 * allow symmetrically. Lets the UI show what will really happen rather than
 * what was asked for. */
uint16_t    Motion_GetArcSteerUs(void);

/* ---------------------------------------------------------------------------
 * PER-SIDE ARC STEERING
 *
 * The same pulse either side of centre does not give the same wheel angle:
 * the linkage is not symmetric, and neither is the play in it. So a left
 * and a right turn at the same deflection trace different circles - the gyro
 * still stops both at 90 degrees, but they land the robot in different
 * places, while the planner and the A5 orbit assume one radius both ways.
 *
 * Each side's deflection is therefore its own calibration value, measured
 * by `calibrate.py radius` (floor chord per side) and restored with the rest
 * of the profile via !CALSL / !CALSR. 0 means "use the profile's symmetric
 * value", which is what the robot does until something is restored.
 *
 * Bounds: never past the servo limits on that side, and never below
 * MOTION_ARC_STEER_MIN_US, which is far wider than any turn the planner
 * could use. Refused, not clamped, like every other setter.
 * ------------------------------------------------------------------------- */
#define MOTION_ARC_STEER_MIN_US   300U

uint8_t     Motion_SetArcSteerSideUs(uint8_t right, uint16_t us);
uint16_t    Motion_GetArcSteerSideUs(uint8_t right);   /* the value in force */

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_ALIGN,
    MOTION_RUN,
    MOTION_BRAKE,
    MOTION_DONE,
    MOTION_TIMEOUT
} MotionState_t;

/* Call once at startup, after Odom_Init(). */
void Motion_Init(void);

/* Call from the TIM6 ISR, between Encoders_Update() and Odom_Update(). */
void Motion_Tick(void);

/* Drive a straight line. Negative mm reverses. Holds the heading the robot
 * has at the moment of the call. Returns immediately. */
void Motion_DriveDistance(int32_t mm);

/* Drive an arc through the given angle. Returns immediately.
 *   degrees  > 0, the swept angle
 *   forward  1 to travel forwards, 0 to reverse
 *   right    1 to curve right, 0 to curve left
 *
 * Reversing DOES flip which way the body rotates for a given steering angle.
 * Backing up with the front wheels turned right swings the nose to the left.
 * The steering side is set by 'right' alone; the resulting body rotation is
 * set by both. */
void Motion_DriveArc(int16_t degrees, uint8_t forward, uint8_t right);

/* ---------------------------------------------------------------------------
 * PER-MOVE SPEED AND THE CALIBRATION TABLE, protocol 10
 *
 * The same two moves with their own cruise, rpm at the axle centre. rpm 0
 * means "no V": the table's vs / va default if set, otherwise exactly the
 * protocol 9 move (profile speed for arcs, !FAST or normal for straights).
 *
 * Every move looks up its calibration at its cruise speed when it is queued
 * (caltab.h), and holds those values to the end:
 *
 *   straights  trim (steering centre for this direction), coast (replaces
 *              MOTION_BRAKE_MM), dist (odometry scale)
 *   arcs       steer (replaces the per-side deflection), decel and lag
 *              (replace the learned braking model), radius and boost
 *              (replace the profile's wheel split)
 *
 * Anything the table does not have comes from where protocol 9 got it.
 *
 * A V straight ramps like fast mode (MOTION_RAMP_*) up to its cruise and
 * back down to MOTION_APPROACH_RPM for the last MOTION_FAST_TAIL_MM, so its
 * stop is the same 40 rpm stop whatever the cruise. A V arc runs at its
 * cruise and slows to the lower of the profile's approach_rpm and its own
 * cruise for the last approach_deg.
 * ------------------------------------------------------------------------- */
void Motion_DriveDistanceV(int32_t mm, int16_t rpm);
void Motion_DriveArcV(int16_t degrees, uint8_t forward, uint8_t right, int16_t rpm);

/* ---------------------------------------------------------------------------
 * MOVE REPORT, protocol 10 (?MV)
 *
 * Calibration used to see a move through three numbers: where the gyro said
 * it stopped, the learned decel, and a tape. Everything in between - which
 * wheel worked harder, how much steering the straight needed, how fast it
 * was turning when it braked - was thrown away at the end of the move.
 *
 * Now every move fills one of these, kept until the next move starts, so one
 * run answers many questions at once and calibrate10.py can fit the table
 * from the robot's own sensors and only ask for a tape where nothing on
 * board can know the answer (absolute distance, the radius once per move).
 *
 * "Cruise" means the steady part: past MOTION_REPORT_WARMUP_TICKS, at the
 * move's cruise speed, before any slow-down. Per-wheel rpm and duty there
 * are what the motor model is fitted from.
 * ------------------------------------------------------------------------- */
#define MOTION_REPORT_WARMUP_TICKS  30U     /* 0.3 s */

typedef struct
{
    uint16_t seq;        /* +1 per move launched, so a stale report shows   */
    uint8_t  move;       /* CtMove_t + 1 (1 F .. 6 RL), 0 = no move yet     */
    uint8_t  result;     /* 0 ok, 1 timeout, 2 wrong way, 3 missed, 4 RST   */
    int16_t  rpm;        /* cruise it was asked for                         */
    uint16_t ticks;      /* RUN + BRAKE, 10 ms                              */
    int32_t  enc_a;      /* counts, launch to settle, signed                */
    int32_t  enc_b;
    uint16_t n;          /* cruise samples                                  */
    float    rpm_a;      /* mean |rpm| per wheel in cruise                  */
    float    rpm_b;
    float    duty_a;     /* mean |duty| per wheel in cruise, 0..1000        */
    float    duty_b;
    float    corr;       /* straights: mean steering correction, us        */
    float    max_err;    /* straights: worst |heading error| held, deg     */
    float    dist_brake; /* odometry mm when the brake went on              */
    float    dist;       /* odometry mm at settle                           */
    float    aim;        /* arcs: deg aimed for (after carry-over), signed  */
    float    turned;     /* arcs: deg turned at settle, signed              */
    float    brake_rate; /* arcs: |yaw rate| at brake onset, deg/s          */
    float    brake_ang;  /* arcs: |angle| at brake onset, deg               */
    float    decel_meas; /* arcs: decel this coast implies (lag in force), 0 = none */
    float    x;          /* pose at settle in the move's own frame, mm      */
    float    y;
    uint16_t servo;      /* arcs: pulse held; straights: centre + trim      */
    float    trim;       /* straights: trim in force, us                    */
    float    target;     /* odometry mm (straights) / deg (arcs) aimed for  */
} MotionReport_t;

const MotionReport_t *Motion_GetReport(void);

/* Abort now: brake, recentre steering, go to IDLE. */
void Motion_Stop(void);

/* 1 while a primitive is running. Poll this to know when to reply OK. */
uint8_t Motion_IsBusy(void);

/* 1 if the last arc was aborted for turning AWAY from its target rather than
 * for the watchdog. Both land in MOTION_TIMEOUT but they mean different
 * things: the watchdog is a stalled wheel, this is a sign inversion or a gyro
 * that is lying. Cleared when the next primitive launches. */
uint8_t Motion_WrongWayAborted(void);

/* 1 if the last arc finished more than MOTION_ARC_FAIL_DEG from its aim.
 * It ends in MOTION_TIMEOUT and the line replies FAIL,SHORT. Normal stops
 * land well under a degree off, and the carry-over only corrects 5, so an
 * arc this far off has to be reported rather than answered OK - one that
 * braked 36 degrees early off a wheel surge used to say OK. Cleared when the
 * next primitive launches. */
#define MOTION_ARC_FAIL_DEG     10.0f
uint8_t Motion_ArcMissed(void);

MotionState_t Motion_GetState(void);

/* Clears DONE/TIMEOUT back to IDLE once the caller has read the result. */
void Motion_ClearState(void);

/* Distance still to go, mm. For the OLED while tuning. */
int32_t Motion_GetRemaining(void);

/* Distance actually covered by the current or last primitive, mm. */
int32_t Motion_GetTravelled(void);

/* Degrees turned so far in the current or last arc, signed, unwrapped. */
int32_t Motion_GetTurnedDeg(void);

/* Degrees the current arc is aiming for, signed. */
int32_t Motion_GetTurnTargetDeg(void);

/* ---------------------------------------------------------------------------
 * LEARNING SWITCH
 *
 * The arc decel/lag learner and the steering trim learner used to run after
 * every move, forever. That meant a task run kept rewriting the calibration
 * it was being driven on, one noisy sample at a time - the numbers a robot
 * finished a run with were not the numbers it started with, and neither was
 * the averaged value calibrate.py measured.
 *
 * Now learning is OFF at power-on and runs only while something explicitly
 * asks for it: calibrate.py (!LEARN1 ... !LEARN0) or the OLED CALIB cycle.
 * During a task the calibration is exactly what was restored, nothing else.
 * ------------------------------------------------------------------------- */
void    Motion_SetLearning(uint8_t on);
uint8_t Motion_LearningEnabled(void);

/* ---------------------------------------------------------------------------
 * HEADING CARRY-OVER
 *
 * Every move zeroes the odometry and the gyro total when it launches, so on
 * its own each move only knows about itself: an FR90 that stopped at 89.5
 * left the robot 0.5 degrees off for good, the next FR90 turned another 90
 * from there, and errors added up turn after turn. Straights held whatever
 * heading they started on, crooked or not.
 *
 * So the motion layer keeps two running totals that survive between moves:
 *
 *      commanded  - the sum of every arc angle ASKED for since !ZERO
 *      actual     - the sum of every rotation the gyro MEASURED since !ZERO
 *
 * and every move aims at the commanded total rather than at its own start:
 *
 *      arc       turns  asked + (commanded - actual)
 *      straight  holds  (commanded - actual) relative to where it starts
 *
 * An FR90 after an 89.5 turns 90.5; a straight after it steers the half
 * degree out. The error left behind by any one move is corrected by the next,
 * so it can never accumulate beyond a single move's error.
 *
 * The correction is capped at MOTION_CARRY_MAX_DEG. Anything bigger than that
 * is not braking error - the robot was bumped, a wheel slipped, a move was
 * aborted - and chasing it with a big steering input would do more harm than
 * the error. It is applied up to the cap and counted, so ?HDG shows it.
 *
 * RST and a failed move re-sync commanded to actual: the robot is no longer
 * where the plan thinks, and the sender has to re-plan anyway.
 * ------------------------------------------------------------------------- */
#define MOTION_CARRY_ENABLE     1
#define MOTION_CARRY_MAX_DEG    5.0f

/* Zero both totals and the odometry. What !ZERO does. */
void    Motion_ZeroHeading(void);

/* Throw away any outstanding error: commanded := actual. For RST. */
void    Motion_ResyncHeading(void);

float   Motion_GetCommandedHeading(void);   /* degrees, CCW positive      */
float   Motion_GetActualHeading(void);      /* degrees, live              */
float   Motion_GetLastArcAimDeg(void);      /* signed, after carry-over   */
float   Motion_GetLastArcTurnedDeg(void);   /* signed, gyro, at stop      */
uint16_t Motion_GetCarryClampCount(void);   /* corrections capped since zero */

/* ---------------------------------------------------------------------------
 * PROFILE LOCK
 *
 * Only TIGHT (0) and SAFE (3) are driven, each with its OWN saved
 * calibration: the braking model, steering and radius are one set of numbers
 * that only fits the profile it was measured on. The firmware holds one set
 * at a time; the RPi restores the right one with the profile (cal_profile.py
 * records which profile each calibration was taken on).
 *
 * With the lock on, Motion_SetArcProfile() ignores CLEAN and SLOW, !PROF1/2
 * answer RESEND, and the OLED profile mode only cycles TIGHT <-> SAFE. The
 * other two stay in the table so turning the lock off restores them.
 * ------------------------------------------------------------------------- */
#define MOTION_PROFILE_LOCK         1

/* ---------------------------------------------------------------------------
 * MOTION FAST MODE  (!FAST1 / !FAST0)
 *
 * For Task 2, where time is the score. Off at power-on, and every task but
 * Task 2 switches it off at startup, so nothing calibrated or tuned for the
 * other tasks changes.
 *
 *   straights cruise at MOTION_FAST_CRUISE_RPM instead of MOTION_CRUISE_RPM
 *   they RAMP: start at MOTION_RAMP_START_RPM and climb by
 *     MOTION_RAMP_UP_RPM_PER_TICK, then on the way in fall by
 *     MOTION_RAMP_DOWN_RPM_PER_TICK to MOTION_APPROACH_RPM - no step change
 *     in wheel speed anywhere, so no wheelspin off the line and no nose dip
 *     at the slow-down, both of which kicked the heading
 *   the slow-down starts as late as it can: when what is left is the
 *     distance the down-ramp needs FROM THE SPEED ACTUALLY REACHED, plus
 *     MOTION_FAST_TAIL_MM at MOTION_APPROACH_RPM. A short move that never
 *     reached cruise brakes later than a long one. From full cruise that is
 *     MOTION_FAST_APPROACH_MM (computed below, for reference)
 *   the last stretch is still MOTION_APPROACH_RPM, which is what
 *     MOTION_BRAKE_MM was measured at - so straight calibration still holds
 *   every move settles MOTION_FAST_BRAKE_TICKS after braking instead of
 *     MOTION_BRAKE_TICKS - still only once the wheels read stopped and the
 *     steering is back on centre (see the BRAKE phase in motion.c)
 *
 * Arcs are unchanged: their speed is the profile's, and their braking model
 * is what calibration measured. Normal mode is unchanged too - no ramp, the
 * same steps it was tuned and calibrated with.
 *
 * Why 200 and not more: the slower motor free-runs at 362 rpm on a full
 * 12 V with the wheels up. Loaded and on a sagging battery it has far less,
 * and the PID needs headroom to hold the two wheels together. 200 leaves
 * ~45%. Past ~250 it starts running out. */
#define MOTION_FAST_CRUISE_RPM      200

/* Ramps, rpm per 10 ms tick. Up: 60 -> 200 in 0.35 s, ~16 cm. Down is
 * steeper - the PID can brake harder than the motors can accelerate - and
 * 200 -> 40 takes 0.27 s, ~11 cm. Start above the ~32 rpm the deadband
 * duty gives, or the first ticks only wind up the integrator. */
#define MOTION_RAMP_START_RPM        60
#define MOTION_RAMP_UP_RPM_PER_TICK   4
#define MOTION_RAMP_DOWN_RPM_PER_TICK 6

/* Distance driven at MOTION_APPROACH_RPM after the down-ramp, mm. Covers the
 * wheels lagging the ramp; the stop itself then happens from the steady
 * 40 rpm MOTION_BRAKE_MM was measured at. */
#define MOTION_FAST_TAIL_MM         120.0f

/* Wheel travel per rpm, (mm/s)/rpm: pi * WHEEL_DIAMETER_MM (65.9) / 60. */
#define MOTION_MM_S_PER_RPM         3.4505f

/* Down-ramp distance from rpm r to MOTION_APPROACH_RPM, mm:
 * v^2 / 2a with v and a both in wheel mm. */
#define MOTION_RAMP_DOWN_MM(r) \
    ((MOTION_MM_S_PER_RPM * ((float)(r) * (float)(r) - \
      (float)MOTION_APPROACH_RPM * (float)MOTION_APPROACH_RPM)) / \
     (2.0f * (float)MOTION_RAMP_DOWN_RPM_PER_TICK * 100.0f))

/* From full cruise: ~110 + 120 = ~230 mm. For reference - motion.c works it
 * out from the speed actually reached. */
#define MOTION_FAST_APPROACH_MM \
    (MOTION_RAMP_DOWN_MM(MOTION_FAST_CRUISE_RPM) + MOTION_FAST_TAIL_MM)

#define MOTION_FAST_BRAKE_TICKS     20U

/* Wheel distance to ramp down from `from` to `to` rpm at the fast-mode rate,
 * mm. MOTION_RAMP_DOWN_MM is this with `to` fixed at MOTION_APPROACH_RPM. */
#define MOTION_RAMP_DOWN_MM_TO(from, to) \
    ((MOTION_MM_S_PER_RPM * ((float)(from) * (float)(from) - \
      (float)(to) * (float)(to))) / \
     (2.0f * (float)MOTION_RAMP_DOWN_RPM_PER_TICK * 100.0f))

void    Motion_SetFast(uint8_t on);
uint8_t Motion_FastEnabled(void);

/* ---------------------------------------------------------------------------
 * SAFE BUILD
 *
 * 0 - normal build. !PROF0 is TIGHT, as it always was, so calibrations saved
 *     on TIGHT keep fitting.
 * 1 - SAFE build. The robot boots on SAFE and answers !PROF0 with SAFE too.
 *
 * Exists for RPi code that only knows profile 0: its task startup and
 * calibrate.py both send !PROF0, so without this they could never reach
 * SAFE. Flash a SAFE build to calibrate and run SAFE with that code, and
 * save its calibration under a DIFFERENT name - it will be labelled profile 0
 * on the RPi, but it only fits SAFE. ?STAT reports 3 on a SAFE build.
 * ------------------------------------------------------------------------- */
#define MOTION_BUILD_SAFE           0

#endif /* __MOTION_H */
