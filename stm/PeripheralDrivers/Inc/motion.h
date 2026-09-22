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
 * The two terms are separately observable, which is what makes learning both
 * of them well posed: alpha comes from the coast actually measured between
 * brake onset and standstill, t_lag from whatever terminal error is left over
 * after alpha has converged.
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

#define MOTION_ARC_PROFILE_COUNT  3

/* Select the active profile. Out-of-range values are ignored. */
void        Motion_SetArcProfile(uint8_t idx);
uint8_t     Motion_GetArcProfile(void);
const ArcProfile_t *Motion_GetArcProfileInfo(uint8_t idx);

/* The deflection actually used, after clamping to whatever the servo limits
 * allow symmetrically. Lets the UI show what will really happen rather than
 * what was asked for. */
uint16_t    Motion_GetArcSteerUs(void);

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

/* Abort now: brake, recentre steering, go to IDLE. */
void Motion_Stop(void);

/* 1 while a primitive is running. Poll this to know when to reply OK. */
uint8_t Motion_IsBusy(void);

/* 1 if the last arc was aborted for turning AWAY from its target rather than
 * for the watchdog. Both land in MOTION_TIMEOUT but they mean different
 * things: the watchdog is a stalled wheel, this is a sign inversion or a gyro
 * that is lying. Cleared when the next primitive launches. */
uint8_t Motion_WrongWayAborted(void);

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

#endif /* __MOTION_H */
