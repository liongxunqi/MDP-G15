#include "motion.h"
#include "odom.h"
#include "pid.h"
#include "motors.h"
#include "encoders.h"
#include <math.h>

typedef enum { MOVE_STRAIGHT = 0, MOVE_ARC } MoveKind_t;

static MotionState_t s_state;
static MoveKind_t s_kind;
static float    s_targetMm;      /* absolute, brake compensation applied */
static float    s_holdHeading;   /* heading latched at the start of a straight */
static int8_t   s_dir;           /* +1 forward, -1 reverse                   */
static uint8_t  s_arcRight;      /* which way the arc curves                 */
static uint16_t s_arcServoUs;    /* servo pulse held for the whole arc       */
static int16_t  s_lastRpm;       /* last speed issued to the odom layer      */
static uint32_t s_ticks;
static uint32_t s_brakeTicks;
static uint32_t s_alignTicks;
static uint16_t s_startServoUs;
static float    s_arcTargetDeg;   /* signed, unwrapped, relative to launch */
static uint16_t s_settleTarget;   /* servo angle being settled onto        */

/* ---------------------------------------------------------------------------
 * Arc profiles.
 *
 * TIGHT and CLEAN have MEASURED radii, by the floor-chord method - mark under
 * the rear axle, run a 90, mark again, R = chord / 1.414. That is the only
 * number in the whole system that does not pass through the gyro. SLOW's 306
 * came from a boost-1.5 measurement at a different speed, so treat it as
 * approximate until someone runs the chord on it.
 *
 * The three sit monotonically in diff_boost - 1.0 gives 318, 1.5 gives 306,
 * 2.0 gives 291 - which is what the differential assist is supposed to do and
 * is decent evidence none of them is wildly wrong.
 *
 * brake_deg is NOT calibrated and does not need to be: it is only read when
 * MOTION_ARC_ADAPTIVE_BRAKE is 0, and it is 1. The field is kept so the
 * fallback still compiles.
 * ------------------------------------------------------------------------- */
static const ArcProfile_t s_profiles[MOTION_ARC_PROFILE_COUNT] =
{
    /* 0: the calibrated one. Tight and quick.                              */
    { "TIGHT", 575U, 150, 100, 20.0f,   8.0f, 291.0f, 2.0f },

    /* 1: no differential assist. Wider, but the rear tyres are not scrubbed
     *    so the path should be a cleaner circle - which matters more than
     *    radius if the planner needs the robot to finish where it predicted.
     *
     *    radius_mm MEASURED at 318 by floor chord, three runs, all 45 cm.
     *    Replaces a 340 guess. This one has to be right: the encoder-vs-gyro
     *    cross-check only runs at boost <= 1.05, so CLEAN is the only profile
     *    it can use, and radius_mm is its denominator. A guessed radius makes
     *    the second witness agree with nothing in particular. */
    { "CLEAN", 575U, 150, 100, 20.0f,   8.0f, 318.0f, 1.0f },

    /* 2: slow. Longer to execute, but the coast is much smaller so the stop
     *    is more repeatable, and the inner wheel has plenty of margin above
     *    the deadband. MEASURE brake_deg and radius_mm. */
    { "SLOW",  575U,  70,  70, 15.0f,   3.0f, 306.0f, 1.5f }
};

static uint8_t s_profileIdx = 0U;

/* Learned angular deceleration and the state needed to measure it. */
static float s_arcDecel   = MOTION_ARC_DECEL_DPS2;
static float s_brakeRate  = 0.0f;   /* |yaw rate| when braking started */
static float s_brakeAngle = 0.0f;   /* |heading| when braking started  */
static float s_arcLag     = MOTION_ARC_LAG_S;

/* Cross-check results for the last arc. */
static float   s_xcheckDeg    = 0.0f;
static float   s_xcheckErrPct = 0.0f;
static uint8_t s_xcheckFailed = 0U;
static float   s_arcStartDist = 0.0f;

float   Motion_GetXCheckDeg(void)    { return s_xcheckDeg; }
float   Motion_GetXCheckErrPct(void) { return s_xcheckErrPct; }
uint8_t Motion_XCheckFailed(void)    { return s_xcheckFailed; }

float Motion_GetArcDecel(void) { return s_arcDecel; }
float Motion_GetArcLag(void)   { return s_arcLag; }

/* Restore a previously learned value. Both REJECT rather than clamp: a
 * sender that asks for something impossible has a bug, and clamping would
 * hide it behind an OK. Same reasoning as the learner itself, which throws
 * out an out-of-range measurement rather than believing it. */
uint8_t Motion_SetArcDecel(float dps2)
{
    if ((dps2 <= MOTION_ARC_DECEL_MIN) || (dps2 >= MOTION_ARC_DECEL_MAX))
    {
        return 0U;
    }

    s_arcDecel = dps2;
    return 1U;
}

uint8_t Motion_SetArcLag(float secs)
{
    if ((secs < 0.0f) || (secs > MOTION_ARC_LAG_MAX_S)) { return 0U; }

    s_arcLag = secs;
    return 1U;
}

static const ArcProfile_t *prof(void) { return &s_profiles[s_profileIdx]; }

/* Clamp the deflection to what the servo limits allow on the TIGHTER side.
 *
 * Servo_SetMicroseconds() clamps silently, so an over-large deflection would
 * shorten one direction only and turns would come out lopsided with nothing
 * on screen to say why. Taking the smaller of the two available sides keeps
 * left and right identical whatever a profile asks for. */
static uint16_t arc_steer_us(void)
{
    uint16_t hi = (uint16_t)(SERVO_MAX_US - SERVO_CENTER_US);
    uint16_t lo = (uint16_t)(SERVO_CENTER_US - SERVO_MIN_US);
    uint16_t cap = (hi < lo) ? hi : lo;

    return (prof()->steer_us > cap) ? cap : prof()->steer_us;
}

void Motion_SetArcProfile(uint8_t idx)
{
    if (idx < MOTION_ARC_PROFILE_COUNT) { s_profileIdx = idx; }
}

uint8_t  Motion_GetArcProfile(void)  { return s_profileIdx; }
uint16_t Motion_GetArcSteerUs(void)  { return arc_steer_us(); }

const ArcProfile_t *Motion_GetArcProfileInfo(uint8_t idx)
{
    return (idx < MOTION_ARC_PROFILE_COUNT) ? &s_profiles[idx] : &s_profiles[0];
}

/* Set when an arc was aborted for rotating AWAY from its target, cleared when
 * the next primitive launches. Both that and the watchdog land in
 * MOTION_TIMEOUT, but they mean completely different things - a stalled wheel
 * versus a sign inversion or a lying gyro - and the command layer reports them
 * separately so the fix is obvious from the reply alone. */
static uint8_t  s_wrongWay;

static int16_t  s_arcCommandDeg;  /* what the caller ASKED for, uncompensated */
static uint8_t  s_recentreStep;   /* 0 waiting to stop, 1 preloaded, 2 done */
static uint16_t s_recentreTick;

/* ---------------------------------------------------------------------------
 * MEASURING MOTION_BRAKE_MM
 *
 * Set MOTION_BRAKE_MM to 0, command 1000 mm, and measure where the robot
 * actually stops with a tape. If it lands at 1015, the coast is 15 mm - put
 * that in MOTION_BRAKE_MM and it will land on 1000.
 *
 * Do it five times and average. Coast varies with battery charge and floor
 * surface, so re-check if you demo on a different floor to the one you
 * calibrated on.
 *
 * Measure at MOTION_APPROACH_RPM, not cruise - the last stretch before the
 * stop is always at approach speed, so that is the speed that sets the coast.
 * ------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * Starting and stopping a primitive is done from the MAIN LOOP, while the
 * control tick is running in an interrupt at 100 Hz. Every one of these entry
 * points rewrites state the tick reads on the very next pass - the target,
 * the direction, the odometry pose - and none of those writes is atomic.
 *
 * Without a critical section a command arriving from the RPi at the wrong
 * microsecond can leave the tick reading a new target against an old pose,
 * which shows up as a move that stops instantly or overshoots by a metre and
 * never reproduces on the bench. Cheap to prevent, miserable to debug.
 *
 * PRIMASK is saved and restored rather than blindly re-enabled, so these are
 * safe to nest and safe to call from an interrupt if that is ever needed.
 * ------------------------------------------------------------------------- */
static inline uint32_t crit_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void crit_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/* ---------------------------------------------------------------------------
 * Stopping the wheels.
 *
 * PID_Enable(0) MUST come before Motors_Brake(). Braking is done by driving
 * both H-bridge inputs high; the PID drives them from the same tick, so if it
 * is left enabled the very next PID_Update() writes a zero duty over the top
 * and the brake silently becomes a coast. That was costing the whole of the
 * stopping-distance consistency the +/-6% requirement depends on.
 * ------------------------------------------------------------------------- */
static void motion_halt(void)
{
    Odom_Stop();        /* recentres steering, drops the odom drive mode */
    PID_Enable(0);      /* stop the loop writing duty - MUST precede brake */
    Motors_Brake();
    s_lastRpm = 0;
}


/* Park the steering where the move needs it and hold everything still while
 * it gets there. The odometry is NOT zeroed here - that happens when the
 * settle finishes, so any twitch while the servo swings is discarded rather
 * than counted as travel. */
static void motion_begin_align(uint16_t servo_us)
{
    uint16_t now = Odom_GetServoUs();
    uint16_t diff = (now > servo_us) ? (uint16_t)(now - servo_us)
                                     : (uint16_t)(servo_us - now);

    s_startServoUs = servo_us;
    s_settleTarget = servo_us;

    PID_Enable(0);
    Motors_Coast();

    /* Approach from BELOW, always. See SERVO_APPROACH_US in motors.h - this
     * is what makes the linkage slack resolve the same way every time
     * instead of depending on which direction the last move left it. */
    Servo_SetMicroseconds((uint16_t)(servo_us - SERVO_APPROACH_US));

    /* Already there - no point waiting for a servo that will not move.
     *
     * The MOTION_ALIGN_SKIP_US > 0 test is not redundant. Without it, setting
     * the threshold to zero to force the settle does nothing, because a servo
     * sitting exactly on target gives diff == 0 and 0 <= 0 still skips. And
     * exactly on target is the normal case: Odom_Stop() recentres the
     * steering at the end of every move, so the next straight run starts with
     * diff of precisely zero. Zero now means never skip. */
    s_alignTicks = ((MOTION_ALIGN_SKIP_US > 0U) && (diff <= MOTION_ALIGN_SKIP_US))
                 ? MOTION_ALIGN_TICKS
                 : 0U;

    s_ticks = 0U;
    s_state = MOTION_ALIGN;
}

/* Settle finished: zero the odometry now, then start driving. */
static void motion_launch(void)
{
    s_wrongWay = 0U;

    Odom_Reset();

    s_arcStartDist = Odom_GetDistance();   /* zero, just after the reset */

    s_ticks      = 0U;
    s_brakeTicks = 0U;
    s_lastRpm    = (s_kind == MOVE_ARC)
                 ? (int16_t)(s_dir * prof()->rpm)
                 : (int16_t)(s_dir * MOTION_CRUISE_RPM);
    s_state      = MOTION_RUN;

    PID_Enable(1);

    if (s_kind == MOVE_ARC)
    {
        Odom_DriveArc(s_lastRpm, s_arcServoUs, prof()->radius_mm,
                      s_arcRight, prof()->diff_boost);
    }
    else
    {
        s_holdHeading = Odom_GetHeading();   /* zero, just after the reset */
        Odom_DriveHeading(s_lastRpm, s_holdHeading);
    }
}

void Motion_Init(void)
{
    s_state       = MOTION_IDLE;
    s_kind        = MOVE_STRAIGHT;
    s_targetMm    = 0.0f;
    s_holdHeading = 0.0f;
    s_dir         = 1;
    s_arcRight    = 0U;
    s_arcServoUs  = SERVO_CENTER_US;
    s_lastRpm     = 0;
    s_ticks       = 0U;
    s_brakeTicks  = 0U;
    s_alignTicks  = 0U;
    s_startServoUs = SERVO_CENTER_US;
    s_recentreStep = 0U;
    s_recentreTick = 0U;
    s_arcTargetDeg = 0.0f;
    s_arcCommandDeg = 0;
}

void Motion_DriveDistance(int32_t mm)
{
    float    target;
    uint32_t pm;

    if (mm == 0) { return; }

    pm = crit_enter();

    s_kind = MOVE_STRAIGHT;
    s_dir  = (mm < 0) ? -1 : 1;

    target = (float)((mm < 0) ? -mm : mm) - MOTION_BRAKE_MM;
    if (target < 0.0f) { target = 0.0f; }
    s_targetMm = target;

    motion_begin_align(SERVO_CENTER_US);

    crit_exit(pm);
}

void Motion_DriveArc(int16_t degrees, uint8_t forward, uint8_t right)
{
    float    rad;
    float    arc_mm;
    uint32_t pm;

    if (degrees <= 0) { return; }

    pm = crit_enter();

    s_kind     = MOVE_ARC;
    s_dir      = forward ? 1 : -1;
    s_arcRight = right ? 1U : 0U;

    s_arcServoUs = right
                 ? (uint16_t)(SERVO_CENTER_US + arc_steer_us())
                 : (uint16_t)(SERVO_CENTER_US - arc_steer_us());

    /* Which way the BODY rotates.
     *
     * Steering right and driving forward swings the nose right, which is
     * clockwise - and heading counts counter-clockwise positive, so the
     * target is negative. Reversing with the same steering angle swings the
     * nose the other way, so the sign flips again.
     *
     *      forward + right  ->  -degrees
     *      forward + left   ->  +degrees
     *      reverse + right  ->  +degrees
     *      reverse + left   ->  -degrees
     *
     * VERIFY THIS ON THE ROBOT before trusting a 360. If a commanded FR90
     * turns left, or runs away instead of stopping, invert the expression -
     * it means the heading source counts the other way round on your build. */
#if MOTION_ARC_ADAPTIVE_BRAKE
    /* Aim for the full commanded angle. The tick decides when to brake by
     * predicting the coast from the live yaw rate. */
    rad = (float)degrees;
#else
    rad = (float)degrees - prof()->brake_deg;
#endif
    if (rad < 0.0f) { rad = 0.0f; }

    /* Keep the commanded angle so the result screen can report error against
     * what was ASKED for. Reporting against s_arcTargetDeg instead compares
     * the outcome to the internal target, and on the non-adaptive path that
     * target is short by the profile's brake_deg - so a perfectly executed
     * turn would show that as a permanent error. */
    s_arcCommandDeg = degrees;
    if (right)    { s_arcCommandDeg = (int16_t)(-s_arcCommandDeg); }
    if (!forward) { s_arcCommandDeg = (int16_t)(-s_arcCommandDeg); }
    s_arcCommandDeg = (int16_t)(s_arcCommandDeg * MOTION_ARC_SIGN);

    s_arcTargetDeg = rad;
    if (right)    { s_arcTargetDeg = -s_arcTargetDeg; }
    if (!forward) { s_arcTargetDeg = -s_arcTargetDeg; }
    s_arcTargetDeg *= (float)MOTION_ARC_SIGN;

    /* Distance is not the termination condition any more, but keep a
     * generous arc-length estimate so Motion_GetRemaining() and the OLED
     * still show something sensible. */
    arc_mm     = prof()->radius_mm * ((float)degrees * (3.14159265f / 180.0f));
    s_targetMm = arc_mm;

    /* An arc needs a much bigger steering movement than a straight line, so
     * the settle matters more here, not less. */
    motion_begin_align(s_arcServoUs);

    crit_exit(pm);
}

void Motion_Stop(void)
{
    uint32_t pm = crit_enter();

    motion_halt();

    /* Reset the recentre sequence. Aborting mid-brake used to leave this
     * part-way through, and the NEXT move's brake phase would then skip the
     * "wait until the wheels have stopped" check and swing the servo while
     * the robot was still coasting - reintroducing the sideways nudge on
     * every stop, but only after an abort, which made it look intermittent. */
    s_recentreStep = 0U;
    s_recentreTick = 0U;

    s_state = MOTION_IDLE;

    crit_exit(pm);
}

/* Change speed mid-move.
 *
 * Odom_SetSpeed(), NOT Odom_DriveHeading()/Odom_DriveArc(). Those two start a
 * move: they latch a heading, clear the accumulated error and recentre the
 * servo. Using them for the approach taper would discard the steering
 * correction the loop had settled on and snap the wheels straight 150 mm
 * before the stop. */
static void motion_issue(int16_t rpm)
{
    Odom_SetSpeed(rpm);
}

void Motion_Tick(void)
{
    float   travelled;
    float   remaining;
    int16_t want;

    if (s_state == MOTION_ALIGN)
    {
        s_alignTicks++;

        /* Halfway through the settle, come UP onto the target. */
        if (s_alignTicks == (MOTION_ALIGN_TICKS / 2U))
        {
            Servo_SetMicroseconds(s_settleTarget);
        }

        if (s_alignTicks >= MOTION_ALIGN_TICKS)
        {
            motion_launch();
        }
        return;
    }

    if ((s_state != MOTION_RUN) && (s_state != MOTION_BRAKE))
    {
        return;
    }

    s_ticks++;
    if (s_ticks > MOTION_TIMEOUT_TICKS)
    {
        /* Wheel stalled, or the encoder stopped counting. Give up cleanly
         * rather than sit here forever. */
        motion_halt();
        s_state = MOTION_TIMEOUT;
        return;
    }

    travelled = Odom_GetDistance();     /* always positive */
    remaining = s_targetMm - travelled;

    if (s_state == MOTION_RUN)
    {
        /* An arc ends on ANGLE, a straight line on DISTANCE. */
        if (s_kind == MOVE_ARC)
        {
            float turned = Odom_GetHeadingTotal();
            float togo   = (s_arcTargetDeg >= 0.0f)
                         ? (s_arcTargetDeg - turned)
                         : (turned - s_arcTargetDeg);

            {
                float lead = 0.0f;
#if MOTION_ARC_ADAPTIVE_BRAKE
                /* Predict the coast from the rate we are turning at RIGHT NOW.
                 * A faster profile has a larger w and so brakes earlier, with
                 * nothing to retune. */
                float w = Odom_GetRateDps();
                if (w < 0.0f) { w = -w; }

                /* Quadratic term is the coast once braking; linear term is
                 * the distance covered during the engagement lag. */
                lead = ((w * w) / (2.0f * s_arcDecel)) + (w * s_arcLag);
                if (lead < MOTION_ARC_MIN_LEAD_DEG)
                {
                    lead = MOTION_ARC_MIN_LEAD_DEG;
                }
#endif
                if (togo <= lead)
                {
                    /* Remember what we were doing at brake onset - this is
                     * what makes the coast measurable afterwards. */
                    s_brakeRate  = (Odom_GetRateDps() < 0.0f)
                                 ? -Odom_GetRateDps() : Odom_GetRateDps();
                    s_brakeAngle = (turned < 0.0f) ? -turned : turned;

                    motion_halt();
                    s_brakeTicks = 0U;
                    s_state      = MOTION_BRAKE;
                    return;
                }
            }

            /* Going the wrong way. togo starts at the full turn and should
             * only shrink; if it has grown well past its starting value the
             * robot is rotating away from the target and will never arrive.
             * Stop now rather than let the watchdog run the full 15 s. */
            {
                float target_mag = (s_arcTargetDeg >= 0.0f)
                                 ? s_arcTargetDeg : -s_arcTargetDeg;

                if (togo > (target_mag + MOTION_ARC_WRONGWAY_DEG))
                {
                    motion_halt();
                    s_wrongWay = 1U;
                    s_state    = MOTION_TIMEOUT;
                    return;
                }
            }

            want = (togo <= prof()->approach_deg)
                 ? (int16_t)(s_dir * prof()->approach_rpm)
                 : (int16_t)(s_dir * prof()->rpm);

            if (want != s_lastRpm)
            {
                motion_issue(want);
                s_lastRpm = want;
            }
            return;
        }

        if (remaining <= 0.0f)
        {
            /* Target reached. Brake and let it settle before declaring done -
             * reading the distance mid-coast would give a short answer. */
            motion_halt();
            s_brakeTicks = 0U;
            s_state      = MOTION_BRAKE;
            return;
        }

        want = (remaining <= MOTION_APPROACH_MM)
             ? (int16_t)(s_dir * MOTION_APPROACH_RPM)
             : (int16_t)(s_dir * MOTION_CRUISE_RPM);

        /* Only re-issue when the speed actually changes.
         *
         * Odom_DriveHeading() zeroes the heading error and slams the servo
         * back to centre. Calling it every tick means the heading loop is
         * reset 100 times a second and never keeps any state - the reported
         * error is always stale and the servo is fighting itself. */
        if (want != s_lastRpm)
        {
            motion_issue(want);
            s_lastRpm = want;
        }
    }
    else /* MOTION_BRAKE */
    {
        s_brakeTicks++;

        /* Recentre the steering, but ONLY ONCE THE WHEELS HAVE STOPPED.
         *
         * This used to fire at the start of the brake phase, and that was
         * wrong in a way that was easy to miss: the robot is still moving
         * during braking - it coasts about 8 degrees of rotation before it
         * settles - so a 60 us deflection held for 200 ms steers the robot
         * through exactly that coast. Every stop nudged to one side.
         *
         * Waiting for both encoders to report no movement removes the
         * guesswork. It also costs nothing on average, because the robot is
         * usually stationary well before the brake settle ends. */
        if (s_recentreStep == 0U)
        {
            if ((Encoder_A_GetDelta() == 0) && (Encoder_B_GetDelta() == 0))
            {
                Servo_SetMicroseconds((uint16_t)(SERVO_CENTER_US - SERVO_APPROACH_US));
                s_recentreStep = 1U;
                s_recentreTick = 0U;
            }
        }
        else if (s_recentreStep == 1U)
        {
            s_recentreTick++;
            if (s_recentreTick >= 15U)          /* 150 ms to travel 60 us */
            {
                Servo_SetMicroseconds(SERVO_CENTER_US);
                s_recentreStep = 2U;
            }
        }

        /* Finish when the brake has settled AND the steering is centred.
         * The second condition is capped by the tick count below so a wheel
         * that never quite reads zero cannot hang the primitive. */
        if ((s_brakeTicks >= MOTION_BRAKE_TICKS) &&
            ((s_recentreStep == 2U) || (s_brakeTicks >= (MOTION_BRAKE_TICKS * 2U))))
        {
            Servo_SetMicroseconds(SERVO_CENTER_US);
            Motors_Coast();
            s_recentreStep = 0U;

#if MOTION_XCHECK_ENABLE
            /* Second witness. The encoders measured how far the robot drove;
             * the radius is a calibrated constant; so arc/radius is an angle
             * that owes the gyro nothing. */
            if (s_kind == MOVE_ARC)
            {
                s_xcheckDeg    = 0.0f;
                s_xcheckErrPct = 0.0f;
                s_xcheckFailed = 0U;

                if (prof()->diff_boost <= MOTION_XCHECK_MAX_BOOST)
                {
                    float arc  = Odom_GetDistance() - s_arcStartDist;
                    float gyro = Odom_GetHeadingTotal();

                    if (gyro < 0.0f) { gyro = -gyro; }

                    /* Below about 20 degrees the two agree to within their
                     * own noise, so a comparison says nothing useful. */
                    if ((arc > 1.0f) && (gyro > 20.0f) &&
                        (prof()->radius_mm > 1.0f))
                    {
                        s_xcheckDeg = (arc / prof()->radius_mm)
                                    * (180.0f / 3.14159265f);

                        s_xcheckErrPct = ((s_xcheckDeg - gyro) / gyro) * 100.0f;

                        {
                            float m = s_xcheckErrPct;
                            if (m < 0.0f) { m = -m; }
                            s_xcheckFailed = (m > MOTION_XCHECK_TOL_PCT) ? 1U : 0U;
                        }
                    }
                }
            }
#endif

            /* Straight runs: fold this run's mean heading error into the
             * steering centre trim. Once per move, after it has finished, so
             * it cannot interfere with the loop while it is running. */
            if (s_kind == MOVE_STRAIGHT)
            {
                Odom_LearnTrim();
            }

#if MOTION_ARC_ADAPTIVE_BRAKE
            /* Learn. The coast just observed, against the rate we entered the
             * brake at, gives this floor's deceleration directly. */
            if (s_kind == MOVE_ARC)
            {
                float ended = Odom_GetHeadingTotal();
                float coast;

                if (ended < 0.0f) { ended = -ended; }
                coast = ended - s_brakeAngle;

                if ((coast > 0.5f) && (s_brakeRate > 5.0f))
                {
                    float meas = (s_brakeRate * s_brakeRate) / (2.0f * coast);

                    /* Reject nonsense rather than believe it - a turn stopped
                     * by hand, or a wheel slipping, would otherwise poison the
                     * estimate for every run after it. */
                    if ((meas > MOTION_ARC_DECEL_MIN) &&
                        (meas < MOTION_ARC_DECEL_MAX))
                    {
                        s_arcDecel += MOTION_ARC_LEARN_GAIN
                                    * (meas - s_arcDecel);
                    }

                    /* Whatever error survives alpha is latency. Positive
                     * means overshoot, so the lag is bigger than believed
                     * and the brakes need to go on sooner. Dividing by the
                     * rate converts degrees of error into seconds of lag,
                     * which is why this stays correct at other speeds. */
                    {
                        float want = (s_arcCommandDeg < 0)
                                   ? (float)(-s_arcCommandDeg)
                                   : (float)s_arcCommandDeg;
                        float over = ended - want;

                        s_arcLag += MOTION_ARC_LAG_GAIN * (over / s_brakeRate);

                        if (s_arcLag < 0.0f)                { s_arcLag = 0.0f; }
                        if (s_arcLag > MOTION_ARC_LAG_MAX_S) { s_arcLag = MOTION_ARC_LAG_MAX_S; }
                    }
                }
            }
#endif
            s_state        = MOTION_DONE;
        }
    }
}

uint8_t Motion_WrongWayAborted(void) { return s_wrongWay; }

uint8_t Motion_IsBusy(void)
{
    return ((s_state == MOTION_ALIGN) ||
            (s_state == MOTION_RUN)   ||
            (s_state == MOTION_BRAKE)) ? 1U : 0U;
}

MotionState_t Motion_GetState(void) { return s_state; }

void Motion_ClearState(void)
{
    uint32_t pm = crit_enter();

    if ((s_state == MOTION_DONE) || (s_state == MOTION_TIMEOUT))
    {
        s_state = MOTION_IDLE;
    }

    crit_exit(pm);
}

int32_t Motion_GetRemaining(void)
{
    return (int32_t)(s_targetMm - Odom_GetDistance());
}

int32_t Motion_GetTravelled(void)
{
    return (int32_t)Odom_GetDistance();
}

int32_t Motion_GetTurnedDeg(void)
{
    return (int32_t)Odom_GetHeadingTotal();
}

int32_t Motion_GetTurnTargetDeg(void)
{
    /* The COMMANDED angle, not the brake-compensated one. */
    return (int32_t)s_arcCommandDeg;
}
