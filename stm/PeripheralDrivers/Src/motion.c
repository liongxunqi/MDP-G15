#include "motion.h"
#include "odom.h"
#include "pid.h"
#include "motors.h"
#include "encoders.h"
#include "caltab.h"
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
static uint8_t  s_approaching;   /* fast straight: the down-ramp has begun   */
static uint32_t s_ticks;
static uint32_t s_brakeTicks;
static uint32_t s_alignTicks;
static uint16_t s_startServoUs;
static float    s_arcTargetDeg;   /* signed, unwrapped, relative to launch */

/* Protocol 10 - PER-MOVE SPEED AND THE CALIBRATION TABLE in motion.h. Set
 * when a move is queued, held to its end. A field the table did not have is
 * filled from wherever protocol 9 took it, so the tick reads only these. */
static CtMove_t s_move;           /* which row of the table               */
static int16_t  s_vRpm;           /* V cruise, 0 = protocol 9 speed       */
static int16_t  s_cruiseRpm;      /* the cruise actually used, |rpm|      */
static float    s_mDecel;         /* arcs, from the table when s_mDecelTab */
static float    s_mLag;
static uint8_t  s_mDecelTab;      /* 0: live s_arcDecel, as protocol 9    */
static uint8_t  s_mLagTab;
static float    s_mRadius;        /* arcs: wheel split                    */
static float    s_mBoost;
static uint8_t  s_mTrimOn;        /* straights: the table set the trim    */

/* MOVE REPORT in motion.h. Filled as the move runs, final at settle. */
static MotionReport_t s_rep;
static int32_t  s_encA0;
static int32_t  s_encB0;
static float    s_sumRpmA, s_sumRpmB, s_sumDutyA, s_sumDutyB;
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
    /* 0: the calibrated one. Tight and quick. Saved calibrations taken
     *    before SAFE existed were all taken on this, and still fit it. */
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
    { "SLOW",  575U,  70,  70, 15.0f,   3.0f, 306.0f, 1.5f },

    /* 3: TIGHT slowed down and de-scrubbed. Same steering, cruise 150 ->
     *    120, and diff_boost 2.0 -> 1.2.
     *
     *    At 2.0 the rear wheels are split for a ~15 cm circle while the front
     *    steers ~29 cm, so the radius is a tug-of-war between tyre slip and
     *    rear scrub: it moves with floor, battery and direction, forward and
     *    reverse disagree, and per-side steering barely moves it - which is
     *    why the radius step would not settle. At 1.2 the steering decides
     *    the radius again. Expect it WIDER than TIGHT's.
     *
     *    approach_rpm 100: inner wheel at ~0.74 of centre, ~74 rpm, well
     *    clear of the ~55 rpm floor.
     *
     *    radius_mm is TIGHT's, NOT MEASURED. Floor-chord it on SAFE
     *    (calibrate.py radius NAME --arc safe --target auto) and put the
     *    answer here and in the planner's TURN_RADIUS_MM[PROFILE_SAFE]. It
     *    only sets the wheel split; the gyro still ends the turn. */
    { "SAFE",  575U, 120, 100, 20.0f,   8.0f, 291.0f, 1.2f }
};

#if MOTION_BUILD_SAFE
static uint8_t s_profileIdx = MOTION_PROFILE_SAFE;
#else
static uint8_t s_profileIdx = MOTION_PROFILE_TIGHT;
#endif

/* Learned angular deceleration and the state needed to measure it. */
static float s_arcDecel   = MOTION_ARC_DECEL_DPS2;

/* Has anything better than the compiled constant been put into s_arcDecel /
 * s_arcLag yet this power-on? Until it has, the first valid measurement is
 * taken OUTRIGHT rather than filtered toward - see the learner at the end of
 * an arc. Set by the !CAL* setters too: a restored value is a real prior and
 * must not be thrown away by the next arc.
 *
 * Declared here rather than beside the learner so they sit with the values
 * whose provenance they describe. */
static uint8_t s_decelSeeded = 0U;
static float s_brakeRate  = 0.0f;   /* |yaw rate| when braking started */
static float s_brakeAngle = 0.0f;   /* |heading| when braking started  */
static float s_arcLag     = MOTION_ARC_LAG_S;
static uint8_t s_lagSeeded   = 0U;

/* The last arc's own decel measurement, before any filtering. 0 when the
 * last arc did not produce one (learning off, rejected, or not an arc). */
static float s_lastDecelMeas = 0.0f;

/* Learning switch - see Motion_SetLearning() in motion.h. OFF at power-on:
 * nothing changes the calibration unless calibration asked it to. */
static uint8_t s_learnOn = 0U;

/* Heading carry-over - see HEADING CARRY-OVER in motion.h.
 *
 * s_worldBase is the actual heading folded in at the last launch; the live
 * actual heading is s_worldBase + Odom_GetHeadingTotal(), because the odometry
 * total runs from that same launch. Folding at LAUNCH rather than at the end
 * of a move means anything that happened in between - drift while parked, a
 * nudge during the steering settle - is counted as well. */
static float    s_worldCmd      = 0.0f;
static float    s_worldBase     = 0.0f;
static float    s_lastArcAim    = 0.0f;
static float    s_lastArcTurned = 0.0f;
static uint16_t s_carryClamps   = 0U;

/* Cross-check results for the last arc. */
static float   s_xcheckDeg    = 0.0f;
static float   s_xcheckErrPct = 0.0f;
static uint8_t s_xcheckFailed = 0U;
static float   s_arcStartDist = 0.0f;

float   Motion_GetXCheckDeg(void)    { return s_xcheckDeg; }
float   Motion_GetXCheckErrPct(void) { return s_xcheckErrPct; }
uint8_t Motion_XCheckFailed(void)    { return s_xcheckFailed; }

float Motion_GetArcDecel(void) { return s_arcDecel; }
float Motion_GetLastDecelMeas(void) { return s_lastDecelMeas; }
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

    s_arcDecel    = dps2;
    s_decelSeeded = 1U;   /* a restored value is a prior worth keeping */
    return 1U;
}

uint8_t Motion_SetArcLag(float secs)
{
    if ((secs < 0.0f) || (secs > MOTION_ARC_LAG_MAX_S)) { return 0U; }

    s_arcLag    = secs;
    s_lagSeeded = 1U;
    return 1U;
}

static const ArcProfile_t *prof(void) { return &s_profiles[s_profileIdx]; }

/* Clamp the deflection to what the servo limits allow on the TIGHTER side.
 *
 * Servo_SetMicroseconds() clamps silently, so an over-large deflection would
 * shorten one direction only and turns would come out lopsided with nothing
 * on screen to say why. Taking the smaller of the two available sides keeps
 * left and right identical whatever a profile asks for. */
/* Per-side deflection, [0] left [1] right. 0 = the symmetric profile value.
 * See PER-SIDE ARC STEERING in motion.h. */
static uint16_t s_steerSideUs[2] = { 0U, 0U };

static uint16_t arc_steer_us(void)
{
    uint16_t hi = (uint16_t)(SERVO_MAX_US - SERVO_CENTER_US);
    uint16_t lo = (uint16_t)(SERVO_CENTER_US - SERVO_MIN_US);
    uint16_t cap = (hi < lo) ? hi : lo;

    return (prof()->steer_us > cap) ? cap : prof()->steer_us;
}

uint8_t Motion_ArcProfileAllowed(uint8_t idx)
{
    if (idx >= MOTION_ARC_PROFILE_COUNT) { return 0U; }
#if MOTION_PROFILE_LOCK
    return ((idx == MOTION_PROFILE_TIGHT) || (idx == MOTION_PROFILE_SAFE)) ? 1U : 0U;
#else
    return 1U;
#endif
}

uint8_t Motion_SetArcProfile(uint8_t idx)
{
#if MOTION_BUILD_SAFE
    /* See SAFE BUILD in motion.h: profile 0 means SAFE on this build. */
    if (idx == MOTION_PROFILE_TIGHT) { idx = MOTION_PROFILE_SAFE; }
#endif
    if (!Motion_ArcProfileAllowed(idx)) { return 0U; }
    s_profileIdx = idx;
    return 1U;
}

void    Motion_SetLearning(uint8_t on)  { s_learnOn = on ? 1U : 0U; }

/* Fast mode - see MOTION FAST MODE in motion.h. */
static uint8_t s_fast = 0U;

void    Motion_SetFast(uint8_t on)      { s_fast = on ? 1U : 0U; }
uint8_t Motion_FastEnabled(void)        { return s_fast; }

static int16_t  cruise_rpm(void)  { return s_fast ? MOTION_FAST_CRUISE_RPM : MOTION_CRUISE_RPM; }
static uint32_t brake_ticks(void) { return s_fast ? MOTION_FAST_BRAKE_TICKS : MOTION_BRAKE_TICKS; }
uint8_t Motion_LearningEnabled(void)    { return s_learnOn; }

float Motion_GetCommandedHeading(void)  { return s_worldCmd; }
float Motion_GetActualHeading(void)     { return s_worldBase + Odom_GetHeadingTotal(); }
float Motion_GetLastArcAimDeg(void)     { return s_lastArcAim; }
float Motion_GetLastArcTurnedDeg(void)  { return s_lastArcTurned; }
uint16_t Motion_GetCarryClampCount(void) { return s_carryClamps; }

uint8_t  Motion_GetArcProfile(void)  { return s_profileIdx; }
uint16_t Motion_GetArcSteerUs(void)  { return arc_steer_us(); }

uint16_t Motion_GetArcSteerSideUs(uint8_t right)
{
    uint16_t us = s_steerSideUs[right ? 1U : 0U];
    return (us != 0U) ? us : arc_steer_us();
}

uint8_t Motion_SetArcSteerSideUs(uint8_t right, uint16_t us)
{
    /* How far this side can go before the servo clamps it. */
    uint16_t room = right ? (uint16_t)(SERVO_MAX_US - SERVO_CENTER_US)
                          : (uint16_t)(SERVO_CENTER_US - SERVO_MIN_US);

    if ((us < MOTION_ARC_STEER_MIN_US) || (us > room)) { return 0U; }

    s_steerSideUs[right ? 1U : 0U] = us;
    return 1U;
}

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

/* Set when an arc finished more than MOTION_ARC_FAIL_DEG from its aim -
 * see there. Cleared when the next primitive launches. */
static uint8_t  s_arcMissed;

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

/* Brake out of an arc with the steering left at the arc angle.
 *
 * motion_halt() recentres at once, and at brake onset the robot still has
 * several degrees of coast to go - so the front wheels swung through ~575 us
 * while it slid, and the coast was neither the arc nor a straight. That tail
 * is in every stop angle, every learned decel and every radius chord, and it
 * differs forward to reverse. The BRAKE phase recentres once the encoders
 * read still, which is what that code was always meant to do. */
static void motion_halt_arc(void)
{
    Odom_StopHoldSteer();
    PID_Enable(0);
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

#if MOTION_ALIGN_SKIP_FIX
    /* Skipping the settle also skips the halfway step that brings the servo
     * back UP onto the target, so the approach above would launch the move
     * with the servo on its way to target - SERVO_APPROACH_US. Go to the
     * target itself instead. See MOTION_ALIGN_SKIP_FIX in motion.h. */
    if (s_alignTicks != 0U)
    {
        Servo_SetMicroseconds(servo_us);
    }
#endif

    s_ticks = 0U;
    s_state = MOTION_ALIGN;
}

/* How far the actual heading trails the commanded one, capped. Positive means
 * the robot needs to turn further counter-clockwise. Call AFTER the fold in
 * motion_launch(), when s_worldBase is the whole of the actual heading. */
static float carry_error(void)
{
#if MOTION_CARRY_ENABLE
    float err = s_worldCmd - s_worldBase;

    if ((err > MOTION_CARRY_MAX_DEG) || (err < -MOTION_CARRY_MAX_DEG))
    {
        s_carryClamps++;
        err = (err > 0.0f) ? MOTION_CARRY_MAX_DEG : -MOTION_CARRY_MAX_DEG;
    }
    return err;
#else
    return 0.0f;
#endif
}

/* MOVE REPORT: start a fresh one. Called at launch, after the reset. */
static void report_begin(void)
{
    uint16_t seq = (uint16_t)(s_rep.seq + 1U);
    uint8_t  i;
    uint8_t *p = (uint8_t *)&s_rep;

    for (i = 0U; i < (uint8_t)sizeof(s_rep); i++) { p[i] = 0U; }

    s_rep.seq    = seq;
    s_rep.move   = (uint8_t)(s_move + 1U);
    s_rep.rpm    = s_cruiseRpm;
    s_rep.target = (s_kind == MOVE_ARC) ? s_arcTargetDeg : s_targetMm;
    s_rep.servo  = (s_kind == MOVE_ARC)
                 ? s_arcServoUs
                 : (uint16_t)((float)SERVO_CENTER_US + Odom_GetEffectiveTrim() + 0.5f);
    s_encA0      = Encoder_A_GetCount();
    s_encB0      = Encoder_B_GetCount();
    s_sumRpmA    = 0.0f;
    s_sumRpmB    = 0.0f;
    s_sumDutyA   = 0.0f;
    s_sumDutyB   = 0.0f;
}

/* One tick of the steady part - see "Cruise" in MOVE REPORT. */
static void report_sample(void)
{
    int32_t ra, rb;
    int16_t da, db;
    int16_t now = (s_lastRpm < 0) ? (int16_t)(-s_lastRpm) : s_lastRpm;

    if ((s_ticks <= MOTION_REPORT_WARMUP_TICKS) || (now != s_cruiseRpm) ||
        (s_rep.n >= 0xFFFFU))
    {
        return;
    }

    ra = Encoder_A_GetRPM();
    rb = Encoder_B_GetRPM();
    da = PID_GetOutputA();
    db = PID_GetOutputB();

    s_sumRpmA  += (float)((ra < 0) ? -ra : ra);
    s_sumRpmB  += (float)((rb < 0) ? -rb : rb);
    s_sumDutyA += (float)((da < 0) ? -da : da);
    s_sumDutyB += (float)((db < 0) ? -db : db);
    s_rep.n++;
}

/* Close the report. Straights must call this BEFORE Odom_LearnTrim(), which
 * clears the correction it reads. */
static void report_finish(uint8_t result)
{
    Odom_Pose_t p;

    Odom_GetPose(&p);
    s_rep.result = result;
    s_rep.ticks  = (uint16_t)((s_ticks > 0xFFFFU) ? 0xFFFFU : s_ticks);
    s_rep.enc_a  = Encoder_A_GetCount() - s_encA0;
    s_rep.enc_b  = Encoder_B_GetCount() - s_encB0;
    if (s_rep.n > 0U)
    {
        s_rep.rpm_a  = s_sumRpmA  / (float)s_rep.n;
        s_rep.rpm_b  = s_sumRpmB  / (float)s_rep.n;
        s_rep.duty_a = s_sumDutyA / (float)s_rep.n;
        s_rep.duty_b = s_sumDutyB / (float)s_rep.n;
    }
    s_rep.dist = Odom_GetDistance();
    s_rep.x    = p.x_mm;
    s_rep.y    = p.y_mm;

    if (s_kind == MOVE_ARC)
    {
        s_rep.aim        = s_arcTargetDeg;
        s_rep.turned     = Odom_GetHeadingTotal();
        s_rep.brake_rate = s_brakeRate;
        s_rep.brake_ang  = s_brakeAngle;
    }
    else
    {
        s_rep.corr    = Odom_GetMeanCorrection();
        s_rep.max_err = Odom_GetMaxHeadingError();
        s_rep.trim    = Odom_GetEffectiveTrim();
    }
}

const MotionReport_t *Motion_GetReport(void) { return &s_rep; }

/* Settle finished: zero the odometry now, then start driving. */
static void motion_launch(void)
{
    float carry;

    s_wrongWay      = 0U;
    s_arcMissed     = 0U;
    s_lastDecelMeas = 0.0f;

    /* Fold everything the gyro saw since the last launch into the running
     * actual heading BEFORE the reset below throws it away. */
    s_worldBase += Odom_GetHeadingTotal();
    carry = carry_error();

    Odom_Reset();

    if (s_kind == MOVE_ARC)
    {
        /* Aim for the commanded total, not for "asked from here". Never let
         * the correction flip the turn round or shrink it to nothing - a
         * correction that big has already been capped above anyway. */
        float aim = s_arcTargetDeg + carry;

        if ((s_arcTargetDeg >= 0.0f) && (aim < 1.0f))  { aim = 1.0f;  }
        if ((s_arcTargetDeg <  0.0f) && (aim > -1.0f)) { aim = -1.0f; }

        s_arcTargetDeg = aim;
        s_lastArcAim   = aim;
        s_worldCmd    += (float)s_arcCommandDeg;
    }

    s_arcStartDist = Odom_GetDistance();   /* zero, just after the reset */

    s_ticks      = 0U;
    s_brakeTicks = 0U;
    /* A fast straight starts at the bottom of its ramp - see FAST MODE in
     * motion.h. Arcs and normal straights start at full speed, as they
     * always have: their calibration was measured that way. */
    s_approaching = 0U;
    if (s_vRpm > 0)
    {
        /* A V move: arcs start at their speed, as arcs always have;
         * straights from the bottom of the fast ramp. */
        s_lastRpm = (s_kind == MOVE_ARC)
                  ? (int16_t)(s_dir * s_vRpm)
                  : (int16_t)(s_dir * ((s_vRpm < MOTION_RAMP_START_RPM)
                                       ? s_vRpm : MOTION_RAMP_START_RPM));
    }
    else
    {
        s_lastRpm = (s_kind == MOVE_ARC)
                  ? (int16_t)(s_dir * prof()->rpm)
                  : (s_fast ? (int16_t)(s_dir * MOTION_RAMP_START_RPM)
                            : (int16_t)(s_dir * cruise_rpm()));
    }
    s_state      = MOTION_RUN;

    report_begin();
    PID_Enable(1);

    if (s_kind == MOVE_ARC)
    {
        Odom_DriveArc(s_lastRpm, s_arcServoUs, s_mRadius, s_arcRight, s_mBoost);
    }
    else
    {
        /* Zero just after the reset, plus whatever the last move left
         * behind - so the straight steers the error out instead of driving
         * the whole way on it. */
        s_holdHeading = Odom_GetHeading() + carry;
        Odom_DriveHeading(s_lastRpm, s_holdHeading);
    }
}

void Motion_ZeroHeading(void)
{
    uint32_t pm = crit_enter();

    Odom_Reset();
    s_worldCmd    = 0.0f;
    s_worldBase   = 0.0f;
    s_carryClamps = 0U;

    crit_exit(pm);
}

void Motion_ResyncHeading(void)
{
    uint32_t pm = crit_enter();
    s_worldCmd = s_worldBase + Odom_GetHeadingTotal();
    crit_exit(pm);
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
    s_decelSeeded   = 0U;
    s_lagSeeded     = 0U;
    s_lastDecelMeas = 0.0f;
    s_learnOn       = 0U;
    s_fast          = 0U;
    s_worldCmd      = 0.0f;
    s_worldBase     = 0.0f;
    s_lastArcAim    = 0.0f;
    s_lastArcTurned = 0.0f;
    s_carryClamps   = 0U;
}

void Motion_DriveDistance(int32_t mm)
{
    Motion_DriveDistanceV(mm, 0);
}

void Motion_DriveDistanceV(int32_t mm, int16_t rpm)
{
    float    target;
    float    v;
    float    coast = MOTION_BRAKE_MM;
    float    ratio = 1.0f;
    uint32_t pm;

    if (mm == 0) { return; }

    pm = crit_enter();

    s_kind = MOVE_STRAIGHT;
    s_dir  = (mm < 0) ? -1 : 1;
    s_move = (mm < 0) ? CT_R : CT_F;

    /* V, else the table's default, else protocol 9 (0). */
    s_vRpm      = (rpm > 0) ? rpm : CalTab_Global(CTG_VS);
    s_cruiseRpm = (s_vRpm > 0) ? s_vRpm
                : (s_fast ? (int16_t)MOTION_FAST_CRUISE_RPM : (int16_t)MOTION_CRUISE_RPM);

    /* This direction's calibration at this speed (caltab.h). */
    if (CalTab_Lookup(s_move, CTF_COAST, (float)s_cruiseRpm, &v)) { coast = v / 10.0f; }
    if (CalTab_Lookup(s_move, CTF_DIST,  (float)s_cruiseRpm, &v)) { ratio = v / 10000.0f; }
    s_mTrimOn = CalTab_Lookup(s_move, CTF_TRIM, (float)s_cruiseRpm, &v);
    Odom_SetTrimOverride(s_mTrimOn, s_mTrimOn ? v : 0.0f);

    /* dist is true / odometry, so the odometry has to read target / ratio
     * when the robot has really gone the distance asked for. */
    target = ((float)((mm < 0) ? -mm : mm) / ratio) - coast;
    if (target < 0.0f) { target = 0.0f; }
    s_targetMm = target;

    /* Settle onto where the wheels actually point straight - centre PLUS the
     * learned trim - not onto bare centre. Parking at centre launched every
     * straight steered off by the trim for its first ticks, and resolved the
     * linkage slack at a point the run never holds: approached from below
     * for a positive trim, from above for a negative one, so the run's
     * behaviour flipped whenever the trim crossed zero. */
    {
        float t = Odom_GetEffectiveTrim();
        int32_t us = (int32_t)SERVO_CENTER_US
                   + (int32_t)((t >= 0.0f) ? (t + 0.5f) : (t - 0.5f));

        motion_begin_align((uint16_t)us);
    }

    crit_exit(pm);
}

void Motion_DriveArc(int16_t degrees, uint8_t forward, uint8_t right)
{
    Motion_DriveArcV(degrees, forward, right, 0);
}

void Motion_DriveArcV(int16_t degrees, uint8_t forward, uint8_t right, int16_t rpm)
{
    float    rad;
    float    arc_mm;
    float    v;
    float    steer;
    uint32_t pm;

    if (degrees <= 0) { return; }

    pm = crit_enter();

    s_kind     = MOVE_ARC;
    s_dir      = forward ? 1 : -1;
    s_arcRight = right ? 1U : 0U;
    s_move     = forward ? (right ? CT_FR : CT_FL) : (right ? CT_RR : CT_RL);

    s_vRpm      = (rpm > 0) ? rpm : CalTab_Global(CTG_VA);
    s_cruiseRpm = (s_vRpm > 0) ? s_vRpm : prof()->rpm;

    /* This arc's calibration at this speed (caltab.h); protocol 9's values
     * for anything the table does not have. The deflection falls back to
     * the side's own - see PER-SIDE ARC STEERING. */
    steer = (float)Motion_GetArcSteerSideUs(right ? 1U : 0U);
    if (CalTab_Lookup(s_move, CTF_STEER, (float)s_cruiseRpm, &v)) { steer = v; }
    s_arcServoUs = right
                 ? (uint16_t)((float)SERVO_CENTER_US + steer + 0.5f)
                 : (uint16_t)((float)SERVO_CENTER_US - steer + 0.5f);

    s_mDecelTab = CalTab_Lookup(s_move, CTF_DECEL, (float)s_cruiseRpm, &v);
    s_mDecel    = s_mDecelTab ? (v / 10.0f) : 0.0f;
    s_mLagTab   = CalTab_Lookup(s_move, CTF_LAG, (float)s_cruiseRpm, &v);
    s_mLag      = s_mLagTab ? (v / 10000.0f) : 0.0f;
#if MOTION_ARC_GEOMETRIC_SPLIT
    /* Pure geometry - see MOTION_ARC_GEOMETRIC_SPLIT in motion.h. */
    s_mRadius   = CalTab_Lookup(s_move, CTF_RADIUS, (float)s_cruiseRpm, &v)
                ? v : MOTION_ARC_GEOM_RADIUS_MM;
    s_mBoost    = 1.0f;
#else
    s_mRadius   = CalTab_Lookup(s_move, CTF_RADIUS, (float)s_cruiseRpm, &v)
                ? v : prof()->radius_mm;
    s_mBoost    = CalTab_Lookup(s_move, CTF_BOOST, (float)s_cruiseRpm, &v)
                ? (v / 100.0f) : prof()->diff_boost;
#endif

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
    arc_mm     = s_mRadius * ((float)degrees * (3.14159265f / 180.0f));
    s_targetMm = arc_mm;

    /* An arc needs a much bigger steering movement than a straight line, so
     * the settle matters more here, not less. */
    motion_begin_align(s_arcServoUs);

    crit_exit(pm);
}

void Motion_Stop(void)
{
    uint32_t pm = crit_enter();

    if ((s_state == MOTION_RUN) || (s_state == MOTION_BRAKE))
    {
        report_finish(4U);
    }

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

/* Fast-mode straight: this tick's speed. Climbs toward cruise until what is
 * left is the down-ramp from the speed reached so far plus the tail, then
 * falls to MOTION_APPROACH_RPM and stays there. The switch latches: while
 * the wheels lag the ramp the distance left can briefly exceed the shrinking
 * threshold again, and flipping back to cruise there would surge. */
static int16_t fast_straight_rpm(float remaining)
{
    int16_t now = (s_lastRpm < 0) ? (int16_t)(-s_lastRpm) : s_lastRpm;
    int16_t want;

    if (!s_approaching &&
        (remaining <= (MOTION_RAMP_DOWN_MM(now) + MOTION_FAST_TAIL_MM)))
    {
        s_approaching = 1U;
    }

    if (s_approaching)
    {
        want = (int16_t)(now - MOTION_RAMP_DOWN_RPM_PER_TICK);
        if (want < MOTION_APPROACH_RPM) { want = MOTION_APPROACH_RPM; }
    }
    else
    {
        want = (int16_t)(now + MOTION_RAMP_UP_RPM_PER_TICK);
        if (want > MOTION_FAST_CRUISE_RPM) { want = MOTION_FAST_CRUISE_RPM; }
    }
    return (int16_t)(s_dir * want);
}

/* One ramp step from `now` toward `target`, rpm, positive, at the fast-mode
 * ramp rates. */
static int16_t ramp_toward(int16_t now, int16_t target)
{
    if (now < target)
    {
        now = (int16_t)(now + MOTION_RAMP_UP_RPM_PER_TICK);
        return (now > target) ? target : now;
    }
    if (now > target)
    {
        now = (int16_t)(now - MOTION_RAMP_DOWN_RPM_PER_TICK);
        return (now < target) ? target : now;
    }
    return now;
}

/* V straight: this tick's speed. Up toward its own cruise, then down in time
 * to run the last MOTION_FAST_TAIL_MM at MOTION_APPROACH_RPM - the same tail
 * as a fast straight, so the stop is the calibrated one. Latches like fast
 * mode. */
static int16_t v_straight_rpm(float remaining)
{
    int16_t now  = (s_lastRpm < 0) ? (int16_t)(-s_lastRpm) : s_lastRpm;
    float   down = (now > MOTION_APPROACH_RPM)
                 ? MOTION_RAMP_DOWN_MM_TO(now, MOTION_APPROACH_RPM) : 0.0f;

    if (!s_approaching && (remaining <= (down + MOTION_FAST_TAIL_MM)))
    {
        s_approaching = 1U;
    }
    return (int16_t)(s_dir * ramp_toward(now, s_approaching
                                              ? (int16_t)MOTION_APPROACH_RPM : s_vRpm));
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
        report_finish(1U);
        motion_halt();
        s_state = MOTION_TIMEOUT;
        Motion_ResyncHeading();   /* the plan no longer describes the robot */
        return;
    }

    travelled = Odom_GetDistance();     /* always positive */
    remaining = s_targetMm - travelled;

    if (s_state == MOTION_RUN)
    {
        report_sample();

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
                lead = ((w * w) / (2.0f * (s_mDecelTab ? s_mDecel : s_arcDecel)))
                     + (w * (s_mLagTab ? s_mLag : s_arcLag));
#if MOTION_ARC_TAPER
                /* Arriving at a crawl, the forced 2 degree lead would stop
                 * the turn short by most of it - see MOTION_ARC_TAPER. */
                if (lead < MOTION_ARC_TAPER_MIN_LEAD_DEG)
                {
                    lead = MOTION_ARC_TAPER_MIN_LEAD_DEG;
                }
#else
                if (lead < MOTION_ARC_MIN_LEAD_DEG)
                {
                    lead = MOTION_ARC_MIN_LEAD_DEG;
                }
#endif

                /* Brake on whichever tick is NEARER the ideal point. The
                 * robot turns w*dt between ticks (~0.7 deg at approach), so
                 * waiting until togo has fallen to the lead brakes late by
                 * anything up to a whole tick - half a tick on average, a
                 * bias that is the same size as the +/-0.5 deg target.
                 * Braking now when the ideal point is less than half a tick
                 * away makes the error zero-mean and halves the worst case. */
                lead += 0.5f * w * MOTION_TICK_S;
#endif
                if (togo <= lead)
                {
                    /* Remember what we were doing at brake onset - this is
                     * what makes the coast measurable afterwards. */
                    s_brakeRate  = (Odom_GetRateDps() < 0.0f)
                                 ? -Odom_GetRateDps() : Odom_GetRateDps();
                    s_brakeAngle = (turned < 0.0f) ? -turned : turned;
                    s_rep.dist_brake = Odom_GetDistance();

                    motion_halt_arc();
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
                    report_finish(2U);
                    motion_halt();
                    s_wrongWay = 1U;
                    s_state    = MOTION_TIMEOUT;
                    Motion_ResyncHeading();
                    return;
                }
            }

#if MOTION_ARC_TAPER
            {
                /* Speed falls with the angle left, down to a crawl floor -
                 * see MOTION_ARC_TAPER in motion.h. The cruise is this
                 * move's V if it has one, else the profile's. */
                int16_t cruise = (s_vRpm > 0) ? s_vRpm : prof()->rpm;
                float   v      = MOTION_ARC_TAPER_RPM_PER_DEG * togo;
                int16_t floor_rpm = (cruise < MOTION_ARC_TAPER_MIN_RPM)
                                  ? cruise : (int16_t)MOTION_ARC_TAPER_MIN_RPM;

                if (v > (float)cruise)    { v = (float)cruise; }
                if (v < (float)floor_rpm) { v = (float)floor_rpm; }

                want = (int16_t)(s_dir * (int16_t)(v + 0.5f));
            }
#else
            if (s_vRpm > 0)
            {
                /* This move's speed; the last approach_deg still slow to
                 * the profile's approach (or V, if lower), which is what the
                 * braking model is measured from. */
                int16_t apr = (prof()->approach_rpm < s_vRpm)
                            ? prof()->approach_rpm : s_vRpm;

                want = (togo <= prof()->approach_deg)
                     ? (int16_t)(s_dir * apr) : (int16_t)(s_dir * s_vRpm);
            }
            else
            {
                want = (togo <= prof()->approach_deg)
                     ? (int16_t)(s_dir * prof()->approach_rpm)
                     : (int16_t)(s_dir * prof()->rpm);
            }
#endif

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
            s_rep.dist_brake = Odom_GetDistance();
            motion_halt();
            s_brakeTicks = 0U;
            s_state      = MOTION_BRAKE;
            return;
        }

        if (s_vRpm > 0)
        {
            want = v_straight_rpm(remaining);
        }
        else if (s_fast)
        {
            want = fast_straight_rpm(remaining);
        }
        else
        {
            want = (remaining <= MOTION_APPROACH_MM)
                 ? (int16_t)(s_dir * MOTION_APPROACH_RPM)
                 : (int16_t)(s_dir * MOTION_CRUISE_RPM);
        }

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
        if ((s_brakeTicks >= brake_ticks()) &&
            ((s_recentreStep == 2U) || (s_brakeTicks >= (brake_ticks() * 2U))))
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

                if (s_mBoost <= MOTION_XCHECK_MAX_BOOST)
                {
                    float arc  = Odom_GetDistance() - s_arcStartDist;
                    float gyro = Odom_GetHeadingTotal();

                    if (gyro < 0.0f) { gyro = -gyro; }

                    /* Below about 20 degrees the two agree to within their
                     * own noise, so a comparison says nothing useful. */
                    if ((arc > 1.0f) && (gyro > 20.0f) &&
                        (s_mRadius > 1.0f))
                    {
                        s_xcheckDeg = (arc / s_mRadius)
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
            /* The report reads the straight's correction, which the trim
             * learner clears - so it goes first. */
            report_finish(0U);

            /* Not while the table set the trim: the correction is then
             * relative to the table's value, not to the learned one. */
            if ((s_kind == MOVE_STRAIGHT) && s_learnOn && !s_mTrimOn)
            {
                Odom_LearnTrim();
            }

            if (s_kind == MOVE_ARC)
            {
                float miss;

                s_lastArcTurned = Odom_GetHeadingTotal();

                miss = s_lastArcTurned - s_arcTargetDeg;
                if (miss < 0.0f) { miss = -miss; }
                s_arcMissed = (miss > MOTION_ARC_FAIL_DEG) ? 1U : 0U;
            }

#if MOTION_ARC_ADAPTIVE_BRAKE
            /* Learn. The coast just observed, against the rate we entered the
             * brake at, gives this floor's deceleration directly.
             *
             * Only while learning is switched on - see Motion_SetLearning().
             * During a task the braking model is exactly what was restored. */
            /* Not from a missed arc: its coast says nothing about braking. */
            if ((s_kind == MOVE_ARC) && !s_arcMissed)
            {
                float ended = Odom_GetHeadingTotal();
                float coast;

                if (ended < 0.0f) { ended = -ended; }

                /* The coast past brake onset, less the part the lag term
                 * already predicts. Then the decel below is exactly the one
                 * that would have made the predicted lead equal this coast:
                 *
                 *      w^2/(2a) + w*lag  =  coast
                 *
                 * Measuring a from the whole coast instead (as this did)
                 * puts the latency in BOTH terms, and the two learners then
                 * fight over the same degrees - which is why lag scattered
                 * by a quarter and decel never sat still. */
                coast = (ended - s_brakeAngle)
                      - (s_brakeRate * (s_mLagTab ? s_mLag : s_arcLag));

                if ((coast > 0.5f) && (s_brakeRate > 5.0f))
                {
                    float meas = (s_brakeRate * s_brakeRate) / (2.0f * coast);

                    /* Reject nonsense rather than believe it - a turn stopped
                     * by hand, or a wheel slipping, would otherwise poison the
                     * estimate for every run after it. */
                    if ((meas > MOTION_ARC_DECEL_MIN) &&
                        (meas < MOTION_ARC_DECEL_MAX))
                    {
                        /* Raw, for calibrate.py to average itself - the
                         * filtered value below still carries the prior. */
                        s_lastDecelMeas  = meas;
                        s_rep.decel_meas = meas;

                        /* Measured on every arc for the report; LEARNED only
                         * while learning is on, and never over a table value
                         * - the table is what was asked for. */
                        if (!s_learnOn || s_mDecelTab)
                        {
                        }
                        else
#if MOTION_ARC_SEED_FIRST
                        /* The first valid arc of a power-on is a direct
                         * measurement of THIS floor. What it is being
                         * filtered toward is a constant from a different day,
                         * so there is no prior here worth defending - take
                         * the measurement outright and spend the remaining
                         * arcs rejecting noise instead of catching up.
                         *
                         * At gain 0.25 the old rule needed about ten arcs to
                         * arrive, which meant the cold first run - the one
                         * A.3 and A.4 are graded on - was always driven on
                         * numbers nobody had measured here. */
                        if (!s_decelSeeded)
                        {
                            s_arcDecel    = meas;
                            s_decelSeeded = 1U;
                        }
                        else
#endif
                        {
                            s_arcDecel += MOTION_ARC_LEARN_GAIN
                                        * (meas - s_arcDecel);
                        }
                    }

#if MOTION_ARC_LEARN_LAG
                    if (s_learnOn && !s_mLagTab)
                    /* Whatever error survives alpha is latency. Positive
                     * means overshoot, so the lag is bigger than believed
                     * and the brakes need to go on sooner. Dividing by the
                     * rate converts degrees of error into seconds of lag,
                     * which is why this stays correct at other speeds. */
                    {
                        /* Against what the turn AIMED for, which after the
                         * heading carry-over is not always what was asked.
                         * Scoring a deliberate 90.5 against 90 would teach
                         * the lag a half-degree overshoot that never was. */
                        float want = (s_arcTargetDeg < 0.0f)
                                   ? -s_arcTargetDeg : s_arcTargetDeg;
                        float over = ended - want;

                        /* over / rate IS the lag error in seconds, not a
                         * proxy for it: braking Dt too late over-rotates by
                         * rate * Dt. So gain 1.0 is the deadbeat correction
                         * and MOTION_ARC_LAG_GAIN is pure damping. Same
                         * argument as the decel above - on the first arc
                         * there is nothing to damp against. */
#if MOTION_ARC_SEED_FIRST
                        if (!s_lagSeeded)
                        {
                            s_arcLag  += (over / s_brakeRate);
                            s_lagSeeded = 1U;
                        }
                        else
#endif
                        {
                            s_arcLag += MOTION_ARC_LAG_GAIN
                                      * (over / s_brakeRate);
                        }

                        if (s_arcLag < 0.0f)                { s_arcLag = 0.0f; }
                        if (s_arcLag > MOTION_ARC_LAG_MAX_S) { s_arcLag = MOTION_ARC_LAG_MAX_S; }
                    }
#endif
                }
            }
#endif
            if (s_arcMissed)
            {
                /* Finished, but nowhere near where it aimed - braked early
                 * off a wheel surge, or was stopped. The carry-over can only
                 * correct MOTION_CARRY_MAX_DEG, so the plan no longer
                 * describes the robot: fail it like any other move that did
                 * not happen as asked. */
                s_state = MOTION_TIMEOUT;
                s_rep.result = 3U;
                Motion_ResyncHeading();
            }
            else
            {
                s_state = MOTION_DONE;
            }
        }
    }
}

uint8_t Motion_WrongWayAborted(void) { return s_wrongWay; }
uint8_t Motion_ArcMissed(void)       { return s_arcMissed; }

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
