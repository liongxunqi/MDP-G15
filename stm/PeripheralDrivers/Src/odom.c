#include "odom.h"
#include "encoders.h"
#include "pid.h"
#include "motors.h"
#include "imu.h"
#include <math.h>

static Odom_Pose_t s_pose;

static int32_t s_lastCountA;
static int32_t s_lastCountB;

/* Previous gyro heading, so the per-tick delta can be differenced out of the
 * IMU's free-running total. */
static float   s_lastImuHeading;

/* Unwrapped heading, degrees, since the last Odom_Reset(). s_pose.heading_deg
 * is wrapped to +-180 because the heading-hold loop needs a bounded error;
 * an arc needs the opposite - a 270 degree turn has to read 270, not -90. */
static float   s_headingTotal;
static float   s_rateDps;

/* Drive mode. OFF means nothing here touches the motors or the servo. */
typedef enum
{
    ODOM_OFF = 0,
    ODOM_STRAIGHT,
    ODOM_ARC
} OdomMode_t;

static OdomMode_t s_mode;

/* Arc state: the two rear wheel setpoints, precomputed from the geometry,
 * plus the scale factors so a speed change can be reapplied without
 * re-deriving the geometry or touching the servo. */
static int16_t  s_arcRpmA;
static int16_t  s_arcRpmB;
static float    s_arcScaleA = 1.0f;
static float    s_arcScaleB = 1.0f;

/* Heading hold state */
static uint8_t  s_holdActive;
static int16_t  s_holdRpm;
static float    s_holdHeading;
static float    s_error;
static float    s_headingTrim;   /* learned centre offset, us */
static float    s_errSum;        /* mean-error accumulator, this run */
static uint32_t s_errCount;
static uint16_t s_servoUs;

/* Spin calibration state */
static int32_t s_spinStartA;
static int32_t s_spinStartB;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/* Heading change from the wheel difference, degrees.
 *
 * NOTE THE MINUS SIGN. Without it this is the OPPOSITE SIGN to the gyro, and
 * that is not a cosmetic difference - it inverts the feedback.
 *
 * Motor A is the LEFT rear. On a left (counter-clockwise) turn the left wheel
 * is on the inside and travels less, so (dA - dB) is negative: the raw
 * expression is CLOCKWISE-positive. The gyro is counter-clockwise-positive.
 *
 * This bit us. On a run where the IMU had not come up, the fallback silently
 * handed the heading loop and the arc logic a sign-flipped heading. A
 * commanded 90 degree right turn ran to 369 degrees before the watchdog
 * stopped it, because "degrees still to go" was growing instead of shrinking.
 *
 * A fallback that inverts the sign is worse than no fallback at all - it
 * fails loudly in the wrong direction instead of quietly losing accuracy.
 * With the minus sign both sources agree, so HEADING_SIGN and
 * MOTION_ARC_SIGN stay valid whichever one is in use. */
#define ODOM_ENCODER_HEADING() \
    (-((dA_mm - dB_mm) / WHEEL_BASE_MM) * (180.0f / 3.14159265f))

/* Keeps an angle in -180..+180. Without this, heading errors across the
 * +-180 boundary come out as ~360 degrees and the correction slams the
 * wrong way. */
static float wrap180(float deg)
{
    while (deg >  180.0f) { deg -= 360.0f; }
    while (deg < -180.0f) { deg += 360.0f; }
    return deg;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/* ------------------------------------------------------------------ */
/* Pose integration                                                    */
/* ------------------------------------------------------------------ */

void Odom_Init(void)
{
    s_lastCountA = Encoder_A_GetCount();
    s_lastCountB = Encoder_B_GetCount();

    s_mode       = ODOM_OFF;
    s_holdActive = 0U;
    s_holdRpm    = 0;
    s_arcRpmA    = 0;
    s_arcRpmB    = 0;
    s_error      = 0.0f;
    s_servoUs    = SERVO_CENTER_US;

    Odom_Reset();
}

void Odom_Reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    s_pose.x_mm        = 0.0f;
    s_pose.y_mm        = 0.0f;
    s_pose.heading_deg = 0.0f;
    s_pose.distance_mm = 0.0f;
    s_headingTotal     = 0.0f;

    /* Resync the cached counts against the live encoders.
     *
     * Without this, Odom_Reset() only zeroes the pose while s_lastCountA/B
     * keep whatever they held, so the next Odom_Update() computes its delta
     * against a stale baseline. That is harmless as long as the encoder
     * counters themselves never move underneath us - but the moment anything
     * calls Encoders_Reset(), the counts drop to zero and the next tick sees
     * a delta of minus fifteen thousand counts, which is about two metres of
     * phantom travel injected in 10 ms.
     *
     * Interrupts are masked because the control tick reads exactly these two
     * variables, and a reset landing mid-update would leave one wheel synced
     * and the other not. */
    s_lastCountA = Encoder_A_GetCount();
    s_lastCountB = Encoder_B_GetCount();

    /* Zero the gyro total too, and resync the baseline against it. Without
     * this the next tick differences the new pose against a heading the IMU
     * accumulated before the reset, and the first tick of every move gets a
     * step of whatever the robot had turned since boot. */
    IMU_ResetHeading();
    s_lastImuHeading = 0.0f;

    __set_PRIMASK(primask);
}

void Odom_Update(void)
{
    int32_t countA;
    int32_t countB;
    float   dA_mm;
    float   dB_mm;
    float   d_centre;
    float   d_theta_deg;
    float   heading_rad;
    float   correction;

    countA = Encoder_A_GetCount();
    countB = Encoder_B_GetCount();

    /* Distance each wheel rolled since the last tick. */
    dA_mm = (float)(countA - s_lastCountA) * MM_PER_COUNT_A;
    dB_mm = (float)(countB - s_lastCountB) * MM_PER_COUNT_B;

    s_lastCountA = countA;
    s_lastCountB = countB;

    /* Centre of the axle moves the average of the two wheels; the robot
     * rotates by their difference over the track width. At 10 ms and 250 RPM
     * a step is under 4 mm, so treating the arc as a straight line is fine.
     *
     * On an Ackermann chassis the two rear wheels differ only slightly on a
     * gentle curve, so this heading estimate is noisy. It is good enough for
     * A.3 and A.4; the IMU will replace it later. */
    /* Distance source. The heading term below ALWAYS uses both wheels - it is
     * a difference, so there is no single-wheel substitute - but the distance
     * travelled can come from one encoder alone when the other is not
     * trustworthy. See ODOM_DIST_SOURCE in odom.h. */
#if   ODOM_DIST_SOURCE == ODOM_DIST_A_ONLY
    d_centre    = dA_mm;
#elif ODOM_DIST_SOURCE == ODOM_DIST_B_ONLY
    d_centre    = dB_mm;
#else
    d_centre    = (dA_mm + dB_mm) * 0.5f;
#endif
    /* Heading change this tick.
     *
     * The gyro measures yaw directly, so wheel scale error, slip and tyre
     * scrub cannot corrupt it - all three of which the encoder difference is
     * wide open to. IMU_GetHeading() is a free-running total, so difference
     * it rather than using it as an absolute: that keeps everything below,
     * including the wrap to +-180 and the pose integration, working exactly
     * as it did with the encoder estimate.
     *
     * Falls back automatically if the IMU never came up, so a dead sensor
     * costs accuracy rather than leaving the robot with no heading at all. */
#if ODOM_HEADING_SOURCE == ODOM_HEADING_IMU
    if (IMU_IsReady())
    {
        float h = IMU_GetHeading();
        d_theta_deg      = h - s_lastImuHeading;
        s_lastImuHeading = h;
    }
    else
    {
        d_theta_deg = ODOM_ENCODER_HEADING();
    }
#else
    d_theta_deg = ODOM_ENCODER_HEADING();
#endif

    /* Integrate at the midpoint heading rather than the start heading. */
    heading_rad = (s_pose.heading_deg + d_theta_deg * 0.5f) * (3.14159265f / 180.0f);

    s_pose.x_mm += d_centre * cosf(heading_rad);
    s_pose.y_mm += d_centre * sinf(heading_rad);

    s_pose.heading_deg = wrap180(s_pose.heading_deg + d_theta_deg);
    s_headingTotal    += d_theta_deg;
    s_rateDps          = d_theta_deg * 100.0f;   /* 10 ms tick */

    /* Path length, always positive, so reversing does not subtract from it. */
    s_pose.distance_mm += (d_centre < 0.0f) ? -d_centre : d_centre;

    /* ---- heading hold, via the steering servo ---- */

    if (s_mode == ODOM_ARC)
    {
        /* Servo is already parked at the arc angle. Just keep the two wheel
         * setpoints applied; there is no heading correction during a turn. */
        PID_SetTargets(s_arcRpmA, s_arcRpmB);
    }
    else if (s_holdActive)
    {
        /* Aim slightly off the held heading, leaning back toward the line.
         *
         * y_mm is left-positive and heading is counter-clockwise positive, so
         * being LEFT of the line calls for a NEGATIVE (rightward) aim - hence
         * the minus sign. Reversing flips it: backing up, leaning the nose
         * right moves the tail left. */
        {
            float aim = s_holdHeading;

#if CROSS_TRACK_ENABLE
            float cross = -CROSS_KP_DEG_PER_MM * s_pose.y_mm;

            cross = clampf(cross, -CROSS_MAX_DEG, CROSS_MAX_DEG);
            if (s_holdRpm < 0) { cross = -cross; }

            aim += cross;
#endif
            s_error = wrap180(aim - s_pose.heading_deg);
        }

        /* Deadband. Below this the servo would only hunt and buzz. */
        /* Collect the error for the between-runs trim update. Accumulating
         * is safe; ACTING on it inside the run is what destabilised the loop
         * against the linkage backlash. */
        s_errSum += s_error * ((s_holdRpm < 0) ? -1.0f : 1.0f);
        s_errCount++;

        if ((s_error < HEADING_DEADBAND_DEG) && (s_error > -HEADING_DEADBAND_DEG))
        {
            correction = 0.0f;
        }
        else
        {
            correction = HEADING_KP_US * s_error * (float)HEADING_SIGN;

            /* Reversing: the same steering angle bends the path the other
             * way relative to travel, so flip the correction. */
            if (s_holdRpm < 0) { correction = -correction; }

            /* Push through the linkage slack. Without this, corrections
             * smaller than the backlash do nothing at all, the error grows
             * until they exceed it, and the robot weaves. */
            if (correction > 0.0f)      { correction += SERVO_BACKLASH_US; }
            else if (correction < 0.0f) { correction -= SERVO_BACKLASH_US; }

            correction = clampf(correction, -HEADING_MAX_US, HEADING_MAX_US);
        }

        /* Trim added AFTER the direction flip - it is a servo-space constant,
         * not a heading-space correction. */
        s_servoUs = (uint16_t)((float)SERVO_CENTER_US + correction + s_headingTrim);
        Servo_SetMicroseconds(s_servoUs);   /* clamps to MIN/MAX internally */

        /* Both rear wheels at the same speed. The PID's only job now is to
         * make A and B match each other - steering is the servo's. */
        PID_SetTargets(s_holdRpm, s_holdRpm);
    }
}

void Odom_GetPose(Odom_Pose_t *out)
{
    if (out != 0)
    {
        *out = s_pose;
    }
}

float    Odom_GetHeading(void)      { return s_pose.heading_deg; }
float    Odom_GetHeadingTotal(void) { return s_headingTotal; }
float    Odom_GetRateDps(void)      { return s_rateDps; }
float    Odom_GetDistance(void)     { return s_pose.distance_mm; }
uint16_t Odom_GetServoUs(void)      { return s_servoUs; }
float    Odom_GetHeadingError(void) { return s_error; }
float    Odom_GetHeadingTrim(void)  { return s_headingTrim; }
float    Odom_GetCrossTrack(void)   { return s_pose.y_mm; }

/* ------------------------------------------------------------------ */
/* Heading hold                                                        */
/* ------------------------------------------------------------------ */

/* SETTING HEADING_SIGN
 *
 * Two conventions have to agree and neither has been verified on this robot:
 * which way Odom heading counts, and which way rising microseconds steer.
 * HEADING_SIGN absorbs both. Find it like this, on the floor:
 *
 *   1. Flash with HEADING_SIGN as +1.
 *   2. Start a straight run and let it settle.
 *   3. Nudge the robot's nose to one side by hand, a few degrees.
 *
 *   Correct: the front wheels turn to steer BACK toward the original
 *   heading and the robot recovers.
 *
 *   Wrong: the front wheels turn the SAME way you nudged, the error grows,
 *   and the robot leaves the line immediately. Flip HEADING_SIGN to -1.
 *
 * The wrong sign is unmistakable - it diverges within a metre. Do this test
 * before spending any time on HEADING_KP_US. */

void Odom_DriveStraight(int16_t rpm)
{
    Odom_DriveHeading(rpm, s_pose.heading_deg);
}

uint8_t Odom_SetHeadingTrim(float us)
{
    if ((us < -HEADING_TRIM_MAX_US) || (us > HEADING_TRIM_MAX_US))
    {
        return 0U;
    }

    s_headingTrim = us;
    return 1U;
}

void Odom_LearnTrim(void)
{
    float mean;

    /* Too few samples to mean anything - a very short move, or one that was
     * aborted before the heading loop had settled. */
    if (s_errCount < 50U) { return; }

    mean = s_errSum / (float)s_errCount;

    /* A positive mean error means the robot sat to one side for the whole
     * run, which is exactly a centre offset. HEADING_SIGN converts heading
     * degrees into the servo direction that cancels them. */
    s_headingTrim += mean * HEADING_TRIM_GAIN * (float)HEADING_SIGN;
    s_headingTrim  = clampf(s_headingTrim,
                            -HEADING_TRIM_MAX_US, HEADING_TRIM_MAX_US);

    s_errSum   = 0.0f;
    s_errCount = 0U;
}

void Odom_DriveHeading(int16_t rpm, float heading_deg)
{
    s_errSum      = 0.0f;
    s_errCount    = 0U;
    s_holdHeading = wrap180(heading_deg);
    s_holdRpm     = rpm;
    s_error       = 0.0f;
    s_servoUs     = SERVO_CENTER_US;

    Servo_SetMicroseconds(SERVO_CENTER_US);
    s_holdActive  = 1U;
    s_mode        = ODOM_STRAIGHT;
}

void Odom_DriveArc(int16_t rpm, uint16_t servo_us, float radius_mm,
                   uint8_t right, float diff_boost)
{
    float half_track = WHEEL_BASE_MM * 0.5f;
    float inner_scale;
    float outer_scale;
    float mag;

    /* A radius inside the track width would ask the inner wheel to run
     * backwards, which this chassis cannot steer tightly enough to need.
     * Clamp rather than produce a nonsense setpoint. */
    if (radius_mm < (half_track + 1.0f))
    {
        radius_mm = half_track + 1.0f;
    }

    /* Geometric split, then the assist. ratio is how far each wheel departs
     * from the centre speed; boosting it exaggerates the difference without
     * moving the mean, so the axle centre still runs at the requested rpm. */
    {
        float ratio = (half_track / radius_mm) * diff_boost;

        if (ratio > ODOM_ARC_DIFF_MAX) { ratio = ODOM_ARC_DIFF_MAX; }

        inner_scale = 1.0f - ratio;
        outer_scale = 1.0f + ratio;
    }

    /* Scale about the axle-centre speed so the CENTRE runs at the requested
     * rpm. That keeps Odom_GetDistance(), which averages the two wheels,
     * measuring the arc length the caller asked for. */
    mag = (float)rpm;

    if (right)
    {
        /* Curving right: B (right rear) is the inner wheel. */
        s_arcScaleA = outer_scale;
        s_arcScaleB = inner_scale;
    }
    else
    {
        s_arcScaleA = inner_scale;
        s_arcScaleB = outer_scale;
    }

    s_arcRpmA = (int16_t)(mag * s_arcScaleA);
    s_arcRpmB = (int16_t)(mag * s_arcScaleB);

    s_holdActive = 0U;
    s_error      = 0.0f;
    s_servoUs    = servo_us;

    Servo_SetMicroseconds(servo_us);
    PID_SetTargets(s_arcRpmA, s_arcRpmB);

    s_mode = ODOM_ARC;
}

void Odom_SetSpeed(int16_t rpm)
{
    if (s_mode == ODOM_ARC)
    {
        s_arcRpmA = (int16_t)((float)rpm * s_arcScaleA);
        s_arcRpmB = (int16_t)((float)rpm * s_arcScaleB);
        PID_SetTargets(s_arcRpmA, s_arcRpmB);
    }
    else if (s_mode == ODOM_STRAIGHT)
    {
        /* s_holdRpm also carries the sign the heading correction is flipped
         * by when reversing, so it must be updated even though the servo is
         * deliberately left alone. */
        s_holdRpm = rpm;
        PID_SetTargets(rpm, rpm);
    }
    else
    {
        /* Not driving. Nothing to change. */
    }
}

void Odom_Stop(void)
{
    s_mode       = ODOM_OFF;
    s_holdActive = 0U;
    s_arcRpmA    = 0;
    s_arcRpmB    = 0;
    s_arcScaleA  = 1.0f;
    s_arcScaleB  = 1.0f;
    s_error      = 0.0f;
    s_servoUs    = SERVO_CENTER_US;

    Servo_SetMicroseconds(SERVO_CENTER_US);
    PID_Stop();
}

/* ------------------------------------------------------------------ */
/* Spin calibration                                                    */
/* ------------------------------------------------------------------ */

void Odom_CalibrateSpinStart(void)
{
    s_spinStartA = Encoder_A_GetCount();
    s_spinStartB = Encoder_B_GetCount();
}

float Odom_CalibrateSpinResult(void)
{
    float dA_mm;
    float dB_mm;
    float diff;

    dA_mm = (float)(Encoder_A_GetCount() - s_spinStartA) * MM_PER_COUNT;
    dB_mm = (float)(Encoder_B_GetCount() - s_spinStartB) * MM_PER_COUNT;

    diff = dA_mm - dB_mm;
    if (diff < 0.0f) { diff = -diff; }

    return diff / 3.14159265f;
}
