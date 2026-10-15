#include "pid.h"
#include "motors.h"
#include "encoders.h"

static PID_t s_pidA;
static PID_t s_pidB;

/* One flag for the pair. The per-struct 'enabled' fields are kept for
 * introspection but this is what PID_Update() gates on - the old code tested
 * s_pidA.enabled only, which meant wheel B silently inherited A's state. */
static volatile uint8_t s_enabled;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/* Open-loop duty estimate for a requested speed.
 *
 * The wheel does nothing below MOTOR_DEADBAND and reaches max_rpm at
 * MOTOR_SPEED_MAX, so the usable region is a line between those two points.
 * Seeding the output with this estimate means the integrator only has to
 * correct the error in the fit rather than discover the whole operating
 * point from zero, which is the difference between settling in a tenth of a
 * second and taking two seconds with a big overshoot on the way. */
/* Protocol 10: each wheel's own line, per direction, when the calibration
 * table has measured it (PID_SetMotorModel). A weak motor needs more duty for
 * the same speed; seeding it with the strong one's line leaves its integrator
 * to make up the difference on every launch, and the robot yaws off the line
 * while it does. */
static int16_t deadband_of(const PID_t *p)
{
    return (p->deadband > 0) ? p->deadband : (int16_t)MOTOR_DEADBAND;
}

static float feedforward(const PID_t *p, float rpm)
{
    float   span;
    float   mag = (rpm < 0.0f) ? -rpm : rpm;
    int16_t max_rpm = (rpm < 0.0f) ? p->ff_rev : p->ff_fwd;
    int16_t db = deadband_of(p);

    if (max_rpm <= 0) { max_rpm = p->max_rpm; }

    if (mag < 1.0f || max_rpm <= 0)
    {
        return 0.0f;
    }

#if PID_LOW_DUTY
    /* The running line, not the breakaway one - see PID_LOW_DUTY. */
    db = (int16_t)(db - PID_RUN_DB_DROP);
#endif

    span = (float)(MOTOR_SPEED_MAX - db);

    return (float)db + (mag / (float)max_rpm) * span;
}

/* Maps a signed controller output onto duty the motor can actually act on.
 *
 * Anything between 1 and MOTOR_DEADBAND is a command the motor ignores: the
 * wheel sits still, the error never shrinks, and the integrator keeps
 * climbing until the output finally clears the threshold and the wheel
 * lurches. So small outputs get pushed up to the deadband instead. Genuine
 * zero still means stop. */
static int16_t apply_deadband(float out, int16_t db, float measured_rpm)
{
    float mag = (out < 0.0f) ? -out : out;

    if (mag < 1.0f)
    {
        return 0;
    }

#if PID_LOW_DUTY
    /* Already turning: kinetic friction, no breakaway needed. See
     * PID_LOW_DUTY in pid.h. */
    if ((measured_rpm >= (float)PID_MOVING_RPM) ||
        (measured_rpm <= -(float)PID_MOVING_RPM))
    {
        if (mag < (float)PID_ZERO_DUTY)
        {
            return 0;
        }
        if (mag > (float)MOTOR_SPEED_MAX)
        {
            mag = (float)MOTOR_SPEED_MAX;
        }
        return (out < 0.0f) ? (int16_t)(-mag) : (int16_t)mag;
    }
#else
    (void)measured_rpm;
#endif

    if (mag < (float)db)
    {
        mag = (float)db;
    }
    else if (mag > (float)MOTOR_SPEED_MAX)
    {
        mag = (float)MOTOR_SPEED_MAX;
    }

    return (out < 0.0f) ? (int16_t)(-mag) : (int16_t)mag;
}

static void pid_reset(PID_t *p)
{
    p->target_rpm = 0.0f;
    p->integral   = 0.0f;
    p->last_error = 0.0f;
    p->output     = 0;
}

/* One controller step for a single wheel. */
static int16_t pid_step(PID_t *p, float measured_rpm)
{
    float error;
    float ff;
    float p_term;
    float d_term;
    float raw;
    float mag_target;

    mag_target = (p->target_rpm < 0.0f) ? -p->target_rpm : p->target_rpm;

    /* A target the wheel physically cannot hold. Stop cleanly rather than
     * chattering around a speed that is below the deadband. */
#if PID_LOW_DUTY
    if (mag_target < (float)PID_MIN_RPM_LOW)
#else
    if (mag_target < (float)PID_MIN_RPM)
#endif
    {
        pid_reset(p);
        return 0;
    }

    error = p->target_rpm - measured_rpm;

    /* Overspeeding with the integral still pushing the same way it is
     * already going too fast: drop the integral fast. See PID_UNWIND_FRAC.
     * Works for reverse too - there the integral is negative and an
     * overspeed makes the error positive. */
    if ((((error < 0.0f) && (p->integral > 0.0f)) ||
         ((error > 0.0f) && (p->integral < 0.0f))) &&
        (((error < 0.0f) ? -error : error) > (PID_UNWIND_FRAC * mag_target)))
    {
        p->integral *= PID_UNWIND_KEEP;
    }

    p_term = p->kp * error;

    /* Integrate before saturation is known, then undo it below if the output
     * saturated in the same direction. Simpler than conditional integration
     * and behaves the same for this kind of plant. */
    p->integral += p->ki * error * PID_DT_S;
    p->integral  = clampf(p->integral, -PID_I_LIMIT, PID_I_LIMIT);

    d_term = p->kd * (error - p->last_error) / PID_DT_S;
    p->last_error = error;

    ff = feedforward(p, p->target_rpm);
    if (p->target_rpm < 0.0f)
    {
        ff = -ff;
    }

    raw = ff + p_term + p->integral + d_term;

    /* Anti-windup: if we are pinned at the rail and the integral is pushing
     * further into it, the extra accumulation buys nothing and costs recovery
     * time when the setpoint drops. Back it out. */
    if (raw > (float)MOTOR_SPEED_MAX && p->integral > 0.0f)
    {
        p->integral -= (raw - (float)MOTOR_SPEED_MAX);
        if (p->integral < 0.0f) { p->integral = 0.0f; }
        raw = (float)MOTOR_SPEED_MAX;
    }
    else if (raw < -(float)MOTOR_SPEED_MAX && p->integral < 0.0f)
    {
        p->integral += (-(float)MOTOR_SPEED_MAX - raw);
        if (p->integral > 0.0f) { p->integral = 0.0f; }
        raw = -(float)MOTOR_SPEED_MAX;
    }

    p->output = apply_deadband(raw, deadband_of(p), measured_rpm);

    return p->output;
}

/* ------------------------------------------------------------------ */
/* Public                                                              */
/* ------------------------------------------------------------------ */

void PID_Init(void)
{
    s_pidA.kp = PID_KP_DEFAULT;
    s_pidA.ki = PID_KI_DEFAULT;
    s_pidA.kd = PID_KD_DEFAULT;
    s_pidA.max_rpm = MOTOR_A_MAX_RPM;
    s_pidA.ff_fwd  = 0;
    s_pidA.ff_rev  = 0;
    s_pidA.deadband = 0;
    s_pidA.enabled = 0U;
    pid_reset(&s_pidA);

    s_pidB.kp = PID_KP_DEFAULT;
    s_pidB.ki = PID_KI_DEFAULT;
    s_pidB.kd = PID_KD_DEFAULT;
    s_pidB.max_rpm = MOTOR_B_MAX_RPM;
    s_pidB.ff_fwd  = 0;
    s_pidB.ff_rev  = 0;
    s_pidB.deadband = 0;
    s_pidB.enabled = 0U;
    pid_reset(&s_pidB);

    s_enabled = 0U;
}

void PID_Enable(uint8_t on)
{
    if (!on)
    {
        Motor_A_Set(0);
        Motor_B_Set(0);
    }

    pid_reset(&s_pidA);
    pid_reset(&s_pidB);

    s_pidA.enabled = on ? 1U : 0U;
    s_pidB.enabled = s_pidA.enabled;
    s_enabled      = s_pidA.enabled;
}

uint8_t PID_IsEnabled(void) { return s_enabled; }

static int16_t clamp_target(int16_t rpm, int16_t max_rpm)
{
    if (rpm >  max_rpm) { return  max_rpm; }
    if (rpm < -max_rpm) { return -max_rpm; }
    return rpm;
}

void PID_SetTargetA(int16_t rpm)
{
    s_pidA.target_rpm = (float)clamp_target(rpm, s_pidA.max_rpm);
}

void PID_SetTargetB(int16_t rpm)
{
    s_pidB.target_rpm = (float)clamp_target(rpm, s_pidB.max_rpm);
}

void PID_SetTargets(int16_t rpm_a, int16_t rpm_b)
{
    PID_SetTargetA(rpm_a);
    PID_SetTargetB(rpm_b);
}

void PID_Stop(void)
{
    pid_reset(&s_pidA);
    pid_reset(&s_pidB);
    Motor_A_Set(0);
    Motor_B_Set(0);
}

void PID_Update(void)
{
    int16_t out_a;
    int16_t out_b;

    /* While disabled the motors are left entirely alone. This is what lets
     * the motion layer brake: it calls PID_Enable(0) and THEN Motors_Brake(),
     * and the next tick does not come along and overwrite the brake with a
     * coast. */
    if (!s_enabled)
    {
        return;
    }

    out_a = pid_step(&s_pidA, (float)Encoder_A_GetRPM());
    out_b = pid_step(&s_pidB, (float)Encoder_B_GetRPM());

    Motor_A_Set(out_a);
    Motor_B_Set(out_b);
}

int16_t PID_GetOutputA(void) { return s_pidA.output; }
int16_t PID_GetOutputB(void) { return s_pidB.output; }

int16_t PID_GetTargetA(void) { return (int16_t)s_pidA.target_rpm; }
int16_t PID_GetTargetB(void) { return (int16_t)s_pidB.target_rpm; }

int16_t PID_GetErrorA(void)
{
    return (int16_t)(s_pidA.target_rpm - (float)Encoder_A_GetRPM());
}

int16_t PID_GetErrorB(void)
{
    return (int16_t)(s_pidB.target_rpm - (float)Encoder_B_GetRPM());
}

void PID_SetGains(float kp, float ki, float kd)
{
    s_pidA.kp = kp;  s_pidA.ki = ki;  s_pidA.kd = kd;
    s_pidB.kp = kp;  s_pidB.ki = ki;  s_pidB.kd = kd;

    /* An integral accumulated under the old Ki means nothing under the new
     * one, and carrying it over is a classic source of "why did it kick when
     * I changed a gain". */
    s_pidA.integral = 0.0f;
    s_pidB.integral = 0.0f;
}

void PID_SetMotorModel(uint8_t wheel_b, int16_t ff_fwd, int16_t ff_rev,
                       int16_t deadband)
{
    PID_t *p = wheel_b ? &s_pidB : &s_pidA;

    p->ff_fwd   = ff_fwd;
    p->ff_rev   = ff_rev;
    p->deadband = deadband;
}
