#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Closed-loop wheel speed control for the WHEELTEC C30D-V2.
 *
 * One PI(D) controller per wheel. Setpoint and feedback are both in RPM at
 * the wheel; the output is a duty value in the same units Motor_x_Set() takes
 * (-1000..+1000).
 *
 * PID_Update() must be called from the TIM6 interrupt, immediately after
 * Encoders_Update(), so the controller always acts on fresh feedback and the
 * sample period is exactly PID_DT_MS.
 * ------------------------------------------------------------------------- */

/* Must match the TIM6 period. Change both together or the integral and
 * derivative terms are scaled wrong. */
#define PID_DT_MS       10U
#define PID_DT_S        (0.010f)

/* Starting gains. Tune Kp first with Ki and Kd at zero, then add Ki.
 * Units: Kp is duty counts per RPM of error. Full-scale error is ~360 RPM
 * and the usable output span is 400 counts, so Kp near 1.0 is a sane start. */
#define PID_KP_DEFAULT  1.0f
#define PID_KI_DEFAULT  2.0f
#define PID_KD_DEFAULT  0.0f

/* Ceiling on the integral term, in duty counts. Stops windup from parking a
 * huge accumulated value that takes seconds to unwind after a disturbance. */
#define PID_I_LIMIT     300.0f

/* Fast unwind. When a wheel runs more than PID_UNWIND_FRAC faster than asked
 * while the integral is still pushing it ON, the integral is multiplied by
 * PID_UNWIND_KEEP every tick instead of bleeding off at Ki.
 *
 * Why: in a turn the inner wheel is dragged against the rear scrub. If it
 * bogs down, the integral climbs to the limit; when the wheel breaks free
 * that 300 counts of extra duty is still there, and at Ki 2 it took ~0.75 s
 * of overspeed to bleed away - the robot visibly lurched mid-turn. At 0.5 per
 * tick it is gone in about 50 ms. The threshold keeps it out of normal
 * regulation, where measured speed sits within a few rpm of the target. */
#define PID_UNWIND_FRAC  0.10f
#define PID_UNWIND_KEEP  0.5f

/* Below this setpoint the wheel is commanded to a hard stop rather than
 * creeping. MOTOR_DEADBAND duty produces roughly 32 RPM, so anything under
 * that is not reachable and asking for it only winds up the integrator. */
#define PID_MIN_RPM     25

/* ---------------------------------------------------------------------------
 * PID_LOW_DUTY  (A/B switch, 2026-10 calibration fixes)
 *
 * 1  MOTOR_DEADBAND is what it takes to START a wheel from rest (static
 *    friction, measured wheels-free). A wheel that is already turning keeps
 *    turning on less. So:
 *      - from rest (|measured| < PID_MOVING_RPM) a small output is still
 *        pushed up to the deadband, to break it away, as before;
 *      - once turning, the output is passed through as it is, and only one
 *        under PID_ZERO_DUTY is dropped to zero - a duty that small is the
 *        loop passing through zero, not a speed;
 *      - the feedforward line starts PID_RUN_DB_DROP below the deadband
 *        instead of at it. The bench figure "deadband duty gives ~32 rpm"
 *        puts the running line's zero about 37 counts under it;
 *      - the hard-stop setpoint is PID_MIN_RPM_LOW instead of PID_MIN_RPM.
 *    This is what lets an arc's inner wheel be regulated at ~20 rpm, which
 *    MOTION_ARC_TAPER depends on.
 * 0  every non-zero output snaps up to the deadband, PID_MIN_RPM 25, as
 *    before.
 *
 * Re-check PID_RUN_DB_DROP from the move report (?MV duty against rpm at
 * a low cruise) once on the floor: too large and slow wheels lag their
 * setpoint at launch, too small and they overspeed. */
#define PID_LOW_DUTY        1
#define PID_MOVING_RPM      8
#define PID_ZERO_DUTY       150
#define PID_RUN_DB_DROP     40
#define PID_MIN_RPM_LOW     10

typedef struct
{
    float   kp;
    float   ki;
    float   kd;

    float   target_rpm;     /* what we are asking for        */
    float   integral;       /* accumulated error, duty counts */
    float   last_error;     /* for the derivative term        */
    int16_t ff_fwd;         /* protocol 10: rpm at full duty, forward, 0 = max_rpm */
    int16_t ff_rev;         /*              ... and reverse                         */
    int16_t deadband;       /*              duty to start turning, 0 = MOTOR_DEADBAND */

    int16_t output;         /* last duty written to the motor */
    int16_t max_rpm;        /* free-running speed, this wheel */
    uint8_t enabled;
} PID_t;

/* Zeroes both controllers and loads the default gains. Call once at startup,
 * after Motors_Init() and Encoders_Init(). */
void PID_Init(void);

/* Enable/disable closed-loop control. While disabled the motors are left
 * alone, so open-loop Motor_x_Set() calls still work for testing. */
void PID_Enable(uint8_t on);

/* 1 while the closed loop is driving the motors. */
uint8_t PID_IsEnabled(void);

/* Setpoints in wheel RPM. Negative runs the wheel in reverse. Values beyond
 * that wheel's measured maximum are clamped. */
void PID_SetTargetA(int16_t rpm);
void PID_SetTargetB(int16_t rpm);
void PID_SetTargets(int16_t rpm_a, int16_t rpm_b);

/* Stops both wheels and clears the integrators. Use this rather than
 * setting the targets to zero if you want an immediate stop. */
void PID_Stop(void);

/* One control step. Call from the TIM6 ISR, after Encoders_Update(). */
void PID_Update(void);

/* Introspection, for putting numbers on the OLED while tuning. */
int16_t PID_GetOutputA(void);
int16_t PID_GetOutputB(void);
int16_t PID_GetTargetA(void);
int16_t PID_GetTargetB(void);
int16_t PID_GetErrorA(void);
int16_t PID_GetErrorB(void);

/* Change gains at runtime. Applies to both wheels. Clears the integrators,
 * since an accumulated value scaled by the old Ki is meaningless. */
void PID_SetGains(float kp, float ki, float kd);

/* Protocol 10: one wheel's measured motor line (calibration table globals
 * ff*, db* - caltab.h). ff_fwd / ff_rev are the rpm the wheel would reach at
 * full duty driving forward / reverse; deadband the duty it needs to start.
 * 0 for any of them = the compiled constant. Takes effect on the next tick. */
void PID_SetMotorModel(uint8_t wheel_b, int16_t ff_fwd, int16_t ff_rev,
                       int16_t deadband);

#endif /* __PID_H */
