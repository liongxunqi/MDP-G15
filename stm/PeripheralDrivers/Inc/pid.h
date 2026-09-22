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

/* Below this setpoint the wheel is commanded to a hard stop rather than
 * creeping. MOTOR_DEADBAND duty produces roughly 32 RPM, so anything under
 * that is not reachable and asking for it only winds up the integrator. */
#define PID_MIN_RPM     25

typedef struct
{
    float   kp;
    float   ki;
    float   kd;

    float   target_rpm;     /* what we are asking for        */
    float   integral;       /* accumulated error, duty counts */
    float   last_error;     /* for the derivative term        */

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

#endif /* __PID_H */
