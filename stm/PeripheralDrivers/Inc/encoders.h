#ifndef __ENCODERS_H
#define __ENCODERS_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Board: WHEELTEC STM32F407VET6 C30D-V2 (schematic rev 23.0)
 *
 *   Encoder A (header MOTORA) -> PA15 TIM2_CH1, PB3 TIM2_CH2  (AF1)
 *   Encoder B (header MOTORB) -> PB4  TIM3_CH1, PB5 TIM3_CH2  (AF2)
 *
 * Encoder supply on the motor headers is 3V3, and R56-R59 are 100R series
 * resistors on the signal lines, so no level shifting is needed.
 *
 * *** PA15 and PB3 are JTDI and JTDO. ***
 * Encoder A will not work unless CubeMX has SYS -> Debug -> Serial Wire.
 * The board's debug header is SWD (PA13/PA14), so nothing is lost, but
 * leaving Debug set to JTAG or No Debug produces an encoder that silently
 * never counts.
 *
 * CubeMX for TIM2 and TIM3, identically:
 *   Combined Channels = Encoder Mode
 *   Encoder Mode = TI1 and TI2
 *   Prescaler 0, Counter Period 65535
 *   Input Filter 10 on both channels
 *
 * "TI1 and TI2" counts every edge on both channels, so one encoder pulse
 * gives 4 counts. That is already folded into COUNTS_PER_REV below.
 *
 * The timers count in hardware, so a slow main loop costs speed resolution
 * but never loses position.
 * ---------------------------------------------------------------------------
 * TIMING CONTRACT  (changed - read this)
 *
 * Encoders_Update() is now called from the TIM6 control tick at a FIXED
 * ENCODER_TICK_MS interval, NOT from the main loop. It no longer rate-limits
 * itself and no longer calls HAL_GetTick().
 *
 * Why: the old version rate-limited on HAL_GetTick() at 20 ms while the PID
 * runs at 10 ms, so every second control step acted on a stale measurement
 * and Odom_Update() saw a wheel delta that alternated between 0 and double.
 * It also read HAL_GetTick() from an interrupt that pre-empts SysTick, so
 * the measured interval jittered between 9 and 11 ms and scaled every RPM
 * reading by up to +-10%.
 *
 * Call it FIRST in the tick, exactly once per tick, and nowhere else.
 * ------------------------------------------------------------------------- */

/* ---- CALIBRATE THESE FOR YOUR MOTORS --------------------------------------
 * ENCODER_PPR is pulses per revolution of the MOTOR shaft on ONE channel,
 * before the gearbox and before the x4 quadrature multiplication.
 * 13 * 4 * 30 = 1560, which is the ten-revolution measured figure.
 *
 * Easiest measurement: mark the wheel, call Encoders_Reset(), turn the wheel
 * exactly 10 revolutions by hand, read Encoder_A_GetCount(), divide by 10.
 * That result is COUNTS_PER_REV directly.
 *
 * *** DO NOT CHANGE 13 TO 11 ON THE STRENGTH OF THE KIT'S DATASHEET PDF. ***
 *
 * DCMotor_Encoder_ServoMotor_DataSheets_v2.pdf describes a JGB37-520 with an
 * 11 PPR encoder. That is not this motor. The issued part is an MG513P3012V
 * (see the component list), and 13 PPR is what the ten-revolution test gave.
 * The measured free speed agrees: 378 RPM at 1560 counts/rev is sane for a
 * 12 V 1:30 gearmotor on a 12.6 V pack, where 11 PPR would imply 447.
 *
 * 11 would give 1320 counts/rev and the robot would stop 15% short of every
 * commanded distance - outside A.3's +/-6%. Last year's firmware shipped with
 * exactly that value, taken from exactly that PDF.
 * ------------------------------------------------------------------------- */
#define ENCODER_PPR             13U
#define ENCODER_GEAR_RATIO      30U
#define ENCODER_COUNTS_PER_REV  (ENCODER_PPR * 4U * ENCODER_GEAR_RATIO)

/* If a wheel counts down while its motor is commanded forward, set the
 * matching flag to 1. */
#define ENCODER_A_INVERT        0
#define ENCODER_B_INVERT        1

/* Period at which Encoders_Update() is called. MUST equal the TIM6 tick and
 * PID_DT_MS. Position accumulates every tick. */
#define ENCODER_TICK_MS         10U

/* RPM is averaged over this many ticks.
 *
 * One count at a 10 ms window is 60000/(1560*10) = 3.85 RPM, which is far
 * too coarse to feed a loop that is meant to hold +-1 RPM - the feedback
 * would quantise into 4 RPM steps and the integrator would chase the
 * rounding. Over 4 ticks (40 ms) one count is 0.96 RPM instead.
 *
 * Position is still integrated every single tick, so odometry loses nothing.
 * The only cost is 40 ms of lag in the speed feedback, which at these motor
 * time constants is well inside the loop bandwidth. */
#define ENCODER_RPM_WINDOW      4U

void Encoders_Init(void);

/* One fixed-period sample. Call from the TIM6 ISR, first, once per tick. */
void Encoders_Update(void);

/* Zeroes accumulated position. Does not disturb speed readings. */
void Encoders_Reset(void);

int32_t Encoder_A_GetCount(void);
int32_t Encoder_B_GetCount(void);

/* Counts in the most recent single tick. */
int16_t Encoder_A_GetDelta(void);
int16_t Encoder_B_GetDelta(void);

/* Output-shaft speed in whole RPM, signed, averaged over ENCODER_RPM_WINDOW
 * ticks. Integer maths throughout, so it is safe with a non-float printf. */
int32_t Encoder_A_GetRPM(void);
int32_t Encoder_B_GetRPM(void);

#endif /* __ENCODERS_H */
