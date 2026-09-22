#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * HC-SR04 front distance sensor.
 *
 *   Trigger  PB14, plain GPIO output, header J6 (3-pin, 5V5 + GND)
 *   Echo     PC7  -> TIM8_CH2 input capture, AF3, header J2
 *
 * PC7 is a 5 V tolerant pin, so the module's 5 V echo will not damage it.
 * The 1k / 2.2k resistors in the kit are the recommended divider for that
 * line anyway - fit them. Either way PC7 is configured with a PULL-DOWN, so
 * an unplugged sensor reads as a permanent no-echo rather than picking up
 * noise and inventing distances.
 *
 * WHY INPUT CAPTURE AND NOT A BUSY-WAIT
 * The obvious HC-SR04 driver polls the echo pin in a loop and times it with
 * HAL_GetTick() or a delay. At 4 m the echo is 25 ms long - that is two and a
 * half control ticks spent inside a spin loop with the motors under closed
 * loop control. It stalls the PID, wrecks the odometry timing and shows up as
 * the robot lurching every time it pings. Capture the edges in hardware and
 * return immediately instead.
 *
 * TIMER SETUP (TIM8, APB2, 168 MHz timer clock)
 *   PSC 167 -> 1 MHz, so one count is exactly 1 us
 *   ARR 65535 -> 65.5 ms wrap, comfortably longer than the longest echo
 *   CH2, direct TI2, BOTH edges, so one channel sees the rise and the fall
 *
 * TIM8_CC runs at NVIC priority 3, which pre-empts the 100 Hz control tick at
 * priority 6. That is deliberate: a capture delayed by the control tick would
 * add up to 10 ms of error, and 10 ms of flight time is 1.7 m.
 * ------------------------------------------------------------------------- */

/* Ping period, in 10 ms control ticks.
 *
 * The datasheet asks for at least 60 ms between triggers. Ping faster and the
 * tail of the previous burst is still bouncing around the room when the next
 * one goes out, which reads back as a randomly short distance. 60 ms it is. */
#define US_PERIOD_TICKS         6U

/* Give up waiting for the falling edge after this many ticks and re-arm.
 * Without it a single missed edge leaves the driver waiting forever. */
#define US_ECHO_TIMEOUT_TICKS   5U      /* 50 ms */

/* A reading older than this is reported as SENSOR_NO_READING rather than
 * being served up stale. Nothing downstream should act on a distance from
 * half a second ago. */
#define US_STALE_TICKS          30U     /* 300 ms */

/* Usable span, cm. Below the minimum the module cannot distinguish the
 * outgoing burst from the echo; above the maximum the echo is lost in noise. */
#define US_MIN_CM               2U
#define US_MAX_CM               400U

/* Microseconds of echo per cm of distance, round trip.
 * Sound travels ~343 m/s at 20 C, so 1 cm out and back is 2/34300 s = 58.3 us.
 * This is temperature dependent - about 0.17% per degree C - which is well
 * inside the sensor's own error, so a constant is fine. */
#define US_US_PER_CM            58.3f

void Ultrasonic_Init(TIM_HandleTypeDef *htim);

/* Call once per control tick from the TIM6 ISR. Fires the trigger when due
 * and ages out stale readings. Never blocks for more than ~10 us. */
void Ultrasonic_Tick(void);

/* Call from HAL_TIM_IC_CaptureCallback() when the instance is TIM8. */
void Ultrasonic_CaptureCallback(TIM_HandleTypeDef *htim);

/* Latest median-filtered distance in cm, or SENSOR_NO_READING. */
uint16_t Ultrasonic_GetCm(void);

/* Raw last echo width in microseconds, and a count of completed echoes.
 * Both exist for bring-up: if the count is not rising, the trigger or the
 * echo wiring is the problem, not the maths. */
uint16_t Ultrasonic_GetLastUs(void);
uint32_t Ultrasonic_GetEchoCount(void);

#endif /* __ULTRASONIC_H */
