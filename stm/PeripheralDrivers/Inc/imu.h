#ifndef __IMU_H
#define __IMU_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * ICM-20948 gyroscope, for heading.
 *
 *   U18 on the C30D schematic. I2C2 on PB10 (SCL) and PB11 (SDA), AF4.
 *
 * THREE THINGS ABOUT THIS PART ON THIS BOARD
 *
 * 1. It runs at 1.8 V, not 3.3 V. U20 (SSP6206-18NR) makes the 1.8 V rail and
 *    U19 (RS0102YH8) level-shifts SCL and SDA. So the pull-ups you can see
 *    next to the chip are on the far side of the shifter and are not yours.
 *    From PB10/PB11 it looks like an ordinary 3.3 V I2C bus.
 *
 * 2. PB12 must be driven HIGH. It is wired into the same block, and on the
 *    ICM-20948 pin 22 is nCS: low selects SPI, high selects I2C. Left low or
 *    floating the part simply never acknowledges, and it looks exactly like a
 *    dead sensor or a wrong address. IMU_Init() drives it.
 *
 * 3. The 7-bit address is 0x68 or 0x69 depending on the AD0 pin. IMU_Init()
 *    probes both rather than assuming, and IMU_GetAddress() reports which one
 *    answered.
 *
 * ---------------------------------------------------------------------------
 * WHY ONLY THE GYRO
 *
 * Yaw rate is the one quantity we need. The accelerometer cannot see yaw at
 * all when the robot is level, and the magnetometer needs hard-iron
 * calibration and is useless next to two motors pulling amps.
 *
 * The usual objection to a bare gyro is drift, and it does not really apply
 * here. An A.3 run lasts about three seconds. Bias-corrected, residual drift
 * over three seconds is a fraction of a degree. Gyros are poor at holding a
 * heading for minutes and excellent for seconds - and seconds is all we need,
 * because Motion_DriveDistance() zeroes the heading at the start of every
 * move anyway.
 *
 * ---------------------------------------------------------------------------
 * SPLIT BETWEEN MAIN LOOP AND TICK
 *
 * IMU_Poll()  main loop. Does the I2C transaction.
 * IMU_Tick()  control tick. Integrates the most recent sample at fixed dt.
 *
 * The read is deliberately NOT in the tick. A two-byte transfer at 400 kHz is
 * about 90 us, which is tolerable, but a bus glitch turns it into a HAL
 * timeout that stalls the control loop for milliseconds. Same reasoning as
 * the ultrasonic trigger.
 *
 * The gyro's output data rate is set to ~102 Hz, near enough to the 100 Hz
 * tick that integrating the latest sample each tick loses nothing.
 * ------------------------------------------------------------------------- */

/* Full scale, and the matching sensitivity.
 *
 * CHANGED FROM +-250 TO +-1000 dps. 250 was clipping.
 *
 * The robot only turns at about 30 dps, so 250 looked like enormous headroom
 * and the most sensitive setting looked like the obvious choice. It was
 * wrong, twice over:
 *
 *   By hand   a "fast" 180 degree rotation averages 180 dps, but the motion
 *             is not smooth - the peak mid-rotation is two or three times the
 *             average and goes straight past 250. Everything above the limit
 *             is discarded, so the integral comes out short.
 *
 *   Driving   the gearboxes shake the chassis. Vibration swinging several
 *             hundred dps around a +30 dps mean clips HARDER ON THE POSITIVE
 *             SIDE than the negative, because the mean is offset. That
 *             rectifies the signal and drags the average down - which is why
 *             a commanded 90 degree turn read 87 while the robot physically
 *             turned about 156.
 *
 * The low-pass filter cannot fix this: saturation happens at the sensor,
 * before the filter sees the signal. Only more range helps.
 *
 * Resolution is not a real cost. At +-1000 dps one count is 0.03 dps against
 * a 30 dps signal - a tenth of a percent, far below the noise floor.
 *
 * CONFIRMED ON THE FLOOR. IMU_GetPeakRaw() during normal TIGHT turns reads
 * 4000-5000, about 137 dps, against a steady turn rate near 102 - so the
 * vibration overhead is about 1.4x the mean and the peak sits at 14% of full
 * scale. The same signal at +-250 dps would sit at 55%, where a spike rails
 * it. The headroom is doing real work.
 *
 * Note what that check can and cannot prove: the peak is read from the
 * DLPF-filtered output, while saturation happens upstream at the ADC. A high
 * peak is proof of clipping; a low one is strong evidence against it, not
 * proof. With 900 dps of headroom above the mean, clipping is implausible.
 *
 *   FS_SEL  00 = +-250 dps,  131.0 LSB/dps
 *           01 = +-500 dps,   65.5
 *           10 = +-1000 dps,  32.8   <- this one
 *           11 = +-2000 dps,  16.4 */
#define IMU_GYRO_FS_SEL         2U
#define IMU_GYRO_FS_DPS         1000.0f
#define IMU_GYRO_LSB_PER_DPS    32.8f

/* Gyro low-pass bandwidth, GYRO_DLPFCFG.
 *
 *   0 = 196.6 Hz   3 = 51.2 Hz   6 = 5.7 Hz
 *   1 = 151.8 Hz   4 = 23.9 Hz
 *   2 = 119.5 Hz   5 = 11.6 Hz
 *
 * Was 0, which is 196.6 Hz against an output data rate of 102 Hz - the filter
 * sat ABOVE Nyquist, so vibration between 51 and 196 Hz folded down into the
 * measurement instead of being removed. 4 gives 23.9 Hz: comfortably below
 * Nyquist, and still ten times faster than anything the chassis does. */
#define IMU_GYRO_DLPFCFG        4U

#define IMU_DT_S                0.01f

/* Mounting sign. Odometry counts heading counter-clockwise positive. If the
 * board is mounted with the chip's Z axis pointing UP, the gyro agrees and
 * this is +1. Mounted upside down, use -1.
 *
 * Check it: with the robot flat, rotate it counter-clockwise (to the left)
 * seen from above. IMU_GetHeading() must INCREASE. */
#define IMU_Z_SIGN              (+1)

/* Samples averaged for the bias measurement, at ~10 ms each. 200 is two
 * seconds. The robot MUST be still and on the ground for this. */
#define IMU_BIAS_SAMPLES        200U

/* Bias larger than this in raw counts means the robot was moving during
 * calibration, or the part is faulty. At 32.8 LSB/dps, 500 counts is 15 dps -
 * already far more than a healthy part should show.
 *
 * This was 2000, carried over from the +-250 dps setting where it meant
 * 15 dps. At +-1000 the same number means 61 dps, so the check had quietly
 * stopped catching anything. */
#define IMU_BIAS_SANITY_LSB      500

/* Bring-up attempts before giving up, 100 ms apart. Covers a slow 1.8 V rail
 * and a warm MCU reset that left the sensor mid-reset. */
#define IMU_INIT_RETRIES        5U

/* Go stale if no successful read arrives for this long, ms.
 *
 * IMU_Poll() returns early on a failed read, which leaves the last rate in
 * place - and IMU_Tick() goes on integrating it. If the bus drops mid-turn
 * with the robot rotating at 100 deg/s, the heading climbs at 100 deg/s
 * forever on a reading that is no longer being taken. Nothing detects it,
 * because readiness was decided once at startup.
 *
 * The sensor updates at 102 Hz and the main loop polls far faster, so a
 * 100 ms gap is already dozens of missed reads - comfortably a fault, not
 * jitter. Going stale zeroes the rate, clears ready, and lets odom fall back
 * to the encoders, which is degraded but bounded. */
#define IMU_STALE_MS            100U

/* Start-up. Call after MX_I2C2_Init(), BEFORE the TIM6 tick is started -
 * it uses HAL_Delay() and takes about 2.5 seconds, most of it bias
 * calibration. Returns 1 on success. */
uint8_t IMU_Init(I2C_HandleTypeDef *hi2c);

/* Re-measure the zero-rate bias. Robot must be stationary. Blocking, ~2 s. */
uint8_t IMU_CalibrateBias(void);

/* Main loop: one I2C read of the Z gyro. Call as often as you like. */
void IMU_Poll(void);

/* Control tick: integrate the last sample. Call once per tick, from TIM6. */
void IMU_Tick(void);

/* Zero the integrated heading. Call whenever a move starts. */
void IMU_ResetHeading(void);

/* ---------------------------------------------------------------------------
 * Zero-rate update. Call from the control tick ONLY when the robot is known
 * to be stationary - motors idle and both encoders showing no movement.
 *
 * A stationary gyro's true yaw rate is exactly zero, so whatever it reads is
 * bias, and that is a free measurement available any time the robot is
 * standing still. Slowly pulling the stored bias toward it tracks the drift
 * out continuously.
 *
 * This matters more than it did before the full scale went to +-1000 dps. One
 * raw count is now 0.0305 dps instead of 0.0076, so the same handful of counts
 * of residual bias integrates four times faster into the heading. And a
 * one-shot calibration at power-on cannot hold anyway: MEMS bias moves with
 * temperature, and boot is when the chip is coldest.
 *
 * The filter is deliberately slow - about a ten second time constant - so a
 * genuine slow rotation cannot be mistaken for bias and calibrated away. */
void IMU_TrackBias(void);

/* Integrated heading in degrees, counter-clockwise positive. Free-running,
 * does NOT wrap - wrap it at the point of use if you need +-180. */
float IMU_GetHeading(void);

/* Current yaw rate, degrees per second, bias removed. */
float IMU_GetRateDps(void);

/* 1 once the part has answered and been configured. */
uint8_t IMU_IsReady(void);

/* Diagnostics for the OLED. */
uint8_t  IMU_GetAddress(void);      /* 0x68, 0x69, or 0 if nothing answered */
uint8_t  IMU_GetWhoAmI(void);       /* should be 0xEA                       */
int16_t  IMU_GetBias(void);         /* raw counts subtracted from every read */
int16_t  IMU_GetRawZ(void);         /* last raw reading, before bias         */
uint32_t IMU_GetErrorCount(void);   /* failed I2C transactions since boot    */
uint32_t IMU_GetPollRate(void);     /* successful reads in the last second    */
uint32_t IMU_GetStallCount(void);   /* times the gyro went stale mid-run     */
int16_t  IMU_GetPeakRaw(void);      /* largest raw magnitude since reset      */
void     IMU_ResetStats(void);      /* zero the poll rate and peak            */

#endif /* __IMU_H */
