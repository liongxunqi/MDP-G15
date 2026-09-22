#ifndef __RPILINK_H
#define __RPILINK_H

#include "stm32f4xx_hal.h"
#include "commands.h"

/* ---------------------------------------------------------------------------
 * USART3 link to the Raspberry Pi, and the executor that turns queued
 * commands into motion primitives.
 *
 *   PD8 = TX, PD9 = RX, 115200 8N1, no flow control.
 *
 * PD8/PD9 and NOT PB10/PB11. USART3 is available on both, but PB10/PB11 are
 * the I2C2 bus the IMU sits on. Moving USART3 there takes the IMU off the
 * board and the failure looks like a dead sensor, not a pin clash.
 *
 * RECEIVE PATH
 *   One byte at a time under interrupt. Bytes accumulate into a line buffer
 *   in the ISR; on '\n' the line is copied into a second buffer and a flag is
 *   raised. Nothing is parsed in the ISR - RpiLink_Poll() does that from the
 *   main loop, where a blocking transmit is harmless.
 *
 *   Double-buffering matters: without it a line arriving while the previous
 *   one is still being parsed would overwrite it mid-parse.
 *
 * REPLY RULE
 *   Exactly one reply per LINE, never per token. "OK" goes out only once the
 *   last primitive on the line has finished moving. RST replies nothing at
 *   all - that is deliberate and the RPi side already expects it.
 *
 * TRANSMIT PATH
 *   Blocking, from the main loop. "OK\n" is three bytes, 260 us at 115200.
 *   Doing it under interrupt would buy nothing and cost a state machine.
 * ------------------------------------------------------------------------- */

/* Distance at which F0 gives up and stops, cm. Only has any effect once a
 * real Sensors_ObstacleAhead() replaces the weak stub. */
#define RPILINK_F0_STOP_CM      15U

/* How far F0 will run before the motion watchdog calls it a day, mm. */
#define RPILINK_F0_MAX_MM       20000

/* ---------------------------------------------------------------------------
 * FU{n} - STOP A DISTANCE THE SENDER CHOOSES, MEASURED RATHER THAN TRIPPED
 *
 * F0 is a trip-wire: drive, and the moment the ultrasound reads under 15 cm,
 * brake. That is fine for "do not hit the wall" and useless for "stand off at
 * exactly n cm", because the reading that trips it is OLD.
 *
 * How old is the whole problem. The driver pings every 60 ms and reports the
 * MEDIAN OF THE LAST THREE, so the number the executor sees describes where
 * the robot was up to about 120 ms ago. At MOTION_CRUISE_RPM the robot covers
 * 345 mm/s - wheels are 65.9 mm across, so 100 rpm is 345 mm/s - and 120 ms
 * of that is 41 mm. A trip-wire approach at cruise therefore overshoots by
 * around 4 cm, and by a DIFFERENT 4 cm each run, because how much of the lag
 * you eat depends on where in the ping cycle you happened to arrive.
 *
 * So FU does not trip on the sensor at all. It uses the sensor to MEASURE and
 * the odometry to MOVE:
 *
 *   1. stand still, let the median filter fill with stationary samples,
 *      and read the gap - a stationary reading has no lag to speak of
 *   2. hand (gap - n) to Motion_DriveDistance(), which is the same closed
 *      loop that holds A.3 to +/-6% and already compensates its own coast
 *   3. stop, re-measure, and correct if it is still out
 *
 * Each pass divides the error down, so two passes land inside CMD_FU_TOL_CM
 * from any starting error the sensor can produce. The cost is the settle -
 * roughly 400 ms per pass, paid stationary.
 *
 * The trip-wire is still there underneath as a GUARD, not as the terminator:
 * if the gap closes to n while a pass is still running - the first echo came
 * off something further away, or someone put a hand in front of the robot -
 * the move is cut short and the next pass re-measures. Belt and braces.
 * ------------------------------------------------------------------------- */

/* Stationary settle before a reading is believed, ms.
 *
 * Two things have to finish inside this window. The robot has to physically
 * stop - Motion_Stop() drops to IDLE while the chassis is still coasting, so
 * "not busy" is not "not moving". And the median filter has to flush the
 * samples it took while moving, which needs three fresh pings at 60 ms.
 *
 * 400 ms covers both with margin. Shortening it does not fail loudly; it
 * quietly biases every FU long, because the filter still holds a sample taken
 * further back. If FU reads consistently high, look here first. */
#define RPILINK_FU_SETTLE_MS    400U

/* Give up correcting after this many closing moves.
 *
 * Running out of passes is NOT a failure and does not fail the line. The
 * robot is stopped, near the target, and pointing the right way - the sender
 * can ask ?US for the gap it actually got. It means the reading is jittering
 * by more than CMD_FU_TOL_CM, which is a sensor or surface problem that more
 * driving will not fix. */
#define RPILINK_FU_MAX_PASSES   4U

/* Cap on a single closing move, mm. A gap larger than this is covered in
 * stages, each one re-measured, so a first reading that was off a distant
 * surface cannot commit the robot to a long blind run. */
#define RPILINK_FU_MAX_STEP_MM  1000

/* Cap on a reverse correction, mm.
 *
 * FU is "forward until", so backing up is only ever a small tidy-up after
 * overshooting the target. If the robot finds itself a long way INSIDE n -
 * the obstacle moved, or the approach was started from too close - that is
 * the sender's problem to solve with an explicit R{n}, not something for a
 * forward command to silently undo. */
#define RPILINK_FU_MAX_BACK_MM  200

void RpiLink_Init(UART_HandleTypeDef *huart);

/* Call from the main loop as often as possible. Never blocks for long. */
void RpiLink_Poll(void);

/* Call from HAL_UART_RxCpltCallback() when the instance is USART3. */
void RpiLink_RxCallback(void);

/* ---------------------------------------------------------------------------
 * Plain text out on the same port, for telemetry and bring-up logging.
 * Main loop only.
 *
 * THIS GOES SILENT ONCE THE RPi HAS SPOKEN, and that is deliberate.
 *
 * USART3 is both the bench console and the command link - there is only one
 * port and the protocol owns it. While no host is connected the reports are
 * the most useful diagnostic on the robot, so they run freely. The moment a
 * valid command line parses off the wire, a real host is driving and anything
 * else on this port lands in the middle of the OK/RESEND stream it is parsing.
 * So the first parsed line latches the console off for good.
 *
 * Consequence worth knowing on the bench: connect the RPi and your terminal
 * output stops. That is not a fault. Reset the board to get it back, with the
 * host quiet.
 *
 * Protocol replies do NOT go through here - they use an internal path that is
 * never gated, because a suppressed OK hangs the host forever. */
void RpiLink_Log(const char *s);

/* 1 once the console has latched off. */
uint8_t RpiLink_IsQuiet(void);

/* Times reception had to be re-armed after the HAL tore it down on a line
 * error. Should be 0. Anything else means the link is glitching and being
 * silently recovered - go and look at the wiring before it bites in a run. */
uint32_t RpiLink_GetRearmCount(void);

/* 1 while a line is being executed. For the OLED. */
uint8_t RpiLink_IsBusy(void);

/* Last command the executor started. For the OLED. */
CmdOpcode_t RpiLink_LastOpcode(void);

#endif /* __RPILINK_H */
