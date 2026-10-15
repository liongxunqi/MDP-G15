#ifndef __TASK2FW_H
#define __TASK2FW_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * TASK 2 ON THE STM, protocol 11
 *
 * The robot side of the Task 2 course in task2_mission.c: every Task2Ops
 * function, built on the same Motion / Odom / IR / ultrasonic calls the wire
 * commands use. So every straight and arc of the course runs on the
 * calibration table the RPi restored (protocol 10) exactly like F / FR / FL
 * sent over the wire, and the course plans its turns on the table's radii.
 *
 * Wire (PROTOCOL.md section 14):
 *
 *   RPi -> STM  !t2go<s>     run the course; s = mission seconds, 1..180.
 *                            Alone on its line. One reply when it ends:
 *                            OK parked, or FAIL,<why>.
 *   STM -> RPi  P,<n>        photograph obstacle n (1 or 2). Repeated every
 *                            T2_PROMPT_REPEAT_MS until answered.
 *   RPi -> STM  !t2a<n><d>   obstacle n's arrow, d 0 left / 1 right. No reply.
 *   RPi -> STM  !t2y<n><ddd> where that arrow sat sideways in the photo,
 *                            ddd - 500 mm from the camera's centre line, left
 *                            positive. Optional, sent before its !t2a. No reply.
 *   STM -> RPi  D,<step>,... the run log, one line per decision.
 *   RPi -> STM  rst          abort: brake, and the !t2go line replies
 *                            FAIL,STOP.
 *
 * Queries (?US, ?WPOSE, ...) are still answered during the run. Every other
 * line is RESEND: nothing but the course moves the robot while it runs.
 *
 * Task2Fw_Run() BLOCKS for the whole course. RpiLink_Poll() calls it from the
 * main loop, and while it runs it keeps the main loop's two jobs going itself:
 * IMU_Poll() and the link (RpiLink_Service()). The control loop is in the
 * TIM6 interrupt and never notices.
 * ------------------------------------------------------------------------- */

/* Run the course. Returns the reply line for the !t2go line. */
const char *Task2Fw_Run(uint16_t seconds);

/* 1 while Task2Fw_Run() is in progress. */
uint8_t Task2Fw_Running(void);

/* From the link during a run. obstacle 1 or 2. */
void Task2Fw_Answer(uint8_t obstacle, char arrow);      /* 'L' or 'R' */
void Task2Fw_Lateral(uint8_t obstacle, int16_t mm);     /* left positive */
void Task2Fw_Abort(void);                               /* rst */

/* Board-level abort (the user button), checked every pump. Weak: returns 0.
 * main.c overrides it. */
uint8_t Task2Fw_BoardAbort(void);

#endif /* __TASK2FW_H */
