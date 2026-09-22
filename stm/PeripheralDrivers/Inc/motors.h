#ifndef __MOTORS_H
#define __MOTORS_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Board: WHEELTEC STM32F407VET6 C30D-V2 (schematic rev 23.0)
 *
 * Ackermann chassis: the two rear wheels drive, the front axle is steered by
 * a single servo. The robot CANNOT turn on the spot - every turn is an arc.
 *
 * Four onboard AT8236 H-bridges. This driver uses the first two.
 *
 *   Motor A  = U8,  header MOTORA   (left rear)
 *       drive   PB8, PB9   -> TIM4_CH3, TIM4_CH4   (AF2)
 *   Motor B  = U9,  header MOTORB   (right rear)
 *       drive   PE5, PE6   -> TIM9_CH1, TIM9_CH2   (AF3)
 *
 *   Steering servo, header per Robot car test V30D:
 *       PB15       -> TIM12_CH2                    (AF9)
 *       Powered from the 5V5 rail (U7), separate from the Pi 5V supply.
 *
 * The AT8236 has no direction pin. Direction comes from which of the two
 * inputs carries the PWM:
 *      IN1=PWM IN2=0   one way
 *      IN1=0   IN2=PWM the other way
 *      IN1=0   IN2=0   coast
 *      IN1=1   IN2=1   brake
 *
 * Which physical direction is "forward" depends on how the motor leads are
 * crimped, so check on the bench and flip the INVERT flags below if needed.
 *
 * ---------------------------------------------------------------------------
 * PINS DELIBERATELY LEFT FREE - do not use:
 *
 *   PB14       HC-SR04 trigger, plain GPIO out, header J6
 *   PC7        HC-SR04 echo, TIM8_CH2 capture, header J2
 *   PB10, PB11 I2C2, ICM20948 IMU                       - Person B
 *
 * Note PB10/PB11 are also USART3_TX/RX on AF7. USART3 must stay on PD8/PD9
 * or it takes the IMU bus, and the failure looks nothing like a pin clash.
 *
 * PB14 IS NOT AVAILABLE FOR A SECOND SERVO. It is TIM12_CH1 electrically, and
 * J6 is a 3-pin servo-style header with 5V5 and GND on it, which is exactly
 * why it suits the HC-SR04 - the module gets its 5 V from the same connector.
 * But the trigger owns that pin. There is ONE servo on this robot, on PB15.
 *
 * PC6, PC8 and PC9 are also free servo-style headers (J1, J4, J5) if a second
 * servo is ever needed. PC7 is not - the echo capture has it.
 *
 * ---------------------------------------------------------------------------
 * NOT USED HERE, but noted so you do not trip over it later:
 *
 * Motor A's drive sits on TIM4_CH3/CH4, and Motor C's ENCODER sits on
 * PB6/PB7 = TIM4_CH1/CH2. Encoder mode claims the whole timer, so you cannot
 * have both. If you add Motor C, move Motor A's drive to TIM10_CH1 (PB8) and
 * TIM11_CH1 (PB9), both AF3, which frees TIM4 for the encoder. Splitting the
 * two inputs across separate timers is harmless - only one is ever driven at
 * a time.
 *
 * ---------------------------------------------------------------------------
 * CLOCK ASSUMPTION
 *
 * Values below assume HSE 8 MHz crystal -> PLL -> 168 MHz SYSCLK,
 * APB1 prescaler 4 (TIM4/TIM12 clock 84 MHz), APB2 prescaler 2 (TIM9 168 MHz).
 *
 *   TIM4:  PSC = 0,   ARR = 4199   -> 84 MHz / 4200  = 20 kHz
 *   TIM9:  PSC = 1,   ARR = 4199   -> 84 MHz / 4200  = 20 kHz  (same scale)
 *   TIM12: PSC = 83,  ARR = 19999  ->  1 MHz / 20000 = 50 Hz, 1 us resolution
 *
 * TIM12 is a general-purpose timer, 16-bit, two channels, no complementary
 * outputs - so unlike TIM8 there is no MOE to enable before PWM appears.
 *
 * If you stay on the default HSI 16 MHz with no PLL, use instead:
 *   TIM4 PSC 0 ARR 799, TIM9 PSC 0 ARR 799, TIM12 PSC 15 ARR 19999,
 * and change MOTOR_TIM_ARR below to 799.
 * ------------------------------------------------------------------------- */

/* Must match the Counter Period set for BOTH TIM4 and TIM9 in CubeMX. */
#define MOTOR_TIM_ARR       4199U

/* Public speed scale. Motor_x_Set() takes -1000 .. +1000. */
#define MOTOR_SPEED_MAX     1000

/* Measured on the bench, 12 V, wheels free.
 * Deadband: lowest duty where both wheels start from rest (575 observed,
 * rounded up for margin). Max RPM: free-running speed at duty 1000. */
#define MOTOR_DEADBAND   600
#define MOTOR_A_MAX_RPM  378
#define MOTOR_B_MAX_RPM  362

/* Set to 1 if a motor spins the wrong way for a positive speed.
 *
 * BOTH ARE 1. VERIFIED ON THE STAND - do not "correct" these from reading
 * the code.
 *
 * B was briefly changed to 0 on the theory that the Phase 2 calibration
 * build in main.c, which had MOTOR_B_INVERT 0, was the bench-verified one.
 * It was not. That build's comment said "Determine in M2" - it was still a
 * placeholder. The real measurement was in the bring-up report all along:
 *
 *     MOTOR_A_INVERT  1   Positive command drove backwards
 *     MOTOR_B_INVERT  1   Positive command drove backwards
 *
 * Flashing the 0 made wheel B run backwards on the stand, which confirms
 * the report. A measured value beats a newer file every time.
 *
 * For reference, the mechanism: motor_drive() puts PWM on in1_ch for a
 * positive speed. Motor_B_Set passes CH1 as in1, so INVERT 0 drives PE5 and
 * INVERT 1 drives PE6. PE6 is forward on this chassis. */
#define MOTOR_A_INVERT      1
#define MOTOR_B_INVERT      1

/* Servo pulse limits, microseconds.
 * The linkage - not the servo - sets the usable range. Per the datasheet
 * MEASURED with the sweep, replacing the Phase 2 guesses of 1250/1750.
 *
 * Confirmed safe travel is 850 to 2125, with straight-ahead at 1500. The
 * midpoint of that is 1487, so the linkage IS symmetric about centre - an
 * earlier partial sweep that stopped at 1100 made it look badly offset and it
 * is not.
 *
 * Limits are SYMMETRIC about 1500 on purpose: 900 and 2100, 600 us either
 * side. Servo_SetMicroseconds() clamps to them, so an asymmetric pair would
 * silently shorten an arc in one direction only and turns would come out
 * lopsided with nothing on screen to say why. */
#define SERVO_MIN_US         900U
#define SERVO_MAX_US        2100U
/* Straight-ahead, MEASURED - this is NOT the mechanical midpoint of
 * SERVO_MIN_US..SERVO_MAX_US, and it is not supposed to be. The linkage
 * decides where the wheels point; the servo's travel limits do not.
 *
 * Seven 1000 mm A.3 runs, tape-measured lateral offset against the steering
 * trim that was actually in force (the trim on screen is learned at the END
 * of a run, so it applies to the NEXT one):
 *
 *     trim in force   0    -7   -16   -22   -25   -35   -34  us
 *     offset        +30   +30   +15   +10     0   -10   -15  mm, right +
 *
 * Fits lateral = 1.335*trim + 35.1, r = 0.98, so zero offset sits at
 * -26 us and 1500 - 26 = 1474. Sensitivity is 7.5 us per cm over a metre,
 * which is what to scale by if you re-measure.
 *
 * What this fixes is the COLD run: trim starts at zero every power-on, so
 * before this the first run of a session went 3 cm right. That is the run
 * A.3 and A.4 are graded on.
 *
 * The -26 above is where ZERO LATERAL OFFSET sits, and the seven runs are
 * still the evidence for it. What has changed is the learner underneath.
 *
 * It used to minimise mean HEADING error, which is a different target, and it
 * converged to about -34 rather than -26 - so warm runs settled roughly 1 cm
 * left while cold ones landed on zero. It now minimises mean CORRECTION: the
 * average of what the steering loop actually had to put out, whose zero is
 * "the robot needs no steering", which is much closer to zero lateral offset.
 * See TRIM_LEARN_FROM_OUTPUT in odom.h.
 *
 * WHERE IT NOW LANDS HAS NOT BEEN MEASURED. The expectation is nearer -26
 * than -34, but that is reasoning, not a tape measure. Re-run the seven-point
 * fit before trusting 1474 for warm runs; the cold-run case it was chosen for
 * is unaffected either way, because a cold trim is 0 whatever learns it.
 *
 * Note the travel is now asymmetric: 626 us of pulse above centre, 574
 * below. arc_steer_us() already takes the smaller side so left and right
 * arcs stay identical, which costs MOTION_ARC_STEER_US 575 exactly 1 us.
 * About 0.05 degrees of steer. Do not "fix" this by moving SERVO_MIN_US or
 * SERVO_MAX_US - those are the mechanical stops from the mode 7 sweep. */
#define SERVO_CENTER_US     1474U

/* Absolute safety bounds for the end-stop sweep ONLY.
 *
 * Servo_SetRawUs() clamps to these instead of SERVO_MIN_US/MAX_US, so the
 * sweep can explore past the provisional limits to find where the linkage
 * actually binds.
 *
 * 500-2500 is the widest range a hobby servo will accept. USE IT CAREFULLY.
 *
 * The limit on this robot is the STEERING LINKAGE, not the servo's travel.
 * Once the front wheels stop moving, more pulse does not turn them further -
 * it pushes the servo into a stop it cannot pass, at full stall torque. The
 * HWZ020 has plastic gears and they can STRIP under that load. This is not
 * just a heat problem; it can end the steering permanently.
 *
 * Step slowly, watch the WHEELS rather than the numbers, and stop the moment
 * they stop moving. The known-good span is roughly 1300-1700; anything beyond
 * that is unexplored and the display flags it.
 *
 * Nothing in normal operation should use these. */
/* Overshoot used when settling the steering onto a target angle.
 *
 * The linkage has roughly 50 us of slack. Command 1500 coming DOWN from a
 * right turn and the slack sits on one side; command 1500 coming UP from a
 * left turn and it sits on the other. The servo is at 1500 either way, but
 * the WHEELS end up a little right or a little left depending on which turn
 * came last - which is exactly the "stops but does not fully align" symptom.
 *
 * So always arrive from the same side: go this far BELOW the target first,
 * then come up onto it. The slack is then resolved identically every time,
 * and whatever residual offset is left becomes a constant that
 * SERVO_CENTER_US can absorb.
 *
 * Slightly larger than the measured slack so it is guaranteed to clear it. */
#define SERVO_APPROACH_US     60U

#define SERVO_ABS_MIN_US     500U
#define SERVO_ABS_MAX_US    2500U


void Motors_Init(void);

/* speed: -1000 (full reverse) .. 0 (coast) .. +1000 (full forward) */
void Motor_A_Set(int16_t speed);
void Motor_B_Set(int16_t speed);

/* Both inputs high - shorts the motor terminals, stops hard. */
void Motor_A_Brake(void);
void Motor_B_Brake(void);
void Motors_Brake(void);

/* Both inputs low - outputs float, motor freewheels. */
void Motors_Coast(void);

/* Starts the steering servo on TIM12_CH2, held at centre. */
void Servos_Init(void);

/* Pulse width in microseconds, clamped to SERVO_MIN_US..SERVO_MAX_US. */
void Servo_SetMicroseconds(uint16_t us);

/* Bypasses SERVO_MIN_US/MAX_US and clamps to SERVO_ABS_* instead. For the
 * end-stop sweep only - it exists to find what SERVO_MIN_US and SERVO_MAX_US
 * should be, so it cannot be bounded by them. Step gently and stop the moment
 * the wheels stop moving or the servo starts buzzing: that is the mechanical
 * stop, and holding against it stalls the servo. */
void Servo_SetRawUs(uint16_t us);

/* 0..180 mapped linearly across the clamped range. 90 is NOT the straight-
 * ahead position unless SERVO_CENTER_US happens to sit mid-range - use
 * Servo_SetMicroseconds(SERVO_CENTER_US) to centre the steering. */
void Servo_SetAngle(uint8_t degrees);

/* Blocking bring-up sequence. Chassis on blocks before calling. */
void Motors_TestSequence(void);

#endif /* __MOTORS_H */
