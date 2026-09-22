/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FUNCTIONAL TEST BUILD  (C30D V2.1 / F407VET6)
  *
  *  MDP Group 15. Ackermann chassis: two driven rear wheels, one steering
  *  servo on the front axle. The robot cannot turn on the spot.
  *
  *  PURPOSE OF THIS BUILD
  *  Prove checklist A.3 (straight line, 80-120 cm, +/-6%, no visible
  *  deviation) and prove the IR and ultrasonic sensors read distance
  *  correctly - both WITHOUT the Raspberry Pi.
  *
  *  Everything is driven through the same PeripheralDrivers the RPi command
  *  layer will call later, so the numbers measured here carry straight over.
  *  Nothing calibrated in this build has to be redone after RPi integration.
  *
  *  MODES - LONG press the user button (PE0) to cycle, SHORT press to act.
  *  A long press also aborts whatever is moving.
  *
  *    1 CALIB    SHORT drives one PROTOCOL.md §7 calibration cycle - a
  *               straight, then four arcs - and the learning that already
  *               happens at the end of every move does the rest. Shows what
  *               the cycle CHANGED. Press again until that stops moving.
  *               Boots into mode 2, not this one, on purpose.
  *    2 DRIVE    A.3 run. SHORT drives the selected distance and holds the
  *               result on screen: target, odometry, error %, and B/A
  *               encoder agreement for that run.
  *    3 SETDIST  SHORT steps the target 800..1200 mm in 100 mm steps.
  *    4 TURN     A.4 run. SHORT turns the selected angle and holds the
  *               result: commanded, measured, error, radius, cross-check.
  *    5 SETANGLE SHORT steps angle 90/180/270/360 and the direction.
  *    6 PROFILE  SHORT cycles the arc profiles. Shows the learned decel and
  *               brake lag, which is where converged seed values come from.
  *    7 SERVO    End-stop sweep. Auto-centres 2 s after the last press.
  *    8 SENSE    Live calibrated distances from both IRs and the ultrasonic,
  *               plus the echo counter. SHORT streams a sample to USART3.
  *    9 IRCAL    Median-filtered ADC counts, for fitting the IR curve in
  *               ir.h. SHORT streams a sample to USART3.
  *   10 IMU      Gyro diagnostics: heading, rate, poll rate, stalls, and the
  *               peak raw value against the full-scale rail.
  *   11 CAL      What ?CAL would answer right now, in the EXACT integers the
  *               wire carries, plus a tally of what the !CAL* setters have
  *               done. The console latches off the moment the host speaks, so
  *               during a calibrate.py run this is the only window. SHORT
  *               zeroes the tally.
  *
  *  CONTROL TICK - TIM6, 100 Hz, priority 6. Order is not negotiable:
  *      Encoders_Update() -> IR_Update() -> Ultrasonic_Tick() -> IMU_Tick()
  *                        -> Motion_Tick() -> Odom_Update() -> PID_Update()
  *
  *  Sensors first, then control. IMU_Tick() must precede Odom_Update() or the
  *  heading loop acts on a value one tick stale.
  *
  *  The sensors are cheap by construction, so they ride in the tick without
  *  disturbing it: IR_Update() only reads a buffer the DMA has already
  *  filled, and Ultrasonic_Tick() does nothing on five ticks out of six. The
  *  only busy-wait anywhere is the 11 us trigger pulse, once every 60 ms.
  *  See ir.h and ultrasonic.h.
  *
  *  SAFETY: motor PWM is brought up and pinned at zero before anything else
  *  can touch it. Floating AT8236 inputs are a confirmed runaway mode on this
  *  board.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motors.h"
#include "encoders.h"
#include "pid.h"
#include "odom.h"
#include "motion.h"
#include "ir.h"
#include "ultrasonic.h"
#include "imu.h"
#include "commands.h"
#include "rpilink.h"
#include "oled.h"
#include <stdio.h>

/* Private define ------------------------------------------------------------*/

/* Distance the DRIVE mode commands, mm. A.3 asks for a supervisor-specified
   distance between 800 and 1200. 1000 is the middle of that band. */
#define DIST_TARGET_MM       1000

#define BTN_LONG_TICKS       60       /* 600 ms */
#define BTN_DEBOUNCE_TICKS   3        /* 30 ms  */

/* Selectable range for the A.3 run. The checklist says the supervisor names
   a distance between 80 and 120 cm on the day, so the whole band has to be
   reachable from the button without a rebuild. */
#define DIST_MIN_MM          800
#define DIST_MAX_MM          1200
#define DIST_STEP_MM         100

typedef enum { MODE_CALIB = 0, MODE_DRIVE, MODE_SETDIST, MODE_TURN,
               MODE_SETANGLE, MODE_PROFILE, MODE_SERVO, MODE_SENSE,
               MODE_IRCAL, MODE_IMU, MODE_CAL, MODE_COUNT } uimode_t;

/* ---------------------------------------------------------------------------
 * MODE 1: THE CALIBRATION CYCLE
 *
 * PROTOCOL.md §7 gives the sequence that converges the three learned values:
 * a straight long enough to teach the trim, then four arcs. This drives it
 * from the button so the whole thing can be done without the Pi.
 *
 * Nothing here learns anything. The learning already happens at the end of
 * every move inside motion.c - Odom_LearnTrim() after a straight, the decel
 * and lag update after an arc - and it does not care whether the move came
 * from this button or off the wire. This mode only drives §7's moves in
 * §7's order. That is the whole trick, and it is why this is a sequencer and
 * not a calibration routine.
 *
 * ONE PRESS IS ONE CYCLE, NOT THE WHOLE CONVERGENCE. Convergence is
 * iterative by nature: press, watch dec and lag on mode 11, press again,
 * stop when they stop moving. Doing it in one press would mean a firmware-
 * driven sequence running for minutes, which is exactly what PROTOCOL.md §7
 * rule 1 exists to prevent.
 *
 * THE HOST ALWAYS WINS. If a command line arrives while a cycle is running,
 * the cycle cancels itself - see Calib_Tick(). §7 rule 1 says the firmware
 * never drives itself so that no mode can still be running when a task
 * starts; a button sequence is not a wire command, but it creates the same
 * hazard, and yielding to the host is how this mode stays inside the spirit
 * of that rule.
 * ------------------------------------------------------------------------- */

/* The trim needs TRIM_WARMUP_TICKS + TRIM_MIN_SAMPLES of HOLDING - one second
   - and the accel and brake ramps are not free. 800 mm leaves real margin on
   a low battery; 600 cleared the bar on paper and not much else. A straight
   that falls short teaches nothing AND says nothing, which is why the screen
   below calls it out rather than showing a trim that merely did not move. */
#define CALIB_STRAIGHT_MM    800
#define CALIB_ARC_DEG        90

/* Between steps. Each move should start from a robot that has stopped
   rocking, and the gap is also what makes the step advance unambiguous -
   see Calib_Tick(). */
#define CALIB_SETTLE_MS      400U

/* The §7 sequence: one straight, then four arcs ALTERNATING FORWARD AND
   REVERSE ON THE SAME STEERING SIDE. FR90 then RR90 retraces the same arc
   backwards, so the robot returns to its start pose instead of walking
   across the room - four arcs fit on one patch of floor.

   CAVEAT: reverse arcs are implemented but motion.c:346 still marks their
   sign convention "VERIFY THIS ON THE ROBOT". If the reverse legs go the
   wrong way, change the two 0s below to 1s: FR90/FL90 also returns the
   heading, it just needs more floor because the robot walks forward in an S.
   Watch the first cycle before walking away from it. */
static const uint8_t g_calArcFwd[]   = { 1U, 0U, 1U, 0U };
static const uint8_t g_calArcRight[] = { 1U, 1U, 1U, 1U };
#define CALIB_ARCS      (sizeof(g_calArcFwd) / sizeof(g_calArcFwd[0]))
#define CALIB_STEPS     (1U + CALIB_ARCS)   /* the straight, then the arcs */

typedef enum { CALIB_IDLE = 0, CALIB_RUNNING, CALIB_DONE } calibstate_t;

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef  htim2;    /* encoder A  PA15 / PB3      */
TIM_HandleTypeDef  htim3;    /* encoder B  PB4  / PB5      */
TIM_HandleTypeDef  htim4;    /* motor A    PB8=CH3 PB9=CH4 */
TIM_HandleTypeDef  htim6;    /* 10 ms control tick         */
TIM_HandleTypeDef  htim8;    /* US echo capture PC7 = CH2  */
TIM_HandleTypeDef  htim9;    /* motor B    PE5=CH1 PE6=CH2 */
TIM_HandleTypeDef  htim12;   /* servo      PB15 = CH2      */
UART_HandleTypeDef huart3;   /* PD8 / PD9                  */
ADC_HandleTypeDef  hadc1;    /* IR L PC0, IR R PC1         */
I2C_HandleTypeDef  hi2c2;    /* ICM-20948  PB10/PB11       */
DMA_HandleTypeDef  hdma_adc1;

static volatile uint8_t  g_evtShort = 0, g_evtLong = 0;

/* Boots into DRIVE, not CALIB, deliberately. A stray short press after
   power-on should cost one straight run, not a five-move sequence nobody is
   standing next to. Long press once to reach CALIB. */
static volatile uimode_t g_mode = MODE_DRIVE;

/* Calibration cycle state. Touched only from the main loop - the button ISR
   sets g_evtShort and nothing else - so none of it is volatile. */
static calibstate_t g_calState = CALIB_IDLE;
static uint8_t      g_calStep  = 0U;    /* 0 = straight, 1..4 = the arcs   */
static uint32_t     g_calMark  = 0U;    /* HAL_GetTick() of the last motion */
static uint16_t     g_calRuns  = 0U;    /* cycles completed since power-on  */

/* How the current step ended, latched before anything clears it. MOTION_IDLE
   means "not latched yet" - see Calib_Tick(). */
static MotionState_t g_calOutcome = MOTION_IDLE;

/* Learned values as they stood when the cycle STARTED, so the screen can
   show what the cycle changed. "Have they stopped moving?" is the only
   question this mode exists to answer, and a delta answers it directly
   while two absolute numbers make you remember the previous one. */
static int32_t g_calPrevDecel10 = 0;
static int32_t g_calPrevLag10   = 0;
static int32_t g_calPrevTrim    = 0;
static int32_t g_calDeltaDecel  = 0;
static int32_t g_calDeltaLag    = 0;
static int32_t g_calDeltaTrim   = 0;

/* Commanded distance for the next run. Changed in MODE_SETDIST. */
static volatile int32_t  g_targetMm = DIST_TARGET_MM;

/* Turn selection for A.4, changed in MODE_SETANGLE. The checklist says the
   supervisor names an angle between 90 and 360, so the whole range has to be
   reachable from the button. Both directions, because "specified" does not
   promise which way. */
static const int16_t  g_angleList[] = { 90, 180, 270, 360 };
#define ANGLE_COUNT  (sizeof(g_angleList) / sizeof(g_angleList[0]))
static volatile uint8_t g_angleIdx   = 0;
static volatile uint8_t g_turnRight  = 1;   /* 1 = right, 0 = left */
static volatile uint8_t g_turnFwd    = 1;   /* 1 = forward, 0 = reverse */

/* Latched turn result, same pattern as the distance run. */
static volatile uint8_t g_turnValid  = 0;
static volatile int32_t g_turnTarget = 0;
static volatile int32_t g_turnGot    = 0;

/* Servo end-stop sweep. Steps across SERVO_ABS_MIN_US..SERVO_ABS_MAX_US so
   the real mechanical limits can be found - the ones in motors.h are still
   the provisional Phase 2 guesses, and they are what caps the turn radius. */
#define SERVO_SWEEP_STEP_US  25U

/* Auto-centre the sweep this long after the last button press.
 *
 * The whole point of the sweep is to push the steering until it stops moving,
 * which means it ends every session parked against a mechanical stop with the
 * servo stalled. A stalled servo draws its full stall current continuously and
 * gets hot within a minute. Long-pressing out of the mode recentres it, but
 * that relies on remembering; this does not. */
#define SERVO_SWEEP_HOLD_MS  2000U

static volatile uint16_t g_sweepUs   = SERVO_CENTER_US;
static volatile uint32_t g_sweepAtMs = 0;
static volatile uint8_t  g_sweepHeld = 0;

/* Latched at the end of a DRIVE run so the main loop can print it, and so
   the display can hold the result instead of reverting to a live readout
   the moment the state machine goes back to IDLE. */
static volatile uint8_t  g_reportReady = 0;
static volatile int32_t  g_repMm = 0;
static volatile int32_t  g_repTarget = 0;
static volatile int32_t  g_repCntA = 0;
static volatile int32_t  g_repCntB = 0;

/* Encoder counts at the MOMENT THE RUN STARTS.
   Encoder_x_GetCount() accumulates since boot and nothing resets it during
   normal operation, so subtracting these is the only way to get counts for
   THIS run. Without them B/A becomes a lifetime average that quietly stops
   showing run-to-run scatter - which is the whole thing we are looking for. */
/* Which kind of move produced the latched result. Without it a finished TURN
   sets g_repValid and the DRIVE screen shows a distance error and a B/A ratio
   belonging to something else entirely - numbers that look authoritative and
   are meaningless. */
static volatile uint8_t  g_lastWasTurn = 0;

static volatile int32_t  g_runStartA = 0;
static volatile int32_t  g_runStartB = 0;
static volatile uint8_t  g_repTimeout = 0;
static volatile uint8_t  g_repValid = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);

static void Btn_Poll(void);
static void Display(void);

/* ==========================================================================
 *  Button. Polled from the control tick so the timing is exact.
 * ========================================================================== */

static void Btn_Poll(void)
{
    static uint8_t  down  = 0;
    static uint16_t held  = 0;
    static uint8_t  fired = 0;

    uint8_t now = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET) ? 1U : 0U;

    if (now)
    {
        if (held < 0xFFFFU) { held++; }
        if (!down && (held >= BTN_DEBOUNCE_TICKS)) { down = 1U; }

        /* Fire on crossing the threshold, not on release, so holding the
           button is an immediate stop. */
        if (down && !fired && (held >= BTN_LONG_TICKS))
        {
            g_evtLong = 1U;
            fired     = 1U;
        }
    }
    else
    {
        if (down && !fired) { g_evtShort = 1U; }
        down = 0U; held = 0U; fired = 0U;
    }
}

/* ==========================================================================
 *  Control tick. Short, non-blocking. No HAL_Delay, no UART, no OLED, and
 *  no sensor polling - the ultrasonic trigger lives in the main loop.
 * ========================================================================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* Sensors first, then control. IMU_Tick() integrates the sample
           IMU_Poll() fetched from the main loop, and Odom_Update() now reads
           that heading - so it has to run BEFORE Odom, or the heading loop
           acts on a value one tick stale.

           All three are cheap: IR_Update() reads a buffer the DMA already
           filled, Ultrasonic_Tick() does nothing on five ticks out of six,
           and IMU_Tick() is one multiply-accumulate. No I2C here - that is
           in IMU_Poll(), out in the main loop. */
        Encoders_Update();
        IR_Update();
        Ultrasonic_Tick();
        IMU_Tick();

        /* Zero-rate update. The wheels not moving and no primitive running
           means the true yaw rate is zero, so anything the gyro reads right
           now is bias - a free measurement, taken every time the robot sits
           still between runs. Without it the heading creeps while parked and
           the drift is baked into the next run. */
        if (!Motion_IsBusy() &&
            (Encoder_A_GetDelta() == 0) && (Encoder_B_GetDelta() == 0))
        {
            IMU_TrackBias();
        }

        Motion_Tick();
        Odom_Update();
        PID_Update();

        Btn_Poll();

        /* Latch the result of a finished run for the main loop to print. */
        if (!g_reportReady)
        {
            MotionState_t st = Motion_GetState();

            if ((st == MOTION_DONE) || (st == MOTION_TIMEOUT))
            {
                g_repMm       = (int32_t)Odom_GetDistance();
                g_repCntA     = Encoder_A_GetCount() - g_runStartA;
                g_repCntB     = Encoder_B_GetCount() - g_runStartB;
                g_turnTarget  = Motion_GetTurnTargetDeg();
                g_turnGot     = Motion_GetTurnedDeg();
                g_repTimeout  = (st == MOTION_TIMEOUT) ? 1U : 0U;
                g_reportReady = 1U;
                g_repValid    = g_lastWasTurn ? 0U : 1U;
            }
        }
    }
}

/* HC-SR04 echo, both edges, TIM8_CH2 on PC7. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    Ultrasonic_CaptureCallback(htim);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        RpiLink_RxCallback();
    }
}

/* ==========================================================================
 *  Calibration cycle (mode 1). See the block comment above the mode enum.
 * ========================================================================== */

/* The learned values in the units ?CAL puts on the wire, so the screens and
   the protocol agree without anyone doing arithmetic in their head. */
static int32_t CalDecel10(void)
{
    return (int32_t)((Motion_GetArcDecel() * 10.0f) + 0.5f);
}

static int32_t CalLag10(void)
{
    return (int32_t)((Motion_GetArcLag() * 10000.0f) + 0.5f);
}

/* Kick off step g_calStep. Step 0 is the straight; 1..CALIB_ARCS are arcs. */
static void Calib_StartStep(void)
{
    g_calMark    = HAL_GetTick();
    g_calOutcome = MOTION_IDLE;   /* nothing latched for this step yet */

    if (g_calStep == 0U)
    {
        Motion_DriveDistance(CALIB_STRAIGHT_MM);
    }
    else
    {
        uint8_t i = (uint8_t)(g_calStep - 1U);
        Motion_DriveArc(CALIB_ARC_DEG, g_calArcFwd[i], g_calArcRight[i]);
    }
}

static void Calib_Start(void)
{
    g_calPrevDecel10 = CalDecel10();
    g_calPrevLag10   = CalLag10();
    g_calPrevTrim    = (int32_t)Odom_GetHeadingTrim();
    g_calStep        = 0U;
    g_calState       = CALIB_RUNNING;
    RpiLink_Log("\r\ncalib cycle starting\r\n");
    Calib_StartStep();
}

/* Stop the SEQUENCE. Does not touch the motors - every caller has either
   just stopped them or is about to. Separated for exactly that reason: a
   cancel that also braked would hide which of the two actually happened. */
static void Calib_Cancel(void)
{
    if (g_calState == CALIB_RUNNING)
    {
        g_calState = CALIB_IDLE;
        RpiLink_Log("calib cycle cancelled\r\n");
    }
}

static void Calib_Tick(void)
{
    uint32_t now;

    if (g_calState != CALIB_RUNNING) { return; }

    /* The host outranks the button. A line arriving mid-cycle will start its
       own move, and from here that is indistinguishable from this cycle's
       move finishing - so the next step would be launched underneath the
       host's command. Yield instead. See §7 rule 1 in the mode comment. */
    if (RpiLink_IsBusy())
    {
        Calib_Cancel();
        return;
    }

    now = HAL_GetTick();

    /* While anything is moving, keep the mark fresh. The settle timer then
       only starts counting once the move has actually ended, and the same
       gap covers the first few ticks after Motion_DriveX() is called, when
       the motion layer has not reported busy yet. Without that, a step would
       be declared finished the instant it was started. */
    if (Motion_IsBusy())
    {
        g_calMark = now;
        return;
    }

    /* Latch HOW the step ended, on the first tick after it ended.
     *
     * It cannot be read later: the g_reportReady block further down this
     * same loop calls Motion_ClearState(), which drops TIMEOUT back to IDLE,
     * so by the time the settle expires every move looks like a success.
     * Same trap the line executor documents in rpilink.c, same answer.
     *
     * MOTION_IDLE doubles as "nothing latched": it is also what the state
     * reads for the tick or two between Motion_DriveX() being called and the
     * control tick picking the move up, and latching that would be wrong. */
    if (g_calOutcome == MOTION_IDLE) { g_calOutcome = Motion_GetState(); }

    if ((now - g_calMark) < CALIB_SETTLE_MS) { return; }

    if (g_calOutcome == MOTION_TIMEOUT)
    {
        /* A stalled wheel or a dead encoder. Launching the next arc from a
           pose the odometry has lost is how a bench test becomes a repair. */
        g_calState = CALIB_IDLE;
        RpiLink_Log("calib cycle ABORTED: a move timed out\r\n");
        return;
    }

    /* Step finished and the robot has settled. */
    g_calStep++;

    if (g_calStep < (uint8_t)CALIB_STEPS)
    {
        Calib_StartStep();
        return;
    }

    g_calDeltaDecel = CalDecel10() - g_calPrevDecel10;
    g_calDeltaLag   = CalLag10()   - g_calPrevLag10;
    g_calDeltaTrim  = (int32_t)Odom_GetHeadingTrim() - g_calPrevTrim;
    g_calRuns++;
    g_calState = CALIB_DONE;

    {
        char msg[112];
        char trimNote[40];

        if (Odom_TrimWasUpdated())
        {
            snprintf(trimNote, sizeof(trimNote), "%+ld", (long)g_calDeltaTrim);
        }
        else
        {
            snprintf(trimNote, sizeof(trimNote), "SKIPPED, straight too short");
        }

        snprintf(msg, sizeof(msg),
                 "calib cycle %u done: dec %ld (%+ld) lag %ld (%+ld) "
                 "trim %ld (%s)\r\n",
                 (unsigned)g_calRuns,
                 (long)CalDecel10(), (long)g_calDeltaDecel,
                 (long)CalLag10(),   (long)g_calDeltaLag,
                 (long)Odom_GetHeadingTrim(), trimNote);
        RpiLink_Log(msg);
    }
}

/* ==========================================================================
 *  Display
 * ========================================================================== */

static void ShowLine(uint8_t y, const char *s)
{
    char pad[17];
    int i = 0;
    while ((s[i] != '\0') && (i < 16)) { pad[i] = s[i]; i++; }
    while (i < 16) { pad[i++] = ' '; }
    pad[16] = '\0';
    OLED_ShowString(0, y, (const uint8_t *)pad);
}

/* Prints a distance, or four dashes when the sensor has nothing to say.
   Never print a number for a missing reading - a stale value that looks
   real is worse than an obvious gap. */
static void FmtCm(char *buf, int n, const char *label, uint16_t cm)
{
    if (cm == SENSOR_NO_READING) { snprintf(buf, n, "%s ---", label); }
    else                         { snprintf(buf, n, "%s %3u cm", label, cm); }
}

static void Display(void)
{
    static const char *st[] = { "IDLE", "ALGN", "RUN ", "BRK ", "DONE", "TMO " };
    char    line[40];
    int32_t h10;
    long    whole, frac;

    switch (g_mode)
    {
    case MODE_CALIB:
        {
            /* The profile is on the title line because it is what you are
               calibrating. The learned values belong to whichever profile
               was active when they were learned, and this mode does not set
               one - use mode 6 first if it is not the one you will drive. */
            const ArcProfile_t *pr =
                Motion_GetArcProfileInfo(Motion_GetArcProfile());

            snprintf(line, sizeof(line), "1 CALIB %s", pr->name);
            ShowLine(0, line);

            if (g_calState == CALIB_RUNNING)
            {
                snprintf(line, sizeof(line), "step %u/%u %s",
                         (unsigned)(g_calStep + 1U), (unsigned)CALIB_STEPS,
                         (g_calStep == 0U) ? "str" : "arc");
            }
            else if (g_calState == CALIB_DONE)
            {
                snprintf(line, sizeof(line), "done  run %u",
                         (unsigned)g_calRuns);
            }
            else
            {
                snprintf(line, sizeof(line), "READY  run %u",
                         (unsigned)g_calRuns);
            }
            ShowLine(12, line);

            /* One line per learned value, each carrying what the last
               finished cycle CHANGED it by. The delta is the whole point: a
               cycle that moved nothing is a converged robot, and putting it
               beside the value beats making you remember the previous one.
               trm's delta is the one to watch — it is the value that used to
               walk every run and never settle. */
            if (g_calState == CALIB_DONE)
            {
                snprintf(line, sizeof(line), "dec %5ld %+5ld",
                         (long)CalDecel10(), (long)g_calDeltaDecel);
                ShowLine(24, line);
                snprintf(line, sizeof(line), "lag %4ld %+4ld",
                         (long)CalLag10(), (long)g_calDeltaLag);
                ShowLine(36, line);
                /* SKIP, not a delta of zero. The straight was too short to
                   teach the trim anything, and an untouched trim reads
                   exactly like a converged one - so say which it was. */
                if (Odom_TrimWasUpdated())
                {
                    snprintf(line, sizeof(line), "trm %+4ld %+4ld",
                             (long)Odom_GetHeadingTrim(), (long)g_calDeltaTrim);
                }
                else
                {
                    snprintf(line, sizeof(line), "trm %+4ld SKIP",
                             (long)Odom_GetHeadingTrim());
                }
                ShowLine(48, line);
            }
            else
            {
                snprintf(line, sizeof(line), "dec %5ld", (long)CalDecel10());
                ShowLine(24, line);
                snprintf(line, sizeof(line), "lag %4ld", (long)CalLag10());
                ShowLine(36, line);
                snprintf(line, sizeof(line), "trm %+4ld  %s",
                         (long)Odom_GetHeadingTrim(),
                         (g_calState == CALIB_RUNNING) ? "STOP" : "GO");
                ShowLine(48, line);
            }
        }
        break;

    case MODE_SETANGLE:
        ShowLine(0,  "5 SET ANGLE");
        snprintf(line, sizeof(line), "%s%d  %s",
                 g_turnRight ? "R" : "L",
                 (int)g_angleList[g_angleIdx],
                 g_turnFwd ? "fwd" : "rev");
        ShowLine(12, line);
        ShowLine(24, "short = next");
        ShowLine(36, "90/180/270/360");
        ShowLine(48, "R fwd L fwd R rev");
        break;

    case MODE_TURN:
        snprintf(line, sizeof(line), "4 TURN %s%s",
                 Motion_IsBusy() ? st[(int)Motion_GetState()] : "",
                 IMU_IsReady() ? "" : " !NOGYRO");
        ShowLine(0, line);
        snprintf(line, sizeof(line), "%s%d %s",
                 g_turnRight ? "R" : "L",
                 (int)g_angleList[g_angleIdx],
                 g_turnFwd ? "fwd" : "rev");
        ShowLine(12, line);

        if (Motion_IsBusy())
        {
            snprintf(line, sizeof(line), "now %ld deg",
                     (long)Motion_GetTurnedDeg());
            ShowLine(24, line);
            snprintf(line, sizeof(line), "A %ld B %ld",
                     (long)Encoder_A_GetRPM(), (long)Encoder_B_GetRPM());
            ShowLine(36, line);
            snprintf(line, sizeof(line), "sv %u us",
                     (unsigned)Odom_GetServoUs());
            ShowLine(48, line);
        }
        else if (g_turnValid)
        {
            /* Radius implied by the arc length the encoders measured and the
               angle the gyro measured:

                   R = arc_length / angle_in_radians

               Both numbers are already known, so this costs nothing. TREAT IT
               AS INDICATIVE ONLY. On a boosted profile the rear tyres are
               deliberately scrubbed, so the wheels turn further than the
               ground travelled and this reads high by an unknown amount - it
               is honest only at diff_boost 1.0.

               The authoritative radius comes off the floor: mark under the
               rear axle, run a 90, mark again, R = chord / 1.414. That is the
               only measurement in the system that owes the gyro nothing, and
               it is what the per-profile radius_mm values were set from. */
            long turned = (long)g_turnGot;
            long mag    = (turned < 0) ? -turned : turned;
            long radius = (mag > 0)
                        ? (long)(Motion_GetTravelled() * 57.2958f / (float)mag)
                        : 0L;

            snprintf(line, sizeof(line), "got %ld deg  %s", (long)g_turnGot,
                     Motion_GetArcProfileInfo(Motion_GetArcProfile())->name);
            ShowLine(24, line);
            snprintf(line, sizeof(line), "err %ld deg",
                     (long)(g_turnGot - g_turnTarget));
            ShowLine(36, line);
            /* Second witness on the bottom line. XOK means the encoders
               agree with the gyro; XBAD means they do not and something needs
               a tape measure. X-- means the check did not run, which is
               normal on a boosted profile - scrubbed tyres make the encoder
               arc meaningless. */
            if (Motion_GetXCheckDeg() > 0.0f)
            {
                snprintf(line, sizeof(line), "R%ld %s %+d%%", radius,
                         Motion_XCheckFailed() ? "XBAD" : "XOK",
                         (int)Motion_GetXCheckErrPct());
            }
            else
            {
                snprintf(line, sizeof(line), "arc%ld R%ld X--",
                         (long)Motion_GetTravelled(), radius);
            }
            ShowLine(48, line);
        }
        else
        {
            ShowLine(24, "short = GO");
            ShowLine(36, "");
            ShowLine(48, "");
        }
        break;

    case MODE_PROFILE:
        {
            const ArcProfile_t *pr = Motion_GetArcProfileInfo(Motion_GetArcProfile());
            uint16_t steer = Motion_GetArcSteerUs();

            snprintf(line, sizeof(line), "6 PROFILE %s", pr->name);
            ShowLine(0, line);
            snprintf(line, sizeof(line), "steer %u us%s", (unsigned)steer,
                     (steer < pr->steer_us) ? " CLMP" : "");
            ShowLine(12, line);
            snprintf(line, sizeof(line), "rpm %d/%d", pr->rpm, pr->approach_rpm);
            ShowLine(24, line);
            snprintf(line, sizeof(line), "R%d a%d lag%dms",
                     (int)pr->radius_mm, (int)Motion_GetArcDecel(),
                     (int)(Motion_GetArcLag() * 1000.0f));
            ShowLine(36, line);
            ShowLine(48, "short = next");
        }
        break;

    case MODE_SERVO:
        ShowLine(0,  "7 SERVO SWEEP");
        snprintf(line, sizeof(line), "us  %u", (unsigned)g_sweepUs);
        ShowLine(12, line);
        snprintf(line, sizeof(line), "off %+d",
                 (int)g_sweepUs - (int)SERVO_CENTER_US);
        ShowLine(24, line);
        snprintf(line, sizeof(line), "lim %u-%u",
                 (unsigned)SERVO_MIN_US, (unsigned)SERVO_MAX_US);
        ShowLine(36, line);
        {
            /* Flag anything outside the known-good 1300-1700 span. Past the
               linkage stop the servo is stalling at full torque, and on a
               plastic-geared servo that strips teeth rather than just getting
               hot. The wheels are the real indicator - if they have stopped
               moving, stop stepping. */
            int off = (int)g_sweepUs - (int)SERVO_CENTER_US;
            if ((off > 200) || (off < -200))
            {
                ShowLine(48, g_sweepHeld ? "** PAST SPAN **" : "centred");
            }
            else
            {
                ShowLine(48, g_sweepHeld ? "HELD short=+25" : "centred short=+25");
            }
        }
        break;

    case MODE_SENSE:
        ShowLine(0, "8 SENSE");
        FmtCm(line, sizeof(line), "IRL", IR_LeftCm());       ShowLine(12, line);
        FmtCm(line, sizeof(line), "IRR", IR_RightCm());      ShowLine(24, line);
        FmtCm(line, sizeof(line), "US ", Ultrasonic_GetCm()); ShowLine(36, line);
        /* rx is the UART re-arm count and belongs on a screen rather than in
           the log, because it matters most while the RPi is driving - and by
           then the console has gone quiet. Anything but 0 means the link is
           glitching and being silently recovered. */
        snprintf(line, sizeof(line), "ech%lu rx%lu",
                 (unsigned long)Ultrasonic_GetEchoCount(),
                 (unsigned long)RpiLink_GetRearmCount());
        ShowLine(48, line);
        break;

    case MODE_IRCAL:
        /* Median, not the live sample. The screen exists to have a number
           copied off it at a known distance, and a raw Sharp reading moves
           too much to read. Title says so - a screen labelled "raw" showing
           a filtered value would be its own trap. */
        ShowLine(0, "9 IRCAL median");
        snprintf(line, sizeof(line), "L %4u cnt", IR_LeftFiltered());
        ShowLine(12, line);
        snprintf(line, sizeof(line), "R %4u cnt", IR_RightFiltered());
        ShowLine(24, line);
        snprintf(line, sizeof(line), "US %5u us", Ultrasonic_GetLastUs());
        ShowLine(36, line);
        ShowLine(48, "card @ known d");
        break;

    case MODE_IMU:
        ShowLine(0, "10 IMU gyro");
        if (!IMU_IsReady())
        {
            ShowLine(12, "NOT READY");
            snprintf(line, sizeof(line), "addr %02X id %02X",
                     IMU_GetAddress(), IMU_GetWhoAmI());
            ShowLine(24, line);
            ShowLine(36, "PB12 high?");
            ShowLine(48, "1.8V rail?");
        }
        else
        {
            long h10 = (long)(IMU_GetHeading() * 10.0f);
            long r10 = (long)(IMU_GetRateDps() * 10.0f);

            /* Largest |raw| since the last reset, against the +-32767 rail.
               This is the clipping detector. A turn that reads short with pk
               sitting near 32767 is a saturated gyro, not a bad calibration -
               and that fault is invisible on every other number on this
               screen, because they all derive from the same clipped samples.
               Short press zeroes it: zero, run a turn, come back and read.
               Address and ID have moved to the NOT READY branch, which is
               where they actually help. */
            snprintf(line, sizeof(line), "pk%5d/32767",
                     (int)IMU_GetPeakRaw());
            ShowLine(12, line);
            snprintf(line, sizeof(line), "hd %ld.%ld deg",
                     h10 / 10, (h10 < 0 ? -h10 : h10) % 10);
            ShowLine(24, line);
            snprintf(line, sizeof(line), "rate %ld.%ld dps",
                     r10 / 10, (r10 < 0 ? -r10 : r10) % 10);
            ShowLine(36, line);
            snprintf(line, sizeof(line), "hz%lu er%lu st%lu",
                     (unsigned long)IMU_GetPollRate(),
                     (unsigned long)IMU_GetErrorCount(),
                     (unsigned long)IMU_GetStallCount());
            ShowLine(48, line);
        }
        break;

    case MODE_CAL:
        {
            /* The three numbers ?CAL puts on the wire, in the units it puts
               them there in: decel and lag scaled x10, trim in whole us. NOT
               converted to something friendlier, because the point of this
               screen is to be compared against what the host printed. A
               screen showing 1234 dps^2 next to a log line reading 12340
               costs you the comparison it exists to make.

               Mode 6 already shows decel and lag in real units, and mode 2
               shows the trim after a run. This is the only place all three
               appear together, and the only place they appear as sent. */
            long decel10 = (long)CalDecel10();
            long lag10   = (long)CalLag10();
            long trim    = (long)Odom_GetHeadingTrim();

            /* Clamped for the display only. Four columns of counter would not
               fit beside their labels, and the counts that matter here are
               small - a calibrate.py restore sends exactly three setters. */
            unsigned okN  = (unsigned)RpiLink_GetCalOkCount();
            unsigned bsyN = (unsigned)RpiLink_GetCalBusyCount();
            unsigned rgN  = (unsigned)RpiLink_GetCalRangeCount();
            if (okN  > 99U) { okN  = 99U; }
            if (bsyN > 99U) { bsyN = 99U; }
            if (rgN  > 99U) { rgN  = 99U; }

            ShowLine(0, "11 CAL wire");
            snprintf(line, sizeof(line), "dec %6ld", decel10);
            ShowLine(12, line);
            /* Both on one line so the tally can have its own. Exactly 16
               characters at the widest, which is what ShowLine fits. */
            snprintf(line, sizeof(line), "lag%5ld trm%+4ld", lag10, trim);
            ShowLine(24, line);
            /* bsy and rg are both RESEND on the wire. Telling them apart is
               the whole reason this screen exists - see rpilink.h. */
            snprintf(line, sizeof(line), "ok%2u bsy%2u rg%2u", okN, bsyN, rgN);
            ShowLine(36, line);
            ShowLine(48, "short = zero");
        }
        break;

    case MODE_SETDIST:
        ShowLine(0,  "3 SET DISTANCE");
        snprintf(line, sizeof(line), "tgt %4ld mm", (long)g_targetMm);
        ShowLine(12, line);
        ShowLine(24, "short = +100");
        snprintf(line, sizeof(line), "range %d-%d",
                 (int)DIST_MIN_MM, (int)DIST_MAX_MM);
        ShowLine(36, line);
        ShowLine(48, "long = next mode");
        break;

    case MODE_DRIVE:
    default:
        /* No %f anywhere. newlib-nano will not print floats unless the float
           printf is linked, and it fails silently rather than loudly. */
        h10   = (int32_t)(Odom_GetHeading() * 10.0f);
        whole = (long)(h10 / 10);
        frac  = (long)(h10 % 10);
        if (frac < 0) { frac = -frac; }

        if (Motion_IsBusy())
        {
            /* Live view while moving. */
            /* The '!' means the gyro is not running and heading has fallen
               back to the encoder difference. Accuracy drops, and the fact
               that it is silent is exactly how a 90 degree turn once ran to
               369 - so say so on the screen the run is being watched on. */
            snprintf(line, sizeof(line), "2 DRIVE %s%s",
                     st[(int)Motion_GetState()],
                     IMU_IsReady() ? "" : " !");
            ShowLine(0, line);
            snprintf(line, sizeof(line), "tgt %4ld mm", (long)g_targetMm);
            ShowLine(12, line);
            snprintf(line, sizeof(line), "now %4ld mm",
                     (long)Motion_GetTravelled());
            ShowLine(24, line);
            snprintf(line, sizeof(line), "A %ld B %ld",
                     (long)Encoder_A_GetRPM(), (long)Encoder_B_GetRPM());
            ShowLine(36, line);
            snprintf(line, sizeof(line), "sv%u y%+d hd%ld.%ld",
                     (unsigned)Odom_GetServoUs(),
                     (int)Odom_GetCrossTrack(), whole, frac);
            ShowLine(48, line);
        }
        else if (g_repValid)
        {
            /* Held result. This stays on screen until the next run starts,
               so you can walk over with a tape measure and still read it.
               Deliberately NOT driven off Motion_GetState() - the main loop
               clears that to IDLE as soon as it has taken the reading, so a
               live view would blank the answer almost immediately. */
            long e10 = (g_repTarget != 0)
                     ? (long)(((g_repMm - g_repTarget) * 1000) / g_repTarget)
                     : 0L;
            long ei  = e10 / 10;
            long ef  = e10 % 10;
            if (ef < 0) { ef = -ef; }

            ShowLine(0, g_repTimeout ? "2 DRIVE TIMEOUT" : "2 DRIVE DONE");
            /* Target and result share a line so the learned steering trim
               can have one. Exactly 16 characters, which is what ShowLine
               fits. */
            snprintf(line, sizeof(line), "tgt%4ld got%4ld",
                     (long)g_repTarget, (long)g_repMm);
            ShowLine(12, line);
            /* The settled steering trim. It otherwise exists only in the
               serial report, and RpiLink_Log() goes silent for good the
               moment the RPi sends its first line - so on a robot driven by
               the Pi there was no way to read it at all. Fold this into
               SERVO_CENTER_US once it stops moving between runs. */
            snprintf(line, sizeof(line), "trim %+d us",
                     (int)Odom_GetHeadingTrim());
            ShowLine(24, line);
            snprintf(line, sizeof(line), "err %s%ld.%ld %%",
                     (e10 < 0) ? "-" : "+", (ei < 0) ? -ei : ei, ef);
            ShowLine(36, line);
            /* y is where the firmware BELIEVES it finished, left-positive.
               Compare it against a tape measure: the heading loop drives this
               to zero, so if y reads 0 and the tape says otherwise, the error
               is in the gyro rather than in the steering.

               B/A is encoder agreement for THIS run. 100% means both wheels
               counted the same. Anything well below that is wheel B slipping
               under power, and it is the number to watch before trusting any
               distance figure. It is NOT a straightness meter - seven runs
               put its correlation with real offset at r = -0.57, with 99.5%
               turning up at both +30 mm and -15 mm. */
            if (g_repCntA != 0)
            {
                long r10 = (long)((g_repCntB * 1000) / g_repCntA);
                if (r10 < 0) { r10 = -r10; }
                snprintf(line, sizeof(line), "y%+d BA%ld.%ld",
                         (int)Odom_GetCrossTrack(), r10 / 10, r10 % 10);
            }
            else
            {
                snprintf(line, sizeof(line), "y%+d BA---",
                         (int)Odom_GetCrossTrack());
            }
            ShowLine(48, line);
        }
        else
        {
            ShowLine(0,  "2 DRIVE READY");
            snprintf(line, sizeof(line), "tgt %4ld mm", (long)g_targetMm);
            ShowLine(12, line);
            ShowLine(24, "short = GO");
            snprintf(line, sizeof(line), "A %ld B %ld rpm",
                     (long)Encoder_A_GetRPM(), (long)Encoder_B_GetRPM());
            ShowLine(36, line);
            snprintf(line, sizeof(line), "sv %u us",
                     (unsigned)Odom_GetServoUs());
            ShowLine(48, line);
        }
        break;
    }

    OLED_Refresh_Gram();
}

/* ==========================================================================
 *  Reporting
 * ========================================================================== */

static void PrintDriveReport(void)
{
    char     line[96];
    int32_t  target   = g_repTarget;
    int32_t  measured = g_repMm;
    int32_t  err      = measured - target;
    int32_t  err_x10  = (target != 0) ? ((err * 1000) / target) : 0;
    long     ipart, fpart;

    ipart = (long)(err_x10 / 10);
    fpart = (long)(err_x10 % 10);
    if (fpart < 0) { fpart = -fpart; }

    RpiLink_Log("\r\n--- A.3 RUN ");
    RpiLink_Log(g_repTimeout ? "TIMEOUT ---\r\n" : "DONE ---\r\n");

    snprintf(line, sizeof(line), "commanded  %ld mm\r\n", (long)target);
    RpiLink_Log(line);
    snprintf(line, sizeof(line), "odometry   %ld mm  (%ld.%ld %% off)\r\n",
             (long)measured, ipart, fpart);
    RpiLink_Log(line);

    snprintf(line, sizeof(line), "counts     A %ld   B %ld\r\n",
             (long)g_repCntA, (long)g_repCntB);
    RpiLink_Log(line);
    if (g_repCntA != 0)
    {
        snprintf(line, sizeof(line), "B/A        %ld.%ld %%  (100 = agree)\r\n",
                 (long)((g_repCntB * 1000) / g_repCntA) / 10,
                 (long)((g_repCntB * 1000) / g_repCntA) % 10);
        RpiLink_Log(line);
    }

    snprintf(line, sizeof(line),
             "steer trim %+d us  -> add to SERVO_CENTER_US when settled\r\n",
             (int)Odom_GetHeadingTrim());
    RpiLink_Log(line);

    RpiLink_Log("Now TAPE MEASURE the real distance.\r\n");
    RpiLink_Log("Two separate faults, two separate numbers:\r\n");
    RpiLink_Log("  odometry vs TAPE  = wheel diameter\r\n");
    RpiLink_Log("    WHEEL_DIAMETER_MM *= tape / odometry\r\n");
    RpiLink_Log("  odometry vs COMMANDED = brake coast\r\n");
    RpiLink_Log("    put the excess in MOTION_BRAKE_MM\r\n");
    RpiLink_Log("Use odometry, not commanded, for the wheel -\r\n");
    RpiLink_Log("odometry keeps counting through the coast, so\r\n");
    RpiLink_Log("the tape/odometry ratio is coast-immune.\r\n");
    RpiLink_Log("Five runs, average, then re-check.\r\n\r\n");
}

static void PrintTurnReport(void)
{
    char line[96];

    {
        const ArcProfile_t *pr = Motion_GetArcProfileInfo(Motion_GetArcProfile());
        snprintf(line, sizeof(line), "\r\n[profile %s]  ", pr->name);
        RpiLink_Log(line);
    }
    RpiLink_Log("--- A.4 TURN ");
    RpiLink_Log(g_repTimeout ? "TIMEOUT ---\r\n" : "DONE ---\r\n");

    snprintf(line, sizeof(line), "commanded  %ld deg\r\n", (long)g_turnTarget);
    RpiLink_Log(line);
    snprintf(line, sizeof(line), "gyro       %ld deg  (err %ld)\r\n",
             (long)g_turnGot, (long)(g_turnGot - g_turnTarget));
    RpiLink_Log(line);
    {
        long mag = (g_turnGot < 0) ? -g_turnGot : g_turnGot;
        snprintf(line, sizeof(line), "arc length %ld mm   radius %ld mm\r\n",
                 (long)Motion_GetTravelled(),
                 (mag > 0) ? (long)(Motion_GetTravelled() * 57.2958f / (float)mag) : 0L);
        RpiLink_Log(line);
    }

    snprintf(line, sizeof(line), "learned decel %d deg/s2, lag %d ms\r\n",
             (int)Motion_GetArcDecel(), (int)(Motion_GetArcLag() * 1000.0f));
    RpiLink_Log(line);

    if (Motion_GetXCheckDeg() > 0.0f)
    {
        snprintf(line, sizeof(line),
                 "cross-check: encoders say %d deg, gyro says %d, %+d%% %s\r\n",
                 (int)Motion_GetXCheckDeg(), (int)Motion_GetTurnedDeg(),
                 (int)Motion_GetXCheckErrPct(),
                 Motion_XCheckFailed() ? "*** DISAGREE ***" : "agree");
        RpiLink_Log(line);
    }
    else
    {
        RpiLink_Log("cross-check skipped - needs a profile with boost 1.0\r\n");
    }
    RpiLink_Log("Braking is adaptive - no constant to set.\r\n\r\n");
}

static void PrintSensors(void)
{
    char line[96];

    snprintf(line, sizeof(line), "ENC A %ld  B %ld   (/10: %ld %ld)\r\n",
             (long)Encoder_A_GetCount(), (long)Encoder_B_GetCount(),
             (long)(Encoder_A_GetCount() / 10),
             (long)(Encoder_B_GetCount() / 10));
    RpiLink_Log(line);

    /* Median first - that is the calibration number. Raw is alongside it as a
       liveness check: frozen raw means the ADC or the wiring, not the fit. */
    snprintf(line, sizeof(line),
             "IR med L %4u R %4u   (raw %4u %4u)\r\n",
             IR_LeftFiltered(), IR_RightFiltered(),
             IR_LeftRaw(), IR_RightRaw());
    RpiLink_Log(line);

    snprintf(line, sizeof(line),
             "US echo %5u us  n=%lu\r\n",
             Ultrasonic_GetLastUs(),
             (unsigned long)Ultrasonic_GetEchoCount());
    RpiLink_Log(line);

    snprintf(line, sizeof(line),
             "IR cm  L %5u R %5u   US %5u cm   (%u = no reading)\r\n",
             IR_LeftCm(), IR_RightCm(), Ultrasonic_GetCm(),
             (unsigned)SENSOR_NO_READING);
    RpiLink_Log(line);
}

/* ==========================================================================
 *  main
 * ========================================================================== */

int main(void)
{
    uint32_t tDisp = 0, tLed = 0;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();

    /* Motor PWM up and pinned at zero BEFORE anything can spin. */
    MX_TIM4_Init();
    MX_TIM9_Init();
    Motors_Init();

    /* Steering next, so the front wheels are straight before the rears can
       ever turn. */
    MX_TIM12_Init();
    Servos_Init();

    MX_TIM2_Init();
    MX_TIM3_Init();
    Encoders_Init();

    PID_Init();
    Odom_Init();
    Motion_Init();

    /* DMA clock must be up before the ADC MSP tries to HAL_DMA_Init on it. */
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM8_Init();
    IR_Init(&hadc1);
    Ultrasonic_Init(&htim8);

    MX_USART3_UART_Init();
    RpiLink_Init(&huart3);

    OLED_Init();
    OLED_Clear();

    /* IMU last, and before the tick starts. IMU_Init() blocks for about two
       and a half seconds measuring the zero-rate bias, and the robot must be
       STILL and on the ground for all of it. Tell the user so they do not
       pick the robot up while it is happening. */
    MX_I2C2_Init();
    ShowLine(0,  "IMU CALIBRATING");
    ShowLine(12, "HOLD STILL...");
    OLED_Refresh_Gram();

    if (IMU_Init(&hi2c2))
    {
        RpiLink_Log("IMU ok\r\n");
    }
    else
    {
        RpiLink_Log("IMU FAILED - check PB12 high, 1.8V rail\r\n");
    }
    OLED_Clear();

    RpiLink_Log("\r\n=== C30D FUNCTIONAL TEST BUILD ===\r\n");
    RpiLink_Log("LONG press = mode, SHORT = action\r\n");
    RpiLink_Log("1 CALIB 2 DRIVE 3 SETDIST 4 TURN 5 SETANGLE\r\n");
    RpiLink_Log("6 PROFILE 7 SERVO 8 SENSE 9 IRCAL 10 IMU 11 CAL\r\n\r\n");

    /* Tick LAST - nothing fires against an uninitialised module. */
    MX_TIM6_Init();
    HAL_TIM_Base_Start_IT(&htim6);

    while (1)
    {
        uint32_t now = HAL_GetTick();

        IMU_Poll();
        RpiLink_Poll();
        Calib_Tick();   /* after RpiLink_Poll(): the host's line wins */

        if (g_reportReady)
        {
            g_reportReady = 0U;
            Motion_ClearState();
            if (g_mode == MODE_DRIVE) { PrintDriveReport(); }
            if ((g_mode == MODE_TURN) && g_lastWasTurn)
            {
                g_turnValid = 1U;
                PrintTurnReport();
            }
        }

        if (g_evtLong)
        {
            g_evtLong  = 0U;
            g_evtShort = 0U;

            Cmd_QueueFlush();
            Motion_Stop();
            Motion_ClearState();
            Motors_Coast();

            /* Stopping the motors is not enough: the cycle would see an idle
               robot a moment later and launch its next step into a mode the
               operator has already walked away from. */
            Calib_Cancel();

            /* Leaving the sweep, put the steering back to centre. Walking
               away with the servo held against a mechanical stop stalls it
               and it will get hot. */
            if (g_mode == MODE_SERVO)
            {
                g_sweepUs   = SERVO_CENTER_US;
                g_sweepHeld = 0U;
                Servo_SetRawUs(SERVO_CENTER_US);
            }

            g_mode = (uimode_t)(((int)g_mode + 1) % (int)MODE_COUNT);
        }
        else if (g_evtShort)
        {
            g_evtShort = 0U;

            if (Motion_IsBusy())
            {
                Motion_Stop();
                Motion_ClearState();
                Motors_Coast();
                Calib_Cancel();   /* same reason as the long press */
                RpiLink_Log("STOP\r\n");
            }
            else if (g_mode == MODE_CALIB)
            {
                if (g_calState == CALIB_RUNNING)
                {
                    /* Press again to stop. This branch has to handle it
                       because a press lands in the 400 ms gap between steps
                       as often as not, and Motion_IsBusy() is false there -
                       so the STOP branch above never sees it and the cycle
                       would silently restart from step 1. */
                    Calib_Cancel();
                }
                else
                {
                    /* Re-pressing after DONE runs another cycle, which is the
                       intended way to use it: press, read the deltas, press
                       again, stop when they stop moving. */
                    Calib_Start();
                }
            }
            else if (g_mode == MODE_SETANGLE)
            {
                /* One button, three things to choose. Step the angle, and
                   roll over into the next direction when it wraps. */
                g_angleIdx++;
                if (g_angleIdx >= ANGLE_COUNT)
                {
                    g_angleIdx = 0U;
                    if (g_turnRight) { g_turnRight = 0U; }
                    else             { g_turnRight = 1U; g_turnFwd = !g_turnFwd; }
                }
            }
            else if (g_mode == MODE_TURN)
            {
                g_turnValid   = 0U;
                g_lastWasTurn = 1U;
                g_runStartA   = Encoder_A_GetCount();
                g_runStartB   = Encoder_B_GetCount();
                RpiLink_Log("\r\nA.4 turn starting\r\n");
                Motion_DriveArc(g_angleList[g_angleIdx], g_turnFwd, g_turnRight);
            }
            else if (g_mode == MODE_PROFILE)
            {
                char msg[64];
                uint8_t n = (uint8_t)((Motion_GetArcProfile() + 1U)
                                      % MOTION_ARC_PROFILE_COUNT);
                Motion_SetArcProfile(n);
                snprintf(msg, sizeof(msg), "arc profile -> %s\r\n",
                         Motion_GetArcProfileInfo(n)->name);
                RpiLink_Log(msg);
            }
            else if (g_mode == MODE_SERVO)
            {
                char msg[48];

                g_sweepUs += SERVO_SWEEP_STEP_US;
                if (g_sweepUs > SERVO_ABS_MAX_US)
                {
                    g_sweepUs = SERVO_ABS_MIN_US;
                }

                Servo_SetRawUs(g_sweepUs);
                g_sweepAtMs = HAL_GetTick();
                g_sweepHeld = 1U;

                snprintf(msg, sizeof(msg), "servo %u us (%+d)\r\n",
                         (unsigned)g_sweepUs,
                         (int)g_sweepUs - (int)SERVO_CENTER_US);
                RpiLink_Log(msg);
            }
            else if (g_mode == MODE_IMU)
            {
                IMU_ResetHeading();
                IMU_ResetStats();
                RpiLink_Log("IMU heading + stats zeroed\r\n");
            }
            else if (g_mode == MODE_CAL)
            {
                /* Zero the tally so one phase of a calibration run can be
                   read on its own. The learned values themselves are NOT
                   touched: they are what the host is in the middle of
                   setting, and clearing them from the button would be a way
                   to silently ruin a run while watching it. */
                RpiLink_ClearCalCounts();
                RpiLink_Log("CAL counters zeroed\r\n");
            }
            else if (g_mode == MODE_SETDIST)
            {
                g_targetMm += DIST_STEP_MM;
                if (g_targetMm > DIST_MAX_MM) { g_targetMm = DIST_MIN_MM; }
            }
            else if (g_mode == MODE_DRIVE)
            {
                /* Latch the target with the run. If it were read back at the
                   end instead, stepping the target afterwards would silently
                   rewrite the error figure for a run already finished. */
                g_repTarget   = g_targetMm;
                g_repValid    = 0U;
                g_lastWasTurn = 0U;
                g_runStartA   = Encoder_A_GetCount();
                g_runStartB   = Encoder_B_GetCount();

                RpiLink_Log("\r\nA.3 run starting\r\n");
                Motion_DriveDistance(g_targetMm);
            }
            else
            {
                PrintSensors();
            }
        }

        /* Sweep watchdog: never leave the servo stalled against a stop. */
        if (g_sweepHeld && ((now - g_sweepAtMs) >= SERVO_SWEEP_HOLD_MS))
        {
            g_sweepHeld = 0U;
            Servo_SetRawUs(SERVO_CENTER_US);
            RpiLink_Log("sweep timed out, centred\r\n");
        }

        if (now - tDisp >= 150u) { tDisp = now; Display(); }

        /* LED3 (PE8, active low): slow blink idle, fast blink running */
        if (now - tLed >= (Motion_IsBusy() ? 100u : 500u))
        {
            tLed = now;
            HAL_GPIO_TogglePin(GPIOE, LED3_Pin);
        }
    }
}

/* ==========================================================================
 *  Clocks
 *
 *  CubeMX 6.5.0 does NOT emit the voltage scale or the flash latency needed
 *  for 168 MHz on this part. Both are hand-added below and will be silently
 *  dropped again on the next code generation - check them every time.
 * ========================================================================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 8;      /* 8 MHz crystal, not the 25 MHz default */
    osc.PLL.PLLN       = 336;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;    /* PCLK1 42 MHz, timers 84 MHz  */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;    /* PCLK2 84 MHz, timers 168 MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* ==========================================================================
 *  Peripheral init. GPIO/AF/NVIC for every peripheral below is already
 *  handled by the generated stm32f4xx_hal_msp.c, so these only set registers.
 * ========================================================================== */

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Enabled so a transfer error is still reported. The half- and
       full-transfer interrupts are masked off in IR_Init(); see the comment
       there for why leaving them on wrecks the control tick. */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* ADC1 - IR left PC0 (IN10), IR right PC1 (IN11).
   Scan, continuous, circular DMA. Once started it never needs touching.

   480-cycle sampling is deliberate. The GP2Y0A21YK is a high-impedance
   source behind a 1k/2.2k divider, and the datasheet's RAIN limit means a
   short sample time would not let the ADC's hold capacitor charge fully -
   the reading would sag toward whichever channel was converted before it. */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef ch = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 2;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    ch.Channel      = ADC_CHANNEL_10;         /* PC0, IR left  */
    ch.Rank         = 1;
    ch.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();

    ch.Channel      = ADC_CHANNEL_11;         /* PC1, IR right */
    ch.Rank         = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();
}

/* TIM8 - HC-SR04 echo capture on PC7 (CH2), both edges.
   APB2 timer clock 168 MHz, PSC 167 -> 1 MHz, so one count is one
   microsecond and the echo width is already in the units the HC-SR04
   distance formula wants. ARR 65535 wraps every 65.5 ms, comfortably longer
   than the 38 ms the sensor takes to give up.

   ICFilter is 0 to match the .ioc. If the echo line picks up motor noise and
   the lost-echo count in SENSE mode climbs, raise it to 3 - that rejects
   glitches under ~50 ns and delays both edges equally, so the measured width
   is unaffected. */
static void MX_TIM8_Init(void)
{
    TIM_MasterConfigTypeDef mst = {0};
    TIM_IC_InitTypeDef      ic  = {0};

    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 168 - 1;
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 65535;
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* Base init first: the generated MSP hangs the TIM8 clock, the PC7 AF3
       pin config and the NVIC entry off HAL_TIM_Base_MspInit, not off the
       IC one. Calling only HAL_TIM_IC_Init gives a timer with no clock. */
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK) Error_Handler();
    if (HAL_TIM_IC_Init(&htim8) != HAL_OK) Error_Handler();

    mst.MasterOutputTrigger = TIM_TRGO_RESET;
    mst.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &mst) != HAL_OK) Error_Handler();

    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim8, &ic, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
}

/* TIM2 - encoder A, PA15 / PB3. 32-bit part, ARR forced to 65535 so the
   (int16_t) delta cast in encoders.c is valid. */
static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef enc = {0};
    TIM_MasterConfigTypeDef mst = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 65535;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = 10;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = 10;
    if (HAL_TIM_Encoder_Init(&htim2, &enc) != HAL_OK) Error_Handler();

    mst.MasterOutputTrigger = TIM_TRGO_RESET;
    mst.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &mst) != HAL_OK) Error_Handler();
}

/* TIM3 - encoder B, PB4 / PB5. */
static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef enc = {0};
    TIM_MasterConfigTypeDef mst = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = 10;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = 10;
    if (HAL_TIM_Encoder_Init(&htim3, &enc) != HAL_OK) Error_Handler();

    mst.MasterOutputTrigger = TIM_TRGO_RESET;
    mst.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &mst) != HAL_OK) Error_Handler();
}

/* TIM4 - motor A PWM. APB1 timer clock 84 MHz, PSC 0, ARR 4199 -> 20 kHz. */
static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef mst = {0};
    TIM_OC_InitTypeDef      oc  = {0};

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = MOTOR_TIM_ARR;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();

    mst.MasterOutputTrigger = TIM_TRGO_RESET;
    mst.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &mst) != HAL_OK) Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim4);
}

/* TIM6 - control tick. 84 MHz / 8400 = 10 kHz, / 100 = 100 Hz = 10 ms. */
static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef mst = {0};

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 8399;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 99;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

    mst.MasterOutputTrigger = TIM_TRGO_RESET;
    mst.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &mst) != HAL_OK) Error_Handler();
}

/* TIM9 - motor B PWM. APB2 timer clock 168 MHz, PSC 1 -> 84 MHz,
   ARR 4199 -> 20 kHz, identical scale to TIM4. */
static void MX_TIM9_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    htim9.Instance               = TIM9;
    htim9.Init.Prescaler         = 1;
    htim9.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim9.Init.Period            = MOTOR_TIM_ARR;
    htim9.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim9);
}

/* TIM12 - servo. APB1 timer clock 84 MHz, PSC 83 -> 1 MHz (1 us/count),
   ARR 19999 -> 50 Hz. NOTE: PSC is 83, not 167. TIM12 is on APB1. */
static void MX_TIM12_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    htim12.Instance               = TIM12;
    htim12.Init.Prescaler         = 83;
    htim12.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim12.Init.Period            = 19999;
    htim12.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = SERVO_CENTER_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim12, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim12);
}

/* I2C2 - ICM-20948 gyro on PB10 (SCL) and PB11 (SDA), AF4.
 *
 * The pin setup is done HERE rather than in a HAL_I2C_MspInit(). I2C2 is not
 * in the .ioc, so the generated stm32f4xx_hal_msp.c has no I2C handler and
 * HAL_I2C_Init() will call the empty __weak one. Defining our own MspInit in
 * this file would work today and collide the day anyone regenerates from
 * CubeMX with I2C enabled. Doing it inline avoids that entirely.
 *
 * Open drain with the internal pull-ups on. The board's visible pull-ups sit
 * on the 1.8 V side of the RS0102 level shifter, so they do NOT pull up this
 * side of the bus. Without GPIO_PULLUP here the lines never rise and every
 * transaction times out. */
static void MX_I2C2_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &g);

    /* PB12 = nCS on the ICM-20948. HIGH selects I2C; low leaves the part in
       SPI mode where it ignores the bus completely. IMU_Init() drives it too,
       but set it here so it is high before the first transaction. */
    g.Pin   = GPIO_PIN_12;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    __HAL_RCC_I2C2_CLK_ENABLE();

    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 400000;          /* fast mode */
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = CMD_BAUD_RATE;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

/* Only the pins the MSP does not own: OLED bit-bang, LED3, user button,
   and the ultrasonic trigger. PC0/PC1 (analog) and PC7 (AF3) are configured
   by HAL_ADC_MspInit and HAL_TIM_Base_MspInit respectively. */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* OLED: PD11 DC, PD12 RES, PD13 SDA, PD14 SCL */
    g.Pin   = OLED_DC_Pin | OLED_RES_Pin | OLED_SDA_Pin | OLED_SCL_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &g);
    HAL_GPIO_WritePin(GPIOD, g.Pin, GPIO_PIN_RESET);

    /* LED3 PE8, active low -> start off */
    g.Pin   = LED3_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &g);
    HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_SET);

    /* User button PE0, active low, external 10k pull-up already fitted */
    g.Pin  = GPIO_PIN_0;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &g);

    /* Ultrasonic trigger PB14, starts low so the sensor cannot chirp before
       Ultrasonic_Init() owns it. */
    g.Pin   = US_Trig_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, US_Trig_Pin, GPIO_PIN_RESET);

    /* Servo signal PB15, held low until MX_TIM12_Init() hands it to the timer.
       A floating input on the servo makes it twitch and draw stall current,
       which the board's care notes list as a way to damage the STM32. This
       covers the window from reset to TIM12; the vendor's suggested 2.2k
       pull-down covers the window before the MCU is running at all. */
    g.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}

void Error_Handler(void)
{
    /* Kill the motors before parking. An Error_Handler that spins with the
       bridges still driven is how a bench test becomes a chase. */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);

    __disable_irq();
    while (1) { }
}

void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
