#include "ultrasonic.h"
#include "commands.h"
#include "main.h"

static TIM_HandleTypeDef *s_tim;

/* Echo state machine. Driven from two contexts - the control tick arms it,
 * the capture ISR advances it - so everything shared is volatile. */
typedef enum
{
    US_IDLE = 0,        /* nothing in flight                        */
    US_WAIT_RISE,       /* triggered, waiting for the echo to start */
    US_WAIT_FALL        /* echo started, waiting for it to end      */
} UsState_t;

static volatile UsState_t s_state;
static volatile uint16_t  s_riseCap;
static volatile uint16_t  s_lastUs;
static volatile uint32_t  s_echoCount;
static volatile uint8_t   s_newSample;

static uint16_t s_med[3];
static uint8_t  s_medIdx;
static uint8_t  s_medFill;
static uint16_t s_distanceCm = SENSOR_NO_READING;

static uint16_t s_sinceTrig;      /* ticks since the last trigger  */
static uint16_t s_sinceReading;   /* ticks since the last good one */

/* ------------------------------------------------------------------ */

void Ultrasonic_Init(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef g = {0};

    s_tim          = htim;
    s_state        = US_IDLE;
    s_riseCap      = 0U;
    s_lastUs       = 0U;
    s_echoCount    = 0U;
    s_newSample    = 0U;
    s_medIdx       = 0U;
    s_medFill      = 0U;
    s_distanceCm   = SENSOR_NO_READING;
    s_sinceTrig    = 0U;
    s_sinceReading = US_STALE_TICKS;

    /* Trigger pin, parked low. MX_GPIO_Init() already did this, but doing it
     * here too means the driver is correct on its own and does not depend on
     * the order the inits happen to run in. */
    g.Pin   = US_Trig_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(US_Trig_GPIO_Port, &g);
    HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);

    /* Re-init the echo pin with a pull-down. The generated MSP configures it
     * as AF with GPIO_NOPULL, which leaves the line floating when no sensor
     * is plugged in - it then picks up switching noise from the motor bridges
     * and produces a stream of plausible-looking garbage distances. A
     * pull-down makes "not connected" read as "no echo", which is the truth. */
    g.Pin       = US_Echo_Pin;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLDOWN;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(US_Echo_GPIO_Port, &g);

    HAL_TIM_Base_Start(s_tim);
    HAL_TIM_IC_Start_IT(s_tim, TIM_CHANNEL_2);
}

/* Exactly 10 us of trigger, timed off TIM8's own 1 MHz counter rather than a
 * cycle-counted delay loop. The datasheet minimum is 10 us; a short pulse is
 * ignored and looks exactly like a dead sensor. */
static void trigger_pulse(void)
{
    uint16_t start;

    HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_SET);

    start = (uint16_t)__HAL_TIM_GET_COUNTER(s_tim);
    while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(s_tim) - start) < 11U)
    {
        /* ~11 us. Bounded and short enough to sit in the control tick. */
    }

    HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
}

/* Median of three. One bad sample in three is the classic HC-SR04 failure -
 * a stray reflection off the floor or a table leg - and a mean would let it
 * drag the answer halfway to the spurious value. A median rejects it
 * outright. */
static uint16_t median3(const uint16_t *v)
{
    uint16_t a = v[0], b = v[1], c = v[2], t;

    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { b = a; }

    (void)c;
    return b;
}

void Ultrasonic_Tick(void)
{
    if (s_sinceTrig   < 0xFFFFU) { s_sinceTrig++;   }
    if (s_sinceReading < 0xFFFFU) { s_sinceReading++; }

    /* Fold a completed echo into the filter. Done here, in the tick, rather
     * than in the capture ISR - the ISR's only job is to grab the timestamps
     * and get out. */
    if (s_newSample)
    {
        uint16_t us;
        uint16_t cm;

        s_newSample = 0U;
        us = s_lastUs;

        cm = (uint16_t)((float)us / US_US_PER_CM);

        if ((cm >= US_MIN_CM) && (cm <= US_MAX_CM))
        {
            s_med[s_medIdx] = cm;
            s_medIdx = (uint8_t)((s_medIdx + 1U) % 3U);
            if (s_medFill < 3U) { s_medFill++; }

            if (s_medFill >= 3U)
            {
                s_distanceCm   = median3(s_med);
                s_sinceReading = 0U;
            }
            else
            {
                /* Not enough history to filter yet - use it raw so the
                 * display comes alive immediately during bring-up. */
                s_distanceCm   = cm;
                s_sinceReading = 0U;
            }
        }
    }

    /* Echo never came back. Re-arm rather than wait forever. */
    if ((s_state != US_IDLE) && (s_sinceTrig >= US_ECHO_TIMEOUT_TICKS))
    {
        s_state = US_IDLE;
    }

    /* Age out a reading nobody has refreshed. */
    if (s_sinceReading >= US_STALE_TICKS)
    {
        s_distanceCm = SENSOR_NO_READING;
        s_medFill    = 0U;
        s_medIdx     = 0U;
    }

    /* Time for the next ping. */
    if ((s_state == US_IDLE) && (s_sinceTrig >= US_PERIOD_TICKS))
    {
        s_sinceTrig = 0U;
        s_state     = US_WAIT_RISE;
        trigger_pulse();
    }
}

void Ultrasonic_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint16_t cap;
    uint8_t  high;

    if (htim->Instance != TIM8) { return; }
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2) { return; }

    cap = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    /* Which edge this was. Reading the pin is more robust than assuming the
     * edges strictly alternate: if one is ever missed - and they are, when a
     * burst is swallowed - a driver that just toggles a flag stays inverted
     * forever and every subsequent reading is nonsense. */
    high = (HAL_GPIO_ReadPin(US_Echo_GPIO_Port, US_Echo_Pin) == GPIO_PIN_SET) ? 1U : 0U;

    if (high)
    {
        if (s_state == US_WAIT_RISE)
        {
            s_riseCap = cap;
            s_state   = US_WAIT_FALL;
        }
    }
    else
    {
        if (s_state == US_WAIT_FALL)
        {
            /* Unsigned 16-bit subtraction handles the counter wrapping past
             * 65535 mid-echo with no special case. */
            uint16_t width = (uint16_t)(cap - s_riseCap);

            s_lastUs    = width;
            s_newSample = 1U;
            s_echoCount++;
            s_state     = US_IDLE;
        }
    }
}

uint16_t Ultrasonic_GetCm(void)      { return s_distanceCm; }
uint16_t Ultrasonic_GetLastUs(void)  { return s_lastUs;     }
uint32_t Ultrasonic_GetEchoCount(void) { return s_echoCount; }

/* ===================================================================
 * Strong definitions of the Sensors_* hooks from commands.h. These
 * override the weak stubs in rpilink.c automatically at link time.
 * =================================================================== */

uint16_t Sensors_FrontDistanceCm(void)
{
    return s_distanceCm;
}

uint8_t Sensors_ObstacleAhead(uint16_t stop_cm)
{
    uint16_t d = s_distanceCm;

    /* No reading is NOT "no obstacle". Returning 0 here would let F0 drive
     * confidently into a wall whenever the sensor drops out. Report an
     * obstacle so the robot stops, which is the safe way to be wrong. */
    if (d == SENSOR_NO_READING) { return 1U; }

    return (d <= stop_cm) ? 1U : 0U;
}
