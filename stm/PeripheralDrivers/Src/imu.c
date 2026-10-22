#include "imu.h"
#include "main.h"

/* ---------------------------------------------------------------------------
 * ICM-20948 register map.
 *
 * The registers are BANKED. Address 0x7F selects the bank and is the only
 * register visible from every bank; everything else means different things
 * depending on which bank is selected. Forgetting a bank switch is the single
 * most common way to get this part half-working - you configure a register in
 * the wrong bank, it silently writes to something unrelated, and the gyro
 * comes back at the wrong scale or not at all.
 * ------------------------------------------------------------------------- */
#define REG_BANK_SEL        0x7F

/* Bank 0 */
#define B0_WHO_AM_I         0x00
#define B0_PWR_MGMT_1       0x06
#define B0_PWR_MGMT_2       0x07
#define B0_GYRO_ZOUT_H      0x37

/* Bank 2 */
#define B2_GYRO_SMPLRT_DIV  0x00
#define B2_GYRO_CONFIG_1    0x01

#define WHO_AM_I_EXPECTED   0xEA

#define PWR1_DEVICE_RESET   0x80
#define PWR1_SLEEP          0x40
#define PWR1_CLKSEL_AUTO    0x01

#define I2C_TIMEOUT_MS      10U

static I2C_HandleTypeDef *s_i2c;
static uint8_t  s_addr;             /* 7-bit, 0 until something answers */
static uint8_t  s_whoami;
static uint8_t  s_ready;
static uint8_t  s_bank = 0xFF;      /* forces the first select to happen  */

static volatile int16_t  s_rawZ;
static volatile float    s_bias;  /* raw counts, sub-count precision */
static volatile float    s_rateDps;
static volatile float    s_headingDeg;
static volatile uint32_t s_errors;

/* Measured correction on the datasheet sensitivity - see imu.h. RAM only,
 * restored by the RPi with !CALG like the other calibration values. */
static float    s_gyroScale = 1.0f;

/* Poll rate diagnostics. IMU_Tick() integrates at a fixed 100 Hz using
 * whatever sample IMU_Poll() last fetched, so if the main loop is not
 * polling at least as fast as the sensor updates (102 Hz), samples go
 * unseen - and during a fast rotation the unseen ones are exactly the
 * large ones. This counts what is actually happening. */
static volatile uint32_t s_polls;
static volatile uint32_t s_pollRate;
static volatile int16_t  s_peakRaw;
static volatile uint32_t s_lastGoodMs;
static volatile uint32_t s_stalls;

/* Large-offset re-baseline - see IMU_BIAS_STEADY_LSB. s_tickNo counts
 * IMU_Tick() calls so IMU_TrackBias() can tell it was called on consecutive
 * ticks: a gap means the robot moved in between and the run starts over. */
static uint32_t s_tickNo;
static uint32_t s_trackTick;
static float    s_steadyMean;
static uint16_t s_steadyTicks;

/* Re-arming - see IMU_REARM_STILL_TICKS. s_configured: the part has been
 * through imu_bringup() since the bus last failed. s_biasValid: s_bias came
 * from a proper at-rest measurement, at boot or since. Both survive a stall,
 * so a dropout mid-run only needs the bus back, not a new bias. */
static volatile uint8_t  s_configured;
static uint8_t           s_biasValid;
static volatile uint16_t s_stillTicks;
static uint32_t s_recoveries;
static uint32_t s_lastRecoverMs;
static uint32_t s_rearmLastMs;
static uint16_t s_rearmGood;
static int32_t  s_rearmSum;
static uint16_t s_rearmGot;

/* ------------------------------------------------------------------ */
/* Register access                                                     */
/* ------------------------------------------------------------------ */

static uint8_t reg_write(uint8_t reg, uint8_t val)
{
    if (HAL_I2C_Mem_Write(s_i2c, (uint16_t)(s_addr << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, &val, 1U,
                          I2C_TIMEOUT_MS) != HAL_OK)
    {
        s_errors++;
        return 0U;
    }
    return 1U;
}

static uint8_t reg_read(uint8_t reg, uint8_t *buf, uint16_t n)
{
    if (HAL_I2C_Mem_Read(s_i2c, (uint16_t)(s_addr << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, n,
                         I2C_TIMEOUT_MS) != HAL_OK)
    {
        s_errors++;
        return 0U;
    }
    return 1U;
}

/* Bank number goes in bits [5:4], everything else must be zero. Cached so a
 * repeated select costs nothing on the bus. */
static uint8_t bank_select(uint8_t bank)
{
    if (bank == s_bank) { return 1U; }

    if (!reg_write(REG_BANK_SEL, (uint8_t)(bank << 4))) { return 0U; }

    s_bank = bank;
    return 1U;
}

/* ------------------------------------------------------------------ */
/* Bus recovery                                                        */
/* ------------------------------------------------------------------ */

/* ~5 us at 168 MHz. Only needs to be slow enough for the RS0102 shifter and
 * the internal pull-ups to swing the lines; the exact rate does not matter. */
static void bus_delay(void)
{
    volatile uint32_t n = SystemCoreClock / 1000000U;
    while (n--) { }
}

/* Free a stuck I2C2 and give the peripheral a clean start.
 *
 * Two ways the bus wedges, and neither clears itself:
 *
 *   The SENSOR holds SDA low. An MCU-only reset (RESET button, debugger, a
 *   brown-out the 1.8 V rail rides through) can land mid-read, with the
 *   ICM-20948 halfway through shifting a byte out. It keeps waiting for
 *   clocks that never come and holds SDA low forever. Every START then
 *   fails and the part looks dead until a full power cycle.
 *
 *   The PERIPHERAL wedges. A glitch on the lines - motor noise is the usual
 *   source - can leave the STM32F4 I2C block with BUSY set and no transfer
 *   in progress (a known STM32F4 I2C erratum). HAL_I2C_* then returns
 *   HAL_BUSY on every call.
 *
 * The cure for the first is up to nine clocks on SCL, as GPIO, until the
 * sensor finishes its byte and lets SDA go, then a STOP. The cure for the
 * second is an RCC reset of the block and a fresh HAL_I2C_Init(). Do both,
 * every time: it costs well under a millisecond.
 *
 * The pin setup mirrors MX_I2C2_Init() in main.c - see there for why it is
 * not in an MspInit. */
static void i2c_bus_recover(void)
{
    GPIO_InitTypeDef g = {0};
    uint8_t i;

    if ((s_i2c == 0) || (s_i2c->Instance != I2C2)) { return; }

    (void)HAL_I2C_DeInit(s_i2c);

    /* PB10 SCL, PB11 SDA as plain open-drain outputs, both released. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    g.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    bus_delay();

    /* Clock until the sensor lets go of SDA. Nine covers a whole byte plus
     * its ACK slot. */
    for (i = 0U; i < 9U; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) { break; }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        bus_delay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        bus_delay();
    }

    /* STOP: SDA rising while SCL is high. Leaves the sensor idle. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    bus_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    bus_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    bus_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    bus_delay();

    /* Back to the I2C alternate function, as main.c sets it up. */
    g.Mode      = GPIO_MODE_AF_OD;
    g.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &g);

    /* Clear a stuck BUSY the only reliable way, then start the block over. */
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();
    __HAL_UNLOCK(s_i2c);
    s_i2c->State = HAL_I2C_STATE_RESET;
    (void)HAL_I2C_Init(s_i2c);

    /* The sensor may have lost its bank pointer along with everything else. */
    s_bank = 0xFF;
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

static uint8_t probe(uint8_t addr)
{
    /* Two tries: the first transaction after power-up is often NAKed while
     * the part is still bringing its regulator up. */
    if (HAL_I2C_IsDeviceReady(s_i2c, (uint16_t)(addr << 1), 2U, 20U) == HAL_OK)
    {
        return 1U;
    }
    return 0U;
}

/* One bring-up attempt: find the part, reset it, configure it. Split out of
 * IMU_Init() so the whole sequence can simply be retried.
 *
 * Retrying matters because of two things that are only true on a warm start.
 * The 1.8 V rail feeding the sensor comes from U20 and may still be rising
 * when the MCU has already finished booting - the MCU runs from a different
 * regulator with no ordering between them. And after an MCU-only reset (the
 * RESET button, or the debugger) the ICM-20948 is NOT reset: it keeps
 * whatever state it was left in, possibly mid-DEVICE_RESET, and NAKs until
 * it finishes. A cold power cycle hides both; a warm one exposes them. */
static uint8_t imu_bringup(void)
{
    uint8_t v;

    s_addr = 0U;
    s_bank = 0xFF;

    /* Probe both possible addresses rather than assuming what AD0 is
     * strapped to. */
    if      (probe(0x68U)) { s_addr = 0x68U; }
    else if (probe(0x69U)) { s_addr = 0x69U; }
    else                   { return 0U; }

    /* Full reset, then wake. The datasheet asks for 100 ms after a reset
     * before the part will accept configuration. */
    if (!bank_select(0U))                                  { return 0U; }
    if (!reg_write(B0_PWR_MGMT_1, PWR1_DEVICE_RESET))      { return 0U; }
    HAL_Delay(100);

    /* The reset also resets the bank pointer, so drop the cache. */
    s_bank = 0xFF;
    if (!bank_select(0U))                                  { return 0U; }

    /* Clear SLEEP and take the best available clock. Leaving CLKSEL at 0
     * runs the part off its internal 20 MHz oscillator, which is specified
     * as less stable than the auto-select option. */
    if (!reg_write(B0_PWR_MGMT_1, PWR1_CLKSEL_AUTO))       { return 0U; }
    HAL_Delay(20);

    if (!reg_read(B0_WHO_AM_I, &v, 1U))                    { return 0U; }
    s_whoami = v;
    if (v != WHO_AM_I_EXPECTED)                            { return 0U; }

    /* Both accel and gyro on. The accel costs nothing here and leaves the
     * door open for a level check later. */
    if (!reg_write(B0_PWR_MGMT_2, 0x00U))                  { return 0U; }

    if (!bank_select(2U))                                  { return 0U; }

    /* Output data rate = 1125 / (1 + div). 10 gives 102.3 Hz, which sits
     * just above the 100 Hz control tick so every tick sees a fresh sample
     * without the two rates beating against each other. */
    if (!reg_write(B2_GYRO_SMPLRT_DIV, 10U))               { return 0U; }

    /* GYRO_CONFIG_1:
     *   bit    0  GYRO_FCHOICE  1 = enable the low-pass filter
     *   bits 2:1  GYRO_FS_SEL   full scale
     *   bits 5:3  GYRO_DLPFCFG  bandwidth
     *
     * See imu.h for why the full scale is +-1000 and not +-250. */
    if (!reg_write(B2_GYRO_CONFIG_1,
                   (uint8_t)((IMU_GYRO_DLPFCFG << 3) |
                             (IMU_GYRO_FS_SEL  << 1) | 1U)))  { return 0U; }

    if (!bank_select(0U))                                  { return 0U; }
    HAL_Delay(50);

    return 1U;
}

uint8_t IMU_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t attempt;

    s_i2c        = hi2c;
    s_addr       = 0U;
    s_whoami     = 0U;
    s_ready      = 0U;
    s_bank       = 0xFF;
    s_rawZ       = 0;
    s_bias       = 0.0f;
    s_rateDps    = 0.0f;
    s_headingDeg = 0.0f;
    s_errors     = 0U;
    s_lastGoodMs = HAL_GetTick();
    s_stalls     = 0U;
    s_configured = 0U;
    s_biasValid  = 0U;
    s_recoveries = 0U;
    s_stillTicks = 0U;
    s_lastRecoverMs = HAL_GetTick();

    /* nCS high selects I2C. Without this the part sits in SPI mode and will
     * not acknowledge at any address. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(50);

    /* Recover the bus before EVERY attempt, the first included: a warm
     * reset is exactly when the sensor is most likely to be holding SDA, and
     * retrying a bring-up against a held bus only fails the same way again. */
    for (attempt = 0U; attempt < IMU_INIT_RETRIES; attempt++)
    {
        i2c_bus_recover();
        if (imu_bringup())
        {
            s_configured = 1U;
            s_ready      = 1U;
            break;
        }
        HAL_Delay(100);
    }

    /* Either failure below leaves the gyro off for now, not for good:
     * IMU_Poll() keeps trying to bring it up and measure its bias while the
     * robot stands still - see IMU_REARM_STILL_TICKS. */
    if (!s_ready) { return 0U; }

    if (!IMU_CalibrateBias())
    {
        s_ready = 0U;
        return 0U;
    }
    s_biasValid = 1U;

    /* Start the staleness clock from HERE, not from the top of init. The
     * control tick begins right after this returns and checks staleness on
     * its very first pass, possibly before the main loop has managed a single
     * IMU_Poll(). Anything older than this moment is startup, not a fault. */
    s_lastGoodMs = HAL_GetTick();

    return 1U;
}

/* ------------------------------------------------------------------ */
/* Bias                                                                */
/* ------------------------------------------------------------------ */

uint8_t IMU_CalibrateBias(void)
{
    int32_t  sum = 0;
    uint16_t got = 0U;
    uint16_t i;
    uint8_t  buf[2];

    if (!s_ready) { return 0U; }

    s_bias = 0.0f;

    /* A gyro at rest does not read zero - it reads its own bias, and that
     * bias is what gets integrated into a growing heading error if it is not
     * removed. Measuring it is the single most important thing here. */
    for (i = 0U; i < IMU_BIAS_SAMPLES; i++)
    {
        if (reg_read(B0_GYRO_ZOUT_H, buf, 2U))
        {
            sum += (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
            got++;

            /* Keep the staleness timer alive. This loop reads the sensor
             * directly rather than through IMU_Poll(), so without this the
             * 2.5 seconds spent here look like 2.5 seconds of no data and the
             * gyro is declared stale before it has ever been polled. */
            s_lastGoodMs = HAL_GetTick();
        }
        HAL_Delay(10);
    }

    /* Demand most of the samples. A handful of I2C failures is survivable;
     * a bias averaged over a quarter of the intended samples is not. */
    if (got < (IMU_BIAS_SAMPLES / 2U)) { return 0U; }

    /* Float mean: 200 samples resolve the bias well below one count, and an
     * integer divide would throw that away. */
    {
        float mean = (float)sum / (float)got;

        /* If this trips, the robot was moved during calibration. Refusing is
         * better than silently baking a motion artefact into every future
         * reading. */
        if ((mean > (float)IMU_BIAS_SANITY_LSB) || (mean < -(float)IMU_BIAS_SANITY_LSB))
        {
            return 0U;
        }

        s_bias = mean;
    }
    return 1U;
}

/* ------------------------------------------------------------------ */
/* Re-arming                                                           */
/* ------------------------------------------------------------------ */

/* Called from IMU_Poll() while the gyro is off. Brings it back in three
 * steps, all only while the robot has been standing still:
 *
 *   1. Bus recovery and a full bring-up, at most once per
 *      IMU_REARM_PERIOD_MS - so a sensor that is really gone costs a few
 *      milliseconds a second, not the whole main loop.
 *   2. If the bias was never measured (boot calibration failed), measure it
 *      here, one sample per 10 ms, without blocking. Any movement starts it
 *      over, exactly as the boot calibration would want.
 *   3. IMU_REARM_GOOD_READS clean reads in a row, then ready again.
 *
 * Heading is NOT touched. IMU_Tick() stopped integrating when the gyro went
 * stale, so IMU_GetHeading() still holds the value Odom last differenced;
 * the first tick after re-arming adds nothing, and the encoder-integrated
 * pose carries straight across the gap. */
static void imu_rearm(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  buf[2];
    int16_t  raw;

    if (s_stillTicks < IMU_REARM_STILL_TICKS)
    {
        /* Moving, or not still for long enough: nothing half-done counts. */
        s_rearmGood = 0U;
        s_rearmSum  = 0;
        s_rearmGot  = 0U;
        return;
    }

    if (!s_configured)
    {
        if ((now - s_lastRecoverMs) < IMU_REARM_PERIOD_MS) { return; }
        s_lastRecoverMs = now;
        s_recoveries++;

        i2c_bus_recover();
        s_configured = imu_bringup();

        s_rearmGood  = 0U;
        s_rearmSum   = 0;
        s_rearmGot   = 0U;
        s_rearmLastMs = HAL_GetTick();
        return;
    }

    /* The part samples at ~102 Hz; reading faster only repeats samples. */
    if ((now - s_rearmLastMs) < 10U) { return; }
    s_rearmLastMs = now;

    if (!reg_read(B0_GYRO_ZOUT_H, buf, 2U))
    {
        /* Still failing: back to step 1 on the next period. */
        s_configured = 0U;
        return;
    }
    raw    = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    s_rawZ = raw;

    if (!s_biasValid)
    {
        s_rearmSum += raw;
        s_rearmGot++;
        if (s_rearmGot < IMU_BIAS_SAMPLES) { return; }

        {
            float mean = (float)s_rearmSum / (float)s_rearmGot;

            s_rearmSum = 0;
            s_rearmGot = 0U;

            /* Same sanity limit as IMU_CalibrateBias(): past it the robot
             * was being turned, so measure again. */
            if ((mean > (float)IMU_BIAS_SANITY_LSB) ||
                (mean < -(float)IMU_BIAS_SANITY_LSB))
            {
                return;
            }
            s_bias      = mean;
            s_biasValid = 1U;
            s_rearmGood = IMU_REARM_GOOD_READS;  /* 200 reads just proved it */
        }
    }
    else
    {
        s_rearmGood++;
    }

    if (s_rearmGood >= IMU_REARM_GOOD_READS)
    {
        /* Staleness clock first, so the very next tick does not see the old
         * timestamp and drop the gyro straight back out. */
        s_rateDps    = 0.0f;
        s_lastGoodMs = HAL_GetTick();
        s_steadyTicks = 0U;
        s_rearmGood  = 0U;
        s_ready      = 1U;
    }
}

/* ------------------------------------------------------------------ */
/* Runtime                                                             */
/* ------------------------------------------------------------------ */

void IMU_Poll(void)
{
    uint8_t buf[2];
    int16_t raw;

    if (!s_ready)
    {
        /* s_i2c is set by IMU_Init(); nothing to recover before that. */
        if (s_i2c != 0) { imu_rearm(); }
        return;
    }

    if (!reg_read(B0_GYRO_ZOUT_H, buf, 2U))
    {
        /* Deliberately do NOT touch s_rateDps here - IMU_Tick() decides what
         * to do about a gap, based on how long it has been going on. */
        return;
    }

    raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);

    s_rawZ       = raw;
    s_rateDps    = (((float)raw - s_bias) / IMU_GYRO_LSB_PER_DPS) * s_gyroScale;
    s_lastGoodMs = HAL_GetTick();

    s_polls++;

    /* Largest magnitude seen since the last reset. If this ever approaches
     * 32767 the gyro is clipping at its full-scale limit and every reading
     * above it is lost - which would under-report a fast turn exactly the
     * way a scale error does. */
    {
        int16_t mag = (raw < 0) ? (int16_t)(-raw) : raw;
        if (mag > s_peakRaw) { s_peakRaw = mag; }
    }
}

void IMU_ResetStats(void)
{
    s_polls    = 0U;
    s_pollRate = 0U;
    s_peakRaw  = 0;
}

uint32_t IMU_GetPollRate(void) { return s_pollRate; }
int16_t  IMU_GetPeakRaw(void)  { return s_peakRaw; }

void IMU_Tick(void)
{
    static uint16_t ticks = 0U;

    if (!s_ready) { return; }

    s_tickNo++;

    /* Staleness check FIRST. A frozen rate integrated forever is worse than
     * no heading at all: it is confidently wrong, and everything downstream -
     * heading hold, cross-track, arc termination - acts on it. */
    if ((HAL_GetTick() - s_lastGoodMs) > IMU_STALE_MS)
    {
        s_rateDps    = 0.0f;   /* stop integrating garbage */
        s_ready      = 0U;     /* odom falls back to the encoders */
        s_configured = 0U;     /* IMU_Poll() recovers the bus and re-arms */
        s_stalls++;
        return;
    }

    /* Once a second, latch how many polls happened. Should be well above
     * 102; anywhere near or below it means the main loop is the bottleneck. */
    ticks++;
    if (ticks >= 100U)
    {
        ticks      = 0U;
        s_pollRate = s_polls;
        s_polls    = 0U;
    }

    /* Rectangular integration at a fixed step. Trapezoidal would be more
     * accurate in principle, but the sample rate is barely above the tick
     * rate so there is no real intermediate value to interpolate towards -
     * it would just add arithmetic for no gain.
     *
     * No deadband on the rate. A deadband would stop small genuine turns
     * from registering, and the whole point of measuring the bias properly
     * is that it makes one unnecessary. */
    s_headingDeg += s_rateDps * IMU_DT_S * (float)IMU_Z_SIGN;
}

void IMU_TrackBias(void)
{
    float err;

    if (!s_ready) { return; }

    err = (float)s_rawZ - s_bias;

    /* Still wheels, turning body: lifted, or twisted by hand on the floor.
     * That is rotation, not bias - see IMU_BIAS_TRACK_GATE_LSB. Unless it
     * holds dead steady for seconds, which a hand cannot: then it is a bias
     * that was wrong from power-on - see IMU_BIAS_STEADY_LSB. */
    if ((err > IMU_BIAS_TRACK_GATE_LSB) || (err < -IMU_BIAS_TRACK_GATE_LSB))
    {
        float raw = (float)s_rawZ;
        float dev;

        if ((s_steadyTicks == 0U) || (s_tickNo != (s_trackTick + 1U)))
        {
            s_steadyMean  = raw;          /* (re)start the run here */
            s_steadyTicks = 0U;
        }
        s_trackTick = s_tickNo;

        dev = raw - s_steadyMean;
        if ((dev > IMU_BIAS_STEADY_LSB) || (dev < -IMU_BIAS_STEADY_LSB))
        {
            s_steadyTicks = 0U;           /* moving: not an offset */
            return;
        }

        s_steadyMean += dev / 16.0f;
        s_steadyTicks++;
        if (s_steadyTicks >= IMU_BIAS_RESET_TICKS)
        {
            s_bias        = s_steadyMean;
            s_steadyTicks = 0U;
        }
        return;
    }
    s_steadyTicks = 0U;

    /* First-order low-pass toward the stationary reading, ~10 s time
     * constant. The old integer version could only step a whole count at a
     * time and dithered a count either side of the true bias. */
    s_bias += err / IMU_BIAS_TRACK_TICKS;
}

void IMU_ResetHeading(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_headingDeg = 0.0f;
    __set_PRIMASK(primask);
}

/* ------------------------------------------------------------------ */

float    IMU_GetHeading(void)    { return s_headingDeg; }
float    IMU_GetRateDps(void)    { return s_rateDps; }
uint8_t  IMU_IsReady(void)       { return s_ready; }

uint8_t IMU_SetGyroScale(float scale)
{
    if ((scale < IMU_GYRO_SCALE_MIN) || (scale > IMU_GYRO_SCALE_MAX))
    {
        return 0U;
    }
    s_gyroScale = scale;
    return 1U;
}

float IMU_GetGyroScale(void) { return s_gyroScale; }
uint8_t  IMU_GetAddress(void)    { return s_addr; }
uint8_t  IMU_GetWhoAmI(void)     { return s_whoami; }
int16_t  IMU_GetBias(void)
{
    float b = s_bias;
    return (int16_t)((b >= 0.0f) ? (b + 0.5f) : (b - 0.5f));
}
int16_t  IMU_GetRawZ(void)       { return s_rawZ; }
uint32_t IMU_GetErrorCount(void) { return s_errors; }
uint32_t IMU_GetStallCount(void)  { return s_stalls; }
uint32_t IMU_GetRecoveryCount(void) { return s_recoveries; }

void IMU_NoteStill(uint8_t still)
{
    if (!still)
    {
        s_stillTicks = 0U;
    }
    else if (s_stillTicks < 0xFFFFU)
    {
        s_stillTicks++;
    }
}
