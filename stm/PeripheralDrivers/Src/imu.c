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
static int16_t  s_bias;
static volatile float    s_rateDps;
static volatile float    s_headingDeg;
static volatile uint32_t s_errors;

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
    s_bias       = 0;
    s_rateDps    = 0.0f;
    s_headingDeg = 0.0f;
    s_errors     = 0U;
    s_lastGoodMs = HAL_GetTick();
    s_stalls     = 0U;

    /* nCS high selects I2C. Without this the part sits in SPI mode and will
     * not acknowledge at any address. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(50);

    for (attempt = 0U; attempt < IMU_INIT_RETRIES; attempt++)
    {
        if (imu_bringup())
        {
            s_ready = 1U;
            break;
        }
        HAL_Delay(100);
    }

    if (!s_ready) { return 0U; }

    if (!IMU_CalibrateBias())
    {
        s_ready = 0U;
        return 0U;
    }

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

    s_bias = 0;

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

    sum /= (int32_t)got;

    /* If this trips, the robot was moved during calibration. Refusing is
     * better than silently baking a motion artefact into every future
     * reading. */
    if ((sum > IMU_BIAS_SANITY_LSB) || (sum < -IMU_BIAS_SANITY_LSB))
    {
        return 0U;
    }

    s_bias = (int16_t)sum;
    return 1U;
}

/* ------------------------------------------------------------------ */
/* Runtime                                                             */
/* ------------------------------------------------------------------ */

void IMU_Poll(void)
{
    uint8_t buf[2];
    int16_t raw;

    if (!s_ready) { return; }

    if (!reg_read(B0_GYRO_ZOUT_H, buf, 2U))
    {
        /* Deliberately do NOT touch s_rateDps here - IMU_Tick() decides what
         * to do about a gap, based on how long it has been going on. */
        return;
    }

    raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);

    s_rawZ       = raw;
    s_rateDps    = (float)(raw - s_bias) / IMU_GYRO_LSB_PER_DPS;
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

    /* Staleness check FIRST. A frozen rate integrated forever is worse than
     * no heading at all: it is confidently wrong, and everything downstream -
     * heading hold, cross-track, arc termination - acts on it. */
    if ((HAL_GetTick() - s_lastGoodMs) > IMU_STALE_MS)
    {
        s_rateDps = 0.0f;   /* stop integrating garbage */
        s_ready   = 0U;     /* odom falls back to the encoders */
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
    /* s_bias is an integer count, so a plain EMA would never move for small
     * errors - the update would round to zero every time. Accumulate the
     * error in a wider running sum instead and shift a count across only when
     * the sum has genuinely built up. */
    static int32_t accum = 0;

    if (!s_ready) { return; }

    accum += ((int32_t)s_rawZ - (int32_t)s_bias);

    if (accum > 1024)
    {
        s_bias++;
        accum = 0;
    }
    else if (accum < -1024)
    {
        s_bias--;
        accum = 0;
    }
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
uint8_t  IMU_GetAddress(void)    { return s_addr; }
uint8_t  IMU_GetWhoAmI(void)     { return s_whoami; }
int16_t  IMU_GetBias(void)       { return s_bias; }
int16_t  IMU_GetRawZ(void)       { return s_rawZ; }
uint32_t IMU_GetErrorCount(void) { return s_errors; }
uint32_t IMU_GetStallCount(void)  { return s_stalls; }
