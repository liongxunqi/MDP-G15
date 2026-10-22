#include "rpilink.h"
#include "motion.h"
#include "motors.h"
#include "pid.h"
#include "odom.h"
#include "encoders.h"
#include "imu.h"
#include "ir.h"
#include "ultrasonic.h"
#include "caltab.h"
#include "task2fw.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *s_uart;

/* Receive path ----------------------------------------------------- */
static uint8_t  s_rxByte;
static char     s_rxLine[CMD_LINE_MAX];    /* filled by the ISR          */
static uint16_t s_rxLen;
static char     s_pending[CMD_LINE_MAX];   /* handed to the main loop    */
static volatile uint8_t s_lineReady;
static volatile uint8_t s_overrun;

/* Executor --------------------------------------------------------- */
static uint8_t     s_lineActive;   /* a line is in flight, owes a reply */
static uint8_t     s_f0Active;     /* F0 running, poll the ultrasound   */
static CmdOpcode_t s_lastOp = CMD_NONE;

/* FU{n} approach. See the long note in rpilink.h for why this MEASURES and
 * drives rather than tripping on the sensor. */
typedef enum
{
    FU_OFF = 0,     /* no FU in flight                                */
    FU_SETTLE,      /* stopped, waiting for a lag-free reading         */
    FU_DRIVE        /* a closing move is running on the odometry       */
} FuState_t;

static FuState_t s_fuState;
static uint16_t  s_fuStopCm;    /* the gap the SENDER asked for         */
static uint8_t   s_fuPass;      /* closing moves used so far            */
static uint32_t  s_fuMark;      /* HAL_GetTick() when the settle began  */
static int32_t   s_fuLastMm;    /* signed length of the pass in flight  */

/* Set once a real command line has been parsed off the wire. From then on
 * RpiLink_Log() is silent - see the note in rpilink.h. */
static volatile uint8_t s_quiet;

/* How many times reception had to be re-armed. See the watchdog in
 * RpiLink_Poll(). Should stay at zero; anything else is line trouble. */
static volatile uint32_t s_rearms;

/* What the !CAL* setters have done. Written only from answer_immediate(),
 * which runs in RpiLink_Poll() on the main loop; read from Display() on the
 * same loop, so no guard is needed. See the note in rpilink.h. */
static uint32_t s_calOk;
static uint32_t s_calBusy;
static uint32_t s_calRange;

/* 0 none, 1 watchdog timeout, 2 wrong-way abort, 3 FU had no echo to aim at.
 * Latched when a primitive ends badly and held until the whole line has been
 * answered. */
static uint8_t s_lineFailed;

/* IR SNAPSHOT AND EDGE LATCH, rpilink.h. One tick's worth of side IR and
 * world pose. Written only by RpiLink_Tick() in the TIM6 ISR; read in the
 * main loop with interrupts masked so a reply never mixes two ticks. */
typedef struct
{
    uint16_t l_cm;
    uint16_t r_cm;
    uint16_t l_raw;
    uint16_t r_raw;
    int32_t  x_mm;
    int32_t  y_mm;
    int32_t  hdg_x10;
    uint32_t ms;
} IrSnap_t;

static volatile IrSnap_t s_irSnap;

/* Latch: 0 off, 1 armed, 2 latched. Side 1 left, 2 right. s_ireRun counts
 * consecutive in-window ticks; s_ireFirst holds the first of them. */
static volatile uint8_t  s_ireState;
static volatile uint8_t  s_ireSide;
static volatile uint16_t s_ireLo;
static volatile uint16_t s_ireHi;
static volatile uint8_t  s_ireRun;
static volatile IrSnap_t s_ireFirst;
static volatile IrSnap_t s_ireHit;

/* ===================================================================
 * Weak sensor stubs.
 *
 * These let the whole firmware link and run the checklist before any
 * sensor driver exists. Person B's strong definitions override them
 * automatically at link time - no #ifdef, no coordination.
 *
 * They are deliberately pessimistic: no reading, no obstacle, no valid
 * heading. Nothing downstream should ever mistake a stub for data.
 * =================================================================== */

__attribute__((weak)) uint16_t Sensors_FrontDistanceCm(void)
{
    return SENSOR_NO_READING;
}

__attribute__((weak)) uint8_t Sensors_ObstacleAhead(uint16_t stop_cm)
{
    (void)stop_cm;
    return 0U;
}

__attribute__((weak)) float Sensors_HeadingDeg(void)
{
    return 0.0f;
}

__attribute__((weak)) uint8_t Sensors_HeadingValid(void)
{
    return 0U;
}

/* ------------------------------------------------------------------ */
/* Transmit                                                            */
/* ------------------------------------------------------------------ */

/* Unconditional. Protocol replies go out through this and are never gated -
 * a suppressed OK would hang the RPi forever. */
static void link_reply(const char *s)
{
    if ((s_uart == 0) || (s == 0)) { return; }

    (void)HAL_UART_Transmit(s_uart, (uint8_t *)s, (uint16_t)strlen(s), 100U);
}

void RpiLink_Send(const char *s) { link_reply(s); }

/* Human-readable telemetry. Silent once the RPi has spoken. */
void RpiLink_Log(const char *s)
{
    if (s_quiet) { return; }

    link_reply(s);
}

uint8_t  RpiLink_IsQuiet(void)      { return s_quiet; }
uint32_t RpiLink_GetRearmCount(void) { return s_rearms; }

uint32_t RpiLink_GetCalOkCount(void)    { return s_calOk; }
uint32_t RpiLink_GetCalBusyCount(void)  { return s_calBusy; }
uint32_t RpiLink_GetCalRangeCount(void) { return s_calRange; }

void RpiLink_ClearCalCounts(void)
{
    s_calOk    = 0U;
    s_calBusy  = 0U;
    s_calRange = 0U;
}

/* ------------------------------------------------------------------ */
/* Receive                                                             */
/* ------------------------------------------------------------------ */

void RpiLink_Init(UART_HandleTypeDef *huart)
{
    s_uart       = huart;
    s_rxLen      = 0U;
    s_lineReady  = 0U;
    s_overrun    = 0U;
    s_lineActive = 0U;
    s_f0Active   = 0U;
    s_fuState    = FU_OFF;
    s_lastOp     = CMD_NONE;
    s_quiet      = 0U;
    s_rearms     = 0U;
    s_lineFailed = 0U;

    Cmd_Init();

    (void)HAL_UART_Receive_IT(s_uart, &s_rxByte, 1U);
}

void RpiLink_RxCallback(void)
{
    char c = (char)s_rxByte;

    if (c == CMD_TERMINATOR)
    {
        s_rxLine[s_rxLen] = '\0';

        /* If the main loop has not consumed the previous line yet, drop this
         * one and remember to say RESEND. Overwriting s_pending mid-parse
         * would corrupt a line that has already been accepted. */
        if (s_lineReady)
        {
            s_overrun = 1U;
        }
        else
        {
            (void)memcpy(s_pending, s_rxLine, (size_t)s_rxLen + 1U);
            s_lineReady = 1U;
        }

        s_rxLen = 0U;
    }
    else if (c == '\r')
    {
        /* Tolerated and discarded, per the wire format. */
    }
    else
    {
        if (s_rxLen < (CMD_LINE_MAX - 1U))
        {
            s_rxLine[s_rxLen] = c;
            s_rxLen++;
        }
        else
        {
            /* Line too long. Keep swallowing until the terminator so the
             * next line starts clean; the parse will fail and reply RESEND. */
            s_rxLine[CMD_LINE_MAX - 2U] = '?';
        }
    }

    (void)HAL_UART_Receive_IT(s_uart, &s_rxByte, 1U);
}

/* ------------------------------------------------------------------ */
/* Executor                                                            */
/* ------------------------------------------------------------------ */

/* ===================================================================
 * Immediate opcodes.
 *
 * Answered the instant the line is parsed, without touching the queue or the
 * in-flight line, so a query is safe to send WHILE a move is running - which
 * is the point. The RPi needs to watch the ultrasonic as it drives, not only
 * between moves.
 *
 * Everything on the wire is an integer. Anything needing a decimal is scaled
 * x10, because newlib-nano will not print floats unless the float printf is
 * linked and it fails silently rather than loudly.
 * =================================================================== */

/* int32, not int16. IMU_GetHeading() free-runs and is only zeroed when a move
 * launches, so a robot left spinning between moves can pass 3276.7 degrees and
 * wrap a 16-bit result into a plausible-looking negative. */
static int32_t scaled10(float v)
{
    return (int32_t)((v * 10.0f) + ((v < 0.0f) ? -0.5f : 0.5f));
}

/* ------------------------------------------------------------------ */
/* IR snapshot and edge latch - see rpilink.h                          */
/* ------------------------------------------------------------------ */

void RpiLink_Tick(void)
{
    IrSnap_t    s;
    Odom_Pose_t p;

    Odom_GetWorldPose(&p);
    s.l_cm    = IR_LeftCm();
    s.r_cm    = IR_RightCm();
    s.l_raw   = IR_LeftFiltered();
    s.r_raw   = IR_RightFiltered();
    s.x_mm    = (int32_t)p.x_mm;
    s.y_mm    = (int32_t)p.y_mm;
    s.hdg_x10 = scaled10(p.heading_deg);
    s.ms      = HAL_GetTick();
    s_irSnap  = s;

    if (s_ireState != 1U) { return; }

    /* Only while driving: a robot parked beside something would otherwise
     * latch the moment it is armed, before the move it was armed for. */
    if (!Motion_IsBusy())
    {
        s_ireRun = 0U;
        return;
    }

    {
        uint16_t cm = (s_ireSide == 1U) ? s.l_cm : s.r_cm;

        if ((cm != SENSOR_NO_READING) && (cm >= s_ireLo) && (cm <= s_ireHi))
        {
            if (s_ireRun == 0U) { s_ireFirst = s; }
            s_ireRun++;
            if (s_ireRun >= RPILINK_IRE_CONFIRM_TICKS)
            {
                s_ireHit   = s_ireFirst;
                s_ireState = 2U;
            }
        }
        else
        {
            s_ireRun = 0U;
        }
    }
}

/* Copy a tick-written snapshot without tearing it. */
static IrSnap_t ir_snap_read(const volatile IrSnap_t *src)
{
    IrSnap_t out;
    uint32_t pm = __get_PRIMASK();

    __disable_irq();
    out = *src;
    __set_PRIMASK(pm);
    return out;
}

/* Push the table's hardware globals into the layers that use them. After
 * every !KG set and !KCLR - cheap, and it means no layer can hold a stale
 * copy. 0 = compiled constant throughout. */
static void apply_globals(void)
{
    int16_t w = CalTab_Global(CTG_WHEEL);

    (void)Odom_SetWheelRatio((w != 0) ? ((float)w / 10000.0f) : 1.0f);
    PID_SetMotorModel(0U, CalTab_Global(CTG_FFAF), CalTab_Global(CTG_FFAR),
                      CalTab_Global(CTG_DBA));
    PID_SetMotorModel(1U, CalTab_Global(CTG_FFBF), CalTab_Global(CTG_FFBR),
                      CalTab_Global(CTG_DBB));
}

/* One table value for ?K: the number, or "-" when unset. */
static int fmt_k(char *out, int room, uint8_t move, uint8_t anchor, uint8_t field)
{
    int16_t v;

    if (!CalTab_FieldAllowed(move, field)) { return snprintf(out, (size_t)room, ",."); }
    if (!CalTab_Get(move, anchor, field, &v)) { return snprintf(out, (size_t)room, ",-"); }
    return snprintf(out, (size_t)room, ",%d", (int)v);
}

static void answer_immediate(const Command_t *c)
{
    char b[256];

    switch (c->op)
    {
    case CMD_Q_US:
        snprintf(b, sizeof(b), "US,%u\n", (unsigned)Ultrasonic_GetCm());
        break;

    case CMD_Q_IR:
        snprintf(b, sizeof(b), "IR,%u,%u\n",
                 (unsigned)IR_LeftCm(), (unsigned)IR_RightCm());
        break;

    case CMD_Q_IRR:
        snprintf(b, sizeof(b), "IRR,%u,%u\n",
                 (unsigned)IR_LeftFiltered(), (unsigned)IR_RightFiltered());
        break;

    case CMD_Q_POSE:
        {
            Odom_Pose_t p;
            Odom_GetPose(&p);
            snprintf(b, sizeof(b), "POSE,%d,%d,%d\n",
                     (int)p.x_mm, (int)p.y_mm, (int)scaled10(p.heading_deg));
        }
        break;

    case CMD_Q_DIST:
        snprintf(b, sizeof(b), "DIST,%ld\n", (long)Motion_GetTravelled());
        break;

    case CMD_Q_WPOSE:
        {
            Odom_Pose_t p;
            Odom_GetWorldPose(&p);
            snprintf(b, sizeof(b), "WPOSE,%ld,%ld,%ld\n",
                     (long)p.x_mm, (long)p.y_mm, (long)scaled10(p.heading_deg));
        }
        break;

    case CMD_Q_IRW:
        {
            IrSnap_t s = ir_snap_read(&s_irSnap);
            snprintf(b, sizeof(b), "IRW,%u,%u,%u,%u,%ld,%ld,%ld,%lu\n",
                     (unsigned)s.l_cm, (unsigned)s.r_cm,
                     (unsigned)s.l_raw, (unsigned)s.r_raw,
                     (long)s.x_mm, (long)s.y_mm, (long)s.hdg_x10,
                     (unsigned long)s.ms);
        }
        break;

    case CMD_Q_IRE:
        {
            IrSnap_t s;
            uint8_t  state, side;
            uint32_t pm = __get_PRIMASK();

            __disable_irq();
            state = s_ireState;
            side  = s_ireSide;
            s     = s_ireHit;
            __set_PRIMASK(pm);

            if (state != 2U)
            {
                (void)memset(&s, 0, sizeof(s));
            }
            snprintf(b, sizeof(b), "IRE,%u,%u,%u,%u,%ld,%ld,%ld,%lu\n",
                     (unsigned)state, (unsigned)side,
                     (unsigned)((side == 2U) ? s.r_cm : s.l_cm),
                     (unsigned)((side == 2U) ? s.r_raw : s.l_raw),
                     (long)s.x_mm, (long)s.y_mm, (long)s.hdg_x10,
                     (unsigned long)s.ms);
        }
        break;

    case CMD_SET_IR_EDGE:
        {
            /* Arm or disarm in one go, so the tick never sees a half-set
               window. Already validated by the parser. */
            uint32_t pm  = __get_PRIMASK();
            int16_t  arg = c->arg;

            __disable_irq();
            s_ireRun = 0U;
            if (arg == 0)
            {
                s_ireState = 0U;
                s_ireSide  = 0U;
            }
            else
            {
                s_ireSide  = (uint8_t)(arg / 10000);
                s_ireLo    = (uint16_t)((arg / 100) % 100);
                s_ireHi    = (uint16_t)(arg % 100);
                s_ireState = 1U;
            }
            __set_PRIMASK(pm);
        }
        snprintf(b, sizeof(b), "%s", CMD_REPLY_OK);
        break;

    case CMD_Q_ENC:
        /* Raw cumulative counts, signed, as Odom_Update() reads them. Only
           differences mean anything - see CMD_Q_ENC in commands.h. */
        snprintf(b, sizeof(b), "ENC,%ld,%ld\n",
                 (long)Encoder_A_GetCount(), (long)Encoder_B_GetCount());
        break;

    case CMD_Q_TURN:
        snprintf(b, sizeof(b), "TURN,%d\n",
                 (int)scaled10(Odom_GetHeadingTotal()));
        break;

    case CMD_Q_STAT:
        /* busy is the LINE, not the motion layer.
         *
         * Motion_IsBusy() alone used to be close enough, because a movement
         * line was always either driving or finished. FU breaks that: it
         * spends RPILINK_FU_SETTLE_MS stopped between passes, deliberately,
         * and a sender polling ?STAT to decide the robot had arrived would
         * read busy 0 in that window and carry on mid-approach.
         *
         * s_lineActive is the honest answer to the question senders are
         * actually asking - "is my command finished?" - and it is also right
         * in the gap between two primitives on one line, where the old field
         * could flicker to 0 as well. */
        snprintf(b, sizeof(b), "STAT,%d,%u,%u,%u\n",
                 (int)Motion_GetState(),
                 (unsigned)((Motion_IsBusy() || s_lineActive) ? 1U : 0U),
                 (unsigned)IMU_IsReady(),
                 (unsigned)Motion_GetArcProfile());
        break;

    case CMD_Q_IMU:
        snprintf(b, sizeof(b), "IMU,%u,%d,%d,%lu,%d\n",
                 (unsigned)IMU_IsReady(),
                 (int)scaled10(IMU_GetHeading()),
                 (int)scaled10(IMU_GetRateDps()),
                 (unsigned long)IMU_GetStallCount(),
                 (int)IMU_GetPeakRaw());
        break;

    case CMD_Q_XCHK:
        snprintf(b, sizeof(b), "XCHK,%d,%d,%d,%u\n",
                 (int)scaled10(Motion_GetXCheckDeg()),
                 (int)scaled10((float)Motion_GetTurnedDeg()),
                 (int)Motion_GetXCheckErrPct(),
                 (unsigned)(Motion_XCheckFailed() ? 0U : 1U));
        break;

    case CMD_Q_VER:
        snprintf(b, sizeof(b), "VER,%s,%d\n",
                 CMD_FIRMWARE_NAME, (int)CMD_PROTOCOL_VERSION);
        break;

    case CMD_SET_PROFILE:
        /* Locked to TIGHT/SAFE (MOTION_PROFILE_LOCK). Refused rather than
           OK'd and ignored: a sender asking for CLEAN believes the robot
           will turn at 318 mm, and an OK would let it go on believing it. */
        snprintf(b, sizeof(b), "%s",
                 Motion_SetArcProfile((uint8_t)c->arg)
                 ? CMD_REPLY_OK : CMD_REPLY_RESEND);
        break;

    case CMD_SET_ZERO:
        /* Zeroes the heading carry-over too: the pose the robot is in now
           becomes heading 0, commanded and actual alike. */
        Motion_ZeroHeading();
        snprintf(b, sizeof(b), "%s", CMD_REPLY_OK);
        break;

    case CMD_Q_HDG:
        /* HDG,<commanded x10>,<actual x10>,<last arc aim x10>,
               <last arc turned x10>,<corrections capped since !ZERO>
           commanded - actual is the error the next move will correct.
           turned - aim is how precisely the last arc stopped. */
        snprintf(b, sizeof(b), "HDG,%d,%d,%d,%d,%u\n",
                 (int)scaled10(Motion_GetCommandedHeading()),
                 (int)scaled10(Motion_GetActualHeading()),
                 (int)scaled10(Motion_GetLastArcAimDeg()),
                 (int)scaled10(Motion_GetLastArcTurnedDeg()),
                 (unsigned)Motion_GetCarryClampCount());
        break;

    case CMD_SET_FAST:
        /* Read when each move launches and while it runs - a change lands on
           the next move, or on the speed of the one in flight, harmlessly. */
        Motion_SetFast((uint8_t)c->arg);
        snprintf(b, sizeof(b), "%s", CMD_REPLY_OK);
        break;

    case CMD_SET_LEARN:
        /* Takes effect at the END of the move in flight, which is where the
           learners run - so unlike the setters there is nothing to guard. */
        Motion_SetLearning((uint8_t)c->arg);
        snprintf(b, sizeof(b), "%s", CMD_REPLY_OK);
        break;

    case CMD_Q_CAL:
        /* Everything the firmware has learned this power-on. Safe mid-move
           like any other query - it only reads. Lag is x10 MILLISECONDS, not
           x10 seconds: MOTION_ARC_LAG_MAX_S is 0.25, so seconds x10 would
           quantise the whole usable range into three steps. */
        /* Protocol 4 appends gyro scale x10000 and the learning switch;
           protocol 5 the left and right arc deflections in force. Appended,
           not inserted, so older readers still read the right fields.
           Protocol 6 appends the last arc's raw decel measurement, x10, 0
           when it gave none - see Motion_GetLastDecelMeas(). */
        snprintf(b, sizeof(b), "CAL,%d,%d,%d,%d,%u,%u,%u,%d\n",
                 (int)scaled10(Motion_GetArcDecel()),
                 (int)scaled10(Motion_GetArcLag() * 1000.0f),
                 /* Rounded, not truncated: (int) cut 1.9 to 1 and -1.9
                    to -1, so a saved trim lost up to 1 us and every value
                    in -1..1 read as 0. */
                 (int)((Odom_GetHeadingTrim() >= 0.0f)
                       ? (Odom_GetHeadingTrim() + 0.5f)
                       : (Odom_GetHeadingTrim() - 0.5f)),
                 (int)((IMU_GetGyroScale() * 10000.0f) + 0.5f),
                 (unsigned)Motion_LearningEnabled(),
                 (unsigned)Motion_GetArcSteerSideUs(0U),
                 (unsigned)Motion_GetArcSteerSideUs(1U),
                 (int)scaled10(Motion_GetLastDecelMeas()));
        break;

    case CMD_Q_MV:
    {
        /* MOVE REPORT in motion.h. Floats x10 where a tenth matters. */
        const MotionReport_t *r = Motion_GetReport();

        snprintf(b, sizeof(b),
                 "MV,%u,%u,%u,%d,%u,%ld,%ld,%u,%d,%d,%d,%d,%d,%d,%ld,%ld,"
                 "%d,%d,%d,%d,%d,%ld,%ld,%u,%d,%ld\n",
                 (unsigned)r->seq, (unsigned)r->move, (unsigned)r->result,
                 (int)r->rpm, (unsigned)r->ticks,
                 (long)r->enc_a, (long)r->enc_b, (unsigned)r->n,
                 (int)scaled10(r->rpm_a), (int)scaled10(r->rpm_b),
                 (int)(r->duty_a + 0.5f), (int)(r->duty_b + 0.5f),
                 (int)scaled10(r->corr), (int)scaled10(r->max_err),
                 (long)scaled10(r->dist_brake), (long)scaled10(r->dist),
                 (int)scaled10(r->aim), (int)scaled10(r->turned),
                 (int)scaled10(r->brake_rate), (int)scaled10(r->brake_ang),
                 (int)scaled10(r->decel_meas),
                 (long)scaled10(r->x), (long)scaled10(r->y),
                 (unsigned)r->servo, (int)scaled10(r->trim),
                 (long)scaled10(r->target));
        break;
    }

    case CMD_Q_K:
    {
        /* K,<move>,<anchor>,<rpm>,<trim>,<coast>,<dist>,<steer>,<decel>,
               <lag>,<radius>,<boost>   - "-" unset, "." not this move's
           KG,<wheel>,<ffaf>,<ffar>,<ffbf>,<ffbr>,<dba>,<dbb>,<vs>,<va> */
        int     n;
        uint8_t f;

        if (c->kmove == (uint8_t)CT_GLOBAL)
        {
            n = snprintf(b, sizeof(b), "KG");
            for (f = 0U; f < (uint8_t)CTG_COUNT; f++)
            {
                n += snprintf(&b[n], sizeof(b) - (size_t)n, ",%d",
                              (int)CalTab_Global((CtGlobal_t)f));
            }
        }
        else
        {
            n = snprintf(b, sizeof(b), "K,%s,%u", CalTab_MoveName(c->kmove),
                         (unsigned)c->kanchor);
            for (f = 0U; f < (uint8_t)CTF_COUNT; f++)
            {
                n += fmt_k(&b[n], (int)sizeof(b) - n, c->kmove, c->kanchor, f);
            }
        }
        snprintf(&b[n], sizeof(b) - (size_t)n, "\n");
        break;
    }

    case CMD_SET_K:
    case CMD_SET_K_CLR:
        /* Same guard as the !CAL* setters below: a move in flight latched
           its values when it was queued, but the next one on the line has
           not, and a table changing between two moves of one line is a
           calibration nobody asked for. */
        if (Motion_IsBusy() || s_lineActive)
        {
            s_calBusy++;
            snprintf(b, sizeof(b), "%s", CMD_REPLY_RESEND);
        }
        else
        {
            uint8_t ok = 1U;

            if (c->op == CMD_SET_K_CLR) { CalTab_Clear(); }
            else { ok = CalTab_Set(c->kmove, c->kanchor, c->kfield, c->arg); }

            if (ok && (c->op == CMD_SET_K_CLR || c->kmove == (uint8_t)CT_GLOBAL))
            {
                apply_globals();
            }
            if (ok) { s_calOk++; } else { s_calRange++; }
            snprintf(b, sizeof(b), "%s", ok ? CMD_REPLY_OK : CMD_REPLY_RESEND);
        }
        break;

    case CMD_SET_CAL_DECEL:
    case CMD_SET_CAL_LAG:
    case CMD_SET_CAL_TRIM:
    case CMD_SET_CAL_GYRO:
    case CMD_SET_CAL_STEER_L:
    case CMD_SET_CAL_STEER_R:
        /* THE INTERFERENCE GUARD.
         *
         * These are the only immediate commands that write state a move in
         * flight is already reading - Motion_Tick() evaluates the braking
         * lead from s_arcDecel and s_arcLag on every tick of a turn. Letting
         * a restore land mid-arc would have the robot finish braking on a
         * model that changed underneath it, and the failure would look like
         * a bad turn rather than a protocol mistake.
         *
         * Refused rather than deferred: a queued restore would be worse,
         * because the sender would have no idea when it took effect. The
         * sender retries once ?STAT reports idle. */
        if (Motion_IsBusy())
        {
            s_calBusy++;
            snprintf(b, sizeof(b), "%s", CMD_REPLY_RESEND);
        }
        else
        {
            uint8_t ok;

            if (c->op == CMD_SET_CAL_DECEL)
            {
                ok = Motion_SetArcDecel((float)c->arg / 10.0f);
            }
            else if (c->op == CMD_SET_CAL_LAG)
            {
                ok = Motion_SetArcLag((float)c->arg / 10000.0f);
            }
            else if (c->op == CMD_SET_CAL_GYRO)
            {
                ok = IMU_SetGyroScale((float)c->arg / 10000.0f);
            }
            else if ((c->op == CMD_SET_CAL_STEER_L) ||
                     (c->op == CMD_SET_CAL_STEER_R))
            {
                /* Busy-guarded with the rest: the deflection is latched
                   when an arc is queued, and changing it under a turn in
                   its steering settle would split the difference. */
                ok = Motion_SetArcSteerSideUs(
                         (c->op == CMD_SET_CAL_STEER_R) ? 1U : 0U,
                         (uint16_t)c->arg);
            }
            else
            {
                ok = Odom_SetHeadingTrim((float)c->arg);
            }

            /* Counted apart from the busy refusal above. Both answer RESEND,
               and which one it was is the difference between "retry when the
               robot stops" and "the number you sent is wrong". */
            if (ok) { s_calOk++; } else { s_calRange++; }

            snprintf(b, sizeof(b), "%s",
                     ok ? CMD_REPLY_OK : CMD_REPLY_RESEND);
        }
        break;

    default:
        snprintf(b, sizeof(b), "%s", CMD_REPLY_RESEND);
        break;
    }

    link_reply(b);
}

static void abort_everything(void)
{
    Cmd_QueueFlush();
    Motion_Stop();            /* PID off, both bridges shorted: braking */
    Motion_ClearState();

    /* NO Motors_Coast() here. It used to follow straight after the brake
     * and undo it within microseconds, so RST released the wheels: on the
     * floor a robot at ~50 cm/s rolled on 18.5 cm after RST and drifted a
     * degree, eating the clearance the RPi had just checked. The shorted
     * bridges stay on - they only resist while the wheels turn, so a parked
     * robot draws nothing - until the next move's PID takes the motors
     * back. Matches how a normal stop holds the brake until it settles. */
    Motion_ResyncHeading();   /* whatever was planned no longer applies */
    s_lineActive = 0U;
    s_f0Active   = 0U;
    s_fuState    = FU_OFF;
    s_lineFailed = 0U;
}

/* ------------------------------------------------------------------ */
/* FU{n}: measured approach to a sender-chosen standoff                */
/* ------------------------------------------------------------------ */

/* Advance the approach. Returns 1 while the primitive still owns the line.
 *
 * Called from RpiLink_Poll(), so "time" here is HAL_GetTick() milliseconds and
 * not control ticks - nothing in this file runs at a fixed rate. */
static uint8_t fu_poll(void)
{
    switch (s_fuState)
    {
    case FU_SETTLE:
    {
        uint16_t d;
        int32_t  gap_cm;
        int32_t  mm;

        /* The robot is stopped, or stopping. Do not look at the sensor until
         * the filter can only be holding stationary samples - see
         * RPILINK_FU_SETTLE_MS for why that takes as long as it does. */
        if ((HAL_GetTick() - s_fuMark) < RPILINK_FU_SETTLE_MS) { return 1U; }

        d = Sensors_FrontDistanceCm();

        if (d == SENSOR_NO_READING)
        {
            /* Nothing in front, nothing in range, or no sensor at all.
             * Whichever it is there is no gap to close, and FU will not
             * invent one by driving forward hopefully. Say so and stop. */
            s_lineFailed = 3U;
            s_fuState    = FU_OFF;
            return 0U;
        }

        gap_cm = (int32_t)d - (int32_t)s_fuStopCm;

        /* Arrived. The tolerance is symmetric on purpose: a stop 1 cm close
         * is exactly as good as one 1 cm short, and chasing the difference
         * would burn passes on the sensor's own quantisation noise. */
        if ((gap_cm <= (int32_t)CMD_FU_TOL_CM) &&
            (gap_cm >= -(int32_t)CMD_FU_TOL_CM))
        {
            s_fuState = FU_OFF;
            return 0U;
        }

        if (s_fuPass >= RPILINK_FU_MAX_PASSES)
        {
            /* Out of passes. NOT a failure - see RPILINK_FU_MAX_PASSES. The
             * robot is stopped and stable; ?US reports where it really is. */
            s_fuState = FU_OFF;
            return 0U;
        }

        mm = gap_cm * 10;

        if (mm > RPILINK_FU_MAX_STEP_MM)  { mm = RPILINK_FU_MAX_STEP_MM;  }
        if (mm < -RPILINK_FU_MAX_BACK_MM) { mm = -RPILINK_FU_MAX_BACK_MM; }

        s_fuPass++;
        s_fuLastMm = mm;
        s_fuState  = FU_DRIVE;
        Motion_DriveDistance(mm);
        return 1U;
    }

    case FU_DRIVE:
        if (Motion_IsBusy())
        {
            /* The guard, NOT the terminator. The measurement that set this
             * move could have come off a surface behind the real obstacle,
             * and the world is allowed to change while the robot drives. If
             * the gap closes to the target early, cut the move short and let
             * the next settle work out where it actually ended up.
             *
             * Forward passes only. Sensors_ObstacleAhead() reports an
             * obstacle when the reading is MISSING, which is the right answer
             * when driving into something and the wrong one when backing away
             * from it - on a reverse correction it would abort the move on
             * its first poll and leave the robot where it started. */
            if ((s_fuLastMm > 0) && Sensors_ObstacleAhead(s_fuStopCm))
            {
                Motion_Stop();
            }
            return 1U;
        }

        /* Latch a stalled wheel before Motion_ClearState() hides it, exactly
         * as the line executor below does. */
        if (Motion_GetState() == MOTION_TIMEOUT)
        {
            s_lineFailed = Motion_WrongWayAborted() ? 2U
                         : Motion_ArcMissed()      ? 4U : 1U;
            Motion_ClearState();
            s_fuState = FU_OFF;
            return 0U;
        }

        Motion_ClearState();
        s_fuState = FU_SETTLE;
        s_fuMark  = HAL_GetTick();
        return 1U;

    default:
        s_fuState = FU_OFF;
        return 0U;
    }
}

/* Start one primitive. Returns 1 if it set the motion layer running. */
static uint8_t dispatch(Command_t c)
{
    s_lastOp   = c.op;
    s_f0Active = 0U;
    s_fuState  = FU_OFF;

    switch (c.op)
    {
    case CMD_FORWARD:
        if (c.arg == 0)
        {
            /* F0: run forward until an obstacle stops us. Implemented as a
             * very long move that RpiLink_Poll() cuts short, rather than a
             * special motion mode, so the motion layer stays unaware of
             * sensors and the watchdog still covers it.
             *
             * With the weak stub in place this will run to the motion
             * timeout. That is the correct behaviour for "no sensor yet" -
             * it stops, and the line still gets its reply. */
            Motion_DriveDistance(RPILINK_F0_MAX_MM);
            s_f0Active = 1U;
        }
        else
        {
            /* cm -> mm; speed 0 is the protocol 9 move, exactly. */
            Motion_DriveDistanceV((int32_t)c.arg * 10, (int16_t)(c.speed * 10U));
        }
        return 1U;

    case CMD_FWD_UNTIL_US:
        /* Nothing moves yet. The first thing FU does is stand still and read
         * the gap, because a reading taken while rolling is 120 ms stale and
         * that is the whole error budget. RpiLink_Poll() drives the rest of
         * it through fu_poll().
         *
         * Returning 0 would normally mean "finished instantly, pop the next
         * token" - the poll loop is gated on s_fuState so that it does not,
         * and the return value here is honest: the motion layer really is
         * still idle. */
        s_fuStopCm = (uint16_t)c.arg;
        s_fuPass   = 0U;
        s_fuLastMm = 0;
        s_fuState  = FU_SETTLE;
        s_fuMark   = HAL_GetTick();
        return 0U;

    case CMD_REVERSE:
        Motion_DriveDistanceV(-((int32_t)c.arg * 10), (int16_t)(c.speed * 10U));
        return 1U;

    case CMD_ARC_FWD_RIGHT: Motion_DriveArcV(c.arg, 1U, 1U, (int16_t)(c.speed * 10U)); return 1U;
    case CMD_ARC_FWD_LEFT:  Motion_DriveArcV(c.arg, 1U, 0U, (int16_t)(c.speed * 10U)); return 1U;
    case CMD_ARC_REV_RIGHT: Motion_DriveArcV(c.arg, 0U, 1U, (int16_t)(c.speed * 10U)); return 1U;
    case CMD_ARC_REV_LEFT:  Motion_DriveArcV(c.arg, 0U, 0U, (int16_t)(c.speed * 10U)); return 1U;

    case CMD_STOP:
        Motion_Stop();
        Motion_ClearState();
        Motors_Coast();
        return 0U;      /* completes instantly */

    case CMD_RESET:
        /* Should have been intercepted before it ever reached the queue,
         * but handle it defensively rather than fall through to default. */
        abort_everything();
        return 0U;

    default:
        return 0U;
    }
}

/* Take the received line: lowercased, surrounding whitespace trimmed. Returns
 * the start of the text inside line[]. */
static char *take_line(char *line)
{
    char     *p = line;
    uint16_t  i;

    (void)memcpy(line, s_pending, CMD_LINE_MAX);
    s_lineReady = 0U;

    /* Lowercase on receive, so sender casing is irrelevant everywhere
     * downstream. */
    for (i = 0U; i < CMD_LINE_MAX; i++)
    {
        if (line[i] == '\0') { break; }
        if ((line[i] >= 'A') && (line[i] <= 'Z'))
        {
            line[i] = (char)(line[i] - 'A' + 'a');
        }
    }

    /* Trim surrounding whitespace before the single-token tests. Cmd_ParseLine()
     * treats spaces as delimiters and skips empty runs, so movement lines never
     * cared - but RST and the immediate opcodes are matched against the WHOLE
     * line, and "?US " with a trailing space would otherwise fall through to
     * RESEND. Confusing, and trivial to prevent. */
    {
        uint16_t end;

        while ((*p == ' ') || (*p == '\t')) { p++; }

        end = (uint16_t)strlen(p);
        while ((end > 0U) && ((p[end - 1U] == ' ') || (p[end - 1U] == '\t')))
        {
            p[end - 1U] = '\0';
            end--;
        }
    }
    return p;
}

/* Reception torn down by a line error: re-arm it. See RpiLink_Poll(). */
static void rearm_rx(void)
{
    if ((s_uart != 0) && (s_uart->RxState == HAL_UART_STATE_READY))
    {
        s_rearms++;
        (void)HAL_UART_Receive_IT(s_uart, &s_rxByte, 1U);
    }
}

void RpiLink_Service(void)
{
    char      line[CMD_LINE_MAX];
    char     *p;
    Command_t c;

    rearm_rx();

    if (s_overrun)
    {
        s_overrun = 0U;
        link_reply(CMD_REPLY_RESEND);
    }

    if (!s_lineReady) { return; }

    p = take_line(line);
    c = Cmd_ParseToken(p);

    if (strcmp(p, "rst") == 0)
    {
        /* The !t2go line answers it, FAIL,STOP - RST's own silence stands. */
        Task2Fw_Abort();
    }
    else if (c.op == CMD_T2_ARROW)
    {
        Task2Fw_Answer((uint8_t)(c.arg / 10), ((c.arg % 10) == 1) ? 'R' : 'L');
    }
    else if (c.op == CMD_T2_LATERAL)
    {
        Task2Fw_Lateral((uint8_t)(c.arg / 1000), (int16_t)((c.arg % 1000) - 500));
    }
    else if ((p[0] == '?') && Cmd_IsImmediate(c.op))
    {
        /* Queries only. A setter mid-course would change the calibration
         * between two of its moves. */
        answer_immediate(&c);
    }
    else
    {
        link_reply(CMD_REPLY_RESEND);
    }
}

void RpiLink_Poll(void)
{
    char      line[CMD_LINE_MAX];
    char     *p = line;
    Command_t next;

    /* ---- 0. re-arm reception if it has been torn down ----
     *
     * HAL_UART_IRQHandler() calls UART_EndRxTransfer() on an overrun, framing
     * or noise error, which drops RxState back to READY and disables the RXNE
     * interrupt. It then calls HAL_UART_ErrorCallback() - and this project
     * does not define one, so the weak stub runs, nothing re-arms, and the
     * link is deaf for the rest of the session. Motion still works, the OLED
     * still updates, this function still runs; it simply never hears another
     * command. One glitch on the line is all it takes.
     *
     * Checking the state here recovers from that, and also from a much
     * narrower race where an RX interrupt lands inside HAL_UART_Transmit()'s
     * locked section and the re-arm inside the callback returns HAL_BUSY.
     *
     * The counter is the point: silent recovery hides the underlying fault, so
     * anything other than zero here means go and look at the wiring. */
    rearm_rx();

    /* ---- 1. a dropped line still owes the sender an answer ---- */
    if (s_overrun)
    {
        s_overrun = 0U;
        link_reply(CMD_REPLY_RESEND);
    }

    /* ---- 2. new line in ---- */
    if (s_lineReady)
    {
        p = take_line(line);

        if (strcmp(p, "rst") == 0)
        {
            /* Emergency abort. Drops everything queued, brakes now, and
             * replies NOTHING. The silence is part of the frozen protocol -
             * do not "helpfully" add an OK here. */
            abort_everything();
        }
        else if (Cmd_ParseToken(p).op == CMD_T2_GO)
        {
            /* The Task 2 course (task2fw.h). Blocks here until it ends -
             * RpiLink_Service() keeps the link alive meanwhile - and answers
             * the line once. Refused while anything else owns the robot. */
            Command_t go = Cmd_ParseToken(p);

            s_quiet = 1U;
            if (s_lineActive || Motion_IsBusy() || (s_fuState != FU_OFF))
            {
                link_reply(CMD_REPLY_RESEND);
            }
            else
            {
                link_reply(Task2Fw_Run((uint16_t)go.arg));
            }
        }
        else if (Cmd_IsImmediate(Cmd_ParseToken(p).op))
        {
            /* A lone query or setter. Answered here and now, without touching
             * the queue or s_lineActive, so it is safe to ask while a move is
             * still running - which is the point of having queries at all.
             * Cmd_ParseLine() rejects these inside a multi-token line, so this
             * is the only path that can accept one. */
            Command_t imm = Cmd_ParseToken(p);

            s_quiet = 1U;
            answer_immediate(&imm);
        }
        else if (Cmd_ParseLine(p))
        {
            /* A line parsed off the wire means a real host is driving, so the
             * console goes quiet from here. Anything else sharing this port
             * would land in the middle of the OK/RESEND stream the host is
             * parsing. One way only - it stays quiet until reset. */
            s_quiet      = 1U;
            s_lineActive = 1U;
        }
        else
        {
            /* All-or-nothing: nothing was queued, so nothing to undo. */
            link_reply(CMD_REPLY_RESEND);
        }
    }

    /* ---- 3. F0 obstacle check ---- */
    if (s_f0Active && Motion_IsBusy())
    {
        if (Sensors_ObstacleAhead(RPILINK_F0_STOP_CM))
        {
            Motion_Stop();
            s_f0Active = 0U;
        }
    }

    /* ---- 3b. FU approach ----
     *
     * BEFORE the line advances, and gated on its own state rather than on
     * Motion_IsBusy(). FU is stationary for most of its life - settling
     * between passes - so a check that only asked the motion layer would see
     * an idle robot, decide the primitive was over and pop the next token
     * out from under an approach that had not finished. */
    if (s_fuState != FU_OFF)
    {
        if (fu_poll()) { return; }
    }

    /* ---- 4. advance the line ---- */
    if (!s_lineActive) { return; }

    if (Motion_IsBusy()) { return; }

    /* Latch how the primitive ended BEFORE clearing it - Motion_ClearState()
     * drops TIMEOUT back to IDLE, so reading it afterwards always says the
     * move succeeded. The failure has to survive to the end of the line,
     * because the reply is owed to the line and not to the primitive. */
    if (Motion_GetState() == MOTION_TIMEOUT)
    {
        s_lineFailed = Motion_WrongWayAborted() ? 2U
                     : Motion_ArcMissed()      ? 4U : 1U;
    }

    Motion_ClearState();

    next = Cmd_QueuePop();

    if (next.op == CMD_NONE)
    {
        /* Whole line executed. One reply, now.
         *
         * A failed primitive still replies - the sender must never be left
         * blocked - but it no longer replies OK. That used to make a stalled
         * wheel indistinguishable from a completed move, so the RPi carried
         * on believing the robot had gone somewhere it had not. */
        uint8_t failed = s_lineFailed;

        s_lineActive = 0U;
        s_f0Active   = 0U;
        s_fuState    = FU_OFF;
        s_lineFailed = 0U;

        if      (failed == 3U) { link_reply(CMD_REPLY_FAIL_NOECHO);   }
        else if (failed == 2U) { link_reply(CMD_REPLY_FAIL_WRONGWAY); }
        else if (failed == 4U) { link_reply(CMD_REPLY_FAIL_SHORT);    }
        else if (failed == 1U) { link_reply(CMD_REPLY_FAIL_TIMEOUT);  }
        else                   { link_reply(CMD_REPLY_OK);            }
        return;
    }

    if (s_lineFailed == 3U)
    {
        /* FU found nothing to aim at, so it did not move - and the rest of
         * the line was written assuming it had. "FU20,FR90" with no echo
         * would otherwise still turn, from wherever the robot happened to be
         * standing rather than 20 cm off the wall.
         *
         * Drop what is left and let the next pass answer the line with the
         * failure. Exactly one reply either way, as the protocol promises; it
         * just is not OK.
         *
         * DELIBERATELY narrower than "any failure". A timed-out primitive
         * still lets the rest of its line run, which is the behaviour senders
         * have today - changing that is a protocol decision for the RPi owner
         * and not something to slip in alongside a new command. */
        Cmd_QueueFlush();
        return;
    }

    if (!dispatch(next))
    {
        /* Instant primitive such as S. Nothing to wait for - the next Poll()
         * pass picks up whatever follows it on the line. */
    }
}

uint8_t RpiLink_IsBusy(void) { return s_lineActive; }

CmdOpcode_t RpiLink_LastOpcode(void) { return s_lastOp; }
