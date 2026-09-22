#include "rpilink.h"
#include "motion.h"
#include "motors.h"
#include "pid.h"
#include "odom.h"
#include "imu.h"
#include "ir.h"
#include "ultrasonic.h"
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

static void answer_immediate(const Command_t *c)
{
    char b[96];

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
        Motion_SetArcProfile((uint8_t)c->arg);
        snprintf(b, sizeof(b), "%s", CMD_REPLY_OK);
        break;

    case CMD_SET_ZERO:
        Odom_Reset();
        snprintf(b, sizeof(b), "%s", CMD_REPLY_OK);
        break;

    case CMD_Q_CAL:
        /* Everything the firmware has learned this power-on. Safe mid-move
           like any other query - it only reads. Lag is x10 MILLISECONDS, not
           x10 seconds: MOTION_ARC_LAG_MAX_S is 0.25, so seconds x10 would
           quantise the whole usable range into three steps. */
        snprintf(b, sizeof(b), "CAL,%d,%d,%d\n",
                 (int)scaled10(Motion_GetArcDecel()),
                 (int)scaled10(Motion_GetArcLag() * 1000.0f),
                 (int)Odom_GetHeadingTrim());
        break;

    case CMD_SET_CAL_DECEL:
    case CMD_SET_CAL_LAG:
    case CMD_SET_CAL_TRIM:
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
    Motion_Stop();
    Motion_ClearState();
    Motors_Coast();
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
            s_lineFailed = Motion_WrongWayAborted() ? 2U : 1U;
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
            Motion_DriveDistance((int32_t)c.arg * 10);   /* cm -> mm */
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
        Motion_DriveDistance(-((int32_t)c.arg * 10));
        return 1U;

    case CMD_ARC_FWD_RIGHT: Motion_DriveArc(c.arg, 1U, 1U); return 1U;
    case CMD_ARC_FWD_LEFT:  Motion_DriveArc(c.arg, 1U, 0U); return 1U;
    case CMD_ARC_REV_RIGHT: Motion_DriveArc(c.arg, 0U, 1U); return 1U;
    case CMD_ARC_REV_LEFT:  Motion_DriveArc(c.arg, 0U, 0U); return 1U;

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

void RpiLink_Poll(void)
{
    char      line[CMD_LINE_MAX];
    char     *p = line;
    Command_t next;
    uint16_t  i;

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
    if ((s_uart != 0) && (s_uart->RxState == HAL_UART_STATE_READY))
    {
        s_rearms++;
        (void)HAL_UART_Receive_IT(s_uart, &s_rxByte, 1U);
    }

    /* ---- 1. a dropped line still owes the sender an answer ---- */
    if (s_overrun)
    {
        s_overrun = 0U;
        link_reply(CMD_REPLY_RESEND);
    }

    /* ---- 2. new line in ---- */
    if (s_lineReady)
    {
        (void)memcpy(line, s_pending, sizeof(line));
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

        /* Trim surrounding whitespace before the single-token tests below.
         * Cmd_ParseLine() treats spaces as delimiters and skips empty runs, so
         * movement lines never cared - but RST and the immediate opcodes are
         * matched against the WHOLE line, and "?US " with a trailing space
         * would otherwise fall through to RESEND. Confusing, and trivial to
         * prevent. */
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

        if (strcmp(p, "rst") == 0)
        {
            /* Emergency abort. Drops everything queued, brakes now, and
             * replies NOTHING. The silence is part of the frozen protocol -
             * do not "helpfully" add an OK here. */
            abort_everything();
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
        s_lineFailed = Motion_WrongWayAborted() ? 2U : 1U;
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
