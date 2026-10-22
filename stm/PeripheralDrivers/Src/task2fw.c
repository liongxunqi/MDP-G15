#include "task2fw.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "caltab.h"
#include "commands.h"
#include "encoders.h"
#include "imu.h"
#include "ir.h"
#include "motion.h"
#include "odom.h"
#include "rpilink.h"
#include "task2_mission.h"
#include "task2_params.h"
#include "ultrasonic.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD(d) ((d) * M_PI / 180.0)

/* Why the run ended, for the reply. The first cause wins. */
typedef enum
{
    T2F_NONE = 0,
    T2F_STOP,       /* rst                                         */
    T2F_BUTTON,     /* long press on the board                     */
    T2F_DEADLINE,   /* the !t2go seconds ran out                   */
    T2F_TIMEOUT,    /* a move's watchdog: a wheel stalled          */
    T2F_WRONGWAY,   /* an arc turned away from its target          */
    T2F_SHORT,      /* an arc finished over 10 degrees off its aim */
    T2F_GYRO,       /* the gyro went stale                         */
    T2F_NOEDGE      /* drove T2_EDGE_MAX_CM along obstacle 2, no end */
} T2Fail_t;

static volatile uint8_t  s_running;
static volatile uint8_t  s_fail;
static uint32_t          s_deadline;
static uint32_t          s_stalls0;

/* Answers from the link, by obstacle (index 1, 2). */
static volatile char     s_answer[3];
static volatile int16_t  s_latMm[3];
static volatile uint8_t  s_latOk[3];

/* The course frame: where the run started, in the world pose. */
static float             s_x0, s_y0;
static volatile Task2Odom s_odom;

static int16_t           s_arcRpm;   /* the speed turn_speed() last chose */
static float             s_guardCm;  /* > 0: stop a move this close ahead */
static uint8_t           s_guardHit;
static float             s_sideCm;   /* > 0: stop a forward move a side IR reads under this */
static uint8_t           s_sideHit;
static uint8_t           s_fwd;      /* the move now running goes forward  */
static uint8_t           s_cut;      /* this move was stopped on purpose  */
/* mouth_watch, per side: 0 waiting for its fence, 1 on the fence, 2 lost it */
static uint8_t           s_mouthOn, s_mouthState[2], s_mouthRun[2], s_mouthOk;
static uint32_t          s_mouthTick;
static double            s_mouthFirst[2], s_mouthX;
static uint32_t          s_echoSeen; /* last ultrasound echo handed out   */

__attribute__((weak)) uint8_t Task2Fw_BoardAbort(void) { return 0U; }

static int ir_read(int right, double *cm, int *near);
static double op_heading(void);

uint8_t Task2Fw_Running(void) { return s_running; }

static void fail(uint8_t why)
{
    if (s_fail == T2F_NONE) { s_fail = why; }
    Motion_Stop();
}

void Task2Fw_Abort(void) { fail(T2F_STOP); }

void Task2Fw_Answer(uint8_t obstacle, char arrow)
{
    if ((obstacle == 1U) || (obstacle == 2U)) { s_answer[obstacle] = arrow; }
}

void Task2Fw_Lateral(uint8_t obstacle, int16_t mm)
{
    if ((obstacle == 1U) || (obstacle == 2U))
    {
        s_latMm[obstacle] = mm;
        s_latOk[obstacle] = 1U;
    }
}

/* ------------------------------------------------------------------ */
/* The pump: what the main loop would be doing, plus the course's own  */
/* watchers. Called continuously while a move runs and while waiting.  */
/* ------------------------------------------------------------------ */

/* World pose -> the course frame (task2_mission.h): x from the start toward
 * the obstacles, y left, cm. h0 is set by the mission from heading(). */
static void refresh_odom(void)
{
    Odom_Pose_t p;
    double      h0, dx, dy;

    Odom_GetWorldPose(&p);
    h0 = RAD((double)s_odom.h0);
    dx = ((double)p.x_mm - (double)s_x0) / 10.0;
    dy = ((double)p.y_mm - (double)s_y0) / 10.0;
    s_odom.x = (float)( dx * cos(h0) + dy * sin(h0));
    s_odom.y = (float)(-dx * sin(h0) + dy * cos(h0));
}

static void pump(void)
{
    IMU_Poll();
    RpiLink_Service();
    refresh_odom();

    if (Task2Fw_BoardAbort())                         { fail(T2F_BUTTON);   }
    if ((int32_t)(HAL_GetTick() - s_deadline) >= 0)   { fail(T2F_DEADLINE); }
    if (IMU_GetStallCount() != s_stalls0)             { fail(T2F_GYRO);     }

    /* The guard (task2_mission.h: guard()). The reading is up to ~120 ms old
     * (three-ping median), which T2_GUARD_CM allows for. */
    if ((s_guardCm > 0.0f) && !s_guardHit && Motion_IsBusy())
    {
        uint16_t d = Sensors_FrontDistanceCm();

        if ((d != SENSOR_NO_READING) && (((float)d + ULTRA_OFFSET_CM) < s_guardCm))
        {
            Motion_Stop();
            s_guardHit = 1U;
            s_cut      = 1U;
        }
    }

    /* The carpark's mouth, on the way out (mouth_watch): a side IR on its
     * fence, then clear for IR_EDGE_CONFIRM ticks, read where its lens was
     * (IR_AHEAD ahead of the axle along the heading, T2_IR_LAG_CM back for
     * lag). Per side: one looking at obstacle 1 reads it as near, not clear,
     * and the first side to lose its fence gives the mouth. */
    if (s_mouthOn && !s_mouthOk && s_fwd && Motion_IsBusy() &&
        ((HAL_GetTick() - s_mouthTick) >= 10U))
    {
        int k;

        s_mouthTick = HAL_GetTick();
        for (k = 0; k < 2; k++)
        {
            double cm;
            int    near;
            const int on = ir_read(k, &cm, &near) ? (cm <= (double)T2_FENCE_SEEN_CM) : near;

            if (s_mouthState[k] == 0U)
            {
                s_mouthRun[k] = on ? (uint8_t)(s_mouthRun[k] + 1U) : 0U;
                if (s_mouthRun[k] >= IR_EDGE_CONFIRM) { s_mouthState[k] = 1U; s_mouthRun[k] = 0U; }
            }
            else if (!on)
            {
                if (s_mouthRun[k] == 0U)
                {
                    const double h = RAD(Task2_WrapDeg(op_heading() - (double)s_odom.h0));

                    s_mouthFirst[k] = s_odom.x + (T2_IR_AHEAD_CM - T2_IR_LAG_CM) * cos(h);
                }
                if (++s_mouthRun[k] >= IR_EDGE_CONFIRM)
                {
                    s_mouthOk = 1U;
                    s_mouthX  = s_mouthFirst[k];
                    break;
                }
            }
            else
            {
                s_mouthRun[k] = 0U;
            }
        }
    }

    /* The carpark's side-safety check (side_guard): forward moves only - a
     * centring move backs out to get away from a wall it is too near. */
    if ((s_sideCm > 0.0f) && !s_sideHit && s_fwd && Motion_IsBusy())
    {
        int k;

        for (k = 0; k < 2; k++)
        {
            double cm;
            int    near;

            if (ir_read(k, &cm, &near) ? (cm < (double)s_sideCm) : near)
            {
                Motion_Stop();
                s_sideHit = 1U;
                s_cut     = 1U;
                break;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Moves                                                               */
/* ------------------------------------------------------------------ */

static int failed(void) { return s_fail != T2F_NONE; }

/* Wait for the move to end and read how. A move stopped on purpose (an edge,
 * the guard, the IR watch) ends in IDLE from Motion_Stop() with the wheels
 * still turning: wait for them, then re-sync the heading - the carry-over
 * would otherwise steer the rest of the stopped move into the next one. */
static void finish_move(void)
{
    MotionState_t st;

    while (Motion_IsBusy() && !failed()) { pump(); }

    st = Motion_GetState();
    if ((st == MOTION_TIMEOUT) && !s_cut)
    {
        fail(Motion_WrongWayAborted() ? T2F_WRONGWAY
           : Motion_ArcMissed()      ? T2F_SHORT : T2F_TIMEOUT);
    }
    Motion_ClearState();

    if (s_cut || failed())
    {
        uint32_t t0 = HAL_GetTick(), still = HAL_GetTick();

        while ((HAL_GetTick() - t0) < 600U)
        {
            pump();
            if ((Encoder_A_GetDelta() != 0) || (Encoder_B_GetDelta() != 0)) { still = HAL_GetTick(); }
            if ((HAL_GetTick() - still) >= 50U) { break; }
        }
        Motion_ResyncHeading();
    }
    s_cut = 0U;
    refresh_odom();
}

static void op_straight(double cm)
{
    int32_t mm = (int32_t)lround(cm * 10.0);

    if (failed() || (labs(mm) < 5)) { return; }
    s_fwd = mm > 0;
    Motion_DriveDistanceV(mm, (int16_t)T2_STRAIGHT_RPM);
    finish_move();
}

/* deg > 0 forward, < 0 in reverse, on the lock `right`. Whole degrees: the
 * motion layer takes nothing finer, and the carry-over takes the rest. */
static void arc(double deg, uint8_t right)
{
    long d = lround(fabs(deg));

    if (failed() || (d == 0L)) { return; }
    s_fwd = deg > 0.0;
    Motion_DriveArcV((int16_t)d, (deg > 0.0) ? 1U : 0U, right, s_arcRpm);
    finish_move();
}

static void op_left(double deg)  { arc(deg, 0U); }
static void op_right(double deg) { arc(deg, 1U); }

/* Side IR: 1 seen, 0 clear. A no-reading with a high raw count is under
 * 10 cm (ir.h) - seen, never clear. */
/* One side IR now, per side (task2_params.h): 1 with *cm inside that side's
 * usable range, else 0 with *near 1 when a no-reading is the sensor closer
 * than its 10 cm floor (high raw count) rather than seeing nothing. */
static int ir_read(int right, double *cm, int *near)
{
    const uint16_t v   = right ? IR_RightCm() : IR_LeftCm();
    const uint16_t raw = right ? IR_RightFiltered() : IR_LeftFiltered();
    const float    max = right ? T2_IR_MAX_R_CM : T2_IR_MAX_L_CM;

    *near = 0;
    if (v == SENSOR_NO_READING)
    {
        *near = raw >= T2_IR_CLOSE_RAW;
        return 0;
    }
    if (((float)v < T2_IR_MIN_CM) || ((float)v > max)) { return 0; }
    *cm = (double)v;
    return 1;
}

/* Side IR: 1 seen (obstacle 2 beside it), 0 clear. Closer than the floor is
 * seen, never clear. */
static int ir_seen(int right)
{
    double cm;
    int    near;

    if (!ir_read(right, &cm, &near)) { return near; }
    return cm <= (right ? T2_IR_SEEN_R_CM : T2_IR_SEEN_L_CM);
}

static int op_ir(int right_sensor, double *cm, int *near)
{
    return ir_read(right_sensor, cm, near);
}

static double op_heading(void);

static void log_line(const char *name, long a, long b)
{
    char line[64];

    snprintf(line, sizeof line, "D,%s,%ld,%ld,%ld,%ld,%ld\n", name, a, b,
             lround(s_odom.x), lround(s_odom.y),
             lround(Task2_WrapDeg(op_heading() - s_odom.h0)));
    RpiLink_Send(line);
}

/* One leg along obstacle 2 (task2_mission.h edge_leg). The stop past the end
 * is a Motion_Stop() brake at T2_EDGE_RPM - at once along the face
 * (may_be_past), T2_EDGE_BACK_EXTRA_CM further on behind it (task2_params.h).
 * The course plans from where the odometry says it stopped, as the original
 * did after its braking envelope; `at` is where the edge was read - the pose
 * of the first sample of the confirming run, T2_IR_LAG_CM back along the leg
 * for the sensor's lag. max_cm > 0 ends the leg there without a fault. */
static int op_edge_leg(int right_sensor, int may_be_past, double max_cm, double at[3])
{
    const int32_t extra_mm = may_be_past ? 0 : (int32_t)(T2_EDGE_BACK_EXTRA_CM * 10.0f);
    const float   leg_cm   = (max_cm > 0.0) ? (float)max_cm : T2_EDGE_MAX_CM;
    int      phase = 0, hits = 0;
    int32_t  stop_at = 0;
    uint32_t tick;
    double   first[3] = {0.0, 0.0, 0.0};

    if (failed()) { return EDGE_FAILED; }

    if (may_be_past)
    {
        int clear = 0;

        tick = HAL_GetTick();
        while (clear < IR_EDGE_CONFIRM)
        {
            pump();
            if (HAL_GetTick() - tick < 10U) { continue; }
            tick = HAL_GetTick();
            if (ir_seen(right_sensor)) { break; }
            clear++;
        }
        if (clear >= IR_EDGE_CONFIRM)
        {
            log_line("ir_past", right_sensor, 0);
            return EDGE_PAST;
        }
    }

    s_fwd = 1U;
    Motion_DriveDistanceV((int32_t)(leg_cm * 10.0f), (int16_t)T2_EDGE_RPM);
    tick = HAL_GetTick();
    while (Motion_IsBusy() && !failed())
    {
        pump();
        if ((phase == 2) && !s_cut && (Motion_GetTravelled() >= stop_at))
        {
            Motion_Stop();
            s_cut = 1U;
        }
        if ((phase == 2) || (HAL_GetTick() - tick < 10U)) { continue; }
        tick = HAL_GetTick();
        {
            int s   = ir_seen(right_sensor);
            int hit = (phase == 0) ? s : !s;

            if (hit && (hits == 0))
            {
                first[0] = s_odom.x;
                first[1] = s_odom.y;
                first[2] = op_heading();
            }
            hits = hit ? hits + 1 : 0;
            if (hits < IR_EDGE_CONFIRM) { continue; }
            hits = 0;
            phase++;
            log_line(phase == 1 ? "ir_seen" : "ir_lost", right_sensor, 0);
            if (phase == 2)
            {
                const double h = RAD(Task2_WrapDeg(first[2] - (double)s_odom.h0));

                at[0] = first[0] - T2_IR_LAG_CM * cos(h);
                at[1] = first[1] - T2_IR_LAG_CM * sin(h);
                at[2] = first[2];
                stop_at = Motion_GetTravelled() + extra_mm;
            }
        }
    }
    finish_move();
    if (failed()) { return EDGE_FAILED; }
    if (phase == 2) { return EDGE_FOUND; }
    if (max_cm > 0.0) { return EDGE_CAPPED; }
    fail(T2F_NOEDGE);
    return EDGE_FAILED;
}

/* deg on the lock, watching the side IR past check_deg (task2_mission.h
 * turn_watch). Clear for IR_EDGE_CONFIRM ticks: stop there, report where the
 * first clear reading was. */
/* Back along the leg until the side IR sees the obstacle (task2_mission.h
 * seek_edge): straight in reverse at T2_EDGE_RPM, stopped on the confirming
 * sample. Seeing something as it starts - the box past obstacle 2 it has
 * just rejected - it first waits for clear, then for the next thing seen.
 * Reversing, a reading describes the robot T2_IR_LAG_CM back along its path
 * - forward of where it is - so `at` is moved that way. */
static int op_seek_edge(int right_sensor, double max_cm, double at[3])
{
    int      hits = 0, found = 0, clears = 0;
    int      passing = ir_seen(right_sensor);
    uint32_t tick;
    double   first[3] = {0.0, 0.0, 0.0};

    if (failed() || (max_cm < 0.5)) { return failed() ? EDGE_FAILED : EDGE_CAPPED; }
    s_fwd = 0U;
    Motion_DriveDistanceV(-(int32_t)lround(max_cm * 10.0), (int16_t)T2_EDGE_RPM);
    tick = HAL_GetTick();
    while (Motion_IsBusy() && !failed())
    {
        pump();
        if (found || (HAL_GetTick() - tick < 10U)) { continue; }
        tick = HAL_GetTick();
        if (passing)
        {
            clears = ir_seen(right_sensor) ? 0 : clears + 1;
            if (clears >= IR_EDGE_CONFIRM) { passing = 0; }
            continue;
        }
        if (!ir_seen(right_sensor)) { hits = 0; continue; }
        if (hits == 0)
        {
            first[0] = s_odom.x;
            first[1] = s_odom.y;
            first[2] = op_heading();
        }
        if (++hits < IR_EDGE_CONFIRM) { continue; }
        found = 1;
        {
            const double h = RAD(Task2_WrapDeg(first[2] - (double)s_odom.h0));

            at[0] = first[0] + T2_IR_LAG_CM * cos(h);
            at[1] = first[1] + T2_IR_LAG_CM * sin(h);
            at[2] = first[2];
        }
        log_line("ir_seen_back", right_sensor, 0);
        Motion_Stop();
        s_cut = 1U;
    }
    finish_move();
    if (failed()) { return EDGE_FAILED; }
    return found ? EDGE_FOUND : EDGE_CAPPED;
}

static int op_turn_watch(int left, double deg, int right_sensor, double check_deg, double at[3])
{
    const double h_start = op_heading();
    int      clears = 0, cut = 0;
    double   first[3] = {0.0, 0.0, 0.0};
    uint32_t tick;

    if (failed()) { return 0; }
    s_fwd = 1U;
    Motion_DriveArcV((int16_t)lround(deg), 1U, left ? 0U : 1U, s_arcRpm);
    tick = HAL_GetTick();
    while (Motion_IsBusy() && !failed())
    {
        pump();
        if (cut || (HAL_GetTick() - tick < 10U)) { continue; }
        tick = HAL_GetTick();
        if (fabs(Task2_WrapDeg(op_heading() - h_start)) < check_deg) { clears = 0; continue; }
        if (ir_seen(right_sensor)) { clears = 0; continue; }
        if (clears == 0)
        {
            first[0] = s_odom.x;
            first[1] = s_odom.y;
            first[2] = op_heading();
        }
        if (++clears < IR_EDGE_CONFIRM) { continue; }
        cut = 1;
        at[0] = first[0];
        at[1] = first[1];
        at[2] = first[2];
        Motion_Stop();
        s_cut = 1U;
    }
    finish_move();
    log_line("watch", cut, lround(fabs(Task2_WrapDeg(op_heading() - h_start))));
    return cut && !failed();
}

static void op_turn_speed(int open)
{
    s_arcRpm = (int16_t)(open ? T2_ARC_RPM_OPEN : T2_ARC_RPM_PRECISE);
}

/* The original held the steering at lock between two turns on the same lock
 * and braked harder into the carpark. This motion layer recentres after every
 * arc and stops on its calibrated coast; both cost a little time, not
 * accuracy. */
static void op_hold_steering(int hold) { (void)hold; }
static void op_brake_hard(int hard)    { (void)hard; }

static void op_rebase_heading(void)    { Motion_ResyncHeading(); }

static void op_mouth_watch(int on)
{
    s_mouthOn = (uint8_t)(on != 0);
}

static int op_mouth_seen(double *x)
{
    if (!s_mouthOk) { return 0; }
    *x = s_mouthX;
    return 1;
}

static void op_side_guard(double cm)
{
    s_sideHit = 0U;
    s_sideCm  = (float)cm;
}

static int op_side_guarded(void)
{
    int hit = s_sideHit;

    s_sideHit = 0U;
    return hit;
}

static void op_guard(double cm)
{
    s_guardHit = 0U;
    s_guardCm  = (float)cm;
}

static int op_guarded(void)
{
    int hit = s_guardHit;

    s_guardHit = 0U;
    return hit;
}

/* 1 with a reading from an echo not handed out before. */
static int op_ultrasonic(uint16_t *cm)
{
    uint32_t n = Ultrasonic_GetEchoCount();
    uint16_t d = Ultrasonic_GetCm();

    if ((n == s_echoSeen) || (d == SENSOR_NO_READING)) { return 0; }
    s_echoSeen = n;
    *cm = d;
    return 1;
}

static double op_heading(void)
{
    Odom_Pose_t p;

    Odom_GetWorldPose(&p);
    return (double)p.heading_deg;
}

static int op_failed(void) { return failed(); }

static void op_send(const char *line) { RpiLink_Send(line); }

static char op_answer(int obstacle)
{
    return ((obstacle == 1) || (obstacle == 2)) ? s_answer[obstacle] : 0;
}

static int op_lateral(int obstacle, double *cm)
{
    if (((obstacle != 1) && (obstacle != 2)) || !s_latOk[obstacle]) { return 0; }
    *cm = s_latMm[obstacle] / 10.0;
    return 1;
}

static void op_clear_answer(void)
{
    s_answer[1] = s_answer[2] = 0;
    s_latOk[1]  = s_latOk[2]  = 0U;
}

static void op_wait(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();

    while (((HAL_GetTick() - t0) < ms) && !failed()) { pump(); }
}

static uint32_t op_now(void) { return HAL_GetTick(); }

/* ------------------------------------------------------------------ */
/* The run                                                             */
/* ------------------------------------------------------------------ */

/* The radius a turn on this side drives, cm: the table's at the arc speed,
 * else T2_ARC_DEFAULT_CM. */
static double table_radius_cm(CtMove_t move)
{
    float rpm = (float)((T2_ARC_RPM_PRECISE > 0) ? T2_ARC_RPM_PRECISE : CalTab_Global(CTG_VA));
    float v;

    if (rpm <= 0.0f)
    {
        const ArcProfile_t *p = Motion_GetArcProfileInfo(Motion_GetArcProfile());

        rpm = (p != 0) ? (float)p->rpm : 0.0f;
    }
    if (CalTab_Lookup(move, CTF_RADIUS, rpm, &v) && (v >= 100.0f)) { return v / 10.0; }
    return T2_ARC_DEFAULT_CM;
}

static const char *reply_for(int result)
{
    switch (s_fail)
    {
    case T2F_STOP:     return "FAIL,STOP\n";
    case T2F_BUTTON:   return "FAIL,STOP\n";
    case T2F_DEADLINE: return "FAIL,DEADLINE\n";
    case T2F_TIMEOUT:  return CMD_REPLY_FAIL_TIMEOUT;
    case T2F_WRONGWAY: return CMD_REPLY_FAIL_WRONGWAY;
    case T2F_SHORT:    return CMD_REPLY_FAIL_SHORT;
    case T2F_GYRO:     return "FAIL,GYRO\n";
    case T2F_NOEDGE:   return "FAIL,NOEDGE\n";
    default:           break;
    }
    if (result == T2M_OK)            { return CMD_REPLY_OK; }
    if (result == T2M_NO_ULTRASONIC) { return CMD_REPLY_FAIL_NOECHO; }
    if (result == T2M_NO_ARROW)      { return "FAIL,NOARROW\n"; }
    return "FAIL,STOP\n";
}

const char *Task2Fw_Run(uint16_t seconds)
{
    static const Task2Ops ops = {
        .straight       = op_straight,
        .left           = op_left,
        .right          = op_right,
        .edge_leg       = op_edge_leg,
        .seek_edge      = op_seek_edge,
        .turn_watch     = op_turn_watch,
        .turn_speed     = op_turn_speed,
        .hold_steering  = op_hold_steering,
        .brake_hard     = op_brake_hard,
        .rebase_heading = op_rebase_heading,
        .guard          = op_guard,
        .guarded        = op_guarded,
        .side_guard     = op_side_guard,
        .side_guarded   = op_side_guarded,
        .mouth_watch    = op_mouth_watch,
        .mouth_seen     = op_mouth_seen,
        .ultrasonic     = op_ultrasonic,
        .ir             = op_ir,
        .heading        = op_heading,
        .failed         = op_failed,
        .send           = op_send,
        .answer         = op_answer,
        .lateral        = op_lateral,
        .clear_answer   = op_clear_answer,
        .wait_ms        = op_wait,
        .now_ms         = op_now,
        .odom           = &s_odom,
    };
    Odom_Pose_t p;
    double      arc_l, arc_r;
    int         result;

    if (!IMU_IsReady()) { return "FAIL,GYRO\n"; }

    s_fail     = T2F_NONE;
    s_deadline = HAL_GetTick() + (uint32_t)seconds * 1000U;
    s_stalls0  = IMU_GetStallCount();
    s_guardCm  = 0.0f;
    s_guardHit = 0U;
    s_cut      = 0U;
    s_sideCm   = 0.0f;
    s_sideHit  = 0U;
    s_fwd      = 0U;
    s_mouthOn  = s_mouthOk = 0U;
    s_mouthState[0] = s_mouthState[1] = s_mouthRun[0] = s_mouthRun[1] = 0U;
    s_echoSeen = Ultrasonic_GetEchoCount();
    op_clear_answer();
    op_turn_speed(0);

    Odom_GetWorldPose(&p);
    s_x0     = p.x_mm;
    s_y0     = p.y_mm;
    s_odom.h0 = p.heading_deg;
    s_odom.x = 0.0f;
    s_odom.y = 0.0f;

    arc_l = table_radius_cm(CT_FL);
    arc_r = table_radius_cm(CT_FR);
    Task2Mission_SetArcs(arc_l, arc_r);
    {
        char line[48];

        snprintf(line, sizeof line, "D,arcs,%ld,%ld\n", lround(arc_l * 10.0), lround(arc_r * 10.0));
        RpiLink_Send(line);
    }

    Motion_ResyncHeading();
    s_running = 1U;
    result = Task2Mission_Run(&ops);
    s_running = 0U;

    s_guardCm = 0.0f;
    s_sideCm  = 0.0f;
    if (failed()) { Motion_Stop(); }
    Motion_ClearState();
    Motion_ResyncHeading();
    return reply_for(result);
}
