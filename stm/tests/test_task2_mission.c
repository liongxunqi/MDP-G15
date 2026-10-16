/*
 * Run the real Task 2 course (task2_mission.c) against a simulated car - OUR
 * car, with task2_params.h's geometry - and check the protocol 11 tokens.
 *
 * Ported from MDP_WORKING_2's tests/test_task2_mission.c, which proved the
 * same course on their car. Task2Mission_Run is the production function; only
 * Task2Ops is simulated, the way task2fw.c drives this robot:
 *   - the reference point is the middle of the rear axle (Odom's pose);
 *   - turns are arcs of that point at the radius for the steering side. The
 *     course PLANS on T2_ARC_DEFAULT_CM both sides (what Task2Mission_SetArcs
 *     gets with no table radius), but the simulated car turns at every set
 *     in RADII, so the course is proven against a car that does not match its
 *     calibration;
 *   - a move stopped on purpose (an edge, the guard, the IR watch) carries on
 *     a little: Motion_Stop() brakes from speed, after the sensor's lag;
 *   - the ultrasound reads ULTRA_OFFSET_CM off the bumper gap; the side IR
 *     "sees" an obstacle within IR_SEE_CM of its lens.
 * The car is a 23 x 18.8 cm rectangle, the rear axle 2.5 cm from its back,
 * checked against every obstacle and wall after every 1 cm or 1 degree.
 *
 * Arena: carpark interior x -50..0, y -30..30 (60 wide, 50 deep, mouth at
 * x = 0, task2mn_park.py); obstacle 1 (10 x 10) GAP1 past the mouth; obstacle
 * 2 (10 deep, LEN2 across) GAP2 past obstacle 1, both on one line LAT cm off
 * the carpark's centre; the camera reports that 2 cm wrong. Matrix: 4 arrow
 * pairs x GAP1, GAP2 in {60, 105, 150} x LEN2 in {30, 60, 100, 140} x LAT in
 * {-8, 0, 8} x RADII.
 *
 * Build and run - anywhere with a C compiler, the RPi included:
 *   cc -std=c11 -O2 -Wall -Wextra -Istm/tests -Istm/PeripheralDrivers/Inc \
 *     stm/tests/test_task2_mission.c stm/PeripheralDrivers/Src/task2_mission.c \
 *     stm/PeripheralDrivers/Src/commands.c stm/PeripheralDrivers/Src/caltab.c \
 *     -lm -o /tmp/test_task2_mission && /tmp/test_task2_mission
 * One course, traced move by move:
 *   test_task2_mission <pL>/<pR>/<oL>/<oR> <gap1> <gap2> <len2> <arrow1 L|R> <arrow2 L|R> [lat]
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "task2_mission.h"
#include "task2_params.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD(d) ((d) * M_PI / 180.0)

/* ---- the car --------------------------------------------------------- */
#define REAR_CM      ((double)T2_REAR_CM)
#define FRONT_CM     ((double)T2_FRONT_CM)
#define HALF_WID_CM  ((double)T2_HALF_WIDTH_CM)
#define IR_AT_CM     ((double)T2_IR_AHEAD_CM)
#define IR_LENS_Y_CM ((double)T2_IR_LENS_CM)
/* The firmware calls the obstacle seen at a reading of T2_IR_SEEN_CM or
 * less; -DIR_SEE_CM=... models an IR that reaches less far on the black. */
#ifndef IR_SEE_CM
#define IR_SEE_CM    ((double)T2_IR_SEEN_CM)
#endif
#define GYRO_BOOT_DEG 34.5  /* the world heading did not start at zero */
#define SIDE_ROOM_CM (WALL_SIM_CM + 10.0) /* past obstacle 2's end: the walls (WALL_SIM_CM) catch it first */
#ifndef CLEAR_CM
#define CLEAR_CM      2.0   /* what the car must keep off both obstacles */
#endif
#define TIGHT_CLEAR_CM 1.0  /* ...at the 60 cm gap, where the course splits it */

/* What a deliberate stop costs, past the moment the course decided it. NOT
 * MEASURED on this robot - measure and update:
 *   EDGE  the IR's median and 3-tick confirm (~60 ms) at T2_EDGE_RPM
 *         (34.5 cm/s at 100 rpm), then Motion_Stop()'s braking;
 *   WATCH the same lag at arc speed (~80 deg/s), then braking;
 *   GUARD the three-ping median's ~120 ms at straight speed, then braking. */
#ifndef EDGE_OVERRUN_CM
#define EDGE_OVERRUN_CM   8.0
#endif
#ifndef WATCH_OVERRUN_DEG
#define WATCH_OVERRUN_DEG 12.0
#endif
/* The arena's side walls, this far past obstacle 2's ends: the rules' least. */
#ifndef WALL_SIM_CM
#define WALL_SIM_CM 50.0
#endif
#define SEEK_OVERRUN_CM 3.0   /* the reverse seek's stop, at T2_EDGE_RPM */
#ifndef GUARD_OVERRUN_CM
#define GUARD_OVERRUN_CM  8.0
#endif

/* {precise left, precise right, open left, open right}: the radius a turn
 * really runs at. The first is the plan; the others 2 cm tighter, 2 cm wider
 * and lopsided. */
static const double RADII[][4] = {
  {30.0, 30.0, 30.0, 30.0}, {28.0, 28.0, 28.0, 28.0}, {32.0, 32.0, 32.0, 32.0},
  {28.0, 32.0, 28.0, 32.0},
};
static double rad[4];                      /* this run's */
static int open_turns;                     /* what turn_speed() last asked for */
#define LAT_ERR_CM 2.0                     /* how wrong the camera's lateral is */

/* Rough timing, only for comparing versions of the course. */
#define SEC_PER_MOVE 0.55
#define SEC_PER_DEG  0.022
#define SEC_PER_CM   0.013

typedef struct { double x0, x1, y0, y1; const char *name; } Box;

static struct {
  double x, y, heading;             /* truth: rear axle middle, CCW degrees */
  Box box[8]; int n_box;
  int collided; char hit[96];
  int lost_edge;
  uint32_t clock;
  char arrow[3];
  uint32_t answer_after[3], asked_at[3];
  int asked[3], prompts[3];
  int answer_wrong_obstacle, never_answer, stale_ultrasonic;
  int missed_echoes, phantoms, park_short, entered, blind1, slant_far;
  double guard; int guard_hit;
  int peek_long, peek_reads, ignore1;
  double lat;
  int no_lateral;
  double first_move;
  int moves;
  double secs;
  double clear;
  int edge_side, edge_calls;
  double photo_gap[3];
  double reach;
  int after_swerve, straights2;
  int swerved, tight; double swerve_y;
  double side_guard; int side_hit;  /* op_side_guard */
  double start_y;                   /* true start off the bay's centreline; odometry says 0 */
  int mouth_on, mouth_state[2], mouth_run[2], mouth_ok; double mouth_first[2], mouth_x;  /* op_mouth_watch */
} sim;

static Task2Odom odom;
static int trace;

#define TRACE(...) do { if (trace) { printf("  "); printf(__VA_ARGS__); \
  printf("  -> axle (%.1f,%.1f) h %.0f\n", sim.x, sim.y, sim.heading); } } while (0)

static int inside(const Box *b, double x, double y)
{
  return x > b->x0 && x < b->x1 && y > b->y0 && y < b->y1;
}

static void car_point(double lx, double ly, double *wx, double *wy)
{
  double c = cos(RAD(sim.heading)), s = sin(RAD(sim.heading));
  *wx = sim.x + lx * c - ly * s;
  *wy = sim.y + lx * s + ly * c;
}

static void touch(double lx, double ly)
{
  double wx, wy;
  car_point(lx, ly, &wx, &wy);
  if (fabs(wy - sim.lat) > sim.reach) sim.reach = fabs(wy - sim.lat);
  for (int o = 0; o < sim.n_box && !sim.collided; ++o) {
    Box b = sim.box[o];
    if (o < 2) { b.x0 -= sim.clear; b.x1 += sim.clear; b.y0 -= sim.clear; b.y1 += sim.clear; }
    if (inside(&b, wx, wy)) {
      sim.collided = 1;
      snprintf(sim.hit, sizeof sim.hit, "%s at (%.0f,%.0f), car point (%.1f,%.1f) h %.0f",
               sim.box[o].name, wx, wy, lx, ly, sim.heading);
    }
  }
}

/* The outline, every edge sampled at 1 cm or finer. */
static void check_collision(void)
{
  const double c[5][2] = {{-REAR_CM, -HALF_WID_CM}, {FRONT_CM, -HALF_WID_CM},
                          {FRONT_CM, HALF_WID_CM}, {-REAR_CM, HALF_WID_CM}, {-REAR_CM, -HALF_WID_CM}};
  for (int e = 0; e < 4 && !sim.collided; ++e) {
    double len = hypot(c[e + 1][0] - c[e][0], c[e + 1][1] - c[e][1]);
    int n = (int)ceil(len);
    for (int i = 0; i <= n && !sim.collided; ++i) {
      double t = (double)i / n;
      touch(c[e][0] + (c[e + 1][0] - c[e][0]) * t, c[e][1] + (c[e + 1][1] - c[e][1]) * t);
    }
  }
}

#ifndef SLANT_OK_DEG
#define SLANT_OK_DEG 10.0
#endif
static double bumper_gap(void);
static int guard_trips(void)
{
  if (sim.guard <= 0.0 || sim.guard_hit) return 0;
  double gap = bumper_gap();
  if (sim.slant_far && fabs(remainder(sim.heading, 180.0)) > SLANT_OK_DEG) gap += 90.0;
  if (gap >= sim.guard) return 0;
  sim.guard_hit = 1;
  TRACE("guard tripped at %.1f cm", gap);
  return 1;
}

static double world_heading(double h) { return Task2_WrapDeg(h + GYRO_BOOT_DEG); }

/* A side IR: distance from its lens, straight out sideways, to the nearest
 * box - obstacles and the carpark's walls alike - as task2fw's ir_read()
 * would report it: 1 inside that side's range, else 0 (*near if under the
 * 10 cm floor). */
static double ray(double x, double y, double deg, double cap);
static int ir_reading(int right, double *cm, int *near)
{
  double sx, sy;
  car_point(IR_AT_CM, right ? -IR_LENS_Y_CM : IR_LENS_Y_CM, &sx, &sy);
  const double t = ray(sx, sy, sim.heading + (right ? -90.0 : 90.0), 90.0);
  const double max = right ? T2_IR_MAX_R_CM : T2_IR_MAX_L_CM;
  *near = t < T2_IR_MIN_CM;
  if (*near || t > max) return 0;
  *cm = t;
  return 1;
}

/* The carpark's side-safety stop (op_side_guard), forward moves only. */
#define SIDE_OVERRUN_CM 2.0
static int side_trips(double d)
{
  if (sim.side_guard <= 0.0 || sim.side_hit || d <= 0.0) return 0;
  for (int k = 0; k < 2; ++k) {
    double cm; int near;
    if (ir_reading(k, &cm, &near) ? cm < sim.side_guard : near) {
      sim.side_hit = 1;
      TRACE("side IR stop (%s)", k ? "right" : "left");
      return 1;
    }
  }
  return 0;
}

/* The mouth on the way out, as task2fw's pump watches for it: both side IRs
 * on the fences, then both clear - one sample per cm here. */
static void mouth_tick(double d)
{
  if (!sim.mouth_on || sim.mouth_ok || d <= 0.0) return;
  for (int k = 0; k < 2; ++k) {
    double cm; int near;
    const int on = ir_reading(k, &cm, &near) ? cm <= T2_FENCE_SEEN_CM : near;
    if (sim.mouth_state[k] == 0) {
      sim.mouth_run[k] = on ? sim.mouth_run[k] + 1 : 0;
      if (sim.mouth_run[k] >= IR_EDGE_CONFIRM) { sim.mouth_state[k] = 1; sim.mouth_run[k] = 0; }
    } else if (!on) {
      if (!sim.mouth_run[k])   /* the sim's IR has no lag */
        sim.mouth_first[k] = odom.x + IR_AT_CM * cos(RAD(sim.heading));
      if (++sim.mouth_run[k] >= IR_EDGE_CONFIRM) {
        sim.mouth_ok = 1;
        sim.mouth_x = sim.mouth_first[k];
        return;
      }
    } else sim.mouth_run[k] = 0;
  }
}

static void step_straight(double cm)
{
  int n = (int)ceil(fabs(cm));
  double d = n ? cm / n : 0.0;
  for (int i = 0; i < n && !sim.collided; ++i) {
#ifdef DRIFT_DEG_PER_M
    /* -DDRIFT_DEG_PER_M=5: a straight that wanders off its heading, the gyro
     * seeing it (2026-09-14 14:39: 3 degrees CCW over the 62 cm home run) -
     * on the way home only, facing the carpark */
    if (fabs(remainder(sim.heading, 360.0)) > 135.0)
      sim.heading += fabs(d) * DRIFT_DEG_PER_M / 100.0;
#endif
    sim.x += d * cos(RAD(sim.heading));
    sim.y += d * sin(RAD(sim.heading));
    Task2Odom_Step(&odom, d, world_heading(sim.heading));
    check_collision();
    if (guard_trips()) {
      int stop = i + 1 + (int)ceil(GUARD_OVERRUN_CM / fmax(fabs(d), 1e-6));
      if (stop < n) n = stop;
    }
    if (side_trips(d)) {
      int stop = i + 1 + (int)ceil(SIDE_OVERRUN_CM / fmax(fabs(d), 1e-6));
      if (stop < n) n = stop;
    }
    mouth_tick(d);
  }
}

/* deg > 0 rotates the car left. backward: driven in reverse - a left lock
 * reversing rotates the car right, on the LEFT radius. */
static void step_turn(double deg, int backward, int left_lock)
{
  double r = rad[(open_turns ? 2 : 0) + !left_lock];
  int n = (int)ceil(fabs(deg));
  double dth = n ? deg / n : 0.0;
  double chord = (backward ? -2.0 : 2.0) * r * sin(RAD(fabs(dth)) / 2.0);
  for (int i = 0; i < n && !sim.collided; ++i) {
    double mid = sim.heading + dth / 2.0;
    sim.x += chord * cos(RAD(mid));
    sim.y += chord * sin(RAD(mid));
    Task2Odom_Step(&odom, chord, world_heading(mid));
    sim.heading += dth;
    check_collision();
    if (guard_trips()) {
      int stop = i + 1 + (int)ceil(GUARD_OVERRUN_CM / fmax(fabs(chord), 1e-6));
      if (stop < n) n = stop;
    }
    if (side_trips(chord)) {
      int stop = i + 1 + (int)ceil(SIDE_OVERRUN_CM / fmax(fabs(chord), 1e-6));
      if (stop < n) n = stop;
    }
    mouth_tick(chord);
  }
}

static double ray(double x, double y, double deg, double cap)
{
  double dx = cos(RAD(deg)), dy = sin(RAD(deg));
  for (double t = 0.0; t < cap; t += 0.25)
    for (int o = sim.ignore1 ? 1 : 0; o < sim.n_box; ++o)
      if (inside(&sim.box[o], x + dx * t, y + dy * t)) return t;
  return cap;
}

#define ULTRA_CONE_DEG 10
static double bumper_gap(void)
{
  double fx, fy, best = 400.0;
  car_point(FRONT_CM, 0.0, &fx, &fy);
  for (int d = -ULTRA_CONE_DEG; d <= ULTRA_CONE_DEG; d += 2)
    best = fmin(best, ray(fx, fy, sim.heading + d, best));
  return best;
}

static double sim_seconds(void) { return sim.secs + sim.clock / 1000.0; }
static void spend_turn(double deg) { sim.secs += SEC_PER_MOVE + fabs(deg) * SEC_PER_DEG; }

/* ---- Task2Ops, as task2fw.c runs them ----------------------------------- */
static void op_straight(double cm)
{
  if (fabs(cm) < 0.5) return;   /* task2fw: under 5 mm is not driven */
  if (!sim.moves++) sim.first_move = cm;
  sim.straights2 += sim.after_swerve;
  sim.secs += SEC_PER_MOVE + fabs(cm) * SEC_PER_CM;
  step_straight(cm);
  TRACE("straight %.1f", cm);
}
/* Whole degrees, like Motion_DriveArcV. */
static void op_left(double deg)
{
  double d = (double)lround(fabs(deg)) * (deg < 0 ? -1.0 : 1.0);
  if (d == 0.0) return;
  sim.moves++; sim.after_swerve = 0; spend_turn(d); step_turn(d, d < 0, 1); TRACE("left %.0f", d);
}
static void op_right(double deg)
{
  double d = (double)lround(fabs(deg)) * (deg < 0 ? -1.0 : 1.0);
  if (d == 0.0) return;
  sim.moves++; sim.after_swerve = 0; spend_turn(d); step_turn(-d, d < 0, 0); TRACE("right %.0f", d);
}

/* -DIR_OBLIQUE_DEG=n: the side IR misses the black obstacle when it looks at
 * its face more than n degrees off square - the floor run of 2026-09-14
 * 13:22, where the right IR read obstacle 2 as clear 21 degrees off square. */
static int ir_beside(int right_sensor)
{
  double sx, sy;
#ifdef IR_OBLIQUE_DEG
  if (fabs(remainder(sim.heading, 90.0)) > IR_OBLIQUE_DEG) return 0;
#endif
  car_point(IR_AT_CM, right_sensor ? -IR_LENS_Y_CM : IR_LENS_Y_CM, &sx, &sy);
  return ray(sx, sy, sim.heading + (right_sensor ? -90.0 : 90.0), IR_SEE_CM) < IR_SEE_CM;
}

static int op_edge_leg(int right_sensor, int may_be_past, double max_cm, double at[3])
{
  int seen = 0, past = may_be_past && !ir_beside(right_sensor);
  double travelled = 0.0;
  const double leg = max_cm > 0.0 ? max_cm : T2_EDGE_MAX_CM;
  if (sim.edge_side != right_sensor) sim.edge_calls++;
  sim.edge_side = right_sensor;
  if (past) {
    TRACE("edge leg already past");
    return EDGE_PAST;
  }
  for (;;) {
    if (ir_beside(right_sensor)) seen = 1;
    else if (seen) break;
    if (sim.collided) return EDGE_FAILED;
    if (travelled >= leg) {
      if (max_cm > 0.0) {   /* the end was known: stop there, no fault */
        TRACE("edge leg capped after %.0f", travelled);
        return EDGE_CAPPED;
      }
      sim.lost_edge = 1;
      return EDGE_FAILED;
    }
    step_straight(1.0);
    travelled += 1.0;
  }
  at[0] = odom.x;   /* where the edge was read: the sim's IR has no lag */
  at[1] = odom.y;
  at[2] = world_heading(sim.heading);
  {   /* task2fw: the leg behind drives on T2_EDGE_BACK_EXTRA_CM first */
    double on = EDGE_OVERRUN_CM + (may_be_past ? 0.0 : (double)T2_EDGE_BACK_EXTRA_CM);
    step_straight(on);
    sim.secs += SEC_PER_MOVE + (travelled + on) / 34.5;
  }
  TRACE("edge leg followed, end after %.0f", travelled);
  return EDGE_FOUND;
}

/* seek_edge as task2fw runs it: back along the leg until the IR sees it. */
static int op_seek_edge(int right_sensor, double max_cm, double at[3])
{
  int passing = ir_beside(right_sensor);   /* seeing something: past it first */
  for (double back = 0.0; back < max_cm; back += 1.0) {
    if (passing) {
      passing = ir_beside(right_sensor);
    } else if (ir_beside(right_sensor)) {
      at[0] = odom.x;
      at[1] = odom.y;
      at[2] = world_heading(sim.heading);
      step_straight(-SEEK_OVERRUN_CM);
      TRACE("seek back found it after %.0f", back);
      return EDGE_FOUND;
    }
    if (sim.collided) return EDGE_FAILED;
    step_straight(-1.0);
  }
  TRACE("seek back capped at %.0f", max_cm);
  return EDGE_CAPPED;
}

static int op_turn_watch(int left, double deg, int right_sensor, double check_deg, double at[3])
{
  int seen = 0;
  sim.moves++;
  sim.after_swerve = 0;
  for (double swept = 0.0; swept < deg && !sim.collided; swept += 1.0) {
    step_turn(left ? 1.0 : -1.0, 0, left);
    if (ir_beside(right_sensor))
      seen = 1;
    else if (swept + 1.0 >= check_deg) {
      at[0] = odom.x;
      at[1] = odom.y;
      at[2] = world_heading(sim.heading);
      double on = fmin(WATCH_OVERRUN_DEG, deg - swept - 1.0);
      step_turn(left ? on : -on, 0, left);
      spend_turn(swept + 1.0 + on);
      TRACE("turn watch cut after %.0f", swept + 1.0 + on);
      return 1;
    }
  }
  spend_turn(deg);
  TRACE("turn watch %s %.0f", seen ? "saw it, whole" : "whole", deg);
  return 0;
}

static int op_ultrasonic(uint16_t *cm)
{
  if (sim.stale_ultrasonic) return 0;
  if (sim.missed_echoes > 0) { sim.missed_echoes--; return 0; }
  if (sim.phantoms > 0) { sim.phantoms--; *cm = 12; TRACE("ultrasonic 12 (phantom)"); return 1; }
  if (sim.peek_reads > 0) {
    sim.peek_reads--;
    *cm = (uint16_t)lround(bumper_gap() - ULTRA_OFFSET_CM + 90.0);
    TRACE("ultrasonic %u (peek: long)", (unsigned)*cm);
    return 1;
  }
  if (sim.slant_far && fabs(remainder(sim.heading, 180.0)) > SLANT_OK_DEG) {
    *cm = (uint16_t)lround(bumper_gap() - ULTRA_OFFSET_CM + 90.0);
    TRACE("ultrasonic %u (slanted: long)", (unsigned)*cm);
    return 1;
  }
  sim.ignore1 = sim.blind1 > 0 && sim.moves > 0;
  if (sim.ignore1) sim.blind1--;
  double reading = bumper_gap() - ULTRA_OFFSET_CM;
  if (sim.entered && sim.park_short > 0) { sim.park_short--; reading -= 24.0; }
  sim.ignore1 = 0;
  *cm = (uint16_t)lround(reading < 0 ? 0 : reading);
  TRACE("ultrasonic %u", (unsigned)*cm);
  return 1;
}

static double op_heading(void) { return world_heading(sim.heading); }
static int    op_failed(void)  { return sim.collided || sim.lost_edge; }

static void op_send(const char *line)
{
  int n;
  if (sscanf(line, "P,%d", &n) == 1 && n >= 1 && n <= 2) {
    sim.prompts[n]++;
    if (!sim.asked[n]) { sim.asked[n] = 1; sim.asked_at[n] = sim.clock; }
  }
  if (!strncmp(line, "D,peek_tight,1", 14) || !strncmp(line, "D,back_short,", 13)) sim.tight = 1;
  if (!strncmp(line, "D,entry,", 8)) sim.entered = 1;
  if (sim.peek_long && !strncmp(line, "D,swerve_diag,", 14)) sim.peek_reads = 6;
  if (!strncmp(line, "D,peek_tight,", 13)) sim.peek_reads = 0;
  if (!strncmp(line, "D,swerved,", 10)) { sim.after_swerve = 1; sim.swerved = 1; sim.swerve_y = sim.y; }
  if (trace && line[0] == 'D') printf("  %s", line);
}

static char op_answer(int obstacle)
{
  if (obstacle < 1 || obstacle > 2) return 0;
  double fx, fy;
  car_point(FRONT_CM, 0.0, &fx, &fy);
  sim.photo_gap[obstacle] = sim.box[obstacle - 1].x0 - fx;
  if (sim.never_answer || !sim.asked[obstacle]) return 0;
  if (sim.clock - sim.asked_at[obstacle] < sim.answer_after[obstacle]) return 0;
  if (obstacle == 2 && sim.answer_wrong_obstacle) return 0;
  return sim.arrow[obstacle];
}

static int op_lateral(int obstacle, double *cm)
{
  if (sim.no_lateral || obstacle != 1) return 0;
  *cm = sim.lat - sim.y + LAT_ERR_CM;
  return 1;
}

static void     op_clear(void)       { }
static void     op_wait(uint32_t ms) { sim.clock += ms; }
static uint32_t op_now(void)         { return sim.clock; }
static void op_turn_speed(int open) { open_turns = open; }
static void op_hold_steering(int hold) { (void)hold; }
static void op_rebase_heading(void) { }
static void op_guard(double cm) { sim.guard = cm; sim.guard_hit = 0; }
static int  op_guarded(void) { int hit = sim.guard_hit; sim.guard_hit = 0; return hit; }
static void op_brake_hard(int hard) { (void)hard; }

static void op_side_guard(double cm) { sim.side_guard = cm; sim.side_hit = 0; }
static void op_mouth_watch(int on) { sim.mouth_on = on; }
static int  op_mouth_seen(double *x) { if (!sim.mouth_ok) return 0; *x = sim.mouth_x; return 1; }
static int  op_side_guarded(void) { int hit = sim.side_hit; sim.side_hit = 0; return hit; }
static int  op_ir(int right_sensor, double *cm, int *near) { return ir_reading(right_sensor, cm, near); }

static const Task2Ops OPS = {
  .straight = op_straight, .left = op_left, .right = op_right, .edge_leg = op_edge_leg, .seek_edge = op_seek_edge,
  .turn_watch = op_turn_watch, .turn_speed = op_turn_speed, .hold_steering = op_hold_steering,
  .brake_hard = op_brake_hard, .rebase_heading = op_rebase_heading,
  .guard = op_guard, .guarded = op_guarded,
  .side_guard = op_side_guard, .side_guarded = op_side_guarded,
  .mouth_watch = op_mouth_watch, .mouth_seen = op_mouth_seen,
  .ultrasonic = op_ultrasonic, .ir = op_ir, .heading = op_heading, .failed = op_failed,
  .send = op_send, .answer = op_answer, .lateral = op_lateral, .clear_answer = op_clear,
  .wait_ms = op_wait, .now_ms = op_now, .odom = &odom,
};

/* ---- arena ------------------------------------------------------------ */
#define BAY_DEPTH 50.0
#define BAY_HALF  30.0
static int slant_far_all, peek_long_all;
static double start_y_all;   /* the next arenas' true start off the bay's centreline */
static void arena(double gap1, double gap2, double len2, char a1, char a2, double lat)
{
  double o1 = gap1, o2 = gap1 + 10.0 + gap2;
  memset(&sim, 0, sizeof sim);
  sim.slant_far = slant_far_all;
  sim.peek_long = peek_long_all;
  sim.start_y = start_y_all;
  open_turns = 0;
  sim.edge_side = -1;
  sim.arrow[1] = a1;
  sim.arrow[2] = a2;
  sim.lat = lat;
  sim.clear = gap2 <= 60.0 ? TIGHT_CLEAR_CM : CLEAR_CM;
  sim.answer_after[1] = sim.answer_after[2] = 600;
  Box b[] = {
    {o1, o1 + 10, lat - 5, lat + 5, "obstacle 1"},
    {o2, o2 + 10, lat - len2 / 2, lat + len2 / 2, "obstacle 2"},
    {-BAY_DEPTH - 10, -BAY_DEPTH, -BAY_HALF - 10, BAY_HALF + 10, "carpark back wall"},
    {-BAY_DEPTH, 0, BAY_HALF, BAY_HALF + 10, "carpark left wall"},
    {-BAY_DEPTH, 0, -BAY_HALF - 10, -BAY_HALF, "carpark right wall"},
    /* the arena's side walls: as near as the rules allow past obstacle 2's ends.
     * -DWALL_BOX: a box there instead, in line with obstacle 2 - the floor
     * test of 2026-09-14 14:28, where the IR took it for obstacle 2's end */
#ifdef WALL_BOX
    {o2, o2 + 10, lat + len2 / 2 + WALL_SIM_CM, lat + len2 / 2 + WALL_SIM_CM + 30, "left wall box"},
    {o2, o2 + 10, lat - len2 / 2 - WALL_SIM_CM - 30, lat - len2 / 2 - WALL_SIM_CM, "right wall box"},
#else
    {-BAY_DEPTH - 10, o2 + 300, lat + len2 / 2 + WALL_SIM_CM, lat + len2 / 2 + WALL_SIM_CM + 10, "left wall"},
    {-BAY_DEPTH - 10, o2 + 300, lat - len2 / 2 - WALL_SIM_CM - 10, lat - len2 / 2 - WALL_SIM_CM, "right wall"},
#endif
  };
  memcpy(sim.box, b, sizeof b);
  sim.n_box = (int)(sizeof b / sizeof *b);
#ifdef START_TAIL_CM
  /* -DSTART_TAIL_CM=n: started at the back, tail n cm off the back wall,
   * as the floor runs place it */
  sim.x = -BAY_DEPTH + START_TAIL_CM + REAR_CM;
#else
  sim.x = -BAY_DEPTH / 2.0 - (FRONT_CM - REAR_CM) / 2.0;   /* centred, facing out */
#endif
  sim.y = sim.start_y;
  /* what Task2Fw_Run does: plan on the default radius */
#ifdef PLAN_TRUE_ARCS
  /* -DPLAN_TRUE_ARCS: the table holds the radii the car really turns at */
  Task2Mission_SetArcs(rad[0], rad[1]);
#else
  Task2Mission_SetArcs(T2_ARC_DEFAULT_CM, T2_ARC_DEFAULT_CM);
#endif
}

static int parked(void)
{
  const double c[4][2] = {{-REAR_CM, HALF_WID_CM}, {-REAR_CM, -HALF_WID_CM},
                          {FRONT_CM, HALF_WID_CM}, {FRONT_CM, -HALF_WID_CM}};
  for (int k = 0; k < 4; ++k) {
    double wx, wy;
    car_point(c[k][0], c[k][1], &wx, &wy);
    if (!(wx > -BAY_DEPTH && wx < 0 && wy > -BAY_HALF && wy < BAY_HALF)) return 0;
  }
  return 1;
}

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; \
  printf("FAIL %s:%d  ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- tests ------------------------------------------------------------ */

/* Protocol 11 tokens (commands.c), as RpiLink_Poll() / RpiLink_Service() see
 * them - already lowercased. */
static void test_tokens(void)
{
  Command_t c = Cmd_ParseToken("!t2go170");
  CHECK(c.op == CMD_T2_GO && c.arg == 170, "!t2go170");
  CHECK(!Cmd_IsImmediate(CMD_T2_GO), "!t2go is not answered immediately");
  const char *bad_go[] = {"!t2go", "!t2go0", "!t2go181", "!t2go05", "!t2gox"};
  for (unsigned i = 0; i < sizeof bad_go / sizeof *bad_go; ++i)
    CHECK(Cmd_ParseToken(bad_go[i]).op == CMD_INVALID, "%s must be rejected", bad_go[i]);
  CHECK(!Cmd_ParseLine("!t2go170,f10"), "!t2go inside a line must be refused");
  CHECK(!Cmd_ParseLine("f10,!t2go170"), "!t2go inside a line must be refused");
  Cmd_QueueFlush();

  c = Cmd_ParseToken("!t2a11");
  CHECK(c.op == CMD_T2_ARROW && c.arg == 11 && Cmd_IsImmediate(c.op), "!t2a11 = obstacle 1 right");
  c = Cmd_ParseToken("!t2a20");
  CHECK(c.op == CMD_T2_ARROW && c.arg == 20, "!t2a20 = obstacle 2 left");
  const char *bad_a[] = {"!t2a", "!t2a1", "!t2a00", "!t2a30", "!t2a12", "!t2a110"};
  for (unsigned i = 0; i < sizeof bad_a / sizeof *bad_a; ++i)
    CHECK(Cmd_ParseToken(bad_a[i]).op == CMD_INVALID, "%s must be rejected", bad_a[i]);

  c = Cmd_ParseToken("!t2y1580");
  CHECK(c.op == CMD_T2_LATERAL && c.arg / 1000 == 1 && c.arg % 1000 - 500 == 80, "!t2y1580 = 1, 80 mm left");
  c = Cmd_ParseToken("!t2y2420");
  CHECK(c.op == CMD_T2_LATERAL && c.arg / 1000 == 2 && c.arg % 1000 - 500 == -80, "!t2y2420 = 2, 80 mm right");
  const char *bad_y[] = {"!t2y", "!t2y158", "!t2y3500", "!t2y15800", "!t2y1x00"};
  for (unsigned i = 0; i < sizeof bad_y / sizeof *bad_y; ++i)
    CHECK(Cmd_ParseToken(bad_y[i]).op == CMD_INVALID, "%s must be rejected", bad_y[i]);

  CHECK(CMD_PROTOCOL_VERSION >= 11, "protocol version not bumped");
}

static void test_wrap(void)
{
  CHECK(fabs(Task2_WrapDeg(190) - -170) < 1e-9, "wrap 190");
  CHECK(fabs(Task2_WrapDeg(-180) - 180) < 1e-9, "wrap -180");
  CHECK(fabs(Task2_TurnToward(170, -170) - 20) < 1e-9, "shortest turn across +/-180");
}

static double widest_swing, widest_landing;
static double worst_park_off;        /* over every course: cm off the bay's centreline, parked */
static double worst_park_us;         /* ...and the furthest the ultrasonic read from PARK_US */
/* Parked: inside the carpark, the body at least FENCE_ROOM off either side
 * fence, and the ultrasonic within PARK_US_SLACK of PARK_US (short of it by
 * no more than the nudge tolerance). */
#define FENCE_ROOM_CM 3.0
#define PARK_US_SLACK_CM 1.5

static void test_plan_csc(void)
{
  const double cases[][6] = {
    {0, 0, 0, 100, 0, 0}, {0, 0, 0, 0, 2 * 30.0, 180}, {200, 40, 90, 60, 25, 180},
    {100, -25, 180, 50, 0, 180}, {0, 0, 90, 10, -80, -90}, {250, 60, 90, 65, 0, 180},
  };
  const double radii[][4] = {{30, 30, 30, 30}, {28, 32, 28, 32}};
  for (unsigned j = 0; j < 2; ++j)
  for (unsigned i = 0; i < sizeof cases / sizeof *cases; ++i) {
    double plan[3], x = cases[i][0], y = cases[i][1], h = cases[i][2];
    const double *r0 = radii[j], *r1 = radii[j] + 2;
    if (!Task2_PlanCSC(cases[i], cases[i] + 3, r0, r1, plan)) { CHECK(0, "radii %u case %u: no plan", j, i); continue; }
    for (int k = 0; k < 3; k += 2) {
      double c = plan[k] > 0 ? 1.0 : -1.0;
      double r = (k ? r1 : r0)[plan[k] > 0 ? 0 : 1];
      double cx = x - c * r * sin(RAD(h)), cy = y + c * r * cos(RAD(h));
      h += plan[k];
      x = cx + c * r * sin(RAD(h));
      y = cy - c * r * cos(RAD(h));
      if (!k) { x += plan[1] * cos(RAD(h)); y += plan[1] * sin(RAD(h)); }
    }
    CHECK(hypot(x - cases[i][3], y - cases[i][4]) < 0.1 && fabs(Task2_WrapDeg(h - cases[i][5])) < 0.1,
          "radii %u case %u: plan {%.1f, %.1f, %.1f} lands at (%.1f,%.1f,%.1f)",
          j, i, plan[0], plan[1], plan[2], x, y, h);
  }
}

static int run_course(double gap1, double gap2, double len2, char a1, char a2, double lat)
{
  arena(gap1, gap2, len2, a1, a2, lat);
  double x0 = sim.x;
  int result = Task2Mission_Run(&OPS);
  char label[128];
  snprintf(label, sizeof label, "R%.0f/%.0f/%.0f/%.0f gap1 %.0f gap2 %.0f len2 %.0f arrows %c%c lat %.0f",
           rad[0], rad[1], rad[2], rad[3], gap1, gap2, len2, a1, a2, lat);
  int before = failures;
  CHECK(!sim.collided, "%s: hit %s", label, sim.hit);
  CHECK(!sim.lost_edge, "%s: side IR never found obstacle 2's end", label);
  CHECK(result == T2M_OK, "%s: course returned %d", label, result);
  CHECK(sim.edge_side == (a2 == 'L'), "%s: edge follow used the wrong sensor", label);
  if (sim.swerved && !sim.tight) {
    double off = (a1 == 'L' ? 1.0 : -1.0) * (sim.swerve_y - (lat + LAT_ERR_CM));
    if (off > widest_landing) widest_landing = off;
    CHECK(off <= T2_SWERVE_LAND_CM + 1.0, "%s: squared up %.1f cm off obstacle 1's line", label, off);
  }
  double swing = sim.reach - len2 / 2.0;
  if (swing > widest_swing) widest_swing = swing;
  CHECK(swing <= SIDE_ROOM_CM, "%s: swung %.0f cm past obstacle 2's end", label, swing);
  if (result == T2M_OK) {
    CHECK(parked(), "%s: not parked - axle (%.0f,%.0f) heading %.0f", label, sim.x, sim.y, sim.heading);
    CHECK(fabs(odom.x - (sim.x - x0)) < 1.0 && fabs(odom.y - (sim.y - sim.start_y)) < 1.0,
          "%s: odometry (%.1f,%.1f) drifted from truth (%.1f,%.1f)", label, odom.x, odom.y,
          sim.x - x0, sim.y - sim.start_y);
    if (fabs(sim.y) > worst_park_off) worst_park_off = fabs(sim.y);
    double side = 0.0;
    for (int k = 0; k < 4; ++k) {
      double wx, wy;
      car_point(k & 1 ? FRONT_CM : -REAR_CM, k & 2 ? HALF_WID_CM : -HALF_WID_CM, &wx, &wy);
      side = fmax(side, fabs(wy));
    }
    CHECK(side <= BAY_HALF - FENCE_ROOM_CM, "%s: parked %.1f cm off the bay's centreline, %.1f cm off a fence",
          label, sim.y, BAY_HALF - side);
    const double us = bumper_gap() - ULTRA_OFFSET_CM - T2_PARK_US_CM;
    if (fabs(us) > fabs(worst_park_us)) worst_park_us = us;
    CHECK(us >= -PARK_US_SLACK_CM - 2.0 && us <= T2_PARK_TOL_CM + PARK_US_SLACK_CM,
          "%s: parked with the ultrasonic at %.1f cm", label, us + T2_PARK_US_CM);
  }
  return failures == before;
}

/* The matrix and the hostile-echo block are scored, not all-or-nothing: the
 * minimum 60 cm gap with the shortest obstacle 2 off to one side has no room
 * for this car on every arc. Each must not fall below where the course is
 * now - a change that loses layouts fails the test. Raise them as the course
 * improves. Individual misses are still printed. */
#ifndef MIN_COURSES
#define MIN_COURSES 1645     /* of 1728 (2026-10-16, walls 50 cm past obstacle 2: 1650 - every wall hit at the 60 cm gap; 1719 with walls at 80) */
#endif
#ifndef MIN_PEEK_LONG
#define MIN_PEEK_LONG 21     /* of 24 (2026-10-16: 21 with walls 50 cm past obstacle 2; 24 at 80) */
#endif
static int course_failures, matrix_short, peek_long_short;
static void test_courses(void)
{
  const char arrows[4][2] = {{'R', 'R'}, {'R', 'L'}, {'L', 'R'}, {'L', 'L'}};
  const double gaps[] = {60, 105, 150}, lens[] = {30, 60, 100, 140}, lats[] = {-8, 0, 8};
  int passed = 0, total = 0;
  double time_sum = 0.0, time_max = 0.0;
  int before = failures;
  for (unsigned r = 0; r < sizeof RADII / sizeof *RADII; ++r)
  for (int a = 0; a < 4; ++a)
    for (int g1 = 0; g1 < 3; ++g1)
      for (int g2 = 0; g2 < 3; ++g2)
        for (int l = 0; l < 4; ++l)
          for (int t = 0; t < 3; ++t) {
            memcpy(rad, RADII[r], sizeof rad);
            total++;
            if (!run_course(gaps[g1], gaps[g2], lens[l], arrows[a][0], arrows[a][1], lats[t])) continue;
            passed++;
            double s = sim_seconds();
            time_sum += s;
            if (s > time_max) time_max = s;
          }
  course_failures = failures - before;
  failures = before;                   /* scored below, not counted one by one */
  matrix_short = passed < MIN_COURSES;
  printf("courses completed: %d / %d; widest swing past obstacle 2's end %.0f cm; "
         "furthest off the line after the swerve %.1f cm\n", passed, total, widest_swing, widest_landing);
  printf("modelled time: mean %.1f s, worst %.1f s; parked at most %.1f cm off centre, "
         "the ultrasonic at most %+.1f cm off %.0f\n",
         time_sum / (passed ? passed : 1), time_max, worst_park_off, worst_park_us, T2_PARK_US_CM);
}

static void test_photo_gaps(void)
{
  memcpy(rad, RADII[0], sizeof rad);
  run_course(105, 105, 60, 'R', 'R', 0);
  CHECK(fabs(sim.photo_gap[1] - T2_PHOTO_1_CM) < 1.5, "photo 1 taken at %.1f cm", sim.photo_gap[1]);
}

static void test_slow_arrow2(void)
{
  for (int k = 0; k < 2; ++k) {
    memcpy(rad, RADII[0], sizeof rad);
    arena(105, 105, 60, 'R', k ? 'L' : 'R', 0);
    sim.answer_after[2] = 3000;
    int result = Task2Mission_Run(&OPS);
    CHECK(result == T2M_OK && !sim.collided, "slow arrow 2 (%c): course %d, hit %s", k ? 'L' : 'R', result, sim.hit);
  }
}

static void test_never_guesses(void)
{
  arena(105, 105, 60, 'R', 'R', 0);
  sim.never_answer = 1;
  CHECK(Task2Mission_Run(&OPS) == T2M_NO_ARROW, "no answer must stop the course");
  CHECK(sim.prompts[1] >= 2, "it should have asked again, asked %d time(s)", sim.prompts[1]);
  CHECK(sim.edge_calls == 0, "it moved on without an arrow");
  CHECK(sim.clock >= T2_ANSWER_TIMEOUT_MS, "it gave up after %u ms", (unsigned)sim.clock);
}

static void test_wrong_obstacle_answer_ignored(void)
{
  arena(105, 105, 60, 'R', 'L', 0);
  sim.answer_wrong_obstacle = 1;
  CHECK(Task2Mission_Run(&OPS) == T2M_NO_ARROW, "obstacle 1's answer must not steer obstacle 2");
  CHECK(sim.edge_calls == 0, "obstacle 2 was rounded on an answer that was not for it");
}

static void test_stale_ultrasonic(void)
{
  arena(105, 105, 60, 'R', 'R', 0);
  double x0 = sim.x;
  sim.stale_ultrasonic = 1;
  CHECK(Task2Mission_Run(&OPS) == T2M_NO_ULTRASONIC, "a stale range must not be driven on");
  CHECK(sim.x == x0, "it moved without a range");
  CHECK(sim.clock >= T2_RANGE_WAIT_MS, "gave up after %u ms, not waiting", (unsigned)sim.clock);
  arena(105, 105, 60, 'R', 'R', 0);
  sim.missed_echoes = 3;
  CHECK(Task2Mission_Run(&OPS) == T2M_OK, "three missed echoes ended the run");
}

/* Started off the bay's centreline - the odometry says centred - the side
 * IRs at the start must see it, and the car park clear of both fences. */
static void test_centring(void)
{
  int ok = 0, n = 0;
  const double offs[] = {-8, -5, 5, 8};
  for (unsigned i = 0; i < sizeof offs / sizeof *offs; ++i)
    for (int a = 0; a < 4; ++a) {
      memcpy(rad, RADII[0], sizeof rad);
      start_y_all = offs[i];
      n++;
      ok += run_course(105, 105, 60, a & 1 ? 'L' : 'R', a & 2 ? 'L' : 'R', 0);
    }
  start_y_all = 0.0;
  printf("off-centre starts: %d / %d parked clear of the fences\n", ok, n);
}

static void test_bad_echoes(void)
{
  memcpy(rad, RADII[0], sizeof rad);
  for (int k = 0; k < 2; ++k) {
    arena(105, 105, 60, 'R', 'L', 0);
    sim.phantoms = 1 + k;
    int result = Task2Mission_Run(&OPS);
    CHECK(sim.moves == 0 || sim.first_move > 0, "%d phantom(s): the first move was %.0f cm", 1 + k, sim.first_move);
    CHECK(!sim.collided, "%d phantom(s): hit %s", 1 + k, sim.hit);
    if (!k) CHECK(result == T2M_OK, "one phantom echo ended the run (%d)", result);
  }
  for (int lat = -8; lat <= 8; lat += 8) {
    arena(150, 105, 60, 'L', 'R', lat);
    sim.blind1 = 4;
    CHECK(Task2Mission_Run(&OPS) == T2M_OK && !sim.collided, "lat %d: a re-check that missed obstacle 1 - hit %s", lat, sim.hit);
  }
  for (int k = 0; k < 4; ++k) {
    arena(105, 105, 60, k & 1 ? 'L' : 'R', k & 2 ? 'L' : 'R', 0);
    sim.park_short = 2;
    int result = Task2Mission_Run(&OPS);
    CHECK(result == T2M_OK && parked(), "short echo at the carpark (%d): course %d, axle (%.0f,%.0f)", k, result, sim.x, sim.y);
  }
  {
    int ok = 0, n = 0;
    for (int k = 0; k < 4; ++k)
      for (int len = 30; len <= 140; len += 110)
        for (int lat = -8; lat <= 8; lat += 8) {
          arena(105, 60, len, k & 1 ? 'L' : 'R', k & 2 ? 'L' : 'R', lat);
          sim.slant_far = 1;
          sim.peek_long = 1;
          int result = Task2Mission_Run(&OPS);
          n++;
          if (!sim.collided && result == T2M_OK) { ok++; continue; }
          printf("miss: peeks heard long, arrows %c%c len2 %d lat %d: course %d, hit %s\n",
                 k & 1 ? 'L' : 'R', k & 2 ? 'L' : 'R', len, lat, result, sim.hit);
        }
    printf("peeks heard long: %d / %d completed\n", ok, n);
    peek_long_short = ok < MIN_PEEK_LONG;
  }
  arena(105, 105, 60, 'R', 'R', 0);
  sim.no_lateral = 1;
  CHECK(Task2Mission_Run(&OPS) == T2M_OK && !sim.collided, "no lateral: hit %s", sim.hit);
}

int main(int argc, char **argv)
{
  memcpy(rad, RADII[0], sizeof rad);
  if (argc == 7 || argc == 8) {
    trace = 1;
    slant_far_all = getenv("SLANT_FAR") != NULL;
    peek_long_all = getenv("PEEK_LONG") != NULL;
    start_y_all = getenv("START_Y") ? atof(getenv("START_Y")) : 0.0;   /* START_Y=6: off centre */
    sscanf(argv[1], "%lf/%lf/%lf/%lf", &rad[0], &rad[1], &rad[2], &rad[3]);
    run_course(atof(argv[2]), atof(argv[3]), atof(argv[4]), argv[5][0], argv[6][0], argc == 8 ? atof(argv[7]) : 0.0);
    printf("modelled time %.1f s\n", sim_seconds());
    return failures != 0;
  }
  Cmd_Init();
  test_tokens();
  test_wrap();
  test_plan_csc();
  test_photo_gaps();
  test_slow_arrow2();
  test_never_guesses();
  test_wrong_obstacle_answer_ignored();
  test_stale_ultrasonic();
  test_bad_echoes();
  test_centring();
  test_courses();
  if (matrix_short) printf("FAIL the course matrix fell below MIN_COURSES (%d)\n", MIN_COURSES);
  if (peek_long_short) printf("FAIL hostile echoes fell below MIN_PEEK_LONG (%d)\n", MIN_PEEK_LONG);
  printf("%d matrix check(s) missed\n", course_failures);
  if (failures || matrix_short || peek_long_short) {
    printf("\n%d other check(s) failed\n", failures);
    return 1;
  }
  puts("task2 mission: all checks passed");
  return 0;
}
