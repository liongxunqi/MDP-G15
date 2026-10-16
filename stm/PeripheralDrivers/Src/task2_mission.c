/*
 * Task 2 course - see task2_mission.h for the contract and why it lives here.
 *
 * Ported from the MDP_WORKING_2 firmware (their Core/Src/task2_mission.c),
 * which ran the whole course on that car's STM. The course logic below is
 * theirs, kept as close to unchanged as possible so the two can still be
 * compared line by line. What changed for this robot:
 *   - constants come from task2_params.h (this car's geometry), not their
 *     car_params.h;
 *   - the turn radii are set once per run by Task2Mission_SetArcs() from the
 *     calibration table, and are not re-learned from the turns
 *     (T2_ARC_LEARN 0): calibration is frozen during a task here;
 *   - the waits for a reading from after a move are T2_ULTRA_SETTLE_MS, which
 *     covers this HC-SR04 driver's three-ping median.
 * The robot side - every Task2Ops function - is task2fw.c.
 *
 * Driven from encoder + gyro odometry in the frame the course started in (x
 * toward the obstacles, y to the left, cm). The route is the one the floor
 * video run drove, in one flowing line with no reversing:
 *   1. ask for arrow 1 and drive up to obstacle 1 while the Pi looks. The Pi
 *      also says where the arrow sat in the photo, which places obstacle 1
 *      sideways - the line both obstacles stand on (axis_y). On the floor it
 *      was up to 8 cm off the car's centre line.
 *   2. swerve past obstacle 1 FORWARD in a fixed S - out 45 on the arrow's
 *      side, back 90, square up 45 - landing back on the line past it;
 *   3. ask for arrow 2, drive up to obstacle 2, round it by the side IR - turn
 *      to the arrow's side, along the face, round the end, along the back;
 *   4. carry the turn on round the far end to face home, run home along that
 *      side - outside obstacle 1 - and one S onto the carpark's centre line,
 *      then in on the ultrasonic.
 * Every distance to an obstacle is the one measured at its stop, not the
 * target the stop aimed for.
 */
#include "task2_mission.h"

#include <math.h>
#include <stdio.h>

#include "task2_params.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG(r) ((r) * 180.0 / M_PI)
#define RAD(d) ((d) * M_PI / 180.0)
#define SLIVER_DEG 1.0 /* a turn this small is rounding, not a turn */

/* One run's state. The module runs one course at a time. */
static double axis_y;  /* the line the obstacles stand on */
static int speed_open; /* the turn speed last asked for */
static double face2_peek; /* obstacle 2's face as the peek placed it (odometry x): as near as it can be */
static double face2_far;  /* ... and as far as it can be */
static int peek_tight;    /* no room to come back onto the line: squared up off it */
/* Obstacle 2 as the side IR placed it (odometry x / y), for the leg behind
 * it, the way home and the walls: its face, and each end where an IR read it
 * (meas_lo / meas_hi, the -y and +y ends; NO_END until read). recompute_end2()
 * turns those into the span the car keeps off (end2_lo..end2_hi: an end not
 * read mirrored about obstacle 1's line, T2_OBST2_OFFSET_CM further out for
 * how far off that line obstacle 2 may stand) and the nearest the arena walls
 * can be (wall_lo / wall_hi: T2_WALL_PAST_CM past the nearest each end can
 * be). */
#define NO_END 1e9
static double face2_x;
static int have_end2;          /* end2_lo / end2_hi are meaningful */
static int assume_max2;        /* no end read: taken as wide as it can be */
static double meas_lo, meas_hi;
static double end2_lo, end2_hi;
static double wall_lo, wall_hi;
/* The carpark: its centreline (odometry y) from the side IRs on its fences
 * before the start, and its mouth (odometry x) where they lost them on the
 * way out (mouth_watch). */
static double bay_y;
static int have_mouth;
static double mouth_x;
/* Squared up off obstacle 1's line (peek_tight) with the ultrasound straight
 * ahead hearing nothing at obstacle 2's face: obstacle 2 does not reach the
 * car's line, so its end on that side is no further out than us_end_bound -
 * the car's y, half the beam's spread at that range in from it. */
static int have_end_bound;
static double us_end_bound;

/* ------------------------------------------------------------ pure helpers -- */

double Task2_WrapDeg(double deg)
{
  deg = fmod(deg, 360.0);
  if (deg <= -180.0)
    deg += 360.0;
  if (deg > 180.0)
    deg -= 360.0;
  return deg;
}

double Task2_TurnToward(double from_deg, double to_deg)
{
  return Task2_WrapDeg(to_deg - from_deg);
}

void Task2Odom_Reset(volatile Task2Odom *o, double start_heading_deg)
{
  o->h0 = (float)start_heading_deg;
  o->x = 0.0f;
  o->y = 0.0f;
}

void Task2Odom_Step(volatile Task2Odom *o, double cm, double heading_deg)
{
  /* Called every control tick with that tick's distance. At 10 ms the heading
   * barely changes inside one step, so integrating along it is exact enough
   * even through the arc of a turn - which is what lets the return trust it. */
  double h = RAD(heading_deg - o->h0);
  o->x = (float)(o->x + cm * cos(h));
  o->y = (float)(o->y + cm * sin(h));
}

int Task2_ParseAnswer(const uint8_t frame[5], int *obstacle, char *arrow)
{
  /* PF0<n><d>: n is the obstacle (1 or 2), d is 0 left / 1 right. Anything
   * else is not an answer, so a corrupted frame can never become a turn. */
  if (frame[0] != 'P' || frame[1] != 'F' || frame[2] != '0')
    return 0;
  if (frame[3] != '1' && frame[3] != '2')
    return 0;
  if (frame[4] != '0' && frame[4] != '1')
    return 0;
  *obstacle = frame[3] - '0';
  *arrow = frame[4] == '1' ? 'R' : 'L';
  return 1;
}

int Task2_ParseLateral(const uint8_t frame[5], int *obstacle, int *mm)
{
  /* Y<F|B><ddd>: where the arrow sat sideways, ddd - 500 mm from the camera's
   * centre line, left positive. F is obstacle 1, B obstacle 2. */
  if (frame[0] != 'Y' || (frame[1] != 'F' && frame[1] != 'B'))
    return 0;
  for (int i = 2; i < 5; ++i)
    if (frame[i] < '0' || frame[i] > '9')
      return 0;
  *obstacle = frame[1] == 'F' ? 1 : 2;
  *mm = (frame[2] - '0') * 100 + (frame[3] - '0') * 10 + (frame[4] - '0') - 500;
  return 1;
}

/* Degrees to turn from `from` to `to` going one way round: [0, 360). A sliver
 * under SLIVER_DEG counts as none, or a 0.2 degree overshoot would become a
 * 359.8 degree loop. */
static double sweep(double from, double to, int left)
{
  double a = fmod(left ? to - from : from - to, 360.0);
  if (a < 0.0)
    a += 360.0;
  return (a < SLIVER_DEG || a > 360.0 - SLIVER_DEG) ? 0.0 : a;
}

/* One turn-straight-turn word: w bit 0 = first turn left, bit 1 = last turn
 * left. r0 / r1 are the first / last arc's radii, {left, right}: the car
 * turns tighter one way than the other, and the first arc may run at a
 * different speed from the last. Each circle's centre sits its radius to the
 * side the car turns toward; the straight is their common tangent, offset k
 * from the line through the centres. Returns the path's length, < 0 if this
 * word cannot join the poses. */
static double plan_word(const double from[3], const double to[3],
                        const double r0[2], const double r1[2], int w, double plan[3])
{
  int l0 = w & 1, l1 = (w >> 1) & 1;
  double ra = l0 ? r0[0] : r0[1], rb = l1 ? r1[0] : r1[1];
  double h0 = RAD(from[2]), h1 = RAD(to[2]);
  double s0 = l0 ? 1.0 : -1.0, s1 = l1 ? 1.0 : -1.0;
  double cx0 = from[0] - s0 * ra * sin(h0), cy0 = from[1] + s0 * ra * cos(h0);
  double cx1 = to[0] - s1 * rb * sin(h1), cy1 = to[1] + s1 * rb * cos(h1);
  double dx = cx1 - cx0, dy = cy1 - cy0, d = hypot(dx, dy);
  double k = s0 * ra - s1 * rb;
  if (d < fabs(k))
    return -1.0;
  double run = sqrt(d * d - k * k);
  double phi = atan2(dy, dx) + atan2(k, run);
  double a0 = sweep(from[2], DEG(phi), l0), a1 = sweep(DEG(phi), to[2], l1);
  plan[0] = s0 * a0;
  plan[1] = run;
  plan[2] = s1 * a1;
  return RAD(a0) * ra + RAD(a1) * rb + run;
}

int Task2_PlanCSC(const double from[3], const double to[3],
                  const double r0[2], const double r1[2], double plan[3])
{
  /* The shortest of the four words (Dubins' CSC paths). */
  double best = -1.0, p[3];
  for (int w = 0; w < 4; ++w)
  {
    double cost = plan_word(from, to, r0, r1, w, p);
    if (cost >= 0.0 && (best < 0.0 || cost < best))
    {
      best = cost;
      plan[0] = p[0];
      plan[1] = p[1];
      plan[2] = p[2];
    }
  }
  return best >= 0.0;
}

/* -------------------------------------------------------------- motion ---- */

/* Heading measured from where the course started, not from power-on. */
static double heading(const Task2Ops *ops)
{
  return Task2_WrapDeg(ops->heading() - ops->odom->h0);
}

/* One line per step for the Pi's run log - D,<step>,<value>,<x>,<y>,<heading>
 * in cm and degrees from the start pose - so a run that goes wrong says where.
 * About 3 ms of UART per line, a few dozen lines a run. */
static void step(const Task2Ops *ops, const char *name, long value)
{
  char line[56];
  snprintf(line, sizeof line, "D,%s,%ld,%ld,%ld,%ld\n", name, value,
           lround(ops->odom->x), lround(ops->odom->y), lround(heading(ops)));
  ops->send(line);
}

/* The arc the rear axle drives on each steering lock, by speed: [open][left].
 * It starts at the floor's T2_ARC_* and is re-measured from every turn of
 * ARC_LEARN_DEG or more this run - chord and angle from the odometry - so what
 * is planned later (the stop at obstacle 2, the route home) is planned for
 * the turns this car is making today, not an average of other days. */
static double arcs[2][2];
static int arcs_seen[2][2];

/* Set by Task2Mission_SetArcs() before the run: {right, left}. */
static double arc_set[2] = {T2_ARC_DEFAULT_CM, T2_ARC_DEFAULT_CM};

void Task2Mission_SetArcs(double left_cm, double right_cm)
{
  arc_set[1] = left_cm;
  arc_set[0] = right_cm;
}

static double arc_base(int left, int open)
{
  return arc_set[left ? 1 : 0] + (open ? T2_ARC_OPEN_EXTRA_CM : 0.0);
}

static double arc(int left, int open)
{
  /* A fast turn is never tighter than a slow one on the same lock, so a
   * slow turn measured wider than the fast default raises both. */
  return open ? fmax(arcs[1][left], arcs[0][left]) : arcs[0][left];
}

static void reset_arcs(void)
{
  for (int open = 0; open < 2; ++open)
    for (int left = 0; left < 2; ++left)
    {
      arcs[open][left] = arc_base(left, open);
      arcs_seen[open][left] = 0;
    }
}

static void set_speed(const Task2Ops *ops, int open)
{
  speed_open = open;
  ops->turn_speed(open);
}

/* deg on the left or right lock (negative = in reverse), at the speed last
 * set, measuring the arc it drove. The mean of this run's measurements
 * replaces the starting value; one wild reading (a slipping wheel) can move it
 * at most twice the open spread. */
static void learn_arc(const Task2Ops *ops, int left, double x0, double y0, double h0)
{
  volatile Task2Odom *o = ops->odom;
  double swept = fabs(Task2_WrapDeg(heading(ops) - h0));
  if (swept >= T2_ARC_LEARN_DEG)
  {
    double r = hypot(o->x - x0, o->y - y0) / (2.0 * sin(RAD(swept) / 2.0));
    double base = arc_base(left, speed_open);
    r = fmin(fmax(r, base - 2.0 * T2_ARC_OPEN_ERR_CM), base + 2.0 * T2_ARC_OPEN_ERR_CM);
    /* Measured and logged either way - the run log is how the radii get
     * checked - but only learned with T2_ARC_LEARN. */
    if (T2_ARC_LEARN)
    {
      int n = arcs_seen[speed_open][left]++;
      arcs[speed_open][left] = n ? (arcs[speed_open][left] * n + r) / (n + 1) : r;
    }
    step(ops, left ? "arc_l" : "arc_r", lround(r));
  }
}

static int turn(const Task2Ops *ops, int left, double deg)
{
  volatile Task2Odom *o = ops->odom;
  double x0 = o->x, y0 = o->y, h0 = heading(ops);
  if (left)
    ops->left(deg);
  else
    ops->right(deg);
  if (ops->failed())
    return 0;
  learn_arc(ops, left, x0, y0, h0);
  return 1;
}

/* A turn worked out from heading(): it counts from where the car points now,
 * and so does the straight after it, even with no turn at all. The firmware
 * otherwise adds it to the heading the last move aimed for - and the hard stop
 * behind obstacle 2 twists the car off that by 3-5 degrees: on the floor the
 * turn home then ended 4 degrees off its plan and the car reached the carpark
 * 9 cm off its line (2026-10-15 05:32, and 04:23, 05:29). */
static int turn_by(const Task2Ops *ops, double deg)
{
  ops->rebase_heading();
  if (deg > 0.5)
    return turn(ops, 1, deg);
  if (deg < -0.5)
    return turn(ops, 0, -deg);
  return !ops->failed();
}

/* ------------------------------------------------------------- ranging ---- */

/* A fresh ultrasonic reading. One missed echo is not a blind sensor: the
 * sensor fires every 100 ms and a cycle can miss, especially just after a
 * turn. Wait a few cycles before calling it - but never return a stale one. */
static int fresh_range(const Task2Ops *ops, uint16_t *cm)
{
  uint32_t since = ops->now_ms();
  while (!ops->ultrasonic(cm))
  {
    if (ops->failed() || ops->now_ms() - since >= T2_RANGE_WAIT_MS)
    {
      step(ops, "no_range", -1);
      return 0;
    }
    ops->wait_ms(20);
  }
  step(ops, "range", *cm);
  return 1;
}

/* Median of three readings, a sensor cycle apart. One phantom echo - the
 * floor, a foot, a reflection - is outvoted instead of driven on. Two that
 * agree within RANGE_AGREE_CM already outvote any third, so the third is only
 * read when they do not: a sensor cycle saved at every stop that ranges (on
 * the floor all three agreed every time, 2026-10-15). It takes the nearer. */
static int median_range(const Task2Ops *ops, uint16_t *cm)
{
  uint16_t v[3];
  for (int i = 0; i < 3; ++i)
  {
    if (i)
      ops->wait_ms(ULTRA_PERIOD_MS + 10U);
    if (!fresh_range(ops, &v[i]))
      return 0;
    if (i == 1 && (v[0] > v[1] ? v[0] - v[1] : v[1] - v[0]) <= T2_RANGE_AGREE_CM)
    {
      *cm = v[0] < v[1] ? v[0] : v[1];
      step(ops, "median", *cm);
      return 1;
    }
  }
  uint16_t lo = v[0] < v[1] ? v[0] : v[1], hi = v[0] < v[1] ? v[1] : v[0];
  *cm = v[2] < lo ? lo : (v[2] > hi ? hi : v[2]);
  step(ops, "median", *cm);
  return 1;
}

/* Stop with the bumper target_cm off whatever is ahead; return where its face
 * is (odometry x). One measurement, one straight move - the U command's
 * convention, offset included - then, after a long move or from the sensor's
 * near limit, one check from standstill and a correction.
 *
 * The check is not trusted blindly. On the floor a car 35 cm short of obstacle
 * 1 re-read 109 cm - the narrow obstacle had slipped out of the beam - and
 * drove 82 cm through it. A check that disagrees with the move just made by
 * more than RECHECK_MAX_CM gets a median of three; if that still disagrees,
 * the wheels are believed over the echo. `start` is the first approach from
 * the carpark: its reading is a median too, and it may not back up - the
 * carpark wall is behind it. rear_x is where the tail should stay ahead of
 * (obstacle 1's clearance, for the stop at obstacle 2). Short of room, the
 * shortfall is shared: half off the target, half - up to half the clearance -
 * into rear_x's margin. At the arena's 60 cm gap that is ~3 cm off each.
 *
 * Later approaches (obstacle 2, the carpark) read a median too, from after
 * the car has stopped - a reading from the last turn, angled, put the stop
 * at obstacle 2 too close. A short move needs no check: from a median at
 * standstill the wheels place the face to a centimetre, and the check was a
 * visible pause before every turn at obstacle 2. Every gap is from the
 * distance the wheels actually drove, not the one asked for: an overshoot
 * counted as asked placed the face that much too far.
 *
 * tol: a move within it is not driven - on the floor 1-2 cm nudges went back
 * and forth before every turn. forward_only (the carpark): never reverse.
 * Already inside the target, the car just stays where it is; and no check
 * after: it only ever nudged a parked car forward, and its missed echo
 * faulted a parked car E,4 (2026-10-15 01:02) - a run lost at the last step. */
static double moved(volatile Task2Odom *o, double x0, double y0, double h_deg)
{
  return (o->x - x0) * cos(RAD(h_deg)) + (o->y - y0) * sin(RAD(h_deg));
}

/* The bumper gap ahead, a median of three - from after the last move stopped,
 * not a reading from the move itself. */
static int range_ahead(const Task2Ops *ops, int start, double *gap, int *no_range)
{
  uint16_t cm;
  if (!start)
    ops->wait_ms(T2_ULTRA_SETTLE_MS); /* not a reading from the last move */
  if (!median_range(ops, &cm))
  {
    *no_range = 1;
    return 0;
  }
  *gap = cm + ULTRA_OFFSET_CM;
  if (start && *gap < T2_START_MIN_GAP_CM)
  {
    /* Something is in front of the car that obstacle 1 cannot be. Driving on
     * it put the car into the carpark wall. Stop and say so. */
    step(ops, "too_close", lround(*gap));
    *no_range = 1;
    return 0;
  }
  return 1;
}

static int drive_to(const Task2Ops *ops, double gap, double target_cm, int start, int forward_only,
                    double tol, double rear_x, double *face_x, int *no_range)
{
  volatile Task2Odom *o = ops->odom;
  const double h = heading(ops);
  uint16_t cm;
  double room = o->x + gap - T2_REAR_CM - rear_x;
  if (room < target_cm)
    target_cm = room + fmin((target_cm - room) / 2.0, T2_SWERVE_CLEAR_CM / 2.0);
  double travel = gap - target_cm;
  if (forward_only && travel < 0.0)
    travel = 0.0; /* already closer than the target: stay, never reverse */
  double x0 = o->x, y0 = o->y;
  if (fabs(travel) > tol)
    ops->straight(travel);
  if (ops->failed())
    return 0;
  double went = moved(o, x0, y0, h);
  double expect = gap - went; /* the bumper gap the wheels say */

  /* The start is one move and nothing after it: a re-check there only ever
   * backed the car up the few cm the straight had coasted past the stop. */
  if (!start && !forward_only &&
      (fabs(went) > T2_RECHECK_CM || gap - ULTRA_OFFSET_CM <= T2_RANGE_NEAR_CM))
  {
    ops->wait_ms(T2_ULTRA_SETTLE_MS); /* a reading from after the stop */
    if (!fresh_range(ops, &cm))
    {
      *no_range = 1;
      return 0;
    }
    double seen = cm + ULTRA_OFFSET_CM;
    if (fabs(seen - expect) > T2_RECHECK_MAX_CM && cm > T2_RANGE_NEAR_CM)
    {
      if (!median_range(ops, &cm))
      {
        *no_range = 1;
        return 0;
      }
      seen = cm + ULTRA_OFFSET_CM;
    }
    if (fabs(seen - expect) > T2_RECHECK_MAX_CM && cm > T2_RANGE_NEAR_CM)
    {
      step(ops, "range_ignored", lround(seen));
    }
    else
    {
      double fix = seen - target_cm;
      x0 = o->x;
      y0 = o->y;
      if (fabs(fix) > tol && !(forward_only && fix < 0.0))
      {
        ops->straight(fix);
        if (ops->failed())
          return 0;
      }
      expect = seen - moved(o, x0, y0, h);
    }
  }
  *face_x = o->x + T2_FRONT_CM + expect;
  return 1;
}

static int approach(const Task2Ops *ops, double target_cm, int start, int forward_only,
                    double rear_x, double *face_x, int *no_range)
{
  double gap;
  const double tol = start ? T2_RECHECK_TOL_CM : T2_NUDGE_MIN_CM;
  return range_ahead(ops, start, &gap, no_range) &&
         drive_to(ops, gap, target_cm, start, forward_only, tol, rear_x, face_x, no_range);
}

/* ------------------------------------------------------------ waypoints --- */

/* Drive to a pose. The car turns on an arc, so aiming at a point and driving
 * misses it by a turn radius; planning the arcs in hits it. The radius is
 * only as good as its calibration, so after each attempt the pose is measured
 * again and the remainder re-planned - but never with a near-full circle,
 * which near the goal would be a loop chasing a few centimetres (on the floor:
 * 327 degrees). open_first / open_last: the speed of each arc. */
static int goto_pose(const Task2Ops *ops, double tx, double ty, double th,
                     int open_first, int open_last)
{
  volatile Task2Odom *o = ops->odom;
  const double to[3] = {tx, ty, th};
  const double r0[2] = {arc(1, open_first), arc(0, open_first)};
  const double r1[2] = {arc(1, open_last), arc(0, open_last)};
  for (int i = 0; i < T2_WAYPOINT_TRIES; ++i)
  {
    double from[3] = {o->x, o->y, heading(ops)}, plan[3] = {0.0, 0.0, 0.0};
    if (hypot(tx - from[0], ty - from[1]) <= T2_WAYPOINT_TOL_CM)
      break;
    if (!Task2_PlanCSC(from, to, r0, r1, plan))
      break;
    if (fabs(plan[0]) + fabs(plan[2]) > T2_MAX_PLAN_TURN_DEG)
    {
      step(ops, "plan_loop", lround(fabs(plan[0]) + fabs(plan[2])));
      break;
    }
    step(ops, "plan_turn", lround(plan[0]));
    step(ops, "plan_run", lround(plan[1]));
    step(ops, "plan_end", lround(plan[2]));
    set_speed(ops, open_first);
    if (!turn_by(ops, plan[0]))
      return 0;
    if (plan[1] > 0.5)
    {
      ops->straight(plan[1]);
      if (ops->failed())
        return 0;
    }
    set_speed(ops, open_last);
    if (!turn_by(ops, plan[2]))
      return 0;
  }
  /* Whatever the position, leave facing the way the next leg needs - but a
   * few degrees is not worth a turn: on the floor it was a servo blip to full
   * lock and back just outside the carpark. */
  double square = Task2_TurnToward(heading(ops), th);
  step(ops, "square", lround(square));
  if (fabs(square) < T2_SQUARE_MIN_DEG)
    return !ops->failed();
  set_speed(ops, open_last);
  return turn_by(ops, square);
}

/* ---------------------------------------------------------------- arrows -- */

static void request_arrow(const Task2Ops *ops, int obstacle)
{
  char line[8];
  /* Clear first, then ask: the other order could wipe an answer that arrived
   * between the two. */
  ops->clear_answer();
  snprintf(line, sizeof line, "P,%d\n", obstacle);
  ops->send(line);
}

/* Wait up to timeout_ms for obstacle n's arrow, asking again every
 * T2_PROMPT_REPEAT_MS in case a request or a reply was lost. Returns 0 rather
 * than a guess when the Pi does not commit: an unread arrow stops the run, a
 * wrong one disqualifies it. */
static char await_arrow(const Task2Ops *ops, int obstacle, uint32_t timeout_ms)
{
  char line[8];
  snprintf(line, sizeof line, "P,%d\n", obstacle);
  uint32_t start = ops->now_ms(), asked = start;
  for (;;)
  {
    char a = ops->answer(obstacle);
    if (a == 'L' || a == 'R')
      return a;
    if (ops->failed())
      return 0;
    uint32_t now = ops->now_ms();
    if (now - start >= timeout_ms)
      return 0;
    if (now - asked >= T2_PROMPT_REPEAT_MS)
    {
      ops->send(line);
      asked = now;
    }
    ops->wait_ms(20);
  }
}

/* --------------------------------------------------------------- swerve --- */

/* Obstacle 1 as the swerve must miss it, in odometry coordinates: grown
 * SWERVE_CLEAR_CM all round, and LATERAL_ERR_CM more across for how well the
 * camera placed it on axis_y. */
typedef struct
{
  float x0, x1, y0, y1;
} Box2;

/* Does the car, rear axle at (x, y) heading h radians, overlap the box? From
 * above the car is an H: the wheels are its widest points - HALF_WIDTH either
 * side from the rear wheels to the front wheels' leading edge - and ahead of
 * them only the nose, NOSE_HALF wide. Its outline, sampled every 4-5 cm, in the
 * box, or a corner of the box inside the outline: between them every way the
 * two can meet. */
#define T2_WHEELS_FRONT_CM (T2_FRONT_CM - T2_NOSE_LEN_CM)
#define SIDE_X(k) (-T2_REAR_CM + (T2_WHEELS_FRONT_CM + T2_REAR_CM) * (k) / 5.0f)
static int car_hits(float x, float y, float h, const Box2 *b)
{
  static const float pts[][2] = {
      {SIDE_X(0), T2_HALF_WIDTH_CM}, {SIDE_X(1), T2_HALF_WIDTH_CM}, {SIDE_X(2), T2_HALF_WIDTH_CM},
      {SIDE_X(3), T2_HALF_WIDTH_CM}, {SIDE_X(4), T2_HALF_WIDTH_CM}, {SIDE_X(5), T2_HALF_WIDTH_CM},
      {SIDE_X(0), -T2_HALF_WIDTH_CM}, {SIDE_X(1), -T2_HALF_WIDTH_CM}, {SIDE_X(2), -T2_HALF_WIDTH_CM},
      {SIDE_X(3), -T2_HALF_WIDTH_CM}, {SIDE_X(4), -T2_HALF_WIDTH_CM}, {SIDE_X(5), -T2_HALF_WIDTH_CM},
      {-T2_REAR_CM, T2_HALF_WIDTH_CM / 2.0f}, {-T2_REAR_CM, 0.0f}, {-T2_REAR_CM, -T2_HALF_WIDTH_CM / 2.0f},
      {T2_WHEELS_FRONT_CM, T2_NOSE_HALF_CM}, {T2_WHEELS_FRONT_CM, -T2_NOSE_HALF_CM},
      {T2_FRONT_CM, T2_NOSE_HALF_CM}, {T2_FRONT_CM, 0.0f}, {T2_FRONT_CM, -T2_NOSE_HALF_CM},
  };
  const float c = cosf(h), s = sinf(h);
  for (unsigned p = 0; p < sizeof pts / sizeof *pts; ++p)
  {
    float px = x + pts[p][0] * c - pts[p][1] * s;
    float py = y + pts[p][0] * s + pts[p][1] * c;
    if (px > b->x0 && px < b->x1 && py > b->y0 && py < b->y1)
      return 1;
  }
  for (int k = 0; k < 4; ++k)
  {
    float dx = (k & 1 ? b->x1 : b->x0) - x, dy = (k & 2 ? b->y1 : b->y0) - y;
    float f = dx * c + dy * s, l = fabsf(-dx * s + dy * c); /* in the car's frame */
    if (f > -T2_REAR_CM && f < T2_FRONT_CM &&
        l < (f < T2_WHEELS_FRONT_CM ? T2_HALF_WIDTH_CM : T2_NOSE_HALF_CM))
      return 1;
  }
  return 0;
}

/* Drive an arc on paper from (x, y, h): deg degrees, positive left, on radius
 * r, in 4 degree steps. 0 if the car meets the box anywhere on it; the pose is
 * left at the arc's end. */
static int arc_clear(float *x, float *y, float *h, float deg, float r, const Box2 *b)
{
  const float s = deg > 0.0f ? 1.0f : -1.0f, h0 = *h;
  const float cx = *x - s * r * sinf(h0), cy = *y + s * r * cosf(h0);
  const int n = fabsf(deg) > 4.0f ? (int)ceilf(fabsf(deg) / 4.0f) : 1;
  for (int k = 0; k <= n; ++k)
  {
    *h = h0 + (float)RAD(deg) * k / n;
    *x = cx + s * r * sinf(*h);
    *y = cy - s * r * cosf(*h);
    if (car_hits(*x, *y, *h, b))
      return 0;
  }
  return 1;
}

static Box2 obstacle1_box(double face1)
{
  const float c = T2_SWERVE_CLEAR_CM, half = T2_OBSTACLE_DEPTH_CM / 2.0f + c + T2_LATERAL_ERR_CM;
  const Box2 box = {(float)face1 - c, (float)face1 + T2_OBSTACLE_DEPTH_CM + c,
                    (float)axis_y - half, (float)axis_y + half};
  return box;
}

/* After the turn back the car is angled in toward obstacle 1's line - and
 * obstacle 2 stands on it. The diagonal's sideways share left in, the car
 * squared up 13-14 cm off the line on the floor (2026-10-15) with obstacle 2's
 * arrow at the very edge of the photo, and a car 13 cm out once missed obstacle
 * 2 on the ultrasonic altogether (E,4). So a straight on in before the finish
 * brings it back onto the line; within SWERVE_LAND_CM it is left. This is that
 * straight's length from (x, y) at the end of the turn back, on the finish's
 * arc ra. */
static float back_length(float y, float side, float ra)
{
  const float a = (float)RAD(T2_SWERVE_DEG);
  const float rem = side * (y - (float)axis_y) - ra * (1.0f - cosf(a)); /* out after the finish */
  /* to within LAND, not onto the line: every cm brought in is ~1 cm closer
   * to obstacle 2, and squared up on the line the car stopped 5 cm short of
   * it (2026-10-15 03:00) - too close even for the camera */
  return rem > T2_SWERVE_LAND_CM ? (rem - T2_SWERVE_LAND_CM) / sinf(a) : 0.0f;
}

/* From the end of the turn back (x, y, h): does the straight in, `back` long,
 * and then the finish on arc ra keep the car off the box? */
static int back_clear(float x, float y, float h, float side, float ra, float back, const Box2 *b)
{
  for (float s = 0.0f; s <= back; s += 2.0f)
    if (car_hits(x + s * cosf(h), y + s * sinf(h), h, b))
      return 0;
  x += back * cosf(h);
  y += back * sinf(h);
  return arc_clear(&x, &y, &h, side * T2_SWERVE_DEG, ra, b);
}

/* The shortest diagonal - a straight at the opening's heading, between the
 * opening arc and the turn back - that keeps the rest of the swerve, the
 * straight back in included, clear of obstacle 1, from where the opening
 * actually left the car. The turn back is on the lock not driven yet, so it is
 * planned ARC_ERR_CM tighter than that lock's arc: a tighter turn swings the
 * car's inside into the obstacle sooner, and ends with its tail still beside
 * the obstacle, where the straight back in cannot start. (Half that margin
 * saved ~1 cm of diagonal on the floor's arcs, but on arcs 2 cm tight the
 * straight back was refused and the car squared up 11-17 cm off the line in
 * the course sim.) The finish is on the opening's lock, just measured. The obstacle is where the camera
 * put it (axis_y): the fixed S had no diagonal and ignored that, and on the
 * floor (2026-10-14/15) it hit obstacle 1 with the obstacle 4-9 cm toward the
 * swerve. Float, a few ms while the car stands. */
static double swerve_diagonal(const Task2Ops *ops, int out, double face1)
{
  volatile Task2Odom *o = ops->odom;
  const Box2 box = obstacle1_box(face1);
  const float h1 = (float)RAD(heading(ops)), side = out ? 1.0f : -1.0f;
  const float rb = (float)(arc(!out, 0) - T2_ARC_ERR_CM), ra = (float)arc(out, 0);
  int d;
  for (d = 0; d <= (int)T2_SWERVE_DIAG_MAX_CM; ++d)
  {
    float x = o->x + d * cosf(h1), y = o->y + d * sinf(h1), h = h1;
    if (car_hits(x, y, h, &box))
      break; /* the diagonal itself runs into it: a longer one only does again */
    if (arc_clear(&x, &y, &h, -2.0f * side * T2_SWERVE_DEG, rb, &box) &&
        back_clear(x, y, h, side, ra, back_length(y, side, ra), &box))
      return d;
  }
  /* Nothing clears with the margins: the widest diagonal that does not run
   * into it gives the most room there is. */
  d = d > (int)T2_SWERVE_DIAG_MAX_CM ? (int)T2_SWERVE_DIAG_MAX_CM : (d > 0 ? d - 1 : 0);
  step(ops, "swerve_tight", d);
  return d;
}

/* The straight back in, from where the turn back really ended: the diagonal
 * was sized for it on the planned arcs, and this re-checks it - a shorter
 * straight if the real turn back left the tail too near obstacle 1, or if the
 * whole of it would take the car into `wall`, obstacle 2's face as the peek
 * found it: then the car squares up further off the line, as far in as fits. */
static double swerve_back(const Task2Ops *ops, int out, double face1, const Box2 *wall)
{
  volatile Task2Odom *o = ops->odom;
  const Box2 box = obstacle1_box(face1);
  const float side = out ? 1.0f : -1.0f, ra = (float)arc(out, 0), h0 = (float)RAD(heading(ops));
  for (float l = back_length((float)o->y, side, ra); l > 0.5f; l -= 1.0f)
    if (back_clear((float)o->x, (float)o->y, h0, side, ra, l, &box) &&
        back_clear((float)o->x, (float)o->y, h0, side, ra, l, wall))
      return l;
  return 0.0;
}

/* Halfway through the turn back - beside obstacle 1, angled PEEK_DEG in toward
 * its line - the ultrasonic looks past obstacle 1 at obstacle 2. In the course
 * sim it found it at every gap, length and place from here, where straight
 * ahead it missed 30 cm ones at 60-80 cm gaps - exactly the gaps where coming
 * back onto the line runs into it. The camera has arrow 2 near mid-photo from
 * here too, so it is asked for now. Returns where obstacle 2's face is
 * (odometry x): the echo placed as near as the cone lets it be, and with no
 * echo, or one further than any arena, the nearest the rules allow. */
static double peek(const Task2Ops *ops, double face1, int ask)
{
  volatile Task2Odom *o = ops->odom;
  const double h = heading(ops);
  const double nearest = face1 + T2_OBSTACLE_DEPTH_CM + T2_GAP2_MIN_CM;
  double face = nearest;
  uint16_t cm;
  if (ask)
    request_arrow(ops, 2);
  ops->wait_ms(T2_ULTRA_SETTLE_MS); /* not a reading from the turn */
  face2_far = face1 + T2_OBSTACLE_DEPTH_CM + T2_GAP2_MAX_CM;
  if (median_range(ops, &cm))
  {
    /* the echo came from somewhere in the cone: the face is between where the
     * cone's far and near edges would put it */
    const double xs = o->x + T2_FRONT_CM * cos(RAD(h)), r = cm + ULTRA_OFFSET_CM;
    const double x = xs + r * cos(RAD(fabs(h) + T2_ULTRA_CONE_DEG));
    if (x < face2_far)
    {
      face = x;
      face2_far = xs + r * cos(RAD(fmax(0.0, fabs(h) - T2_ULTRA_CONE_DEG)));
    }
  }
  step(ops, "peek", lround(face));
  return face;
}

/* Past obstacle 1 and back onto its line, all forward, all precise:
 *   right arrow: SF<pre> RF045 SF<diagonal> LF065 (peek) LF025 SF<back> RF045
 *   left arrow:  SF<pre> LF045 SF<diagonal> RF065 (peek) RF025 SF<back> LF045
 * The opening and closing arcs are on the same lock, so they cancel sideways;
 * the diagonal takes the car out by 0.71 of its length and the straight back
 * brings it in again. The diagonal is only as long as obstacle 1's clearance
 * needs - none when it stands off to the other side. Every turn still goes
 * through turn(), so its arc is measured for the stop at obstacle 2 and the
 * way home.
 *
 * The turn back stops PEEK_DEG past straight for peek(), the steering held at
 * its lock so the pause costs no servo swing. If the rest of the S - even
 * without the straight back - would reach obstacle 2's face, there is no room
 * to come back onto the line (a 60 cm gap): the car squares up where it is,
 * in reverse on the same lock (LB020 / RB020), off the line and well short of
 * obstacle 2, and reads arrow 2 from the peek first. */
/* The guard stopped the swerve's finish short of obstacle 2 (see swerve):
 * square up from wherever it stopped IN REVERSE - away from obstacle 2, on the
 * lock that turns the car back toward straight ahead - and go on as if the
 * peek had found no room: off the line, facing obstacle 2. */
static int guard_recover(const Task2Ops *ops, uint32_t asked)
{
  ops->guard(0.0);
  const double h = heading(ops);
  step(ops, "guard", lround(h));
  peek_tight = 1;
  step(ops, "peek_tight", 1);
  uint32_t waited = ops->now_ms() - asked;
  await_arrow(ops, 2, waited < T2_ARROW2_WAIT_MS ? T2_ARROW2_WAIT_MS - waited : 0U);
  if (ops->failed())
    return 0;
  /* reversing on the left lock turns the car right, and the right lock left */
  return fabs(h) < 0.5 || turn(ops, h > 0.0, -fabs(h));
}

static int swerve(const Task2Ops *ops, char arrow, double face1)
{
  const int out = arrow != 'R'; /* the lock that opens the swerve: right for R */
  set_speed(ops, 0);
  if (T2_SWERVE_PRE_CM > 0.5f)
  {
    ops->straight(T2_SWERVE_PRE_CM);
    if (ops->failed())
      return 0;
  }
  if (!turn(ops, out, T2_SWERVE_DEG))
    return 0;
  double diagonal = swerve_diagonal(ops, out, face1);
  step(ops, "swerve_diag", lround(diagonal));
  if (diagonal > 0.5)
  {
    ops->straight(diagonal);
    if (ops->failed())
      return 0;
  }
  /* Turn back in two: square to obstacle 2 first and range it there, then on
   * PEEK_DEG and range again. From PEEK_DEG its face slants across the beam,
   * and on the floor (2026-10-15) the HC-SR04 heard far past it at a 60 cm
   * gap: the swerve finished as if there were room and hit it. Square on, the
   * echo is good - when the cone catches it, which off the line it can miss
   * on a short obstacle 2; angled, it does catch it. Neither ever echoes
   * short, so the nearer of the two is where the face is. */
  ops->hold_steering(1);
  int ok = turn(ops, !out, T2_SWERVE_DEG);
  const uint32_t asked = ops->now_ms();
  double square = 1e9, square_far = 1e9;
  if (ok)
  {
    square = peek(ops, face1, 1);
    square_far = face2_far;
    ok = turn(ops, !out, T2_PEEK_DEG);
  }
  ops->hold_steering(0);
  if (!ok)
    return 0;
  face2_peek = peek(ops, face1, 0);
  if (square < face2_peek)
  {
    step(ops, "peek_square", lround(square));
    face2_peek = square;
  }
  /* each echo bounds the face from beyond - a long one only more loosely -
   * so the tighter bound holds */
  face2_far = fmin(face2_far, square_far);
  /* What finishing the swerve has to keep clear of. Square on and off the
   * line, the cone can miss a short obstacle 2's face and catch its end
   * instead - up to its depth further (course sim, 60 cm gap, obstacle 2 8 cm
   * toward the swerve) - so a square echo only places the face that much
   * nearer, for this. face2_peek stays as heard: the stop at obstacle 2
   * ranges again from there. */
  const double finish_face = fmin(face2_peek, square - T2_OBSTACLE_DEPTH_CM);
  {
    volatile Task2Odom *o = ops->odom;
    const Box2 wall = {(float)(finish_face - T2_OBST2_CLEAR_CM), 1e6f, -1e6f, 1e6f};
    const float side = out ? 1.0f : -1.0f;
    float x = o->x, y = o->y, h = (float)RAD(heading(ops));
    peek_tight = !(arc_clear(&x, &y, &h, -side * (T2_SWERVE_DEG - T2_PEEK_DEG), (float)arc(!out, 0), &wall) &&
                   back_clear(x, y, h, side, (float)arc(out, 0), 0.0f, &wall));
    step(ops, "peek_tight", peek_tight);
    if (peek_tight)
    {
      uint32_t waited = ops->now_ms() - asked;
      await_arrow(ops, 2, waited < T2_ARROW2_WAIT_MS ? T2_ARROW2_WAIT_MS - waited : 0U);
      return !ops->failed() && turn(ops, !out, -T2_PEEK_DEG);
    }
    /* The peek found room - on echoes that can still be long. From here to
     * the end of the swerve, the ultrasonic stops any move that brings the
     * bumper within GUARD_CM of something: angled, the beam reads long, but
     * the last turn squares it up to obstacle 2 - where the car hit it. */
    ops->guard(T2_GUARD_CM);
    if (!turn(ops, !out, T2_SWERVE_DEG - T2_PEEK_DEG))
      return 0;
    if (ops->guarded())
      return guard_recover(ops, asked);
  }
  const Box2 wall = {(float)(finish_face - T2_OBST2_CLEAR_CM), 1e6f, -1e6f, 1e6f};
  double back = swerve_back(ops, out, face1, &wall);
  step(ops, "swerve_back", lround(back));
  {
    /* short of the line on purpose - no room before obstacle 2, or the tail
     * still beside obstacle 1: the run log says by how much */
    const double full = back_length((float)ops->odom->y, out ? 1.0f : -1.0f, (float)arc(out, 0));
    if (full - back > 0.5)
      step(ops, "back_short", lround(full - back));
  }
  if (back > 0.5)
  {
    ops->straight(back);
    if (ops->failed())
      return 0;
    if (ops->guarded())
      return guard_recover(ops, asked);
  }
  if (!turn(ops, out, T2_SWERVE_DEG))
    return 0;
  if (ops->guarded())
    return guard_recover(ops, asked);
  ops->guard(0.0);
  return 1;
}

/* ------------------------------------------------------------ obstacle 2 -- */

/* Where to stop at obstacle 2 - the bumper gap - to round it on the arrow's
 * side, on the arcs measured this run (the swerve turned on both locks):
 *   - the 90 away: the car's outline (car_hits) sweeps past the bumper - the
 *     outer front wheel's corner, or the nose ahead of it - on the arc
 *     ARC_ERR_CM wider than measured, and keeps OBST2_CLEAR_CM;
 *   - the side comes out of it FACE_GAP_CM or more off the face, and so close
 *     that the 180 round the end, on the other lock, lands it no further than
 *     BACK_REACH_CM off the back face (task2_params.h).
 * On the floor's arcs: ~30 cm for a right arrow, ~29 for a left one. The 20 cm
 * box to the tip and 4 cm clearance it replaced stopped at 32-33 and reversed
 * 12 cm to get there (2026-10-15 02:14) - with ~9 cm to spare in the turn. */
static double turn_stop(char arrow)
{
  const int away = arrow != 'R';
  const double r = arc(away, 0), u = arc(!away, 0), wide = r + T2_ARC_ERR_CM;
  const double reach = fmax(hypot(wide + T2_HALF_WIDTH_CM, T2_WHEELS_FRONT_CM),
                            hypot(wide + T2_NOSE_HALF_CM, T2_FRONT_CM));
  const double side = fmax(T2_FACE_GAP_CM, 2.0 * u - 2.0 * T2_HALF_WIDTH_CM - T2_OBSTACLE_DEPTH_CM -
                                               T2_BACK_REACH_CM);
  return fmax(reach - T2_FRONT_CM + T2_OBST2_CLEAR_CM, r + T2_HALF_WIDTH_CM - T2_FRONT_CM + side);
}

/* The farthest stop - bumper gap - the turn at obstacle 2 still works from,
 * the arcs ARC_ERR tighter than measured (the turn reaches less): the 90 away
 * leaves the side IR's lens within IR_REACH_CM (less TURN_FAR_MARGIN_CM) of
 * the face, and the 180 round its end clears the back face by OBST2_CLEAR_CM.
 * After the 90 the axle is the bumper gap + FRONT - r off the face. */
static double turn_far(char arrow)
{
  const int away = arrow != 'R';
  const double r = arc(away, 0) - T2_ARC_ERR_CM, u = arc(!away, 0) - T2_ARC_ERR_CM;
  const double by_ir = T2_IR_LENS_CM + T2_IR_REACH_CM - T2_TURN_FAR_MARGIN_CM;
  const double by_back = 2.0 * u - T2_OBSTACLE_DEPTH_CM - T2_HALF_WIDTH_CM - T2_OBST2_CLEAR_CM;
  return fmin(by_ir, by_back) - T2_FRONT_CM + r;
}

/* Round obstacle 2 on the arrow's side, from the stop turn_stop() sized: turn
 * away, follow the face to its end, go round behind it in one 180, follow the
 * back to the far end. A right arrow puts the obstacle on the robot's left -
 * the left sensor, IR000's. Left mirrors it. Every turn here runs precise:
 * open, the left 90 once ran on ~55 cm against the planned 28 (2026-10-14:
 * x 173 -> 235, y -11 -> 42 over the 90) and its outer front corner swept into
 * obstacle 2's face. */
/* Where obstacle 2's end can be at the furthest, from a clear IR during the
 * turn away: the ray from where it first read clear (at) meets the face
 * there, so the end is no further out. The face is only known to
 * FACE2_ERR_CM: the ray must reach it at its furthest, and the bound is taken
 * where it meets it at its nearest. 0 if the ray could not reach
 * (T2_IR_REACH_CM) - then the clear reading says nothing about the end - or
 * if it met the face short of where obstacle 2 is sure to be: then the IR
 * was looking past its NEAR end (the car started off to that side), not its
 * far one (course sim: squared up 30 cm off its line after a tight swerve). */
static int end_bound(const Task2Ops *ops, double s, double face2, const double at[3],
                     double *end_y)
{
  const double ha = Task2_WrapDeg(at[2] - ops->odom->h0), cha = cos(RAD(ha)), sha = sin(RAD(ha));
  const double ax = -s * sha, ay = s * cha; /* the ray: out toward the obstacle's side */
  const double sx = at[0] + T2_IR_AHEAD_CM * cha + T2_IR_LENS_CM * ax;
  const double sy = at[1] + T2_IR_AHEAD_CM * sha + T2_IR_LENS_CM * ay;
  if (ax < 0.05)
    return 0;
  const double near = (face2 - T2_FACE2_ERR_CM - sx) / ax, far = (face2 + T2_FACE2_ERR_CM - sx) / ax;
  *end_y = sy + fmax(near, 0.0) * ay;
  /* the part of obstacle 2 that is there whatever its length and place */
  const double sure = T2_OBST2_MIN_HALF_CM - T2_LATERAL_ERR_CM - T2_OBST2_OFFSET_CM;
  return far <= T2_IR_REACH_CM && -s * (*end_y - axis_y) >= sure;
}

/* Cut short (turn_watch): the side IR has swung past obstacle 2's end, so it
 * is short. Turn back toward it, to the heading the 180 would have left - but
 * first straight on along the diagonal until that turn both
 *   - lands the side SHORT_BACK_GAP_CM behind the back face (turning back at
 *     once fell short of it: the back leg drove into the obstacle's end), and
 *   - clears the end: the IR read clear along its ray, so the end is no
 *     further out than where that ray meets the face. Both end corners there
 *     must sit inside the circle the car's inner side sweeps, OBST2_CLEAR_CM
 *     in (turning back at once from 70 swept the side into the end). */
/* Obstacle 2's span and the walls from the ends read so far (see meas_lo /
 * meas_hi). An end not read is the mirror of the one read about obstacle 1's
 * line - OBST2_OFFSET_CM either way, obstacle 2 being placed up to that far
 * off the line: further out for the span the car keeps off, nearer in for
 * where the wall past it can be. Neither read: the walls are where the
 * narrowest obstacle 2 would put them, and the span (assume_max2) the widest. */
static void recompute_end2(void)
{
  const double inner_min = T2_OBST2_MIN_HALF_CM - T2_OBST2_OFFSET_CM; /* nearest an end can be */
  double lo_out, hi_out, lo_in, hi_in;
  const int lo_ok = meas_lo != NO_END, hi_ok = meas_hi != NO_END;
  if (lo_ok && hi_ok)
  {
    lo_out = lo_in = meas_lo;
    hi_out = hi_in = meas_hi;
  }
  else if (hi_ok || lo_ok)
  {
    const double half = hi_ok ? meas_hi - axis_y : axis_y - meas_lo;
    const double out = half + 2.0 * T2_OBST2_OFFSET_CM, in = fmax(half - 2.0 * T2_OBST2_OFFSET_CM, inner_min);
    if (hi_ok)
    {
      hi_out = hi_in = meas_hi;
      lo_out = axis_y - out;
      lo_in = axis_y - in;
    }
    else
    {
      lo_out = lo_in = meas_lo;
      hi_out = axis_y + out;
      hi_in = axis_y + in;
    }
  }
  else
  {
    const double out = T2_OBST2_MAX_HALF_CM + T2_OBST2_OFFSET_CM;
    lo_out = axis_y - out;
    hi_out = axis_y + out;
    lo_in = axis_y - inner_min;
    hi_in = axis_y + inner_min;
  }
  have_end2 = lo_ok || hi_ok || assume_max2;
  end2_lo = lo_out;
  end2_hi = hi_out;
  wall_lo = lo_in - T2_WALL_PAST_CM;
  wall_hi = hi_in + T2_WALL_PAST_CM;
}

/* An end of obstacle 2 read by a side IR, at y. */
static void record_end2(const Task2Ops *ops, double y)
{
  if (y >= axis_y)
    meas_hi = y;
  else
    meas_lo = y;
  recompute_end2();
  step(ops, "width2", lround(end2_hi - end2_lo));
  step(ops, "wall_lo", lround(wall_lo));
  step(ops, "wall_hi", lround(wall_hi));
}

/* The lens of side IR on the leg, from the axle at (y, heading h deg raw). */
static double lens_y(const Task2Ops *ops, double y, double h_raw)
{
  return y + T2_IR_AHEAD_CM * sin(RAD(Task2_WrapDeg(h_raw - ops->odom->h0)));
}

/* Can y be obstacle 2's end, on the side dir (+1 / -1) of obstacle 1's line?
 * Not further out than the widest obstacle 2, nor than us_end_bound when the
 * ultrasound bounded that side. Anything further is something else - a wall,
 * a box - seen by the side IR. */
static int bound_on(double dir)
{
  return have_end_bound && dir * (us_end_bound - axis_y) > 0.0;
}

static int end_plausible(double y, double dir)
{
  if (dir * (y - axis_y) > T2_OBST2_PLAUSIBLE_HALF_CM + T2_OBST2_OFFSET_CM + T2_LATERAL_ERR_CM)
    return 0;
  return !bound_on(dir) || dir * (y - us_end_bound) <= T2_END_MARGIN_CM + T2_LATERAL_ERR_CM;
}

/* The arena walls as the car keeps off them: everything past wall_lo /
 * wall_hi, T2_WALL_CLEAR_CM in. */
static Box2 wall_box(int hi)
{
  const float c = T2_WALL_CLEAR_CM;
  const Box2 lo_b = {-1e6f, 1e6f, -1e6f, (float)wall_lo + c};
  const Box2 hi_b = {-1e6f, 1e6f, (float)wall_hi - c, 1e6f};
  return hi ? hi_b : lo_b;
}

/* Obstacle 2 as the car keeps c off it: its face from the stop at it, its
 * span from recompute_end2(). */
static Box2 obstacle2_box(float c)
{
  const Box2 b = {(float)face2_x - c, (float)face2_x + T2_OBSTACLE_DEPTH_CM + c,
                  (float)end2_lo - c, (float)end2_hi + c};
  return b;
}

static int plan_clear_err(const double from[3], const double plan[3], const double r[2], const Box2 *b);

/* The leg behind obstacle 2, back along its back face to the far end. With
 * the end placed (recompute_end2) it stops past the far end even if the side IR
 * never sees the back face - out of reach after a wide 180 - where it used to
 * drive EDGE_MAX_CM and fault (course sim, turns 2 cm wide). */
static int back_leg(const Task2Ops *ops, int right_sensor)
{
  volatile Task2Odom *o = ops->odom;
  double at[3], cap = 0.0;
  if (have_end2)
  {
    const double s = heading(ops) > 0.0 ? 1.0 : -1.0; /* the leg runs along +y or -y */
    const double lens = o->y + s * T2_IR_AHEAD_CM;
    const double far = s > 0.0 ? end2_hi : end2_lo;
    cap = fmax(1.0, s * (far - lens) + T2_END_MARGIN_CM + T2_EDGE_BACK_EXTRA_CM);
    step(ops, "back_cap", lround(cap));
  }
  step(ops, "behind2", 0);
  const int r = ops->edge_leg(right_sensor, 0, cap, at);
  if (r == EDGE_CAPPED)
    step(ops, "back_capped", 0);
  else if (r == EDGE_FOUND)
  {
    /* the far end, read: the wall past it too - if it can be obstacle 2's */
    const double e = lens_y(ops, at[1], at[2]), side = heading(ops) > 0.0 ? 1.0 : -1.0;
    if (end_plausible(e, side))
      record_end2(ops, e);
    else
      step(ops, "end_reject", lround(e));
  }
  step(ops, "far_end2", 0);
  return !ops->failed() && r != EDGE_FAILED;
}

static int round_short(const Task2Ops *ops, int toward, int right_sensor, double h_start,
                       double face2, double end_y)
{
  volatile Task2Odom *o = ops->odom;
  const double s = toward ? 1.0 : -1.0; /* left positive; the obstacle's side */
  const double h_back = h_start + s * 90.0;
  const double rt = arc(toward, 0) + T2_ARC_ERR_CM;
  const double ri = arc(toward, 0) - T2_HALF_WIDTH_CM - T2_OBST2_CLEAR_CM;
  const double back = face2 + T2_OBSTACLE_DEPTH_CM;
  const double h = heading(ops), ch = cos(RAD(h)), sh = sin(RAD(h));
  const double nx = -s * sh, ny = s * ch; /* toward the obstacle's side */
  double run = 0.0;
  for (; run < T2_SHORT_RUN_MAX_CM; run += 0.5)
  {
    double px = o->x + run * ch, py = o->y + run * sh;
    double cx = px + rt * nx, cy = py + rt * ny; /* the turn back's centre */
    double land = cx + s * rt * sin(RAD(h_back));
    if (land < back + T2_SHORT_BACK_GAP_CM + T2_HALF_WIDTH_CM)
      continue;
    if (hypot(face2 - cx, end_y - cy) <= ri && hypot(back - cx, end_y - cy) <= ri)
      break;
  }
  step(ops, "short_run", lround(run));
  if (run > 0.5)
  {
    ops->straight(run);
    if (ops->failed())
      return 0;
  }
  if (!turn(ops, toward, fabs(Task2_WrapDeg(h_back - heading(ops)))))
    return 0;
  /* the clear IR bounds the end no further out than end_y: the far end from that */
  record_end2(ops, end_y);
  return back_leg(ops, right_sensor);
}

static int round_obstacle2(const Task2Ops *ops, char arrow, double face2)
{
  int right_sensor = arrow == 'L';
  const int toward = arrow == 'R'; /* the lock that turns toward the obstacle */
  const int away = !toward;
  volatile Task2Odom *o = ops->odom;

  /* The 90 away. With T2_WATCH_CUT it watches the side IR: past
   * T2_WATCH_CHECK_DEG with the IR clear, a short obstacle 2 is already
   * behind its end, and the turn stops there rather than swinging the whole
   * 90 out past it. Off (the default) the 90 is always finished: the IR
   * reads the black face badly at an angle, and a false "clear" turned the
   * car back through obstacle 2 on the floor. The face scan below, square on
   * to the face, decides instead. */
  set_speed(ops, 0);
  if (!T2_WATCH_CUT)
  {
    if (!turn(ops, away, IR_EDGE_START_DEG))
      return 0;
  }
  else
  {
    double x0 = o->x, y0 = o->y, h0 = heading(ops);
    double at[3];
    int cut = ops->turn_watch(away, IR_EDGE_START_DEG, right_sensor, T2_WATCH_CHECK_DEG, at);
    if (ops->failed())
      return 0;
    if (cut)
    {
      const double s = toward ? 1.0 : -1.0;
      /* with no ray at all: the furthest a short obstacle 2's end reaches (as past_more) */
      double end_y = axis_y + s * (T2_OBST2_MIN_HALF_CM + T2_LATERAL_ERR_CM + T2_OBST2_OFFSET_CM);
      double swept = fabs(Task2_WrapDeg(heading(ops) - h0));
      step(ops, "cut2", lround(swept));
      int placed = end_bound(ops, s, face2, at, &end_y);
      if (placed || T2_CUT2_ALWAYS)
      {
        if (!placed)
          step(ops, "cut_blind", 0); /* logged only: cut2 wins (T2_CUT2_ALWAYS) */
        step(ops, "end_y", lround(end_y));
        return round_short(ops, toward, right_sensor, h0, face2, end_y);
      }
      /* Clear, but the ray never reached the face: nothing known about the
       * end. Finish the 90 and round it the usual way. */
      step(ops, "cut_blind", 0);
      if (swept < IR_EDGE_START_DEG - 0.5 && !turn(ops, away, IR_EDGE_START_DEG - swept))
        return 0;
    }
    else
      learn_arc(ops, away, x0, y0, h0);
  }
  set_speed(ops, 0);
  step(ops, "beside2", right_sensor);
  const double dir = heading(ops) > 0.0 ? 1.0 : -1.0; /* the leg runs along +y or -y */
  {
    /* The face scan (Gordon's): the side IR losing the face is obstacle 2's
     * end. Already past it - nothing beside the IR after the 90 - the car
     * looks back for it, in reverse, no further than the nearest that end
     * can be: an end it has not read it cannot plan the 180 or the walls
     * round. Not seen even there, the IR cannot see this face: obstacle 2 is
     * taken as wide as it can be (assume_max2). */
    double at[3];
    const double lens0 = lens_y(ops, o->y, ops->heading());
    int r;
    if (bound_on(dir) && dir * (lens0 - us_end_bound) > T2_END_MARGIN_CM + T2_LATERAL_ERR_CM)
    {
      /* The ultrasound already put obstacle 2's end inward of where the IR
       * is: anything it sees here is not obstacle 2. No face scan. */
      step(ops, "past_bound", lround(us_end_bound));
      r = EDGE_CAPPED;
      record_end2(ops, us_end_bound);
    }
    else
      r = ops->edge_leg(right_sensor, 1, 0.0, at);
    if (ops->failed() || r == EDGE_FAILED)
      return 0;
    if (r == EDGE_FOUND)
    {
      const double e = lens_y(ops, at[1], at[2]);
      if (end_plausible(e, dir))
        record_end2(ops, e);
      else
      {
        /* Not obstacle 2 - past where it can reach. Look back for the real
         * end, past whatever this was, as far as the nearest it can be. */
        step(ops, "end_reject", lround(e));
        r = EDGE_PAST;
      }
    }
    if (r == EDGE_PAST)
    {
      const double nearest = axis_y + dir * (T2_OBST2_MIN_HALF_CM - T2_OBST2_OFFSET_CM - T2_END_MARGIN_CM);
      int k = EDGE_CAPPED, found = 0;
      /* Each seek stops on the next thing the IR sees; one that cannot be
       * obstacle 2 (the box past it) is passed by the next seek. */
      for (int tries = 0; tries < 3 && !found; ++tries)
      {
        const double back = dir * (lens_y(ops, o->y, ops->heading()) - nearest);
        step(ops, "seek_back", lround(back));
        k = back > 0.5 ? ops->seek_edge(right_sensor, back, at) : EDGE_CAPPED;
        if (ops->failed() || k == EDGE_FAILED)
          return 0;
        if (k != EDGE_FOUND)
          break;
        if (end_plausible(lens_y(ops, at[1], at[2]), dir))
          found = 1;
        else
          step(ops, "end_reject", lround(lens_y(ops, at[1], at[2])));
      }
      if (found)
        record_end2(ops, lens_y(ops, at[1], at[2]));
      else if (bound_on(dir))
      {
        step(ops, "seek_bound", lround(us_end_bound));
        record_end2(ops, us_end_bound);
      }
      else
      {
        step(ops, "seek_blind", 0);
        assume_max2 = 1;
        recompute_end2();
      }
    }
  }
  {
    /* The 180 round the end starts with the axle TURN_AXLE_SHORT short of the
     * end - where a leg that read the end on the move stops - and keeps off
     * the wall past the end and the end's corner, on arcs ARC_ERR either way:
     * if the wall is too near, it starts that much sooner, as long as the
     * corner still clears. */
    const double end = dir > 0.0 ? end2_hi : end2_lo;
    const double target = end - dir * T2_TURN_AXLE_SHORT_CM;
    const double r2[2] = {arc(1, 0), arc(0, 0)};
    const double plan[3] = {toward ? 180.0 : -180.0, 0.0, 0.0};
    const Box2 b2 = obstacle2_box(T2_OBST2_CLEAR_CM), wl = wall_box(0), wh = wall_box(1);
    double e = 0.0;
    int ok = 0;
    for (; e <= T2_WALL_SHIFT_MAX_CM; e += 1.0)
    {
      const double from[3] = {o->x, target - dir * e, heading(ops)};
      if (plan_clear_err(from, plan, r2, &b2) && plan_clear_err(from, plan, r2, &wl) &&
          plan_clear_err(from, plan, r2, &wh))
      {
        ok = 1;
        break;
      }
    }
    if (!ok)
    {
      e = 0.0;
      step(ops, "wall_tight", 180);
    }
    else if (e > 0.5)
      step(ops, "wall_180", lround(e));
    const double move = dir * ((target - dir * e) - o->y);
    if (fabs(move) > 0.5)
    {
      step(ops, "to_turn", lround(move));
      ops->straight(move);
      if (ops->failed())
        return 0;
    }
  }
  step(ops, "end2", 0);

  /* Round the end in one 180, no straight: how far off the back face it
   * lands is set by the turn's radius alone. */
  if (!turn(ops, toward, 180.0))
    return 0;
  return back_leg(ops, right_sensor);
}

/* ------------------------------------------------------------------ home -- */

/* Forward room an S needs to shift `lateral` sideways on radius r: two equal
 * arcs, and past 2r two quarter turns with a sideways straight. */
static double s_bend(double lateral, double r)
{
  double c = fmax(0.0, 1.0 - fabs(lateral) / (2.0 * r));
  return 2.0 * r * sqrt(1.0 - c * c);
}

/* Obstacle 1 as the way home must keep off it: the swerve's box and
 * HOME_CLEAR_CM more all round. By the time the car heads home the odometry
 * has run the whole course - round obstacle 2 and back - and on paper the
 * swerve's 2 cm let the diagonal home clip obstacle 1's back corner on the
 * floor. */
static Box2 home_box(double face1)
{
  Box2 b = obstacle1_box(face1);
  b.x0 -= T2_HOME_CLEAR_CM;
  b.x1 += T2_HOME_CLEAR_CM;
  b.y0 -= T2_HOME_CLEAR_CM;
  b.y1 += T2_HOME_CLEAR_CM;
  return b;
}

/* Obstacle 2 as the way home keeps off it, once the face scan has placed its
 * ends: its face from the stop at it, its span from recompute_end2, and
 * HOME_CLEAR_CM all round. Before this the way home planned round obstacle 1
 * alone and turned home on the stop past obstacle 2's end, which on this car
 * could swing its side into that end (course sim). */
static Box2 home_box2(void)
{
  return obstacle2_box(T2_HOME_CLEAR_CM);
}

/* Does a planned turn-straight-turn from `from` keep the car off the box? */
static int plan_clear(const double from[3], const double plan[3], const double r[2], const Box2 *b)
{
  float x = (float)from[0], y = (float)from[1], h = (float)RAD(from[2]);
  if (!arc_clear(&x, &y, &h, (float)plan[0], (float)(plan[0] > 0.0 ? r[0] : r[1]), b))
    return 0;
  for (float s = 0.0f; s < plan[1]; s += 2.0f)
    if (car_hits(x + s * cosf(h), y + s * sinf(h), h, b))
      return 0;
  x += (float)plan[1] * cosf(h);
  y += (float)plan[1] * sinf(h);
  return arc_clear(&x, &y, &h, (float)plan[2], (float)(plan[2] > 0.0 ? r[0] : r[1]), b);
}

/* ...on the arcs as measured, and on arcs ARC_ERR_CM wider and tighter: the
 * same plan driven on a turn that strays sweeps the car somewhere else. */
static int plan_clear_err(const double from[3], const double plan[3], const double r[2], const Box2 *b)
{
  for (int k = -1; k <= 1; ++k)
  {
    const double rk[2] = {r[0] + k * T2_ARC_ERR_CM, r[1] + k * T2_ARC_ERR_CM};
    if (!plan_clear(from, plan, rk, b))
      return 0;
  }
  return 1;
}

/* Can the car make the entry pose - on the carpark's line, facing in - from
 * here in one turn-straight-turn that keeps obstacle 1 clear on paper, with
 * the swerve's margins? If not at *entry_x, then deeper in, down to
 * PARK_ENTRY_MIN_CM: the further past obstacle 1 the car straightens up, the
 * steeper the diagonal can cut in behind it - and a direct route home is ~1.5 s
 * quicker than the lane's (2026-10-15: 7.0 s against 8.7). The depth costs
 * nothing: the diagonal covers at full speed what the straight in no longer
 * does. *entry_x becomes the depth that fits. (Only a check: goto_pose drives
 * it, without this frame under it - the course runs on a 2 KB task stack.) */
/* The deepest the way home may straighten up on the carpark's line: the nose
 * ENTRY_CLEAR outside the mouth, once the mouth is known. */
static double entry_min(void)
{
  return have_mouth ? fmax(T2_PARK_ENTRY_MIN_CM, mouth_x + T2_FRONT_CM + T2_ENTRY_CLEAR_CM)
                    : T2_PARK_ENTRY_MIN_CM;
}

/* ...off the carpark's side fences' ends (FENCE_CLEAR), once the mouth is
 * known: the fences run BAY_DEPTH in from it, BAY_HALF either side of bay_y. */
static int fence_clear(const double from[3], const double plan[3], const double r[2])
{
  if (!have_mouth)
    return 1;
  const float c = T2_FENCE_CLEAR_CM, x0 = (float)(mouth_x - T2_BAY_DEPTH_CM - 10.0), x1 = (float)mouth_x + c;
  const Box2 fl = {x0, x1, (float)(bay_y + T2_BAY_HALF_CM) - c, (float)(bay_y + T2_BAY_HALF_CM + 10.0)};
  const Box2 fr = {x0, x1, (float)(bay_y - T2_BAY_HALF_CM - 10.0), (float)(bay_y - T2_BAY_HALF_CM) + c};
  return plan_clear_err(from, plan, r, &fl) && plan_clear_err(from, plan, r, &fr);
}

/* ...and off the arena walls (wall_box), the same way. */
static int walls_clear(const double from[3], const double plan[3], const double r[2])
{
  const Box2 wl = wall_box(0), wh = wall_box(1);
  return plan_clear_err(from, plan, r, &wl) && plan_clear_err(from, plan, r, &wh);
}

static int home_direct(const Task2Ops *ops, double *entry_x, const Box2 *box1, const Box2 *box2)
{
  volatile Task2Odom *o = ops->odom;
  const double from[3] = {o->x, o->y, heading(ops)};
  const double r[2] = {arc(1, 0), arc(0, 0)}, deeper = (*entry_x - entry_min()) / 2.0;
  double plan[3];
  for (int k = 0; k < 3 && (k == 0 || deeper > 1.0); ++k)
  {
    const double to[3] = {*entry_x - k * deeper, bay_y, 180.0};
    if (Task2_PlanCSC(from, to, r, r, plan) && fabs(plan[0]) + fabs(plan[2]) <= T2_MAX_PLAN_TURN_DEG &&
        plan_clear_err(from, plan, r, box1) && (!box2 || plan_clear_err(from, plan, r, box2)) &&
        walls_clear(from, plan, r) && fence_clear(from, plan, r))
    {
      *entry_x = to[0];
      step(ops, "home_direct", lround(plan[1]));
      return 1;
    }
  }
  return 0;
}

/* ...off box b, which carries `margin` of clearance on every side - keeping
 * at least what clearance the car has now, if it is already inside that
 * margin. A straight home that wandered 14 cm off its line (course sim,
 * 5 degrees a metre) left the car 4.6 cm off the nearest a wall can be:
 * against the full 5 cm every plan from there failed, and the car drove in
 * from 43 cm off the carpark's line. */
static int plan_clear_now(const double from[3], const double plan[3], const double r[2], const Box2 *b,
                          float margin)
{
  Box2 c = *b;
  const float x = (float)from[0], y = (float)from[1], h = (float)RAD(from[2]);
  for (float m = 0.0f; m < margin && car_hits(x, y, h, &c); m += 1.0f)
  {
    c.x0 += 1.0f;
    c.x1 -= 1.0f;
    c.y0 += 1.0f;
    c.y1 -= 1.0f;
  }
  return plan_clear_err(from, plan, r, &c);
}

/* Does a plan from `from` keep off everything the way home must: obstacle 1,
 * obstacle 2 once placed, the walls and the carpark's fences' ends? */
static int route_clear(const double from[3], const double plan[3], const double r[2],
                       const Box2 *box1, const Box2 *box2)
{
  const Box2 wl = wall_box(0), wh = wall_box(1);
  if (!plan_clear_now(from, plan, r, box1, T2_HOME_CLEAR_CM) ||
      (box2 && !plan_clear_now(from, plan, r, box2, T2_HOME_CLEAR_CM)) ||
      !plan_clear_now(from, plan, r, &wl, T2_WALL_CLEAR_CM) || !plan_clear_now(from, plan, r, &wh, T2_WALL_CLEAR_CM))
    return 0;
  if (!have_mouth)
    return 1;
  const float c = T2_FENCE_CLEAR_CM, x0 = (float)(mouth_x - T2_BAY_DEPTH_CM - 10.0), x1 = (float)mouth_x + c;
  const Box2 fl = {x0, x1, (float)(bay_y + T2_BAY_HALF_CM) - c, (float)(bay_y + T2_BAY_HALF_CM + 10.0)};
  const Box2 fr = {x0, x1, (float)(bay_y - T2_BAY_HALF_CM - 10.0), (float)(bay_y - T2_BAY_HALF_CM) + c};
  return plan_clear_now(from, plan, r, &fl, c) && plan_clear_now(from, plan, r, &fr, c);
}

/* The entry depth for an S from `from` onto the carpark's line: want_x
 * first, then 2 cm at a time down to PARK_ENTRY_MIN, the first one a plan
 * reaches without a near-full circle and route_clear. */
static int entry_plan(const double from[3], double want_x, const Box2 *box1, const Box2 *box2, double *tx,
                      double plan[3])
{
  const double r[2] = {arc(1, 0), arc(0, 0)};
  for (double x = fmin(want_x, from[0] - 2.0); x >= T2_PARK_ENTRY_MIN_CM - 0.01; x -= 2.0)
  {
    const double to[3] = {x, bay_y, 180.0};
    if (Task2_PlanCSC(from, to, r, r, plan) && fabs(plan[0]) + fabs(plan[2]) <= T2_MAX_PLAN_TURN_DEG &&
        route_clear(from, plan, r, box1, box2))
    {
      *tx = x;
      return 1;
    }
  }
  return 0;
}

/* Onto the carpark's line facing in, within ENTRY_ALIGN on the odometry:
 * the park drives straight in from there, and an entry off the line drives on
 * past the carpark beside its fence. On the floor (2026-09-14 14:39) the home
 * run's straight drifted 3 degrees, the lane's S no longer fitted the room
 * left, goto_pose refused it (plan_loop 357) and squared up 51 cm off the
 * line; the park drove in from there.
 *
 * Now the S goes to the shallowest depth entry_plan finds. With none from
 * here, facing home, the car first moves along the lane - on past obstacle 1,
 * or back the way it came for room - by the least (4 cm steps, LINE_UP_SHIFT
 * either way) from which one fits. Then it looks again from wherever the S
 * left it.
 *
 * A plan with a straight over LINE_UP_LEG drives its first turn and the
 * straight bar half of LINE_UP_LEG, then plans the rest again: the straights
 * wander off their heading on the floor (2026-09-14: 2-3 degrees over the
 * ~60 cm home run, every run), and over a 2 m diagonal that is a fence. */
static int line_up(const Task2Ops *ops, double want_x, const Box2 *box1, const Box2 *box2)
{
  volatile Task2Odom *o = ops->odom;
  const double r[2] = {arc(1, 0), arc(0, 0)};
  for (int k = 0; k < T2_LINE_UP_TRIES; ++k)
  {
    const double from[3] = {o->x, o->y, heading(ops)};
    const double crooked = Task2_TurnToward(from[2], 180.0);
    double tx = want_x, plan[3];
    if (k && fabs(o->y - bay_y) <= T2_ENTRY_ALIGN_CM && fabs(crooked) < T2_SQUARE_MIN_DEG)
      return 1;
    const int found = entry_plan(from, want_x, box1, box2, &tx, plan);
    if (found && plan[1] > T2_LINE_UP_LEG_CM && k + 1 < T2_LINE_UP_TRIES)
    {
      step(ops, "line_leg", lround(plan[1]));
      set_speed(ops, 0);
      if (!turn_by(ops, plan[0]))
        return 0;
      ops->straight(plan[1] - T2_LINE_UP_LEG_CM / 2.0);
      if (ops->failed())
        return 0;
      continue;
    }
    if (!found && fabs(crooked) < 30.0)
    {
      int shifted = 0;
      const uint32_t t0 = ops->now_ms();
      for (int j = 1; j <= 2 * (int)(T2_LINE_UP_SHIFT_CM / 4.0f) && !shifted; ++j)
      {
        const double e = (j & 1 ? 4.0 : -4.0) * (double)((j + 1) / 2), h = RAD(from[2]);
        ops->wait_ms(1); /* a search of up to ~300 plans: keep the gyro and the link serviced */
        const double at[3] = {from[0] + e * cos(h), from[1] + e * sin(h), from[2]};
        const double leg[3] = {0.0, fabs(e), 0.0};
        if (!route_clear(e > 0.0 ? from : at, leg, r, box1, box2) ||
            !entry_plan(at, want_x, box1, box2, &tx, plan))
          continue;
        step(ops, "line_ms", (long)(ops->now_ms() - t0));
        step(ops, "line_shift", lround(e));
        ops->straight(e);
        if (ops->failed())
          return 0;
        shifted = 1;
      }
      if (shifted)
        continue;
      step(ops, "line_ms", (long)(ops->now_ms() - t0));
      tx = want_x; /* nothing fits: as goto_pose plans it */
    }
    step(ops, "line_up", lround(tx));
    if (!goto_pose(ops, tx, bay_y, 180.0, 0, 0))
      return 0;
  }
  /* Still off the line after every try: in, if the car fits between the
   * fences from here with FENCE_CLEAR to spare; otherwise it stays out -
   * FAIL,STOP - rather than drive into a fence's end. */
  const double off = o->y - bay_y;
  if (fabs(off) > T2_ENTRY_ALIGN_CM)
    step(ops, "line_off", lround(off * 10.0));
  if (fabs(off) > T2_BAY_HALF_CM - T2_HALF_WIDTH_CM - T2_FENCE_CLEAR_CM)
    return 0;
  return !ops->failed();
}

/* One side IR as a median of five readings a control tick or two apart,
 * standing still: 1 with *cm, else 0 and *near if most said "closer than
 * 10 cm". */
static int ir_median(const Task2Ops *ops, int right, double *cm, int *near)
{
  double v[5];
  int n = 0, nears = 0;
  for (int k = 0; k < 5; ++k)
  {
    double c;
    int nr = 0;
    if (k)
      ops->wait_ms(20);
    if (ops->ir(right, &c, &nr))
      v[n++] = c;
    else
      nears += nr;
  }
  *near = nears >= 3;
  if (n < 3)
    return 0;
  for (int i = 1; i < n; ++i) /* n <= 5: insertion sort */
    for (int j = i; j > 0 && v[j] < v[j - 1]; --j)
    {
      double t = v[j];
      v[j] = v[j - 1];
      v[j - 1] = t;
    }
  *cm = v[n / 2];
  return 1;
}

/* The robot's offset from the bay's centreline, heading in, from the side IRs
 * (task2mn_park.centre_offset_measured). Facing in, the left IR faces -y:
 * left = y + c and right = c - y, c the lens-to-wall gap when centred, so
 * y = (left - right) / 2 whatever the mounting. Both walls, an IR inside its
 * 10 cm floor, or one wall alone; 0 when none of them says. */
static int bay_offset(const Task2Ops *ops, double *y)
{
  const double c = T2_BAY_HALF_CM - T2_IR_LENS_CM, limit = 2.0 * c + T2_CENTRE_SUM_TOL_CM;
  double l = 0.0, r = 0.0;
  int ln = 0, rn = 0;
  const int lok = ir_median(ops, 0, &l, &ln), rok = ir_median(ops, 1, &r, &rn);
  if (lok && rok && fabs(l + r - 2.0 * c) <= T2_CENTRE_SUM_TOL_CM)
  {
    *y = (l - r - T2_IR_LR_BIAS_CM) / 2.0;
    step(ops, "bay_both", lround(*y * 10.0));
  }
  else if (ln && !rn)
  {
    *y = T2_NEAR_FENCE_GAP_CM - c;
    step(ops, "bay_near_l", lround(*y * 10.0));
  }
  else if (rn && !ln)
  {
    *y = c - T2_NEAR_FENCE_GAP_CM;
    step(ops, "bay_near_r", lround(*y * 10.0));
  }
  else if (lok && !rok && l <= limit)
  {
    *y = l - c - T2_IR_LR_BIAS_CM / 2.0;
    step(ops, "bay_left", lround(*y * 10.0));
  }
  else if (rok && !lok && r <= limit)
  {
    *y = c - r - T2_IR_LR_BIAS_CM / 2.0;
    step(ops, "bay_right", lround(*y * 10.0));
  }
  else
  {
    step(ops, "bay_none", lround((lok ? l : -1.0) * 10.0));
    return 0;
  }
  return 1;
}

/* A side IR under PARK_SIDE_STOP on the way in: one lane change onto the
 * carpark's line as the side IRs read it, backing out first, straight, if the
 * bay has not the room left for the S before stop_front. */
static int park_side_fix(const Task2Ops *ops, double stop_front)
{
  volatile Task2Odom *o = ops->odom;
  const double r = fmax(arc(1, 0), arc(0, 0));
  double y;
  ops->wait_ms(100); /* the IR medians full of standing samples */
  if (!bay_offset(ops, &y))
    return !ops->failed();
  const double sb = s_bend(y, r) + 2.0, room = (o->x - T2_FRONT_CM) - stop_front;
  if (room < sb)
  {
    const double back = fmin(sb - room, T2_CENTRE_BACK_MAX_CM);
    step(ops, "centre_back", lround(back));
    ops->straight(-back);
    if (ops->failed())
      return 0;
  }
  step(ops, "centre", lround(-y * 10.0));
  return goto_pose(ops, o->x - sb, o->y - y, 180.0, 0, 0);
}

/* In from the entry pose - line_up has put it on the carpark's line - on the
 * ultrasonic, forward only, until it reads PARK_US: one straight, then a look
 * and a nudge more while it is short by over PARK_TOL. The side IRs only
 * guard: one under PARK_SIDE_STOP stops the car, and park_side_fix moves it
 * off that fence once. No centring otherwise - on the floor (2026-09-14
 * 14:38) it backed out 23 cm and changed lanes for a 4.5 cm offset, with 15 cm
 * to spare each side.
 *
 * A reading past the deepest the back wall can be is not the back wall: the
 * car drives to that depth and no further.
 *
 * The mouth is where the mouth watch saw it, else no nearer than the start
 * nose (the car started inside). A stop that leaves the tail out past it
 * stopped on something short of the back wall: on the floor (2026-10-15 05:32)
 * an echo 24 cm short parked the car 22 cm short with its tail out of the
 * carpark. Look once more from there. A missed echo after the first look
 * leaves it where it is: it must not fault a car that has parked. */
static int park(const Task2Ops *ops, int *no_range)
{
  volatile Task2Odom *o = ops->odom;
  const double stop = T2_PARK_US_CM + ULTRA_OFFSET_CM; /* the bumper gap at that reading */
  const double mouth = have_mouth ? fmax(mouth_x, T2_FRONT_CM) : T2_FRONT_CM;
  const double back_min = mouth - T2_BAY_DEPTH_CM - T2_PARK_BACK_SLACK_CM;
  int ok = 1, fixed = 0, out = 0;
  ops->brake_hard(1);
  for (int k = 0; k < T2_PARK_LOOKS + fixed + out; ++k)
  {
    double gap;
    int missed = 0;
    if (!range_ahead(ops, 0, &gap, &missed))
    {
      if (!k)
        *no_range = 1, ok = 0;
      break;
    }
    const double most = o->x - T2_FRONT_CM - back_min;
    if (gap > most)
    {
      step(ops, "park_far", lround(gap));
      gap = most;
    }
    const double back_x = o->x - T2_FRONT_CM - gap; /* facing in: forward is -x */
    if (!k)
      step(ops, "bay", lround(back_x + T2_BAY_DEPTH_CM));
    const double travel = gap - stop;
    if (travel <= T2_PARK_TOL_CM)
    {
      if (out || o->x + T2_REAR_CM <= mouth)
        break;
      out = 1;
      step(ops, "park_out", lround(o->x + T2_REAR_CM - mouth));
      continue;
    }
    ops->side_guard(T2_PARK_SIDE_STOP_CM);
    ops->straight(travel);
    ops->side_guard(0.0);
    if (ops->failed())
    {
      ok = 0;
      break;
    }
    if (ops->side_guarded())
    {
      step(ops, "side_stop", k);
      if (fixed)
        continue;
      fixed = 1;
      if (!park_side_fix(ops, back_x + stop))
      {
        ok = 0;
        break;
      }
    }
  }
  if (ok)
  {
    double y;
    if (bay_offset(ops, &y))
      step(ops, "parked_off", lround(y * 10.0));
  }
  ops->brake_hard(0);
  return ok;
}

/* Home, from obstacle 2's far end, then in on the ultrasonic - forward only,
 * it never backs out. Odometry all the way to the entry pose; the ultrasonic
 * only for the last straight in.
 *
 * Straight for the entry when obstacle 1 allows (home_direct): from the far
 * end itself - one turn on round it, a diagonal, one turn onto the line - or
 * else from the lane after turning to face home, as one long shallow S. The
 * route it replaces, still the fallback, drove past obstacle 1 first and then
 * squeezed its S into the room between obstacle 1 and the carpark: on the
 * floor (2026-10-15 01:02) two ~70 degree turns, 5 s on their own.
 *
 * The fallback, round the outside as the floor video drove it - always clear
 * of obstacle 1, which stands on the carpark's line:
 *   1. carry on round obstacle 2's far end to face home (one open turn);
 *   2. straight on past obstacle 1 at whatever width the far end left;
 *   3. one S onto the carpark's line, sized from the real width, ending at
 *      the entry pose - on the line, facing in, just outside the carpark.
 * Every way one plan, one attempt: a second would be a second S. */
static int go_home(const Task2Ops *ops, double face1, int *no_range)
{
  volatile Task2Odom *o = ops->odom;
  const double r = fmax(arc(1, 0), arc(0, 0));
  /* On the carpark's line, facing in: PARK_ENTRY_CM out, or with the mouth
   * known the nose ENTRY_CLEAR outside it if that is further - and short of
   * obstacle 1 either way. */
  double entry_x = fmin(fmax(T2_PARK_ENTRY_CM, entry_min()),
                        face1 - T2_REAR_CM - T2_SWERVE_CLEAR_CM - 2.0);
  const Box2 box1 = home_box(face1);
  const Box2 b2 = home_box2();
  const Box2 *box2 = have_end2 ? &b2 : 0;

  int direct = home_direct(ops, &entry_x, &box1, box2);
  if (!direct)
  {
    {
      /* The turn to face home starts beside obstacle 2's far end, and swings
       * the car out past it by about its radius plus the car: on the floor
       * (2026-09-14 13:49) it ran into the wall there (FAIL,WRONGWAY). Move
       * along the leg first - back toward the end, or on past it - by the
       * least that keeps the turn off obstacle 2's end AND the wall: 0, -2,
       * +2, -4, ... */
      const double rr[2] = {arc(1, 1), arc(0, 1)};
      const double h = heading(ops), turn_deg = Task2_TurnToward(h, 180.0);
      const double plan[3] = {turn_deg, 0.0, 0.0};
      double e = 0.0;
      int ok = 0;
      for (int k = 0; k <= 2 * (int)T2_HOME_TURN_SHIFT_CM && !ok; ++k)
      {
        e = (k & 1 ? -1.0 : 1.0) * (double)((k + 1) / 2) * 2.0;
        if (fabs(e) > T2_HOME_TURN_SHIFT_CM)
          continue;
        const double from[3] = {o->x + e * cos(RAD(h)), o->y + e * sin(RAD(h)), h};
        ok = (!box2 || plan_clear_err(from, plan, rr, box2)) && walls_clear(from, plan, rr);
      }
      if (!ok)
      {
        e = 0.0;
        step(ops, "wall_tight", 90);
      }
      else if (fabs(e) > 0.5)
      {
        step(ops, "home_shift", lround(e));
        ops->straight(e);
        if (ops->failed())
          return 0;
      }
    }
    set_speed(ops, 1);
    if (!turn_by(ops, Task2_TurnToward(heading(ops), 180.0)))
      return 0;
    set_speed(ops, 0);
    step(ops, "lane", lround(o->y));
    direct = home_direct(ops, &entry_x, &box1, box2);
  }
  if (direct)
  {
    if (!line_up(ops, entry_x, &box1, box2))
      return 0;
    step(ops, "entry", 0);
    return park(ops, no_range);
  }
  /* The S turns everything ahead of the axle inward, so it starts only once
   * the axle is past obstacle 1 - and, room allowing, ends at the entry. */
  double sb = s_bend(o->y - bay_y, r);
  double s_start = fmin(face1 - T2_SWERVE_CLEAR_CM, entry_x + sb + 2.0);
  /* Start the S HOME_SHORTEN_CM sooner: a shorter straight before it, and
   * the ultrasonic takes in the extra at the end. */
  double run = o->x - s_start - T2_HOME_SHORTEN_CM;
  {
    /* Starting HOME_SHORTEN_CM sooner can begin the S with the car still
     * beside obstacle 1: go on further, 2 cm at a time, until the S - as
     * goto_pose will plan it from there - keeps off it. */
    const double r2[2] = {arc(1, 0), arc(0, 0)};
    double extra = 0.0;
    for (double e = 0.0; e <= T2_HOME_SHORTEN_CM + T2_HOME_CLEAR_CM; e += 2.0)
    {
      const double sx = o->x - fmax(run + e, 0.0);
      const double from[3] = {sx, o->y, heading(ops)}, to[3] = {sx - sb - 2.0, bay_y, 180.0};
      double plan[3];
      if (to[0] < entry_min())
        break; /* no deeper than the lane's S has parked from */
      extra = e;
      if (Task2_PlanCSC(from, to, r2, r2, plan) && plan_clear_err(from, plan, r2, &box1) &&
          (!box2 || plan_clear_err(from, plan, r2, box2)) && walls_clear(from, plan, r2) &&
          fence_clear(from, plan, r2))
        break;
    }
    if (extra > 0.0)
      step(ops, "home_later", lround(extra));
    run += extra;
  }
  step(ops, "home_run", lround(run));
  if (run > 0.5)
  {
    ops->straight(run); /* heading 180: forward is toward home */
    if (ops->failed())
      return 0;
  }
  if (!line_up(ops, o->x - sb - 2.0, &box1, box2))
    return 0;
  step(ops, "entry", 0);
  return park(ops, no_range);
}

/* ---------------------------------------------------------------- course -- */

int Task2Mission_Run(const Task2Ops *ops)
{
  int no_range = 0;
  double face1, face2, lateral;
  Task2Odom_Reset(ops->odom, ops->heading());
  axis_y = 0.0;
  speed_open = 0;
  face2_peek = face2_far = 1e9;
  peek_tight = 0;
  have_end2 = assume_max2 = have_mouth = have_end_bound = 0;
  bay_y = mouth_x = 0.0;
  meas_lo = meas_hi = NO_END;
  face2_x = end2_lo = end2_hi = 0.0;
  ops->side_guard(0.0);
  ops->guard(0.0);
  reset_arcs();
  step(ops, "start", 0);

  /* Obstacle 1: ask first, so the Pi is looking while the car closes in. */
  request_arrow(ops, 1);
  /* The carpark's centreline from its fences, before moving: facing out,
   * bay_offset()'s (left - right) / 2 is where the centreline is. */
  bay_y = 0.0;
  if (!bay_offset(ops, &bay_y) || fabs(bay_y) > T2_BAY_HALF_CM / 2.0)
    bay_y = 0.0;
  step(ops, "bay_y", lround(bay_y * 10.0));
  ops->mouth_watch(1);
  if (!approach(ops, T2_PHOTO_1_CM, 1, 0, -1e9, &face1, &no_range))
    return no_range ? T2M_NO_ULTRASONIC : T2M_FAULT;
  step(ops, "photo1", lround(face1));
  char arrow1 = await_arrow(ops, 1, T2_ANSWER_TIMEOUT_MS);
  if (!arrow1)
    return ops->failed() ? T2M_FAULT : T2M_NO_ARROW;
  step(ops, "arrow1", arrow1 == 'R');
  /* Where obstacle 1 sits sideways, from the arrow's place in the photo. The
   * car drove straight along y = 0 to here, so that is where it looked from. */
  if (ops->lateral(1, &lateral) && fabs(lateral) <= T2_LATERAL_MAX_CM)
    axis_y = ops->odom->y + lateral;
  step(ops, "axis_y", lround(axis_y));
  recompute_end2(); /* the walls where the narrowest obstacle 2 would put them */
  if (!swerve(ops, arrow1, face1))
    return T2M_FAULT;
  step(ops, "swerved", 0);
  /* Out of the carpark by now even when obstacle 1 stands close to its mouth:
   * the photo stop can leave the side IRs still beside the fences. */
  ops->mouth_watch(0);
  have_mouth = ops->mouth_seen(&mouth_x);
  step(ops, "mouth", have_mouth ? lround(mouth_x) : -1);

  /* Obstacle 2. Ask, then range it from standstill while the Pi looks: on the
   * floor the answer came 0.38-0.62 s after P,2, about what the median takes.
   * With the arrow in, stop once - turn_stop() for that side, no move at all
   * if the swerve left the car within NUDGE_MIN_CM past it - and turn. That
   * was three steps: a stop sized for either side, a re-check, and a creep
   * in to the arrow's side. If the Pi is slower than ARROW2_WAIT_MS, still
   * that way: stop where either side fits, wait there, close in. Never back
   * up to close in - obstacle 1 may be right behind. */
  /* The peek asked for arrow 2 already: ask again from here, for a photo from
   * where the car squared up - without clearing an answer on its way in. */
  if (!ops->answer(2))
    ops->send("P,2\n");
  const uint32_t asked2 = ops->now_ms();
  /* Far enough off the line, the tail may pass beside obstacle 1. */
  const double rear2 = fabs(ops->odom->y - axis_y) >= T2_OBSTACLE_DEPTH_CM / 2.0 + T2_HALF_WIDTH_CM +
                                                       T2_SWERVE_CLEAR_CM + T2_LATERAL_ERR_CM
                           ? -1e9
                           : face1 + T2_OBSTACLE_DEPTH_CM + T2_SWERVE_CLEAR_CM + 0.5;
  const double tol2 = T2_NUDGE_MIN_CM / 2.0; /* aim tol2 past the stop: the window is [stop, stop + NUDGE_MIN] */
  double gap2;
  const double peek_gap = face2_peek - (ops->odom->x + T2_FRONT_CM);
  int missed = 0;
  if (!range_ahead(ops, 0, &gap2, &no_range))
  {
    /* off the line, the beam can pass a short obstacle 2 with no echo */
    if (!peek_tight || ops->failed())
      return no_range ? T2M_NO_ULTRASONIC : T2M_FAULT;
    no_range = 0;
    gap2 = peek_gap;
    missed = 1;
  }
  if (peek_tight && gap2 > peek_gap + T2_RECHECK_MAX_CM)
  {
    step(ops, "peek_wins", lround(peek_gap)); /* the echo looked past it */
    gap2 = peek_gap;
    missed = 1;
  }
  if (missed && fabs(ops->odom->y - axis_y) > T2_OBST2_MIN_HALF_CM - T2_OBST2_OFFSET_CM)
  {
    /* The beam missed obstacle 2 from off its line: its end on this side is
     * inward of the car's line, by half the beam's spread at that range
     * (on 2026-09-14 14:28 the IR then took a box 50 cm further out for
     * obstacle 2's end). */
    const double in = ops->odom->y > axis_y ? -1.0 : 1.0;
    have_end_bound = 1;
    us_end_bound = ops->odom->y + in * 0.5 * fmax(peek_gap, 0.0) * tan(RAD(T2_ULTRA_CONE_DEG));
    step(ops, "end_bound", lround(us_end_bound));
  }
  /* Off the line the cone can catch obstacle 2's end, up to its depth beyond
   * the face (course sim: 5 cm long, and the stop that close put the back face
   * out of the IR's reach). The peek bounds where the face can be. */
  if (peek_tight)
    gap2 = fmin(gap2, face2_far - (ops->odom->x + T2_FRONT_CM));
  uint32_t waited = ops->now_ms() - asked2;
  /* Inside both sides' stops the car backs up whatever the arrow says - and so
   * close the camera may not read it at all (5 cm, 2026-10-15 03:00: nothing
   * in three tries): back up first, read it from the stop. */
  const double stop_l = turn_stop('L'), stop_r = turn_stop('R');
  char arrow2 = gap2 < fmin(stop_l, stop_r) ? 0
                : await_arrow(ops, 2, waited < T2_ARROW2_WAIT_MS ? T2_ARROW2_WAIT_MS - waited : 0U);
  if (ops->failed())
    return T2M_FAULT;
  double stop2 = arrow2 ? turn_stop(arrow2) : fmax(stop_l, stop_r);
  double aim2 = stop2 + tol2;
  /* Squared up off the line - too close to come back onto it - no creep in to
   * the stop: turn from here, if the turn still works from this far out
   * (turn_far), else from as far out as it does (T2_TURN_HERE). */
  if (T2_TURN_HERE && peek_tight && arrow2 && gap2 > aim2)
  {
    aim2 = fmin(gap2, turn_far(arrow2));
    step(ops, "turn_here", lround(gap2 - aim2)); /* how far it still moves in: 0 = none */
  }
  step(ops, "stop2", lround(stop2));
  if (!drive_to(ops, gap2, aim2, 0, 0, tol2, rear2, &face2, &no_range))
    return no_range ? T2M_NO_ULTRASONIC : T2M_FAULT;
  step(ops, "photo2", lround(face2));
  if (!arrow2)
  {
    arrow2 = await_arrow(ops, 2, T2_ANSWER_TIMEOUT_MS);
    if (!arrow2)
      return ops->failed() ? T2M_FAULT : T2M_NO_ARROW;
    double close_in = face2 - (ops->odom->x + T2_FRONT_CM) - (turn_stop(arrow2) + tol2);
    step(ops, "close_in", lround(close_in));
    if (close_in > tol2)
    {
      ops->straight(close_in);
      if (ops->failed())
        return T2M_FAULT;
    }
  }
  step(ops, "arrow2", arrow2 == 'R');

  face2_x = face2;
  if (!round_obstacle2(ops, arrow2, face2))
    return T2M_FAULT;

  if (!go_home(ops, face1, &no_range))
    return no_range ? T2M_NO_ULTRASONIC : T2M_FAULT;
  step(ops, "parked", 0);
  return T2M_OK;
}
