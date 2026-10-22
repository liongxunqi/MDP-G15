#ifndef TASK2_MISSION_H
#define TASK2_MISSION_H
/*
 * Task 2 (fastest car), run entirely on the STM.
 *
 * Ported from the MDP_WORKING_2 firmware. On THIS robot the wire contract is
 * the !t2go / !t2a / !t2y tokens of PROTOCOL.md section 14, not the CF000 /
 * PF0nd / Y frames described below - task2fw.c translates. The rest of this
 * comment is the original's and still describes the course.
 *
 * The Pi's only part is the arrow. The firmware drives, senses, decides and
 * parks; when it needs an arrow it asks, and carries on driving while the Pi
 * looks. That ordering is the whole reason this lives here: the previous
 * Pi-side state machine paid a serial round trip for every decision and took
 * 39.9 s to reach obstacle 2.
 *
 * Original wire contract (theirs; ours is PROTOCOL.md section 14):
 *   Pi  -> STM   MF<sss>       arm the mission clock (existing)
 *   Pi  -> STM   CF000         run the course; 'A' when it ends, nothing on a fault
 *   STM -> Pi    P,<n>\n       photograph obstacle n (1 or 2); repeated until answered
 *   Pi  -> STM   PF0<n><d>     obstacle n's arrow: d = 0 left, 1 right
 *   Pi  -> STM   Y<F|B><ddd>   where that arrow sat sideways (sent just before
 *                              its PF): ddd - 500 mm, left positive
 *   STM -> Pi    E,<code>\n    fault; the course has stopped
 *
 * The answer names its obstacle so a late duplicate for obstacle 1 can never be
 * taken as obstacle 2's arrow. Under FAQ 10 that mistake is a disqualification.
 *
 * This module does not touch hardware. It reaches the robot only through
 * Task2Ops, which main.c fills with the real primitives and the host tests fill
 * with a simulated car - so the whole course can be exercised off the robot.
 */
#include <stdint.h>

/* Dead reckoning, updated every control tick from the encoders and the gyro.
 * x points from the start pose toward the obstacles, y to the robot's left;
 * centimetres, in the frame the course STARTED in.
 *
 * Headings are the gyro's total_angle (degrees, counter-clockwise positive -
 * the convention moveCarLeft/Right use), but total_angle accumulates from
 * power-on, so it is not zero when the course begins. h0 records it, and every
 * heading the course reasons about is measured from there.
 *
 * float, not double: the encoder task writes these while the course task reads
 * them, and a 32-bit float is read in one instruction on this Cortex-M4 where
 * a double is two - a read straddling an update would be torn. */
typedef struct {
  volatile float x, y, h0;
} Task2Odom;

void Task2Odom_Reset(volatile Task2Odom *o, double start_heading_deg);
void Task2Odom_Step(volatile Task2Odom *o, double cm, double heading_deg);

typedef struct {
  /* Motion. Each blocks until the move has finished and settled. */
  void (*straight)(double cm);           /* negative = backward */
  void (*left)(double deg);
  void (*right)(double deg);
  /* One IR000/IR001 leg: drive along the obstacle until the side IR loses it,
   * then IR_EDGE_EXTRA_CM more. may_be_past: the first leg after turning
   * beside a short obstacle may already be past its end (see edge_leg).
   * max_cm > 0: a leg whose end is already known (the face scan measured
   * obstacle 2) stops after max_cm even with no edge seen, without a fault;
   * 0: T2_EDGE_MAX_CM, and no edge is a fault.
   * Returns EDGE_FOUND with at = the rear axle's odometry x, y and raw
   * heading() where the IR read the edge - the first "lost" sample, lag
   * corrected - EDGE_PAST if may_be_past and it was already past (nothing
   * moved), EDGE_CAPPED if max_cm ran out first, EDGE_FAILED on a fault. */
  int (*edge_leg)(int right_sensor, int may_be_past, double max_cm, double at[3]);
  /* Back along the leg - straight, in reverse - up to max_cm until the side
   * IR SEES the obstacle: a face scan that started already past the end
   * looks back for it. EDGE_FOUND and stopped, with at = where the edge was
   * read (as edge_leg), or EDGE_CAPPED; EDGE_FAILED on a fault. */
  int (*seek_edge)(int right_sensor, double max_cm, double at[3]);
  /* deg on the left/right lock, watching the side IR right_sensor. Clear
   * (IR_EDGE_OFF_V) at any point once check_deg has been swept - never seen,
   * or seen and swung past its end: the obstacle is too short to be beside
   * the car, so the turn stops there and returns 1. Still seeing it, the
   * whole turn runs: returns 0. at: where the car was at the first clear
   * reading of the run that stopped it - odometry x, y and the raw heading()
   * - not where it stopped, a few degrees on. */
  int (*turn_watch)(int left, double deg, int right_sensor, double check_deg, double at[3]);
  /* Turn speed for the turns that follow. 0 = precise: where the radius has
   * to be what the course planned for (obstacle 1, the turn at obstacle 2,
   * the bend onto the axis). 1 = open: where the course measures the turn as
   * it goes, so a wider, faster arc costs nothing. On the floor a fast right
   * turn ran 32-40 cm against 28-30 at the slower speed. */
  void (*turn_speed)(int open);
  /* 1: the turns that follow end with the steering left at full lock, not
   * swung back to centre - for a turn split in two on the same lock, where the
   * swing back and out again was half a second of standing. A straight still
   * centres it first. 0: back to centring after every move. */
  void (*hold_steering)(int hold);
  /* 1: the straights that follow brake into their stop at T2_HARD_A_DECEL, not
   * the course's gentler T2_A_DECEL - for the last one, into the carpark. */
  void (*brake_hard)(int hard);
  /* The next turn counts from the heading the car has NOW. Otherwise each turn
   * adds to the heading the last one aimed for, which keeps the fixed turns
   * (the swerve, the 90 and the 180 at obstacle 2) square with the arena
   * whatever the car settled at - but a turn the course worked out from
   * heading() then misses by however far the car had drifted off that aim. */
  void (*rebase_heading)(void);
  /* While cm > 0, any move stops early - braked where it is, not a fault -
   * once a fresh ultrasonic reading puts the bumper under cm from whatever is
   * ahead. guarded() says whether one did, and clears it. guard(0): off. */
  void (*guard)(double cm);
  int  (*guarded)(void);
  /* The same for the side IRs: while cm > 0, a FORWARD move stops once
   * either side IR reads under cm (or under its 10 cm floor) - the carpark
   * entry's side-safety check (task2mn_park PARK_SIDE_STOP_CM). */
  void (*side_guard)(double cm);
  int  (*side_guarded)(void);
  /* While on, forward moves watch for the carpark's mouth: both side IRs
   * reading the carpark's side fences (within T2_FENCE_SEEN_CM), then both
   * clear. mouth_seen() gives the odometry x of the lens where they went
   * clear - the mouth - once that has happened this run. */
  void (*mouth_watch)(int on);
  int  (*mouth_seen)(double *x);

  /* Sensing. ultrasonic() returns 1 with a FRESH reading, 0 if stale. */
  int    (*ultrasonic)(uint16_t *cm);
  /* A side IR now: 1 with *cm (lens to target, inside that side's usable
   * range, T2_IR_MIN_CM..T2_IR_MAX_L/R_CM), else 0 - *near 1 when it is
   * closer than its 10 cm floor (saturated), 0 when it sees nothing. */
  int    (*ir)(int right_sensor, double *cm, int *near);
  double (*heading)(void);
  int    (*failed)(void);                /* a fault latched or a move timed out */

  /* The Pi. answer(n) returns 'L' or 'R' once obstacle n has been answered;
   * lateral(n) gives where the arrow sat sideways from the camera's centre
   * line when it was read (cm, left positive), if the Pi said (Y frame). */
  void (*send)(const char *line);
  char (*answer)(int obstacle);
  int  (*lateral)(int obstacle, double *cm);
  void (*clear_answer)(void);

  void     (*wait_ms)(uint32_t ms);
  uint32_t (*now_ms)(void);

  volatile Task2Odom *odom;
} Task2Ops;

/* edge_leg() results. */
enum { EDGE_FAILED = 0, EDGE_FOUND, EDGE_PAST, EDGE_CAPPED };

/* Why the course ended. T2M_OK is the only one that earns an 'A'. */
enum {
  T2M_OK = 0,
  T2M_FAULT,         /* a primitive faulted; its own code is already latched */
  T2M_NO_ULTRASONIC, /* no fresh range for an approach */
  T2M_NO_ARROW       /* the Pi never committed - stop, never guess */
};

int Task2Mission_Run(const Task2Ops *ops);

/* The radii the course plans every turn on, cm, for the next run. task2fw.c
 * sets them from the calibration table just before Task2Mission_Run(). */
void Task2Mission_SetArcs(double left_cm, double right_cm);

/* Pure helpers, exposed for the host tests. */
double Task2_WrapDeg(double deg);                         /* into (-180, 180] */
double Task2_TurnToward(double from_deg, double to_deg); /* signed, left positive */
int    Task2_ParseAnswer(const uint8_t frame[5], int *obstacle, char *arrow);
/* Y<F|B><ddd>: obstacle 1 (F) or 2 (B), lateral ddd - 500 mm, left positive. */
int    Task2_ParseLateral(const uint8_t frame[5], int *obstacle, int *mm);
/* Shortest turn-straight-turn from pose `from` to pose `to` ({x, y, heading
 * deg}): plan = {first turn deg (left +), straight cm, second turn deg}. r0 /
 * r1 are the first / last arc's radii, {left, right}. 0 if no such path. */
int    Task2_PlanCSC(const double from[3], const double to[3],
                     const double r0[2], const double r1[2], double plan[3]);

#endif
