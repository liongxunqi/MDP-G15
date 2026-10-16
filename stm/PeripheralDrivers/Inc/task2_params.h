#ifndef __TASK2_PARAMS_H
#define __TASK2_PARAMS_H

/* ---------------------------------------------------------------------------
 * TASK 2 COURSE CONSTANTS (task2_mission.c, task2fw.c)
 *
 * The course logic is ported from the MDP_WORKING_2 firmware, whose car ran
 * the whole course on the STM (CF000). Its constants were measured on THAT
 * car; the ones below are either course rules (the same for every team) or
 * re-derived for this robot from task2mn_park.py's measured geometry. Every
 * one marked MEASURE has not been checked on this car yet.
 *
 * Distances are cm from the middle of the rear axle - where Odom's world pose
 * is - unless they say bumper. Stops are BUMPER GAPS: ultrasound reading +
 * ULTRA_OFFSET_CM.
 *
 * What this file deliberately does NOT have, unlike the original: speeds,
 * ramps and braking. Every move the course makes is an ordinary
 * Motion_DriveDistanceV / Motion_DriveArcV, so it runs on the calibration
 * table (protocol 10) the RPi restored before the run - trim, coast, distance
 * scale, steer, decel, lag - exactly like a move sent over the wire.
 * ------------------------------------------------------------------------- */

/* ---- this robot --------------------------------------------------------- */
#define T2_CAR_LENGTH_CM      23.0f  /* task2mn_park.ROBOT_LENGTH_CM            */
#define T2_FRONT_CM           20.5f  /* rear axle -> front bumper / ultrasound  */
#define T2_REAR_CM            (T2_CAR_LENGTH_CM - T2_FRONT_CM)
#define T2_HALF_WIDTH_CM      9.4f   /* 18.8 cm wide                            */
/* The original car was an H seen from above (wheels widest, a narrow nose).
 * Nose half width = half width makes this one a plain rectangle - the
 * conservative outline until the real shape is measured. MEASURE. */
#define T2_NOSE_LEN_CM        0.0f
#define T2_NOSE_HALF_CM       T2_HALF_WIDTH_CM

/* Side IRs: ahead of the rear axle, and out from the centreline (their faces,
 * what a reading is measured from). task2mn_park.IR_FROM_AXLE_CM /
 * IR_SIDE_OFFSET_CM. */
#define T2_IR_AHEAD_CM        17.0f
#define T2_IR_LENS_CM         5.25f

/* The arc the rear axle drives in a turn, cm. The course plans every turn on
 * these, so they must be what the robot really drives on the arc profile and
 * speed the run uses (SAFE for Task 2).
 *
 * Taken at run start from the calibration table's radius for FL / FR at the
 * arc speed (caltab.h, mm), else T2_ARC_DEFAULT_CM - the 30.0 the RPi's
 * T2T_RADIUS_CM uses. FROZEN for the run: the original re-measured them from
 * every turn, which this robot does not do during a task (calibration is
 * restored, never learned, in a task). T2_ARC_LEARN 1 brings that back. */
#define T2_ARC_DEFAULT_CM     30.0f
#define T2_ARC_OPEN_EXTRA_CM  0.0f   /* open turns run this much wider          */
#define T2_ARC_ERR_CM         2.0f   /* how far a turn strays from its radius   */
#define T2_ARC_OPEN_ERR_CM    4.0f
#define T2_ARC_LEARN          0
#define T2_ARC_LEARN_DEG      40.0f  /* turns this long are measured and logged */

/* Speeds, rpm at the axle centre. 0 = no V: the table's vs / va defaults if
 * set, else the profile's arc speed and the normal / !FAST straight. Keep them
 * at speeds the table has anchors for (it never extrapolates). */
#define T2_STRAIGHT_RPM       0
#define T2_ARC_RPM_PRECISE    0
#define T2_ARC_RPM_OPEN       0
/* Along obstacle 2 (edge_leg): the stop past an edge is a Motion_Stop()
 * brake from this speed, so slower here means less overshoot. MEASURE how far
 * it rolls. */
#define T2_EDGE_RPM           100
#define T2_EDGE_MAX_CM        200.0f /* no edge within this: the course stops   */
/* The leg BEHIND obstacle 2 drives on this far past the IR losing its far
 * end before it stops. The original car overshot ~13 cm there on its braking
 * alone, and its way home relied on it to clear obstacle 2's corner. Here the
 * turn home is placed against that corner and the wall past it (go_home), so
 * any overshoot only eats the room to the wall: 0. */
#ifndef T2_EDGE_BACK_EXTRA_CM
#define T2_EDGE_BACK_EXTRA_CM 0.0f
#endif

/* ---- sensors ------------------------------------------------------------ */
/* HC-SR04: pings every 60 ms, reports the median of the last three
 * (ultrasonic.h), and reads ~1.3 cm long (PROTOCOL.md 4.1). */
#define ULTRA_PERIOD_MS       60U
#define ULTRA_OFFSET_CM       (-1.3f)
/* Waiting for a reading that cannot include the last move: the median holds
 * three pings, so three periods plus one. */
#define T2_ULTRA_SETTLE_MS    (4U * ULTRA_PERIOD_MS)

/* Side IRs, cm from the lens, PER SIDE - task2mn_park.py's measured ranges.
 * A reading is used only inside MIN..MAX for its side (the right unit is the
 * weaker one and quits past ~50 cm; ir.h); outside it is no reading. No
 * reading with the raw count at or above T2_IR_CLOSE_RAW is closer than the
 * 10 cm floor (saturated) - "near", never clear.
 * SEEN: obstacle 2 beside that sensor, at or under this. MEASURE against the
 * black obstacle 2: it must cover T2_BACK_REACH_CM + the lens inset. */
#define T2_IR_MIN_CM          10.0f  /* ir.h IR_MIN_VALID_CM                    */
#define T2_IR_MAX_L_CM        65.0f  /* task2mn_park IR_MAX_GAP_CM["left"]      */
#define T2_IR_MAX_R_CM        50.0f  /* ... ["right"]                           */
#ifndef T2_IR_SEEN_CM
#define T2_IR_SEEN_CM         32U    /* both sides, unless set apart below      */
#endif
#define T2_IR_SEEN_L_CM       ((float)T2_IR_SEEN_CM)
#define T2_IR_SEEN_R_CM       ((float)T2_IR_SEEN_CM)
#define T2_IR_CLOSE_RAW       2000U
/* The ADC median and the confirm count make a reading describe the robot
 * this far back along its path (task2mn_park IR_LAG_CM, at cruise). An edge
 * is placed that much behind where it was confirmed. NOT MEASURED. */
#define T2_IR_LAG_CM          2.0f
/* Left IR minus right with the robot centred square in the bay by hand
 * (task2mn_park IR_LR_BIAS_CM): the two units are fitted separately. */
#define T2_IR_LR_BIAS_CM      0.0f
#ifndef T2_IR_REACH_CM
#define T2_IR_REACH_CM        28.0f  /* farthest the IR reliably sees obstacle 2 */
#endif
#define IR_EDGE_CONFIRM       3      /* consecutive 10 ms samples for a change  */
#define IR_EDGE_START_DEG     90.0f  /* the turn away at obstacle 2             */

/* ---- the course (task brief; same for every team) ------------------------ */
#define T2_PHOTO_1_CM         30.0f  /* bumper -> obstacle 1 at the photo stop  */
#define T2_ARROW2_WAIT_MS     800U
#define T2_SWERVE_DEG         45.0f
#define T2_SWERVE_PRE_CM      0.0f
#define T2_SWERVE_CLEAR_CM    2.0f
#define T2_SWERVE_DIAG_MAX_CM 40.0f
#define T2_SWERVE_LAND_CM     6.0f
#define T2_PEEK_DEG           20.0f
#define T2_GUARD_CM           12.0f
#define T2_GAP2_MIN_CM        60.0f
#define T2_GAP2_MAX_CM        200.0f
#define T2_ULTRA_CONE_DEG     15.0f
#define T2_LATERAL_MAX_CM     15.0f
#define T2_LATERAL_ERR_CM     2.0f
#define T2_PROMPT_REPEAT_MS   1500U
#define T2_ANSWER_TIMEOUT_MS  20000U

#define T2_OBSTACLE_DEPTH_CM  10.0f
/* Side to obstacle 2's face along it: lens inset 4.15 cm + this must stay
 * over the IR's 10 cm minimum. */
#ifndef T2_FACE_GAP_CM
#define T2_FACE_GAP_CM        9.0f
#endif
#ifndef T2_BACK_REACH_CM
#define T2_BACK_REACH_CM      23.0f
#endif
#ifndef T2_OBST2_CLEAR_CM
#define T2_OBST2_CLEAR_CM     2.5f
#endif
#ifndef T2_WATCH_CHECK_DEG
#define T2_WATCH_CHECK_DEG    60.0f
#endif
#define T2_CUT2_ALWAYS        1
#define T2_SHORT_BACK_GAP_CM  12.0f
#define T2_TURN_HERE          1
#define T2_TURN_FAR_MARGIN_CM 3.0f
#ifndef T2_SHORT_RUN_MAX_CM
/* The straight before turning back round a short obstacle 2's end
 * (round_short). The original ran 0 - its car's H-shaped outline and wider
 * right lock cleared the end anyway - but this car's rectangle hit the end on
 * an immediate turn back in 30 of 1728 sim courses; up to 30 cm of straight
 * first cleared all but 3 of those. */
#define T2_SHORT_RUN_MAX_CM   30.0f
#endif
#define T2_FACE2_ERR_CM       8.0f
#define T2_OBST2_MIN_HALF_CM  15.0f
#define T2_OBST2_OFFSET_CM    8.0f
#define T2_MAX_PLAN_TURN_DEG  200.0f

#define T2_RANGE_WAIT_MS      500U
#define T2_RANGE_AGREE_CM     2
#define T2_RECHECK_CM         25.0f
#define T2_RANGE_NEAR_CM      4U
#define T2_RECHECK_TOL_CM     1.5f
#define T2_NUDGE_MIN_CM       4.0f
#define T2_RECHECK_MAX_CM     12.0f
#define T2_START_MIN_GAP_CM   40.0f

#define T2_PARK_ENTRY_CM      65.0f
#define T2_PARK_ENTRY_MIN_CM  35.0f
#define T2_SQUARE_MIN_DEG     5.0f
#define T2_HOME_SHORTEN_CM    15.0f
#define T2_HOME_CLEAR_CM      6.0f
#define T2_WAYPOINT_TOL_CM    8.0f
#define T2_WAYPOINT_TRIES     1

/* ---- obstacle 2's measured end (Gordon's face scan) ----------------------
 * The leg along obstacle 2's face ends on the side IR losing it: that is its
 * near end, and obstacle 2 is centred on obstacle 1's line, so the far end
 * is the same distance the other way - its width. With the width known the
 * leg behind it stops at the far end even if the IR never sees the back face,
 * and the way home keeps off obstacle 2 as well as obstacle 1. The arena
 * walls stand at least WALL_PAST past either end (task2mn_gordon). */
#define T2_END_MARGIN_CM      3.0f   /* task2mn_gordon END_MARGIN_CM           */
/* The widest obstacle 2 is taken to be when the IR cannot say: half of
 * task2mn's 90 cm width cap. */
#define T2_OBST2_MAX_HALF_CM  45.0f
/* An end the side IR reads further out than this (plus the offset) is not
 * obstacle 2 but whatever stands past it - a wall, a box. The rules give no
 * maximum width (the course sim runs to 140 cm); 2 x 80 cm is beyond any. */
#define T2_OBST2_PLAUSIBLE_HALF_CM 80.0f
/* 1: the 90 away from obstacle 2 stops early when the side IR reads clear
 * past T2_WATCH_CHECK_DEG (the original's short-obstacle cut). 0: the 90 is
 * always finished and the face scan decides. On the floor (2026-09-14
 * 13:22) the right IR read the black face as clear 21 degrees off square,
 * from ~14 cm: the cut took obstacle 2 for short and turned back through it.
 * Sharp IRs read black badly at an angle; square on, they see it. */
#ifndef T2_WATCH_CUT
#define T2_WATCH_CUT          0
#endif
/* After the 90, a side IR already clear may be clear by a centimetre: the
 * end right at the lens, the axle IR_AHEAD short of it. The 180 round the
 * end then starts too early and clips its corner (course sim, 100 cm
 * obstacle 2). A leg that found the end overruns it before the 180; one that
 * started past it drives this far on first, to the same place. */
#ifndef T2_PAST_RUN_CM
#define T2_PAST_RUN_CM        8.0f
#endif
/* The 180 round obstacle 2's end starts with the axle this far short of the
 * end - the lens IR_AHEAD ahead of it, less the ~8 cm a leg that reads the
 * end on the move rolls on before it stops. */
#define T2_TURN_AXLE_SHORT_CM (T2_IR_AHEAD_CM - T2_PAST_RUN_CM)

/* ---- the arena walls -------------------------------------------------------
 * The walls can stand as near as WALL_PAST past either end of obstacle 2
 * (task2mn_gordon). Every turn behind obstacle 2 - the 180 round its end, the
 * turn home - and every route home keeps WALL_CLEAR off the nearest each can
 * be (recompute_end2 in task2_mission.c), moving the turn up to
 * WALL_SHIFT_MAX along the leg to fit. */
#ifndef T2_WALL_CLEAR_CM
#define T2_WALL_CLEAR_CM      5.0f
#endif
#define T2_WALL_SHIFT_MAX_CM  20.0f
#define T2_HOME_TURN_SHIFT_CM 30.0f  /* the turn home: this far either way      */
#define T2_WALL_PAST_CM       50.0f

/* ---- the carpark, as task2mn_park.py has it ------------------------------
 * The way home lines up on the carpark's line (bay_y, the side IRs at the
 * start) facing in, within ENTRY_ALIGN on the odometry - re-planning deeper,
 * or moving along the lane first for room, until it is (line_up in task2_mission.c). The park
 * is then one straight in on the ultrasonic until it reads PARK_US, a look
 * and a nudge more if it is short by over PARK_TOL. No centring: lined up
 * that close the car has ~15 cm each side. A side IR under SIDE_STOP stops
 * the drive in; then, and only then, one lane change off that fence, backing
 * out up to CENTRE_BACK_MAX for the room. */
#define T2_BAY_DEPTH_CM       50.0f  /* T2_BAY_DEPTH_CM                         */
#define T2_BAY_HALF_CM        30.0f  /* T2_BAY_WIDTH_CM / 2                     */
#define T2_CENTRE_SUM_TOL_CM  10.0f  /* T2_CENTRE_SUM_TOL_CM                    */
#define T2_NEAR_FENCE_GAP_CM  8.0f   /* task2mn_park NEAR_FENCE_GAP_CM          */
#define T2_PARK_SIDE_STOP_CM  10.0f  /* T2_PARK_SIDE_STOP_CM                    */
#define T2_CENTRE_BACK_MAX_CM 40.0f
#define T2_PARK_US_CM         10.0f  /* the ultrasonic's reading, parked        */
#define T2_PARK_TOL_CM        3.0f   /* short of it by more: one more nudge     */
#define T2_PARK_LOOKS         3      /* drive in, then up to two nudges         */
/* The back wall is no deeper than BAY_DEPTH in from the mouth (or from the
 * start nose, the car having started inside), less this for the odometry. A
 * reading past that is not the back wall: on the floor (2026-09-14 14:39) one
 * of 142 cm from 50 cm off the line drove the car on past the carpark. */
#define T2_PARK_BACK_SLACK_CM 10.0f
/* The entry: on the carpark's line by this or less, on the odometry. */
#define T2_ENTRY_ALIGN_CM     5.0f
#define T2_LINE_UP_TRIES      6
#define T2_LINE_UP_LEG_CM     60.0f  /* a longer straight is re-planned near its end */
#define T2_LINE_UP_SHIFT_CM   40.0f  /* no S fits: along the lane, either way  */
/* The mouth, measured on the way OUT (mouth_watch): the side IRs read the
 * carpark's side fences - each under FENCE_SEEN, ~25 cm when centred - and
 * the point where both go clear is the mouth. The way home then straightens
 * up with the nose ENTRY_CLEAR outside it, and keeps FENCE_CLEAR off the
 * fences' ends: the robot started at the back of the carpark on the floor,
 * where the fixed T2_PARK_ENTRY_CM left its nose past the mouth mid-S (the
 * course sim clipped the fence's end). */
#define T2_FENCE_SEEN_CM      40.0f
#define T2_ENTRY_CLEAR_CM     6.0f
#define T2_FENCE_CLEAR_CM     3.0f

/* ---- the run ------------------------------------------------------------ */
#define T2_MAX_SECONDS        180    /* !t2go<s> ceiling                        */

#endif /* __TASK2_PARAMS_H */
