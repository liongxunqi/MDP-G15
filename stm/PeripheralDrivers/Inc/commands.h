/**
  ******************************************************************************
  * @file    commands.h
  * @brief   RPi <-> STM32 command protocol.  FROZEN INTERFACE.
  *
  *          MDP Group 15 - WHEELTEC C30D V2.1 (STM32F407VET6)
  *
  *          This header is the ONLY coupling point between the motion/firmware
  *          workstream (Person A) and the sensors/comms workstream (Person B).
  *          Wire format is inherited verbatim from the previous year's proven
  *          implementation so that the RPi-side driver needs no changes.
  *
  *          DO NOT change token spellings, argument units, or reply strings
  *          without agreement from both owners AND the RPi team.
  ******************************************************************************
  * OWNERSHIP
  *
  *   Person A (Mahan)  : USART3 driver, line parser, command queue,
  *                       motion primitives, OK/RESEND emission.
  *   Person B          : sensor drivers behind the Sensors_* hooks at the
  *                       bottom of this file.  Nothing else in here.
  ******************************************************************************
  */

#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * 1. WIRE FORMAT
 * ===========================================================================
 *
 *   RPi -> STM :   <tok1>,<tok2>,...,<tokN>\n
 *   STM -> RPi :   OK\n          after the ENTIRE line has executed
 *                  RESEND\n      on parse failure, nothing executed
 *
 *   - Tokens separated by comma OR space. Both accepted.
 *   - Line terminated by '\n'. A '\r' before it is tolerated and discarded.
 *   - Input is lowercased on receive, so sender casing is irrelevant.
 *   - ONE reply per LINE, never per token.
 *   - RST is the sole exception: it aborts immediately and replies nothing.
 *
 *   Example:  "FR90,F20,S\n"  ->  arc right 90 deg, forward 20 cm, stop, "OK\n"
 */

#define CMD_BAUD_RATE           115200u   /* USART3, 8N1, no flow control     */
#define CMD_LINE_MAX            128u      /* max bytes in one line inc. '\n'  */
#define CMD_QUEUE_DEPTH         16u       /* max tokens buffered per line     */
#define CMD_TERMINATOR          '\n'
#define CMD_DELIM_PRIMARY       ','
#define CMD_DELIM_SECONDARY     ' '

#define CMD_REPLY_OK            "OK\n"
#define CMD_REPLY_RESEND        "RESEND\n"

/* Per-primitive watchdog. If a primitive has not completed within this many
 * milliseconds the motion layer aborts it, brakes, and the line still replies
 * so the RPi is never left waiting forever on a stalled wheel.
 *
 * *** NOT CURRENTLY HONOURED - THE FIRMWARE USES 15 s. ***
 *
 * motion.c enforces MOTION_TIMEOUT_TICKS, which is 1500 ticks at 10 ms. This
 * constant is left here rather than deleted because the gap is a protocol
 * question, not dead code: if the RPi gives up at 8 s while the STM is still
 * working through a 15 s watchdog, the two desynchronise and the next reply
 * lands against the wrong command. Reconcile the two numbers with the RPi
 * owner before the first wire test, and make whichever survives the one both
 * sides read. */
#define CMD_PRIMITIVE_TIMEOUT_MS   8000u


/* ===========================================================================
 * 2. OPCODES
 * ===========================================================================
 *
 *  Token    Arg unit   Meaning
 *  -------  ---------  ------------------------------------------------------
 *  F{n}     cm         Forward n cm.
 *  F0       -          Forward indefinitely until an obstacle stops it.
 *                      Requires Sensors_ObstacleAhead(). Person B dependency.
 *  FU{n}    cm         Forward until the front ultrasound reads n cm, where
 *                      n is CHOSEN BY THE SENDER. Unlike F0 this is a
 *                      measured approach, not a trip-wire: the firmware reads
 *                      the gap while stationary, drives it on odometry, then
 *                      re-measures and corrects. Arrives within CMD_FU_TOL_CM
 *                      of n. Range CMD_FU_MIN_CM..CMD_FU_MAX_CM; outside that
 *                      is a parse failure.
 *  R{n}     cm         Reverse n cm.
 *  FR{n}    degrees    Arc forward-right through n degrees.
 *  FL{n}    degrees    Arc forward-left  through n degrees.
 *  RR{n}    degrees    Arc reverse-right through n degrees.
 *  RL{n}    degrees    Arc reverse-left  through n degrees.
 *  S        -          Stop: brake motors, recentre servo. Replies OK.
 *  RST      -          Emergency abort: drop queue, brake NOW. Replies nothing.
 *
 *  NOTE ON ARCS: this chassis is Ackermann-steered and CANNOT turn on the
 *  spot. Every turn is an arc with forward or reverse travel. The RPi path
 *  planner must account for the swept area.
 */
typedef enum
{
    CMD_NONE = 0,       /* empty slot / parser sentinel            */
    CMD_FORWARD,        /* F{n}   arg = cm      (arg 0 => until obstacle) */
    CMD_REVERSE,        /* R{n}   arg = cm                         */
    CMD_ARC_FWD_RIGHT,  /* FR{n}  arg = degrees                    */
    CMD_ARC_FWD_LEFT,   /* FL{n}  arg = degrees                    */
    CMD_ARC_REV_RIGHT,  /* RR{n}  arg = degrees                    */
    CMD_ARC_REV_LEFT,   /* RL{n}  arg = degrees                    */
    CMD_STOP,           /* S      arg unused                       */
    CMD_RESET,          /* RST    arg unused                       */
    CMD_INVALID,        /* unrecognised token -> whole line RESEND */

    /* --- IMMEDIATE opcodes, appended after CMD_INVALID on purpose ---------
     *
     * The block above is frozen and must never be renumbered - the Task 2
     * tokens still have to slot into it later without shifting anything. New
     * opcodes therefore go here, past the sentinel. Nothing compares against
     * the numeric values and the wire format is strings, so the ordering is
     * internal detail.
     *
     * These are NOT queued. They are answered the moment the line is parsed,
     * even mid-move, and they never touch the command queue or the in-flight
     * line. See Cmd_IsImmediate(). */
    CMD_Q_US,           /* ?US    front distance                   */
    CMD_Q_IR,           /* ?IR    both IR, centimetres             */
    CMD_Q_IRR,          /* ?IRR   both IR, raw filtered counts     */
    CMD_Q_POSE,         /* ?POSE  x, y, heading                    */
    CMD_Q_DIST,         /* ?DIST  distance this/last move          */
    CMD_Q_TURN,         /* ?TURN  degrees this/last arc            */
    CMD_Q_STAT,         /* ?STAT  motion state, busy, imu, profile */
    CMD_Q_IMU,          /* ?IMU   gyro health                      */
    CMD_Q_XCHK,         /* ?XCHK  last arc cross-check             */
    CMD_Q_VER,          /* ?VER   identity and protocol version    */
    CMD_SET_PROFILE,    /* !PROFn arg = 0..2                       */
    CMD_SET_ZERO,       /* !ZERO  zero odometry and heading        */

    /* --- Calibration, protocol 2 -----------------------------------------
     *
     * Everything the firmware learns at run time - arc deceleration, brake
     * engagement lag, steering trim - is held in RAM and lost at power-off,
     * so a cold robot spends its first three or four arcs converging. ?CAL
     * lets the sender watch that happen; the setters let it skip the wait by
     * restoring values it saved from a previous session.
     *
     * The SEQUENCE is not here on purpose. Calibration is a script on the
     * RPi built from ordinary primitives, not a mode the firmware can enter
     * by itself - a self-driving firmware mode is exactly the thing that
     * could interfere with a task. These four commands only read and write
     * numbers.
     *
     * Three setters rather than one, because Command_t carries a single
     * argument and widening it would touch the queue. */
    CMD_Q_CAL,          /* ?CAL   learned decel, lag and trim      */
    CMD_SET_CAL_DECEL,  /* !CALDn arc decel, dps^2 x10             */
    CMD_SET_CAL_LAG,    /* !CALLn brake lag, milliseconds x10      */
    CMD_SET_CAL_TRIM,   /* !CALTn steering trim, us, MAY BE NEGATIVE */

    /* --- Sensor-terminated movement, protocol 3 --------------------------
     *
     * MUST stay OUTSIDE the CMD_Q_US..CMD_SET_CAL_TRIM range Cmd_IsImmediate()
     * tests, because this one IS queued and DOES move the robot. Appended
     * after CMD_SET_CAL_TRIM for exactly that reason - do not "tidy" it into
     * the block above. */
    CMD_FWD_UNTIL_US    /* FU{n}  arg = cm, the gap to stop at     */
} CmdOpcode_t;

/* Bumped whenever the wire format changes in a way a sender must care about.
 * Reported by ?VER so the RPi can assert compatibility at startup instead of
 * discovering a mismatch halfway through a run. */
/* 2: added ?CAL, !CALD, !CALL, !CALT. Purely additive - every protocol 1
 * sender keeps working unchanged.
 * 3: added FU{n} and the FAIL,NOECHO reply. Additive for movement, but a
 * sender that treats an unrecognised reply as fatal must learn NOECHO before
 * it sends its first FU. */
#define CMD_PROTOCOL_VERSION    3
#define CMD_FIRMWARE_NAME       "MDPG15-STM32"

/* Replies for a primitive that did not complete. Previously a timed-out move
 * replied OK, which made a stalled wheel indistinguishable from success - the
 * sender carried on believing the robot had moved. */
#define CMD_REPLY_FAIL_TIMEOUT   "FAIL,TIMEOUT\n"
#define CMD_REPLY_FAIL_WRONGWAY  "FAIL,WRONGWAY\n"

/* FU had nothing to aim at - the ultrasound gave no usable reading when the
 * command started. The robot did NOT move, and that is the point of a
 * separate reply: NOECHO means "nothing happened, the sensor is the problem",
 * where TIMEOUT means "it tried and got stuck". FU never guesses by creeping
 * forward blind, because driving at an obstacle it cannot see is the one
 * thing a distance-keeping command must not do. */
#define CMD_REPLY_FAIL_NOECHO    "FAIL,NOECHO\n"

/* Bounds on the FU stop distance, cm. Rejected rather than clamped, in line
 * with every other argument here - a distance the sender did not mean is
 * worse than a RESEND.
 *
 * The floor is not the sensor's 2 cm limit. It is where the approach still
 * has room to correct itself: below about 5 cm the robot's own brake coast is
 * a large fraction of the target, and the HC-SR04 cannot separate the
 * outgoing burst from the echo anyway.
 *
 * The ceiling is about trusting the reading, not reaching it. Past roughly
 * 2 m the beam has spread wide enough that the nearest thing it hears is
 * often not the thing the planner meant, so a long FU would confidently drive
 * to the wrong gap. Send F{n} for the bulk of the travel, FU for the last
 * stretch. */
#define CMD_FU_MIN_CM            5
#define CMD_FU_MAX_CM            200

/* How close to n counts as arrived, cm. One cm is below what the sensor
 * resolves repeatably, so a tolerance of 1 would have the robot chase its own
 * quantisation noise until it ran out of correction passes. */
#define CMD_FU_TOL_CM            2

/* 1 for opcodes answered immediately rather than queued. Such a token is only
 * valid ALONE on a line - mixed into a movement line it is a parse failure,
 * because one line may only ever produce one reply. */
uint8_t Cmd_IsImmediate(CmdOpcode_t op);

typedef struct
{
    CmdOpcode_t op;
    int16_t     arg;    /* cm or degrees, positive - direction lives in the
                         * opcode. The ONE exception is CMD_SET_CAL_TRIM,
                         * which is a signed servo offset and has no opcode
                         * to carry its sign. */
} Command_t;


/* ===========================================================================
 * 3. RESERVED - TASK 2 ONLY. Not implemented for the checklist.
 * ===========================================================================
 * Listed so the enum above is never renumbered when these are added later.
 * Parser must return CMD_INVALID for these until Task 2 begins.
 *
 *   FIR     Forward until right IR sees wall disappear
 *   FIL     Forward until left  IR sees wall disappear
 *   FIRO    Forward until right IR sees wall appear
 *   FILO    Forward until left  IR sees wall appear
 *   SR / SL Diagonal slide right / left
 *
 * Task 2 replies, also reserved:
 *   ir{d}\n          IR distance, cm
 *
 * The reserved "us{d1},{d2}" reply is NOT used by FU{n}, which answers with
 * an ordinary OK. One reply per LINE is the rule the whole protocol rests on,
 * and FU can sit mid-line behind other primitives, so a bespoke data reply
 * would either arrive out of order or turn one line into two replies. Both
 * numbers it carried are queryable anyway - ?US gives the gap actually
 * achieved, which is the number that says whether FU worked.
 */


/* ===========================================================================
 * 4. PARSER + QUEUE API   -- implemented by Person A in commands.c
 * ===========================================================================
 */

/** Reset parser and queue to empty. Call on init and on RST. */
void Cmd_Init(void);

/**
 * Parse one complete received line into the queue.
 * The line must be NUL-terminated with the '\n' already stripped.
 * Parsing is all-or-nothing: on any invalid token the queue is left
 * untouched and the caller must reply RESEND.
 *
 * @return 1 if the whole line parsed and was queued, 0 on failure.
 */
uint8_t Cmd_ParseLine(const char *line);

/** Parse a single token. Returns op = CMD_INVALID if unrecognised. */
Command_t Cmd_ParseToken(const char *token);

/** Pop the next queued primitive. Returns op = CMD_NONE when empty. */
Command_t Cmd_QueuePop(void);

/** Number of primitives still queued. */
uint8_t Cmd_QueueCount(void);

/** Drop everything queued. Used by RST. */
void Cmd_QueueFlush(void);


/* ===========================================================================
 * 5. SENSOR HOOKS   -- implemented by Person B
 * ===========================================================================
 * These are the ONLY symbols Person B must provide to the motion layer.
 * Person A ships weak default stubs so the firmware links and runs the
 * checklist before any sensor exists. Person B's strong definitions override
 * them automatically at link time - no #ifdef, no coordination needed.
 *
 * Keep these non-blocking. They are polled from the 10 ms TIM6 control tick
 * and MUST return promptly. Do not busy-wait for an ultrasound echo inside
 * them; capture in your own interrupt and return the last cached value.
 */

/** Most recent front distance in cm. Return SENSOR_NO_READING if unknown. */
#define SENSOR_NO_READING   0xFFFFu
uint16_t Sensors_FrontDistanceCm(void);

/** 1 if an obstacle is within stop_cm ahead, else 0. Backs the F0 command. */
uint8_t  Sensors_ObstacleAhead(uint16_t stop_cm);

/** IMU yaw in degrees, positive counter-clockwise, wrapped to (-180, 180]. */
float    Sensors_HeadingDeg(void);

/** 1 once the IMU has been initialised and its heading is trustworthy. */
uint8_t  Sensors_HeadingValid(void);


#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H */
