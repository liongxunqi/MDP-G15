#ifndef __CALTAB_H
#define __CALTAB_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * CALIBRATION TABLE, protocol 10
 *
 * Up to protocol 9 the calibration was a handful of global numbers: one
 * braking model for every arc, one steering trim for both directions, one
 * deflection per side shared by forward and reverse, one brake coast compiled
 * in. This robot is not that symmetric. One motor pulls less than the other,
 * one turn swings wider, forward and reverse arcs on the same pulse trace
 * different circles, and all of it moves with speed.
 *
 * So each MOVE TYPE (F, R, FR, FL, RR, RL) carries its own values, measured at
 * up to CT_ANCHORS speeds. A move looks its values up at its own cruise speed
 * and the table interpolates between the anchors either side of it (and holds
 * the end value outside them - it never extrapolates). A field nobody set
 * falls back to what protocol 9 used, so an empty table IS protocol 9.
 *
 *   straights (F, R)        rpm  trim  coast  dist
 *   arcs (FR, FL, RR, RL)   rpm  steer decel  lag  radius  boost
 *
 *   rpm     the speed this anchor was measured at, axle centre. Setting it
 *           to 0 clears the whole anchor. Every other field needs it set.
 *   trim    steering centre offset for this direction, us, signed
 *   coast   distance the robot rolls after the brake, mm x10
 *   dist    true / odometry distance, x10000 (10000 = odometry is right)
 *   steer   arc deflection from centre, us (magnitude; the side is the move's)
 *   decel   arc braking deceleration, deg/s^2 x10
 *   lag     arc brake engagement lag, ms x10
 *   radius  radius used to split the rear wheel speeds, mm
 *   boost   differential assist, x100 (100 = pure geometry). IGNORED while
 *           MOTION_ARC_GEOMETRIC_SPLIT is 1 (motion.h): every arc then runs
 *           at 1.0, and only radius sets the wheel split. Radii measured
 *           with a boost above 1.0 do not carry over - re-measure them.
 *
 * Global values describe the hardware rather than a move:
 *
 *   wheel   right/left ground travel per encoder count, x10000. Corrects the
 *           odometry AND the speed split, so both rear wheels cover the same
 *           ground on a straight instead of the steering fighting the
 *           difference.
 *   ffaf ffar ffbf ffbr   per wheel (A left, B right) and direction (f, r):
 *           the rpm that wheel would reach at full duty - the feedforward
 *           slope. 0 = the compiled MOTOR_x_MAX_RPM.
 *   dba dbb per wheel deadband duty: what it takes to start turning. 0 = the
 *           compiled MOTOR_DEADBAND. A dragging wheel needs more.
 *   vs va   default cruise for straights / arcs that carry no V suffix, rpm.
 *           0 = protocol 9 speeds.
 *
 * Everything here is RAM: power-off clears it, and the RPi restores the saved
 * table with the rest of the calibration before every task. Nothing on the
 * robot writes it - learning stays off, the table is only what was sent.
 * ------------------------------------------------------------------------- */

#define CT_ANCHORS      3U

typedef enum
{
    CT_F = 0,
    CT_R,
    CT_FR,
    CT_FL,
    CT_RR,
    CT_RL,
    CT_MOVES,
    CT_GLOBAL = CT_MOVES      /* the "g" row of the key */
} CtMove_t;

typedef enum
{
    CTF_RPM = 0,
    CTF_TRIM,
    CTF_COAST,
    CTF_DIST,
    CTF_STEER,
    CTF_DECEL,
    CTF_LAG,
    CTF_RADIUS,
    CTF_BOOST,
    CTF_COUNT
} CtField_t;

typedef enum
{
    CTG_WHEEL = 0,
    CTG_FFAF,
    CTG_FFAR,
    CTG_FFBF,
    CTG_FFBR,
    CTG_DBA,
    CTG_DBB,
    CTG_VS,
    CTG_VA,
    CTG_COUNT
} CtGlobal_t;

/* Clear everything: back to protocol 9 behaviour. */
void    CalTab_Clear(void);

/* Set one value. move CT_GLOBAL takes a CtGlobal_t as field and ignores
 * anchor. Returns 0 and changes nothing if the field does not belong to that
 * move, the value is out of range, or the anchor has no rpm yet. Refused,
 * never clamped, like every other setter on this robot. */
uint8_t CalTab_Set(uint8_t move, uint8_t anchor, uint8_t field, int16_t value);

/* Read one stored value. Returns 1 if it is set. */
uint8_t CalTab_Get(uint8_t move, uint8_t anchor, uint8_t field, int16_t *value);

/* The value for a move at a speed: interpolated between the anchors that
 * have this field, held flat outside them. Returns 0 (and leaves *out alone)
 * when no anchor has it - the caller uses its protocol 9 value. */
uint8_t CalTab_Lookup(CtMove_t move, CtField_t field, float rpm, float *out);

/* A global, 0 when unset. */
int16_t CalTab_Global(CtGlobal_t g);

/* 1 if this field belongs to this move. */
uint8_t CalTab_FieldAllowed(uint8_t move, uint8_t field);

/* Names, as the wire spells them ("fr", "steer", "ffaf"). */
const char *CalTab_MoveName(uint8_t move);
const char *CalTab_FieldName(uint8_t move, uint8_t field);

/* Parse "<move><anchor>.<field>" (or "g.<field>") from the start of s, as it
 * follows "!k". Returns the number of characters used, 0 if it is not a key.
 * Case: lower only - the link lowercases every line before parsing. */
uint8_t CalTab_ParseKey(const char *s, uint8_t *move, uint8_t *anchor,
                        uint8_t *field);

#endif /* __CALTAB_H */
