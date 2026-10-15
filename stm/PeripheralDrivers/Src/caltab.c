#include "caltab.h"
#include "motors.h"   /* SERVO_* - the steer bounds per side */

/* See CALIBRATION TABLE in caltab.h. Pure data: no hardware, no HAL calls,
 * so it runs in the host test as it does on the robot. */

typedef struct
{
    int16_t  val[CTF_COUNT];
    uint16_t set;             /* bit per field */
} CtAnchor_t;

static CtAnchor_t s_tab[CT_MOVES][CT_ANCHORS];
static int16_t    s_glob[CTG_COUNT];

static const char *const s_moveNames[CT_MOVES + 1U] =
    { "f", "r", "fr", "fl", "rr", "rl", "g" };

static const char *const s_fieldNames[CTF_COUNT] =
    { "rpm", "trim", "coast", "dist", "steer", "decel", "lag", "radius", "boost" };

static const char *const s_globNames[CTG_COUNT] =
    { "wheel", "ffaf", "ffar", "ffbf", "ffbr", "dba", "dbb", "vs", "va" };

static uint8_t is_arc(uint8_t move) { return (move >= CT_FR) && (move <= CT_RL); }
static uint8_t is_right(uint8_t move) { return (move == CT_FR) || (move == CT_RR); }

void CalTab_Clear(void)
{
    uint8_t m, a, f;

    for (m = 0U; m < CT_MOVES; m++)
    {
        for (a = 0U; a < CT_ANCHORS; a++)
        {
            s_tab[m][a].set = 0U;
            for (f = 0U; f < CTF_COUNT; f++) { s_tab[m][a].val[f] = 0; }
        }
    }
    for (f = 0U; f < CTG_COUNT; f++) { s_glob[f] = 0; }
}

uint8_t CalTab_FieldAllowed(uint8_t move, uint8_t field)
{
    if (move == CT_GLOBAL) { return (field < CTG_COUNT) ? 1U : 0U; }
    if (move >= CT_MOVES)  { return 0U; }
    if (field == CTF_RPM)  { return 1U; }

    if (is_arc(move))
    {
        return ((field == CTF_STEER) || (field == CTF_DECEL) || (field == CTF_LAG) ||
                (field == CTF_RADIUS) || (field == CTF_BOOST)) ? 1U : 0U;
    }
    return ((field == CTF_TRIM) || (field == CTF_COAST) || (field == CTF_DIST)) ? 1U : 0U;
}

/* Bounds. Wide enough for anything this chassis could need, narrow enough
 * that a typo or a units slip (seconds for ms, mm for cm) is refused. */
static uint8_t in_range(uint8_t move, uint8_t field, int16_t v)
{
    if (move == CT_GLOBAL)
    {
        switch (field)
        {
        case CTG_WHEEL: return (v >= 9000) && (v <= 11000);
        case CTG_FFAF: case CTG_FFAR: case CTG_FFBF: case CTG_FFBR:
            return (v == 0) || ((v >= 150) && (v <= 600));
        case CTG_DBA: case CTG_DBB:
            return (v == 0) || ((v >= 300) && (v <= 900));
        case CTG_VS: return (v == 0) || ((v >= 40) && (v <= 250));
        case CTG_VA: return (v == 0) || ((v >= 60) && (v <= 170));
        default:     return 0U;
        }
    }

    switch (field)
    {
    case CTF_RPM:    return (v == 0) || ((v >= 40) && (v <= 250));
    case CTF_TRIM:   return (v >= -80) && (v <= 80);
    case CTF_COAST:  return (v >= 0) && (v <= 400);
    case CTF_DIST:   return (v >= 8000) && (v <= 12000);
    case CTF_STEER:
    {
        /* Never past what the servo can reach on that side. */
        int16_t room = is_right(move) ? (int16_t)(SERVO_MAX_US - SERVO_CENTER_US)
                                      : (int16_t)(SERVO_CENTER_US - SERVO_MIN_US);
        return (v >= 300) && (v <= room);
    }
    case CTF_DECEL:  return (v > 500) && (v < 30000);
    case CTF_LAG:    return (v >= 0) && (v <= 2500);
    case CTF_RADIUS: return (v >= 150) && (v <= 1000);
    case CTF_BOOST:  return (v >= 100) && (v <= 250);
    default:         return 0U;
    }
}

uint8_t CalTab_Set(uint8_t move, uint8_t anchor, uint8_t field, int16_t value)
{
    CtAnchor_t *an;
    uint8_t     a;

    if (!CalTab_FieldAllowed(move, field) || !in_range(move, field, value))
    {
        return 0U;
    }

    if (move == CT_GLOBAL)
    {
        s_glob[field] = value;
        return 1U;
    }

    if (anchor >= CT_ANCHORS) { return 0U; }
    an = &s_tab[move][anchor];

    if (field == CTF_RPM)
    {
        if (value == 0)
        {
            an->set = 0U;     /* clears the anchor and everything on it */
            return 1U;
        }
        /* Two anchors at one speed would make the interpolation divide by
         * zero, and could only ever disagree with each other. */
        for (a = 0U; a < CT_ANCHORS; a++)
        {
            if ((a != anchor) && (s_tab[move][a].set & (1U << CTF_RPM)) &&
                (s_tab[move][a].val[CTF_RPM] == value))
            {
                return 0U;
            }
        }
    }
    else if (!(an->set & (1U << CTF_RPM)))
    {
        /* A value with no speed attached cannot be placed anywhere. */
        return 0U;
    }

    an->val[field] = value;
    an->set |= (uint16_t)(1U << field);
    return 1U;
}

uint8_t CalTab_Get(uint8_t move, uint8_t anchor, uint8_t field, int16_t *value)
{
    if (move == CT_GLOBAL)
    {
        if (field >= CTG_COUNT) { return 0U; }
        *value = s_glob[field];
        return (s_glob[field] != 0) ? 1U : 0U;
    }
    if ((move >= CT_MOVES) || (anchor >= CT_ANCHORS) || (field >= CTF_COUNT))
    {
        return 0U;
    }
    if (!(s_tab[move][anchor].set & (1U << field))) { return 0U; }
    *value = s_tab[move][anchor].val[field];
    return 1U;
}

uint8_t CalTab_Lookup(CtMove_t move, CtField_t field, float rpm, float *out)
{
    float   r[CT_ANCHORS];
    float   v[CT_ANCHORS];
    uint8_t n = 0U;
    uint8_t a, i, j;

    if ((move >= CT_MOVES) || (field >= CTF_COUNT) || (field == CTF_RPM)) { return 0U; }

    for (a = 0U; a < CT_ANCHORS; a++)
    {
        const CtAnchor_t *an = &s_tab[move][a];
        uint16_t need = (uint16_t)((1U << CTF_RPM) | (1U << field));

        if ((an->set & need) == need)
        {
            r[n] = (float)an->val[CTF_RPM];
            v[n] = (float)an->val[field];
            n++;
        }
    }
    if (n == 0U) { return 0U; }

    /* Sort by speed - at most three, so insertion. */
    for (i = 1U; i < n; i++)
    {
        for (j = i; (j > 0U) && (r[j - 1U] > r[j]); j--)
        {
            float t;
            t = r[j]; r[j] = r[j - 1U]; r[j - 1U] = t;
            t = v[j]; v[j] = v[j - 1U]; v[j - 1U] = t;
        }
    }

    if (rpm <= r[0])      { *out = v[0];      return 1U; }
    if (rpm >= r[n - 1U]) { *out = v[n - 1U]; return 1U; }

    for (i = 1U; i < n; i++)
    {
        if (rpm <= r[i])
        {
            float t = (rpm - r[i - 1U]) / (r[i] - r[i - 1U]);
            *out = v[i - 1U] + (t * (v[i] - v[i - 1U]));
            return 1U;
        }
    }
    *out = v[n - 1U];
    return 1U;
}

int16_t CalTab_Global(CtGlobal_t g)
{
    return (g < CTG_COUNT) ? s_glob[g] : 0;
}

const char *CalTab_MoveName(uint8_t move)
{
    return (move <= CT_GLOBAL) ? s_moveNames[move] : "?";
}

const char *CalTab_FieldName(uint8_t move, uint8_t field)
{
    if (move == CT_GLOBAL) { return (field < CTG_COUNT) ? s_globNames[field] : "?"; }
    return (field < CTF_COUNT) ? s_fieldNames[field] : "?";
}

/* Whole-word compare: s must hold exactly `lit` up to `len` characters. */
static uint8_t word_is(const char *s, uint8_t len, const char *lit)
{
    uint8_t i;

    for (i = 0U; i < len; i++)
    {
        if (lit[i] != s[i]) { return 0U; }
    }
    return (lit[len] == '\0') ? 1U : 0U;
}

uint8_t CalTab_ParseKey(const char *s, uint8_t *move, uint8_t *anchor,
                        uint8_t *field)
{
    uint8_t i = 0U;
    uint8_t len;
    uint8_t m;
    uint8_t f;

    /* Move: letters up to the anchor digit (or '.' for the global row). */
    while ((s[i] >= 'a') && (s[i] <= 'z')) { i++; }
    len = i;
    for (m = 0U; m <= CT_GLOBAL; m++)
    {
        if ((len > 0U) && word_is(s, len, s_moveNames[m])) { break; }
    }
    if (m > CT_GLOBAL) { return 0U; }

    *anchor = 0U;
    if (m != CT_GLOBAL)
    {
        if ((s[i] < '0') || (s[i] > '9')) { return 0U; }
        *anchor = (uint8_t)(s[i] - '0');
        if (*anchor >= CT_ANCHORS) { return 0U; }
        i++;
    }

    if (s[i] != '.') { return 0U; }
    i++;

    {
        const char *fs = &s[i];
        uint8_t fl = 0U;
        uint8_t nf = (m == CT_GLOBAL) ? (uint8_t)CTG_COUNT : (uint8_t)CTF_COUNT;

        while ((fs[fl] >= 'a') && (fs[fl] <= 'z')) { fl++; }
        for (f = 0U; f < nf; f++)
        {
            if ((fl > 0U) && word_is(fs, fl, CalTab_FieldName(m, f))) { break; }
        }
        if (f >= nf) { return 0U; }
        i = (uint8_t)(i + fl);
    }

    if (!CalTab_FieldAllowed(m, f)) { return 0U; }

    *move  = m;
    *field = f;
    return i;
}
