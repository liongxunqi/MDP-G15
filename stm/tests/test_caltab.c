/* Host regression test: protocol 10 parser (V suffix, !K / ?K / ?MV) and the
 * calibration table's interpolation. No hardware. */
#include <assert.h>
#include <math.h>
#include <string.h>
#include "commands.h"
#include "caltab.h"

static float look(CtMove_t m, CtField_t f, float rpm)
{
    float v = -12345.0f;
    assert(CalTab_Lookup(m, f, rpm, &v));
    return v;
}

int main(void)
{
    Command_t c;
    float v;
    int16_t g;

    /* --- V suffix ---------------------------------------------------- */
    c = Cmd_ParseToken("fr90v12");
    assert(c.op == CMD_ARC_FWD_RIGHT && c.arg == 90 && c.speed == 12);
    c = Cmd_ParseToken("f100");
    assert(c.op == CMD_FORWARD && c.arg == 100 && c.speed == 0);
    c = Cmd_ParseToken("r50v25");
    assert(c.op == CMD_REVERSE && c.arg == 50 && c.speed == 25);
    assert(Cmd_ParseToken("f100v26").op == CMD_INVALID);   /* over straight max */
    assert(Cmd_ParseToken("fr90v18").op == CMD_INVALID);   /* over arc max      */
    assert(Cmd_ParseToken("fl90v5").op == CMD_INVALID);    /* under arc min     */
    assert(Cmd_ParseToken("fu20v10").op == CMD_INVALID);   /* FU takes no V     */
    assert(Cmd_ParseToken("f0v10").op == CMD_INVALID);     /* nor F0            */
    assert(Cmd_ParseToken("f100v").op == CMD_INVALID);
    assert(Cmd_ParseToken("f100v010").op == CMD_INVALID);  /* leading zero      */
    assert(Cmd_ParseToken("f100x").op == CMD_INVALID);
    assert(Cmd_ParseToken("f00").op == CMD_INVALID);
    assert(Cmd_ParseToken("f0").op == CMD_FORWARD);
    Cmd_Init();
    assert(Cmd_ParseLine("f100v20,fr90v15,rl90"));
    assert(Cmd_QueueCount() == 3);
    c = Cmd_QueuePop();
    assert(c.op == CMD_FORWARD && c.speed == 20);
    c = Cmd_QueuePop();
    assert(c.op == CMD_ARC_FWD_RIGHT && c.speed == 15);
    c = Cmd_QueuePop();
    assert(c.op == CMD_ARC_REV_LEFT && c.speed == 0);

    /* --- !K / ?K / ?MV ----------------------------------------------- */
    c = Cmd_ParseToken("!kfr1.steer=575");
    assert(c.op == CMD_SET_K && c.kmove == CT_FR && c.kanchor == 1 &&
           c.kfield == CTF_STEER && c.arg == 575);
    c = Cmd_ParseToken("!kf0.trim=-12");
    assert(c.op == CMD_SET_K && c.kmove == CT_F && c.kfield == CTF_TRIM && c.arg == -12);
    c = Cmd_ParseToken("!kg.wheel=10050");
    assert(c.op == CMD_SET_K && c.kmove == CT_GLOBAL && c.kfield == CTG_WHEEL);
    c = Cmd_ParseToken("!kr2.rpm=0");
    assert(c.op == CMD_SET_K && c.kmove == CT_R && c.kanchor == 2 && c.arg == 0);
    assert(Cmd_ParseToken("!kf0.steer=500").op == CMD_INVALID);  /* arc field on F */
    assert(Cmd_ParseToken("!kfr0.trim=5").op == CMD_INVALID);    /* straight field on FR */
    assert(Cmd_ParseToken("!kfr3.rpm=100").op == CMD_INVALID);   /* anchor 3 */
    assert(Cmd_ParseToken("!kfr.rpm=100").op == CMD_INVALID);    /* no anchor */
    assert(Cmd_ParseToken("!kg0.wheel=10000").op == CMD_INVALID);
    assert(Cmd_ParseToken("!kfx0.rpm=100").op == CMD_INVALID);
    assert(Cmd_ParseToken("!kfr0.rpm").op == CMD_INVALID);
    assert(Cmd_ParseToken("!kfr0.rpm=").op == CMD_INVALID);
    assert(Cmd_ParseToken("!kfr0.rpmx=1").op == CMD_INVALID);
    assert(Cmd_ParseToken("!kclr").op == CMD_SET_K_CLR);
    c = Cmd_ParseToken("?kfl2");
    assert(c.op == CMD_Q_K && c.kmove == CT_FL && c.kanchor == 2);
    c = Cmd_ParseToken("?kg");
    assert(c.op == CMD_Q_K && c.kmove == CT_GLOBAL);
    assert(Cmd_ParseToken("?kfl").op == CMD_INVALID);
    assert(Cmd_ParseToken("?kfl3").op == CMD_INVALID);
    assert(Cmd_ParseToken("?mv").op == CMD_Q_MV);
    assert(Cmd_IsImmediate(CMD_SET_K) && Cmd_IsImmediate(CMD_SET_K_CLR) &&
           Cmd_IsImmediate(CMD_Q_K) && Cmd_IsImmediate(CMD_Q_MV));
    assert(Cmd_IsImmediate(CMD_SET_IR_EDGE) && Cmd_IsImmediate(CMD_Q_HDG));
    assert(!Cmd_IsImmediate(CMD_FWD_UNTIL_US));
    Cmd_Init();
    assert(!Cmd_ParseLine("f10,!kclr"));
    assert(!Cmd_ParseLine("f10,?mv"));

    /* --- table -------------------------------------------------------- */
    CalTab_Clear();
    assert(!CalTab_Lookup(CT_FR, CTF_STEER, 150.0f, &v));
    assert(!CalTab_Set(CT_FR, 0, CTF_STEER, 575));     /* no rpm yet          */
    assert(CalTab_Set(CT_FR, 0, CTF_RPM, 150));
    assert(!CalTab_Set(CT_FR, 1, CTF_RPM, 150));       /* duplicate speed     */
    assert(CalTab_Set(CT_FR, 1, CTF_RPM, 80));
    assert(CalTab_Set(CT_FR, 0, CTF_STEER, 600));
    assert(CalTab_Set(CT_FR, 1, CTF_STEER, 560));
    assert(!CalTab_Set(CT_FR, 0, CTF_STEER, 700));     /* past the servo      */
    assert(!CalTab_Set(CT_FL, 0, CTF_RPM, 30));
    assert(!CalTab_Set(CT_F, 0, CTF_DIST, 7000));

    /* One anchor: flat everywhere. Two: interpolated, held outside. */
    assert(fabsf(look(CT_FR, CTF_STEER, 80.0f)  - 560.0f) < 0.01f);
    assert(fabsf(look(CT_FR, CTF_STEER, 150.0f) - 600.0f) < 0.01f);
    assert(fabsf(look(CT_FR, CTF_STEER, 115.0f) - 580.0f) < 0.01f);
    assert(fabsf(look(CT_FR, CTF_STEER, 40.0f)  - 560.0f) < 0.01f);
    assert(fabsf(look(CT_FR, CTF_STEER, 250.0f) - 600.0f) < 0.01f);

    /* Third anchor in the middle: piecewise. */
    assert(CalTab_Set(CT_FR, 2, CTF_RPM, 120));
    assert(CalTab_Set(CT_FR, 2, CTF_STEER, 590));
    assert(fabsf(look(CT_FR, CTF_STEER, 100.0f) - 575.0f) < 0.01f);
    assert(fabsf(look(CT_FR, CTF_STEER, 135.0f) - 595.0f) < 0.01f);

    /* A field only some anchors have uses only those. */
    assert(CalTab_Set(CT_FR, 1, CTF_DECEL, 5000));
    assert(fabsf(look(CT_FR, CTF_DECEL, 150.0f) - 5000.0f) < 0.01f);
    assert(!CalTab_Lookup(CT_FR, CTF_LAG, 150.0f, &v));

    /* Clearing an anchor's rpm drops the anchor. */
    assert(CalTab_Set(CT_FR, 2, CTF_RPM, 0));
    assert(fabsf(look(CT_FR, CTF_STEER, 115.0f) - 580.0f) < 0.01f);
    assert(!CalTab_Get(CT_FR, 2, CTF_STEER, &g));

    /* Other moves untouched. */
    assert(!CalTab_Lookup(CT_FL, CTF_STEER, 150.0f, &v));

    /* Globals. */
    assert(CalTab_Global(CTG_WHEEL) == 0);
    assert(CalTab_Set(CT_GLOBAL, 0, CTG_WHEEL, 10050));
    assert(!CalTab_Set(CT_GLOBAL, 0, CTG_WHEEL, 12000));
    assert(CalTab_Set(CT_GLOBAL, 0, CTG_FFBR, 340));
    assert(CalTab_Global(CTG_WHEEL) == 10050 && CalTab_Global(CTG_FFBR) == 340);
    assert(CalTab_Set(CT_GLOBAL, 0, CTG_FFBR, 0));     /* back to compiled */

    /* Negative trim round trip. */
    assert(CalTab_Set(CT_R, 0, CTF_RPM, 100));
    assert(CalTab_Set(CT_R, 0, CTF_TRIM, -37));
    assert(CalTab_Get(CT_R, 0, CTF_TRIM, &g) && g == -37);

    CalTab_Clear();
    assert(!CalTab_Lookup(CT_FR, CTF_STEER, 150.0f, &v));
    assert(CalTab_Global(CTG_WHEEL) == 0);

    /* Names round-trip through the key parser for every legal key. */
    {
        uint8_t m, f;
        char key[24];
        for (m = 0; m <= CT_GLOBAL; m++)
        {
            uint8_t nf = (m == CT_GLOBAL) ? CTG_COUNT : CTF_COUNT;
            for (f = 0; f < nf; f++)
            {
                uint8_t mm, aa, ff, used;
                if (!CalTab_FieldAllowed(m, f)) { continue; }
                strcpy(key, CalTab_MoveName(m));
                if (m != CT_GLOBAL) { strcat(key, "2"); }
                strcat(key, ".");
                strcat(key, CalTab_FieldName(m, f));
                used = CalTab_ParseKey(key, &mm, &aa, &ff);
                assert(used == strlen(key));
                assert(mm == m && ff == f);
            }
        }
    }
    return 0;
}
