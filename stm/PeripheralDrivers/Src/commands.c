/**
  ******************************************************************************
  * @file    commands.c
  * @brief   Parser and queue for the RPi <-> STM32 protocol frozen in
  *          commands.h. Person A side. No hardware here - this file is pure
  *          string handling and can be unit-tested off-target.
  *
  *          Parsing is all-or-nothing per LINE. Tokens are validated into a
  *          scratch array first and only committed to the queue once every
  *          one of them has passed, so a line ending in garbage never leaves
  *          half a manoeuvre queued behind a RESEND.
  ******************************************************************************
  */

#include "commands.h"
#include <string.h>

/* Circular queue. Head is the next slot to pop, count is what is in it. */
static Command_t s_queue[CMD_QUEUE_DEPTH];
static uint8_t   s_head;
static uint8_t   s_count;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static char lower(char c)
{
    return ((c >= 'A') && (c <= 'Z')) ? (char)(c - 'A' + 'a') : c;
}

/* Case-insensitive compare of a whole token against a literal. */
static uint8_t token_is(const char *tok, const char *lit)
{
    uint8_t i = 0U;

    while ((lit[i] != '\0') && (tok[i] != '\0'))
    {
        if (lower(tok[i]) != lit[i]) { return 0U; }
        i++;
    }

    return ((lit[i] == '\0') && (tok[i] == '\0')) ? 1U : 0U;
}

/* Case-insensitive prefix match. Returns the prefix length, or 0. */
static uint8_t token_starts(const char *tok, const char *pfx)
{
    uint8_t i = 0U;

    while (pfx[i] != '\0')
    {
        if (lower(tok[i]) != pfx[i]) { return 0U; }
        i++;
    }

    return i;
}

/* Strict unsigned decimal. The whole remainder must be digits and there must
 * be at least one, so "F", "F1x" and "F-5" are all rejected rather than
 * silently becoming F0.
 *
 * Returns 1 on success. Anything above INT16_MAX is a parse failure, not a
 * clamp - a distance the sender did not mean is worse than a RESEND. */
static uint8_t parse_uint(const char *s, int16_t *out)
{
    uint32_t v = 0U;
    uint8_t  n = 0U;

    while (s[n] != '\0')
    {
        if ((s[n] < '0') || (s[n] > '9')) { return 0U; }

        v = (v * 10U) + (uint32_t)(s[n] - '0');
        if (v > 32767U) { return 0U; }

        n++;
    }

    if (n == 0U) { return 0U; }

    *out = (int16_t)v;
    return 1U;
}

/* As parse_uint, but accepts a leading '-'.
 *
 * Deliberately NOT used for movement arguments. Every one of those is a
 * magnitude with its direction in the opcode, and keeping it that way is what
 * stops "R-90" and "FL90" both being spellable for the same manoeuvre. The
 * steering trim is a signed servo offset with no opcode to carry the sign,
 * so it is the one argument that needs this. */
static uint8_t parse_int(const char *s, int16_t *out)
{
    int16_t v;

    if (s[0] == '-')
    {
        if (!parse_uint(&s[1], &v)) { return 0U; }
        *out = (int16_t)(-v);
        return 1U;
    }

    return parse_uint(s, out);
}

/* ------------------------------------------------------------------ */
/* Token parsing                                                       */
/* ------------------------------------------------------------------ */

uint8_t Cmd_IsImmediate(CmdOpcode_t op)
{
    /* A RANGE, not a list - so a new immediate opcode added OUTSIDE it is
     * silently handed to the movement parser instead. Append new ones before
     * this endpoint and move the endpoint with them. */
    return ((op >= CMD_Q_US) && (op <= CMD_SET_CAL_TRIM)) ? 1U : 0U;
}

Command_t Cmd_ParseToken(const char *token)
{
    Command_t   cmd;
    uint8_t     n;
    int16_t     arg = 0;
    CmdOpcode_t op  = CMD_INVALID;

    cmd.op  = CMD_INVALID;
    cmd.arg = 0;

    if ((token == 0) || (token[0] == '\0')) { return cmd; }

    /* Immediate opcodes live in their own '?' and '!' namespaces, deliberately
     * disjoint from the movement tokens. That is what stops a repeat of the
     * 'S' problem, where one letter meant stop here and reverse on the sender.
     * Neither character is touched by lower(), so the comparisons below work
     * on raw input. */
    if (token[0] == '?')
    {
        if (token_is(token, "?us"))   { cmd.op = CMD_Q_US;   }
        else if (token_is(token, "?ir"))   { cmd.op = CMD_Q_IR;   }
        else if (token_is(token, "?irr"))  { cmd.op = CMD_Q_IRR;  }
        else if (token_is(token, "?pose")) { cmd.op = CMD_Q_POSE; }
        else if (token_is(token, "?dist")) { cmd.op = CMD_Q_DIST; }
        else if (token_is(token, "?turn")) { cmd.op = CMD_Q_TURN; }
        else if (token_is(token, "?stat")) { cmd.op = CMD_Q_STAT; }
        else if (token_is(token, "?imu"))  { cmd.op = CMD_Q_IMU;  }
        else if (token_is(token, "?xchk")) { cmd.op = CMD_Q_XCHK; }
        else if (token_is(token, "?ver"))  { cmd.op = CMD_Q_VER;  }
        else if (token_is(token, "?cal"))  { cmd.op = CMD_Q_CAL;  }
        return cmd;
    }

    if (token[0] == '!')
    {
        if (token_is(token, "!zero")) { cmd.op = CMD_SET_ZERO; return cmd; }

        if ((n = token_starts(token, "!prof")) != 0U)
        {
            if (!parse_uint(&token[n], &arg)) { return cmd; }
            if (arg > 2)                      { return cmd; }
            cmd.op  = CMD_SET_PROFILE;
            cmd.arg = arg;
            return cmd;
        }

        /* The three calibration setters. Longer prefixes are all distinct at
         * the fifth character, so order between them does not matter - but
         * they must all be tested, because a bare "!cal" is not a command and
         * has to fall through to CMD_INVALID rather than half-match. */
        if ((n = token_starts(token, "!cald")) != 0U)
        {
            if (!parse_uint(&token[n], &arg)) { return cmd; }
            cmd.op  = CMD_SET_CAL_DECEL;
            cmd.arg = arg;
            return cmd;
        }

        if ((n = token_starts(token, "!call")) != 0U)
        {
            if (!parse_uint(&token[n], &arg)) { return cmd; }
            cmd.op  = CMD_SET_CAL_LAG;
            cmd.arg = arg;
            return cmd;
        }

        if ((n = token_starts(token, "!calt")) != 0U)
        {
            if (!parse_int(&token[n], &arg)) { return cmd; }
            cmd.op  = CMD_SET_CAL_TRIM;
            cmd.arg = arg;
            return cmd;
        }

        return cmd;
    }

    /* Exact matches first. "RST" has to be tested before the R{n} prefix or
     * it parses as a reverse of "st" and fails for the wrong reason. */
    if (token_is(token, "rst")) { cmd.op = CMD_RESET; return cmd; }
    if (token_is(token, "s"))   { cmd.op = CMD_STOP;  return cmd; }

    /* Two-letter prefixes before the one-letter drive prefixes, so "fr" and
     * "fu" are not eaten by "f". */
    if      ((n = token_starts(token, "fr")) != 0U) { op = CMD_ARC_FWD_RIGHT; }
    else if ((n = token_starts(token, "fl")) != 0U) { op = CMD_ARC_FWD_LEFT;  }
    else if ((n = token_starts(token, "fu")) != 0U) { op = CMD_FWD_UNTIL_US;  }
    else if ((n = token_starts(token, "rr")) != 0U) { op = CMD_ARC_REV_RIGHT; }
    else if ((n = token_starts(token, "rl")) != 0U) { op = CMD_ARC_REV_LEFT;  }
    else if ((n = token_starts(token, "f"))  != 0U) { op = CMD_FORWARD;       }
    else if ((n = token_starts(token, "r"))  != 0U) { op = CMD_REVERSE;       }
    else                                            { return cmd; }

    if (!parse_uint(&token[n], &arg)) { return cmd; }

    /* A zero-degree arc is not a manoeuvre. F0 IS meaningful - it is
     * "forward until an obstacle stops you" - and R0 is not, so only F is
     * allowed to carry a zero. */
    if (arg == 0)
    {
        if (op != CMD_FORWARD) { return cmd; }
    }

    /* FU's argument is a gap to stand off at, not a distance to travel, and
     * it is the one movement argument with a meaningful FLOOR as well as a
     * ceiling. Checked here in the parser rather than in the executor so a
     * bad value costs a RESEND with nothing queued, instead of being
     * discovered halfway down a line that has already started moving. */
    if (op == CMD_FWD_UNTIL_US)
    {
        if ((arg < CMD_FU_MIN_CM) || (arg > CMD_FU_MAX_CM)) { return cmd; }
    }

    cmd.op  = op;
    cmd.arg = arg;
    return cmd;
}

/* ------------------------------------------------------------------ */
/* Line parsing                                                        */
/* ------------------------------------------------------------------ */

uint8_t Cmd_ParseLine(const char *line)
{
    Command_t staged[CMD_QUEUE_DEPTH];
    char      tok[16];
    uint8_t   nstaged = 0U;
    uint8_t   tlen    = 0U;
    uint16_t  i       = 0U;
    uint8_t   j;
    char      c;

    if (line == 0) { return 0U; }

    /* Walk the line one character at a time, cutting a token at every comma,
     * space or end of string. Empty runs of delimiters are skipped so
     * "F10,,F10" and "F10  F10" both behave. */
    for (;;)
    {
        c = line[i];

        if ((c == CMD_DELIM_PRIMARY) || (c == CMD_DELIM_SECONDARY) ||
            (c == '\t') || (c == '\r') || (c == '\0'))
        {
            if (tlen > 0U)
            {
                tok[tlen] = '\0';

                if (nstaged >= CMD_QUEUE_DEPTH) { return 0U; }

                staged[nstaged] = Cmd_ParseToken(tok);
                if (staged[nstaged].op == CMD_INVALID) { return 0U; }

                /* An immediate opcode is answered with a data line of its own,
                 * so allowing one inside a movement line would mean two replies
                 * for one line - and the sender's whole sequencing rests on
                 * there being exactly one. Reject rather than silently pick a
                 * winner. The caller handles a lone immediate token before it
                 * ever reaches here. */
                if (Cmd_IsImmediate(staged[nstaged].op)) { return 0U; }

                nstaged++;
                tlen = 0U;
            }

            if (c == '\0') { break; }
        }
        else
        {
            /* A token longer than the buffer cannot be valid, and letting it
             * run would overflow tok[]. */
            if (tlen >= (sizeof(tok) - 1U)) { return 0U; }

            tok[tlen] = c;
            tlen++;
        }

        i++;
        if (i >= CMD_LINE_MAX) { return 0U; }
    }

    /* An empty line is not a command. Replying OK to it would let a stray
     * newline look like a completed manoeuvre. */
    if (nstaged == 0U) { return 0U; }

    /* Room check before committing anything. */
    if ((uint16_t)s_count + (uint16_t)nstaged > (uint16_t)CMD_QUEUE_DEPTH)
    {
        return 0U;
    }

    for (j = 0U; j < nstaged; j++)
    {
        s_queue[(uint8_t)((s_head + s_count) % CMD_QUEUE_DEPTH)] = staged[j];
        s_count++;
    }

    return 1U;
}

/* ------------------------------------------------------------------ */
/* Queue                                                               */
/* ------------------------------------------------------------------ */

void Cmd_Init(void)
{
    s_head  = 0U;
    s_count = 0U;
}

Command_t Cmd_QueuePop(void)
{
    Command_t out;

    if (s_count == 0U)
    {
        out.op  = CMD_NONE;
        out.arg = 0;
        return out;
    }

    out    = s_queue[s_head];
    s_head = (uint8_t)((s_head + 1U) % CMD_QUEUE_DEPTH);
    s_count--;

    return out;
}

uint8_t Cmd_QueueCount(void) { return s_count; }

void Cmd_QueueFlush(void)
{
    s_head  = 0U;
    s_count = 0U;
}
