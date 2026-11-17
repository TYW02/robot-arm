/*
 * cli.c - operator command parser for the ArUco sorting car
 * Owner: Reiner
 *
 * Freestanding C: no stdio (newlib's snprintf needs OS hooks the µT-Kernel
 * port does not provide). Host-tested by test/test_cli_host.c.
 */
#include "cli.h"

#include <ctype.h>
#include <stdarg.h>
#include <string.h>

#define CLI_MAX_TOKENS 4   /* one more than any command needs, to detect extras */

/* ------------------------------------------------------------------ */
/* Small helpers                                                       */
/* ------------------------------------------------------------------ */

/* Case-insensitive string compare (strcasecmp is not standard C). */
static bool word_eq(const char *a, const char *b) {
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
        a++; b++;
    }
    return *a == '\0' && *b == '\0';
}

#if CLI_ENABLE_MANUAL
/* Strict signed integer in [lo, hi]: optional '-', digits only, no junk. */
static bool parse_int(const char *s, int lo, int hi, int *out) {
    bool neg = false;
    if (*s == '-') { neg = true; s++; }
    if (*s == '\0') return false;
    long v = 0;
    for (const char *p = s; *p; p++) {
        if (!isdigit((unsigned char)*p)) return false;
        v = v * 10 + (*p - '0');
        if (v > 100000) return false;          /* stops overflow on long input */
    }
    if (neg) v = -v;
    if (v < lo || v > hi) return false;
    *out = (int)v;
    return true;
}
#endif

/* Split a copy of the line into whitespace-separated tokens (no strtok). */
static int tokenize(char *buf, char *tok[], int max_tok) {
    int n = 0;
    char *p = buf;
    while (*p) {
        while (*p == ' ' || *p == '\t') *p++ = '\0';
        if (!*p) break;
        if (n == max_tok) return max_tok + 1;   /* too many tokens */
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
    }
    return n;
}

/* Concatenate NULL-terminated string arguments into reply, truncating safely. */
static void set_reply(char *reply, size_t len, ...) {
    if (!reply || len == 0) return;
    size_t used = 0;
    va_list ap;
    va_start(ap, len);
    for (const char *s = va_arg(ap, const char *); s; s = va_arg(ap, const char *))
        while (*s && used < len - 1) reply[used++] = *s++;
    va_end(ap);
    reply[used] = '\0';
}

/* Signed integer to decimal text; buf needs 12 bytes. */
static const char *num_text(int v, char buf[12]) {
    unsigned u = (v < 0) ? (unsigned)(-(long)v) : (unsigned)v;
    char *p = buf + 11;
    *p = '\0';
    do { *--p = (char)('0' + u % 10); u /= 10; } while (u);
    if (v < 0) *--p = '-';
    return p;
}

/* ------------------------------------------------------------------ */
/* Help and names                                                      */
/* ------------------------------------------------------------------ */

const char *cli_cmd_name(cmd_type_t t) {
    switch (t) {
    case CMD_START:    return "start";
    case CMD_STOP:     return "stop";
    case CMD_RESET:    return "reset";
    case CMD_STATUS:   return "status";
    case CMD_DRIVE:    return "drive";
    case CMD_CLAW:     return "claw";
    }
    return "?";
}

const char *cli_help_text(void) {
    return "Commands:\n"
           "  start              start the mission\n"
           "  stop  (or x)       emergency stop, latched, works at any time\n"
           "  reset              clear a stop, back to IDLE\n"
           "  status             state, held object, IMU\n"
           "  help               show this list\n"
#if CLI_ENABLE_MANUAL
           "Debug (IDLE only):\n"
           "  f|b|l|r [speed] [ms]   forward/back/spin left/spin right (30%, 500 ms)\n"
           "  drive <L> <R> [ms]     each wheel -100..100 %, ms 1-5000\n"
           "  open | close           claw to its open / close angle\n"
           "  claw <0-180> | off     claw to an angle, or limp\n"
#endif
           "Commands are not case-sensitive.";
}

/* ------------------------------------------------------------------ */
/* Parser                                                              */
/* ------------------------------------------------------------------ */

static cli_result_t usage(char *reply, size_t len, const char *u) {
    set_reply(reply, len, "usage: ", u, NULL);
    return CLI_ERR_ARGS;
}

#if CLI_ENABLE_MANUAL
/* f/b/l/r [speed] [ms]: dl/dr give the wheel signs. */
static cli_result_t parse_move(char *tok[], int n, int dl, int dr,
                               cmd_msg_t *m, char *reply, size_t len) {
    int speed = CLI_DRIVE_DEFAULT_SPEED, ms = CLI_DRIVE_DEFAULT_MS;
    if (n > 3) return usage(reply, len, "f|b|l|r [speed 1-100] [ms 1-5000]");
    if (n >= 2 && !parse_int(tok[1], 1, 100, &speed)) {
        set_reply(reply, len, "error: speed '", tok[1], "' must be 1-100", NULL);
        return CLI_ERR_VALUE;
    }
    if (n == 3 && !parse_int(tok[2], 1, CLI_DRIVE_MAX_MS, &ms)) {
        set_reply(reply, len, "error: time '", tok[2], "' must be 1-5000 ms", NULL);
        return CLI_ERR_VALUE;
    }
    m->type = CMD_DRIVE;
    m->a = dl * speed;
    m->b = dr * speed;
    m->c = ms;
    return CLI_OK;
}
#endif

cli_result_t cli_parse_line(const char *line, cmd_msg_t *out,
                            char *reply, size_t reply_len) {
    if (reply && reply_len) reply[0] = '\0';

    if (strlen(line) > CLI_LINE_MAX) {
        char nb[12];
        set_reply(reply, reply_len, "error: line too long (max ",
                  num_text(CLI_LINE_MAX, nb), " chars)", NULL);
        return CLI_ERR_TOO_LONG;
    }

    char buf[CLI_LINE_MAX + 1];
    strcpy(buf, line);

    char *tok[CLI_MAX_TOKENS];
    int n = tokenize(buf, tok, CLI_MAX_TOKENS);
    if (n == 0) return CLI_EMPTY;

    const char *w = tok[0];
    cmd_msg_t m = { CMD_STATUS, 0, 0, 0 };
    cli_result_t r = CLI_OK;
    char n1[12], n2[12], n3[12];
    (void)r;   /* only used by the debug commands */

    if (word_eq(w, "help") || word_eq(w, "?")) return CLI_HELP;

    /* ---- operator commands ---- */
    if (word_eq(w, "stop") || word_eq(w, "x")) {
        if (n != 1) return usage(reply, reply_len, "stop");
        m.type = CMD_STOP;
        set_reply(reply, reply_len, "ok: stop", NULL);
    } else if (word_eq(w, "start")) {
        if (n != 1) return usage(reply, reply_len, "start");
        m.type = CMD_START;
        set_reply(reply, reply_len, "ok: start", NULL);
    } else if (word_eq(w, "reset")) {
        if (n != 1) return usage(reply, reply_len, "reset");
        m.type = CMD_RESET;
        set_reply(reply, reply_len, "ok: reset", NULL);
    } else if (word_eq(w, "status")) {
        if (n != 1) return usage(reply, reply_len, "status");
        m.type = CMD_STATUS;
    }
#if CLI_ENABLE_MANUAL
    /* ---- debug commands ---- */
    else if (word_eq(w, "f") || word_eq(w, "fwd")) {
        if ((r = parse_move(tok, n,  1,  1, &m, reply, reply_len)) != CLI_OK) return r;
    } else if (word_eq(w, "b") || word_eq(w, "back")) {
        if ((r = parse_move(tok, n, -1, -1, &m, reply, reply_len)) != CLI_OK) return r;
    } else if (word_eq(w, "l") || word_eq(w, "left")) {
        if ((r = parse_move(tok, n, -1,  1, &m, reply, reply_len)) != CLI_OK) return r;
    } else if (word_eq(w, "r") || word_eq(w, "right")) {
        if ((r = parse_move(tok, n,  1, -1, &m, reply, reply_len)) != CLI_OK) return r;
    } else if (word_eq(w, "drive")) {
        if (n < 3 || n > 4)
            return usage(reply, reply_len, "drive <L -100..100> <R -100..100> [ms 1-5000]");
        m.type = CMD_DRIVE;
        m.c = CLI_DRIVE_DEFAULT_MS;
        if (!parse_int(tok[1], -100, 100, &m.a) || !parse_int(tok[2], -100, 100, &m.b)) {
            set_reply(reply, reply_len, "error: wheel speeds must be -100..100", NULL);
            return CLI_ERR_VALUE;
        }
        if (n == 4 && !parse_int(tok[3], 1, CLI_DRIVE_MAX_MS, &m.c)) {
            set_reply(reply, reply_len, "error: time '", tok[3], "' must be 1-5000 ms", NULL);
            return CLI_ERR_VALUE;
        }
    } else if (word_eq(w, "open") || word_eq(w, "close")) {
        if (n != 1) return usage(reply, reply_len, word_eq(w, "open") ? "open" : "close");
        m.type = CMD_CLAW;
        m.a = word_eq(w, "open") ? CLI_CLAW_OPEN : CLI_CLAW_CLOSE;
        set_reply(reply, reply_len, "ok: claw ", word_eq(w, "open") ? "open" : "close", NULL);
    } else if (word_eq(w, "claw")) {
        if (n != 2) return usage(reply, reply_len, "claw <0-180> | claw off");
        m.type = CMD_CLAW;
        if (word_eq(tok[1], "off")) {
            m.a = CLI_CLAW_OFF;
            set_reply(reply, reply_len, "ok: claw off", NULL);
        } else if (parse_int(tok[1], 0, 180, &m.a)) {
            set_reply(reply, reply_len, "ok: claw ", num_text(m.a, n1), " deg", NULL);
        } else {
            set_reply(reply, reply_len, "error: angle '", tok[1], "' must be 0-180 or off", NULL);
            return CLI_ERR_VALUE;
        }
    }
#endif
    else {
        set_reply(reply, reply_len, "error: unknown command '", w, "' (try: help)", NULL);
        return CLI_ERR_UNKNOWN;
    }

    if (m.type == CMD_DRIVE)
        set_reply(reply, reply_len, "ok: drive L=", num_text(m.a, n1), " R=",
                  num_text(m.b, n2), " ", num_text(m.c, n3), " ms", NULL);

    if (out) *out = m;
    return CLI_OK;
}

/* ------------------------------------------------------------------ */
/* Line editor                                                         */
/* ------------------------------------------------------------------ */

void cli_line_reset(cli_line_t *l) {
    l->len = 0;
    l->overflow = false;
    l->buf[0] = '\0';
}

cli_feed_t cli_line_feed(cli_line_t *l, int ch) {
    if (ch == '\r' || ch == '\n') {
        l->buf[l->len] = '\0';
        if (l->len == 0 && !l->overflow) {
            /* Ignore the '\n' of a "\r\n" pair and stray blank lines. */
            return (ch == '\n') ? CLI_FEED_NONE : CLI_FEED_LINE;
        }
        return l->overflow ? CLI_FEED_TOO_LONG : CLI_FEED_LINE;
    }
    if (ch == '\b' || ch == 0x7f) {
        if (l->len == 0 || l->overflow) return CLI_FEED_NONE;
        l->len--;
        return CLI_FEED_BACKSPACE;
    }
    if (ch >= 32 && ch < 127) {
        if (l->len >= CLI_LINE_MAX) { l->overflow = true; return CLI_FEED_NONE; }
        l->buf[l->len++] = (char)ch;
        return CLI_FEED_ECHO;
    }
    return CLI_FEED_NONE;   /* other control characters */
}
