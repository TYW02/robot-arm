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

/* Accept R/G/B or red/green/blue in any case. Returns 'R','G','B' or 0. */
static char parse_colour(const char *s) {
    if (word_eq(s, "r") || word_eq(s, "red"))   return 'R';
    if (word_eq(s, "g") || word_eq(s, "green")) return 'G';
    if (word_eq(s, "b") || word_eq(s, "blue"))  return 'B';
    return 0;
}

/* Strict unsigned integer: digits only, no sign, no trailing junk, in range. */
static bool parse_id(const char *s, int max, int *out) {
    if (*s == '\0') return false;
    long v = 0;
    for (const char *p = s; *p; p++) {
        if (!isdigit((unsigned char)*p)) return false;
        v = v * 10 + (*p - '0');
        if (v > max) return false;          /* also stops overflow on long input */
    }
    *out = (int)v;
    return true;
}

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

/* Reply building without stdio: snprintf pulls in newlib's stdio, which
   needs OS hooks (_isatty, _write...) that the µT-Kernel port does not
   provide. These helpers keep cli.c freestanding on bare-metal, FreeRTOS
   and µT-Kernel alike. */

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

/* Unsigned integer to decimal text; buf needs 11 bytes. */
static const char *num_text(unsigned v, char buf[11]) {
    char *p = buf + 10;
    *p = '\0';
    do { *--p = (char)('0' + v % 10); v /= 10; } while (v);
    return p;
}

/* ------------------------------------------------------------------ */
/* Command table                                                       */
/* ------------------------------------------------------------------ */

typedef enum { ARG_NONE, ARG_COLOUR, ARG_COLOUR_ID } arg_kind_t;

typedef struct {
    const char *word;
    cmd_type_t  type;
    arg_kind_t  args;
    const char *usage;
} cli_cmd_def_t;

static const cli_cmd_def_t CMDS[] = {
    { "pick",   CMD_PICK,      ARG_COLOUR,    "pick <R|G|B>"     },
    { "map",    CMD_MAP,       ARG_COLOUR_ID, "map <R|G|B> <id>" },
    { "home",   CMD_HOME,      ARG_NONE,      "home"             },
    { "stop",   CMD_STOP,      ARG_NONE,      "stop"             },
    { "reset",  CMD_RESET,     ARG_NONE,      "reset"            },
    { "cal",    CMD_RECAL_IMU, ARG_NONE,      "cal"              },
    { "status", CMD_STATUS,    ARG_NONE,      "status"           },
};
#define N_CMDS (sizeof CMDS / sizeof CMDS[0])

const char *cli_cmd_name(cmd_type_t t) {
    for (size_t i = 0; i < N_CMDS; i++)
        if (CMDS[i].type == t) return CMDS[i].word;
    return "?";
}

const char *cli_help_text(void) {
    return "Commands:\n"
           "  pick <R|G|B>      pick an object of that colour and shelve it\n"
           "  map <R|G|B> <id>  set shelf ArUco marker id (0-999) for a colour\n"
           "  home              move arm to home pose\n"
           "  stop              abort the current motion (arm holds position)\n"
           "  reset             clear a FAULT\n"
           "  cal               re-capture IMU level reference (arm must be level)\n"
           "  status            show state, mapping, IMU\n"
           "  help              show this list\n"
           "Colours: R/G/B or red/green/blue. Commands are not case-sensitive.";
}

/* ------------------------------------------------------------------ */
/* Parser                                                              */
/* ------------------------------------------------------------------ */

cli_result_t cli_parse_line(const char *line, cmd_msg_t *out,
                            char *reply, size_t reply_len) {
    if (reply && reply_len) reply[0] = '\0';

    if (strlen(line) > CLI_LINE_MAX) {
        char nb[11];
        set_reply(reply, reply_len, "error: line too long (max ",
                  num_text(CLI_LINE_MAX, nb), " chars)", NULL);
        return CLI_ERR_TOO_LONG;
    }

    char buf[CLI_LINE_MAX + 1];
    strcpy(buf, line);

    char *tok[CLI_MAX_TOKENS];
    int n = tokenize(buf, tok, CLI_MAX_TOKENS);
    if (n == 0) return CLI_EMPTY;

    if (word_eq(tok[0], "help") || word_eq(tok[0], "?")) {
        return CLI_HELP;   /* caller prints cli_help_text(); too long for reply */
    }

    const cli_cmd_def_t *def = NULL;
    for (size_t i = 0; i < N_CMDS; i++)
        if (word_eq(tok[0], CMDS[i].word)) { def = &CMDS[i]; break; }

    if (!def) {
        set_reply(reply, reply_len, "error: unknown command '", tok[0], "' (try: help)", NULL);
        return CLI_ERR_UNKNOWN;
    }

    int want = (def->args == ARG_NONE) ? 0 : (def->args == ARG_COLOUR) ? 1 : 2;
    if (n - 1 != want) {
        set_reply(reply, reply_len, "usage: ", def->usage, NULL);
        return CLI_ERR_ARGS;
    }

    cmd_msg_t m = { def->type, 0, 0 };

    if (def->args != ARG_NONE) {
        m.colour = parse_colour(tok[1]);
        if (!m.colour) {
            set_reply(reply, reply_len, "error: colour '", tok[1], "' must be R, G or B", NULL);
            return CLI_ERR_COLOUR;
        }
    }
    if (def->args == ARG_COLOUR_ID) {
        if (!parse_id(tok[2], CLI_MAX_MARKER_ID, &m.value)) {
            char nb[11];
            set_reply(reply, reply_len, "error: marker id '", tok[2],
                      "' must be a whole number 0-", num_text(CLI_MAX_MARKER_ID, nb), NULL);
            return CLI_ERR_ID;
        }
    }

    if (out) *out = m;

    char col[2] = { m.colour, '\0' };
    char nb[11];
    if (def->args == ARG_NONE)        set_reply(reply, reply_len, "ok: ", def->word, NULL);
    else if (def->args == ARG_COLOUR) set_reply(reply, reply_len, "ok: ", def->word, " ", col, NULL);
    else set_reply(reply, reply_len, "ok: ", def->word, " ", col, " ",
                   num_text((unsigned)m.value, nb), NULL);
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
