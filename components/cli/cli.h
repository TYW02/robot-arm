#ifndef CLI_H
#define CLI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---------- Limits ---------- */
#define CLI_LINE_MAX        48    /* max characters in one typed command   */
#define CLI_REPLY_MAX       96    /* max characters in one reply line      */
#define CLI_MAX_MARKER_ID   999   /* must equal VISION_MAX_MARKER_ID (ArUco 4x4_1000) */

/* ---------- Command message sent to the Coordinator via q_cmd ---------- */
#ifndef CMD_MSG_T_DEFINED
#define CMD_MSG_T_DEFINED
typedef enum {
    CMD_PICK,       /* colour = 'R' | 'G' | 'B'               */
    CMD_MAP,        /* colour, value = shelf ArUco marker id   */
    CMD_HOME,
    CMD_STOP,
    CMD_RESET,      /* clear FAULT                             */
    CMD_STATUS,
    CMD_RECAL_IMU,  /* re-capture IMU level reference          */
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    char       colour;
    int        value;
} cmd_msg_t;
#endif

/* ---------- Parse result ---------- */
typedef enum {
    CLI_OK = 0,        /* valid command written to *out, send it to q_cmd */
    CLI_EMPTY,         /* blank line, nothing to do, no reply              */
    CLI_HELP,          /* help text requested (local, not sent)            */
    CLI_ERR_UNKNOWN,   /* unknown command word                             */
    CLI_ERR_ARGS,      /* wrong number of arguments                        */
    CLI_ERR_COLOUR,    /* colour is not R, G or B                          */
    CLI_ERR_ID,        /* marker id not an integer in 0..CLI_MAX_MARKER_ID */
    CLI_ERR_TOO_LONG,  /* line exceeded CLI_LINE_MAX                       */
} cli_result_t;

/*
 * Parse one complete command line.
 *   line  : NUL-terminated text, without the newline
 *   out   : filled only when CLI_OK is returned
 *   reply : filled with the text to show the operator, e.g. "ok: pick R" or
 *           "error: colour 'x' must be R, G or B". Empty for CLI_EMPTY and
 *           CLI_HELP (print cli_help_text() instead).
 */
cli_result_t cli_parse_line(const char *line, cmd_msg_t *out,
                            char *reply, size_t reply_len);

/* Multi-line help text shown for the 'help' command. */
const char *cli_help_text(void);

/* Short name for a command type, for logs ("pick", "map", ...). */
const char *cli_cmd_name(cmd_type_t t);

/* ---------- Line editor: feed one received character at a time ---------- */
typedef struct {
    char   buf[CLI_LINE_MAX + 1];
    size_t len;
    bool   overflow;   /* input went past CLI_LINE_MAX; whole line rejected */
} cli_line_t;

typedef enum {
    CLI_FEED_NONE,       /* ignored (control char, or nothing to delete) */
    CLI_FEED_ECHO,       /* printable char stored: echo it back          */
    CLI_FEED_BACKSPACE,  /* last char removed: echo "\b \b"              */
    CLI_FEED_LINE,       /* Enter pressed: line->buf holds the command   */
    CLI_FEED_TOO_LONG,   /* Enter pressed after overflow: reject line    */
} cli_feed_t;

void       cli_line_reset(cli_line_t *l);
cli_feed_t cli_line_feed(cli_line_t *l, int ch);

#endif /* CLI_H */
