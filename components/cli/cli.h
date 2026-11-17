/*
 * cli.h - operator command parser for the ArUco sorting car
 * Owner: Reiner
 *
 * Pure C, no RTOS: parses one typed line into a cmd_msg_t and a reply line.
 * cli_utkernel.c runs it in the UI task and hands each command to the
 * integration layer (integration/cli_bridge.c), which decides what happens.
 *
 * The car picks up any block and delivers it to the nearest wall marker;
 * the CLI starts, stops, resets and monitors the mission.
 */
#ifndef CLI_H
#define CLI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---------- Limits ---------- */
#define CLI_LINE_MAX        48    /* max characters in one typed command   */
#define CLI_REPLY_MAX       96    /* max characters in one reply line      */

/* 1 = accept the debug commands (manual drive and claw). 0 = operator only. */
#ifndef CLI_ENABLE_MANUAL
#define CLI_ENABLE_MANUAL   1
#endif

/* Manual drive limits (debug commands). */
#define CLI_DRIVE_DEFAULT_SPEED  30
#define CLI_DRIVE_DEFAULT_MS     500
#define CLI_DRIVE_MAX_MS         5000

/* Special claw targets for CMD_CLAW (otherwise a = angle 0-180). */
#define CLI_CLAW_OFF    (-1)
#define CLI_CLAW_OPEN   (-2)
#define CLI_CLAW_CLOSE  (-3)

/* ---------- Command message handed to the integration layer ---------- */
#ifndef CMD_MSG_T_DEFINED
#define CMD_MSG_T_DEFINED
typedef enum {
    CMD_START,      /* begin the autonomous mission                      */
    CMD_STOP,       /* emergency stop, latched (always accepted)         */
    CMD_RESET,      /* clear a latched stop, back to IDLE                */
    CMD_STATUS,     /* print state, held object, IMU                     */
    CMD_DRIVE,      /* debug: a = left %, b = right %, c = ms            */
    CMD_CLAW,       /* debug: a = angle 0-180 or CLI_CLAW_*              */
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    int        a;
    int        b;
    int        c;
} cmd_msg_t;
#endif

/* ---------- Parse result ---------- */
typedef enum {
    CLI_OK = 0,        /* valid command written to *out                    */
    CLI_EMPTY,         /* blank line, nothing to do, no reply              */
    CLI_HELP,          /* help text requested (local, not sent)            */
    CLI_ERR_UNKNOWN,   /* unknown command word                             */
    CLI_ERR_ARGS,      /* wrong number of arguments                        */
    CLI_ERR_VALUE,     /* speed, time or angle out of range                */
    CLI_ERR_TOO_LONG,  /* line exceeded CLI_LINE_MAX                       */
} cli_result_t;

/*
 * Parse one complete command line.
 *   line  : NUL-terminated text, without the newline
 *   out   : filled only when CLI_OK is returned
 *   reply : the text to show the operator, e.g. "ok: start" or
 *           "error: unknown command 'fly' (try: help)".
 *           Empty for CLI_EMPTY and CLI_HELP (print cli_help_text()).
 *           For CLI_OK the integration layer may replace it (e.g. "rejected").
 */
cli_result_t cli_parse_line(const char *line, cmd_msg_t *out,
                            char *reply, size_t reply_len);

/* Multi-line help text shown for the 'help' command. */
const char *cli_help_text(void);

/* Short name for a command type, for logs ("start", "stop", ...). */
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
