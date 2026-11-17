/*
 * test_cli_host.c - desktop unit test for the CLI parser and line editor.
 * Owner: Reiner
 *
 * Build and run on a PC (no Pico needed), from components/cli:
 *   gcc -std=c11 -Wall -Wextra -I. cli.c test/test_cli_host.c -o test_cli && ./test_cli
 *
 * Exit code 0 = all tests passed. Paste the output into the report as evidence.
 */
#include "cli.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

static int pass = 0, fail = 0;

/* want_a/b/c are checked only for CLI_OK. */
static void check_parse(const char *line, cli_result_t want_res,
                        cmd_type_t want_type, int want_a, int want_b, int want_c) {
    cmd_msg_t m = { (cmd_type_t)-1, -99, -99, -99 };
    char reply[CLI_REPLY_MAX];
    cli_result_t r = cli_parse_line(line, &m, reply, sizeof reply);

    int ok = (r == want_res);
    if (ok && r == CLI_OK)
        ok = (m.type == want_type && m.a == want_a && m.b == want_b && m.c == want_c);

    printf("%s  %-26s -> %s\n", ok ? "PASS" : "FAIL", line, reply[0] ? reply : "(no reply)");
    ok ? pass++ : fail++;
}

/* Feed a string through the line editor exactly as typed characters. */
static cli_feed_t feed_string(cli_line_t *l, const char *s) {
    cli_feed_t last = CLI_FEED_NONE;
    for (; *s; s++) last = cli_line_feed(l, (unsigned char)*s);
    return last;
}

static void check_editor(const char *name, const char *typed,
                         cli_feed_t want_feed, const char *want_buf) {
    cli_line_t l;
    cli_line_reset(&l);
    cli_feed_t f = feed_string(&l, typed);
    int ok = (f == want_feed) && (want_buf == NULL || strcmp(l.buf, want_buf) == 0);
    printf("%s  editor: %-19s -> \"%s\"\n", ok ? "PASS" : "FAIL", name, l.buf);
    ok ? pass++ : fail++;
}

int main(void) {
    printf("== Operator commands ==\n");
    check_parse("start",             CLI_OK, CMD_START,    0, 0, 0);
    check_parse("stop",              CLI_OK, CMD_STOP,     0, 0, 0);
    check_parse("X",                 CLI_OK, CMD_STOP,     0, 0, 0);
    check_parse("reset",             CLI_OK, CMD_RESET,    0, 0, 0);
    check_parse("Status",            CLI_OK, CMD_STATUS,   0, 0, 0);
    check_parse("  START  ",         CLI_OK, CMD_START,    0, 0, 0);

    printf("\n== Debug commands ==\n");
    check_parse("f",                 CLI_OK, CMD_DRIVE,  30,  30, 500);
    check_parse("b 50 1000",         CLI_OK, CMD_DRIVE, -50, -50, 1000);
    check_parse("l 40",              CLI_OK, CMD_DRIVE, -40,  40, 500);
    check_parse("right 20 300",      CLI_OK, CMD_DRIVE,  20, -20, 300);
    check_parse("drive 40 -60 800",  CLI_OK, CMD_DRIVE,  40, -60, 800);
    check_parse("drive 0 50",        CLI_OK, CMD_DRIVE,   0,  50, 500);
    check_parse("open",              CLI_OK, CMD_CLAW, CLI_CLAW_OPEN, 0, 0);
    check_parse("close",             CLI_OK, CMD_CLAW, CLI_CLAW_CLOSE, 0, 0);
    check_parse("claw 90",           CLI_OK, CMD_CLAW, 90, 0, 0);
    check_parse("claw off",          CLI_OK, CMD_CLAW, CLI_CLAW_OFF, 0, 0);

    printf("\n== Invalid / boundary commands (must be rejected) ==\n");
    check_parse("",                  CLI_EMPTY,       0, 0, 0, 0);
    check_parse("   ",               CLI_EMPTY,       0, 0, 0, 0);
    check_parse("help",              CLI_HELP,        0, 0, 0, 0);
    check_parse("fly",               CLI_ERR_UNKNOWN, 0, 0, 0, 0);
    check_parse("pick R",            CLI_ERR_UNKNOWN, 0, 0, 0, 0);
    check_parse("start now",         CLI_ERR_ARGS,    0, 0, 0, 0);
    check_parse("stop 1",            CLI_ERR_ARGS,    0, 0, 0, 0);
    check_parse("status all",        CLI_ERR_ARGS,    0, 0, 0, 0);
    check_parse("map 12 W",          CLI_ERR_UNKNOWN, 0, 0, 0, 0);
    check_parse("f 0",               CLI_ERR_VALUE,   0, 0, 0, 0);
    check_parse("f 101",             CLI_ERR_VALUE,   0, 0, 0, 0);
    check_parse("f 30 0",            CLI_ERR_VALUE,   0, 0, 0, 0);
    check_parse("f 30 6000",         CLI_ERR_VALUE,   0, 0, 0, 0);
    check_parse("f 30 500 1",        CLI_ERR_ARGS,    0, 0, 0, 0);
    check_parse("drive 50",          CLI_ERR_ARGS,    0, 0, 0, 0);
    check_parse("drive 50 120",      CLI_ERR_VALUE,   0, 0, 0, 0);
    check_parse("claw 200",          CLI_ERR_VALUE,   0, 0, 0, 0);
    check_parse("claw",              CLI_ERR_ARGS,    0, 0, 0, 0);

    printf("\n== Line editor ==\n");
    check_editor("plain line",        "f 30 500\r",        CLI_FEED_LINE,     "f 30 500");
    check_editor("LF only",           "start\n",           CLI_FEED_LINE,     "start");
    check_editor("backspace fix",     "strat\b\b\bart\r", CLI_FEED_LINE,      "start");
    check_editor("DEL key",           "stopx\x7f\r",       CLI_FEED_LINE,     "stop");
    check_editor("control chars",     "st\x01op\r",        CLI_FEED_LINE,     "stop");
    check_editor("too long",
                 "drive 1111111111111111111111111111111111111111111111111\r",
                 CLI_FEED_TOO_LONG, NULL);

    /* Parser cost: worst-case valid command, averaged. */
    const int N = 200000;
    cmd_msg_t m;
    char reply[CLI_REPLY_MAX];
    clock_t t0 = clock();
    for (int i = 0; i < N; i++) cli_parse_line("drive -100 -100 5000", &m, reply, sizeof reply);
    double us = (double)(clock() - t0) / CLOCKS_PER_SEC * 1e6 / N;

    printf("\nParser cost on this PC: %.3f us per command (%d runs)\n", us, N);
    printf("RESULT: %d passed, %d failed\n", pass, fail);
    return fail ? 1 : 0;
}
