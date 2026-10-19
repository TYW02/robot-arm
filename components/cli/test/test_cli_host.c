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

static void check_parse(const char *line, cli_result_t want_res,
                        cmd_type_t want_type, char want_colour, int want_value) {
    cmd_msg_t m = { (cmd_type_t)-1, 0, -1 };
    char reply[CLI_REPLY_MAX];
    cli_result_t r = cli_parse_line(line, &m, reply, sizeof reply);

    int ok = (r == want_res);
    if (ok && r == CLI_OK)
        ok = (m.type == want_type && m.colour == want_colour && m.value == want_value);

    printf("%s  %-28s -> %s\n", ok ? "PASS" : "FAIL", line, reply[0] ? reply : "(no reply)");
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
    printf("%s  editor: %-21s -> \"%s\"\n", ok ? "PASS" : "FAIL", name, l.buf);
    ok ? pass++ : fail++;
}

int main(void) {
    printf("== Valid commands ==\n");
    check_parse("pick R",            CLI_OK, CMD_PICK,      'R', 0);
    check_parse("pick g",            CLI_OK, CMD_PICK,      'G', 0);
    check_parse("PICK blue",         CLI_OK, CMD_PICK,      'B', 0);
    check_parse("map R 12",          CLI_OK, CMD_MAP,       'R', 12);
    check_parse("map green 0",       CLI_OK, CMD_MAP,       'G', 0);
    check_parse("map B 999",         CLI_OK, CMD_MAP,       'B', 999);
    check_parse("  map   r   7  ",   CLI_OK, CMD_MAP,       'R', 7);
    check_parse("home",              CLI_OK, CMD_HOME,      0,   0);
    check_parse("stop",              CLI_OK, CMD_STOP,      0,   0);
    check_parse("reset",             CLI_OK, CMD_RESET,     0,   0);
    check_parse("cal",               CLI_OK, CMD_RECAL_IMU, 0,   0);
    check_parse("Status",            CLI_OK, CMD_STATUS,    0,   0);

    printf("\n== Invalid / boundary commands (must be rejected) ==\n");
    check_parse("",                  CLI_EMPTY,        0, 0, 0);
    check_parse("   ",               CLI_EMPTY,        0, 0, 0);
    check_parse("help",              CLI_HELP,         0, 0, 0);
    check_parse("fly",               CLI_ERR_UNKNOWN,  0, 0, 0);
    check_parse("pick",              CLI_ERR_ARGS,     0, 0, 0);
    check_parse("pick R G",          CLI_ERR_ARGS,     0, 0, 0);
    check_parse("pick yellow",       CLI_ERR_COLOUR,   0, 0, 0);
    check_parse("pick 1",            CLI_ERR_COLOUR,   0, 0, 0);
    check_parse("map R",             CLI_ERR_ARGS,     0, 0, 0);
    check_parse("map X 5",           CLI_ERR_COLOUR,   0, 0, 0);
    check_parse("map R -1",          CLI_ERR_ID,       0, 0, 0);
    check_parse("map R 1000",        CLI_ERR_ID,       0, 0, 0);
    check_parse("map R 12abc",       CLI_ERR_ID,       0, 0, 0);
    check_parse("map R 99999999999", CLI_ERR_ID,       0, 0, 0);
    check_parse("home now",          CLI_ERR_ARGS,     0, 0, 0);
    check_parse("map R 1 2 3",       CLI_ERR_ARGS,     0, 0, 0);

    printf("\n== Line editor ==\n");
    check_editor("plain line",        "pick R\r",          CLI_FEED_LINE,     "pick R");
    check_editor("LF only",           "home\n",            CLI_FEED_LINE,     "home");
    check_editor("backspace fix",     "pikc\b\bck R\r",    CLI_FEED_LINE,     "pick R");
    check_editor("DEL key",           "stopx\x7f\r",       CLI_FEED_LINE,     "stop");
    check_editor("control chars",     "ho\x01me\r",        CLI_FEED_LINE,     "home");
    check_editor("too long",
                 "pick RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR\r",
                 CLI_FEED_TOO_LONG, NULL);

    /* Parser cost: worst-case valid command, averaged. On the Pico this is
       measured again by cli_test.c with time_us_64(). */
    const int N = 200000;
    cmd_msg_t m;
    char reply[CLI_REPLY_MAX];
    clock_t t0 = clock();
    for (int i = 0; i < N; i++) cli_parse_line("map green 999", &m, reply, sizeof reply);
    double us = (double)(clock() - t0) / CLOCKS_PER_SEC * 1e6 / N;

    printf("\nParser cost on this PC: %.3f us per command (%d runs)\n", us, N);
    printf("RESULT: %d passed, %d failed\n", pass, fail);
    return fail ? 1 : 0;
}
