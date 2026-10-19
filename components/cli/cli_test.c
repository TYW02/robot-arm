#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "cli.h"

static uint64_t worst_ack_us = 0;
static uint32_t n_ok = 0, n_rejected = 0;
static bool     load_on = false;

static void prompt(void) { printf("> "); fflush(stdout); }

static void print_cmd(const cmd_msg_t *m) {
    printf("   -> q_cmd: { type=%s, colour=%c, value=%d }\n",
           cli_cmd_name(m->type), m->colour ? m->colour : '-', m->value);
}

/* Test-only commands handled here, not part of the real CLI. */
static bool handle_test_cmd(const char *line) {
    if (strcmp(line, "stats") == 0) {
        printf("ok=%lu rejected=%lu worst_ack=%llu us (NFR1 limit 200000 us)\n",
               (unsigned long)n_ok, (unsigned long)n_rejected,
               (unsigned long long)worst_ack_us);
        return true;
    }
    if (strcmp(line, "load on") == 0)  { load_on = true;  printf("simulated load ON (5 ms busy per loop)\n");  return true; }
    if (strcmp(line, "load off") == 0) { load_on = false; printf("simulated load OFF\n"); return true; }
    return false;
}

static void handle_line(const char *line, uint64_t t_enter) {
    if (handle_test_cmd(line)) return;

    cmd_msg_t m;
    char reply[CLI_REPLY_MAX];
    cli_result_t r = cli_parse_line(line, &m, reply, sizeof reply);

    if (r == CLI_EMPTY) return;
    if (r == CLI_HELP) { printf("%s\n", cli_help_text()); return; }

    printf("%s", reply);
    uint64_t ack = time_us_64() - t_enter;
    printf("  [ack %llu us]\n", (unsigned long long)ack);
    if (ack > worst_ack_us) worst_ack_us = ack;

    if (r == CLI_OK) { n_ok++; print_cmd(&m); }
    else             { n_rejected++; }
}

int main(void) {
    stdio_init_all();

    /* Wait up to 5 s for the terminal so the banner is not lost (USB CDC). */
    for (int i = 0; i < 50 && !stdio_usb_connected(); i++) sleep_ms(100);

    printf("\n=== CLI standalone test (feature/cli) ===\n");
    printf("Type 'help'. Test-only: 'stats', 'load on', 'load off'.\n");
    prompt();

    cli_line_t line;
    cli_line_reset(&line);

    for (;;) {
        int ch;
        /* Non-blocking: never wait for input, so other work keeps running. */
        while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            switch (cli_line_feed(&line, ch)) {
            case CLI_FEED_ECHO:      putchar(ch); break;
            case CLI_FEED_BACKSPACE: printf("\b \b"); break;
            case CLI_FEED_LINE: {
                uint64_t t_enter = time_us_64();
                printf("\n");
                handle_line(line.buf, t_enter);
                cli_line_reset(&line);
                prompt();
                break;
            }
            case CLI_FEED_TOO_LONG:
                printf("\nerror: line too long (max %d chars)\n", CLI_LINE_MAX);
                n_rejected++;
                cli_line_reset(&line);
                prompt();
                break;
            default: break;
            }
        }
        fflush(stdout);

        if (load_on) busy_wait_us(5000);   /* stand-in for other tasks' work */
        sleep_ms(1);
    }
}
