/*
 * cli_utkernel.c - User Interface Task (USB serial CLI) for µT-Kernel 3 SMP
 * Owner: Reiner
 *
 *   input   : tm_console_getc() (port patch, non-blocking USB read)
 *   output  : tm_printf / tm_putchar / tm_putstring (thread-safe on SMP)
 *   timing  : RP2040 TIMERAWL (1 MHz free-running) for the ack measurement
 *   task    : tk_cre_tsk + tk_sta_tsk, pinned with TA_ASSPRC / TP_PRC1
 *   commands: handed to config.execute() (integration/cli_bridge.c)
 */
#include "cli_utkernel.h"

#include <tm/tmonitor.h>
#include <string.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif
#ifndef USB_CONSOLE_COMPAT_H              /* already included by cli_task_app.c */
#include "usb_console_compat.h"
#endif

/* 1 = append "[ack N us]" to every reply and accept the test-only 'stats'. */
#ifndef CLI_MEASURE_ACK
#define CLI_MEASURE_ACK 1
#endif

#define TIMER_RAWL (*(volatile UW *)0x40054028)  /* µs, wraps every ~71 min */

static cli_task_config_t cfg;
static ID  cli_task_id  = 0;
static UW  worst_ack_us = 0;

UW cli_utkernel_worst_ack_us(void) { return worst_ack_us; }

static void prompt(void) { tm_putstring((UB *)"> "); }

static void handle_line(const char *line, UW t_enter) {
#if CLI_MEASURE_ACK
    if (strcmp(line, "stats") == 0) {               /* test-only */
        tm_printf((UB *)"worst_ack=%d us  usb_rx_drops=%d\n",
                  (INT)worst_ack_us, (INT)tm_usb_rx_dropped_bytes());
        return;
    }
#endif
    cmd_msg_t m;
    char reply[CLI_REPLY_MAX];
    cli_result_t r = cli_parse_line(line, &m, reply, sizeof reply);

    if (r == CLI_EMPTY) return;
    if (r == CLI_HELP) {
        tm_putstring((UB *)cli_help_text());
        tm_putstring((UB *)"\n");
        return;
    }
    if (r == CLI_OK && cfg.execute)
        cfg.execute(&m, reply, sizeof reply);       /* may rewrite reply */

    if (reply[0] == '\0') return;                   /* status printed its own lines */

#if CLI_MEASURE_ACK
    UW ack = TIMER_RAWL - t_enter;                  /* unsigned: wrap-safe */
    if (ack > worst_ack_us) worst_ack_us = ack;
    tm_printf((UB *)"%s  [ack %d us]\n", reply, (INT)ack);
#else
    (void)t_enter;
    tm_printf((UB *)"%s\n", reply);
#endif
}


/* Copy text into out (truncating) and return the length. */
static int put_text(char *out, unsigned int out_len, const char *text) {
    unsigned int n = 0;
    while (text[n] != '\0' && n + 1U < out_len) { out[n] = text[n]; n++; }
    out[n] = '\0';
    return (int)n;
}

int cli_utkernel_exec_line(const char *line, char *out, unsigned int out_len) {
    cmd_msg_t m;
    char reply[CLI_REPLY_MAX];
    cli_result_t r;

    if (!out || out_len == 0U) return 0;
    out[0] = '\0';

    if (!cfg.execute)                      /* UI task not started yet */
        return put_text(out, out_len, "rejected: not ready yet (IMU start-up)");
#if CLI_MEASURE_ACK
    if (strcmp(line, "stats") == 0) {
        char t[64];
        tm_sprintf((UB *)t, (UB *)"worst_ack=%d us  usb_rx_drops=%d",
                   (INT)worst_ack_us, (INT)tm_usb_rx_dropped_bytes());
        return put_text(out, out_len, t);
    }
#endif
    r = cli_parse_line(line, &m, reply, sizeof reply);
    if (r == CLI_EMPTY) return 0;
    if (r == CLI_HELP)  return put_text(out, out_len, cli_help_text());

    if (r == CLI_OK) {
        cfg.execute(&m, reply, sizeof reply);          /* may rewrite reply */
        if (reply[0] == '\0' && m.type == CMD_STATUS && cfg.status_text) {
            cfg.status_text(out, out_len);
            return (int)strlen(out);
        }
    }
    return put_text(out, out_len, reply);
}

static void task_entry(INT stacd, void *exinf) {
    (void)stacd;
    (void)exinf;

    /* Output before the host enumerates is lost, so wait (max ~20 s). */
    for (INT i = 0; i < 200 && tm_usb_state() < 2; i++) tk_dly_tsk(100);
    tk_dly_tsk(300);

#if TK_SUPPORT_SMP
    tm_printf((UB *)"\nCLI ready on prc%d. Type 'help'.\n", tk_get_prc());
#else
    tm_printf((UB *)"\nCLI ready. Type 'help'.\n");
#endif
    prompt();

    cli_line_t line;
    cli_line_reset(&line);

    for (;;) {
        INT ch;
        while ((ch = tm_console_getc()) >= 0) {
            switch (cli_line_feed(&line, ch)) {
            case CLI_FEED_ECHO:      tm_putchar(ch); break;
            case CLI_FEED_BACKSPACE: tm_putstring((UB *)"\b \b"); break;
            case CLI_FEED_LINE: {
                UW t_enter = TIMER_RAWL;
                tm_putstring((UB *)"\n");
                handle_line(line.buf, t_enter);
                cli_line_reset(&line);
                prompt();
                break;
            }
            case CLI_FEED_TOO_LONG:
                tm_printf((UB *)"\nerror: line too long (max %d chars)\n", CLI_LINE_MAX);
                cli_line_reset(&line);
                prompt();
                break;
            default: break;
            }
        }
        (void)tk_dly_tsk(CLI_POLL_MS);
    }
}

ID cli_utkernel_start(const cli_task_config_t *c) {
    if (!c || !c->execute || c->priority <= 0 ||
        c->stack_bytes < CLI_STACK_MIN || cli_task_id > 0) {
        return E_PAR;
    }
    cfg = *c;

    T_CTSK t;
    memset(&t, 0, sizeof t);
    t.task    = (FP)task_entry;
    t.itskpri = cfg.priority;
    t.stksz   = cfg.stack_bytes;
#if TK_SUPPORT_SMP
    /* Same processor as the USB service task, which owns the USB interrupt. */
    t.tskatr  = TA_HLNG | TA_RNG3 | TA_ASSPRC;
    t.assprc  = TP_PRC1;
#else
    t.tskatr  = TA_HLNG | TA_RNG3;
#endif

    ID id = tk_cre_tsk(&t);
    if (id <= 0) return id;

    ER err = tk_sta_tsk(id, 0);
    if (err != E_OK) { (void)tk_del_tsk(id); return err; }

    cli_task_id = id;
    return id;
}
