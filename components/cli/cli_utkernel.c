/*
 * cli_utkernel.c - User Interface Task (USB serial CLI) for µT-Kernel 3 SMP
 * Owner: Reiner
 *
 * cli.c only had its stdio (snprintf) dependency removed. Changes here
 * from the bare-metal / FreeRTOS versions:
 *   printf / putchar      -> tm_printf / tm_putchar / tm_putstring (no Pico SDK)
 *   getchar_timeout_us(0) -> tm_console_getc() (port patch, non-blocking)
 *   time_us_64()          -> RP2040 TIMERAWL register (1 MHz free-running)
 *   xTaskCreate           -> tk_cre_tsk + tk_sta_tsk, TA_ASSPRC / TP_PRC1
 *   vTaskDelay            -> tk_dly_tsk
 *   xQueueSend(q_cmd, 0)  -> tk_snd_mbf(cmd_mbf, ..., TMO_POL)
 *   xTaskNotify           -> tk_set_flg(coord_flg, bit)
 *   xQueueReceive(q_ui,0) -> tk_rcv_mbf(ui_mbf, ..., TMO_POL)
 */
#include "cli_utkernel.h"

#include <tm/tmonitor.h>
#include <string.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif
#include "usb_console_compat.h"

/* 1 = append "[ack N us]" to every reply and accept the test-only 'stats'. */
#ifndef CLI_MEASURE_ACK
#define CLI_MEASURE_ACK 1
#endif

#define TIMER_RAWL (*(volatile UW *)0x40054028)  /* µs, wraps every ~71 min */

static cli_task_config_t cfg;
static ID  cli_task_id  = 0;
static UW  worst_ack_us = 0;
static UW  n_dropped    = 0;

UW cli_utkernel_worst_ack_us(void) { return worst_ack_us; }

static void prompt(void) { tm_putstring((UB *)"> "); }

/* Print every pending Coordinator message. Never blocks. */
static void drain_ui_messages(void) {
    UB text[CLI_UI_LINE_MAX + 1];
    for (;;) {
        INT n = tk_rcv_mbf(cfg.ui_mbf, text, TMO_POL);
        if (n <= 0) break;                          /* empty */
        if (n > CLI_UI_LINE_MAX) n = CLI_UI_LINE_MAX;
        text[n] = '\0';
        tm_printf((UB *)"\n%s\n", text);
        prompt();
    }
}

/* Hand a validated command to the Coordinator. Returns FALSE if dropped. */
static BOOL post_command(const cmd_msg_t *m) {
    if (m->type == CMD_STOP)    /* not queued: Coordinator checks this bit first */
        return tk_set_flg(cfg.coord_flg, cfg.stop_flag_bit) == E_OK;

    if (tk_snd_mbf(cfg.cmd_mbf, m, (INT)sizeof *m, TMO_POL) != E_OK) {
        n_dropped++;
        return FALSE;                               /* full: never block */
    }
    (void)tk_set_flg(cfg.coord_flg, cfg.cmd_flag_bit);
    return TRUE;
}

static void handle_line(const char *line, UW t_enter) {
#if CLI_MEASURE_ACK
    if (strcmp(line, "stats") == 0) {               /* test-only */
        tm_printf((UB *)"worst_ack=%u us  busy_drops=%u  usb_rx_drops=%u\n",
                  worst_ack_us, n_dropped, tm_usb_rx_dropped_bytes());
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
    if (r == CLI_OK && !post_command(&m))
        strcpy(reply, "busy: command dropped, try again");

#if CLI_MEASURE_ACK
    UW ack = TIMER_RAWL - t_enter;                  /* unsigned: wrap-safe */
    if (ack > worst_ack_us) worst_ack_us = ack;
    tm_printf((UB *)"%s  [ack %u us]\n", reply, ack);
#else
    (void)t_enter;
    tm_printf((UB *)"%s\n", reply);
#endif
}

static void task_entry(INT stacd, void *exinf) {
    (void)stacd;
    (void)exinf;

    /* Output before the host enumerates is lost, so wait (max ~20 s). */
    for (INT i = 0; i < 200 && tm_usb_state() < 2; i++) tk_dly_tsk(100);
    tk_dly_tsk(300);

#if TK_SUPPORT_SMP
    tm_printf((UB *)"\nRobot arm CLI ready on prc%d. Type 'help'.\n", tk_get_prc());
#else
    tm_printf((UB *)"\nRobot arm CLI ready. Type 'help'.\n");
#endif
    prompt();

    cli_line_t line;
    cli_line_reset(&line);

    for (;;) {
        drain_ui_messages();

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
    if (!c || c->cmd_mbf <= 0 || c->ui_mbf <= 0 || c->coord_flg <= 0 ||
        c->cmd_flag_bit == 0 || c->stop_flag_bit == 0 ||
        (c->cmd_flag_bit & c->stop_flag_bit) != 0 ||
        c->priority <= 0 || c->stack_bytes < CLI_STACK_MIN || cli_task_id > 0) {
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
