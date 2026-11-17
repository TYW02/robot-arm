/*
 * cli_utkernel.h - User Interface Task (USB serial CLI) for µT-Kernel 3 SMP
 * Owner: Reiner
 *
 * Built for the course port (third_party/mtk3smp-rp2040, CONSOLE=usb_cdc).
 * Wraps the RTOS-free parser in cli.c:
 *
 *   USB input -> UI Task (core 0) -> cli_parse_line() -> config.execute()
 *
 * execute() is supplied by the integration layer (integration/cli_bridge.c).
 * It applies the command (latch STOP, signal START, run a debug move...)
 * and may overwrite the reply, e.g. "rejected: mission running".
 *
 * Rules: the UI Task never blocks on a kernel object, and execute() must
 * return quickly, so STOP is acknowledged within the 200 ms target (NFR1).
 * Needs the port patch that adds tm_console_getc() (non-blocking USB input).
 */
#ifndef CLI_UTKERNEL_H
#define CLI_UTKERNEL_H

/* cli.h uses the standard size_t (<stddef.h>); stop the kernel's syslib.h
   redefining it as SZ (long). Include this header before <tk/tkernel.h>. */
#ifndef PROHIBIT_DEF_SIZE_T
#define PROHIBIT_DEF_SIZE_T
#endif
#include <stddef.h>
#include <tk/tkernel.h>
#include "cli.h"

#define CLI_POLL_MS       10   /* input polling period */
#define CLI_STACK_MIN     4096 /* port guidance: SMP kernel calls need >= 4 KiB */

typedef struct {
    /* Apply one valid command. Runs in the UI Task; must not wait long. */
    void (*execute)(const cmd_msg_t *cmd, char *reply, size_t reply_len);
    PRI  priority;       /* 1 = highest                                    */
    SZ   stack_bytes;    /* bytes, >= CLI_STACK_MIN                         */
} cli_task_config_t;

/* Create and start the UI Task, pinned to processor 1 (core 0), the core
   that owns the USB service task. Returns task ID or a negative error. */
ID cli_utkernel_start(const cli_task_config_t *config);

/* Worst-case acknowledgement time seen so far (µs), for NFR1 evidence. */
UW cli_utkernel_worst_ack_us(void);

#endif /* CLI_UTKERNEL_H */
