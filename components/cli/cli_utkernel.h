/*
 * cli_utkernel.h - User Interface Task (USB serial CLI) for µT-Kernel 3 SMP
 * Owner: Reiner
 *
 * Built for the course port (sirfonzie/mtk3smp-rp2040, CONSOLE=usb_cdc).
 * Wraps the RTOS-free parser in cli.c. Same pattern as imu_utkernel:
 * start-up code creates the kernel objects and passes their IDs in.
 *
 *   UI -> Coordinator : cmd_mbf, one cmd_msg_t per message,
 *                       then cmd_flag_bit set on coord_flg
 *   STOP              : stop_flag_bit set on coord_flg ONLY (not queued),
 *                       so the Coordinator can act on it before queued commands
 *   Coordinator -> UI : ui_mbf, variable-length text (max CLI_UI_LINE_MAX bytes)
 *
 * Rules: the UI Task is the only task that prints after boot; it never blocks
 * on a kernel object (every send/receive uses TMO_POL).
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

#define CLI_UI_LINE_MAX   96   /* max bytes per Coordinator -> UI message */
#define CLI_POLL_MS       10   /* input polling period */
#define CLI_STACK_MIN     4096 /* port guidance: SMP kernel calls need >= 4 KiB */

typedef struct {
    ID   cmd_mbf;        /* UI -> Coordinator (maxmsz >= sizeof(cmd_msg_t)) */
    ID   ui_mbf;         /* Coordinator -> UI (maxmsz >= CLI_UI_LINE_MAX)   */
    ID   coord_flg;      /* Coordinator's event flag                        */
    UINT cmd_flag_bit;   /* set after each queued command                   */
    UINT stop_flag_bit;  /* set for STOP (not queued)                       */
    PRI  priority;       /* 1 = HIGHEST. UI must be the lowest app task     */
    SZ   stack_bytes;    /* bytes, >= CLI_STACK_MIN                         */
} cli_task_config_t;

/* Create and start the UI Task, pinned to processor 1 (core 0), the core
   that owns the USB service task. Returns task ID or a negative error. */
ID cli_utkernel_start(const cli_task_config_t *config);

/* Worst-case acknowledgement time seen so far (µs), for NFR1 evidence. */
UW cli_utkernel_worst_ack_us(void);

#endif /* CLI_UTKERNEL_H */
