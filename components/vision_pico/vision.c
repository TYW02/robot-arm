#include <tk/device.h>
#include <tm/tmonitor.h>
#include "vision_task.h"
#include "vision_internal.h"

#define COMMAND_DEPTH 4
#define RESULT_DEPTH  8

/* A UW of message-buffer bookkeeping is needed for each queued message. */
static UW command_storage[COMMAND_DEPTH *
    ((sizeof(VisionMode) + sizeof(UW) - 1) / sizeof(UW) + 1)];
static UW result_storage[RESULT_DEPTH *
    ((sizeof(VisionResult) + sizeof(UW) - 1) / sizeof(UW) + 1)];
static ID uart_device, command_mbf, result_mbf, uart_task_id;
static BOOL started;

static void publish_result(const VisionResult *result)
{
    /* Never block UART reception behind a slow consumer. Drop NEW results if
     * full, reporting the loss; the Coordinator must keep draining this FIFO. */
    ER error = tk_snd_mbf(result_mbf, result, sizeof(*result), TMO_POL);
    if (error < E_OK) tm_printf((UB *)"[vision] result not queued: %d\n", error);
}

static void vision_uart_task(INT start_code, void *extended_info)
{
    char line[VISION_RX_LINE_SIZE], pending[32];
    SZ pending_size = 0, pending_index = 0;
    unsigned line_length = 0;
    BOOL overflow = FALSE;
    (void)start_code;
    (void)extended_info;

    while (1) {
        VisionMode mode;
        INT size = tk_rcv_mbf(command_mbf, &mode, TMO_POL);
        if (size == (INT)sizeof(mode)) {
            /* Existing camera command protocol; no checksum on commands. */
            UB command[4] = {'M', ',', 'I', '\n'};
            if (mode == VISION_MODE_SCAN) command[2] = 'S';
            if (mode == VISION_MODE_COLOUR) command[2] = 'C';
            SZ written = 0;
            ER error = tk_swri_dev(uart_device, 0, command, sizeof(command), &written);
            if (error < E_OK || written != sizeof(command))
                tm_printf((UB *)"[vision] mode write failed: %d\n", error);
        }

        /* Preserve the reviewed driver's nonblocking availability query and
         * 32-byte batching; partial lines survive across device reads. */
        if (pending_index >= pending_size) {
            SZ available = 0;
            ER error = tk_srea_dev(uart_device, 0, NULL, 0, &available);
            if (error < E_OK || available <= 0) {
                tk_dly_tsk(1);
                continue;
            }
            pending_size = available < (SZ)sizeof(pending) ? available : (SZ)sizeof(pending);
            pending_index = 0;
            error = tk_srea_dev(uart_device, 0, pending, pending_size, &pending_size);
            if (error < E_OK || pending_size <= 0) {
                pending_size = pending_index = 0;
                tk_dly_tsk(1);
                continue;
            }
        }

        while (pending_index < pending_size) {
            char ch = pending[pending_index++];
            if (ch == '\n') {
                VisionResult result;
                if (overflow) {
                    vision_clear_result(&result, VISION_INVALID_PACKET);
                    publish_result(&result);
                } else if (vision_parse_line(line, line_length, &result)) {
                    publish_result(&result);
                }
                line_length = 0;
                overflow = FALSE;
                break;  /* Service a queued mode command between packets. */
            } else if (line_length < sizeof(line) - 1) {
                line[line_length++] = ch;
            } else {
                overflow = TRUE; /* Discard through newline; never parse a prefix. */
            }
        }
    }
}

static T_CMBF command_definition = {
    .mbfatr = TA_TFIFO, .bufsz = sizeof(command_storage),
    .maxmsz = sizeof(VisionMode), .bufptr = command_storage
};
static T_CMBF result_definition = {
    .mbfatr = TA_TFIFO, .bufsz = sizeof(result_storage),
    .maxmsz = sizeof(VisionResult), .bufptr = result_storage
};
static T_CTSK uart_definition = {
    .itskpri = 5, .stksz = 4096, .task = vision_uart_task,
#if TK_SUPPORT_SMP
    /* Processor 1 = physical core 0, which owns the BSP UART interrupts. */
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1
#else
    .tskatr = TA_HLNG | TA_RNG3
#endif
};

ER VisionStart(void)
{
    ER error;
    if (started) return E_OBJ;
    uart_device = tk_opn_dev((const UB *)"sera", TD_UPDATE | TD_EXCL);
    if (uart_device < E_OK) return uart_device;
    command_mbf = tk_cre_mbf(&command_definition);
    if (command_mbf < E_OK) { error = command_mbf; goto close_uart; }
    result_mbf = tk_cre_mbf(&result_definition);
    if (result_mbf < E_OK) { error = result_mbf; goto delete_command; }
    uart_task_id = tk_cre_tsk(&uart_definition);
    if (uart_task_id < E_OK) { error = uart_task_id; goto delete_result; }
    error = tk_sta_tsk(uart_task_id, 0);
    if (error < E_OK) {
        tk_del_tsk(uart_task_id);
        goto delete_result;
    }
    started = TRUE;
    return E_OK;

delete_result:
    tk_del_mbf(result_mbf);
delete_command:
    tk_del_mbf(command_mbf);
close_uart:
    tk_cls_dev(uart_device, 0);
    return error;
}

ER VisionSetMode(VisionMode mode)
{
    if (!started) return E_OBJ;
    if (mode != VISION_MODE_SCAN && mode != VISION_MODE_COLOUR &&
        mode != VISION_MODE_IDLE) return E_PAR;
    return tk_snd_mbf(command_mbf, &mode, sizeof(mode), TMO_POL);
}

ER VisionReceive(VisionResult *result, TMO timeout)
{
    if (result == NULL) return E_PAR;
    vision_clear_result(result, VISION_INVALID_PACKET);
    if (!started) return E_OBJ;
    INT size = tk_rcv_mbf(result_mbf, result, timeout);
    if (size == (INT)sizeof(*result)) return E_OK;
    vision_clear_result(result, size == E_TMOUT ? VISION_TIMEOUT : VISION_INVALID_PACKET);
    return size < E_OK ? size : E_SYS;
}
