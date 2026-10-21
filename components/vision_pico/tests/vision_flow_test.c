/* Run the real Vision UART task and Coordinator against a small FIFO/device
 * simulator. This tests wiring, not actual kernel scheduling or hardware. */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../vision.c"
#include "../../state_machine/coordinator.c"
#include "../../../integration/main.c"

typedef struct {
    unsigned char messages[8][sizeof(VisionResult)];
    INT sizes[8], capacity, count, head;
} Queue;
static Queue queues[3];
static ID queue_count;
static jmp_buf task_exit;
static const char *rx;
static size_t rx_length, rx_position;
static char tx[128], console[4096];
static size_t tx_length, console_length;
static int coordinator_running, receive_timeouts;

ID tk_cre_mbf(const T_CMBF *definition)
{
    ID id = ++queue_count;
    assert(id < 3);
    queues[id].capacity = definition->bufsz / (definition->maxmsz + (INT)sizeof(UW));
    assert(queues[id].capacity > 0 && queues[id].capacity <= 8);
    return id;
}
ER tk_del_mbf(ID id) { (void)id; return E_OK; }
ER tk_snd_mbf(ID id, const void *data, INT size, TMO timeout)
{
    Queue *q = &queues[id];
    assert(timeout == TMO_POL);
    if (q->count == q->capacity) return E_TMOUT;
    INT index = (q->head + q->count++) % q->capacity;
    assert(size > 0 && size <= (INT)sizeof(VisionResult));
    memcpy(q->messages[index], data, (size_t)size);
    q->sizes[index] = size;
    return E_OK;
}
INT tk_rcv_mbf(ID id, void *data, TMO timeout)
{
    Queue *q = &queues[id];
    if (!q->count) {
        if (coordinator_running && timeout == 2000 && ++receive_timeouts > 1)
            longjmp(task_exit, 1);
        return E_TMOUT;
    }
    INT size = q->sizes[q->head];
    memcpy(data, q->messages[q->head], (size_t)size);
    q->head = (q->head + 1) % q->capacity;
    --q->count;
    return size;
}
ID tk_cre_tsk(const T_CTSK *definition)
{
    if (definition->task == vision_uart_task) {
        assert(definition->assprc == TP_PRC1);
        return 10;
    }
    assert(definition->assprc == TP_PRC2 && definition->task == coordinator_task);
    return 11;
}
ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
ER tk_del_tsk(ID id) { (void)id; return E_OK; }
ER tk_dly_tsk(INT delay)
{
    if (delay == 1 && rx_position == rx_length) longjmp(task_exit, 1);
    return E_OK;
}
ER tk_slp_tsk(TMO timeout) { (void)timeout; assert(0); return E_SYS; }
ID tk_opn_dev(const UB *name, INT mode)
{
    assert(strcmp((const char *)name, "sera") == 0);
    assert(mode == (TD_UPDATE | TD_EXCL));
    return 20;
}
ER tk_cls_dev(ID id, INT option) { (void)id; (void)option; return E_OK; }
ER tk_swri_dev(ID id, INT start, const void *data, SZ size, SZ *written)
{
    (void)id; (void)start;
    assert(size == 4 && tx_length + (size_t)size < sizeof(tx));
    memcpy(tx + tx_length, data, (size_t)size); tx_length += (size_t)size;
    *written = size; return E_OK;
}
ER tk_srea_dev(ID id, INT start, void *data, SZ size, SZ *available)
{
    (void)id; (void)start;
    if (size == 0) { *available = (SZ)(rx_length - rx_position); return E_OK; }
    /* Force 7-byte reads to split packets and checksums across calls. */
    if (size > 7) size = 7;
    if ((size_t)size > rx_length - rx_position) size = (SZ)(rx_length - rx_position);
    memcpy(data, rx + rx_position, (size_t)size); rx_position += (size_t)size;
    *available = size; return E_OK;
}
INT tm_printf(const UB *format, ...)
{
    va_list args;
    va_start(args, format);
    int count = vsnprintf(console + console_length, sizeof(console) - console_length,
                          (const char *)format, args);
    va_end(args);
    assert(count >= 0 && console_length + (size_t)count < sizeof(console));
    console_length += (size_t)count;
    return count;
}
static void run_uart(const char *data, size_t length)
{
    rx = data; rx_length = length; rx_position = 0;
    if (setjmp(task_exit) == 0) vision_uart_task(0, NULL);
}

int main(void)
{
    VisionResult result;
    assert(VisionReceive(&result, 1) == E_OBJ);
    assert(VisionReceive(NULL, 1) == E_PAR);
    assert(VisionStart() == E_OK && VisionStart() == E_OBJ);
    assert(CoordinatorStart() == E_OK);
    assert(VisionSetMode((VisionMode)99) == E_PAR);
    assert(VisionSetMode(VISION_MODE_COLOUR) == E_OK);

    const char *red = "C,R,160,120";
    unsigned checksum = 0;
    for (const char *p = red; *p; ++p) checksum ^= (unsigned char)*p;
    char input[256];
    snprintf(input, sizeof(input), "boot log\n%s*%02X\nC,N*21\nC,N*20\n", red, checksum);
    run_uart(input, strlen(input));
    assert(tx_length == 4 && memcmp(tx, "M,C\n", 4) == 0);
    coordinator_running = 1;
    if (setjmp(task_exit) == 0) coordinator_task(0, NULL);
    coordinator_running = 0;
    assert(strstr(console, "VISION_OK colour=RED x_px=160 y_px=120"));
    assert(strstr(console, "VISION_NO_OBJECT"));
    assert(strstr(console, "VISION_INVALID_PACKET"));
    assert(strstr(console, "VISION_TIMEOUT (not NO_OBJECT)"));
    assert(VisionReceive(&result, 123) == E_TMOUT && result.status == VISION_TIMEOUT);
    assert(result.x_px == -1 && result.y_px == -1);

    /* Overflow is discarded through newline and the next packet recovers. */
    memset(input, 'X', 150); input[150] = '\n';
    memcpy(input + 151, "C,N*21\n", 7);
    run_uart(input, 158);
    assert(VisionReceive(&result, 0) == E_OK && result.status == VISION_INVALID_PACKET);
    assert(VisionReceive(&result, 0) == E_OK && result.status == VISION_NO_OBJECT);

    const char *marker = "A,42,160,120,40.0,554.0";
    checksum = 0;
    for (const char *p = marker; *p; ++p) checksum ^= (unsigned char)*p;
    snprintf(input, sizeof(input), "%s*%02X\n", marker, checksum);
    run_uart(input, strlen(input));
    assert(VisionReceive(&result, 0) == E_OK && result.status == VISION_OK);
    assert(result.kind == VISION_ARUCO && result.marker_id == 42);
    assert(result.x_px == 160 && result.y_px == 120);
    assert(result.marker_width_px == 40.0f && result.distance_mm == 554.0f);

    /* A full result queue must not block the UART owner. */
    vision_clear_result(&result, VISION_NO_OBJECT);
    for (int i = 0; i < RESULT_DEPTH + 1; ++i) publish_result(&result);
    assert(queues[result_mbf].count == RESULT_DEPTH);
    assert(strstr(console, "result not queued"));
    puts(console);
    puts("PASS: UART chunking -> VisionResult -> FIFO -> real Coordinator; timeout/overflow/full queue");
    return 0;
}
