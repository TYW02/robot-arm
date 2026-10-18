// main.c - SYSTEM INTEGRATION (single file).  Replaces integration/main.c.
//
// Integrates: components/imu/imu.c (unchanged, linked from CMake) + the XIAO vision UART protocol
// (xiao_vision/main/main.c) into one FreeRTOS application on the Pico W.
//   Tasks : Coordinator, Vision Comm, IMU, Motion, UI (USB CLI)
//   IPC   : queues, task notifications, fault event group, I2C mutex  (rules in the IPC section)
// The arm is SIMULATED in the "SIMULATED ARM" section until the servo owner implements arm_hal_*.


/* ============================================================================
 * CONFIGURATION (pins, timeouts, tuning)
 * ============================================================================ */
// system_config.h - single place for every pin, timeout and tuning value.
// Change numbers HERE (and in the report) - never inside a task.

/* ---------- Shared I2C bus (PCA9685 + IMU) - report: GP2 SDA / GP3 SCL ---------- */
#define I2C_PORT                i2c1
#define PIN_I2C_SDA             2
#define PIN_I2C_SCL             3
#define I2C_BAUD_HZ             100000      /* imu_test used 100 kHz; PCA9685 can go 400 kHz */
#define I2C_MUTEX_TIMEOUT_MS    20          /* max wait for the bus; on expiry -> counted as bus fault */

/* ---------- Vision UART0 - report: GP0 TX / GP1 RX, 115200 8N1 ---------- */
#define VISION_UART             uart0
#define VISION_UART_IRQ         UART0_IRQ
#define PIN_VISION_TX           0
#define PIN_VISION_RX           1
#define VISION_BAUD             115200
#define VISION_FRAME_W          320         /* XIAO runs QVGA */
#define VISION_FRAME_H          240
#define VISION_MAX_MARKER_ID    999         /* ARUCO_DB_4X4_1000 */
#define VISION_BAD_LINE_LIMIT   10          /* consecutive malformed lines -> vision comm fault */

/* ---------- Timeouts (FR8 / NFR3) ---------- */
#define VISION_OBJECT_TIMEOUT_MS   2500     /* colour mode re-sends at least every ~1 s, silence = link down */
#define VISION_OBJECT_RETRIES      1
#define VISION_SHELF_TIMEOUT_MS    5000
#define VISION_SHELF_RETRIES       1
#define MOTION_PICK_TIMEOUT_MS     20000
#define MOTION_PLACE_TIMEOUT_MS    20000
#define MOTION_HOME_TIMEOUT_MS     15000

/* ---------- IMU ---------- */
#define IMU_PERIOD_MS           20          /* NFR2: samples no more than 50 ms apart */
#define IMU_TOLERANCE_DEG       5.0f
#define IMU_FAIL_LIMIT          3           /* consecutive failed samples -> IMU fault */
/* Report decision pending (Option A/B). 1 = out-of-tolerance while carrying is a FAULT. */
#define IMU_OOT_IS_FAULT        1

/* ---------- Default shelf mapping: object colour -> ArUco shelf marker id ---------- */
/* ASSUMPTION - replace with the real shelf marker ids (or use the CLI: map R 12). */
#define DEFAULT_SHELF_RED       1
#define DEFAULT_SHELF_GREEN     2
#define DEFAULT_SHELF_BLUE      3

/* ---------- RTOS sizing ---------- */
#define Q_CMD_LEN               4
#define Q_VISION_LEN            4
#define Q_MOTION_LEN            2
#define Q_UI_LEN                16
#define Q_VISION_CMD_LEN        4
#define VISION_RX_STREAM_BYTES  256

#define PRIO_IDLE_UI            1
#define PRIO_MOTION             2
#define PRIO_COORDINATOR        3
#define PRIO_VISION             3
#define PRIO_IMU                4           /* hard-ish 20 ms period -> highest */

#define STACK_COORDINATOR       1024        /* words */
#define STACK_VISION            768
#define STACK_IMU               512
#define STACK_MOTION            512
#define STACK_UI                768


/* ============================================================================
 * SHARED MESSAGE TYPES
 * ============================================================================ */
// system_types.h - message formats shared by all tasks (the "interface contract").
#include <stdint.h>
#include <stdbool.h>

/* ---- CLI/UI task -> Coordinator (q_cmd) ---- */
typedef enum {
    CMD_PICK,       /* colour = 'R' | 'G' | 'B'                */
    CMD_MAP,        /* colour, value = shelf marker id          */
    CMD_HOME,
    CMD_STOP,
    CMD_RESET,      /* clear FAULT                              */
    CMD_STATUS,
    CMD_RECAL_IMU,  /* re-capture level reference               */
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    char       colour;
    int        value;
} cmd_msg_t;

/* ---- Vision task -> Coordinator (q_vision) ---- */
typedef enum {
    VIS_OBJECT,     /* "C,<R|G|B>,x,y"                          */
    VIS_NO_OBJECT,  /* "C,N"                                    */
    VIS_MARKER,     /* "A,id,x,y,w,dist"                        */
} vision_kind_t;

typedef struct {
    vision_kind_t kind;
    char          colour;       /* VIS_OBJECT */
    int           id;           /* VIS_MARKER */
    int           x, y;         /* pixel centroid */
    int           width_px;     /* VIS_MARKER */
    float         dist_mm;      /* VIS_MARKER */
} vision_msg_t;

/* ---- Coordinator -> Vision task (q_vision_cmd): one char: 'S' scan, 'C' colour, 'I' idle ---- */

/* ---- Coordinator -> Motion task (q_motion) ---- */
typedef enum { MOT_HOME, MOT_PICK, MOT_PLACE } motion_type_t;

typedef struct {
    motion_type_t type;
    int           x, y;         /* pixel position (object or shelf marker) */
    float         dist_mm;      /* MOT_PLACE: marker distance */
    uint32_t      gen;          /* motion_generation() at send time; stale => aborted */
} motion_cmd_t;

/* ---- Coordinator -> UI task (q_ui) ---- */
#define UI_LINE_MAX 96
typedef struct { char text[UI_LINE_MAX]; } ui_msg_t;

/* ---- Task notification bits delivered to the Coordinator ---- */
#define NOTIF_CMD          (1u << 0)   /* q_cmd has data      */
#define NOTIF_VISION       (1u << 1)   /* q_vision has data   */
#define NOTIF_MOTION_DONE  (1u << 2)   /* no data             */
#define NOTIF_MOTION_FAIL  (1u << 3)   /* no data             */
#define NOTIF_FAULT        (1u << 4)   /* evg_fault changed   */

/* ---- Fault event group bits (set by any task, cleared only by Coordinator on RESET) ---- */
#define FAULT_VISION_COMM  (1u << 0)
#define FAULT_IMU          (1u << 1)
#define FAULT_MOTION       (1u << 2)
#define FAULT_I2C_BUS      (1u << 3)
#define FAULT_ALL          (FAULT_VISION_COMM | FAULT_IMU | FAULT_MOTION | FAULT_I2C_BUS)


/* ============================================================================
 * IPC OBJECTS (queues, event group, mutex)
 * ============================================================================ */
// ipc.h - every queue / event group / mutex / task handle lives here.
//
// RULES (copy into report section 4):
//  Q1. Every queue has exactly ONE producer task and ONE consumer task (see table below).
//  Q2. Producers never block on a full queue (timeout 0): drop-oldest for q_vision, drop for q_ui.
//  M1. The I2C mutex guards the shared bus. Only the code that touches the bus takes it.
//  M2. Take it with a timeout (I2C_MUTEX_TIMEOUT_MS), never hold it across any other blocking call,
//      never take another lock while holding it.  => no deadlock, bounded priority inversion.
//  N1. The Coordinator is woken ONLY by task notifications (NOTIF_* bits); data is read from queues.
//  E1. evg_fault bits are SET by the detecting task, CLEARED only by the Coordinator on RESET.
//
//   queue         producer      consumer      notification to coordinator
//   q_cmd         UI/CLI        Coordinator   NOTIF_CMD
//   q_vision      Vision        Coordinator   NOTIF_VISION
//   q_vision_cmd  Coordinator   Vision        -
//   q_motion      Coordinator   Motion        -
//   q_ui          Coordinator   UI/CLI        -   (other tasks never print; they raise faults)
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "stream_buffer.h"

extern QueueHandle_t      q_cmd, q_vision, q_vision_cmd, q_motion, q_ui;
extern EventGroupHandle_t evg_fault;
extern SemaphoreHandle_t  i2c_mutex;
extern TaskHandle_t       h_coordinator;

bool ipc_init(void);

/* printf-style message to the operator via q_ui (non-blocking, drops if full) */
void ui_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* raise a fault from any task (ISR-unsafe) and wake the coordinator */
void fault_raise(uint32_t bits);


/* ============================================================================
 * TASK / MODULE DECLARATIONS
 * ============================================================================ */
#include <stdbool.h>
#include <stdint.h>
#include "imu.h"

/* task entry points / init hooks */
void coordinator_task(void *arg);
void vision_task(void *arg);
void imu_task(void *arg);
void motion_task(void *arg);
void ui_task(void *arg);

void vision_hw_init(void);                     /* UART + RX ISR, before scheduler */
bool imu_hw_init(void);                        /* before scheduler; false = sensor not found */

/* IMU snapshot written by imu_task, read by coordinator (copy under critical section) */
typedef struct {
    bool         valid;
    imu_status_t status;
    float        pitch, roll;
    float        target_pitch, target_roll;
} imu_snapshot_t;
void imu_snapshot_get(imu_snapshot_t *out);
void imu_request_recapture(void);              /* take current attitude as new level reference */

/* motion task state, read by coordinator */
void motion_abort(void);                       /* ask the arm to stop at its current position */
bool motion_busy(void);
uint32_t motion_generation(void);              /* stamp into every motion_cmd_t */


/* ---- prototypes for modules below ---- */
typedef enum { VP_OK = 0, VP_EMPTY, VP_BAD_FORMAT, VP_OUT_OF_RANGE } vp_result_t;
vp_result_t vision_parse_line(const char *line, vision_msg_t *out);
void shelf_map_init(void);
bool shelf_map_set(char colour, int marker_id);
int  shelf_map_get(char colour);
bool colour_valid(char colour);
void i2c_bus_init(void);
bool i2c_bus_lock(void);
void i2c_bus_unlock(void);


/* ============================================================================
 * ARM HAL CONTRACT (servo owner implements this)
 * ============================================================================ */
// arm_hal.h - CONTRACT between the integration layer and the servo owner (components/servo is empty).
//
// The servo owner implements these functions (e.g. in components/servo/arm_hal_pca9685.c).
// Rules for the implementation:
//   * Each call BLOCKS until the motion finishes (or fails) and returns true on success.
//   * Any PCA9685 I2C access must be wrapped in i2c_bus_lock()/i2c_bus_unlock().
//   * Do not hold the lock while waiting for a servo to move (lock only around the I2C write).
//   * Check arm_hal_abort_requested() between small steps and return false quickly when set.
//   * Pixel -> joint-angle conversion (kinematics/calibration) lives here, not in the coordinator.
// Until then src/arm_hal_stub.c simulates the arm with delays so the whole cycle can be tested.
#include <stdbool.h>

bool arm_hal_init(void);
bool arm_hal_home(void);
bool arm_hal_approach_object(int px, int py);          /* move gripper over object (vision pixels) */
bool arm_hal_grip(bool close);
bool arm_hal_lift(void);                               /* raise object to carry pose */
bool arm_hal_move_to_shelf(int px, int py, float dist_mm);
bool arm_hal_retreat(void);

/* provided by motion_task.c; implementations poll it */
bool arm_hal_abort_requested(void);


/* ============================================================================
 * IPC IMPLEMENTATION
 * ============================================================================ */
#include <stdio.h>
#include <stdarg.h>

QueueHandle_t      q_cmd, q_vision, q_vision_cmd, q_motion, q_ui;
EventGroupHandle_t evg_fault;
SemaphoreHandle_t  i2c_mutex;
TaskHandle_t       h_coordinator;

bool ipc_init(void) {
    q_cmd        = xQueueCreate(Q_CMD_LEN,        sizeof(cmd_msg_t));
    q_vision     = xQueueCreate(Q_VISION_LEN,     sizeof(vision_msg_t));
    q_vision_cmd = xQueueCreate(Q_VISION_CMD_LEN, sizeof(char));
    q_motion     = xQueueCreate(Q_MOTION_LEN,     sizeof(motion_cmd_t));
    q_ui         = xQueueCreate(Q_UI_LEN,         sizeof(ui_msg_t));
    evg_fault    = xEventGroupCreate();
    i2c_mutex    = xSemaphoreCreateMutex();      /* mutex => priority inheritance */
    return q_cmd && q_vision && q_vision_cmd && q_motion && q_ui && evg_fault && i2c_mutex;
}

void ui_printf(const char *fmt, ...) {
    ui_msg_t m;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(m.text, sizeof m.text, fmt, ap);
    va_end(ap);
    (void)xQueueSend(q_ui, &m, 0);               /* drop when full: never block a control task */
}

void fault_raise(uint32_t bits) {
    xEventGroupSetBits(evg_fault, bits);
    if (h_coordinator) xTaskNotify(h_coordinator, NOTIF_FAULT, eSetBits);
}


/* ============================================================================
 * I2C BUS + MUTEX
 * ============================================================================ */
#include "pico/stdlib.h"

void i2c_bus_init(void) {
    i2c_init(I2C_PORT, I2C_BAUD_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);               /* internal pull-ups are weak: external 4.7k recommended */
    gpio_pull_up(PIN_I2C_SCL);
}

bool i2c_bus_lock(void) {
    return xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE;
}

void i2c_bus_unlock(void) {
    xSemaphoreGive(i2c_mutex);
}


/* ============================================================================
 * VISION PROTOCOL PARSER
 * ============================================================================ */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#define MAX_FIELDS 8

static bool parse_int(const char *s, long lo, long hi, int *out, bool *range_err) {
    if (!s || !*s) return false;
    char *end;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0' || errno) return false;
    if (v < lo || v > hi) { *range_err = true; return false; }
    *out = (int)v;
    return true;
}

static bool parse_float(const char *s, float lo, float hi, float *out, bool *range_err) {
    if (!s || !*s) return false;
    char *end;
    errno = 0;
    float v = strtof(s, &end);
    if (end == s || *end != '\0' || errno || !isfinite(v)) return false;
    if (v < lo || v > hi) { *range_err = true; return false; }
    *out = v;
    return true;
}

vp_result_t vision_parse_line(const char *line, vision_msg_t *out) {
    char buf[96];
    size_t n = strlen(line);
    while (n && (line[n - 1] == '\r' || line[n - 1] == ' ')) n--;
    if (n == 0) return VP_EMPTY;
    if (n >= sizeof buf) return VP_BAD_FORMAT;
    memcpy(buf, line, n);
    buf[n] = '\0';

    char *f[MAX_FIELDS];
    int nf = 0;
    for (char *p = buf; nf < MAX_FIELDS;) {
        f[nf++] = p;
        char *c = strchr(p, ',');
        if (!c) break;
        *c = '\0';
        p = c + 1;
        if (nf == MAX_FIELDS && strchr(p, ',')) return VP_BAD_FORMAT;
    }

    vision_msg_t m = {0};
    bool rng = false;

    if (strcmp(f[0], "A") == 0) {                       /* A,id,x,y,w,dist */
        if (nf != 6) return VP_BAD_FORMAT;
        bool ok = parse_int(f[1], 0, VISION_MAX_MARKER_ID, &m.id, &rng) &&
                  parse_int(f[2], 0, VISION_FRAME_W, &m.x, &rng) &&
                  parse_int(f[3], 0, VISION_FRAME_H, &m.y, &rng) &&
                  parse_int(f[4], 1, VISION_FRAME_W, &m.width_px, &rng) &&
                  parse_float(f[5], 10.0f, 3000.0f, &m.dist_mm, &rng);
        if (!ok) return rng ? VP_OUT_OF_RANGE : VP_BAD_FORMAT;
        m.kind = VIS_MARKER;
    } else if (strcmp(f[0], "C") == 0) {                /* C,N  or  C,R|G|B,x,y */
        if (nf < 2 || strlen(f[1]) != 1) return VP_BAD_FORMAT;
        char c = f[1][0];
        if (c == 'N') {
            if (nf != 2) return VP_BAD_FORMAT;
            m.kind = VIS_NO_OBJECT;
        } else if (c == 'R' || c == 'G' || c == 'B') {
            if (nf != 4) return VP_BAD_FORMAT;
            if (!(parse_int(f[2], 0, VISION_FRAME_W, &m.x, &rng) &&
                  parse_int(f[3], 0, VISION_FRAME_H, &m.y, &rng)))
                return rng ? VP_OUT_OF_RANGE : VP_BAD_FORMAT;
            m.kind = VIS_OBJECT;
            m.colour = c;
        } else {
            return VP_BAD_FORMAT;
        }
    } else {
        return VP_BAD_FORMAT;
    }
    *out = m;
    return VP_OK;
}


/* ============================================================================
 * SHELF MAPPING (FR5)
 * ============================================================================ */
static int map_r = -1, map_g = -1, map_b = -1;

static int *slot(char c) {
    switch (c) {
    case 'R': return &map_r;
    case 'G': return &map_g;
    case 'B': return &map_b;
    default:  return 0;
    }
}

bool colour_valid(char c) { return slot(c) != 0; }

void shelf_map_init(void) {
    map_r = DEFAULT_SHELF_RED;
    map_g = DEFAULT_SHELF_GREEN;
    map_b = DEFAULT_SHELF_BLUE;
}

bool shelf_map_set(char c, int id) {
    int *s = slot(c);
    if (!s || id < 0 || id > VISION_MAX_MARKER_ID) return false;
    *s = id;
    return true;
}

int shelf_map_get(char c) {
    int *s = slot(c);
    return s ? *s : -1;
}


/* ============================================================================
 * SIMULATED ARM (replace with PCA9685 driver)
 * ============================================================================ */
// SIMULATED arm - replace with the real PCA9685/RDS3225 implementation (see arm_hal.h).
#include "FreeRTOS.h"
#include "task.h"

static bool step(int ms) {
    for (int t = 0; t < ms; t += 50) {
        if (arm_hal_abort_requested()) return false;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return true;
}

bool arm_hal_init(void)                                   { return true; }
bool arm_hal_home(void)                                   { return step(1500); }
bool arm_hal_approach_object(int px, int py)              { (void)px; (void)py; return step(2000); }
bool arm_hal_grip(bool close)                             { (void)close; return step(600); }
bool arm_hal_lift(void)                                   { return step(1200); }
bool arm_hal_move_to_shelf(int px, int py, float d)       { (void)px; (void)py; (void)d; return step(2500); }
bool arm_hal_retreat(void)                                { return step(1000); }


/* ============================================================================
 * IMU TASK
 * ============================================================================ */
// IMU Task: wraps components/imu (unchanged) - adds the I2C mutex, periodic sampling and fault debounce.
#include "imu.h"

static imu_snapshot_t snap;
static volatile bool  recapture = false;
static bool           init_ok = false;

bool imu_hw_init(void) {
    /* Runs before the scheduler: no other task can touch the bus, so no mutex needed here. */
    init_ok = imu_init(I2C_PORT);
    if (init_ok) {
        snap.target_pitch = imu_get_pitch();
        snap.target_roll  = imu_get_roll();
        snap.pitch = snap.target_pitch;
        snap.roll  = snap.target_roll;
        snap.status = IMU_WITHIN_TOLERANCE;
        snap.valid = true;
    }
    return init_ok;
}

void imu_snapshot_get(imu_snapshot_t *out) {
    taskENTER_CRITICAL();
    *out = snap;
    taskEXIT_CRITICAL();
}

void imu_request_recapture(void) { recapture = true; }

void imu_task(void *arg) {
    (void)arg;
    int fails = init_ok ? 0 : IMU_FAIL_LIMIT;        /* sensor missing at boot -> fault immediately */
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(IMU_PERIOD_MS));

        bool ok = false;
        if (init_ok) {
            if (i2c_bus_lock()) {                    /* rule M2: bounded wait, released right after */
                ok = imu_update();
                i2c_bus_unlock();
            } else {
                fault_raise(FAULT_I2C_BUS);          /* could not get the bus in time */
            }
        }

        imu_status_t st = IMU_FAULT;
        if (ok) {
            if (recapture) {
                taskENTER_CRITICAL();               /* pitch+roll pair must be read atomically */
                snap.target_pitch = imu_get_pitch();
                snap.target_roll  = imu_get_roll();
                taskEXIT_CRITICAL();
                recapture = false;
            }
            st = imu_check_orientation(snap.target_pitch, snap.target_roll, IMU_TOLERANCE_DEG);
            fails = 0;
        } else if (fails < IMU_FAIL_LIMIT) {
            fails++;
        }

        taskENTER_CRITICAL();
        snap.pitch  = imu_get_pitch();
        snap.roll   = imu_get_roll();
        snap.status = (fails >= IMU_FAIL_LIMIT) ? IMU_FAULT : (ok ? st : snap.status);
        snap.valid  = (fails < IMU_FAIL_LIMIT);
        taskEXIT_CRITICAL();

        if (fails >= IMU_FAIL_LIMIT) fault_raise(FAULT_IMU);
    }
}


/* ============================================================================
 * MOTION CONTROL TASK
 * ============================================================================ */
// Motion Control Task: executes ONE high-level motion at a time and reports with a notification.
//
// Abort protocol (race-free): every command carries the abort generation current when the coordinator
// sent it. motion_abort() bumps the generation, so any command sent earlier - queued, just dequeued
// or already running - is stale and stops silently. A later command carries the new generation.

static volatile uint32_t abort_gen = 0;
static volatile uint32_t running_gen = 0;
static volatile bool     busy = false;

void motion_abort(void)            { abort_gen++; xQueueReset(q_motion); }
bool motion_busy(void)             { return busy; }
uint32_t motion_generation(void)   { return abort_gen; }
bool arm_hal_abort_requested(void) { return busy && running_gen != abort_gen; }

static bool run(const motion_cmd_t *c) {
    switch (c->type) {
    case MOT_HOME:
        return arm_hal_home();
    case MOT_PICK:
        return arm_hal_grip(false)                    /* open */
            && arm_hal_approach_object(c->x, c->y)
            && arm_hal_grip(true)                     /* close - NOTE: closed != grasped (assumption A6) */
            && arm_hal_lift();
    case MOT_PLACE:
        return arm_hal_move_to_shelf(c->x, c->y, c->dist_mm)
            && arm_hal_grip(false)                    /* release */
            && arm_hal_retreat();
    }
    return false;
}

void motion_task(void *arg) {
    (void)arg;
    motion_cmd_t cmd;
    for (;;) {
        if (xQueueReceive(q_motion, &cmd, portMAX_DELAY) != pdTRUE) continue;
        if (cmd.gen != abort_gen) continue;           /* aborted before it started */
        running_gen = cmd.gen;
        busy = true;
        bool ok = run(&cmd);
        bool stale = (cmd.gen != abort_gen);
        busy = false;
        if (!stale)                                   /* stale => coordinator already moved on: stay silent */
            xTaskNotify(h_coordinator, ok ? NOTIF_MOTION_DONE : NOTIF_MOTION_FAIL, eSetBits);
    }
}


/* ============================================================================
 * VISION COMMUNICATION TASK
 * ============================================================================ */
// Vision Communication Task: UART RX ISR -> stream buffer -> line assembler -> validator -> q_vision.
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

static StreamBufferHandle_t rx_stream;
static volatile uint32_t    rx_dropped = 0;     /* ISR could not store a byte (stream full) */

/* ---- ISR: do nothing but move bytes ---- */
static void vision_uart_isr(void) {
    BaseType_t woken = pdFALSE;
    while (uart_is_readable(VISION_UART)) {
        uint8_t c = (uint8_t)uart_getc(VISION_UART);
        if (xStreamBufferSendFromISR(rx_stream, &c, 1, &woken) != 1) rx_dropped++;
    }
    portYIELD_FROM_ISR(woken);
}

void vision_hw_init(void) {
    rx_stream = xStreamBufferCreate(VISION_RX_STREAM_BYTES, 1);
    uart_init(VISION_UART, VISION_BAUD);
    gpio_set_function(PIN_VISION_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_VISION_RX, GPIO_FUNC_UART);
    uart_set_format(VISION_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(VISION_UART, true);
    irq_set_exclusive_handler(VISION_UART_IRQ, vision_uart_isr);
    irq_set_enabled(VISION_UART_IRQ, true);
    uart_set_irq_enables(VISION_UART, true, false);
}

static void send_mode(char m) {
    if (m != 'S' && m != 'C' && m != 'I') return;
    char line[4] = {'M', ',', m, '\n'};
    uart_write_blocking(VISION_UART, (const uint8_t *)line, sizeof line);
}

static void publish(const vision_msg_t *m) {
    if (xQueueSend(q_vision, m, 0) != pdTRUE) {      /* full: drop OLDEST, keep newest (rule Q2) */
        vision_msg_t old;
        xQueueReceive(q_vision, &old, 0);
        xQueueSend(q_vision, m, 0);
    }
    xTaskNotify(h_coordinator, NOTIF_VISION, eSetBits);
}

void vision_task(void *arg) {
    (void)arg;
    char   line[96];
    size_t len = 0;
    int    bad = 0;
    uint32_t last_dropped = 0;

    send_mode('I');                                  /* XIAO defaults to SCAN and streams: quiet it */

    for (;;) {
        char mode;
        while (xQueueReceive(q_vision_cmd, &mode, 0) == pdTRUE) send_mode(mode);

        uint8_t c;
        if (xStreamBufferReceive(rx_stream, &c, 1, pdMS_TO_TICKS(20)) != 1) continue;

        if (c != '\n') {
            if (len < sizeof line - 1) line[len++] = (char)c;
            else { len = 0; bad++; }                 /* overlong line: discard */
        } else {
            line[len] = '\0';
            len = 0;
            vision_msg_t m;
            vp_result_t r = vision_parse_line(line, &m);
            if (r == VP_OK)         { bad = 0; publish(&m); }
            else if (r != VP_EMPTY) { bad++; }
        }

        if (rx_dropped != last_dropped) { last_dropped = rx_dropped; bad++; }
        if (bad >= VISION_BAD_LINE_LIMIT) {          /* garbage on the link: wrong baud / noise / crash */
            fault_raise(FAULT_VISION_COMM);
            bad = 0;
        }
    }
}


/* ============================================================================
 * USER INTERFACE TASK (USB CLI)
 * ============================================================================ */
// User Interface Task: USB-serial CLI. The ONLY task that reads/prints on stdio after boot.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"

static void post(cmd_type_t t, char colour, int value) {
    cmd_msg_t m = { t, colour, value };
    if (xQueueSend(q_cmd, &m, 0) != pdTRUE) { printf("! busy, command dropped\n"); return; }
    xTaskNotify(h_coordinator, NOTIF_CMD, eSetBits);
}

static void help(void) {
    printf("Commands:\n"
           "  pick <R|G|B>     pick an object of that colour and shelve it\n"
           "  map <R|G|B> <id> set shelf ArUco marker id for a colour\n"
           "  home             move arm to home pose\n"
           "  stop             abort the current motion (arm holds position)\n"
           "  reset            clear a FAULT\n"
           "  cal              re-capture IMU level reference (arm must be level)\n"
           "  status           show state, mapping, IMU\n");
}

static void handle_line(char *line) {
    char *argv[3];
    int argc = 0;
    for (char *t = strtok(line, " \t"); t && argc < 3; t = strtok(NULL, " \t")) argv[argc++] = t;
    if (argc == 0) return;
    for (char *p = argv[0]; *p; p++) *p = (char)tolower((unsigned char)*p);

    char colour = (argc > 1) ? (char)toupper((unsigned char)argv[1][0]) : 0;

    if      (!strcmp(argv[0], "pick") && argc == 2)   post(CMD_PICK, colour, 0);   /* validated by coordinator (FR8) */
    else if (!strcmp(argv[0], "map")  && argc == 3)   post(CMD_MAP, colour, atoi(argv[2]));
    else if (!strcmp(argv[0], "home"))                post(CMD_HOME, 0, 0);
    else if (!strcmp(argv[0], "stop"))                post(CMD_STOP, 0, 0);
    else if (!strcmp(argv[0], "reset"))               post(CMD_RESET, 0, 0);
    else if (!strcmp(argv[0], "cal"))                 post(CMD_RECAL_IMU, 0, 0);
    else if (!strcmp(argv[0], "status"))              post(CMD_STATUS, 0, 0);
    else if (!strcmp(argv[0], "help"))                help();
    else printf("? unknown command '%s' (try: help)\n", argv[0]);
}

void ui_task(void *arg) {
    (void)arg;
    char   line[48];
    size_t len = 0;

    printf("\nRobot arm ready. Type 'help'.\n> ");

    for (;;) {
        ui_msg_t m;
        while (xQueueReceive(q_ui, &m, 0) == pdTRUE) printf("\n%s\n> ", m.text);

        int ch;
        while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            if (ch == '\r' || ch == '\n') {
                putchar('\n');
                line[len] = '\0';
                handle_line(line);
                len = 0;
                printf("> ");
            } else if ((ch == 0x7f || ch == '\b') && len) {
                len--; printf("\b \b");
            } else if (ch >= 32 && ch < 127 && len < sizeof line - 1) {
                line[len++] = (char)ch; putchar(ch);
            }
        }
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/* ============================================================================
 * SYSTEM COORDINATOR TASK
 * ============================================================================ */
// System Coordinator Task - the system-level statechart (report section 5).
//
//   IDLE --pick--> SCAN_OBJECT --object seen--> PICK --done--> [IMU check] --> SCAN_SHELF
//   SCAN_SHELF --marker==shelf--> [IMU check] --> PLACE --done--> HOME --done--> IDLE
//   any state --fault bit / motion fail / timeout(no recovery)--> FAULT --reset--> IDLE
//   any state --stop--> IDLE (arm holds position, must 'home' before next pick)
//
// One blocking point only: xTaskNotifyWait() with the current state's deadline as the timeout.
#include <stdio.h>

typedef enum { ST_IDLE, ST_SCAN_OBJECT, ST_PICK, ST_SCAN_SHELF, ST_PLACE, ST_HOME, ST_FAULT } state_t;

static const char *const state_name[] = {
    "IDLE", "SCAN_OBJECT", "PICK", "SCAN_SHELF", "PLACE", "HOME", "FAULT"
};

static struct {
    state_t      st;
    bool         has_deadline;
    TickType_t   deadline;
    char         colour;
    int          shelf_id;
    int          retries;
    bool         vision_alive;       /* any valid vision message seen in this scan window */
    bool         arm_homed;          /* false after STOP: refuse pick until 'home' */
    vision_msg_t obj;
    uint32_t     fault_bits;
    const char  *fault_msg;
} c;

/* ---------- helpers ---------- */
static void set_deadline(uint32_t ms) { c.deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ms); c.has_deadline = true; }
static void clear_deadline(void)      { c.has_deadline = false; }

static void vision_mode(char m) { xQueueSend(q_vision_cmd, &m, 0); }

static void goto_state(state_t s) {
    ui_printf("[%s -> %s]", state_name[c.st], state_name[s]);
    c.st = s;
}

static void enter_fault(uint32_t bits, const char *why) {
    motion_abort();                     /* FAULT = hold last position, stop commanding (report) */
    vision_mode('I');
    clear_deadline();
    c.fault_bits = bits;
    c.fault_msg  = why;
    goto_state(ST_FAULT);
    ui_printf("FAULT: %s (bits 0x%x). Arm holds position. Cut 6V rail if unsafe, then 'reset'.", why, (unsigned)bits);
}

static void back_to_idle(const char *msg) {
    clear_deadline();
    vision_mode('I');
    if (msg) ui_printf("%s", msg);
    goto_state(ST_IDLE);
}

static void send_motion(motion_type_t t, int x, int y, float d, state_t next, uint32_t timeout_ms) {
    motion_cmd_t m = { t, x, y, d, motion_generation() };
    if (xQueueSend(q_motion, &m, 0) != pdTRUE) { enter_fault(FAULT_MOTION, "motion queue full"); return; }
    set_deadline(timeout_ms);
    goto_state(next);
}

/* Returns true when it is safe to continue. */
static bool imu_ok(const char *where) {
    imu_snapshot_t s;
    imu_snapshot_get(&s);
    if (!s.valid || s.status == IMU_FAULT) { enter_fault(FAULT_IMU, "IMU not responding"); return false; }
#if IMU_OOT_IS_FAULT
    if (s.status == IMU_OUT_OF_TOLERANCE) {
        ui_printf("IMU out of tolerance at %s: pitch %.1f roll %.1f (target %.1f / %.1f)",
                  where, s.pitch, s.roll, s.target_pitch, s.target_roll);
        enter_fault(FAULT_IMU, "end-effector orientation out of tolerance");
        return false;
    }
#endif
    return true;
}

static void start_scan_object(void) {
    c.retries = 0;
    c.vision_alive = false;
    vision_mode('C');
    set_deadline(VISION_OBJECT_TIMEOUT_MS);
    goto_state(ST_SCAN_OBJECT);
}

static void start_scan_shelf(void) {
    c.retries = 0;
    vision_mode('S');
    set_deadline(VISION_SHELF_TIMEOUT_MS);
    goto_state(ST_SCAN_SHELF);
}

/* ---------- event handlers ---------- */
static void on_status(void) {
    imu_snapshot_t s;
    imu_snapshot_get(&s);
    ui_printf("state=%s armHomed=%d map R=%d G=%d B=%d", state_name[c.st], c.arm_homed,
              shelf_map_get('R'), shelf_map_get('G'), shelf_map_get('B'));
    ui_printf("IMU valid=%d pitch=%.1f roll=%.1f status=%d", s.valid, s.pitch, s.roll, (int)s.status);
    if (c.st == ST_FAULT) ui_printf("fault: %s", c.fault_msg);
}

static void on_cmd(const cmd_msg_t *m) {
    switch (m->type) {
    case CMD_STATUS: on_status(); break;

    case CMD_MAP:
        if (c.st != ST_IDLE) { ui_printf("busy: change mapping only when IDLE"); break; }
        if (shelf_map_set(m->colour, m->value)) ui_printf("mapped %c -> shelf marker %d", m->colour, m->value);
        else ui_printf("invalid mapping (colour R/G/B, marker id 0..%d)", VISION_MAX_MARKER_ID);
        break;

    case CMD_RECAL_IMU:
        if (c.st != ST_IDLE) { ui_printf("busy"); break; }
        imu_request_recapture();
        ui_printf("IMU level reference will be re-captured");
        break;

    case CMD_RESET:
        if (c.st != ST_FAULT) { ui_printf("no fault to reset"); break; }
        xEventGroupClearBits(evg_fault, FAULT_ALL);
        c.arm_homed = false;                 /* position unknown after a fault */
        back_to_idle("fault cleared - run 'home' before the next pick");
        break;

    case CMD_STOP:
        if (c.st == ST_IDLE || c.st == ST_FAULT) { ui_printf("nothing to stop"); break; }
        motion_abort();
        c.arm_homed = false;
        back_to_idle("stopped by operator - arm holds position, run 'home'");
        break;

    case CMD_HOME:
        if (c.st != ST_IDLE) { ui_printf("busy (%s)", state_name[c.st]); break; }
        if (motion_busy())   { ui_printf("arm still stopping, retry"); break; }
        send_motion(MOT_HOME, 0, 0, 0, ST_HOME, MOTION_HOME_TIMEOUT_MS);
        break;

    case CMD_PICK:
        if (c.st != ST_IDLE)         { ui_printf("busy (%s)", state_name[c.st]); break; }
        if (!colour_valid(m->colour)) { ui_printf("invalid selection '%c' (use R, G or B)", m->colour ? m->colour : '?'); break; }   /* FR8 */
        c.shelf_id = shelf_map_get(m->colour);
        if (c.shelf_id < 0)          { ui_printf("no shelf mapped for %c - use: map %c <id>", m->colour, m->colour); break; }      /* FR8 */
        if (!c.arm_homed)            { ui_printf("arm not homed - run 'home' first"); break; }
        if (motion_busy())           { ui_printf("arm still stopping, retry"); break; }
        c.colour = m->colour;
        ui_printf("pick %c -> shelf marker %d", c.colour, c.shelf_id);
        start_scan_object();
        break;
    }
}

static void on_vision(const vision_msg_t *v) {
    switch (c.st) {
    case ST_SCAN_OBJECT:
        c.vision_alive = true;
        if (v->kind == VIS_OBJECT && v->colour == c.colour) {
            c.obj = *v;
            vision_mode('I');
            ui_printf("object %c at (%d,%d)", v->colour, v->x, v->y);
            send_motion(MOT_PICK, v->x, v->y, 0, ST_PICK, MOTION_PICK_TIMEOUT_MS);
        }
        break;                                   /* other colours / "none" -> keep waiting for deadline */

    case ST_SCAN_SHELF:
        if (v->kind == VIS_MARKER && v->id == c.shelf_id) {
            if (!imu_ok("shelf approach")) return;
            vision_mode('I');
            ui_printf("shelf %d at (%d,%d) %.0f mm", v->id, v->x, v->y, (double)v->dist_mm);
            send_motion(MOT_PLACE, v->x, v->y, v->dist_mm, ST_PLACE, MOTION_PLACE_TIMEOUT_MS);
        }
        break;

    default:
        break;                                   /* stale message from an earlier mode: ignore */
    }
}

static void on_motion_done(void) {
    switch (c.st) {
    case ST_PICK:
        if (!imu_ok("after lift")) return;
        start_scan_shelf();
        break;
    case ST_PLACE:
        send_motion(MOT_HOME, 0, 0, 0, ST_HOME, MOTION_HOME_TIMEOUT_MS);
        break;
    case ST_HOME:
        c.arm_homed = true;
        back_to_idle("cycle complete / homed");
        break;
    default:
        break;
    }
}

static void on_timeout(void) {
    switch (c.st) {
    case ST_SCAN_OBJECT:
        if (c.retries < VISION_OBJECT_RETRIES) {
            c.retries++;
            vision_mode('C');
            set_deadline(VISION_OBJECT_TIMEOUT_MS);
            ui_printf("vision retry %d", c.retries);
        } else if (!c.vision_alive) {
            enter_fault(FAULT_VISION_COMM, "vision module silent (NFR3)");   /* link down */
        } else {
            back_to_idle("object not found - nothing moved");               /* link ok, no object */
        }
        break;
    case ST_SCAN_SHELF:
        if (c.retries < VISION_SHELF_RETRIES) {
            c.retries++;
            vision_mode('S');
            set_deadline(VISION_SHELF_TIMEOUT_MS);
            ui_printf("shelf marker retry %d", c.retries);
        } else {
            /* Holding an object: do NOT drop it or wander - freeze (report FAULT policy). */
            enter_fault(FAULT_VISION_COMM, "shelf marker not found / vision down while carrying");
        }
        break;
    case ST_PICK:
    case ST_PLACE:
    case ST_HOME:
        enter_fault(FAULT_MOTION, "motion timeout");
        break;
    default:
        clear_deadline();
        break;
    }
}

/* ---------- task ---------- */
void coordinator_task(void *arg) {
    (void)arg;
    shelf_map_init();
    c.st = ST_IDLE;
    c.arm_homed = false;
    ui_printf("coordinator up. Run 'home' first.");

    for (;;) {
        TickType_t wait = portMAX_DELAY;
        if (c.has_deadline) {
            TickType_t now = xTaskGetTickCount();
            wait = ((int32_t)(c.deadline - now) > 0) ? (c.deadline - now) : 0;
        }

        uint32_t bits = 0;
        if (xTaskNotifyWait(0, 0xFFFFFFFFu, &bits, wait) != pdTRUE) {
            on_timeout();
            continue;
        }

        /* faults first: they pre-empt everything */
        uint32_t f = xEventGroupGetBits(evg_fault) & FAULT_ALL;
        if (f && c.st != ST_FAULT) {
            const char *why = (f & FAULT_IMU) ? "IMU fault"
                            : (f & FAULT_VISION_COMM) ? "vision link fault"
                            : (f & FAULT_I2C_BUS) ? "I2C bus fault" : "fault";
            enter_fault(f, why);
        }

        if (bits & NOTIF_CMD) {
            cmd_msg_t m;
            while (xQueueReceive(q_cmd, &m, 0) == pdTRUE) on_cmd(&m);
        }
        if (bits & NOTIF_VISION) {
            vision_msg_t v;
            while (xQueueReceive(q_vision, &v, 0) == pdTRUE) on_vision(&v);
        }
        if (c.st != ST_FAULT) {
            if (bits & NOTIF_MOTION_FAIL)      enter_fault(FAULT_MOTION, "motion failed");
            else if (bits & NOTIF_MOTION_DONE) on_motion_done();
        }
    }
}

/* ============================================================================
 * MAIN
 * ============================================================================ */
#include <stdio.h>
#include "pico/stdlib.h"

void vApplicationMallocFailedHook(void) {
    printf("FATAL: out of heap\n");
    for (;;) tight_loop_contents();
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *name) {
    (void)t;
    printf("FATAL: stack overflow in %s\n", name);
    for (;;) tight_loop_contents();
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);                              /* let USB serial attach so boot messages are seen */
    printf("\n=== Robot arm integration build ===\n");

    if (!ipc_init()) { printf("FATAL: IPC allocation failed\n"); for (;;) tight_loop_contents(); }

    /* Hardware that must exist before any task runs */
    i2c_bus_init();
    vision_hw_init();
    bool imu_ok = imu_hw_init();
    bool arm_ok = arm_hal_init();
    printf("IMU: %s, arm HAL: %s\n", imu_ok ? "ok" : "NOT FOUND", arm_ok ? "ok" : "FAILED");

    xTaskCreate(coordinator_task, "coord",  STACK_COORDINATOR, NULL, PRIO_COORDINATOR, &h_coordinator);
    xTaskCreate(vision_task,      "vision", STACK_VISION,      NULL, PRIO_VISION,      NULL);
    xTaskCreate(imu_task,         "imu",    STACK_IMU,         NULL, PRIO_IMU,         NULL);
    xTaskCreate(motion_task,      "motion", STACK_MOTION,      NULL, PRIO_MOTION,      NULL);
    xTaskCreate(ui_task,          "ui",     STACK_UI,          NULL, PRIO_IDLE_UI,     NULL);

    if (!arm_ok) fault_raise(FAULT_MOTION);      /* coordinator picks this up on its first wake-up */

    vTaskStartScheduler();
    for (;;) tight_loop_contents();              /* not reached */
}
