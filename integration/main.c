#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <tk/smp.h>

#include "../components/imu/imu_heading.h"
#include "../components/drive/motor.h"
#include "../components/drive/claw.h"

#include <stdarg.h>

/* ---------- Configuration ---------- */

#define WIFI_LOG_QUEUE_SIZE 16U
#define WIFI_LOG_LINE_SIZE  240U

#define RX_LINE_SIZE 128

#define OBJECT_SCAN_COMMAND          "M,3\n"
#define DIRECTION_SCAN_COMMAND       "M,8\n"
#define SCAN_WINDOW_MS               1200
#define OBJECT_STANDOFF_MM           100
#define DIRECTION_STANDOFF_MM        200

#define DIRECTION_ALIGN_TOLERANCE_PX 74
#define DIRECTION_ALIGN_PULSE_MS     70
#define DIRECTION_SEARCH_PULSE_MS    70

#define REVERSE_AFTER_RELEASE_MS    300
#define OBJECT_SCAN_TURN_SPEED      18
#define OBJECT_SCAN_TURN_PULSE_MS   150

#define CAR_DRIVE_SPEED             25
#define CAR_TURN_SPEED              20
#define CAR_TURN_90_MS               400
#define DRIVE_PULSE_MS              100

#define MAX_APPROACH_STEPS          30
#define MAX_LOST_SCAN_ATTEMPTS      3
#define CAMERA_CENTER_X             160

#define DRIVE_RAMP_STEP             10
#define DRIVE_RAMP_MS               15

#define MAX_OBJECTS                 16
#define MAX_ARUCO_IDS               1000

#define IMU_STARTUP_TIMEOUT_MS      60000U
#define IMU_READY_HOLD_MS           10000U
#define IMU_LOG_PERIOD_MS           500U

/* ---------- Types ---------- */

typedef struct {
    UW length;
    UB bytes[WIFI_LOG_LINE_SIZE];
} WifiLogEntry;

typedef struct {
    UB bytes[4];
} ModeCommand;

typedef struct {
    INT left;
    INT right;
    INT ms;
    BOOL stop;
} DriveCommand;

typedef struct {
    int id;
    int x;
    int y;
    float distance;
} marker_reading_t;

/* ---------- Shared state ---------- */

static WifiLogEntry wifi_log_queue[WIFI_LOG_QUEUE_SIZE];
static volatile UW wifi_log_head;
static volatile UW wifi_log_tail;
static volatile UW wifi_log_count;
static T_SPLOCK wifi_log_lock;

static T_SPLOCK mission_guard_lock;
static BOOL mission_stop_latched = FALSE;

static marker_reading_t object_map[MAX_OBJECTS];
static int object_count;
static BOOL released_object_ids[MAX_ARUCO_IDS];

static marker_reading_t direction_map[3];
static BOOL direction_seen[3];
static int direction_count;

/* ---------- Kernel objects ---------- */

LOCAL ID task_uart_rx_id;
LOCAL ID task_uart_tx_id;
LOCAL ID task_state_id;
LOCAL ID task_drive_id;
LOCAL ID task_imu_monitor_id;

LOCAL ID uart_device_id;
LOCAL ID command_mbf_id;
LOCAL ID mtx_id;
LOCAL ID drive_mbf_id;

LOCAL UW command_mbf_buffer[
    (4 * sizeof(ModeCommand)) / sizeof(UW) + 16
];

LOCAL UW drive_mbf_buffer[
    (4 * (sizeof(DriveCommand) + 4)) / sizeof(UW) + 8
];

LOCAL void task_uart_rx(INT stacd, void *exinf);
LOCAL void task_uart_tx(INT stacd, void *exinf);
LOCAL void task_state_machine(INT stacd, void *exinf);
LOCAL void task_drive(INT stacd, void *exinf);
LOCAL void task_imu_monitor(INT stacd, void *exinf);

LOCAL T_CTSK ctsk_uart_rx = {
    .itskpri = 5,
    .stksz = 4096,
    .task = task_uart_rx,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_uart_tx = {
    .itskpri = 6,
    .stksz = 4096,
    .task = task_uart_tx,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_state = {
    .itskpri = 10,
    .stksz = 4096,
    .task = task_state_machine,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC2,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_drive = {
    .itskpri = 5,
    .stksz = 4096,
    .task = task_drive,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC2,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_imu_monitor = {
    .itskpri = 8,
    .stksz = 4096,
    .task = task_imu_monitor,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC2,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CMBF cmbf = {
    .bufsz = sizeof(command_mbf_buffer),
    .maxmsz = sizeof(ModeCommand),
    .mbfatr = TA_TFIFO,
    .bufptr = command_mbf_buffer
};

LOCAL T_CMTX cmtx = {
    .mtxatr = TA_TFIFO | TA_INHERIT
};

LOCAL T_CMBF drive_cmbf = {
    .bufsz = sizeof(drive_mbf_buffer),
    .maxmsz = sizeof(DriveCommand),
    .mbfatr = TA_TFIFO,
    .bufptr = drive_mbf_buffer
};

/* ---------- Existing log queue ---------- */

static void wifi_log_lock_queue(void)
{
    (void)SpinLock(&wifi_log_lock);
}

static void wifi_log_unlock_queue(void)
{
    (void)SpinUnlock(&wifi_log_lock);
}

static void wifi_log_enqueue(const UB *bytes, UW length)
{
    UW slot;

    if (length == 0)
        return;

    if (length > WIFI_LOG_LINE_SIZE)
        length = WIFI_LOG_LINE_SIZE;

    wifi_log_lock_queue();

    if (wifi_log_count == WIFI_LOG_QUEUE_SIZE) {
        wifi_log_tail =
            (wifi_log_tail + 1U) % WIFI_LOG_QUEUE_SIZE;
        wifi_log_count--;
    }

    slot = wifi_log_head;

    for (UW i = 0; i < length; i++)
        wifi_log_queue[slot].bytes[i] = bytes[i];

    wifi_log_queue[slot].length = length;
    wifi_log_head =
        (wifi_log_head + 1U) % WIFI_LOG_QUEUE_SIZE;
    wifi_log_count++;

    wifi_log_unlock_queue();
}

static void robot_log_append(
    UB *output,
    UW capacity,
    UW *length,
    const UB *text,
    UW text_length
)
{
    while (text_length != 0 && *length < capacity) {
        output[(*length)++] = *text++;
        text_length--;
    }
}

static INT robot_log_vformat(
    UB *output,
    UW capacity,
    const UB *format,
    va_list args
)
{
    UB temporary[96];
    UW length = 0;

    while (*format != '\0' && length < capacity) {
        UB specifier[16];
        UW spec_length = 0;
        INT formatted_length;
        UB conversion;

        if (*format != '%') {
            robot_log_append(
                output, capacity, &length, format++, 1
            );
            continue;
        }

        format++;

        if (*format == '%') {
            robot_log_append(
                output, capacity, &length, format++, 1
            );
            continue;
        }

        if (*format == '\0') {
            robot_log_append(
                output, capacity, &length,
                (const UB *)"%", 1
            );
            break;
        }

        specifier[spec_length++] = '%';

        while (*format == '0' ||
               *format == '-' ||
               *format == '+' ||
               *format == ' ' ||
               *format == '#') {
            if (spec_length + 2U >= sizeof(specifier))
                break;

            specifier[spec_length++] = *format++;
        }

        while (*format >= '0' && *format <= '9') {
            if (spec_length + 2U >= sizeof(specifier))
                break;

            specifier[spec_length++] = *format++;
        }

        conversion = *format++;
        specifier[spec_length++] = conversion;
        specifier[spec_length] = '\0';

        switch (conversion) {
        case 'd':
        case 'i':
            formatted_length = tm_sprintf(
                temporary, specifier, va_arg(args, INT)
            );
            break;

        case 'u':
        case 'o':
        case 'x':
        case 'X':
            formatted_length = tm_sprintf(
                temporary, specifier, va_arg(args, UW)
            );
            break;

        case 'c':
            formatted_length = tm_sprintf(
                temporary, specifier, va_arg(args, INT)
            );
            break;

        case 's':
        {
            const UB *text =
                (const UB *)va_arg(args, const char *);

            if (text == NULL)
                text = (const UB *)"(null)";

            while (*text != '\0' && length < capacity)
                output[length++] = *text++;

            continue;
        }

        default:
            robot_log_append(
                output, capacity, &length,
                (const UB *)"%", 1
            );

            robot_log_append(
                output, capacity, &length,
                &conversion, 1
            );
            continue;
        }

        if (formatted_length > 0) {
            robot_log_append(
                output, capacity, &length,
                temporary, (UW)formatted_length
            );
        }
    }

    output[length] = '\0';
    return (INT)length;
}

INT robot_logf(const UB *format, ...)
{
    UB line[WIFI_LOG_LINE_SIZE + 1U];
    va_list args;
    INT length;

    va_start(args, format);
    length = robot_log_vformat(
        line, WIFI_LOG_LINE_SIZE, format, args
    );
    va_end(args);

    (void)tm_printf((UB *)"%s", line);
    wifi_log_enqueue(line, (UW)length);

    return length;
}

int robot_log_dequeue(
    unsigned char *bytes,
    unsigned int capacity,
    unsigned int *length
)
{
    WifiLogEntry entry;

    wifi_log_lock_queue();

    if (wifi_log_count == 0) {
        wifi_log_unlock_queue();
        return 0;
    }

    entry = wifi_log_queue[wifi_log_tail];
    wifi_log_tail =
        (wifi_log_tail + 1U) % WIFI_LOG_QUEUE_SIZE;
    wifi_log_count--;

    wifi_log_unlock_queue();

    if (entry.length > capacity)
        entry.length = capacity;

    for (unsigned int i = 0; i < entry.length; i++)
        bytes[i] = entry.bytes[i];

    *length = entry.length;
    return 1;
}

/* ---------- IMU startup gate and runtime interlock ---------- */

static BOOL mission_now_ms(UW *now)
{
    SYSTIM time;

    if (tk_get_otm(&time) != E_OK)
        return FALSE;

    *now = time.lo;
    return TRUE;
}

static BOOL mission_is_stopped(void)
{
    UINT intsts;
    BOOL stopped;

    (void)ISpinLock(&mission_guard_lock, &intsts);
    stopped = mission_stop_latched;
    (void)ISpinUnlock(&mission_guard_lock, intsts);

    return stopped;
}

static void mission_latch_stop(const char *reason)
{
    UINT intsts;
    BOOL first;

    (void)ISpinLock(&mission_guard_lock, &intsts);

    first = !mission_stop_latched;
    mission_stop_latched = TRUE;

    (void)ISpinUnlock(&mission_guard_lock, intsts);

    if (first) {
        robot_logf(
            (UB *)"[mission] STOP LATCHED: %s; "
                   "reset required to restart\n",
            reason
        );
    }
}

static BOOL mission_imu_healthy(
    const imu_heading_reading_t *s
)
{
    return
        s->fault == E_OK &&
        s->ready &&
        s->valid &&
        s->mag_valid;
}

static BOOL mission_motion_allowed(void)
{
    imu_heading_reading_t s = {0};

    if (mission_is_stopped())
        return FALSE;

    imu_heading_read(&s);

    if (!mission_imu_healthy(&s)) {
        mission_latch_stop(
            "IMU fault, stale gyro data or lost magnetic heading"
        );
        return FALSE;
    }

    return !mission_is_stopped();
}

static void mission_park(void)
{
    for (;;)
        (void)tk_slp_tsk(TMO_FEVR);
}

/*
 * Called by the mission task before further actions.
 * The drive task independently stops motor output.
 */
static void mission_checkpoint(void)
{
    if (mission_motion_allowed())
        return;

    claw_release();
    mission_park();
}

/*
 * Wait while continuing to check IMU health.
 * The caller must stop its motor output if FALSE is returned.
 */
static BOOL mission_checked_delay(UW delay_ms)
{
    UW started;
    UW now;

    if (!mission_now_ms(&started)) {
        mission_latch_stop("Could not read system time");
        return FALSE;
    }

    for (;;) {
        if (!mission_motion_allowed())
            return FALSE;

        if (!mission_now_ms(&now)) {
            mission_latch_stop("Could not read system time");
            return FALSE;
        }

        UW elapsed = now - started;

        if (elapsed >= delay_ms)
            return TRUE;

        UW remaining = delay_ms - elapsed;

        (void)tk_dly_tsk(
            (RELTIM)(remaining > 10U ? 10U : remaining)
        );
    }
}

static void print_imu_reading(
    const imu_heading_reading_t *s,
    const char *mission_status
)
{
    if (s->fault < E_OK) {
        robot_logf(
            (UB *)"[IMU] FAULT=%d; %s\n",
            s->fault,
            mission_status
        );
        return;
    }

    if (!s->ready) {
        robot_logf(
            (UB *)"[IMU] WAIT; M_ERR=%u REJ=%u; %s\n",
            s->mag_errors,
            s->rejected,
            mission_status
        );
        return;
    }

    const char *mode = s->valid
        ? (s->mag_valid ? "FUSED" : "GYRO_ONLY")
        : "STALE";

    robot_logf(
        (UB *)"[IMU] %s H_cd=%d REL_cd=%d GYRO_cd=%d "
               "MAG_cd=%d GZ_cdps=%d FIELD_milli=%d "
               "G_Hz10=%u M_Hz10=%u M_ERR=%u REJ=%u "
               "OVR=%u LATE=%u; %s\n",
        mode,
        (INT)(s->heading_deg * 100.0f),
        (INT)(s->relative_deg * 100.0f),
        (INT)(s->gyro_relative_deg * 100.0f),
        (INT)(s->magnetic_deg * 100.0f),
        (INT)(s->gyro_z_dps * 100.0f),
        (INT)(s->field_xy * 1000.0f),
        s->gyro_hz_x10,
        s->mag_hz_x10,
        s->mag_errors,
        s->rejected,
        s->overruns,
        s->late,
        mission_status
    );
}

static BOOL wait_for_imu_startup(void)
{
    UW started;
    UW now;
    UW stable_started = 0;
    UW last_print;
    BOOL accumulating = FALSE;

    if (!mission_now_ms(&started))
        return FALSE;

    last_print = started;

    robot_logf(
        (UB *)"[startup] Keep car completely still. "
               "Waiting for calibration and 10 seconds "
               "of healthy IMU readings.\n"
    );

    for (;;) {
        imu_heading_reading_t s = {0};
        imu_heading_read(&s);

        if (!mission_now_ms(&now))
            return FALSE;

        if (s.fault < E_OK) {
            print_imu_reading(&s, "startup held");
            return FALSE;
        }

        BOOL acceptable =
            mission_imu_healthy(&s) &&
            s.gyro_hz_x10 >= 900U &&
            s.gyro_hz_x10 <= 1100U &&
            s.mag_hz_x10 >= 200U &&
            s.mag_hz_x10 <= 300U &&
            s.gyro_z_dps >= -0.50f &&
            s.gyro_z_dps <= 0.50f;

        if (acceptable) {
            if (!accumulating) {
                stable_started = now;
                accumulating = TRUE;
            }

            if ((UW)(now - stable_started) >=
                IMU_READY_HOLD_MS) {

                robot_logf(
                    (UB *)"[startup] IMU ready; "
                           "starting integrated tasks.\n"
                );
                return TRUE;
            }
        } else {
            accumulating = FALSE;
        }

        if ((UW)(now - last_print) >= 1000U) {
            print_imu_reading(&s, "startup held");

            if (accumulating) {
                robot_logf(
                    (UB *)"[startup] Healthy hold=%u/%u ms\n",
                    (UW)(now - stable_started),
                    IMU_READY_HOLD_MS
                );
            }

            last_print = now;
        }

        if ((UW)(now - started) >=
            IMU_STARTUP_TIMEOUT_MS) {

            robot_logf(
                (UB *)"[startup] IMU readiness timed out.\n"
            );
            return FALSE;
        }

        (void)tk_dly_tsk(20);
    }
}

LOCAL void task_imu_monitor(INT stacd, void *exinf)
{
    UW now;
    UW last_print = 0;
    BOOL have_print_time = FALSE;

    (void)stacd;
    (void)exinf;

    for (;;) {
        (void)mission_motion_allowed();

        if (!mission_now_ms(&now)) {
            mission_latch_stop("Could not read system time");
            (void)tk_dly_tsk(100);
            continue;
        }

        if (!have_print_time ||
            (UW)(now - last_print) >= IMU_LOG_PERIOD_MS) {

            imu_heading_reading_t s = {0};
            imu_heading_read(&s);

            print_imu_reading(
                &s,
                mission_is_stopped()
                    ? "mission stopped; reset required"
                    : "mission enabled"
            );

            last_print = now;
            have_print_time = TRUE;
        }

        (void)tk_dly_tsk(100);
    }
}

/* ---------- Camera data helpers ---------- */

static const char *direction_for_marker(int id)
{
    switch (id) {
    case 1: return "North";
    case 2: return "West";
    case 3: return "East";
    default: return NULL;
    }
}

int my_strlen(const char *s)
{
    int len = 0;
    while (*s++)
        len++;
    return len;
}

char *my_strchr(const char *s, int c)
{
    while (*s != (char)c) {
        if (!*s++)
            return 0;
    }
    return (char *)s;
}

char *my_strstr(const char *haystack, const char *needle)
{
    if (!*needle)
        return (char *)haystack;

    char *p1 = (char *)haystack;

    while (*p1) {
        char *p1_begin = p1;
        char *p2 = (char *)needle;

        while (*p1 && *p2 && *p1 == *p2) {
            p1++;
            p2++;
        }

        if (!*p2)
            return p1_begin;

        p1 = p1_begin + 1;
    }

    return 0;
}

int parse_int(char **str)
{
    int res = 0;

    while (**str >= '0' && **str <= '9') {
        res = res * 10 + (**str - '0');
        (*str)++;
    }

    if (**str == ',')
        (*str)++;

    return res;
}

float parse_float(char **str)
{
    float res = 0.0f;
    int sign = 1;

    if (**str == '-') {
        sign = -1;
        (*str)++;
    }

    while (**str >= '0' && **str <= '9') {
        res = res * 10.0f + (**str - '0');
        (*str)++;
    }

    if (**str == '.') {
        float frac = 0.1f;
        (*str)++;

        while (**str >= '0' && **str <= '9') {
            res += (**str - '0') * frac;
            frac *= 0.1f;
            (*str)++;
        }
    }

    if (**str == ',')
        (*str)++;

    return res * sign;
}

static int uart_hex_value(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';

    if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;

    if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;

    return -1;
}

void send_mode_command(const char *mode)
{
    ModeCommand command = {{0, 0, 0, 0}};
    UW len = 0;

    while (len < sizeof(command.bytes) &&
           mode[len] != '\0') {
        command.bytes[len] = (UB)mode[len];
        ++len;
    }

    if (len == 0 ||
        tk_snd_mbf(
            command_mbf_id,
            &command,
            sizeof(command),
            TMO_FEVR
        ) < E_OK) {

        robot_logf(
            (UB *)"[UART] failed to queue mode command\n"
        );
    }
}

static void process_uart_line(char *buffer)
{
    int idx = my_strlen(buffer);

    while (idx > 0 &&
           (buffer[idx - 1] == '\r' ||
            buffer[idx - 1] == '\n')) {
        buffer[--idx] = '\0';
    }

    if (idx == 0)
        return;

    char *star = my_strchr(buffer, '*');

    if (star == NULL || my_strlen(star + 1) != 2) {
        robot_logf(
            (UB *)"[UART RX] malformed frame "
                   "(expected *XX checksum)\n"
        );
        return;
    }

    int hi = uart_hex_value(star[1]);
    int lo = uart_hex_value(star[2]);

    if (hi < 0 || lo < 0) {
        robot_logf(
            (UB *)"[UART RX] invalid checksum characters\n"
        );
        return;
    }

    UB expected = 0;

    for (char *p = buffer; p < star; ++p)
        expected ^= (UB)*p;

    if (expected != (UB)((hi << 4) | lo)) {
        robot_logf(
            (UB *)"[UART RX] checksum mismatch: "
                   "calculated %02X, received %02X\n",
            expected,
            (hi << 4) | lo
        );
        return;
    }

    *star = '\0';

    char *start;

    if ((start = my_strstr(buffer, "A,")) != NULL) {
        start += 2;

        int id = parse_int(&start);
        int x = parse_int(&start);
        int y = parse_int(&start);

        (void)parse_float(&start);
        float dist = parse_float(&start);

        const char *direction = direction_for_marker(id);

        if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK)
            return;

        if (direction != NULL) {
            int slot = id - 1;

            if (!direction_seen[slot]) {
                direction_seen[slot] = TRUE;
                direction_count++;
            }

            direction_map[slot].id = id;
            direction_map[slot].x = x;
            direction_map[slot].y = y;
            direction_map[slot].distance = dist;

        } else if (id >= 4 &&
                   id < MAX_ARUCO_IDS &&
                   !released_object_ids[id]) {

            int slot = -1;

            for (int i = 0; i < object_count; ++i) {
                if (object_map[i].id == id) {
                    slot = i;
                    break;
                }
            }

            if (slot < 0 && object_count < MAX_OBJECTS) {
                slot = object_count++;
                object_map[slot].id = id;
            }

            if (slot >= 0) {
                object_map[slot].x = x;
                object_map[slot].y = y;
                object_map[slot].distance = dist;
            }
        }

        (void)tk_unl_mtx(mtx_id);
        return;
    }

    if ((start = my_strstr(buffer, "C,")) != NULL) {
        start += 2;
        char color = *start;

        if (color == 'N') {
            robot_logf(
                (UB *)"  -> No recognized payload color\n"
            );
        } else if (start[1] == ',') {
            start += 2;

            int x = parse_int(&start);
            int y = parse_int(&start);

            robot_logf(
                (UB *)"  -> Payload found: COLOR %c "
                       "at X:%d Y:%d\n",
                color, x, y
            );
        } else {
            robot_logf(
                (UB *)"[UART RX] malformed color frame\n"
            );
        }

        return;
    }

    robot_logf(
        (UB *)"[UART RX] valid checksum, unknown frame type\n"
    );
}

/* ---------- Camera UART tasks ---------- */

LOCAL void task_uart_tx(INT stacd, void *exinf)
{
    ModeCommand command;

    (void)stacd;
    (void)exinf;

    for (;;) {
        if (tk_rcv_mbf(
                command_mbf_id,
                &command,
                TMO_FEVR
            ) == (INT)sizeof(command)) {

            SZ written;

            ER err = tk_swri_dev(
                uart_device_id,
                0,
                command.bytes,
                sizeof(command.bytes),
                &written
            );

            if (err < E_OK) {
                robot_logf(
                    (UB *)"[UART] write error %d\n",
                    err
                );
            }
        }
    }
}

LOCAL void task_uart_rx(INT stacd, void *exinf)
{
    char buffer[RX_LINE_SIZE];
    char pending[32];

    SZ pending_size = 0;
    SZ pending_index = 0;

    int index = 0;
    UW discarded_bytes = 0;

    UB noise_sample[16];
    INT noise_sample_size = 0;

    (void)stacd;
    (void)exinf;

    for (;;) {
        if (pending_index >= pending_size) {
            SZ available = 0;

            ER err = tk_srea_dev(
                uart_device_id,
                0,
                NULL,
                0,
                &available
            );

            if (err < E_OK || available <= 0) {
                (void)tk_dly_tsk(1);
                continue;
            }

            pending_size =
                available < (SZ)sizeof(pending)
                    ? available
                    : (SZ)sizeof(pending);

            pending_index = 0;

            err = tk_srea_dev(
                uart_device_id,
                0,
                pending,
                pending_size,
                &pending_size
            );

            if (err < E_OK || pending_size <= 0) {
                pending_size = 0;
                pending_index = 0;
                (void)tk_dly_tsk(1);
                continue;
            }
        }

        while (pending_index < pending_size) {
            char ch = pending[pending_index++];

            if (ch == '\r' || ch == '\n') {
                if (index > 2) {
                    buffer[index] = '\0';
                    process_uart_line(buffer);
                }

                index = 0;
                continue;
            }

            if (index == 0) {
                if (ch == 'A' || ch == 'C') {
                    noise_sample_size = 0;
                    buffer[index++] = ch;
                    buffer[index] = '\0';
                } else {
                    if (noise_sample_size <
                        (INT)sizeof(noise_sample)) {
                        noise_sample[noise_sample_size++] =
                            (UB)ch;
                    }

                    discarded_bytes++;

                    if ((discarded_bytes & 127U) == 0U) {
                        static const char hex[] =
                            "0123456789ABCDEF";

                        robot_logf(
                            (UB *)"[UART RX] discarded %u "
                                   "bytes without a frame prefix; "
                                   "sample:",
                            discarded_bytes
                        );

                        for (INT i = 0;
                             i < noise_sample_size;
                             ++i) {
                            robot_logf(
                                (UB *)" %c%c",
                                hex[noise_sample[i] >> 4],
                                hex[noise_sample[i] & 0x0F]
                            );
                        }

                        robot_logf((UB *)"\n");
                        noise_sample_size = 0;
                    }
                }

                continue;
            }

            if (index == 1) {
                if (ch == ',') {
                    buffer[index++] = ch;
                    buffer[index] = '\0';
                } else if (ch == 'A' || ch == 'C') {
                    buffer[0] = ch;
                } else {
                    if (noise_sample_size <
                        (INT)sizeof(noise_sample)) {
                        noise_sample[noise_sample_size++] =
                            (UB)buffer[0];

                        if (noise_sample_size <
                            (INT)sizeof(noise_sample)) {
                            noise_sample[noise_sample_size++] =
                                (UB)ch;
                        }
                    }

                    index = 0;
                    discarded_bytes++;
                }

                continue;
            }

            if (my_strchr(buffer, '*') == NULL &&
                (ch == 'A' || ch == 'C')) {

                buffer[0] = ch;
                buffer[1] = '\0';
                index = 1;
                continue;
            }

            if (index < RX_LINE_SIZE - 1) {
                buffer[index++] = ch;
                buffer[index] = '\0';

                char *star = my_strchr(buffer, '*');

                if (star != NULL &&
                    (SZ)(buffer + index - (star + 1)) == 2) {

                    process_uart_line(buffer);
                    index = 0;
                }
            } else {
                static const char hex[] =
                    "0123456789ABCDEF";

                int sample = index < 16 ? index : 16;

                robot_logf(
                    (UB *)"[UART RX] oversized frame; "
                           "first %d bytes:",
                    sample
                );

                for (int i = 0; i < sample; ++i) {
                    UB value = (UB)buffer[i];

                    robot_logf(
                        (UB *)" %c%c",
                        hex[value >> 4],
                        hex[value & 0x0F]
                    );
                }

                robot_logf((UB *)"\n");
                index = 0;
            }
        }
    }
}

/* ---------- Drive task ---------- */

static INT drive_toward(INT current, INT target)
{
    if (target > current) {
        return target - current > DRIVE_RAMP_STEP
            ? current + DRIVE_RAMP_STEP
            : target;
    }

    if (target < current) {
        return current - target > DRIVE_RAMP_STEP
            ? current - DRIVE_RAMP_STEP
            : target;
    }

    return current;
}

/*
 * This task remains the sole runtime owner of motor PWM.
 * It checks the interlock while ramping and during each pulse.
 */
LOCAL void task_drive(INT stacd, void *exinf)
{
    DriveCommand command;
    INT current_left = 0;
    INT current_right = 0;

    (void)stacd;
    (void)exinf;

    for (;;) {
        if (!mission_motion_allowed()) {
            motor_brake();
            current_left = 0;
            current_right = 0;

            while (tk_rcv_mbf(
                       drive_mbf_id,
                       &command,
                       TMO_POL
                   ) == (INT)sizeof(command)) {
                /* Discard commands after a latched stop. */
            }

            (void)tk_dly_tsk(100);
            continue;
        }

        INT received = tk_rcv_mbf(
            drive_mbf_id,
            &command,
            10
        );

        if (received == E_TMOUT)
            continue;

        if (received != (INT)sizeof(command)) {
            mission_latch_stop("Drive command receive failed");
            motor_brake();
            current_left = 0;
            current_right = 0;
            continue;
        }

        if (!mission_motion_allowed()) {
            motor_brake();
            current_left = 0;
            current_right = 0;
            continue;
        }

        if (command.stop) {
            motor_brake();
            current_left = 0;
            current_right = 0;
            continue;
        }

        BOOL interrupted = FALSE;

        while (current_left != command.left ||
               current_right != command.right) {

            if (!mission_motion_allowed()) {
                interrupted = TRUE;
                break;
            }

            current_left =
                drive_toward(current_left, command.left);

            current_right =
                drive_toward(current_right, command.right);

            motor_drive(current_left, current_right);

            if (!mission_checked_delay(DRIVE_RAMP_MS)) {
                interrupted = TRUE;
                break;
            }
        }

        if (!interrupted && command.ms > 0) {
            (void)mission_checked_delay((UW)command.ms);
        }

        motor_brake();
        current_left = 0;
        current_right = 0;
    }
}

static void drive_pulse(
    INT left,
    INT right,
    INT duration_ms
)
{
    mission_checkpoint();

    DriveCommand command = {
        left, right, duration_ms, FALSE
    };

    if (tk_snd_mbf(
            drive_mbf_id,
            &command,
            sizeof(command),
            TMO_FEVR
        ) < E_OK) {

        mission_latch_stop("Could not queue motor command");
        mission_checkpoint();
        return;
    }

    UW wait_ms = (UW)(
        duration_ms + DRIVE_RAMP_MS * 11
    );

    if (!mission_checked_delay(wait_ms))
        mission_checkpoint();
}

static void drive_stop(void)
{
    DriveCommand command = {0, 0, 0, TRUE};

    if (tk_snd_mbf(
            drive_mbf_id,
            &command,
            sizeof(command),
            TMO_FEVR
        ) < E_OK) {
        mission_latch_stop("Could not queue stop command");
    }

    (void)tk_dly_tsk(20);
}

/* ---------- Camera scans and target selection ---------- */

static void clear_scan_results(void)
{
    if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK)
        return;

    object_count = 0;
    direction_count = 0;

    for (int i = 0; i < 3; ++i)
        direction_seen[i] = FALSE;

    (void)tk_unl_mtx(mtx_id);
}

static void request_marker_scan(
    const char *command,
    INT scan_ms
)
{
    mission_checkpoint();

    clear_scan_results();

    send_mode_command(command);
    (void)tk_dly_tsk(50);
    send_mode_command(command);
    (void)tk_dly_tsk((RELTIM)scan_ms);

    mission_checkpoint();
}

static BOOL nearest_object(marker_reading_t *result)
{
    BOOL found = FALSE;

    if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK)
        return FALSE;

    for (int i = 0; i < object_count; ++i) {
        if (object_map[i].id < 4)
            continue;

        if (!found ||
            object_map[i].distance < result->distance) {
            *result = object_map[i];
            found = TRUE;
        }
    }

    (void)tk_unl_mtx(mtx_id);
    return found;
}

static void mark_object_released(INT marker_id)
{
    if (marker_id < 4 || marker_id >= MAX_ARUCO_IDS)
        return;

    if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK)
        return;

    released_object_ids[marker_id] = TRUE;

    (void)tk_unl_mtx(mtx_id);
}

static BOOL nearest_direction(marker_reading_t *result)
{
    BOOL found = FALSE;

    if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK)
        return FALSE;

    for (int i = 0; i < 3; ++i) {
        if (!direction_seen[i])
            continue;

        if (!found ||
            direction_map[i].distance < result->distance) {
            *result = direction_map[i];
            found = TRUE;
        }
    }

    (void)tk_unl_mtx(mtx_id);
    return found;
}

static BOOL read_direction(
    INT id,
    marker_reading_t *result
)
{
    BOOL found = FALSE;

    if (id < 1 ||
        id > 3 ||
        tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK) {
        return FALSE;
    }

    if (direction_seen[id - 1]) {
        *result = direction_map[id - 1];
        found = TRUE;
    }

    (void)tk_unl_mtx(mtx_id);
    return found;
}

static INT clamp_int(
    INT value,
    INT minimum,
    INT maximum
)
{
    if (value < minimum)
        return minimum;

    if (value > maximum)
        return maximum;

    return value;
}

static BOOL search_direction_marker(
    INT marker_id,
    marker_reading_t *target
)
{
    mission_checkpoint();

    robot_logf(
        (UB *)"[mission] direction marker lost; "
               "searching with a small left/right sweep\n"
    );

    drive_pulse(
        -CAR_TURN_SPEED,
        CAR_TURN_SPEED,
        DIRECTION_SEARCH_PULSE_MS
    );

    request_marker_scan(
        DIRECTION_SCAN_COMMAND,
        SCAN_WINDOW_MS
    );

    if (read_direction(marker_id, target)) {
        robot_logf(
            (UB *)"[mission] direction marker "
                   "reacquired after left turn\n"
        );
        return TRUE;
    }

    drive_pulse(
        CAR_TURN_SPEED,
        -CAR_TURN_SPEED,
        DIRECTION_SEARCH_PULSE_MS * 2
    );

    request_marker_scan(
        DIRECTION_SCAN_COMMAND,
        SCAN_WINDOW_MS
    );

    if (read_direction(marker_id, target)) {
        robot_logf(
            (UB *)"[mission] direction marker "
                   "reacquired after right turn\n"
        );
        return TRUE;
    }

    drive_pulse(
        -CAR_TURN_SPEED,
        CAR_TURN_SPEED,
        DIRECTION_SEARCH_PULSE_MS
    );

    drive_stop();
    return FALSE;
}

static BOOL approach_marker(
    INT marker_id,
    INT standoff_mm,
    BOOL *gripper_opened,
    INT *reached_marker_id
)
{
    INT lost_scans = 0;

    for (INT step = 0;
         step < MAX_APPROACH_STEPS;
         ++step) {

        mission_checkpoint();

        marker_reading_t target = {0};

        request_marker_scan(
            marker_id == 0
                ? OBJECT_SCAN_COMMAND
                : DIRECTION_SCAN_COMMAND,
            SCAN_WINDOW_MS
        );

        BOOL found = marker_id == 0
            ? nearest_object(&target)
            : read_direction(marker_id, &target);

        if (!found) {
            drive_stop();

            if (marker_id == 0) {
                robot_logf(
                    (UB *)"[mission] no object visible; "
                           "slowly rotating right to scan\n"
                );

                drive_pulse(
                    OBJECT_SCAN_TURN_SPEED,
                    -OBJECT_SCAN_TURN_SPEED,
                    OBJECT_SCAN_TURN_PULSE_MS
                );

                drive_stop();
            } else {
                found = search_direction_marker(
                    marker_id,
                    &target
                );
            }

            if (!found) {
                if (++lost_scans >=
                    MAX_LOST_SCAN_ATTEMPTS) {

                    robot_logf(
                        (UB *)"[mission] target lost; "
                               "stopping approach\n"
                    );
                    return FALSE;
                }

                robot_logf(
                    (UB *)"[mission] target not visible; "
                           "rescanning\n"
                );
                continue;
            }
        }

        lost_scans = 0;

        if (marker_id == 0 &&
            gripper_opened != NULL &&
            !*gripper_opened) {

            mission_checkpoint();
            claw_open();

            *gripper_opened = TRUE;

            robot_logf(
                (UB *)"[gripper] object ID:%d seen; "
                       "opening at %d degrees\n",
                target.id,
                claw_get_angle()
            );

            (void)tk_dly_tsk(400);
            mission_checkpoint();
        }

        INT distance_mm =
            (INT)(target.distance + 0.5f);

        robot_logf(
            (UB *)"[mission] target ID:%d at %d mm "
                   "(stop limit %d mm)\n",
            target.id,
            distance_mm,
            standoff_mm
        );

        if (distance_mm <= standoff_mm) {
            robot_logf(
                (UB *)"[mission] stop limit reached "
                       "for target ID:%d\n",
                target.id
            );

            drive_stop();
            mission_checkpoint();

            if (reached_marker_id != NULL)
                *reached_marker_id = target.id;

            return TRUE;
        }

        INT error_x = target.x - CAMERA_CENTER_X;
        INT left;
        INT right;

        if (marker_id != 0 &&
            error_x >= -DIRECTION_ALIGN_TOLERANCE_PX &&
            error_x <= DIRECTION_ALIGN_TOLERANCE_PX) {

            left = CAR_DRIVE_SPEED;
            right = CAR_DRIVE_SPEED;
        } else {
            INT correction =
                clamp_int(error_x / 8, -12, 12);

            left = clamp_int(
                CAR_DRIVE_SPEED + correction,
                10,
                40
            );

            right = clamp_int(
                CAR_DRIVE_SPEED - correction,
                10,
                40
            );
        }

        robot_logf(
            (UB *)"[mission] drive correction x:%d; "
                   "L:%d R:%d\n",
            target.x,
            left,
            right
        );

        drive_pulse(left, right, DRIVE_PULSE_MS);
    }

    drive_stop();

    robot_logf(
        (UB *)"[mission] approach step limit reached; "
               "stopping\n"
    );

    return FALSE;
}

static BOOL align_direction_marker(INT marker_id)
{
    for (INT step = 0; step < 12; ++step) {
        mission_checkpoint();

        request_marker_scan(
            DIRECTION_SCAN_COMMAND,
            SCAN_WINDOW_MS
        );

        marker_reading_t target = {0};

        if (!read_direction(marker_id, &target)) {
            if (!search_direction_marker(
                    marker_id,
                    &target)) {

                drive_stop();

                robot_logf(
                    (UB *)"[mission] direction marker "
                           "not found during alignment\n"
                );
                return FALSE;
            }
        }

        INT error_x = target.x - CAMERA_CENTER_X;

        if (error_x >= -DIRECTION_ALIGN_TOLERANCE_PX &&
            error_x <= DIRECTION_ALIGN_TOLERANCE_PX) {

            drive_stop();

            robot_logf(
                (UB *)"[mission] direction marker "
                       "straight ahead, x:%d\n",
                target.x
            );

            return TRUE;
        }

        if (error_x < 0) {
            robot_logf(
                (UB *)"[mission] marker left, x:%d; "
                       "turning left\n",
                target.x
            );

            drive_pulse(
                -CAR_TURN_SPEED,
                CAR_TURN_SPEED,
                DIRECTION_ALIGN_PULSE_MS
            );
        } else {
            robot_logf(
                (UB *)"[mission] marker right, x:%d; "
                       "turning right\n",
                target.x
            );

            drive_pulse(
                CAR_TURN_SPEED,
                -CAR_TURN_SPEED,
                DIRECTION_ALIGN_PULSE_MS
            );
        }
    }

    drive_stop();

    robot_logf(
        (UB *)"[mission] could not align with "
               "direction marker; stopping\n"
    );

    return FALSE;
}

/* ---------- Mission task ---------- */

LOCAL void task_state_machine(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;) {
        mission_checkpoint();

        robot_logf(
            (UB *)"\n[mission] starting cycle; "
                   "looking for object ArUco IDs 4 and above\n"
        );

        robot_logf(
            (UB *)"[gripper] signal idle; "
                   "waiting for an object marker\n"
        );

        BOOL gripper_opened = FALSE;
        INT delivered_object_id = 0;

        while (!approach_marker(
                    0,
                    OBJECT_STANDOFF_MM,
                    &gripper_opened,
                    &delivered_object_id)) {

            mission_checkpoint();

            robot_logf(
                (UB *)"[mission] no object reached; "
                       "rescanning\n"
            );

            (void)tk_dly_tsk(1000);
        }

        mission_checkpoint();

        robot_logf(
            (UB *)"[mission] object reached; "
                   "closing gripper\n"
        );

        claw_close();

        robot_logf(
            (UB *)"[gripper] close setpoint %d degrees\n",
            claw_get_angle()
        );

        (void)tk_dly_tsk(400);
        claw_release();

        robot_logf(
            (UB *)"[gripper] close pulse finished; "
                   "servo signal released\n"
        );

        mission_checkpoint();

        for (;;) {
            marker_reading_t chosen_direction = {0};

            for (;;) {
                mission_checkpoint();

                request_marker_scan(
                    DIRECTION_SCAN_COMMAND,
                    SCAN_WINDOW_MS
                );

                if (nearest_direction(&chosen_direction))
                    break;

                drive_stop();

                robot_logf(
                    (UB *)"[mission] no direction marker "
                           "(IDs 1-3); rescanning\n"
                );

                (void)tk_dly_tsk(1000);
            }

            const char *direction_name =
                direction_for_marker(chosen_direction.id);

            robot_logf(
                (UB *)"[mission] nearest direction: %s "
                       "(ID:%d, %d mm)\n",
                direction_name,
                chosen_direction.id,
                (INT)(chosen_direction.distance + 0.5f)
            );

            BOOL aligned =
                align_direction_marker(chosen_direction.id);

            if (aligned) {
                robot_logf(
                    (UB *)"[mission] driving toward "
                           "%s marker\n",
                    direction_name
                );
            }

            if (aligned &&
                approach_marker(
                    chosen_direction.id,
                    DIRECTION_STANDOFF_MM,
                    NULL,
                    NULL)) {

                mission_checkpoint();

                robot_logf(
                    (UB *)"[mission] reached %s marker; "
                           "releasing object ID:%d\n",
                    direction_name,
                    delivered_object_id
                );

                claw_open();

                robot_logf(
                    (UB *)"[gripper] release setpoint "
                           "%d degrees\n",
                    claw_get_angle()
                );

                (void)tk_dly_tsk(400);
                claw_release();

                mission_checkpoint();

                mark_object_released(delivered_object_id);

                robot_logf(
                    (UB *)"[mission] reversing away "
                           "from delivery point\n"
                );

                drive_pulse(
                    -CAR_DRIVE_SPEED,
                    -CAR_DRIVE_SPEED,
                    REVERSE_AFTER_RELEASE_MS
                );

                drive_stop();
                break;
            }

            drive_stop();
            mission_checkpoint();

            robot_logf(
                (UB *)"[mission] stopped before reaching "
                       "direction marker; rescanning\n"
            );

            (void)tk_dly_tsk(1000);
        }

        drive_stop();
        mission_checkpoint();

        robot_logf(
            (UB *)"[mission] cycle complete; "
                   "starting next object scan\n"
        );

        (void)tk_dly_tsk(1000);
    }
}

/* ---------- Startup ---------- */

static const char *reset_cause(void)
{
    UW chip = *(volatile UW *)0x40064008;
    UW watchdog = *(volatile UW *)0x40058008;

    if (chip & (1U << 16))
        return "RUN pin/reset button";

    if (chip & (1U << 20))
        return "debugger";

    if (watchdog & 3U)
        return "watchdog/reboot";

    if (chip & (1U << 8))
        return "power-on or brown-out";

    return "unknown";
}

static BOOL start_application_task(
    ID *id,
    T_CTSK *definition,
    const char *name
)
{
    *id = tk_cre_tsk(definition);

    if (*id <= E_OK) {
        robot_logf(
            (UB *)"[startup] Could not create %s task: %d\n",
            name,
            *id
        );
        return FALSE;
    }

    ER er = tk_sta_tsk(*id, 0);

    if (er < E_OK) {
        robot_logf(
            (UB *)"[startup] Could not start %s task: %d\n",
            name,
            er
        );

        (void)tk_del_tsk(*id);
        *id = 0;
        return FALSE;
    }

    return TRUE;
}

EXPORT INT usermain(void)
{
    (void)InitSpinLock(&wifi_log_lock);
    (void)InitSpinLock(&mission_guard_lock);

    (void)tk_dly_tsk(5000);

    robot_logf(
        (UB *)"Pico W integrated vision + IMU mission "
               "starting (reset: %s)...\n",
        reset_cause()
    );

    ER motor_er = motor_init();
    ER claw_er = claw_init();

    if (motor_er < E_OK || claw_er < E_OK) {
        robot_logf(
            (UB *)"[init] drive hardware failed: "
                   "motor=%d claw=%d\n",
            motor_er,
            claw_er
        );

        mission_latch_stop("Hardware initialisation failed");
        mission_park();
    }

    robot_logf(
        (UB *)"[init] gripper PWM ready; "
               "signal idle until an object marker is detected\n"
    );

    ER imu_er = imu_heading_start();

    if (imu_er < E_OK) {
        robot_logf(
            (UB *)"[startup] IMU task start failed: %d\n",
            imu_er
        );

        mission_latch_stop("IMU task could not start");
        mission_park();
    }

    if (!wait_for_imu_startup()) {
        mission_latch_stop("IMU startup checks failed");
        mission_park();
    }

    uart_device_id = tk_opn_dev(
        (const UB *)"sera",
        TD_READ | TD_WRITE
    );

    if (uart_device_id <= E_OK) {
        robot_logf(
            (UB *)"[init] cannot open UART0 device: %d\n",
            uart_device_id
        );

        mission_latch_stop("Camera UART could not open");
        mission_park();
    }

    command_mbf_id = tk_cre_mbf(&cmbf);
    mtx_id = tk_cre_mtx(&cmtx);
    drive_mbf_id = tk_cre_mbf(&drive_cmbf);

    if (command_mbf_id <= E_OK ||
        mtx_id <= E_OK ||
        drive_mbf_id <= E_OK) {

        robot_logf(
            (UB *)"[init] kernel object creation failed: "
                   "uart_mbf=%d mtx=%d drive_mbf=%d\n",
            command_mbf_id,
            mtx_id,
            drive_mbf_id
        );

        mission_latch_stop("Application object creation failed");
        mission_park();
    }

    if (!start_application_task(
            &task_uart_rx_id,
            &ctsk_uart_rx,
            "camera RX") ||
        !start_application_task(
            &task_uart_tx_id,
            &ctsk_uart_tx,
            "camera TX") ||
        !start_application_task(
            &task_drive_id,
            &ctsk_drive,
            "drive") ||
        !start_application_task(
            &task_imu_monitor_id,
            &ctsk_imu_monitor,
            "IMU monitor")) {

        mission_latch_stop("Application task startup failed");
        mission_park();
    }

    if (!mission_motion_allowed())
        mission_park();

    robot_logf(
        (UB *)"[mission] ENABLED: IMU ready; "
               "starting camera-controlled mission.\n"
    );

    if (!start_application_task(
            &task_state_id,
            &ctsk_state,
            "mission")) {

        mission_latch_stop("Mission task startup failed");
        mission_park();
    }

    mission_park();
    return 0;
}