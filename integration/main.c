#include <tk/tkernel.h>
#include "../third_party/mtk3smp-rp2040/device/include/dev_i2c.h"
#include "../components/imu/imu_heading.h"
#include <tm/tmonitor.h>
#include <tk/smp.h>
#include "../components/drive/motor.h"
#include "../components/drive/claw.h"

#include <stdarg.h>

#define WIFI_LOG_QUEUE_SIZE 16U
#define WIFI_LOG_LINE_SIZE  240U

typedef struct {
    UW length;
    UB bytes[WIFI_LOG_LINE_SIZE];
} WifiLogEntry;

static WifiLogEntry wifi_log_queue[WIFI_LOG_QUEUE_SIZE];
static volatile UW wifi_log_head;
static volatile UW wifi_log_tail;
static volatile UW wifi_log_count;
static T_SPLOCK wifi_log_lock;

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

    if(length == 0) return;
    if(length > WIFI_LOG_LINE_SIZE) length = WIFI_LOG_LINE_SIZE;

    wifi_log_lock_queue();
    if(wifi_log_count == WIFI_LOG_QUEUE_SIZE) {
        wifi_log_tail = (wifi_log_tail + 1U) % WIFI_LOG_QUEUE_SIZE;
        wifi_log_count--;
    }
    slot = wifi_log_head;
    for(UW i = 0; i < length; i++) wifi_log_queue[slot].bytes[i] = bytes[i];
    wifi_log_queue[slot].length = length;
    wifi_log_head = (wifi_log_head + 1U) % WIFI_LOG_QUEUE_SIZE;
    wifi_log_count++;
    wifi_log_unlock_queue();
}

static void robot_log_append(UB *output, UW capacity, UW *length,
                             const UB *text, UW text_length)
{
    while(text_length != 0 && *length < capacity) {
        output[(*length)++] = *text++;
        text_length--;
    }
}

static INT robot_log_vformat(UB *output, UW capacity, const UB *format,
                             va_list args)
{
    UB temporary[96];
    UW length = 0;

    while(*format != '\0' && length < capacity) {
        UB specifier[16];
        UW spec_length = 0;
        INT formatted_length;
        UB conversion;

        if(*format != '%') {
            robot_log_append(output, capacity, &length, format++, 1);
            continue;
        }

        format++;
        if(*format == '%') {
            robot_log_append(output, capacity, &length, format++, 1);
            continue;
        }
        if(*format == '\0') {
            robot_log_append(output, capacity, &length, (const UB*)"%", 1);
            break;
        }

        specifier[spec_length++] = '%';
        while(*format == '0' || *format == '-' || *format == '+'
              || *format == ' ' || *format == '#') {
            if(spec_length + 2U >= sizeof(specifier)) break;
            specifier[spec_length++] = *format++;
        }
        while(*format >= '0' && *format <= '9') {
            if(spec_length + 2U >= sizeof(specifier)) break;
            specifier[spec_length++] = *format++;
        }
        conversion = *format++;
        specifier[spec_length++] = conversion;
        specifier[spec_length] = '\0';

        switch(conversion) {
        case 'd':
        case 'i':
            formatted_length = tm_sprintf(temporary, specifier,
                                          va_arg(args, INT));
            break;
        case 'u':
        case 'o':
        case 'x':
        case 'X':
            formatted_length = tm_sprintf(temporary, specifier,
                                          va_arg(args, UW));
            break;
        case 'c':
            formatted_length = tm_sprintf(temporary, specifier,
                                          va_arg(args, INT));
            break;
        case 's':
        {
            const UB *text = (const UB*)va_arg(args, const char*);
            if(text == NULL) text = (const UB*)"(null)";
            while(*text != '\0' && length < capacity) {
                output[length++] = *text++;
            }
            continue;
        }
        default:
            robot_log_append(output, capacity, &length,
                             (const UB*)"%", 1);
            robot_log_append(output, capacity, &length,
                             &conversion, 1);
            continue;
        }
        if(formatted_length > 0) {
            robot_log_append(output, capacity, &length, temporary,
                             (UW)formatted_length);
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
    length = robot_log_vformat(line, WIFI_LOG_LINE_SIZE, format, args);
    va_end(args);

    (void)tm_printf((UB *)"%s", line);
    wifi_log_enqueue(line, (UW)length);
    return length;
}

int robot_log_dequeue(unsigned char *bytes, unsigned int capacity,
                      unsigned int *length)
{
    WifiLogEntry entry;

    wifi_log_lock_queue();
    if(wifi_log_count == 0) {
        wifi_log_unlock_queue();
        return 0;
    }
    entry = wifi_log_queue[wifi_log_tail];
    wifi_log_tail = (wifi_log_tail + 1U) % WIFI_LOG_QUEUE_SIZE;
    wifi_log_count--;
    wifi_log_unlock_queue();

    if(entry.length > capacity) entry.length = capacity;
    for(unsigned int i = 0; i < entry.length; i++) bytes[i] = entry.bytes[i];
    *length = entry.length;
    return 1;
}

// ======================================================
// Kernel Objects
// ======================================================
LOCAL ID task_uart_rx_id;
LOCAL ID task_uart_tx_id;
LOCAL ID task_state_id;
LOCAL ID task_drive_id;
LOCAL ID uart_device_id;
LOCAL ID command_mbf_id;
LOCAL ID mtx_id;
LOCAL ID drive_mbf_id;

#define RX_LINE_SIZE 128
#define OBJECT_SCAN_COMMAND       "M,3\n"  /* block markers are 30 mm */
#define DIRECTION_SCAN_COMMAND    "M,8\n"  /* direction markers are 80 mm */
#define SCAN_WINDOW_MS            1200
#define OBJECT_STANDOFF_MM        100  /* close gripper 10 cm from object */
#define DIRECTION_STANDOFF_MM     200  /* release object 20 cm from marker */
#define DIRECTION_ALIGN_TOLERANCE_PX 74 /* about +/-15 degrees from camera center */
#define DIRECTION_ALIGN_PULSE_MS    70 /* short turn, then recheck camera bearing */
#define DIRECTION_SEARCH_PULSE_MS   70 /* small left/right sweep when marker is lost */
#define REVERSE_AFTER_RELEASE_MS   300
#define OBJECT_SCAN_TURN_SPEED     18
#define OBJECT_SCAN_TURN_PULSE_MS  150
#define CAR_DRIVE_SPEED           25
#define CAR_TURN_SPEED            20
#define CAR_TURN_90_MS            400  /* calibrate on the floor */
#define DRIVE_PULSE_MS            100
#define MAX_APPROACH_STEPS        30
#define MAX_LOST_SCAN_ATTEMPTS    3
#define CAMERA_CENTER_X           160

typedef struct { UB bytes[4]; } ModeCommand;
LOCAL UW command_mbf_buffer[(4 * sizeof(ModeCommand)) / sizeof(UW) + 16];
typedef struct { INT left; INT right; INT ms; BOOL stop; } DriveCommand;
LOCAL UW drive_mbf_buffer[(4 * (sizeof(DriveCommand) + 4)) / sizeof(UW) + 8];

// Task 1: UART RX (Reads data)
LOCAL void task_uart_rx(INT stacd, void *exinf);
LOCAL T_CTSK ctsk_uart_rx = {
    .itskpri = 5, .stksz = 4096, .task = task_uart_rx,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

// Task 2: UART TX (Sends commands)
LOCAL void task_uart_tx(INT stacd, void *exinf);
LOCAL T_CTSK ctsk_uart_tx = {
    .itskpri = 6, .stksz = 4096, .task = task_uart_tx,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

// Task 3: State Machine
LOCAL void task_state_machine(INT stacd, void *exinf);
LOCAL T_CTSK ctsk_state = {
    .itskpri = 10, .stksz = 4096, .task = task_state_machine,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC2,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL void task_drive(INT stacd, void *exinf);
LOCAL T_CTSK ctsk_drive = {
    .itskpri = 5, .stksz = 4096, .task = task_drive,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC2,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CMBF cmbf = {
    .bufsz = sizeof(command_mbf_buffer), .maxmsz = sizeof(ModeCommand),
    .mbfatr = TA_TFIFO, .bufptr = command_mbf_buffer,
};
LOCAL T_CMTX cmtx = { .mtxatr = TA_TFIFO | TA_INHERIT };
LOCAL T_CMBF drive_cmbf = {
    .bufsz = sizeof(drive_mbf_buffer), .maxmsz = sizeof(DriveCommand),
    .mbfatr = TA_TFIFO, .bufptr = drive_mbf_buffer,
};

// ======================================================
// Shared Variables
// ======================================================
#define MAX_OBJECTS 16
#define MAX_ARUCO_IDS 1000
typedef struct { int id; int x; int y; float distance; } marker_reading_t;
static marker_reading_t object_map[MAX_OBJECTS];
static int object_count = 0;
static BOOL released_object_ids[MAX_ARUCO_IDS];
static marker_reading_t direction_map[3];
static BOOL direction_seen[3];
static int direction_count = 0;
static const char *direction_for_marker(int id)
{
    switch (id) {
    case 1: return "North";
    case 2: return "West";
    case 3: return "East";
    default: return NULL;
    }
}

void send_mode_command(const char* mode) {
    ModeCommand command = {{0, 0, 0, 0}};
    UW len = 0;
    while (mode[len] != '\0' && len < sizeof(command.bytes)) {
        command.bytes[len] = (UB)mode[len];
        ++len;
    }
    if (len == 0 || tk_snd_mbf(command_mbf_id, &command, sizeof(command), TMO_FEVR) < E_OK) {
        robot_logf((UB*)"[UART] failed to queue mode command\n");
    }
}

// ======================================================
// Custom Parsing Helpers
// ======================================================
int my_strlen(const char *s) { int len=0; while(*s++) len++; return len; }
char* my_strchr(const char *s, int c) { while (*s != (char)c) { if (!*s++) return 0; } return (char*)s; }
char* my_strstr(const char *haystack, const char *needle) {
    if (!*needle) return (char*)haystack;
    char *p1 = (char*)haystack;
    while (*p1) {
        char *p1_begin = p1; char *p2 = (char*)needle;
        while (*p1 && *p2 && *p1 == *p2) { p1++; p2++; }
        if (!*p2) return p1_begin; p1 = p1_begin + 1;
    }
    return 0;
}
int parse_int(char **str) {
    int res = 0; while (**str >= '0' && **str <= '9') { res = res * 10 + (**str - '0'); (*str)++; }
    if (**str == ',') (*str)++; return res;
}
float parse_float(char **str) {
    float res = 0.0f; int sign = 1; if (**str == '-') { sign = -1; (*str)++; }
    while (**str >= '0' && **str <= '9') { res = res * 10.0f + (**str - '0'); (*str)++; }
    if (**str == '.') { (*str)++; float frac = 0.1f; while (**str >= '0' && **str <= '9') { res += (**str - '0') * frac; frac *= 0.1f; (*str)++; } }
    if (**str == ',') (*str)++; return res * sign;
}

static int uart_hex_value(char ch)
{
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

static void process_uart_line(char *buffer)
{
    int idx = my_strlen(buffer);
    while (idx > 0 && (buffer[idx - 1] == '\r' || buffer[idx - 1] == '\n'))
        buffer[--idx] = '\0';
    if (idx == 0) return;

    char *star = my_strchr(buffer, '*');
    if (star == NULL || my_strlen(star + 1) != 2) {
        robot_logf((UB*)"[UART RX] malformed frame (expected *XX checksum)\n");
        return;
    }

    int hi = uart_hex_value(star[1]);
    int lo = uart_hex_value(star[2]);
    if (hi < 0 || lo < 0) {
        robot_logf((UB*)"[UART RX] invalid checksum characters\n");
        return;
    }
    UB expected = 0;
    for (char *p = buffer; p < star; ++p) expected ^= (UB)*p;
    if (expected != (UB)((hi << 4) | lo)) {
        robot_logf((UB*)"[UART RX] checksum mismatch: calculated %02X, received %02X\n",
                  expected, (hi << 4) | lo);
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
        if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK) return;
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
        } else if (id >= 4 && id < MAX_ARUCO_IDS && !released_object_ids[id]) {
            int slot = -1;
            for (int i = 0; i < object_count; ++i) {
                if (object_map[i].id == id) { slot = i; break; }
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
            robot_logf((UB*)"  -> No recognized payload color\n");
        } else if (start[1] == ',') {
            start += 2;
            int x = parse_int(&start);
            int y = parse_int(&start);
            robot_logf((UB*)"  -> Payload found: COLOR %c at X:%d Y:%d\n", color, x, y);
        } else {
            robot_logf((UB*)"[UART RX] malformed color frame\n");
        }
        return;
    }
    robot_logf((UB*)"[UART RX] valid checksum, unknown frame type\n");
}

// ======================================================
// TX Task (Only sends data)
// ======================================================
LOCAL void task_uart_tx(INT stacd, void *exinf) {
    ModeCommand command;
    (void)stacd; (void)exinf;
    while(1) {
        if (tk_rcv_mbf(command_mbf_id, &command, TMO_FEVR) == sizeof(command)) {
            SZ written;
            ER err = tk_swri_dev(uart_device_id, 0, command.bytes, sizeof(command.bytes), &written);
            if (err < E_OK) robot_logf((UB*)"[UART] write error %d\n", err);
        }
    }
}

// ======================================================
// Drive output task: the only runtime task that controls motor PWM
// ======================================================
#define DRIVE_RAMP_STEP 10
#define DRIVE_RAMP_MS   15

static INT drive_toward(INT current, INT target)
{
    if (target > current)
        return target - current > DRIVE_RAMP_STEP ? current + DRIVE_RAMP_STEP : target;
    if (target < current)
        return current - target > DRIVE_RAMP_STEP ? current - DRIVE_RAMP_STEP : target;
    return current;
}

LOCAL void task_drive(INT stacd, void *exinf)
{
    DriveCommand command;
    INT current_left = 0, current_right = 0;
    (void)stacd; (void)exinf;

    for (;;) {
        INT received = tk_rcv_mbf(drive_mbf_id, &command, TMO_FEVR);
        if (received != (INT)sizeof(command)) continue;
        if (command.stop) {
            motor_brake();
            current_left = current_right = 0;
            continue;
        }

        while (current_left != command.left || current_right != command.right) {
            current_left = drive_toward(current_left, command.left);
            current_right = drive_toward(current_right, command.right);
            motor_drive(current_left, current_right);
            tk_dly_tsk(DRIVE_RAMP_MS);
        }
        if (command.ms > 0) tk_dly_tsk((RELTIM)command.ms);
        motor_brake();
        current_left = current_right = 0;
    }
}

static void drive_pulse(INT left, INT right, INT duration_ms)
{
    DriveCommand command = { left, right, duration_ms, FALSE };
    if (tk_snd_mbf(drive_mbf_id, &command, sizeof(command), TMO_FEVR) < E_OK) {
        robot_logf((UB*)"[drive] failed to queue motor command\n");
        return;
    }
    /* Include the bounded soft-start time so the next scan starts after braking. */
    tk_dly_tsk((RELTIM)(duration_ms + DRIVE_RAMP_MS * 11));
}

static void drive_stop(void)
{
    DriveCommand command = { 0, 0, 0, TRUE };
    (void)tk_snd_mbf(drive_mbf_id, &command, sizeof(command), TMO_FEVR);
    tk_dly_tsk(20);
}

static void clear_scan_results(void)
{
    if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK) return;
    object_count = 0;
    direction_count = 0;
    for (int i = 0; i < 3; ++i) direction_seen[i] = FALSE;
    (void)tk_unl_mtx(mtx_id);
}

static void request_marker_scan(const char *command, INT scan_ms)
{
    clear_scan_results();
    send_mode_command(command);
    tk_dly_tsk(50);
    send_mode_command(command);
    tk_dly_tsk((RELTIM)scan_ms);
}

static BOOL nearest_object(marker_reading_t *result)
{
    BOOL found = FALSE;
    if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK) return FALSE;
    for (int i = 0; i < object_count; ++i) {
        if (object_map[i].id < 4) continue;
        if (!found || object_map[i].distance < result->distance) {
            *result = object_map[i];
            found = TRUE;
        }
    }
    (void)tk_unl_mtx(mtx_id);
    return found;
}

static void mark_object_released(INT marker_id)
{
    if (marker_id < 4 || marker_id >= MAX_ARUCO_IDS) return;
    if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK) return;
    released_object_ids[marker_id] = TRUE;
    (void)tk_unl_mtx(mtx_id);
}

static BOOL nearest_direction(marker_reading_t *result)
{
    BOOL found = FALSE;
    if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK) return FALSE;
    for (int i = 0; i < 3; ++i) {
        if (!direction_seen[i]) continue;
        if (!found || direction_map[i].distance < result->distance) {
            *result = direction_map[i];
            found = TRUE;
        }
    }
    (void)tk_unl_mtx(mtx_id);
    return found;
}

static BOOL read_direction(INT id, marker_reading_t *result)
{
    BOOL found = FALSE;
    if (id < 1 || id > 3 || tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK) return FALSE;
    if (direction_seen[id - 1]) {
        *result = direction_map[id - 1];
        found = TRUE;
    }
    (void)tk_unl_mtx(mtx_id);
    return found;
}

static INT clamp_int(INT value, INT minimum, INT maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static BOOL search_direction_marker(INT marker_id, marker_reading_t *target)
{
    robot_logf((UB*)"[mission] direction marker lost; searching with a small left/right sweep\n");

    drive_pulse(-CAR_TURN_SPEED, CAR_TURN_SPEED, DIRECTION_SEARCH_PULSE_MS);
    request_marker_scan(DIRECTION_SCAN_COMMAND, SCAN_WINDOW_MS);
    if (read_direction(marker_id, target)) {
        robot_logf((UB*)"[mission] direction marker reacquired after left turn\n");
        return TRUE;
    }

    drive_pulse(CAR_TURN_SPEED, -CAR_TURN_SPEED,
                DIRECTION_SEARCH_PULSE_MS * 2);
    request_marker_scan(DIRECTION_SCAN_COMMAND, SCAN_WINDOW_MS);
    if (read_direction(marker_id, target)) {
        robot_logf((UB*)"[mission] direction marker reacquired after right turn; continuing approach\n");
        return TRUE;
    }

    /* No marker found: restore the starting heading before the next retry. */
    drive_pulse(-CAR_TURN_SPEED, CAR_TURN_SPEED, DIRECTION_SEARCH_PULSE_MS);
    drive_stop();
    return FALSE;
}

/* Approach using repeated camera updates and short, braked motor pulses.
   marker_id == 0 selects the closest object (ID > 4); otherwise it selects
   the requested direction marker (ID 1..3). */
static BOOL approach_marker(INT marker_id, INT standoff_mm, BOOL *gripper_opened,
                            INT *reached_marker_id)
{
    INT lost_scans = 0;
    for (INT step = 0; step < MAX_APPROACH_STEPS; ++step) {
        marker_reading_t target = {0};
        request_marker_scan(marker_id == 0 ? OBJECT_SCAN_COMMAND : DIRECTION_SCAN_COMMAND,
                            SCAN_WINDOW_MS);
        BOOL found = marker_id == 0 ? nearest_object(&target)
                                    : read_direction(marker_id, &target);
        if (!found) {
            drive_stop();
            if (marker_id == 0) {
                robot_logf((UB*)"[mission] no object visible; slowly rotating right to scan surroundings\n");
                drive_pulse(OBJECT_SCAN_TURN_SPEED, -OBJECT_SCAN_TURN_SPEED,
                            OBJECT_SCAN_TURN_PULSE_MS);
                drive_stop();
            } else {
                found = search_direction_marker(marker_id, &target);
            }
            if (!found) {
                if (++lost_scans >= MAX_LOST_SCAN_ATTEMPTS) {
                    robot_logf((UB*)"[mission] target lost; stopping approach\n");
                    return FALSE;
                }
                robot_logf((UB*)"[mission] target not visible; rescanning\n");
                continue;
            }
        }
        lost_scans = 0;
        if (marker_id == 0 && gripper_opened != NULL && !*gripper_opened) {
            claw_open();
            *gripper_opened = TRUE;
            robot_logf((UB*)"[gripper] object marker ID:%d seen; opening at %d degrees\n",
                      target.id, claw_get_angle());
            tk_dly_tsk(400);
        }
        INT distance_mm = (INT)(target.distance + 0.5f);
        robot_logf((UB*)"[mission] target ID:%d at %d mm (stop limit %d mm)\n",
                  target.id, distance_mm, standoff_mm);
        if (distance_mm <= standoff_mm) {
            robot_logf((UB*)"[mission] stop limit reached for target ID:%d\n",
                      target.id);
            drive_stop();
            if (reached_marker_id != NULL) *reached_marker_id = target.id;
            return TRUE;
        }

        INT error_x = target.x - CAMERA_CENTER_X;
        INT left;
        INT right;
        if (marker_id != 0 &&
            error_x >= -DIRECTION_ALIGN_TOLERANCE_PX &&
            error_x <= DIRECTION_ALIGN_TOLERANCE_PX) {
            /* Direction markers within +/-15 degrees mean drive straight. */
            left = right = CAR_DRIVE_SPEED;
        } else {
            INT correction = clamp_int(error_x / 8, -12, 12);
            left = clamp_int(CAR_DRIVE_SPEED + correction, 10, 40);
            right = clamp_int(CAR_DRIVE_SPEED - correction, 10, 40);
        }
        robot_logf((UB*)"[mission] drive correction x:%d; L:%d R:%d\n",
                  target.x, left, right);
        drive_pulse(left, right, DRIVE_PULSE_MS);
    }
    drive_stop();
    robot_logf((UB*)"[mission] approach step limit reached; stopping\n");
    return FALSE;
}

static const char *reset_cause(void)
{
    UW chip = *(volatile UW *)0x40064008;
    UW watchdog = *(volatile UW *)0x40058008;
    if (chip & (1U << 16)) return "RUN pin/reset button";
    if (chip & (1U << 20)) return "debugger";
    if (watchdog & 3U) return "watchdog/reboot";
    if (chip & (1U << 8)) return "power-on or brown-out";
    return "unknown";
}

static BOOL align_direction_marker(INT marker_id)
{
    for (INT step = 0; step < 12; ++step) {
        request_marker_scan(DIRECTION_SCAN_COMMAND, SCAN_WINDOW_MS);
        marker_reading_t target = {0};
        if (!read_direction(marker_id, &target)) {
            if (!search_direction_marker(marker_id, &target)) {
                drive_stop();
                robot_logf((UB*)"[mission] direction marker not found during alignment\n");
                return FALSE;
            }
        }

        INT error_x = target.x - CAMERA_CENTER_X;
        if (error_x >= -DIRECTION_ALIGN_TOLERANCE_PX &&
            error_x <= DIRECTION_ALIGN_TOLERANCE_PX) {
            drive_stop();
            robot_logf((UB*)"[mission] direction marker is straight ahead (x:%d; tolerance +/-15 deg)\n",
                      target.x);
            return TRUE;
        }

        if (error_x < 0) {
            robot_logf((UB*)"[mission] marker is left of the +/-15 deg window (x:%d); turning left\n",
                      target.x);
            drive_pulse(-CAR_TURN_SPEED, CAR_TURN_SPEED, DIRECTION_ALIGN_PULSE_MS);
        } else {
            robot_logf((UB*)"[mission] marker is right of the +/-15 deg window (x:%d); turning right\n",
                      target.x);
            drive_pulse(CAR_TURN_SPEED, -CAR_TURN_SPEED, DIRECTION_ALIGN_PULSE_MS);
        }
    }

    drive_stop();
    robot_logf((UB*)"[mission] could not align with direction marker; stopping\n");
    return FALSE;
}

// ======================================================
// RX Task (Only reads data)
// ======================================================
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

    (void)stacd; (void)exinf;

    while(1) {
        if (pending_index >= pending_size) {
            SZ available = 0;
            ER err = tk_srea_dev(uart_device_id, 0, NULL, 0, &available);
            if (err < E_OK || available <= 0) {
                tk_dly_tsk(1);
                continue;
            }

            pending_size = available < (SZ)sizeof(pending)
                         ? available : (SZ)sizeof(pending);
            pending_index = 0;
            err = tk_srea_dev(uart_device_id, 0, pending, pending_size,
                              &pending_size);
            if (err < E_OK || pending_size <= 0) {
                pending_size = pending_index = 0;
                tk_dly_tsk(1);
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

            /* Only these prefixes are emitted by the XIAO vision UART. */
            if (index == 0) {
                if (ch == 'A' || ch == 'C') {
                    noise_sample_size = 0;
                    buffer[index++] = ch;
                    buffer[index] = '\0';
                } else {
                    if (noise_sample_size < (INT)sizeof(noise_sample))
                        noise_sample[noise_sample_size++] = (UB)ch;
                    discarded_bytes++;
                    if ((discarded_bytes & 127U) == 0) {
                        static const char hex[] = "0123456789ABCDEF";
                        robot_logf((UB*)"[UART RX] discarded %u bytes without a frame prefix; sample:",
                                  discarded_bytes);
                        for (INT i = 0; i < noise_sample_size; ++i) {
                            robot_logf((UB*)" %c%c", hex[noise_sample[i] >> 4],
                                      hex[noise_sample[i] & 0x0F]);
                        }
                        robot_logf((UB*)"\n");
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
                    if (noise_sample_size < (INT)sizeof(noise_sample)) {
                        noise_sample[noise_sample_size++] = (UB)buffer[0];
                        if (noise_sample_size < (INT)sizeof(noise_sample))
                            noise_sample[noise_sample_size++] = (UB)ch;
                    }
                    index = 0;
                    discarded_bytes++;
                }
                continue;
            }

            /* Recover immediately if a damaged frame runs into the next one. */
            if (my_strchr(buffer, '*') == NULL && (ch == 'A' || ch == 'C')) {
                buffer[0] = ch;
                buffer[1] = '\0';
                index = 1;
                continue;
            }

            if (index < RX_LINE_SIZE - 1) {
                buffer[index++] = ch;
                buffer[index] = '\0';
                /* Process a complete checksummed frame even if LF is lost. */
                char *star = my_strchr(buffer, '*');
                if (star != NULL && (SZ)(buffer + index - (star + 1)) == 2) {
                    process_uart_line(buffer);
                    index = 0;
                }
            } else {
                static const char hex[] = "0123456789ABCDEF";
                int sample = index < 16 ? index : 16;
                robot_logf((UB*)"[UART RX] oversized frame; first %d bytes:", sample);
                for (int i = 0; i < sample; ++i) {
                    UB value = (UB)buffer[i];
                    robot_logf((UB*)" %c%c", hex[value >> 4], hex[value & 0x0F]);
                }
                robot_logf((UB*)"\n");
                index = 0;
            }
        }
    }
}

// ======================================================
// Task: State Machine
// ======================================================
LOCAL void task_state_machine(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
    for (;;) {
        robot_logf((UB*)"\n[mission] starting cycle; looking for object ArUco IDs 4 and above\n");
        robot_logf((UB*)"[gripper] signal idle; waiting for an object marker\n");
        BOOL gripper_opened = FALSE;

        INT delivered_object_id = 0;
        while (!approach_marker(0, OBJECT_STANDOFF_MM, &gripper_opened,
                                &delivered_object_id)) {
            robot_logf((UB*)"[mission] no object reached; rescanning\n");
            tk_dly_tsk(1000);
        }

        robot_logf((UB*)"[mission] object reached; closing gripper\n");
        claw_close();
        robot_logf((UB*)"[gripper] close setpoint %d degrees\n", claw_get_angle());
        tk_dly_tsk(400);
        claw_release();
        robot_logf((UB*)"[gripper] close pulse finished; servo signal released\n");

        for (;;) {
            marker_reading_t chosen_direction = {0};
            for (;;) {
                request_marker_scan(DIRECTION_SCAN_COMMAND, SCAN_WINDOW_MS);
                if (nearest_direction(&chosen_direction)) break;
                drive_stop();
                robot_logf((UB*)"[mission] no direction marker (IDs 1-3); rescanning\n");
                tk_dly_tsk(1000);
            }

            const char *direction_name = direction_for_marker(chosen_direction.id);
            robot_logf((UB*)"[mission] nearest direction: %s (ID:%d, %d mm)\n",
                      direction_name, chosen_direction.id,
                      (INT)(chosen_direction.distance + 0.5f));
            BOOL aligned = align_direction_marker(chosen_direction.id);
            if (aligned) {
                robot_logf((UB*)"[mission] driving toward %s marker\n", direction_name);
            }
            if (aligned && approach_marker(chosen_direction.id,
                                           DIRECTION_STANDOFF_MM, NULL, NULL)) {
                robot_logf((UB*)"[mission] reached %s marker at 20 cm; releasing object ID:%d\n",
                          direction_name, delivered_object_id);
                claw_open();
                robot_logf((UB*)"[gripper] opened to release at %d degrees\n",
                          claw_get_angle());
                tk_dly_tsk(400);
                claw_release();
                robot_logf((UB*)"[gripper] release pulse finished; servo signal released\n");
                mark_object_released(delivered_object_id);
                robot_logf((UB*)"[mission] reversing away from delivery point\n");
                drive_pulse(-CAR_DRIVE_SPEED, -CAR_DRIVE_SPEED,
                            REVERSE_AFTER_RELEASE_MS);
                drive_stop();
                break;
            }

            drive_stop();
            robot_logf((UB*)"[mission] stopped before reaching direction marker; rescanning\n");
            tk_dly_tsk(1000);
        }

        drive_stop();
        robot_logf((UB*)"[mission] cycle complete; starting next object scan\n");
        tk_dly_tsk(1000);
    }
}

// ======================================================
// Entry Point
// ======================================================
EXPORT INT usermain(void)
{
    tk_dly_tsk(5000);
    tm_printf((UB*)"[IMU TEST] reached usermain\n");
    robot_logf((UB*)"Pico W vision app on micro T-Kernel SMP starting (reset: %s)...\n",
              reset_cause());

    ER motor_er = motor_init();
    ER claw_er = claw_init();
    if (motor_er < E_OK || claw_er < E_OK) {
        robot_logf((UB*)"[init] drive hardware failed: motor=%d claw=%d\n",
                  motor_er, claw_er);
        return 0;
    }
    robot_logf((UB*)"[init] gripper PWM ready; signal idle until an object marker is detected\n");
    // wait for the IMU to be detected before starting the mission
    ER imu_er = imu_heading_start();

    for (;;) {
        imu_heading_reading_t s = {0};

        if (imu_er < E_OK) {
            robot_logf((UB*)
                "[IMU] task start failed: %d; mission held\n",
                imu_er);
        } else {
            imu_heading_read(&s);

            if (s.fault < E_OK) {
                robot_logf((UB*)
                    "[IMU] FAULT=%d; mission held; reset to retry\n",
                    s.fault);
            } else if (!s.ready) {
                robot_logf((UB*)
                    "[IMU] WAIT; M_ERR=%u REJ=%u; mission held\n",
                    s.mag_errors, s.rejected);
            } else {
                robot_logf((UB*)
                    "[IMU] %s H_cd=%d REL_cd=%d GYRO_cd=%d MAG_cd=%d "
                    "GZ_cdps=%d FIELD_milli=%d G_Hz10=%u M_Hz10=%u "
                    "M_ERR=%u REJ=%u OVR=%u LATE=%u; mission held\n",
                    s.valid
                        ? (s.mag_valid ? "FUSED" : "GYRO_ONLY")
                        : "STALE",
                    (INT)(s.heading_deg * 100.0f),
                    (INT)(s.relative_deg * 100.0f),
                    (INT)(s.gyro_relative_deg * 100.0f),
                    (INT)(s.magnetic_deg * 100.0f),
                    (INT)(s.gyro_z_dps * 100.0f),
                    (INT)(s.field_xy * 1000.0f),
                    s.gyro_hz_x10, s.mag_hz_x10,
                    s.mag_errors, s.rejected, s.overruns, s.late);
            }
        }
        tk_dly_tsk(250);
    }

    // CRITICAL FIX: TD_READ | TD_WRITE is required, TD_UPDATE blocks UART
    uart_device_id = tk_opn_dev((const UB*)"sera", TD_READ | TD_WRITE);
    if (uart_device_id <= E_OK) {
        robot_logf((UB*)"[init] cannot open UART0 device: %d\n", uart_device_id);
        return 0;
    }

    command_mbf_id = tk_cre_mbf(&cmbf);
    mtx_id = tk_cre_mtx(&cmtx);
    drive_mbf_id = tk_cre_mbf(&drive_cmbf);
    if (command_mbf_id <= E_OK || mtx_id <= E_OK || drive_mbf_id <= E_OK) {
        robot_logf((UB*)"[init] kernel object creation failed: uart_mbf=%d mtx=%d drive_mbf=%d\n",
                  command_mbf_id, mtx_id, drive_mbf_id);
        return 0;
    }

    // Start RX Task
    task_uart_rx_id = tk_cre_tsk(&ctsk_uart_rx);
    if (task_uart_rx_id <= E_OK || tk_sta_tsk(task_uart_rx_id, 0) < E_OK) {
        robot_logf((UB*)"[init] UART RX task failed\n");
        return 0;
    }

    // Start TX Task
    task_uart_tx_id = tk_cre_tsk(&ctsk_uart_tx);
    if (task_uart_tx_id <= E_OK || tk_sta_tsk(task_uart_tx_id, 0) < E_OK) {
        robot_logf((UB*)"[init] UART TX task failed\n");
        return 0;
    }

    task_drive_id = tk_cre_tsk(&ctsk_drive);
    if (task_drive_id <= E_OK || tk_sta_tsk(task_drive_id, 0) < E_OK) {
        robot_logf((UB*)"[init] drive task failed: %d\n", task_drive_id);
        return 0;
    }

    // Start State Machine Task
    task_state_id = tk_cre_tsk(&ctsk_state);
    if (task_state_id <= E_OK || tk_sta_tsk(task_state_id, 0) < E_OK) {
        robot_logf((UB*)"[init] state task creation/start failed: %d\n", task_state_id);
        return 0;
    }

    tk_slp_tsk(TMO_FEVR);
    return 0;
}
