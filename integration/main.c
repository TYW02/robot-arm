// integration/main.c
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

// ======================================================
// Kernel Objects
// ======================================================
LOCAL ID task_uart_id;
LOCAL ID task_state_id;
LOCAL ID uart_device_id;
LOCAL ID command_mbf_id; // State-machine commands are sent to the UART owner task.
LOCAL ID mtx_id;  // Mutex for shared variables

#define RX_LINE_SIZE 128
typedef struct { UB bytes[4]; } ModeCommand;
LOCAL UW command_mbf_buffer[(4 * sizeof(ModeCommand)) / sizeof(UW) + 16];

LOCAL void task_uart_parser(INT stacd, void *exinf);
LOCAL T_CTSK ctsk_uart = {
    .itskpri = 5, .stksz = 4096, .task = task_uart_parser,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL void task_state_machine(INT stacd, void *exinf);
LOCAL T_CTSK ctsk_state = {
    .itskpri = 10, .stksz = 4096, .task = task_state_machine,
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

// ======================================================
// Shared Variables
// ======================================================
#define MAX_OBJECTS 16
typedef struct { int id; int x; int y; float distance; } aruco_obj_t;
aruco_obj_t shelf_map[MAX_OBJECTS];
int shelf_count = 0;

void send_mode_command(const char* mode) {
    ModeCommand command = {{0, 0, 0, 0}};
    UW len = 0;
    while (mode[len] != '\0' && len < sizeof(command.bytes)) {
        command.bytes[len] = (UB)mode[len];
        ++len;
    }
    if (len == 0 || tk_snd_mbf(command_mbf_id, &command,
                               sizeof(command), TMO_FEVR) < E_OK) {
        tm_printf((UB*)"[UART] failed to queue mode command\n");
    }
}

// ======================================================
// Custom Parsing Helpers (Replaces string.h and stdio.h)
// ======================================================
int my_strlen(const char *s) { int len=0; while(*s++) len++; return len; }

char* my_strchr(const char *s, int c) {
    while (*s != (char)c) { if (!*s++) return 0; }
    return (char*)s;
}

char* my_strstr(const char *haystack, const char *needle) {
    if (!*needle) return (char*)haystack;
    char *p1 = (char*)haystack;
    while (*p1) {
        char *p1_begin = p1;
        char *p2 = (char*)needle;
        while (*p1 && *p2 && *p1 == *p2) { p1++; p2++; }
        if (!*p2) return p1_begin;
        p1 = p1_begin + 1;
    }
    return 0;
}

int parse_int(char **str) {
    int res = 0;
    while (**str >= '0' && **str <= '9') { res = res * 10 + (**str - '0'); (*str)++; }
    if (**str == ',') (*str)++;
    return res;
}

float parse_float(char **str) {
    float res = 0.0f; int sign = 1;
    if (**str == '-') { sign = -1; (*str)++; }
    while (**str >= '0' && **str <= '9') { res = res * 10.0f + (**str - '0'); (*str)++; }
    if (**str == '.') {
        (*str)++; float frac = 0.1f;
        while (**str >= '0' && **str <= '9') { res += (**str - '0') * frac; frac *= 0.1f; (*str)++; }
    }
    if (**str == ',') (*str)++;
    return res * sign;
}

// ======================================================
// Task 1: UART Parser
// ======================================================
LOCAL void task_uart_parser(INT stacd, void *exinf)
{
    char buffer[RX_LINE_SIZE];
    char pending[32];
    SZ pending_size = 0;
    SZ pending_index = 0;
    UW line_index = 0;
    BOOL line_overflow = FALSE;

    (void)stacd;
    (void)exinf;

    while(1) {
        ModeCommand command;
        INT command_size = tk_rcv_mbf(command_mbf_id, &command, TMO_POL);
        if (command_size == (INT)sizeof(command)) {
            SZ written = 0;
            ER write_result = tk_swri_dev(
                uart_device_id, 0, command.bytes, sizeof(command.bytes), &written);
            if (write_result < E_OK || written != sizeof(command.bytes)) {
                tm_printf((UB*)"[UART] command write failed: %d\n", write_result);
            }
        }

        if (pending_index >= pending_size) {
            SZ available = 0;
            ER read_result = tk_srea_dev(uart_device_id, 0, NULL, 0, &available);
            if (read_result < E_OK || available == 0) {
                tk_dly_tsk(1);
                continue;
            }

            pending_size = available < sizeof(pending) ? available : sizeof(pending);
            pending_index = 0;
            read_result = tk_srea_dev(
                uart_device_id, 0, pending, pending_size, &pending_size);
            if (read_result < E_OK || pending_size == 0) {
                pending_size = pending_index = 0;
                tk_dly_tsk(1);
                continue;
            }
        }

        INT msgsz = 0;
        while (pending_index < pending_size) {
            char ch = pending[pending_index++];
            if (ch == '\n') {
                if (!line_overflow) {
                    buffer[line_index] = '\0';
                    msgsz = (INT)(line_index + 1U);
                }
                line_index = 0;
                line_overflow = FALSE;
                if (msgsz > 0) break;
            } else if (line_index < RX_LINE_SIZE - 1U) {
                buffer[line_index++] = ch;
            } else {
                line_overflow = TRUE;
            }
        }
        if (msgsz <= 0) continue;

        if (msgsz > 0) {
            int idx = my_strlen(buffer);
            while (idx > 0 && buffer[idx-1] == '\r') { buffer[--idx] = '\0'; }

            // Ignore boot logs
            if (my_strstr(buffer, "A,") == NULL && my_strstr(buffer, "C,") == NULL) continue;

            char *star = my_strchr(buffer, '*');
            if (star != NULL) {
                *star = '\0';

                // Verify Checksum
                UB expected = 0;
                for (char *p = buffer; *p; p++) expected ^= (UB)*p;

                int received = 0;
                char *cksum_str = star + 1;
                while (*cksum_str) {
                    int val = 0;
                    if (*cksum_str >= '0' && *cksum_str <= '9') val = *cksum_str - '0';
                    else if (*cksum_str >= 'A' && *cksum_str <= 'F') val = *cksum_str - 'A' + 10;
                    else if (*cksum_str >= 'a' && *cksum_str <= 'f') val = *cksum_str - 'a' + 10;
                    else break;
                    received = (received << 4) | val;
                    cksum_str++;
                }

                if (expected == received) {
                    char *start;
                    if ((start = my_strstr(buffer, "A,")) != NULL) {
                        start += 2; // Skip "A,"
                        int id = parse_int(&start);
                        int x = parse_int(&start);
                        int y = parse_int(&start);
                        (void)parse_float(&start);
                        float dist = parse_float(&start);

                        INT total;
                        BOOL new_object = FALSE;
                        if (tk_loc_mtx(mtx_id, TMO_FEVR) < E_OK) continue;
                        int found = 0;
                        for (int i = 0; i < shelf_count; i++) {
                            if (shelf_map[i].id == id) {
                                shelf_map[i].x = x; shelf_map[i].y = y; shelf_map[i].distance = dist;
                                found = 1; break;
                            }
                        }
                        if (!found && shelf_count < MAX_OBJECTS) {
                            shelf_map[shelf_count].id = id;
                            shelf_map[shelf_count].x = x;
                            shelf_map[shelf_count].y = y;
                            shelf_map[shelf_count].distance = dist;
                            shelf_count++;
                            new_object = TRUE;
                        }
                        total = shelf_count;
                        (void)tk_unl_mtx(mtx_id);
                        if (new_object) {
                            tm_printf((UB*)"  -> NEW ARUCO FOUND! ID:%d (Total: %d)\n",
                                      id, total);
                        }
                    } else if ((start = my_strstr(buffer, "C,")) != NULL) {
                        start += 2; // Skip "C,"
                        char color = *start; start += 2; // Skip color and comma
                        int x = parse_int(&start);
                        int y = parse_int(&start);
                        tm_printf((UB*)"  -> Payload found: COLOR %c at X:%d Y:%d\n", color, x, y);
                    }
                } else {
                    tm_printf((UB*)"[Error] Checksum mismatch!\n");
                }
            }
        }
    }
}

// ======================================================
// Task 2: State Machine
// ======================================================
LOCAL void task_state_machine(INT stacd, void *exinf)
{
    while(1) {
        tm_printf((UB*)"\n[State] SCAN_SHELF: Sending M,S...\n");
        tk_loc_mtx(mtx_id, TMO_FEVR); shelf_count = 0; tk_unl_mtx(mtx_id);

        send_mode_command("M,S\n");
        tk_dly_tsk(50);
        send_mode_command("M,S\n");

        // Wait 5 seconds for scan
        tk_dly_tsk(5000);

        tk_loc_mtx(mtx_id, TMO_FEVR);
        INT scanned_object_count = shelf_count;
        tk_unl_mtx(mtx_id);
        tm_printf((UB*)"[State] Finished Scanning. Found %d objects.\n",
                  scanned_object_count);

        tm_printf((UB*)"[State] MOVE_TO_PAYLOAD...\n");
        tk_dly_tsk(2000); // Wait 2s

        tm_printf((UB*)"[State] RE_SCAN_PAYLOAD: Sending M,C...\n");
        send_mode_command("M,C\n");
        tk_dly_tsk(50);
        send_mode_command("M,C\n");

        tk_dly_tsk(2000); // Wait 2s

        tm_printf((UB*)"[State] EXECUTE_PICK...\n");
        tk_dly_tsk(3000); // Wait 3s before restart
    }
}

// ======================================================
// Entry Point (Called by μT-Kernel)
// ======================================================
EXPORT INT usermain(void)
{
    tm_printf((UB*)"Pico W vision app on micro T-Kernel SMP starting...\n");

    /* The BSP registers UART0 as "sera" and routes GP0/GP1 to it.  T-Monitor
     * is built USB-only for this app so it does not share the vision UART. */
    uart_device_id = tk_opn_dev((const UB*)"sera", TD_UPDATE);
    if (uart_device_id <= E_OK) {
        tm_printf((UB*)"[init] cannot open UART0 device: %d\n", uart_device_id);
        return 0;
    }

    command_mbf_id = tk_cre_mbf(&cmbf);
    mtx_id = tk_cre_mtx(&cmtx);
    if (command_mbf_id <= E_OK || mtx_id <= E_OK) {
        tm_printf((UB*)"[init] kernel object creation failed: mbf=%d mtx=%d\n",
                  command_mbf_id, mtx_id);
        return 0;
    }

    task_uart_id = tk_cre_tsk(&ctsk_uart);
    if (task_uart_id <= E_OK || tk_sta_tsk(task_uart_id, 0) < E_OK) {
        tm_printf((UB*)"[init] UART task creation/start failed: %d\n", task_uart_id);
        return 0;
    }
    task_state_id = tk_cre_tsk(&ctsk_state);
    if (task_state_id <= E_OK || tk_sta_tsk(task_state_id, 0) < E_OK) {
        tm_printf((UB*)"[init] state task creation/start failed: %d\n", task_state_id);
        return 0;
    }

    /* Match the μT-Kernel BSP demo: keep the initial task asleep while the
     * two application tasks run. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
