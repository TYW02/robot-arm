/* Manual GP13 claw calibration, called only by the µT-Kernel USB service task. */
#include "claw.h"
#include <tm/tmonitor.h>

IMPORT int tm_tusb_read_char(void);
IMPORT void tm_tusb_discard_input(void);

/* Bench-test command range, independent of the mission setpoints.
 * These bounds do not imply the assembled jaws can travel to both endpoints. */
#define CLAW_TEST_MIN_DEG 0
#define CLAW_TEST_MAX_DEG 180
#define CLAW_TEST_HOLD_MS 2000u
#define RX_BUDGET 64

_Static_assert(CLAW_TEST_MIN_DEG >= 0 && CLAW_TEST_MAX_DEG <= 180 &&
               CLAW_TEST_MIN_DEG <= CLAW_TEST_MAX_DEG, "Invalid claw test limits");

static BOOL session_open, ready, discard_line;
static unsigned length;
static char line[24];
static UW last_command_ms;

/* Returning from usermain shuts the kernel down. Keep the initial task asleep
 * while the existing USB service task runs the claw test. */
EXPORT INT usermain(void)
{
    for (;;)
        (void)tk_slp_tsk(TMO_FEVR);
}

static void run_command(UW now)
{
    if (discard_line) {
        tm_printf((UB *)"ERR invalid or too-long line\n");
        return;
    }
    if (length == 3 && line[0] == 'o' && line[1] == 'f' && line[2] == 'f') {
        claw_release();
        tm_printf((UB *)"OK PWM off\n");
        return;
    }
    if (length == 6 && line[0] == 's' && line[1] == 't' && line[2] == 'a' &&
        line[3] == 't' && line[4] == 'u' && line[5] == 's') {
        INT angle = claw_get_angle();
        if (angle < 0)
            tm_printf((UB *)"PWM off\n");
        else
            tm_printf((UB *)"Commanded %d deg (not measured)\n", angle);
        return;
    }
    if (length >= 6 && line[0] == 'c' && line[1] == 'l' && line[2] == 'a' &&
        line[3] == 'w' && line[4] == ' ') {
        INT angle = 0;
        unsigned i = 5;
        for (; i < length && line[i] >= '0' && line[i] <= '9'; ++i) {
            angle = angle * 10 + (line[i] - '0');
            if (angle > 180) break; /* Bound accumulation even on malformed input. */
        }
        if (i == length && angle >= CLAW_TEST_MIN_DEG && angle <= CLAW_TEST_MAX_DEG) {
            claw_angle(angle);
            last_command_ms = now;
            tm_printf((UB *)"OK commanded %d deg; PWM off after 2 s\n", angle);
            return;
        }
    }
    tm_printf((UB *)"ERR use claw %d..%d, status, or off\n",
              CLAW_TEST_MIN_DEG, CLAW_TEST_MAX_DEG);
}

/* Called in task context, never from the TinyUSB ISR. No cross-core RX state. */
void claw_serial_test_poll(BOOL connected)
{
    if (!connected) {
        if (ready && claw_get_angle() >= 0) claw_release();
        session_open = FALSE;
        length = 0;
        discard_line = FALSE;
        tm_tusb_discard_input();
        return;
    }
    if (!session_open) {
        session_open = TRUE;
        tm_tusb_discard_input();
        if (!ready) ready = claw_init() == E_OK;
        if (!ready) {
            tm_printf((UB *)"ERR claw initialization failed; no commands enabled\n");
            return;
        }
        tm_printf((UB *)"GP13 CLAW TEST; wheels and mission disabled; PWM off\n");
        tm_printf((UB *)"Commands: claw %d..%d, status, off\n",
                  CLAW_TEST_MIN_DEG, CLAW_TEST_MAX_DEG);
    }
    if (!ready) {
        tm_tusb_discard_input();
        return;
    }

    SYSTIM time;
    if (tk_get_otm(&time) != E_OK) {
        if (claw_get_angle() >= 0) claw_release();
        length = 0;
        discard_line = TRUE;
        tm_tusb_discard_input();
        return;
    }
    UW now = time.lo;
    if (claw_get_angle() >= 0 && (UW)(now - last_command_ms) >= CLAW_TEST_HOLD_MS) {
        claw_release();
        tm_printf((UB *)"PWM off (timeout)\n");
    }
    for (unsigned i = 0; i < RX_BUDGET; ++i) {
        int ch = tm_tusb_read_char();
        if (ch < 0) break;
        if (ch == '\r' || ch == '\n') {
            if (length || discard_line) run_command(now);
            length = 0;
            discard_line = FALSE;
        } else if (!discard_line) {
            if (ch < ' ' || ch > '~' || length >= sizeof(line))
                discard_line = TRUE;
            else
                line[length++] = (char)ch;
        }
    }
}
