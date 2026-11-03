#include "drive_turn.h"
#include "motor.h"
#include "../imu/imu_heading.h"
#include <tm/tmonitor.h>

static float turn_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static BOOL turn_now(UW *now)
{
    SYSTIM time;
    if (tk_get_otm(&time) != E_OK)
        return FALSE;

    *now = time.lo;
    return TRUE;
}

ER drive_turn_run(
    float degrees,
    BOOL (*allowed)(void),
    float *actual)
{
    imu_heading_reading_t s = {0};
    UW started, now, progress_at, settled_at = 0;
    BOOL settling = FALSE;
    ER result = E_OK;

    motor_brake();

    if (actual != NULL)
        *actual = 0.0f;

    if (allowed == NULL ||
        !(degrees >= -180.0f && degrees <= 180.0f))
        return E_PAR;

    if (!allowed())
        return E_IO;

    imu_heading_read(&s);

    if (!s.ready || !s.valid || s.fault != E_OK)
        return E_IO;

    if (!turn_now(&started))
        return E_SYS;

    const float origin = s.relative_deg;
    const float target =
        origin + degrees * TURN_GYRO_RIGHT_SIGN;

    float last_progress = origin;
    progress_at = started;

    for (;;) {
        if (!allowed()) {
            result = E_IO;
            break;
        }

        imu_heading_read(&s);

        if (!s.ready || !s.valid || s.fault != E_OK) {
            tm_printf((UB*)"[turn] gyro unavailable\n");
            result = E_IO;
            break;
        }

        if (!turn_now(&now)) {
            result = E_SYS;
            break;
        }

        float moved =
            (s.relative_deg - origin) * TURN_GYRO_RIGHT_SIGN;
        float error =
            (target - s.relative_deg) * TURN_GYRO_RIGHT_SIGN;
        float rate =
            s.gyro_z_dps * TURN_GYRO_RIGHT_SIGN;

        if (actual != NULL)
            *actual = moved;

        if ((degrees > 0.0f && moved < -5.0f) ||
            (degrees < 0.0f && moved > 5.0f)) {
            tm_printf((UB*)
                "[turn] wrong direction; verify gyro sign and motor mapping\n");
            result = E_IO;
            break;
        }

        if ((UW)(now - started) >= TURN_TIMEOUT_MS) {
            tm_printf((UB*)"[turn] timeout\n");
            result = E_TMOUT;
            break;
        }

        if (turn_abs(s.relative_deg - last_progress) >= 0.5f) {
            last_progress = s.relative_deg;
            progress_at = now;
        }

        if (turn_abs(error) <= TURN_TOLERANCE_DEG) {
            motor_brake();

            if (turn_abs(rate) <= TURN_SETTLE_DPS) {
                if (!settling) {
                    settling = TRUE;
                    settled_at = now;
                }

                if ((UW)(now - settled_at) >= TURN_SETTLE_MS)
                    break;
            } else {
                settling = FALSE;
            }
        } else {
            settling = FALSE;

            if ((UW)(now - progress_at) >= TURN_STALL_MS) {
                tm_printf((UB*)"[turn] no angular progress\n");
                result = E_TMOUT;
                break;
            }

            float output = TURN_KP * error - TURN_KD * rate;

            /* Brake when damping opposes the target direction. */
            if (output * error <= 0.0f) {
                motor_brake();
            } else {
                INT speed = (INT)turn_abs(output);

                if (speed < TURN_MIN_SPEED)
                    speed = TURN_MIN_SPEED;
                if (speed > TURN_MAX_SPEED)
                    speed = TURN_MAX_SPEED;
                if (output < 0.0f)
                    speed = -speed;

                motor_drive(speed, -speed);
            }
        }

        (void)tk_dly_tsk(1);
    }

    motor_brake();
    return result;
}