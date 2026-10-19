
#include "imu_utkernel.h"
#include "imu.h"
#include "imu_verify.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <stdbool.h>
#include <stdint.h>

#define IMU_PERIOD_US 20000ULL
#define IMU_LOCK_TIMEOUT_MS 10

static imu_task_config_t cfg;
static ID imu_task_id = 0;

// Use kernel delays during calibration.
static void kernel_delay(
    uint32_t ms,
    void *ctx
) {
    (void)ctx;
    (void)tk_dly_tsk((RELTIM)ms);
}

// Protect the shared I2C bus.
static bool lock_i2c(void *ctx) {
    ID id = *(const ID *)ctx;

    return tk_loc_mtx(
        id,
        IMU_LOCK_TIMEOUT_MS
    ) == E_OK;
}

static void unlock_i2c(void *ctx) {
    ID id = *(const ID *)ctx;
    (void)tk_unl_mtx(id);
}

// Send a result without blocking sensor sampling.
static bool send_result(
    imu_event_type_t type,
    uint32_t request,
    const imu_reading_t *s
) {
    imu_result_t r = {0};

    r.event = type;
    r.request_id = request;

    if (s) {
        r.pitch_deg = s->pitch_deg;
        r.roll_deg = s->roll_deg;
        r.sample_time_us = s->timestamp_us;
        r.interval_us = s->interval_us;
    }

    return tk_snd_mbf(
        cfg.result_mbf,
        &r,
        sizeof(r),
        TMO_POL
    ) == E_OK;
}

// Main IMU Task
static void task_entry(
    INT stacd,
    void *exinf
) {
    (void)stacd;
    (void)exinf;

    imu_verifier_t verifier = {0};
    imu_reading_t reading = {0};

    uint32_t active_id = 0;
    bool online;

    imu_set_bus_lock(
        lock_i2c,
        unlock_i2c,
        &cfg.shared_i2c_mutex
    );

    imu_set_delay(kernel_delay, NULL);

    // The robot must be stationary during calibration.
    // Shared I2C was already set up during startup.
    online = imu_init(i2c1);

    if (!send_result(
            online ? IMU_EVT_READY : IMU_EVT_FAULT,
            0,
            NULL
        )) {
        online = false;
    }

    uint64_t next_us = time_us_64() + IMU_PERIOD_US;

    while (true) {

        // 1. Receive commands from Coordinator.
        imu_command_t command;

        for (unsigned k = 0; k < 4; ++k) {
            INT n = tk_rcv_mbf(
                cfg.command_mbf,
                &command,
                TMO_POL
            );

            if (n < 0) {
                break;
            }

            if (n != (INT)sizeof(command)) {
                imu_verify_cancel(&verifier);
                active_id = 0;
                continue;
            }

            if (command.command == IMU_CMD_CANCEL) {
                imu_verify_cancel(&verifier);
                active_id = 0;
            }

            else if (command.command == IMU_CMD_REINIT) {
                imu_verify_cancel(&verifier);
                active_id = 0;

                // Recalibration requires a stationary arm.
                online = imu_init(i2c1);

                if (!send_result(
                        online
                            ? IMU_EVT_READY
                            : IMU_EVT_FAULT,
                        0,
                        NULL
                    )) {
                    online = false;
                }

                next_us = time_us_64() + IMU_PERIOD_US;
            }

            else if (command.command == IMU_CMD_VERIFY) {
                imu_verify_cancel(&verifier);
                active_id = 0;

                // Coordinator supplies the current
                // placement target and unique request ID.
                if (online && imu_verify_begin(
                        &verifier,
                        command.request_id,
                        command.target_pitch_deg,
                        command.target_roll_deg,
                        time_us_64()
                    )) {

                    active_id = command.request_id;

                } else {
                    if (!send_result(
                            IMU_EVT_FAULT,
                            command.request_id,
                            NULL
                        )) {
                        online = false;
                    }
                }
            }
        }

        // 2. Sample the MPU-6500.
        if (online) {
            bool ok = imu_update();

            (void)imu_get_reading(&reading);

            // 3. Verify orientation if requested.
            if (active_id != 0) {
                imu_verify_result_t result =
                    imu_verify_step(
                        &verifier,
                        &reading,
                        time_us_64()
                    );

                imu_event_type_t event = 0;

                if (!ok ||
                    result == IMU_VERIFY_SENSOR_FAULT) {

                    event = IMU_EVT_FAULT;

                } else if (
                    result == IMU_VERIFY_PASS
                ) {
                    event = IMU_EVT_VERIFIED;

                } else if (
                    result == IMU_VERIFY_OUT_OF_TOLERANCE
                ) {
                    event = IMU_EVT_OUT_OF_TOLERANCE;
                }

                if (event != 0) {
                    // If the result buffer is unavailable,
                    // disable verification until reinit.
                    if (!send_result(
                            event,
                            active_id,
                            &reading
                        )) {
                        online = false;
                    }

                    active_id = 0;
                    imu_verify_cancel(&verifier);
                }
            }
        }

        // 4. Wait for next scheduled sample.
        uint64_t now = time_us_64();

        if (now >= next_us) {
            next_us = now + IMU_PERIOD_US;
        } else {
            uint64_t remaining = next_us - now;

            RELTIM wait_ms = (RELTIM)(
                (remaining + 999ULL) / 1000ULL
            );

            (void)tk_dly_tsk(
                wait_ms ? wait_ms : 1
            );

            next_us += IMU_PERIOD_US;
            continue;
        }

        (void)tk_dly_tsk(1);
    }
}

// Create and start the µT-Kernel IMU Task.
ID imu_utkernel_start(
    const imu_task_config_t *configuration
) {
    if (!configuration ||
        configuration->shared_i2c_mutex <= 0 ||
        configuration->command_mbf <= 0 ||
        configuration->result_mbf <= 0 ||
        configuration->priority <= 0 ||
        configuration->stack_bytes < 1024 ||
        imu_task_id > 0) {

        return E_PAR;
    }

    cfg = *configuration;

    T_CTSK t = {0};

    t.tskatr = TA_HLNG | TA_RNG3;
    t.task = task_entry;
    t.itskpri = cfg.priority;
    t.stksz = cfg.stack_bytes;

    ID id = tk_cre_tsk(&t);

    if (id <= 0) {
        return id;
    }

    ER err = tk_sta_tsk(id, 0);

    if (err != E_OK) {
        (void)tk_del_tsk(id);
        return err;
    }

    imu_task_id = id;

    return id;
}
