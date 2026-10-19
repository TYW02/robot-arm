
#include "imu_utkernel.h"
#include "imu.h"
#include "imu_verify.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <stdbool.h>

#define IMU_TASK_PERIOD_US  20000ULL
#define IMU_BUS_LOCK_MS     10

static imu_task_config_t cfg;
static ID task_id = 0;

// Allow calibration to yield to other tasks.
static void task_delay_ms(
    uint32_t ms,
    void *context
) {
    (void)context;
    (void)tk_dly_tsk(ms);
}

// Shared I2C mutex callbacks
static bool bus_lock(void *context) {
    ID mutex = *(const ID *)context;

    return tk_loc_mtx(
        mutex, IMU_BUS_LOCK_MS
    ) == E_OK;
}

static void bus_unlock(void *context) {
    ID mutex = *(const ID *)context;

    (void)tk_unl_mtx(mutex);
}

// Send IMU result to Coordinator
static bool send_result(
    imu_event_type_t event,
    uint32_t request_id,
    const imu_reading_t *reading
) {
    imu_result_t msg = {0};

    msg.request_id = request_id;
    msg.event = event;

    if (reading != NULL) {
        msg.pitch_deg = reading->pitch_deg;
        msg.roll_deg = reading->roll_deg;
        msg.sample_time_us = reading->timestamp_us;
        msg.interval_us = reading->interval_us;
    }

    // Nonblocking: do not delay sensor sampling.
    return tk_snd_mbf(
        cfg.result_mbf,
        &msg,
        sizeof(msg),
        TMO_POL
    ) == E_OK;
}

// -------------------------------------------------
// µT-Kernel IMU Task
// -------------------------------------------------

static void imu_task(INT stacd, void *exinf) {
    (void)stacd;
    (void)exinf;

    imu_verifier_t verifier = {0};
    imu_reading_t reading = {0};

    uint32_t active_request = 0;

    imu_set_bus_lock(
        bus_lock,
        bus_unlock,
        &cfg.shared_i2c_mutex
    );

    imu_set_delay(task_delay_ms, NULL);

    // I2C pins must already be configured.
    // Keep arm stationary during calibration.
    bool online = imu_init(i2c1);

    send_result(
        online ? IMU_EVT_READY : IMU_EVT_FAULT,
        0,
        NULL
    );

    uint64_t next_sample_us =
        time_us_64() + IMU_TASK_PERIOD_US;

    while (true) {

        // -----------------------------------------
        // 1. Check commands from Coordinator
        // -----------------------------------------

        imu_command_t cmd;

        // Process a bounded number per cycle.
        for (int i = 0; i < 8; i++) {
            INT received = tk_rcv_mbf(
                cfg.command_mbf,
                &cmd,
                TMO_POL
            );

            if (received < 0) {
                break;
            }

            if (received != (INT)sizeof(cmd)) {
                if (active_request != 0) {
                    send_result(
                        IMU_EVT_FAULT,
                        active_request,
                        NULL
                    );
                }

                imu_verify_cancel(&verifier);
                active_request = 0;
                continue;
            }

            if (cmd.command == IMU_CMD_CANCEL) {
                imu_verify_cancel(&verifier);
                active_request = 0;
            }

            else if (cmd.command == IMU_CMD_REINIT) {
                imu_verify_cancel(&verifier);
                active_request = 0;

                // Only reinitialise when stationary.
                online = imu_init(i2c1);

                send_result(
                    online
                        ? IMU_EVT_READY
                        : IMU_EVT_FAULT,
                    0,
                    NULL
                );

                next_sample_us =
                    time_us_64() + IMU_TASK_PERIOD_US;
            }

            else if (cmd.command == IMU_CMD_VERIFY) {
                imu_verify_cancel(&verifier);
                active_request = 0;

                // The request must correspond to the
                // current stationary placement pose.
                if (!online ||
                    !imu_verify_begin(
                        &verifier,
                        cmd.request_id,
                        cmd.target_pitch_deg,
                        cmd.target_roll_deg,
                        time_us_64()
                    )) {

                    send_result(
                        IMU_EVT_FAULT,
                        cmd.request_id,
                        NULL
                    );
                } else {
                    active_request = cmd.request_id;
                }
            }
        }

        // -----------------------------------------
        // 2. Take an IMU sample
        // -----------------------------------------

        if (online) {
            bool updated = imu_update();

            imu_get_reading(&reading);

            // -------------------------------------
            // 3. Check requested orientation
            // -------------------------------------

            if (active_request != 0) {
                imu_verify_result_t result =
                    imu_verify_step(
                        &verifier,
                        &reading,
                        time_us_64()
                    );

                if (!updated ||
                    result == IMU_VERIFY_SENSOR_FAULT) {

                    send_result(
                        IMU_EVT_FAULT,
                        active_request,
                        &reading
                    );

                    imu_verify_cancel(&verifier);
                    active_request = 0;
                }

                else if (result == IMU_VERIFY_PASS) {
                    send_result(
                        IMU_EVT_VERIFIED,
                        active_request,
                        &reading
                    );

                    active_request = 0;
                }

                else if (
                    result == IMU_VERIFY_OUT_OF_TOLERANCE
                ) {
                    send_result(
                        IMU_EVT_OUT_OF_TOLERANCE,
                        active_request,
                        &reading
                    );

                    active_request = 0;
                }
            }
        }

        // -----------------------------------------
        // 4. Sleep until next sampling period
        // -----------------------------------------

        uint64_t now = time_us_64();

        if (now >= next_sample_us) {
            // Missed scheduled target; reset cadence.
            next_sample_us =
                now + IMU_TASK_PERIOD_US;

            tk_dly_tsk(1);
        } else {
            uint64_t remaining_us =
                next_sample_us - now;

            UINT delay_ms = (UINT)(
                (remaining_us + 999ULL) / 1000ULL
            );

            tk_dly_tsk(
                delay_ms > 0 ? delay_ms : 1
            );

            next_sample_us += IMU_TASK_PERIOD_US;
        }
    }
}

// -------------------------------------------------
// Create and start IMU Task
// -------------------------------------------------

ID imu_utkernel_start(
    const imu_task_config_t *config
) {
    if (config == NULL ||
        config->shared_i2c_mutex <= 0 ||
        config->command_mbf <= 0 ||
        config->result_mbf <= 0 ||
        config->priority <= 0 ||
        config->stack_bytes < 1024 ||
        task_id > 0) {

        return E_PAR;
    }

    cfg = *config;

    T_CTSK task = {0};

    task.tskatr = TA_HLNG | TA_RNG3;
    task.task = imu_task;
    task.itskpri = cfg.priority;
    task.stksz = cfg.stack_bytes;

    ID id = tk_cre_tsk(&task);

    if (id <= 0) {
        return id;
    }

    ER result = tk_sta_tsk(id, 0);

    if (result != E_OK) {
        tk_del_tsk(id);
        return result;
    }

    task_id = id;

    return id;
}
