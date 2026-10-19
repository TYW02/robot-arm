#include "imu_service.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <stddef.h>

// Your tested standalone IMU wiring.
#define IMU_SDA 6
#define IMU_SCL 7

static ID i2c_mutex = 0;
static ID command_mbf = 0;
static ID result_mbf = 0;

static bool started = false;

// Return shared mutex to Motion Control.
ID imu_service_i2c_mutex(void) {
    return i2c_mutex;
}

// Initialise IMU subsystem.
bool imu_service_start(void) {

    if (started) {
        return false;
    }

    // 1. Create shared I2C mutex.
    T_CMTX mutex = {0};
    mutex.mtxatr = TA_INHERIT;

    i2c_mutex = tk_cre_mtx(&mutex);

    if (i2c_mutex <= 0) {
        return false;
    }

    // 2. Create Coordinator-to-IMU message buffer.
    T_CMBF in = {0};

    in.mbfatr = TA_TFIFO;
    in.maxmsz = sizeof(imu_command_t);
    in.bufsz = 256;

    command_mbf = tk_cre_mbf(&in);

    if (command_mbf <= 0) {
        return false;
    }

    // 3. Create IMU-to-Coordinator message buffer.
    T_CMBF out = {0};

    out.mbfatr = TA_TFIFO;
    out.maxmsz = sizeof(imu_result_t);
    out.bufsz = 512;

    result_mbf = tk_cre_mbf(&out);

    if (result_mbf <= 0) {
        return false;
    }

    // 4. Configure I2C1.
    // This must only be initialised once
    // across the integrated robot.
    i2c_init(i2c1, 100000);

    gpio_set_function(
        IMU_SDA,
        GPIO_FUNC_I2C
    );

    gpio_set_function(
        IMU_SCL,
        GPIO_FUNC_I2C
    );

    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);

    // 5. Configure and start IMU Task.
    imu_task_config_t cfg = {
        .shared_i2c_mutex = i2c_mutex,
        .command_mbf = command_mbf,
        .result_mbf = result_mbf,

        // Initial proposed values.
        .priority = 8,
        .stack_bytes = 4096
    };

    if (imu_utkernel_start(&cfg) <= 0) {
        return false;
    }

    started = true;
    return true;
}

// Send a new orientation verification request.
bool imu_service_request(
    uint32_t request_id,
    float target_pitch_deg,
    float target_roll_deg
) {
    if (!started || !request_id) {
        return false;
    }

    imu_command_t c = {
        .request_id = request_id,
        .command = IMU_CMD_VERIFY,
        .target_pitch_deg = target_pitch_deg,
        .target_roll_deg = target_roll_deg
    };

    return tk_snd_mbf(
        command_mbf,
        &c,
        sizeof(c),
        TMO_POL
    ) == E_OK;
}

// Cancel current orientation verification.
bool imu_service_cancel(void) {
    if (!started) {
        return false;
    }

    imu_command_t c = {
        .command = IMU_CMD_CANCEL
    };

    return tk_snd_mbf(
        command_mbf,
        &c,
        sizeof(c),
        TMO_POL
    ) == E_OK;
}

// Receive IMU result without blocking.
bool imu_service_poll(imu_result_t *result) {
    if (!started || !result) {
        return false;
    }

    INT n = tk_rcv_mbf(
        result_mbf,
        result,
        TMO_POL
    );

    return n == (INT)sizeof(*result);
}
