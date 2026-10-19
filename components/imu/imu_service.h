#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "imu_utkernel.h"

// Start the IMU subsystem after kernel startup.
bool imu_service_start(void);

// Coordinator requests orientation verification.
bool imu_service_request(
    uint32_t request_id,
    float target_pitch_deg,
    float target_roll_deg
);

// Cancel pending verification.
bool imu_service_cancel(void);

// Check for new IMU results without blocking.
bool imu_service_poll(imu_result_t *result);

// Shared I2C mutex for Motion Control.
ID imu_service_i2c_mutex(void);

#endif
