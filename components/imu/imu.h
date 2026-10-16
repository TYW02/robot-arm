
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include "hardware/i2c.h"

// Orientation verification results
typedef enum {
    IMU_FAULT,
    IMU_OUT_OF_TOLERANCE,
    IMU_WITHIN_TOLERANCE
} imu_status_t;

// Initialise the MPU-6500 on an existing I2C bus
bool imu_init(i2c_inst_t *i2c);

// Read sensor and update orientation
bool imu_update(void);

// Get calculated orientation
float imu_get_pitch(void);
float imu_get_roll(void);

// Verify actual orientation against target
imu_status_t imu_check_orientation(
    float target_pitch,
    float target_roll,
    float tolerance
);

#endif
