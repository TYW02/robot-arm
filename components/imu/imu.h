
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

typedef enum {
    IMU_FAULT,
    IMU_OUT_OF_TOLERANCE,
    IMU_WITHIN_TOLERANCE
} imu_status_t;

typedef struct {
    float pitch_deg;
    float roll_deg;
    uint64_t timestamp_us;
    uint32_t interval_us;
    bool valid;
} imu_reading_t;

// Callbacks for shared I2C protection
typedef bool (*imu_bus_lock_fn)(void *context);
typedef void (*imu_bus_unlock_fn)(void *context);
typedef void (*imu_delay_fn)(uint32_t ms, void *context);

bool imu_set_bus_lock(
    imu_bus_lock_fn lock,
    imu_bus_unlock_fn unlock,
    void *context
);

void imu_set_delay(imu_delay_fn delay, void *context);

// Existing IMU functions
bool imu_init(i2c_inst_t *i2c);
bool imu_update(void);

float imu_get_pitch(void);
float imu_get_roll(void);

// New function: obtain timestamped sensor data
bool imu_get_reading(imu_reading_t *out);

// This checks ONE sample only.
// Use imu_verify.c for the required two-sample approval.
imu_status_t imu_check_orientation(
    float target_pitch,
    float target_roll,
    float tolerance
);

#endif
