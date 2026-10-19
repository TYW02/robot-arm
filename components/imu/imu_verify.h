
#ifndef IMU_VERIFY_H
#define IMU_VERIFY_H

#include <stdbool.h>
#include <stdint.h>
#include "imu.h"

#define IMU_REQUIRED_CONSECUTIVE    2U
#define IMU_MAX_SAMPLE_INTERVAL_US  50000U
#define IMU_FIXED_TOLERANCE_DEG     5.0f

// Proposed verification timeout.
// Confirm this value with the Coordinator.
#define IMU_VERIFY_TIMEOUT_US       1000000ULL

typedef enum {
    IMU_VERIFY_PENDING,
    IMU_VERIFY_PASS,
    IMU_VERIFY_OUT_OF_TOLERANCE,
    IMU_VERIFY_SENSOR_FAULT,
    IMU_VERIFY_NOT_ACTIVE
} imu_verify_result_t;

typedef struct {
    uint32_t request_id;
    float target_pitch;
    float target_roll;
    uint64_t started_us;
    uint64_t previous_sample_us;
    uint8_t consecutive_count;
    bool active;
} imu_verifier_t;

void imu_verify_cancel(imu_verifier_t *v);

bool imu_verify_begin(
    imu_verifier_t *v,
    uint32_t request_id,
    float target_pitch,
    float target_roll,
    uint64_t now_us
);

imu_verify_result_t imu_verify_step(
    imu_verifier_t *v,
    const imu_reading_t *reading,
    uint64_t now_us
);

#endif
