
#include "imu_verify.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

void imu_verify_cancel(imu_verifier_t *v) {
    if (v != NULL) {
        memset(v, 0, sizeof(*v));
    }
}

bool imu_verify_begin(
    imu_verifier_t *v,
    uint32_t request_id,
    float target_pitch,
    float target_roll,
    uint64_t now_us
) {
    if (v == NULL ||
        request_id == 0 ||
        !isfinite(target_pitch) ||
        !isfinite(target_roll) ||
        now_us == 0) {
        return false;
    }

    imu_verify_cancel(v);

    v->active = true;
    v->request_id = request_id;
    v->target_pitch = target_pitch;
    v->target_roll = target_roll;
    v->started_us = now_us;

    return true;
}

imu_verify_result_t imu_verify_step(
    imu_verifier_t *v,
    const imu_reading_t *s,
    uint64_t now_us
) {
    if (v == NULL || !v->active) {
        return IMU_VERIFY_NOT_ACTIVE;
    }

    if (now_us < v->started_us) {
        imu_verify_cancel(v);
        return IMU_VERIFY_SENSOR_FAULT;
    }

    // Check validity, timing and freshness.
    if (s == NULL ||
        !s->valid ||
        s->timestamp_us < v->started_us ||
        s->timestamp_us > now_us ||
        s->interval_us > IMU_MAX_SAMPLE_INTERVAL_US ||
        !isfinite(s->pitch_deg) ||
        !isfinite(s->roll_deg) ||
        now_us - s->timestamp_us >
            IMU_MAX_SAMPLE_INTERVAL_US) {

        imu_verify_cancel(v);
        return IMU_VERIFY_SENSOR_FAULT;
    }

    // Reject duplicate or late consecutive samples.
    if (v->previous_sample_us != 0) {
        if (s->timestamp_us <= v->previous_sample_us ||
            s->timestamp_us - v->previous_sample_us >
                IMU_MAX_SAMPLE_INTERVAL_US) {

            imu_verify_cancel(v);
            return IMU_VERIFY_SENSOR_FAULT;
        }
    }

    v->previous_sample_us = s->timestamp_us;

    float pitch_error = fabsf(
        s->pitch_deg - v->target_pitch
    );

    float roll_error = fabsf(
        remainderf(
            s->roll_deg - v->target_roll,
            360.0f
        )
    );

    bool within_tolerance =
        pitch_error <= IMU_FIXED_TOLERANCE_DEG &&
        roll_error <= IMU_FIXED_TOLERANCE_DEG;

    if (within_tolerance) {
        v->consecutive_count++;

        if (v->consecutive_count >=
            IMU_REQUIRED_CONSECUTIVE) {

            imu_verify_cancel(v);
            return IMU_VERIFY_PASS;
        }
    } else {
        // A bad orientation resets the count.
        v->consecutive_count = 0;
    }

    if (now_us - v->started_us >=
        IMU_VERIFY_TIMEOUT_US) {

        imu_verify_cancel(v);
        return IMU_VERIFY_OUT_OF_TOLERANCE;
    }

    return IMU_VERIFY_PENDING;
}
