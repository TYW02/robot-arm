
#ifndef AAI17_IMU_SERVICE_H
#define AAI17_IMU_SERVICE_H

#include <tk/tkernel.h>
#include "imu_mtk3.h"

typedef enum {
    IMU_EVT_READY = 1,
    IMU_EVT_SAMPLE,
    IMU_EVT_VERIFIED,
    IMU_EVT_OUT_OF_TOLERANCE,
    IMU_EVT_FAULT,
    IMU_EVT_CANCELLED
} imu_event_t;

typedef struct {
    UW request_id;
    UW event;

    imu_sample_t sample;

    /* 1 = RP2040 physical core 0 */
    INT producer_core;
} imu_result_t;

/*
 * Start the IMU Task.
 *
 * Call from an existing kernel task,
 * after I2C1 has been initialised.
 *
 * Motion must use the same I2C mutex.
 */
ER imu_service_start(
    ID shared_i2c_mutex
);

/*
 * Request orientation verification.
 *
 * The Coordinator provides:
 * - unique request ID
 * - target pitch
 * - target roll
 */
ER imu_service_verify(
    UW request_id,
    float pitch_deg,
    float roll_deg
);

/*
 * Request latest IMU reading.
 * Intended for diagnostics/testing.
 */
ER imu_service_snapshot(
    UW request_id
);

/* Cancel current verification. */
ER imu_service_cancel(void);

/*
 * Reinitialise and recalibrate.
 * Only use while the arm is stationary.
 */
ER imu_service_reinit(void);

/*
 * Receive results without blocking.
 *
 * Returns:
 *  1 = result received
 *  0 = no result
 * <0 = error
 *
 * Only one task should consume results.
 */
INT imu_service_poll(
    imu_result_t *out
);

#endif
