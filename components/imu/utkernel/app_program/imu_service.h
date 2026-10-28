#ifndef AAI17_IMU_SERVICE_H
#define AAI17_IMU_SERVICE_H

#include <tk/tkernel.h>
#include "imu_mtk3.h"

/* Events sent from the IMU service to the coordinator. */
typedef enum {
    IMU_EVT_READY = 1,
    IMU_EVT_REFERENCE_SET,
    IMU_EVT_FAULT
} imu_event_t;

typedef struct {
    UW event;

    /*
     * Matches the command that produced this result.
     * Zero is reserved for startup and unsolicited faults.
     */
    UW request_id;

    /* E_OK on success; a negative error code on failure. */
    ER error;

    /*
     * Number of earlier events that could not be queued.
     * The coordinator must also time out missing replies.
     */
    UW dropped_events;

    /* Processor 1 = RP2040 physical core 0. */
    INT producer_core;
} imu_result_t;

/*
 * Start the IMU service task.
 *
 * Call from an existing kernel task after I2C1 setup.
 * All I2C1 users must share the supplied mutex.
 *
 * E_OK means the task started, not that calibration finished.
 * Calibration completion is reported through IMU_EVT_READY.
 *
 * Keep the car stopped throughout startup calibration.
 */
ER imu_service_start(ID shared_i2c_mutex);

/*
 * Copy the latest gyro/yaw sample without queuing a request.
 *
 * Returns E_OK only when the angular-velocity sample is fresh
 * and valid. Check out->heading_valid separately before using
 * the yaw angle.
 *
 * On failure, the returned sample is cleared or marked invalid.
 * Multiple tasks may call this function.
 */
ER imu_service_latest(imu_sample_t *out);

/*
 * Request a new relative-yaw reference.
 * Example: imu_service_set_yaw(request_id, 0.0f).
 *
 * The car must be stopped.
 * This does not establish magnetic north.
 *
 * request_id must be nonzero.
 * E_OK means the command was queued.
 *
 * Wait for IMU_EVT_REFERENCE_SET with the matching request_id,
 * then obtain a fresh sample with the new reference_id.
 */
ER imu_service_set_yaw(
    UW request_id,
    float yaw_deg
);

/*
 * Request sensor reinitialisation and gyro recalibration.
 * Successful calibration sets relative yaw back to zero.
 *
 * Keep the car stopped throughout the operation.
 * request_id must be nonzero.
 *
 * E_OK means the command was queued.
 * Wait for IMU_EVT_READY or IMU_EVT_FAULT with the matching ID.
 */
ER imu_service_reinit(UW request_id);

/*
 * Receive a service event without blocking.
 *
 * Returns:
 *   1 = event received
 *   0 = no event available
 *  <0 = error
 *
 * Only one coordinator task should consume these events.
 * Obtain sensor readings through imu_service_latest().
 */
INT imu_service_poll(imu_result_t *out);

#endif