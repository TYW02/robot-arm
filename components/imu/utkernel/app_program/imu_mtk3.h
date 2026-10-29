#ifndef AAI17_IMU_MTK3_H
#define AAI17_IMU_MTK3_H

#include <tk/tkernel.h>

/* Maximum acceptable interval between readings. */
#define IMU_MAX_GAP_MS 50U

/*
 * Sensor axis used for the car's rotation.
 * X = 0, Y = 1, Z = 2.
 *
 * Default assumes sensor Z points upward.
 * Positive yaw is counterclockwise viewed from above.
 */
#define IMU_YAW_AXIS 2
#define IMU_YAW_SIGN 1

typedef enum {
    IMU_STATUS_OK = 0,
    IMU_STATUS_NOT_READY,
    IMU_STATUS_IO,
    IMU_STATUS_NO_DATA,
    IMU_STATUS_TIMING,
    IMU_STATUS_SATURATED
} imu_status_t;

/*
 * Gyroscope and relative-yaw reading.
 * Timestamps and intervals are in milliseconds.
 */
typedef struct {
    /* Signed, bias-corrected angular velocities. */
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    /* Angular velocity around the car's vertical axis. */
    float yaw_rate_dps;

    /* Accumulated relative rotation; can exceed 360 degrees. */
    float yaw_deg;

    /* Same rotation expressed within [-180, 180) degrees. */
    float yaw_wrapped_deg;

    UW timestamp_ms;
    UW interval_ms;
    UW sequence;

    /* Changes when calibration or the yaw reference changes. */
    UW reference_id;

    INT calibrated;
    INT rate_valid;
    INT heading_valid;

    /*
     * Small measured angular velocity.
     * This alone does not prove the car is stationary.
     */
    INT low_rate;

    imu_status_t status;

    /* Raw gyroscope readings for debugging. */
    H raw_gx;
    H raw_gy;
    H raw_gz;
} imu_sample_t;

/*
 * Initialise MPU-6500 and calibrate gyro offsets.
 * Keep the car stopped throughout calibration.
 *
 * All users of I2C1 must use the same mutex.
 * Call from the IMU service task.
 */
ER imu_mtk3_init(ID shared_i2c_mutex);

/*
 * Read the gyroscope and update relative yaw.
 *
 * Returns 1 when the current angular-velocity reading is valid.
 * Returns 0 when the reading is invalid.
 *
 * Check heading_valid separately before using yaw.
 */
INT imu_mtk3_read(imu_sample_t *out);

/*
 * Establish a new relative-yaw reference while stopped.
 * Example: imu_mtk3_set_yaw(0.0f).
 *
 * This does not establish magnetic north.
 * Call from the IMU service task.
 */
ER imu_mtk3_set_yaw(float yaw_deg);

/* Get µT-Kernel operating time in milliseconds. */
UW imu_mtk3_now_ms(void);

/* Temporary startup diagnostics. */
UW imu_mtk3_init_stage(void);
UW imu_mtk3_calibration_count(void);

#endif