/** @file imu_heading.h
 *  @brief Relative yaw readings from the MPU gyroscope.
 */

#ifndef IMU_HEADING_H
#define IMU_HEADING_H

#include <tk/tkernel.h>

/**
 * One snapshot of the heading task. Existing mission code uses this layout
 * and the kernel types for status flags, timestamps, and log arguments.
 */
typedef struct
{
    BOOL  ready;          /**< Calibration and sampling have started. */
    BOOL  valid;          /**< The sample is still fresh when read. */
    ER    fault;          /**< An error code if sampling stopped. */
    float relative_deg;   /**< Accumulated yaw since calibration, in degrees. */
    float gyro_z_dps;     /**< Bias-corrected Z rate, in degrees per second. */
    UW    gyro_hz_x10;    /**< Gyro sampling rate in tenths of a hertz. */
    UW    late;           /**< Count of samples over the late interval. */
    UW    sample_us;      /**< Low 32 bits of the hardware timer in us. */
} imu_heading_reading_t;

/** Start the task that configures, calibrates, and samples the gyro. */
ER imu_heading_start (void);

/** Copy a snapshot and recheck its age. A NULL destination is ignored. */
void imu_heading_read (imu_heading_reading_t * p_reading);

#endif /* IMU_HEADING_H */

/* End of file. */

