
#ifndef AAI17_IMU_MTK3_H
#define AAI17_IMU_MTK3_H

#include <tk/tkernel.h>

/*
 * IMU sensor reading.
 * Timestamps use milliseconds from µT-Kernel.
 */

typedef struct {
    float pitch_deg;
    float roll_deg;

    float gyro_rate_dps;
    float accel_g;

    UW timestamp_ms;
    UW interval_ms;

    INT valid;
    INT stationary;

    // Raw MPU-6500 values for debugging
    H raw_ax;
    H raw_ay;
    H raw_az;

    H raw_gx;
    H raw_gy;
    H raw_gz;
} imu_sample_t;


/*
 * Initialise MPU-6500.
 *
 * shared_i2c_mutex must also be used by
 * Motion Control when accessing I2C1.
 *
 * Keep sensor stationary during calibration.
 */
ER imu_mtk3_init(ID shared_i2c_mutex);

/*
 * Read sensor and update pitch/roll.
 *
 * Returns 1 if valid, 0 if invalid.
 */
INT imu_mtk3_read(imu_sample_t *out);

/* Get µT-Kernel monotonic time in ms. */
UW imu_mtk3_now_ms(void);

#endif
