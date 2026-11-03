#ifndef IMU_HEADING_H
#define IMU_HEADING_H

#include <tk/tkernel.h>

typedef struct {
    BOOL ready;
    BOOL valid;
    ER fault;

    float relative_deg;
    float gyro_z_dps;

    UW gyro_hz_x10;
    UW late;
    UW sample_us;
} imu_heading_reading_t;

ER imu_heading_start(void);
void imu_heading_read(imu_heading_reading_t *reading);

#endif