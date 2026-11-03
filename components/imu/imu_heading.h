#ifndef IMU_HEADING_H
#define IMU_HEADING_H

#include <tk/tkernel.h>

typedef struct {
    BOOL ready;
    BOOL valid;
    BOOL mag_valid;
    ER fault;

    float heading_deg;
    float relative_deg;
    float gyro_relative_deg;
    float magnetic_deg;
    float gyro_z_dps;
    float field_xy;

    UW gyro_hz_x10;
    UW mag_hz_x10;
    UW mag_errors;
    UW rejected;
    UW overruns;
    UW late;
    UW sample_us;
} imu_heading_reading_t;

ER imu_heading_start(void);
void imu_heading_read(imu_heading_reading_t *reading);

#endif