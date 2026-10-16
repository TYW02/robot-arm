
#include "imu.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdint.h>

// MPU-6500 registers
#define MPU_ADDR        0x68
#define WHO_AM_I        0x75
#define PWR_MGMT_1      0x6B
#define ACCEL_CONFIG    0x1C
#define GYRO_CONFIG     0x1B
#define ACCEL_XOUT_H    0x3B

// Sensor settings
#define ACCEL_SCALE     16384.0f
#define GYRO_SCALE      131.0f
#define FILTER_ALPHA    0.98f

// Maximum acceptable time between samples
#define MAX_SAMPLE_US   50000

static i2c_inst_t *imu_bus = NULL;

static float pitch = 0.0f;
static float roll = 0.0f;

static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;

static bool imu_valid = false;
static uint64_t last_sample_us = 0;

// Write to MPU-6500 register
static bool write_reg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};

    return i2c_write_timeout_us(
        imu_bus, MPU_ADDR, data, 2, false, 5000
    ) == 2;
}

// Read from MPU-6500 registers
static bool read_regs(uint8_t reg, uint8_t *buffer, size_t length) {
    if (i2c_write_timeout_us(
        imu_bus, MPU_ADDR, &reg, 1, true, 5000
    ) != 1) {
        return false;
    }

    return i2c_read_timeout_us(
        imu_bus, MPU_ADDR, buffer, length, false, 5000
    ) == (int)length;
}

static int16_t to_int16(uint8_t high, uint8_t low) {
    return (int16_t)(((uint16_t)high << 8) | low);
}

// Get 14-byte accelerometer, temperature and gyro block
static bool read_sensor(int16_t *ax, int16_t *ay, int16_t *az,
                        int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t data[14];

    if (!read_regs(ACCEL_XOUT_H, data, 14)) {
        return false;
    }

    *ax = to_int16(data[0], data[1]);
    *ay = to_int16(data[2], data[3]);
    *az = to_int16(data[4], data[5]);

    *gx = to_int16(data[8], data[9]);
    *gy = to_int16(data[10], data[11]);
    *gz = to_int16(data[12], data[13]);

    return true;
}

// Initialise and calibrate the MPU-6500
bool imu_init(i2c_inst_t *i2c) {
    imu_bus = i2c;
    imu_valid = false;

    if (imu_bus == NULL) {
        return false;
    }

    uint8_t reg = WHO_AM_I;
    uint8_t id = 0;

    if (!read_regs(reg, &id, 1) || id != 0x70) {
        return false;
    }

    // Wake up sensor
    if (!write_reg(PWR_MGMT_1, 0x01)) {
        return false;
    }

    sleep_ms(100);

    // Accelerometer +/-2g and gyroscope +/-250 dps
    if (!write_reg(ACCEL_CONFIG, 0x00) ||
        !write_reg(GYRO_CONFIG, 0x00)) {
        return false;
    }

    // Gyroscope offset calibration
    // Keep sensor stationary during startup
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    for (int i = 0; i < 100; i++) {
        int16_t ax, ay, az, gx, gy, gz;

        if (!read_sensor(&ax, &ay, &az,
                         &gx, &gy, &gz)) {
            return false;
        }

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;

        sleep_ms(5);
    }

    gyro_offset_x = (sum_x / 100.0f) / GYRO_SCALE;
    gyro_offset_y = (sum_y / 100.0f) / GYRO_SCALE;
    gyro_offset_z = (sum_z / 100.0f) / GYRO_SCALE;

    // Establish initial orientation
    int16_t ax, ay, az, gx, gy, gz;

    if (!read_sensor(&ax, &ay, &az,
                     &gx, &gy, &gz)) {
        return false;
    }

    roll = atan2f((float)ay, (float)az)
           * 180.0f / 3.14159265f;

    pitch = atan2f(
        -(float)ax,
        sqrtf((float)ay * ay + (float)az * az)
    ) * 180.0f / 3.14159265f;

    last_sample_us = time_us_64();
    imu_valid = true;

    return true;
}

// Update orientation using accelerometer + gyroscope
bool imu_update(void) {
    if (imu_bus == NULL) {
        return false;
    }

    int16_t ax, ay, az, gx, gy, gz;

    if (!read_sensor(&ax, &ay, &az,
                     &gx, &gy, &gz)) {
        imu_valid = false;
        return false;
    }

    uint64_t now = time_us_64();
    uint64_t elapsed = now - last_sample_us;

    float accel_roll = atan2f((float)ay, (float)az)
                       * 180.0f / 3.14159265f;

    float accel_pitch = atan2f(
        -(float)ax,
        sqrtf((float)ay * ay + (float)az * az)
    ) * 180.0f / 3.14159265f;

    float gyro_x = gx / GYRO_SCALE - gyro_offset_x;
    float gyro_y = gy / GYRO_SCALE - gyro_offset_y;

    if (!imu_valid || elapsed > MAX_SAMPLE_US) {
        // Re-establish estimate after lost/late sample
        roll = accel_roll;
        pitch = accel_pitch;
    } else {
        float dt = elapsed / 1000000.0f;

        float predicted_roll = roll + gyro_x * dt;
        float predicted_pitch = pitch + gyro_y * dt;

        // Handle roll wrapping around +/-180 degrees
        float roll_error = accel_roll - predicted_roll;

        while (roll_error > 180.0f) roll_error -= 360.0f;
        while (roll_error < -180.0f) roll_error += 360.0f;

        roll = predicted_roll +
               (1.0f - FILTER_ALPHA) * roll_error;

        pitch = FILTER_ALPHA * predicted_pitch +
                (1.0f - FILTER_ALPHA) * accel_pitch;

        if (roll > 180.0f) roll -= 360.0f;
        if (roll < -180.0f) roll += 360.0f;
    }

    last_sample_us = now;
    imu_valid = true;

    // Report late sampling as a failure for this cycle
    return elapsed <= MAX_SAMPLE_US;
}

float imu_get_pitch(void) {
    return pitch;
}

float imu_get_roll(void) {
    return roll;
}

// Determine whether end-effector is correctly oriented
imu_status_t imu_check_orientation(
    float target_pitch,
    float target_roll,
    float tolerance
) {
    if (!imu_valid ||
        time_us_64() - last_sample_us > MAX_SAMPLE_US ||
        !isfinite(target_pitch) ||
        !isfinite(target_roll) ||
        !isfinite(tolerance) ||
        tolerance < 0.0f) {
        return IMU_FAULT;
    }

    float pitch_error = fabsf(pitch - target_pitch);
    float roll_difference = roll - target_roll;

    while (roll_difference > 180.0f)
        roll_difference -= 360.0f;

    while (roll_difference < -180.0f)
        roll_difference += 360.0f;

    float roll_error = fabsf(roll_difference);

    if (pitch_error <= tolerance &&
        roll_error <= tolerance) {
        return IMU_WITHIN_TOLERANCE;
    }

    return IMU_OUT_OF_TOLERANCE;
}
