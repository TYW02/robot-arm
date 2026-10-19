
#include "imu.h"
#include "pico/stdlib.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define MPU_ADDR       0x68
#define WHO_AM_I       0x75
#define PWR_MGMT_1     0x6B
#define ACCEL_CONFIG   0x1C
#define GYRO_CONFIG    0x1B
#define ACCEL_XOUT_H   0x3B

#define GYRO_SCALE     131.0f
#define ACCEL_SCALE    16384.0f
#define FILTER_ALPHA   0.98f
#define MAX_SAMPLE_US  50000ULL
#define RAD_TO_DEG     57.2957795f

static i2c_inst_t *imu_bus = NULL;

static imu_bus_lock_fn bus_lock = NULL;
static imu_bus_unlock_fn bus_unlock = NULL;
static void *bus_context = NULL;

static imu_delay_fn delay_callback = NULL;
static void *delay_context = NULL;

static float pitch = 0.0f;
static float roll = 0.0f;
static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;

static bool imu_valid = false;
static uint64_t last_sample_us = 0;
static uint32_t last_interval_us = 0;

// -------------------------------------------------
// Shared I2C locking
// -------------------------------------------------

bool imu_set_bus_lock(
    imu_bus_lock_fn lock,
    imu_bus_unlock_fn unlock,
    void *context
) {
    if ((lock == NULL) != (unlock == NULL)) {
        return false;
    }

    bus_lock = lock;
    bus_unlock = unlock;
    bus_context = context;
    return true;
}

void imu_set_delay(imu_delay_fn delay, void *context) {
    delay_callback = delay;
    delay_context = context;
}

static void imu_delay_ms(uint32_t ms) {
    if (delay_callback != NULL) {
        delay_callback(ms, delay_context);
    } else {
        sleep_ms(ms);
    }
}

static bool acquire_bus(void) {
    return bus_lock == NULL || bus_lock(bus_context);
}

static void release_bus(void) {
    if (bus_unlock != NULL) {
        bus_unlock(bus_context);
    }
}

// -------------------------------------------------
// MPU-6500 register communication
// -------------------------------------------------

static bool write_reg(uint8_t reg, uint8_t value) {
    if (!acquire_bus()) {
        return false;
    }

    uint8_t data[2] = {reg, value};

    bool ok = i2c_write_timeout_us(
        imu_bus, MPU_ADDR, data, 2, false, 5000
    ) == 2;

    release_bus();
    return ok;
}

static bool read_regs(
    uint8_t reg,
    uint8_t *buffer,
    size_t length
) {
    if (!acquire_bus()) {
        return false;
    }

    // Protect both parts of the same I2C transaction.
    bool ok = i2c_write_timeout_us(
        imu_bus, MPU_ADDR, &reg, 1, true, 5000
    ) == 1;

    if (ok) {
        ok = i2c_read_timeout_us(
            imu_bus, MPU_ADDR, buffer,
            length, false, 5000
        ) == (int)length;
    }

    release_bus();
    return ok;
}

static int16_t to_int16(uint8_t hi, uint8_t lo) {
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

static bool read_sensor(
    int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz
) {
    uint8_t d[14];

    if (!read_regs(ACCEL_XOUT_H, d, sizeof(d))) {
        return false;
    }

    *ax = to_int16(d[0], d[1]);
    *ay = to_int16(d[2], d[3]);
    *az = to_int16(d[4], d[5]);

    *gx = to_int16(d[8], d[9]);
    *gy = to_int16(d[10], d[11]);
    *gz = to_int16(d[12], d[13]);

    return true;
}

static bool accelerometer_plausible(
    int16_t ax, int16_t ay, int16_t az
) {
    float x = ax / ACCEL_SCALE;
    float y = ay / ACCEL_SCALE;
    float z = az / ACCEL_SCALE;

    float magnitude_squared = x*x + y*y + z*z;

    return isfinite(magnitude_squared) &&
           magnitude_squared >= 0.04f &&
           magnitude_squared <= 9.0f;
}

// -------------------------------------------------
// Initialise and calibrate IMU
// -------------------------------------------------

bool imu_init(i2c_inst_t *i2c) {
    imu_valid = false;
    last_sample_us = 0;
    last_interval_us = 0;
    imu_bus = i2c;

    if (imu_bus == NULL) {
        return false;
    }

    uint8_t id = 0;

    if (!read_regs(WHO_AM_I, &id, 1) ||
        id != 0x70) {
        return false;
    }

    if (!write_reg(PWR_MGMT_1, 0x01)) {
        return false;
    }

    imu_delay_ms(100);

    if (!write_reg(ACCEL_CONFIG, 0x00) ||
        !write_reg(GYRO_CONFIG, 0x00)) {
        return false;
    }

    // Keep IMU stationary for calibration.
    float sum_x = 0.0f;
    float sum_y = 0.0f;

    for (int i = 0; i < 100; i++) {
        int16_t ax, ay, az, gx, gy, gz;

        if (!read_sensor(
                &ax, &ay, &az,
                &gx, &gy, &gz
            ) ||
            !accelerometer_plausible(ax, ay, az)) {
            return false;
        }

        sum_x += gx;
        sum_y += gy;
        imu_delay_ms(5);
    }

    gyro_offset_x = sum_x / (100.0f * GYRO_SCALE);
    gyro_offset_y = sum_y / (100.0f * GYRO_SCALE);

    int16_t ax, ay, az, gx, gy, gz;

    if (!read_sensor(
            &ax, &ay, &az,
            &gx, &gy, &gz
        ) ||
        !accelerometer_plausible(ax, ay, az)) {
        return false;
    }

    roll = atan2f(
        (float)ay, (float)az
    ) * RAD_TO_DEG;

    pitch = atan2f(
        -(float)ax,
        hypotf((float)ay, (float)az)
    ) * RAD_TO_DEG;

    if (!isfinite(roll) || !isfinite(pitch)) {
        return false;
    }

    last_sample_us = time_us_64();
    imu_valid = true;

    return true;
}

// -------------------------------------------------
// Update pitch and roll
// -------------------------------------------------

bool imu_update(void) {
    if (imu_bus == NULL) {
        imu_valid = false;
        return false;
    }

    int16_t ax, ay, az, gx, gy, gz;

    if (!read_sensor(
            &ax, &ay, &az,
            &gx, &gy, &gz
        ) ||
        !accelerometer_plausible(ax, ay, az)) {
        imu_valid = false;
        return false;
    }

    uint64_t now = time_us_64();

    uint64_t gap = last_sample_us
        ? now - last_sample_us
        : MAX_SAMPLE_US + 1;

    last_interval_us = gap > UINT32_MAX
        ? UINT32_MAX
        : (uint32_t)gap;

    float accel_roll = atan2f(
        (float)ay, (float)az
    ) * RAD_TO_DEG;

    float accel_pitch = atan2f(
        -(float)ax,
        hypotf((float)ay, (float)az)
    ) * RAD_TO_DEG;

    float gyro_x = gx / GYRO_SCALE - gyro_offset_x;
    float gyro_y = gy / GYRO_SCALE - gyro_offset_y;

    bool on_time = gap > 0 && gap <= MAX_SAMPLE_US;

    if (!imu_valid || !on_time) {
        // Recover the filter, but reject this reading.
        roll = accel_roll;
        pitch = accel_pitch;
    } else {
        float dt = (float)gap / 1000000.0f;

        float predicted_roll = roll + gyro_x * dt;
        float predicted_pitch = pitch + gyro_y * dt;

        float error = remainderf(
            accel_roll - predicted_roll,
            360.0f
        );

        roll = remainderf(
            predicted_roll +
            (1.0f - FILTER_ALPHA) * error,
            360.0f
        );

        pitch =
            FILTER_ALPHA * predicted_pitch +
            (1.0f - FILTER_ALPHA) * accel_pitch;
    }

    last_sample_us = now;

    imu_valid = on_time &&
                isfinite(roll) &&
                isfinite(pitch);

    return imu_valid;
}

// -------------------------------------------------
// Return orientation and sampling information
// -------------------------------------------------

float imu_get_pitch(void) {
    return pitch;
}

float imu_get_roll(void) {
    return roll;
}

bool imu_get_reading(imu_reading_t *out) {
    if (out == NULL) {
        return false;
    }

    out->pitch_deg = pitch;
    out->roll_deg = roll;
    out->timestamp_us = last_sample_us;
    out->interval_us = last_interval_us;

    uint64_t now = time_us_64();

    out->valid =
        imu_valid &&
        last_sample_us != 0 &&
        now >= last_sample_us &&
        now - last_sample_us <= MAX_SAMPLE_US;

    return out->valid;
}

// -------------------------------------------------
// Single-sample orientation classification
// -------------------------------------------------

imu_status_t imu_check_orientation(
    float target_pitch,
    float target_roll,
    float tolerance
) {
    imu_reading_t s;

    if (!imu_get_reading(&s) ||
        !isfinite(target_pitch) ||
        !isfinite(target_roll) ||
        !isfinite(tolerance) ||
        tolerance < 0.0f) {
        return IMU_FAULT;
    }

    float pitch_error = fabsf(
        s.pitch_deg - target_pitch
    );

    float roll_error = fabsf(
        remainderf(
            s.roll_deg - target_roll,
            360.0f
        )
    );

    if (pitch_error <= tolerance &&
        roll_error <= tolerance) {
        return IMU_WITHIN_TOLERANCE;
    }

    return IMU_OUT_OF_TOLERANCE;
}
