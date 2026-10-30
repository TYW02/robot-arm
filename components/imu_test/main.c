#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* Corrected hardware-I2C wiring. */
#define MPU_BUS       i2c1
#define MPU_SDA_PIN   2
#define MPU_SCL_PIN   3

#define MAG_BUS       i2c0
#define MAG_SDA_PIN   16
#define MAG_SCL_PIN   17

#define I2C_SPEED_HZ  100000
#define I2C_TIMEOUT_US 10000

/* MPU-6500 / MPU-9250 */
#define MPU_WHO_AM_I    0x75
#define MPU_PWR_MGMT_1  0x6B
#define MPU_PWR_MGMT_2  0x6C
#define MPU_CONFIG      0x1A
#define MPU_GYRO_CONFIG 0x1B
#define MPU_SAMPLE_DIV  0x19
#define MPU_GYRO_OUT    0x43

#define GYRO_COUNTS_PER_DPS 131.0f
#define GYRO_CAL_SAMPLES    200

/* LSM303D magnetometer */
#define MAG_ADDRESS     0x1E
#define MAG_ID          0x0F
#define MAG_STATUS      0x07
#define MAG_DATA        0x08

#define MAG_CTRL1       0x20
#define MAG_CTRL5       0x24
#define MAG_CTRL6       0x25
#define MAG_CTRL7       0x26

static uint8_t mpu_address;
static float gyro_bias[3];

static bool read_registers(i2c_inst_t *bus,
                           uint8_t address,
                           uint8_t reg,
                           uint8_t *data,
                           size_t length)
{
    int result = i2c_write_timeout_us(
        bus, address, &reg, 1, true, I2C_TIMEOUT_US);

    if (result != 1)
        return false;

    result = i2c_read_timeout_us(
        bus, address, data, length, false, I2C_TIMEOUT_US);

    return result == (int)length;
}

static bool write_register(i2c_inst_t *bus,
                           uint8_t address,
                           uint8_t reg,
                           uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_write_timeout_us(
        bus, address, data, sizeof(data),
        false, I2C_TIMEOUT_US) == (int)sizeof(data);
}

static int16_t signed_big_endian(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void initialise_bus(i2c_inst_t *bus,
                           unsigned int sda,
                           unsigned int scl)
{
    i2c_init(bus, I2C_SPEED_HZ);

    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);

    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

static void scan_bus(i2c_inst_t *bus, const char *name)
{
    unsigned int found = 0;

    printf("\n%s scan:\n", name);

    /* Avoid reserved I2C addresses. */
    for (uint8_t address = 0x08; address <= 0x77; ++address) {
        uint8_t byte;

        int result = i2c_read_timeout_us(
            bus, address, &byte, 1,
            false, I2C_TIMEOUT_US);

        if (result == 1) {
            printf("  Device responding at 0x%02X\n", address);
            ++found;
        }
    }

    if (found == 0)
        printf("  No devices detected\n");
}

static bool initialise_mpu(void)
{
    const uint8_t addresses[] = {0x68, 0x69};
    bool found = false;

    for (unsigned int i = 0; i < sizeof(addresses); ++i) {
        uint8_t identity;

        if (!read_registers(MPU_BUS, addresses[i],
                            MPU_WHO_AM_I, &identity, 1)) {
            continue;
        }

        printf("MPU address 0x%02X: WHO_AM_I = 0x%02X\n",
               addresses[i], identity);

        if (identity == 0x70 || identity == 0x71) {
            mpu_address = addresses[i];
            found = true;

            printf("  Identified as %s\n",
                   identity == 0x70 ? "MPU-6500" : "MPU-9250");
            break;
        }

        printf("  Unsupported identity; not configuring this device\n");
    }

    if (!found)
        return false;

    /* Reset, then wake with gyro PLL clock. */
    if (!write_register(MPU_BUS, mpu_address,
                        MPU_PWR_MGMT_1, 0x80))
        return false;

    sleep_ms(100);

    if (!write_register(MPU_BUS, mpu_address,
                        MPU_PWR_MGMT_1, 0x01))
        return false;

    sleep_ms(100);

    /*
     * Enable all axes.
     * Gyro DLPF enabled, approximately 41 Hz bandwidth.
     * 1 kHz / (9 + 1) = 100 Hz sample rate.
     * Gyroscope range: +/-250 degrees/second.
     */
    return write_register(MPU_BUS, mpu_address,
                          MPU_PWR_MGMT_2, 0x00)
        && write_register(MPU_BUS, mpu_address,
                          MPU_CONFIG, 0x03)
        && write_register(MPU_BUS, mpu_address,
                          MPU_SAMPLE_DIV, 9)
        && write_register(MPU_BUS, mpu_address,
                          MPU_GYRO_CONFIG, 0x00);
}

static bool read_gyro(float rates[3])
{
    uint8_t data[6];

    if (!read_registers(MPU_BUS, mpu_address,
                        MPU_GYRO_OUT, data, sizeof(data)))
        return false;

    for (unsigned int axis = 0; axis < 3; ++axis) {
        rates[axis] =
            signed_big_endian(&data[axis * 2]) /
            GYRO_COUNTS_PER_DPS;
    }

    return true;
}

static bool calibrate_gyro(void)
{
    float sum[3] = {0};
    float sum_squared[3] = {0};

    printf("\nKeep both boards completely stationary.\n");
    printf("Gyro offset measurement starts in 3 seconds...\n");
    sleep_ms(3000);

    for (unsigned int sample = 0;
         sample < GYRO_CAL_SAMPLES;
         ++sample) {
        float rates[3];

        if (!read_gyro(rates)) {
            printf("Gyro read failed during calibration\n");
            return false;
        }

        for (unsigned int axis = 0; axis < 3; ++axis) {
            sum[axis] += rates[axis];
            sum_squared[axis] += rates[axis] * rates[axis];
        }

        sleep_ms(10);
    }

    for (unsigned int axis = 0; axis < 3; ++axis) {
        float mean = sum[axis] / GYRO_CAL_SAMPLES;
        float variance =
            sum_squared[axis] / GYRO_CAL_SAMPLES - mean * mean;

        /*
         * Simple motion/noise check, not a full calibration validation.
         * Constant rotation could still pass: keep the sensor stationary.
         */
        if (variance > 1.0f) {
            printf("Calibration too noisy on axis %u; restart while still\n",
                   axis);
            return false;
        }

        gyro_bias[axis] = mean;
    }

    printf("Gyro offsets: X=%.3f Y=%.3f Z=%.3f deg/s\n",
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    return true;
}

static bool initialise_magnetometer(void)
{
    uint8_t identity = 0;

    if (!read_registers(MAG_BUS, MAG_ADDRESS,
                        MAG_ID, &identity, 1)) {
        printf("Magnetometer identity read failed\n");
        return false;
    }

    printf("Magnetometer WHO_AM_I: 0x%02X\n", identity);

    if (identity != 0x49) {
        printf("Expected LSM303D identity 0x49\n");
        return false;
    }

    /*
     * CTRL1: block data update enabled; accelerometer powered down.
     * CTRL5: high-resolution magnetometer, 25 Hz.
     * CTRL6: +/-2 gauss.
     * CTRL7: continuous magnetic conversion.
     */
    if (!write_register(MAG_BUS, MAG_ADDRESS, MAG_CTRL1, 0x08) ||
        !write_register(MAG_BUS, MAG_ADDRESS, MAG_CTRL5, 0x6C) ||
        !write_register(MAG_BUS, MAG_ADDRESS, MAG_CTRL6, 0x00) ||
        !write_register(MAG_BUS, MAG_ADDRESS, MAG_CTRL7, 0x00)) {
        printf("Magnetometer configuration write failed\n");
        return false;
    }

    sleep_ms(100);
    return true;
}

typedef enum {
    MAG_READ_ERROR = -1,
    MAG_NO_SAMPLE = 0,
    MAG_FRESH = 1,
    MAG_OVERRUN = 2
} mag_read_result_t;

static mag_read_result_t read_magnetometer(int16_t magnetic[3])
{
    uint8_t status;
    uint8_t data[6];

    if (!read_registers(MAG_BUS, MAG_ADDRESS, MAG_STATUS, &status, 1))
        return MAG_READ_ERROR;

    if ((status & 0x08) == 0)
        return MAG_NO_SAMPLE;

    if (!read_registers(MAG_BUS, MAG_ADDRESS, MAG_DATA | 0x80,
                        data, sizeof(data)))
        return MAG_READ_ERROR;

    for (unsigned int axis = 0; axis < 3; ++axis) {
        unsigned int index = axis * 2;
        magnetic[axis] = (int16_t)(
            (uint16_t)data[index] |
            ((uint16_t)data[index + 1] << 8));
    }

    return (status & 0x80) ? MAG_OVERRUN : MAG_FRESH;
}

/*
 * Level-only magnetic heading calibration.
 * Derived from the latest 488-sample rotation capture.
 *
 * field_norm now represents corrected HORIZONTAL field magnitude,
 * not the previous full 3D field magnitude.
 */
static bool calculate_mag_heading(const int16_t raw[3],
                                  float *heading_deg,
                                  float *field_norm)
{
    const float offset_x = -2091.9877f;
    const float offset_y = -1432.6338f;

    float x = (float)raw[0] - offset_x;
    float y = (float)raw[1] - offset_y;

    float corrected_x =
        0.00025042909f * x
    - 0.00000296088f * y;

    float corrected_y =
        -0.00000296088f * x
    + 0.00026967533f * y;

    float horizontal_squared =
        corrected_x * corrected_x +
        corrected_y * corrected_y;

    if (!isfinite(horizontal_squared) ||
        horizontal_squared < 0.000001f)
        return false;

    *field_norm = sqrtf(horizontal_squared);

    float angle = atan2f(-corrected_y, corrected_x)
                  * 57.2957795f;

    if (angle < 0.0f)
        angle += 360.0f;

    *heading_deg = angle;
    return true;
}

#define GYRO_PERIOD_US       10000u
#define PRINT_PERIOD_US     250000u
#define MAG_MAX_AGE_US      300000u
#define FUSION_TAU_S        2.0f
#define FIELD_XY_MIN        0.75f
#define FIELD_XY_MAX        1.25f
#define MAG_MAX_ERROR_DEG   45.0f
#define GYRO_Z_SIGN         1.0f

static float wrap_360(float angle)
{
    angle = fmodf(angle, 360.0f);
    return angle < 0.0f ? angle + 360.0f : angle;
}

static float wrap_180(float angle)
{
    return wrap_360(angle + 180.0f) - 180.0f;
}

int main(void)
{
    stdio_init_all();
    while (!stdio_usb_connected())
        sleep_ms(100);
    sleep_ms(500);

    printf("\n=== Standalone gyro + magnetometer fusion ===\n");
    printf("MPU: SDA GP%u, SCL GP%u\n",
           (unsigned)MPU_SDA_PIN, (unsigned)MPU_SCL_PIN);
    printf("GY-511: SDA GP%u, SCL GP%u\n",
           (unsigned)MAG_SDA_PIN, (unsigned)MAG_SCL_PIN);

    initialise_bus(MPU_BUS, MPU_SDA_PIN, MPU_SCL_PIN);
    initialise_bus(MAG_BUS, MAG_SDA_PIN, MAG_SCL_PIN);
    scan_bus(MPU_BUS, "MPU / I2C1");
    scan_bus(MAG_BUS, "GY-511 / I2C0");

    bool gyro_ok = initialise_mpu();
    bool mag_ok = initialise_magnetometer();

    printf("MPU initialisation: %s\n", gyro_ok ? "OK" : "FAILED");
    printf("Magnetometer initialisation: %s\n",
           mag_ok ? "OK" : "FAILED");

    if (!gyro_ok || !mag_ok)
        return 1;

    if (!calibrate_gyro())
        return 1;

    printf("Keep level. Wait 5 seconds before turning.\n");
    printf("REL and GYRO are cumulative degrees from startup.\n");

    bool ready = false;
    bool have_previous_gyro = false;

    float fused = 0.0f;
    float relative = 0.0f;
    float gyro_relative = 0.0f;
    float magnetic_heading = 0.0f;
    float field_xy = 0.0f;
    float gyro_z = 0.0f;
    float gyro_hz = 0.0f;
    float mag_hz = 0.0f;

    uint32_t mag_errors = 0;
    uint32_t rejected = 0;
    uint32_t overruns = 0;
    uint32_t late = 0;
    uint32_t gyro_count = 0;
    uint32_t mag_count = 0;

    uint64_t next_tick = time_us_64();
    uint64_t previous_gyro_us = next_tick;
    uint64_t last_mag_us = 0;
    uint64_t rate_window_us = next_tick;
    uint64_t next_print_us = next_tick + PRINT_PERIOD_US;

    while (true) {
        sleep_until(from_us_since_boot(next_tick));

        float rates[3];

        if (!read_gyro(rates)) {
            printf("STOP: gyro read failed. Reset to restart the test.\n");
            return 1;
        }

        uint64_t now = time_us_64();
        float dt = (float)(now - previous_gyro_us) * 0.000001f;

        previous_gyro_us = now;
        ++gyro_count;

        if (have_previous_gyro && dt > 0.050f) {
            printf("STOP: sampling gap %.1f ms. Reset to restart.\n",
                   dt * 1000.0f);
            return 1;
        }

        if (fabsf(rates[0]) > 245.0f ||
            fabsf(rates[1]) > 245.0f ||
            fabsf(rates[2]) > 245.0f) {
            printf("STOP: gyro near range limit. Turn more slowly.\n");
            return 1;
        }

        gyro_z = GYRO_Z_SIGN * (rates[2] - gyro_bias[2]);

        /* Predict heading from the gyro using measured elapsed time. */
        if (ready && have_previous_gyro) {
            float delta = gyro_z * dt;

            fused = wrap_360(fused + delta);
            relative += delta;
            gyro_relative += delta;
        }

        have_previous_gyro = true;

        /*
         * Poll readiness each tick. The magnetometer produces about
         * 25 fresh samples per second; each is fused only once.
         */
        int16_t magnetic[3];
        mag_read_result_t result = read_magnetometer(magnetic);

        if (result == MAG_READ_ERROR) {
            ++mag_errors;
        } else if (result > MAG_NO_SAMPLE) {
            ++mag_count;

            if (result == MAG_OVERRUN)
                ++overruns;

            bool usable = calculate_mag_heading(
                magnetic, &magnetic_heading, &field_xy);

            usable = usable &&
                     field_xy >= FIELD_XY_MIN &&
                     field_xy <= FIELD_XY_MAX;

            float error = ready
                ? wrap_180(magnetic_heading - fused) : 0.0f;

            if (usable &&
                (!ready || fabsf(error) <= MAG_MAX_ERROR_DEG)) {

                if (!ready) {
                    fused = magnetic_heading;
                    relative = 0.0f;
                    gyro_relative = 0.0f;
                    ready = true;
                } else {
                    float mag_dt =
                        (float)(now - last_mag_us) * 0.000001f;

                    /* Avoid a sudden large correction after an outage. */
                    if (mag_dt > 0.100f)
                        mag_dt = 0.100f;

                    float gain = mag_dt / (FUSION_TAU_S + mag_dt);
                    float correction = gain * error;

                    fused = wrap_360(fused + correction);
                    relative += correction;
                }

                last_mag_us = now;
            } else {
                ++rejected;
            }
        }

        /* MAG_NO_SAMPLE is normal and requires no correction. */

        now = time_us_64();

        if (now - rate_window_us >= 1000000u) {
            float seconds =
                (float)(now - rate_window_us) * 0.000001f;

            gyro_hz = (float)gyro_count / seconds;
            mag_hz = (float)mag_count / seconds;

            gyro_count = mag_count = 0;
            rate_window_us = now;
        }

        if (now >= next_print_us) {
            next_print_us = now + PRINT_PERIOD_US;

            if (!ready) {
                printf("WAIT_MAG M_ERR=%lu REJ=%lu\n",
                       (unsigned long)mag_errors,
                       (unsigned long)rejected);
            } else {
                const char *mode =
                    now - last_mag_us <= MAG_MAX_AGE_US
                    ? "FUSED" : "GYRO_ONLY";

                printf("FUSED=%7.2f REL=%8.2f GYRO=%8.2f "
                       "MAG=%7.2f GZ=%7.2f FIELD=%.3f MODE=%s "
                       "G_HZ=%.1f M_HZ=%.1f M_ERR=%lu REJ=%lu "
                       "OVR=%lu LATE=%lu\n",
                       fused, relative, gyro_relative,
                       magnetic_heading, gyro_z, field_xy, mode,
                       gyro_hz, mag_hz,
                       (unsigned long)mag_errors,
                       (unsigned long)rejected,
                       (unsigned long)overruns,
                       (unsigned long)late);
            }
        }

        next_tick += GYRO_PERIOD_US;
        now = time_us_64();

        /* Skip missed deadlines instead of taking catch-up readings. */
        if (next_tick <= now) {
            ++late;
            next_tick = now + GYRO_PERIOD_US;
        }
    }
}