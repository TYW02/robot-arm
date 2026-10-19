
#include <stdio.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "imu.h"
#include "imu_verify.h"

#define I2C_PORT i2c1
#define SDA_PIN 6
#define SCL_PIN 7
#define SAMPLE_MS 20

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(I2C_PORT, 100000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    printf("Starting MPU-6500 IMU test...\n");
    printf("Keep IMU stationary during calibration.\n");

    if (!imu_init(I2C_PORT)) {
        printf("IMU initialisation failed!\n");

        while (true) {
            sleep_ms(1000);
        }
    }

    printf("IMU initialised successfully.\n");

    // Temporary target for standalone test.
    float target_pitch = imu_get_pitch();
    float target_roll = imu_get_roll();

    imu_verifier_t verifier = {0};

    uint32_t request_id = 1;
    uint64_t last_print_us = 0;

    printf("Commands:\n");
    printf("r = set current pose as target\n");
    printf("v = start verification\n");
    printf("c = cancel verification\n");

    absolute_time_t next =
        make_timeout_time_ms(SAMPLE_MS);

    while (true) {
        sleep_until(next);

        next = delayed_by_ms(
            next,
            SAMPLE_MS
        );

        bool updated = imu_update();

        imu_reading_t reading = {0};
        imu_get_reading(&reading);

        uint64_t now = time_us_64();

        int key = getchar_timeout_us(0);
        bool started_this_cycle = false;

        if (key == 'r') {
            if (updated && reading.valid) {
                // Only set this while the IMU is still.
                target_pitch = reading.pitch_deg;
                target_roll = reading.roll_deg;

                imu_verify_cancel(&verifier);

                printf(
                    "Target saved: Pitch %.2f Roll %.2f\n",
                    target_pitch,
                    target_roll
                );
            } else {
                printf("Invalid sample. Target not saved.\n");
            }
        }

        else if (key == 'v') {
            if (imu_verify_begin(
                    &verifier,
                    request_id++,
                    target_pitch,
                    target_roll,
                    now
                )) {
                // Verification starts with NEXT sample.
                started_this_cycle = true;

                printf(
                    "Verifying: 2 consecutive valid samples required.\n"
                );
            }
        }

        else if (key == 'c') {
            imu_verify_cancel(&verifier);

            printf("Verification cancelled.\n");
        }

        if (verifier.active && !started_this_cycle) {
            imu_verify_result_t result =
                imu_verify_step(
                    &verifier,
                    &reading,
                    time_us_64()
                );

            if (!updated ||
                result == IMU_VERIFY_SENSOR_FAULT) {

                printf("IMU FAULT: Release blocked.\n");
                imu_verify_cancel(&verifier);
            }

            else if (result == IMU_VERIFY_PASS) {
                printf(
                    "ORIENTATION VERIFIED: 2 valid samples.\n"
                );
            }

            else if (
                result == IMU_VERIFY_OUT_OF_TOLERANCE
            ) {
                printf(
                    "OUT OF TOLERANCE: Release blocked.\n"
                );
            }
        }

        if (now - last_print_us >= 500000ULL) {
            last_print_us = now;

            printf(
                "Pitch %.2f | Roll %.2f | "
                "Gap %" PRIu32 " us | Valid %d\n",
                reading.pitch_deg,
                reading.roll_deg,
                reading.interval_us,
                reading.valid ? 1 : 0
            );
        }
    }
}
