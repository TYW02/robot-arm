
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "imu.h"

// I2C connection
#define I2C_PORT i2c1
#define SDA_PIN 6
#define SCL_PIN 7

// Temporary orientation tolerance for integration testing
#define ANGLE_TOLERANCE 5.0f

int main() {
    stdio_init_all();
    sleep_ms(2000);

    // Initialise the shared I2C bus
    i2c_init(I2C_PORT, 100000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    printf("Robotic Arm IMU System Starting...\n");
    printf("Keep IMU stationary during calibration.\n");

    // Initialise and calibrate the IMU
    if (!imu_init(I2C_PORT)) {
        printf("ERROR: IMU initialisation failed!\n");

        while (true) {
            sleep_ms(1000);
        }
    }

    printf("IMU Initialised Successfully!\n");

    // Temporary target: orientation at startup
    // Later supplied by the robotic arm controller
    float target_pitch = imu_get_pitch();
    float target_roll = imu_get_roll();

    printf("Target Pitch: %.2f\n", target_pitch);
    printf("Target Roll: %.2f\n", target_roll);

    uint32_t last_print_ms = to_ms_since_boot(
        get_absolute_time()
    );

    while (true) {
        // Update IMU orientation
        bool updated = imu_update();

        // Never approve orientation if sampling failed
        imu_status_t status = IMU_FAULT;

        if (updated) {
            status = imu_check_orientation(
                target_pitch,
                target_roll,
                ANGLE_TOLERANCE
            );
        }

        uint32_t now_ms = to_ms_since_boot(
            get_absolute_time()
        );

        // Display results every 500 ms
        if (now_ms - last_print_ms >= 500) {
            last_print_ms = now_ms;

            printf("\nPitch: %.2f degrees\n",
                   imu_get_pitch());

            printf("Roll: %.2f degrees\n",
                   imu_get_roll());

            if (status == IMU_WITHIN_TOLERANCE) {
                printf("Status: WITHIN TOLERANCE\n");
            }
            else if (status == IMU_OUT_OF_TOLERANCE) {
                printf("Status: OUT OF TOLERANCE\n");
            }
            else {
                printf("Status: IMU FAULT\n");
            }
        }

        // Aim for approximately 20 ms sampling
        sleep_ms(20);
    }

    return 0;
}
