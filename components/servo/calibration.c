#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ============================================================
// I2C / PCA9685
// ============================================================

#define I2C_PORT       i2c1
#define SDA_PIN        2
#define SCL_PIN        3

#define PCA9685_ADDR   0x40

#define MODE1          0x00
#define MODE2          0x01
#define LED0_ON_L      0x06
#define PRESCALE       0xFE


// ============================================================
// SERVO BEING TESTED
// ============================================================

#define BASE_CH 0
#define SHOULDER_CH 1
#define ELBOW_CH 4
#define WRIST_PITCH_CH 15
#define WRIST_ROTATE_CH 13
#define GRIPPER_CH 11


// ============================================================
// CHANGE ONLY THIS VALUE FOR EACH TEST
// ============================================================



// ============================================================
// PCA9685 register write
// ============================================================

static void pca_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {
        reg,
        value
    };

    i2c_write_blocking(
        I2C_PORT,
        PCA9685_ADDR,
        data,
        2,
        false
    );
}


// ============================================================
// PCA9685 initialization
// ============================================================

static void pca_init(void)
{
    // Sleep mode
    pca_write(MODE1, 0x10);

    // Approx. 50 Hz
    pca_write(PRESCALE, 121);

    // Totem-pole output
    pca_write(MODE2, 0x04);

    // Wake + auto increment
    pca_write(MODE1, 0x20);

    sleep_ms(10);
}


// ============================================================
// Set servo pulse width
// ============================================================

static void servo_set_us(uint8_t channel, uint16_t pulse_us)
{
    // 50 Hz = 20,000 us period
    uint16_t count =
        ((uint32_t)pulse_us * 4096) / 20000;

    uint8_t reg =
        LED0_ON_L + (4 * channel);

    uint8_t data[5];

    data[0] = reg;

    // ON count = 0
    data[1] = 0x00;
    data[2] = 0x00;

    // OFF count
    data[3] = count & 0xFF;
    data[4] = (count >> 8) & 0x0F;

    i2c_write_blocking(
        I2C_PORT,
        PCA9685_ADDR,
        data,
        5,
        false
    );
}


// ============================================================
// MAIN
// ============================================================

int main(void)
{
    stdio_init_all();

    sleep_ms(2000);

    // ----------------------------
    // Initialise I2C
    // ----------------------------

    i2c_init(I2C_PORT, 100000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);


    // ----------------------------
    // Initialise PCA9685
    // ----------------------------

    pca_init();


    // ----------------------------
    // BASE SERVO TEST
    // ----------------------------

    while (true)
    {
        servo_set_us(BASE_CH,400);
        sleep_ms(2000);
        servo_set_us(BASE_CH,1550);
        sleep_ms(2000);
    }
}
