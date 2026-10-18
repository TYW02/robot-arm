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
// PCA9685 servo channels
// ============================================================

#define BASE_CH          0
#define SHOULDER_CH      1
#define ELBOW_CH         2

#define GRIPPER_CH      13
#define WRIST_ROTATE_CH 14
#define WRIST_PITCH_CH  15


// ============================================================
// SAFE SOFTWARE LIMITS
//
// These are intentionally inside your measured extremes.
// Change them later if your testing proves a larger range is safe.
// ============================================================

// Base measured approximately 400 - 2800 us
#define BASE_MIN_US          500
#define BASE_MAX_US         2600

// Shoulder measured approximately 1250 - 2500 us
#define SHOULDER_MIN_US     1300
#define SHOULDER_MAX_US     2450

// Elbow measured approximately 1000/1100 - 2600 us
#define ELBOW_MIN_US        1100
#define ELBOW_MAX_US        2500

// Wrist pitch measured approximately 450 - 1900 us
#define WRIST_PITCH_MIN_US   500
#define WRIST_PITCH_MAX_US  1850

// Wrist rotate measured approximately 400 - 1450 us
#define WRIST_ROTATE_MIN_US  400
#define WRIST_ROTATE_MAX_US 400

// Gripper measured approximately 1400 - 2800 us
#define GRIPPER_MIN_US      1450
#define GRIPPER_MAX_US      2700


// ============================================================
// Arm pose structure
// ============================================================

typedef struct
{
    int base;
    int shoulder;
    int elbow;
    int wrist_pitch;
    int wrist_rotate;
    int gripper;

} ArmPose;


// ============================================================
// Initial HOME pose
//
// These are starting values based on your calibration.
// Verify physically before using a large movement.
// ============================================================

ArmPose HOME = {
    .base         = 1450,
    .shoulder     = 2000,
    .elbow        = 1800,
    .wrist_pitch  = 700,
    .wrist_rotate = 400,   // head motor: keep fixed
    .gripper      = 2800   // open
};


// ============================================================
// Small coordinated test pose
//
// Intentionally close to HOME for first testing.
// ============================================================

ArmPose SCAN_POSE = {

    .base         = 2750,

    .shoulder     = 1700,

    .elbow        = 1500,

    .wrist_pitch  = 800,

    .wrist_rotate = 400, //keep head fixed 

    .gripper      = 2800
};


// ============================================================
// PCA9685 register write
// ============================================================

void pca_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2];

    data[0] = reg;
    data[1] = value;

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
// Approximately 50 Hz servo PWM
// ============================================================

void pca_init(void)
{
    // Enter sleep so prescaler can be changed
    pca_write(MODE1, 0x10);

    // Approx. 50 Hz assuming nominal 25 MHz PCA9685 oscillator
    pca_write(PRESCALE, 121);

    // Totem-pole outputs
    pca_write(MODE2, 0x04);

    // Wake + auto increment
    pca_write(MODE1, 0x20);

    sleep_ms(10);
}


// ============================================================
// Raw servo pulse output
// ============================================================

void servo_set_us(uint8_t channel, uint16_t pulse_us)
{
    // 50 Hz -> 20,000 us period
    uint16_t count =
        ((uint32_t)pulse_us * 4096) / 20000;

    uint8_t reg =
        LED0_ON_L + (4 * channel);

    uint8_t data[5];

    data[0] = reg;

    // ON count = 0
    data[1] = 0x00;
    data[2] = 0x00;

    // OFF count = calculated PWM count
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
// Clamp helper
// ============================================================

int clamp_value(
    int value,
    int min_value,
    int max_value
)
{
    if (value < min_value)
        return min_value;

    if (value > max_value)
        return max_value;

    return value;
}


// ============================================================
// Apply software safety limits to pose
// ============================================================

ArmPose clamp_pose(ArmPose pose)
{
    pose.base =
        clamp_value(
            pose.base,
            BASE_MIN_US,
            BASE_MAX_US
        );

    pose.shoulder =
        clamp_value(
            pose.shoulder,
            SHOULDER_MIN_US,
            SHOULDER_MAX_US
        );

    pose.elbow =
        clamp_value(
            pose.elbow,
            ELBOW_MIN_US,
            ELBOW_MAX_US
        );

    pose.wrist_pitch =
        clamp_value(
            pose.wrist_pitch,
            WRIST_PITCH_MIN_US,
            WRIST_PITCH_MAX_US
        );

    pose.wrist_rotate =
        clamp_value(
            pose.wrist_rotate,
            WRIST_ROTATE_MIN_US,
            WRIST_ROTATE_MAX_US
        );

    pose.gripper =
        clamp_value(
            pose.gripper,
            GRIPPER_MIN_US,
            GRIPPER_MAX_US
        );

    return pose;
}


// ============================================================
// Write complete arm pose
// ============================================================

void write_pose(ArmPose pose)
{
    pose = clamp_pose(pose);

    servo_set_us(
        BASE_CH,
        pose.base
    );

    servo_set_us(
        SHOULDER_CH,
        pose.shoulder
    );

    servo_set_us(
        ELBOW_CH,
        pose.elbow
    );

    servo_set_us(
        WRIST_PITCH_CH,
        pose.wrist_pitch
    );

    servo_set_us(
        WRIST_ROTATE_CH,
        pose.wrist_rotate
    );

    servo_set_us(
        GRIPPER_CH,
        pose.gripper
    );
}


// ============================================================
// Integer interpolation
//
// Returns a value between start and end.
// ============================================================

int interpolate(
    int start,
    int end,
    int step,
    int total_steps
)
{
    return start +
           ((end - start) * step)
           / total_steps;
}


// ============================================================
// Smooth coordinated movement
//
// All joints progressively move toward their target together.
// ============================================================

void move_arm_smooth(
    ArmPose start,
    ArmPose target,
    int steps,
    int delay_ms
)
{
    target = clamp_pose(target);

    for (int i = 1; i <= steps; i++)
    {
        ArmPose current;

        current.base =
            interpolate(
                start.base,
                target.base,
                i,
                steps
            );

        current.shoulder =
            interpolate(
                start.shoulder,
                target.shoulder,
                i,
                steps
            );

        current.elbow =
            interpolate(
                start.elbow,
                target.elbow,
                i,
                steps
            );

        current.wrist_pitch =
            interpolate(
                start.wrist_pitch,
                target.wrist_pitch,
                i,
                steps
            );

        current.wrist_rotate =
            interpolate(
                start.wrist_rotate,
                target.wrist_rotate,
                i,
                steps
            );

        current.gripper =
            interpolate(
                start.gripper,
                target.gripper,
                i,
                steps
            );

        write_pose(current);

        sleep_ms(delay_ms);
    }
}


// ============================================================
// Print pose
// ============================================================

void print_pose(
    const char *name,
    ArmPose pose
)
{
    printf("\n%s\n", name);

    printf(
        "Base:          %d us\n",
        pose.base
    );

    printf(
        "Shoulder:      %d us\n",
        pose.shoulder
    );

    printf(
        "Elbow:         %d us\n",
        pose.elbow
    );

    printf(
        "Wrist pitch:   %d us\n",
        pose.wrist_pitch
    );

    printf(
        "Wrist rotate:  %d us\n",
        pose.wrist_rotate
    );

    printf(
        "Gripper:       %d us\n",
        pose.gripper
    );
}


// ============================================================
// MAIN
// ============================================================

int main(void)
{
    stdio_init_all();

    sleep_ms(2000);

    printf("\n");
    printf("===============================\n");
    printf(" Coordinated Arm Movement Test\n");
    printf("===============================\n");

    // --------------------------------------------------------
    // I2C initialization
    // --------------------------------------------------------

    i2c_init(
        I2C_PORT,
        100000
    );

    gpio_set_function(
        SDA_PIN,
        GPIO_FUNC_I2C
    );

    gpio_set_function(
        SCL_PIN,
        GPIO_FUNC_I2C
    );

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);


    // --------------------------------------------------------
    // PCA9685 initialization
    // --------------------------------------------------------

    pca_init();

    printf("PCA9685 initialized.\n");


    // --------------------------------------------------------
    // Display configured poses
    // --------------------------------------------------------

    print_pose(
        "HOME POSE",
        HOME
    );

    print_pose(
        "SCAN POSE",
        SCAN_POSE
    );


    // --------------------------------------------------------
    // IMPORTANT:
    //
    // Before powering movement, physically position the arm
    // reasonably close to HOME if possible.
    //
    // With normal hobby servos there is no encoder feedback
    // to tell us the actual starting angle.
    // --------------------------------------------------------

    printf("\nMoving to HOME...\n");

    write_pose(HOME);

    sleep_ms(3000);


    // --------------------------------------------------------
    // Coordinated movement loop
    // --------------------------------------------------------

    while (true)
    {
        printf("\nHOME -> TEST\n");

        move_arm_smooth(
            HOME,
            SCAN_POSE,

            // number of interpolation steps
            100,

            // 20 ms per step
            20
        );

        sleep_ms(2000);


        printf("TEST -> HOME\n");

        move_arm_smooth(
            SCAN_POSE,
            HOME,
            100,
            20
        );

        sleep_ms(3000);
    }
}