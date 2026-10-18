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
// PCA9685 CHANNELS
// ============================================================

#define BASE_CH          0
#define SHOULDER_CH      1
#define ELBOW_CH         2

#define GRIPPER_CH      13
#define WRIST_ROTATE_CH 14
#define WRIST_PITCH_CH  15


// ============================================================
// SAFE LIMITS
// ============================================================

#define BASE_MIN_US          500
#define BASE_MAX_US         2600

#define SHOULDER_MIN_US     1300
#define SHOULDER_MAX_US     2450

#define ELBOW_MIN_US        1100
#define ELBOW_MAX_US        2500

#define WRIST_PITCH_MIN_US   500
#define WRIST_PITCH_MAX_US  1850

#define WRIST_ROTATE_US      400
#define GRIPPER_OPEN_US     1450


// ============================================================
// MOVEMENT SETTINGS
// ============================================================

// 40 x 20 ms = approximately 0.8 s per movement stage
#define MOTION_STEPS       40
#define MOTION_DELAY_MS    20

#define STAGE_PAUSE_MS    300
#define POSE_HOLD_MS     3000


// ============================================================
// ARM POSE
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
// DEFINED POSITIONS
// ============================================================

// 1. HOME
const ArmPose HOME = {
    .base         = 1450,
    .shoulder     = 2000,
    .elbow        = 1800,
    .wrist_pitch  = 700,
    .wrist_rotate = 400,
    .gripper      = 1450
};


// 2. SHELF SCAN
const ArmPose SCAN_SHELF = {
    .base         = 550,
    .shoulder     = 1850,
    .elbow        = 1900,
    .wrist_pitch  = 1600,
    .wrist_rotate = 400,
    .gripper      = 1450
};


// 3. ITEM SCAN

const ArmPose SCAN_ITEMS = {
    .base         = 2900,
    .shoulder     = 1700,
    .elbow        = 1500,
    .wrist_pitch  = 800,
    .wrist_rotate = 400,
    .gripper      = 1450
};


// ============================================================
// TRANSIT POSTURE
//
// Shoulder/elbow are returned here BEFORE large base rotation.
// Base is not included because it changes depending on destination.
// ============================================================

#define TRANSIT_SHOULDER_US   2000
#define TRANSIT_ELBOW_US      1800
#define TRANSIT_WRIST_US       700


// ============================================================
// CURRENT POSE
// ============================================================

ArmPose current_pose;


// ============================================================
// PCA9685 WRITE
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
// PCA9685 INITIALIZATION
// ============================================================

static void pca_init(void)
{
    // Sleep
    pca_write(MODE1, 0x10);

    // Approx. 50 Hz
    pca_write(PRESCALE, 121);

    // Totem-pole outputs
    pca_write(MODE2, 0x04);

    // Wake + auto increment
    pca_write(MODE1, 0x20);

    sleep_ms(10);
}


// ============================================================
// RAW SERVO OUTPUT
// ============================================================

static void servo_set_us(uint8_t channel, uint16_t pulse_us)
{
    uint16_t count =
        ((uint32_t)pulse_us * 4096) / 20000;

    uint8_t reg =
        LED0_ON_L + (4 * channel);

    uint8_t data[5];

    data[0] = reg;

    // ON = 0
    data[1] = 0x00;
    data[2] = 0x00;

    // OFF = count
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
// CLAMP
// ============================================================

static int clamp_value(
    int value,
    int min_value,
    int max_value)
{
    if (value < min_value)
        return min_value;

    if (value > max_value)
        return max_value;

    return value;
}


// ============================================================
// SMOOTH SINGLE-SERVO MOVEMENT
// ============================================================

static void move_servo_smooth(
    uint8_t channel,
    int start_us,
    int target_us,
    int steps,
    int delay_ms)
{
    for (int i = 1; i <= steps; i++)
    {
        int pulse =
            start_us +
            ((target_us - start_us) * i) / steps;

        servo_set_us(channel, pulse);

        sleep_ms(delay_ms);
    }
}


// ============================================================
// SHOULDER + ELBOW TOGETHER
// ============================================================

static void move_shoulder_elbow(
    int shoulder_start,
    int shoulder_target,
    int elbow_start,
    int elbow_target)
{
    shoulder_target =
        clamp_value(
            shoulder_target,
            SHOULDER_MIN_US,
            SHOULDER_MAX_US
        );

    elbow_target =
        clamp_value(
            elbow_target,
            ELBOW_MIN_US,
            ELBOW_MAX_US
        );

    for (int i = 1; i <= MOTION_STEPS; i++)
    {
        int shoulder =
            shoulder_start +
            ((shoulder_target - shoulder_start) * i)
            / MOTION_STEPS;

        int elbow =
            elbow_start +
            ((elbow_target - elbow_start) * i)
            / MOTION_STEPS;

        servo_set_us(
            SHOULDER_CH,
            shoulder
        );

        servo_set_us(
            ELBOW_CH,
            elbow
        );

        sleep_ms(MOTION_DELAY_MS);
    }
}


// ============================================================
// SET A POSE IMMEDIATELY
//
// Only use this for initial HOME setup.
// ============================================================

static void set_pose_direct(ArmPose pose)
{
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
        WRIST_ROTATE_US
    );

    servo_set_us(
        GRIPPER_CH,
        GRIPPER_OPEN_US
    );

    current_pose = pose;
}


// ============================================================
// MOVE TO ANY NAMED POSE
//
// Staged movement:
// 1. Wrist -> transit
// 2. Shoulder/elbow -> transit
// 3. Rotate base
// 4. Shoulder/elbow -> target
// 5. Wrist pitch -> target
// ============================================================

static void move_to_pose(
    const char *name,
    ArmPose target)
{
    printf("\nMoving to %s\n", name);


    // --------------------------------------------------------
    // Stage 1 - bring wrist to safer transit angle
    // --------------------------------------------------------

    printf("  Stage 1: Wrist -> transit\n");

    move_servo_smooth(
        WRIST_PITCH_CH,
        current_pose.wrist_pitch,
        TRANSIT_WRIST_US,
        MOTION_STEPS,
        MOTION_DELAY_MS
    );

    current_pose.wrist_pitch =
        TRANSIT_WRIST_US;

    sleep_ms(STAGE_PAUSE_MS);


    // --------------------------------------------------------
    // Stage 2 - retract shoulder + elbow
    // --------------------------------------------------------

    printf("  Stage 2: Shoulder + Elbow -> transit\n");

    move_shoulder_elbow(
        current_pose.shoulder,
        TRANSIT_SHOULDER_US,

        current_pose.elbow,
        TRANSIT_ELBOW_US
    );

    current_pose.shoulder =
        TRANSIT_SHOULDER_US;

    current_pose.elbow =
        TRANSIT_ELBOW_US;

    sleep_ms(STAGE_PAUSE_MS);


    // --------------------------------------------------------
    // Stage 3 - rotate base
    // --------------------------------------------------------

    printf("  Stage 3: Base rotation\n");

    target.base =
        clamp_value(
            target.base,
            BASE_MIN_US,
            BASE_MAX_US
        );

    move_servo_smooth(
        BASE_CH,
        current_pose.base,
        target.base,
        MOTION_STEPS,
        MOTION_DELAY_MS
    );

    current_pose.base =
        target.base;

    sleep_ms(STAGE_PAUSE_MS);


    // --------------------------------------------------------
    // Stage 4 - shoulder + elbow to target
    // --------------------------------------------------------

    printf("  Stage 4: Shoulder + Elbow -> target\n");

    move_shoulder_elbow(
        current_pose.shoulder,
        target.shoulder,

        current_pose.elbow,
        target.elbow
    );

    current_pose.shoulder =
        target.shoulder;

    current_pose.elbow =
        target.elbow;

    sleep_ms(STAGE_PAUSE_MS);


    // --------------------------------------------------------
    // Stage 5 - wrist pitch to target
    // --------------------------------------------------------

    printf("  Stage 5: Wrist -> target\n");

    target.wrist_pitch =
        clamp_value(
            target.wrist_pitch,
            WRIST_PITCH_MIN_US,
            WRIST_PITCH_MAX_US
        );

    move_servo_smooth(
        WRIST_PITCH_CH,
        current_pose.wrist_pitch,
        target.wrist_pitch,
        MOTION_STEPS,
        MOTION_DELAY_MS
    );

    current_pose.wrist_pitch =
        target.wrist_pitch;


    // --------------------------------------------------------
    // Wrist rotate + gripper remain fixed
    // --------------------------------------------------------

    servo_set_us(
        WRIST_ROTATE_CH,
        WRIST_ROTATE_US
    );

    servo_set_us(
        GRIPPER_CH,
        GRIPPER_OPEN_US
    );

    current_pose.wrist_rotate =
        WRIST_ROTATE_US;

    current_pose.gripper =
        GRIPPER_OPEN_US;


    printf("%s reached\n", name);
}


// ============================================================
// MAIN
// ============================================================

int main(void)
{
    stdio_init_all();

    sleep_ms(2000);

    printf("\n");
    printf("====================================\n");
    printf(" Robot Arm Scan Sequence Test\n");
    printf("====================================\n");


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

    printf("PCA9685 initialized\n");


    // ========================================================
    // 1. HOME
    // ========================================================

    printf("\n1. HOME\n");

    set_pose_direct(HOME);

    sleep_ms(POSE_HOLD_MS);


    // ========================================================
    // 2. SCAN SHELF
    // ========================================================

    printf("\n2. SCAN SHELF\n");

    move_to_pose(
        "SCAN SHELF",
        SCAN_SHELF
    );

    sleep_ms(POSE_HOLD_MS);


    // ========================================================
    // 3. SCAN ITEMS
    // ========================================================

    printf("\n3. SCAN ITEMS\n");

    move_to_pose(
        "SCAN ITEMS",
        SCAN_ITEMS
    );

    sleep_ms(POSE_HOLD_MS);


    // ========================================================
    // 4. SCAN SHELF AGAIN
    // ========================================================

    printf("\n4. SCAN SHELF\n");

    move_to_pose(
        "SCAN SHELF",
        SCAN_SHELF
    );

    sleep_ms(POSE_HOLD_MS);


    // ========================================================
    // 5. HOME
    // ========================================================

    printf("\n5. HOME\n");

    move_to_pose(
        "HOME",
        HOME
    );

    sleep_ms(POSE_HOLD_MS);


    // ========================================================
    // FINISHED
    // ========================================================

    printf("\nSequence complete.\n");

    while (true)
    {
        tight_loop_contents();
    }
}