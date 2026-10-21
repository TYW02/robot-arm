#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>

#include "robot_arm.h"

#define PCA9685_ADDRESS       0x40
#define PCA9685_MODE1         0x00
#define PCA9685_MODE2         0x01
#define PCA9685_LED0_ON_L     0x06
#define PCA9685_PRESCALE      0xFE

#define MOTION_STEPS            30
#define MOTION_UPDATE_MS         15
#define STAGE_PAUSE_MS          300

#define TRANSIT_SHOULDER_US    2000
#define TRANSIT_ELBOW_US       1800
#define TRANSIT_WRIST_US        700

/* Tested HOME correction when approaching from ITEM_SCAN. */
#define HOME_BASE_FROM_ITEM_US 1550

static const ArmPose home_pose = {
    .base = 1600,
    .shoulder = 2000,
    .elbow = 1800,
    .wrist_pitch = 700,
    .wrist_rotate = WRIST_ROTATE_FIXED_US,
    .gripper = GRIPPER_OPEN_US,
};

static const ArmPose shelf_scan_pose = {
    .base = 400,
    .shoulder = 1850,
    .elbow = 1900,
    .wrist_pitch = 1600,
    .wrist_rotate = WRIST_ROTATE_FIXED_US,
    .gripper = GRIPPER_OPEN_US,
};

static const ArmPose item_scan_pose = {
    /* Revalidated by individual-motor testing. */
    .base = 2900,
    .shoulder = 1700,
    .elbow = 1500,
    .wrist_pitch = 800,
    .wrist_rotate = WRIST_ROTATE_FIXED_US,
    .gripper = GRIPPER_OPEN_US,
};

static ID i2c_device_id;
static ID i2c_mutex_id;
static ID command_mbf_id;
static ID report_mbf_id;
static ArmPose current_pose;
/* Last successfully completed named pose; not measured joint feedback. */
static INT last_pose_id = -1;

static INT clamp_value(INT value, INT minimum, INT maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static ArmPose clamp_pose(ArmPose pose)
{
    pose.base = clamp_value(pose.base, BASE_MIN_US, BASE_MAX_US);
    pose.shoulder = clamp_value(pose.shoulder, SHOULDER_MIN_US, SHOULDER_MAX_US);
    pose.elbow = clamp_value(pose.elbow, ELBOW_MIN_US, ELBOW_MAX_US);
    pose.wrist_pitch = clamp_value(
        pose.wrist_pitch,
        WRIST_PITCH_MIN_US,
        WRIST_PITCH_MAX_US
    );
    pose.wrist_rotate = WRIST_ROTATE_FIXED_US;
    pose.gripper = clamp_value(pose.gripper, GRIPPER_MIN_US, GRIPPER_MAX_US);
    return pose;
}

/*
 * This is the only PCA9685 bus-write path. The shared mutex is held only
 * while tk_swri_dev performs the I2C transaction. It is always released
 * before any trajectory or settling delay, allowing a future IMU task to use
 * the bus between servo updates.
 */
static ER pca_write_bytes(const UB *bytes, SZ size)
{
    SZ actual_size = 0;
    ER result;

    result = tk_loc_mtx(i2c_mutex_id, TMO_FEVR);
    if (result < E_OK) return result;

    result = tk_swri_dev(
        i2c_device_id,
        PCA9685_ADDRESS,
        bytes,
        size,
        &actual_size
    );

    {
        ER unlock_result = tk_unl_mtx(i2c_mutex_id);
        if (result >= E_OK && unlock_result < E_OK) result = unlock_result;
    }

    if (result >= E_OK && actual_size != size) return E_IO;
    return result;
}

static ER pca_write_register(UB reg, UB value)
{
    const UB data[2] = {reg, value};
    return pca_write_bytes(data, sizeof(data));
}

static ER pca_init(void)
{
    ER result;

    result = pca_write_register(PCA9685_MODE1, 0x10);
    if (result < E_OK) return result;
    result = pca_write_register(PCA9685_PRESCALE, 121);
    if (result < E_OK) return result;
    result = pca_write_register(PCA9685_MODE2, 0x04);
    if (result < E_OK) return result;
    result = pca_write_register(PCA9685_MODE1, 0x20);
    if (result < E_OK) return result;

    tk_dly_tsk(10);
    return E_OK;
}

static ER servo_set_us(UB channel, UH pulse_us)
{
    UH count = (UH)(((UW)pulse_us * 4096U) / 20000U);
    UB reg = (UB)(PCA9685_LED0_ON_L + (4U * channel));
    const UB data[5] = {
        reg,
        0x00,
        0x00,
        (UB)(count & 0xffU),
        (UB)((count >> 8) & 0x0fU),
    };
    return pca_write_bytes(data, sizeof(data));
}

static ER move_servo_smooth(UB channel, INT start_us, INT target_us, INT steps)
{
    INT step;

    for (step = 1; step <= steps; ++step) {
        INT pulse = start_us + ((target_us - start_us) * step) / steps;
        ER result = servo_set_us(channel, (UH)pulse);
        if (result < E_OK) return result;
        tk_dly_tsk(MOTION_UPDATE_MS);
    }
    return E_OK;
}

static ER move_shoulder_elbow(
    INT shoulder_start,
    INT shoulder_target,
    INT elbow_start,
    INT elbow_target
)
{
    INT step;

    shoulder_target = clamp_value(
        shoulder_target,
        SHOULDER_MIN_US,
        SHOULDER_MAX_US
    );
    elbow_target = clamp_value(elbow_target, ELBOW_MIN_US, ELBOW_MAX_US);

    for (step = 1; step <= MOTION_STEPS; ++step) {
        INT shoulder = shoulder_start
            + ((shoulder_target - shoulder_start) * step) / MOTION_STEPS;
        INT elbow = elbow_start
            + ((elbow_target - elbow_start) * step) / MOTION_STEPS;
        ER result;

        result = servo_set_us(SHOULDER_CHANNEL, (UH)shoulder);
        if (result < E_OK) return result;
        result = servo_set_us(ELBOW_CHANNEL, (UH)elbow);
        if (result < E_OK) return result;
        tk_dly_tsk(MOTION_UPDATE_MS);
    }
    return E_OK;
}

static ER initialize_home(void)
{
    ER result;

    /* Deliberately staged: no six-joint simultaneous startup command. */
    result = servo_set_us(WRIST_ROTATE_CHANNEL, WRIST_ROTATE_FIXED_US);
    if (result < E_OK) return result;
    result = servo_set_us(GRIPPER_CHANNEL, GRIPPER_OPEN_US);
    if (result < E_OK) return result;
    tk_dly_tsk(STAGE_PAUSE_MS);

    result = servo_set_us(WRIST_PITCH_CHANNEL, (UH)home_pose.wrist_pitch);
    if (result < E_OK) return result;
    tk_dly_tsk(STAGE_PAUSE_MS);

    result = servo_set_us(SHOULDER_CHANNEL, (UH)home_pose.shoulder);
    if (result < E_OK) return result;
    result = servo_set_us(ELBOW_CHANNEL, (UH)home_pose.elbow);
    if (result < E_OK) return result;
    tk_dly_tsk(STAGE_PAUSE_MS);

    result = servo_set_us(BASE_CHANNEL, (UH)home_pose.base);
    if (result < E_OK) return result;
    tk_dly_tsk(STAGE_PAUSE_MS);

    current_pose = home_pose;
    last_pose_id = ARM_POSE_HOME;
    return E_OK;
}

static ER move_to_pose(ArmPose target)
{
    ER result;

    target = clamp_pose(target);

    result = move_servo_smooth(
        WRIST_PITCH_CHANNEL,
        current_pose.wrist_pitch,
        TRANSIT_WRIST_US,
        MOTION_STEPS
    );
    if (result < E_OK) return result;
    current_pose.wrist_pitch = TRANSIT_WRIST_US;
    tk_dly_tsk(STAGE_PAUSE_MS);

    result = move_shoulder_elbow(
        current_pose.shoulder,
        TRANSIT_SHOULDER_US,
        current_pose.elbow,
        TRANSIT_ELBOW_US
    );
    if (result < E_OK) return result;
    current_pose.shoulder = TRANSIT_SHOULDER_US;
    current_pose.elbow = TRANSIT_ELBOW_US;
    tk_dly_tsk(STAGE_PAUSE_MS);

    result = move_servo_smooth(
        BASE_CHANNEL,
        current_pose.base,
        target.base,
        MOTION_STEPS
    );
    if (result < E_OK) return result;
    current_pose.base = target.base;
    tk_dly_tsk(STAGE_PAUSE_MS);

    result = move_shoulder_elbow(
        current_pose.shoulder,
        target.shoulder,
        current_pose.elbow,
        target.elbow
    );
    if (result < E_OK) return result;
    current_pose.shoulder = target.shoulder;
    current_pose.elbow = target.elbow;
    tk_dly_tsk(STAGE_PAUSE_MS);

    result = move_servo_smooth(
        WRIST_PITCH_CHANNEL,
        current_pose.wrist_pitch,
        target.wrist_pitch,
        MOTION_STEPS
    );
    if (result < E_OK) return result;
    current_pose.wrist_pitch = target.wrist_pitch;

    result = servo_set_us(WRIST_ROTATE_CHANNEL, WRIST_ROTATE_FIXED_US);
    if (result < E_OK) return result;
    current_pose.wrist_rotate = WRIST_ROTATE_FIXED_US;

    /* Gripper state is intentionally not changed by a pose command. */
    return E_OK;
}

static const ArmPose *pose_from_id(ArmPoseId pose)
{
    switch (pose) {
    case ARM_POSE_HOME:
        return &home_pose;
    case ARM_POSE_SHELF_SCAN:
        return &shelf_scan_pose;
    case ARM_POSE_ITEM_SCAN:
        return &item_scan_pose;
    default:
        return NULL;
    }
}

static ER execute_command(const MotionCommand *command)
{
    const ArmPose *target;
    ArmPose adjusted_target;
    ER result;

    switch (command->type) {
    case MOTION_COMMAND_INITIALIZE_HOME:
        return initialize_home();
    case MOTION_COMMAND_MOVE_POSE:
        target = pose_from_id(command->pose);
        if (target == NULL) return E_PAR;

        adjusted_target = *target;
        if (command->pose == ARM_POSE_HOME) {
            if (last_pose_id == ARM_POSE_ITEM_SCAN) {
                adjusted_target.base = HOME_BASE_FROM_ITEM_US;
            } else if (last_pose_id == ARM_POSE_HOME) {
                /* Repeating HOME must retain the last HOME correction. */
                adjusted_target.base = current_pose.base;
            }
        }

        result = move_to_pose(adjusted_target);
        if (result >= E_OK) last_pose_id = command->pose;
        return result;
    case MOTION_COMMAND_SET_GRIPPER:
        current_pose.gripper = clamp_value(
            command->gripper_us,
            GRIPPER_MIN_US,
            GRIPPER_MAX_US
        );
        return servo_set_us(GRIPPER_CHANNEL, (UH)current_pose.gripper);
    default:
        return E_PAR;
    }
}

void motion_configure(ID i2c_device, ID i2c_mutex, ID command_mbf, ID report_mbf)
{
    i2c_device_id = i2c_device;
    i2c_mutex_id = i2c_mutex;
    command_mbf_id = command_mbf;
    report_mbf_id = report_mbf;
}

void motion_task(INT start_code, void *extended_info)
{
    MotionCommand command;
    MotionReport report;
    ER pca_status;

    (void)start_code;
    (void)extended_info;

    pca_status = pca_init();
    if (pca_status < E_OK) {
        tm_printf((UB *)"[motion] PCA9685 initialization failed: %d\n", pca_status);
    }

    while (1) {
        INT received_size = tk_rcv_mbf(command_mbf_id, &command, TMO_FEVR);

        if (received_size != (INT)sizeof(command)) {
            report.sequence = 0;
            report.result = (received_size < E_OK) ? (ER)received_size : E_IO;
        } else if (pca_status < E_OK) {
            report.sequence = command.sequence;
            report.result = pca_status;
        } else {
            report.sequence = command.sequence;
            report.result = execute_command(&command);
        }

        if (tk_snd_mbf(
                report_mbf_id,
                &report,
                (INT)sizeof(report),
                TMO_FEVR
            ) < E_OK) {
            tm_printf((UB *)"[motion] unable to report command completion\n");
        }
    }
}
