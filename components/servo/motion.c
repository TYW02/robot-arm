#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>

#include "robot_arm.h"

#define PCA9685_ADDRESS       0x40
#define PCA9685_MODE1         0x00
#define PCA9685_MODE2         0x01
#define PCA9685_LED0_ON_L     0x06
#define PCA9685_PRESCALE      0xFE

/* Physical wiring and calibrated limits are private to Motion Control. */
#define BASE_CHANNEL             0
#define SHOULDER_CHANNEL         1
#define ELBOW_CHANNEL            4
#define GRIPPER_CHANNEL         11
#define WRIST_ROTATE_CHANNEL    13
#define WRIST_PITCH_CHANNEL     15

#define BASE_MIN_US            400
#define BASE_MAX_US           2900
#define SHOULDER_MIN_US       1300
#define SHOULDER_MAX_US       2450
#define ELBOW_MIN_US          1100
#define ELBOW_MAX_US          2500
#define WRIST_PITCH_MIN_US     500
#define WRIST_PITCH_MAX_US    1850
#define WRIST_ROTATE_FIXED_US  400
#define GRIPPER_MIN_US        1450
#define GRIPPER_MAX_US        2700
#define GRIPPER_OPEN_US       1450
#define GRIPPER_CLOSE_US      GRIPPER_MAX_US

/* Preserve the current working 30 x 15 ms interpolation per stage. */
#define MOTION_STEPS            30
#define MOTION_UPDATE_MS         15
#define STAGE_PAUSE_MS          300

#define TRANSIT_SHOULDER_US    2000
#define TRANSIT_ELBOW_US       1800
#define TRANSIT_WRIST_US        700

/* Tested HOME correction when approaching from ITEM_SCAN. */
#define HOME_BASE_FROM_ITEM_US 1550

const ArmPose HOME = {
    .base = 1600,
    .shoulder = 2000,
    .elbow = 1800,
    .wrist_pitch = 700,
    .wrist_rotate = WRIST_ROTATE_FIXED_US,
    .gripper = GRIPPER_OPEN_US,
};

const ArmPose SHELF_SCAN = {
    .base = 400,
    .shoulder = 1850,
    .elbow = 1900,
    .wrist_pitch = 1600,
    .wrist_rotate = WRIST_ROTATE_FIXED_US,
    .gripper = GRIPPER_OPEN_US,
};

const ArmPose ITEMS_SCAN = {
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
static ID api_mutex_id;
static ID worker_task_id;
static UW next_sequence;
static BOOL ipc_fault;
static ArmPose commanded_pose;
static BOOL pose_initialized;
static BOOL motion_fault;
/* Last successfully completed named pose; not measured joint feedback. */
static INT last_pose_id = -1;

static BOOL pose_valid(const ArmPose *pose)
{
    return pose != NULL
        && pose->base >= BASE_MIN_US && pose->base <= BASE_MAX_US
        && pose->shoulder >= SHOULDER_MIN_US && pose->shoulder <= SHOULDER_MAX_US
        && pose->elbow >= ELBOW_MIN_US && pose->elbow <= ELBOW_MAX_US
        && pose->wrist_pitch >= WRIST_PITCH_MIN_US
        && pose->wrist_pitch <= WRIST_PITCH_MAX_US
        && pose->wrist_rotate == WRIST_ROTATE_FIXED_US
        && pose->gripper >= GRIPPER_MIN_US && pose->gripper <= GRIPPER_MAX_US;
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

    return tk_dly_tsk(10);
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

static ER move_servo_smooth(
    UB channel, uint16_t *commanded_us, uint16_t target_us, INT steps
)
{
    INT start_us = *commanded_us;
    INT step;

    for (step = 1; step <= steps; ++step) {
        INT pulse = start_us + ((target_us - start_us) * step) / steps;
        ER result = servo_set_us(channel, (UH)pulse);
        if (result < E_OK) return result;
        *commanded_us = (uint16_t)pulse;
        result = tk_dly_tsk(MOTION_UPDATE_MS);
        if (result < E_OK) return result;
    }
    return E_OK;
}

static ER move_shoulder_elbow(uint16_t shoulder_target, uint16_t elbow_target)
{
    INT shoulder_start = commanded_pose.shoulder;
    INT elbow_start = commanded_pose.elbow;
    INT step;

    for (step = 1; step <= MOTION_STEPS; ++step) {
        INT shoulder = shoulder_start
            + ((shoulder_target - shoulder_start) * step) / MOTION_STEPS;
        INT elbow = elbow_start
            + ((elbow_target - elbow_start) * step) / MOTION_STEPS;
        ER result;

        result = servo_set_us(SHOULDER_CHANNEL, (UH)shoulder);
        if (result < E_OK) return result;
        commanded_pose.shoulder = (uint16_t)shoulder;
        result = servo_set_us(ELBOW_CHANNEL, (UH)elbow);
        if (result < E_OK) return result;
        commanded_pose.elbow = (uint16_t)elbow;
        result = tk_dly_tsk(MOTION_UPDATE_MS);
        if (result < E_OK) return result;
    }
    return E_OK;
}

static ER initialize_home(void)
{
    ER result;

    pose_initialized = FALSE;
    /* Deliberately staged: no six-joint simultaneous startup command. */
    result = servo_set_us(WRIST_ROTATE_CHANNEL, WRIST_ROTATE_FIXED_US);
    if (result < E_OK) return result;
    commanded_pose.wrist_rotate = WRIST_ROTATE_FIXED_US;
    result = servo_set_us(GRIPPER_CHANNEL, GRIPPER_OPEN_US);
    if (result < E_OK) return result;
    commanded_pose.gripper = GRIPPER_OPEN_US;
    result = tk_dly_tsk(STAGE_PAUSE_MS);
    if (result < E_OK) return result;

    result = servo_set_us(WRIST_PITCH_CHANNEL, HOME.wrist_pitch);
    if (result < E_OK) return result;
    commanded_pose.wrist_pitch = HOME.wrist_pitch;
    result = tk_dly_tsk(STAGE_PAUSE_MS);
    if (result < E_OK) return result;

    result = servo_set_us(SHOULDER_CHANNEL, HOME.shoulder);
    if (result < E_OK) return result;
    commanded_pose.shoulder = HOME.shoulder;
    result = servo_set_us(ELBOW_CHANNEL, HOME.elbow);
    if (result < E_OK) return result;
    commanded_pose.elbow = HOME.elbow;
    result = tk_dly_tsk(STAGE_PAUSE_MS);
    if (result < E_OK) return result;

    result = servo_set_us(BASE_CHANNEL, HOME.base);
    if (result < E_OK) return result;
    commanded_pose.base = HOME.base;
    result = tk_dly_tsk(STAGE_PAUSE_MS);
    if (result < E_OK) return result;

    pose_initialized = TRUE;
    motion_fault = FALSE;
    last_pose_id = ARM_POSE_HOME;
    return E_OK;
}

static ER move_gripper(uint16_t target_us)
{
    ER result;
    if (commanded_pose.gripper == target_us) return E_OK;
    result = move_servo_smooth(
        GRIPPER_CHANNEL, &commanded_pose.gripper, target_us, MOTION_STEPS
    );
    if (result < E_OK) return result;
    return tk_dly_tsk(STAGE_PAUSE_MS);
}

/* Same five arm stages as the tested implementation; gripper is last. */
static ER move_to_pose(ArmPose target)
{
    ER result;

    result = move_servo_smooth(
        WRIST_PITCH_CHANNEL,
        &commanded_pose.wrist_pitch,
        TRANSIT_WRIST_US,
        MOTION_STEPS
    );
    if (result < E_OK) return result;
    result = tk_dly_tsk(STAGE_PAUSE_MS);
    if (result < E_OK) return result;

    result = move_shoulder_elbow(
        TRANSIT_SHOULDER_US,
        TRANSIT_ELBOW_US
    );
    if (result < E_OK) return result;
    result = tk_dly_tsk(STAGE_PAUSE_MS);
    if (result < E_OK) return result;

    result = move_servo_smooth(
        BASE_CHANNEL,
        &commanded_pose.base,
        target.base,
        MOTION_STEPS
    );
    if (result < E_OK) return result;
    result = tk_dly_tsk(STAGE_PAUSE_MS);
    if (result < E_OK) return result;

    result = move_shoulder_elbow(
        target.shoulder,
        target.elbow
    );
    if (result < E_OK) return result;
    result = tk_dly_tsk(STAGE_PAUSE_MS);
    if (result < E_OK) return result;

    result = move_servo_smooth(
        WRIST_PITCH_CHANNEL,
        &commanded_pose.wrist_pitch,
        target.wrist_pitch,
        MOTION_STEPS
    );
    if (result < E_OK) return result;
    result = servo_set_us(WRIST_ROTATE_CHANNEL, target.wrist_rotate);
    if (result < E_OK) return result;
    commanded_pose.wrist_rotate = target.wrist_rotate;

    /* Completion includes final settling, independently of coordinator holds. */
    result = tk_dly_tsk(STAGE_PAUSE_MS);
    if (result < E_OK) return result;
    return move_gripper(target.gripper);
}

static const ArmPose *pose_from_id(ArmPoseId pose)
{
    switch (pose) {
    case ARM_POSE_HOME:
        return &HOME;
    case ARM_POSE_SHELF_SCAN:
        return &SHELF_SCAN;
    case ARM_POSE_ITEM_SCAN:
        return &ITEMS_SCAN;
    default:
        return NULL;
    }
}

/* Gripper state is independent of the last completed named arm pose. */
static BOOL same_arm_joints(const ArmPose *a, const ArmPose *b)
{
    return a->base == b->base && a->shoulder == b->shoulder
        && a->elbow == b->elbow && a->wrist_pitch == b->wrist_pitch
        && a->wrist_rotate == b->wrist_rotate;
}

static MotionResult execute_movj(const ArmPose *target)
{
    ER result;
    if (!pose_valid(target)) return MOTION_INVALID_POSE;
    if (!pose_initialized) return MOTION_NOT_READY;
    if (motion_fault) return MOTION_IO_ERROR;

    result = move_to_pose(*target);
    if (result < E_OK) {
        motion_fault = TRUE;
        last_pose_id = -1;
        return MOTION_IO_ERROR;
    }

    if (same_arm_joints(target, &ITEMS_SCAN)) last_pose_id = ARM_POSE_ITEM_SCAN;
    else if (same_arm_joints(target, &SHELF_SCAN)) last_pose_id = ARM_POSE_SHELF_SCAN;
    else if (same_arm_joints(target, &HOME)) last_pose_id = ARM_POSE_HOME;
    else last_pose_id = -1;
    return MOTION_OK;
}

static MotionResult execute_command(const MotionCommand *command)
{
    const ArmPose *target;
    ArmPose adjusted_target;
    ER result;

    switch (command->type) {
    case MOTION_COMMAND_INITIALIZE_HOME:
        result = initialize_home();
        if (result < E_OK) {
            motion_fault = TRUE;
            return MOTION_IO_ERROR;
        }
        return MOTION_OK;
    case MOTION_COMMAND_MOVJ:
        return execute_movj(&command->target);
    case MOTION_COMMAND_MOVE_POSE:
        target = pose_from_id(command->pose);
        if (target == NULL) return MOTION_INVALID_POSE;
        if (!pose_initialized) return MOTION_NOT_READY;

        adjusted_target = *target;
        adjusted_target.gripper = commanded_pose.gripper;
        if (command->pose == ARM_POSE_HOME) {
            if (last_pose_id == ARM_POSE_ITEM_SCAN) {
                adjusted_target.base = HOME_BASE_FROM_ITEM_US;
            } else if (last_pose_id == ARM_POSE_HOME) {
                /* Repeating HOME must retain the last HOME correction. */
                adjusted_target.base = commanded_pose.base;
            }
        }

        {
            MotionResult status = execute_movj(&adjusted_target);
            if (status == MOTION_OK) last_pose_id = command->pose;
            return status;
        }
    case MOTION_COMMAND_SET_GRIPPER:
        if (command->gripper_us < GRIPPER_MIN_US
            || command->gripper_us > GRIPPER_MAX_US) return MOTION_INVALID_POSE;
        if (!pose_initialized) return MOTION_NOT_READY;
        if (motion_fault) return MOTION_IO_ERROR;
        result = move_gripper((uint16_t)command->gripper_us);
        if (result < E_OK) {
            motion_fault = TRUE;
            return MOTION_IO_ERROR;
        }
        return MOTION_OK;
    default:
        return MOTION_INVALID_POSE;
    }
}

/* The existing command/report exchange, now hidden behind the public API. */
static MotionResult submit_command(MotionCommand *command)
{
    MotionReport report;
    MotionResult status = MOTION_IPC_ERROR;
    T_RSYS system_state;
    ER result;
    INT size;

    result = tk_ref_sys(&system_state);
    if (result < E_OK
        || (system_state.sysstat & (TSS_INDP | TSS_DDSP | TSS_DINT)) != 0
        || system_state.runtskid <= 0
        || system_state.runtskid == worker_task_id) return MOTION_WRONG_CONTEXT;
    if (api_mutex_id <= 0 || command_mbf_id <= 0 || report_mbf_id <= 0) {
        return MOTION_NOT_READY;
    }

    result = tk_loc_mtx(api_mutex_id, TMO_FEVR);
    if (result < E_OK) return MOTION_IPC_ERROR;

    /* A lost/mismatched reply must not be mistaken for a later completion. */
    if (ipc_fault) goto done;
    command->sequence = ++next_sequence;
    result = tk_snd_mbf(command_mbf_id, command, (INT)sizeof(*command), TMO_FEVR);
    if (result < E_OK) {
        ipc_fault = TRUE;
        goto done;
    }

    size = tk_rcv_mbf(report_mbf_id, &report, TMO_FEVR);
    if (size != (INT)sizeof(report) || report.sequence != command->sequence) {
        ipc_fault = TRUE;
        goto done;
    }
    status = report.result;

done:
    if (tk_unl_mtx(api_mutex_id) < E_OK) return MOTION_IPC_ERROR;
    return status;
}

MotionResult MovJ(const ArmPose *target)
{
    MotionCommand command = {.type = MOTION_COMMAND_MOVJ};
    if (target == NULL) return MOTION_INVALID_POSE;
    command.target = *target;
    if (!pose_valid(&command.target)) return MOTION_INVALID_POSE;
    return submit_command(&command);
}

static MotionResult submit_named_pose(ArmPoseId pose)
{
    MotionCommand command = {.type = MOTION_COMMAND_MOVE_POSE, .pose = pose};
    return submit_command(&command);
}

MotionResult MoveHome(void) { return submit_named_pose(ARM_POSE_HOME); }
MotionResult MoveShelfScan(void) { return submit_named_pose(ARM_POSE_SHELF_SCAN); }
MotionResult MoveItemsScan(void) { return submit_named_pose(ARM_POSE_ITEM_SCAN); }

static MotionResult submit_gripper(INT pulse_us)
{
    MotionCommand command = {
        .type = MOTION_COMMAND_SET_GRIPPER,
        .gripper_us = pulse_us,
    };
    return submit_command(&command);
}

MotionResult GripperOpen(void) { return submit_gripper(GRIPPER_OPEN_US); }
MotionResult GripperClose(void) { return submit_gripper(GRIPPER_CLOSE_US); }

MotionResult motion_initialize_home(void)
{
    MotionCommand command = {.type = MOTION_COMMAND_INITIALIZE_HOME};
    return submit_command(&command);
}

void motion_configure(
    ID i2c_device, ID i2c_mutex, ID command_mbf, ID report_mbf,
    ID api_mutex, ID worker_task
)
{
    i2c_device_id = i2c_device;
    i2c_mutex_id = i2c_mutex;
    command_mbf_id = command_mbf;
    report_mbf_id = report_mbf;
    api_mutex_id = api_mutex;
    worker_task_id = worker_task;
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
            report.result = MOTION_IPC_ERROR;
        } else if (pca_status < E_OK) {
            report.sequence = command.sequence;
            report.result = MOTION_IO_ERROR;
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
