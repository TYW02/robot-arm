/*
 * INF2004 robot-arm application for the micro T-Kernel 3 SMP RP2040 port.
 *
 * Processor 1 is physical RP2040 core 0 and owns peripheral interrupts.
 * The motion task is therefore pinned to processor 1. The coordinator is
 * pinned to processor 2 so sequencing work cannot disturb I2C IRQ ownership.
 * A future IMU task must also use processor 1 and the same I2C mutex below.
 */
#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#include "robot_arm.h"

#define TASK_STACK_SIZE       4096
#define MOTION_QUEUE_DEPTH       4
#define REPORT_QUEUE_DEPTH       4
#define POSE_HOLD_MS          3000

/* Message-buffer storage includes one UW of kernel bookkeeping per message. */
static UW motion_command_storage[
    (MOTION_QUEUE_DEPTH * sizeof(MotionCommand) + sizeof(UW) - 1) / sizeof(UW)
    + MOTION_QUEUE_DEPTH
];
static UW motion_report_storage[
    (REPORT_QUEUE_DEPTH * sizeof(MotionReport) + sizeof(UW) - 1) / sizeof(UW)
    + REPORT_QUEUE_DEPTH
];

static ID i2c_mutex_id;
static ID motion_command_mbf_id;
static ID motion_report_mbf_id;
static ID i2c_device_id;

static BOOL object_created(const char *name, ID id)
{
    if (id <= E_OK) {
        tm_printf((UB *)"[init] %s creation/open failed: %d\n", name, id);
        return FALSE;
    }
    return TRUE;
}

static ER send_motion_command(MotionCommand *command)
{
    MotionReport report;
    INT received_size;
    ER result;

    result = tk_snd_mbf(
        motion_command_mbf_id,
        command,
        (INT)sizeof(*command),
        TMO_FEVR
    );
    if (result < E_OK) {
        return result;
    }

    received_size = tk_rcv_mbf(
        motion_report_mbf_id,
        &report,
        TMO_FEVR
    );
    if (received_size != (INT)sizeof(report)) {
        return (received_size < E_OK) ? (ER)received_size : E_IO;
    }
    if (report.sequence != command->sequence) {
        return E_SYS;
    }

    tm_printf(
        (UB *)"[coordinator] command %u completed with %d\n",
        command->sequence,
        report.result
    );
    return report.result;
}

static ER command_pose(UW sequence, MotionCommandType type, ArmPoseId pose)
{
    MotionCommand command = {
        .sequence = sequence,
        .type = type,
        .pose = pose,
        .gripper_us = GRIPPER_OPEN_US,
    };
    return send_motion_command(&command);
}

static void coordinator_task(INT start_code, void *extended_info)
{
    UW sequence = 0;
    ER result;

    (void)start_code;
    (void)extended_info;

    tm_printf((UB *)"\nINF2004 robot arm - micro T-Kernel 3 SMP\n");
    tm_printf((UB *)"motion/I2C: processor 1; coordinator: processor 2\n");

    result = command_pose(++sequence, MOTION_COMMAND_INITIALIZE_HOME, ARM_POSE_HOME);
    if (result < E_OK) goto fault;
    tk_dly_tsk(POSE_HOLD_MS);

    result = command_pose(++sequence, MOTION_COMMAND_MOVE_POSE, ARM_POSE_SHELF_SCAN);
    if (result < E_OK) goto fault;
    tk_dly_tsk(POSE_HOLD_MS);

    result = command_pose(++sequence, MOTION_COMMAND_MOVE_POSE, ARM_POSE_ITEM_SCAN);
    if (result < E_OK) goto fault;
    tk_dly_tsk(POSE_HOLD_MS);

    result = command_pose(++sequence, MOTION_COMMAND_MOVE_POSE, ARM_POSE_SHELF_SCAN);
    if (result < E_OK) goto fault;
    tk_dly_tsk(POSE_HOLD_MS);

    result = command_pose(++sequence, MOTION_COMMAND_MOVE_POSE, ARM_POSE_HOME);
    if (result < E_OK) goto fault;

    tm_printf((UB *)"[coordinator] scan sequence complete\n");
    tk_slp_tsk(TMO_FEVR);
    return;

fault:
    tm_printf((UB *)"[coordinator] motion fault: %d\n", result);
    tk_slp_tsk(TMO_FEVR);
}

static T_CTSK motion_task_definition = {
    .itskpri = 5,
    .stksz = TASK_STACK_SIZE,
    .task = motion_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

static T_CTSK coordinator_task_definition = {
    .itskpri = 8,
    .stksz = TASK_STACK_SIZE,
    .task = coordinator_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC2,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

static T_CMTX i2c_mutex_definition = {
    .mtxatr = TA_TFIFO | TA_INHERIT,
};

static T_CMBF motion_command_mbf_definition = {
    .mbfatr = TA_TFIFO,
    .bufsz = sizeof(motion_command_storage),
    .maxmsz = sizeof(MotionCommand),
    .bufptr = motion_command_storage,
};

static T_CMBF motion_report_mbf_definition = {
    .mbfatr = TA_TFIFO,
    .bufsz = sizeof(motion_report_storage),
    .maxmsz = sizeof(MotionReport),
    .bufptr = motion_report_storage,
};

EXPORT INT usermain(void)
{
    ID motion_task_id;
    ID coordinator_task_id;

    /* The BSP is configured to register RP2040 I2C1 as "iicb" on GP2/GP3. */
    i2c_device_id = tk_opn_dev((UB *)"iicb", TD_UPDATE | TD_EXCL);
    if (!object_created("I2C1 device", i2c_device_id)) return 1;

    i2c_mutex_id = tk_cre_mtx(&i2c_mutex_definition);
    if (!object_created("I2C mutex", i2c_mutex_id)) return 1;

    motion_command_mbf_id = tk_cre_mbf(&motion_command_mbf_definition);
    if (!object_created("motion command buffer", motion_command_mbf_id)) return 1;

    motion_report_mbf_id = tk_cre_mbf(&motion_report_mbf_definition);
    if (!object_created("motion report buffer", motion_report_mbf_id)) return 1;

    motion_configure(
        i2c_device_id,
        i2c_mutex_id,
        motion_command_mbf_id,
        motion_report_mbf_id
    );

    motion_task_id = tk_cre_tsk(&motion_task_definition);
    if (!object_created("motion task", motion_task_id)) return 1;
    coordinator_task_id = tk_cre_tsk(&coordinator_task_definition);
    if (!object_created("coordinator task", coordinator_task_id)) return 1;

    tk_sta_tsk(motion_task_id, 0);
    tk_sta_tsk(coordinator_task_id, 0);

    /* Returning from usermain shuts the kernel down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
