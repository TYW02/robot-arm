#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <tk/tkernel.h>
#include "motion.h"

/* Internal task/IPC integration. Application callers include motion.h. */

typedef enum {
    ARM_POSE_HOME = 0,
    ARM_POSE_SHELF_SCAN,
    ARM_POSE_ITEM_SCAN,
} ArmPoseId;

typedef enum {
    MOTION_COMMAND_INITIALIZE_HOME = 0,
    MOTION_COMMAND_MOVE_POSE,
    MOTION_COMMAND_SET_GRIPPER,
    MOTION_COMMAND_MOVJ,
} MotionCommandType;

typedef struct {
    UW sequence;
    MotionCommandType type;
    ArmPoseId pose;
    INT gripper_us;
    ArmPose target;
} MotionCommand;

typedef struct {
    UW sequence;
    MotionResult result;
} MotionReport;

void motion_configure(
    ID i2c_device, ID i2c_mutex, ID command_mbf, ID report_mbf,
    ID api_mutex, ID worker_task
);
/* Existing explicit startup operation; not a normal MoveHome trajectory. */
MotionResult motion_initialize_home(void);
void motion_task(INT start_code, void *extended_info);

#endif
