#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <tk/tkernel.h>

/* Physical PCA9685 mapping. */
#define BASE_CHANNEL             0
#define SHOULDER_CHANNEL         1
#define ELBOW_CHANNEL            4
#define GRIPPER_CHANNEL         11
#define WRIST_ROTATE_CHANNEL    13
#define WRIST_PITCH_CHANNEL     15

/* Base endpoints revalidated by testing; other limits retain their margins. */
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

typedef struct {
    INT base;
    INT shoulder;
    INT elbow;
    INT wrist_pitch;
    INT wrist_rotate;
    INT gripper;
} ArmPose;

typedef enum {
    ARM_POSE_HOME = 0,
    ARM_POSE_SHELF_SCAN,
    ARM_POSE_ITEM_SCAN,
} ArmPoseId;

typedef enum {
    MOTION_COMMAND_INITIALIZE_HOME = 0,
    MOTION_COMMAND_MOVE_POSE,
    MOTION_COMMAND_SET_GRIPPER,
} MotionCommandType;

typedef struct {
    UW sequence;
    MotionCommandType type;
    ArmPoseId pose;
    INT gripper_us;
} MotionCommand;

typedef struct {
    UW sequence;
    ER result;
} MotionReport;

void motion_configure(ID i2c_device, ID i2c_mutex, ID command_mbf, ID report_mbf);
void motion_task(INT start_code, void *extended_info);

#endif
