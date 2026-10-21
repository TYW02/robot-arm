#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Joint commands are calibrated pulse widths in microseconds, not degrees. */
typedef struct {
    uint16_t base;
    uint16_t shoulder;
    uint16_t elbow;
    uint16_t wrist_pitch;
    uint16_t wrist_rotate;
    uint16_t gripper;
} ArmPose;

typedef enum {
    MOTION_OK = 0,
    MOTION_INVALID_POSE = -1,
    MOTION_NOT_READY = -2,
    MOTION_IO_ERROR = -3,
    MOTION_IPC_ERROR = -4,
    MOTION_WRONG_CONTEXT = -5,
} MotionResult;

extern const ArmPose HOME;
extern const ArmPose SHELF_SCAN;
extern const ArmPose ITEMS_SCAN;

/*
 * Blocking task-context API. Requests execute on the existing Motion task;
 * callers on either core are serialized until their completion/fault report.
 * Do not call from an ISR, the Motion task itself, or while holding the I2C
 * mutex. Startup must first initialize the commanded pose.
 *
 * MovJ rejects NULL/out-of-range targets without commanding any joint. It
 * copies the target, uses the tested staged trajectory, then moves the
 * gripper separately only if its requested pulse changed. No clamping, IK,
 * degree conversion or measured position/orientation verification is done.
 * MOTION_OK means the commanded trajectory and settling delays completed.
 */
MotionResult MovJ(const ArmPose *target);

/*
 * Named helpers preserve the currently commanded gripper pulse. MoveHome
 * retains the tested 1550 us return-from-ITEMS_SCAN correction; literal
 * MovJ(&HOME) always requests HOME's 1600 us base pulse.
 */
MotionResult MoveHome(void);
MotionResult MoveShelfScan(void);
MotionResult MoveItemsScan(void);

/* Move only the gripper, using the existing software-safe open/close pulses. */
MotionResult GripperOpen(void);
MotionResult GripperClose(void);

#ifdef __cplusplus
}
#endif

#endif
