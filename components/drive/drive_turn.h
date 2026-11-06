#ifndef DRIVE_TURN_H
#define DRIVE_TURN_H

#include <tk/tkernel.h>

/* Starting values: verify direction and tune on the assembled car. */
#define TURN_GYRO_RIGHT_SIGN    (-1.0f)
#define TURN_TOLERANCE_DEG      2.0f
#define TURN_SETTLE_DPS         5.0f
#define TURN_SETTLE_MS          200U
#define TURN_TIMEOUT_MS         10000U
#define TURN_STALL_MS           1500U
#define TURN_MIN_SPEED          18
#define TURN_MAX_SPEED          25
#define TURN_SCAN_MIN_SPEED    14
#define TURN_SCAN_MAX_SPEED    18
#define TURN_KP                 0.8f
#define TURN_KD                 0.12f

/*
 * Call only from task_drive.
 * Positive degrees = right; negative = left.
 * Supported range: -180 to +180 degrees.
 */
ER drive_turn_run(
    float degrees,
    BOOL (*allowed)(void),
    float *actual,
    BOOL slow_turn
);

#endif