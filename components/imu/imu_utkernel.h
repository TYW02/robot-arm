
#ifndef IMU_UTKERNEL_H
#define IMU_UTKERNEL_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef enum {
    IMU_CMD_VERIFY = 1,
    IMU_CMD_CANCEL,
    IMU_CMD_REINIT
} imu_command_type_t;

typedef struct {
    uint32_t request_id;
    uint32_t command;
    float target_pitch_deg;
    float target_roll_deg;
} imu_command_t;

typedef enum {
    IMU_EVT_READY = 1,
    IMU_EVT_VERIFIED,
    IMU_EVT_OUT_OF_TOLERANCE,
    IMU_EVT_FAULT
} imu_event_type_t;

typedef struct {
    uint32_t request_id;
    uint32_t event;
    float pitch_deg;
    float roll_deg;
    uint64_t sample_time_us;
    uint32_t interval_us;
} imu_result_t;

typedef struct {
    ID shared_i2c_mutex;
    ID command_mbf;
    ID result_mbf;
    PRI priority;
    SZ stack_bytes;
} imu_task_config_t;

// Start IMU task from an already-running kernel task.
ID imu_utkernel_start(
    const imu_task_config_t *config
);

#endif
