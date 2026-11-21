/** @file imu_heading.c
 *  @brief Calibrate the MPU gyroscope and publish relative yaw on the Pico.
 */

/* Use the standard size_t alongside the kernel headers. */
#define PROHIBIT_DEF_SIZE_T

#include <stddef.h>
#include "imu_heading.h"
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/sysdef.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include "../../third_party/mtk3smp-rp2040/device/include/dev_i2c.h"

typedef float float32_t;

_Static_assert((sizeof(float32_t) == 4U) && (FLT_RADIX == 2) &&
               (FLT_MANT_DIG == 24), "The gyro requires binary32 float.");

#define MPU_I2C_UNIT             (1U)
#define MPU_I2C_ADDRESS          (0x68U)

/* See TDK register maps RM-MPU-6500A-00 and RM-MPU-9250A-00. */
#define MPU_REG_SAMPLE_DIVIDER   (0x19U)
#define MPU_REG_CONFIG           (0x1AU)
#define MPU_REG_GYRO_CONFIG      (0x1BU)
#define MPU_REG_GYRO_X_HIGH      (0x43U)
#define MPU_REG_POWER_MGMT_ONE   (0x6BU)
#define MPU_REG_POWER_MGMT_TWO   (0x6CU)
#define MPU_REG_WHO_AM_I         (0x75U)
#define MPU_ID_6500              (0x70U)
#define MPU_ID_9250              (0x71U)
#define MPU_POWER_RESET          (0x80U)
#define MPU_POWER_CLOCK_AUTO     (0x01U)
#define MPU_ENABLE_ALL_AXES      (0x00U)
#define MPU_GYRO_FILTER          (0x03U)
#define MPU_GYRO_DIVIDER         (9U)
#define MPU_GYRO_RANGE_250_DPS   (0x00U)
#define MPU_CONFIG_WAIT_MS       (100U)
#define GYRO_AXIS_COUNT          (3U)
#define GYRO_Z_AXIS              (2U)
#define GYRO_BYTES_PER_AXIS      (2U)
#define GYRO_SIGN_BIT            (32768U)
#define GYRO_SIGNED_MODULUS      (65536)

#define SAMPLE_FLAG              (1U)
#define SAMPLE_PERIOD_MS         (10U)
#define SAMPLE_MIN_INTERVAL_US   (5000U)
#define SAMPLE_LATE_INTERVAL_US  (15000U)
#define SAMPLE_STALE_US          (50000U)
#define SAMPLE_RATE_WINDOW_US    (1000000U)
#define CALIBRATION_WAIT_MS      (3000U)
#define CALIBRATION_SAMPLE_COUNT (200U)
#define HEADING_TASK_PRIORITY    (4)
#define HEADING_TASK_STACK_BYTES (4096U)

static float32_t const g_counts_per_dps = 131.0f;
static float32_t const g_gyro_limit_dps = 245.0f;
static float32_t const g_variance_limit = 1.0f;
static float32_t const g_seconds_per_us = 0.000001f;

static ID           g_h_mpu_device  = 0;
static ID           g_h_sample_cycle = 0;
static ID           g_h_heading_task = 0;
static ID volatile  g_h_sample_flag  = 0;

static T_SPLOCK g_reading_lock;
static imu_heading_reading_t volatile g_published = {0};

static uint32_t heading_time_us (void);
static void publish_reading (imu_heading_reading_t const * p_reading);
static bool read_registers (uint8_t reg, uint8_t * p_data, SZ length);
static bool write_register (uint8_t reg, uint8_t value);
static ER initialise_gyro (void);
static bool read_gyro (float32_t * p_rates);
static ER calibrate_gyro (float32_t * p_z_bias);
static void sample_tick (void * p_extension);
static ER start_sample_cycle (void);
static void publish_fault (imu_heading_reading_t * p_reading, ER error);
static ER sample_gyro (imu_heading_reading_t * p_reading, float32_t z_bias);
static void imu_heading_task (INT start_code, void * p_extension);

ER imu_heading_start (void)
{
    if (g_h_heading_task > E_OK)
    {
        return E_OBJ;
    }

    (void)InitSpinLock(&g_reading_lock);

    T_CFLG flag = {0};
    flag.flgatr = TA_TFIFO;

    g_h_sample_flag = tk_cre_flg(&flag);

    if (g_h_sample_flag <= E_OK)
    {
        return ((g_h_sample_flag < E_OK) ? g_h_sample_flag : E_IO);
    }

    /* FP is the kernel's generic callback type. Entry takes INT and void *. */
    T_CTSK task =
    {
        .tskatr  = TA_HLNG | TA_RNG3,
        .task    = (FP)imu_heading_task,
        .itskpri = HEADING_TASK_PRIORITY,
        .stksz   = HEADING_TASK_STACK_BYTES
    };

#if TK_SUPPORT_SMP
    task.tskatr |= TA_ASSPRC;
    task.assprc  = TP_PRC1;
#endif

    g_h_heading_task = tk_cre_tsk(&task);

    if (g_h_heading_task <= E_OK)
    {
        (void)tk_del_flg(g_h_sample_flag);
        return ((g_h_heading_task < E_OK) ? g_h_heading_task : E_IO);
    }

    ER error = tk_sta_tsk(g_h_heading_task, 0);

    if (error < E_OK)
    {
        (void)tk_del_tsk(g_h_heading_task);
        g_h_heading_task = 0;
        (void)tk_del_flg(g_h_sample_flag);
    }

    return error;
}

void imu_heading_read (imu_heading_reading_t * p_reading)
{
    if (p_reading == NULL)
    {
        return;
    }

    UINT interrupt_state = 0U;
    (void)ISpinLock(&g_reading_lock, &interrupt_state);
    *p_reading = g_published;
    (void)ISpinUnlock(&g_reading_lock, interrupt_state);

    /* Both timestamps are unsigned 32-bit on RP2040. Subtraction wraps. */
    uint32_t age_us = heading_time_us() - p_reading->sample_us;

    if ((!p_reading->ready) || (age_us > SAMPLE_STALE_US))
    {
        p_reading->valid = FALSE;
    }
}

/** Read the low word of the microsecond clock for modular time arithmetic. */
static uint32_t heading_time_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

/** Publish the complete snapshot under the same lock used by readers. */
static void publish_reading (imu_heading_reading_t const * p_reading)
{
    UINT interrupt_state = 0U;
    (void)ISpinLock(&g_reading_lock, &interrupt_state);
    g_published = *p_reading;
    (void)ISpinUnlock(&g_reading_lock, interrupt_state);
}

/** Write the register address and read its data in one I2C transaction. */
static bool read_registers (uint8_t reg, uint8_t * p_data, SZ length)
{
    T_I2C_EXEC request =
    {
        .sadr     = MPU_I2C_ADDRESS,
        .snd_size = 1,
        .snd_data = &reg,
        .rcv_size = length,
        .rcv_data = p_data
    };

    /* The descriptor size fits in the kernel's signed size type. */
    SZ const request_size = (SZ)sizeof(request);
    SZ       transferred  = 0;
    ER       error        = tk_swri_dev(g_h_mpu_device, TDN_I2C_EXEC,
                                       &request, request_size,
                                       &transferred);

    return ((error == E_OK) && (transferred == request_size));
}

/** Write one MPU register and verify the transfer completed. */
static bool write_register (uint8_t reg, uint8_t value)
{
    uint8_t data[] = {reg, value};
    /* The two-byte payload fits in the kernel's signed size type. */
    SZ const payload_size = (SZ)sizeof(data);
    SZ       transferred  = 0;
    ER       error        = tk_swri_dev(g_h_mpu_device, MPU_I2C_ADDRESS,
                                       data, payload_size, &transferred);

    return ((error == E_OK) && (transferred == payload_size));
}

/** Check the sensor identity and configure the 100 Hz gyro. */
static ER initialise_gyro (void)
{
    /* The RP2040 I2C driver configures GP2 and GP3 for I2C1. */
    ER error = dev_init_i2c(MPU_I2C_UNIT);

    if (error < E_OK)
    {
        return error;
    }

    /* The kernel uses UB for bytes in device names and log strings. */
    g_h_mpu_device = tk_opn_dev((UB const *)"iicb", TD_READ | TD_WRITE);

    if (g_h_mpu_device <= E_OK)
    {
        return ((g_h_mpu_device < E_OK) ? g_h_mpu_device : E_IO);
    }

    uint8_t device_id = 0U;

    if (!read_registers(MPU_REG_WHO_AM_I, &device_id, sizeof(device_id)))
    {
        /* Kernel log messages use UB byte strings. */
        tm_printf((UB const *)"[MPU] WHO_AM_I read failed\n");
        return E_IO;
    }

    /* UINT can represent the full one-byte device ID for the %X format. */
    tm_printf((UB const *)"[MPU] WHO_AM_I=0x%02X\n", (UINT)device_id);

    if ((device_id != MPU_ID_6500) && (device_id != MPU_ID_9250))
    {
        /* Kernel log messages use UB byte strings. */
        tm_printf((UB const *)"[MPU] Unsupported device ID\n");
        return E_IO;
    }

    if (!write_register(MPU_REG_POWER_MGMT_ONE, MPU_POWER_RESET))
    {
        return E_IO;
    }

    (void)tk_dly_tsk(MPU_CONFIG_WAIT_MS);

    if (!write_register(MPU_REG_POWER_MGMT_ONE, MPU_POWER_CLOCK_AUTO))
    {
        return E_IO;
    }

    (void)tk_dly_tsk(MPU_CONFIG_WAIT_MS);

    if ((!write_register(MPU_REG_POWER_MGMT_TWO, MPU_ENABLE_ALL_AXES)) ||
        (!write_register(MPU_REG_CONFIG, MPU_GYRO_FILTER)) ||
        (!write_register(MPU_REG_SAMPLE_DIVIDER, MPU_GYRO_DIVIDER)) ||
        (!write_register(MPU_REG_GYRO_CONFIG, MPU_GYRO_RANGE_250_DPS)))
    {
        /* Kernel log messages use UB byte strings. */
        tm_printf((UB const *)"[MPU] Gyro configuration failed\n");
        return E_IO;
    }

    (void)tk_dly_tsk(MPU_CONFIG_WAIT_MS);
    return E_OK;
}

/** Decode the three big-endian signed gyro registers and check their range. */
static bool read_gyro (float32_t * p_rates)
{
    uint8_t data[GYRO_AXIS_COUNT * GYRO_BYTES_PER_AXIS] = {0U};

    if (!read_registers(MPU_REG_GYRO_X_HIGH, data, sizeof(data)))
    {
        return false;
    }

    for (uint32_t axis = 0U; axis < GYRO_AXIS_COUNT; ++axis)
    {
        uint32_t offset = axis * GYRO_BYTES_PER_AXIS;

        /* Widen bytes before shifting. The combined word fits in int32_t. */
        uint32_t word = ((uint32_t)data[offset] << 8U) | data[offset + 1U];
        int32_t counts = (int32_t)word;

        if (word >= GYRO_SIGN_BIT)
        {
            counts -= GYRO_SIGNED_MODULUS;
        }

        /* A signed 16-bit sample is exactly representable in binary32. */
        p_rates[axis] = (float32_t)counts / g_counts_per_dps;

        if ((!isfinite(p_rates[axis])) ||
            (p_rates[axis] < -g_gyro_limit_dps) ||
            (p_rates[axis] > g_gyro_limit_dps))
        {
            return false;
        }
    }

    return true;
}

/** Reject movement on any axis and retain the stationary Z-axis mean. */
static ER calibrate_gyro (float32_t * p_z_bias)
{
    float32_t sum[GYRO_AXIS_COUNT]         = {0.0f};
    float32_t sum_squared[GYRO_AXIS_COUNT] = {0.0f};

    /* Kernel log messages use UB byte strings. */
    tm_printf((UB const *)
              "[IMU] Keep still; gyro calibration starts in 3 seconds\n");
    (void)tk_dly_tsk(CALIBRATION_WAIT_MS);

    for (uint32_t sample = 0U; sample < CALIBRATION_SAMPLE_COUNT; ++sample)
    {
        float32_t rates[GYRO_AXIS_COUNT] = {0.0f};

        if (!read_gyro(rates))
        {
            return E_IO;
        }

        for (uint32_t axis = 0U; axis < GYRO_AXIS_COUNT; ++axis)
        {
            sum[axis]         += rates[axis];
            sum_squared[axis] += rates[axis] * rates[axis];
        }

        (void)tk_dly_tsk(SAMPLE_PERIOD_MS);
    }

    /* The integral sample count is exactly representable in binary32. */
    float32_t const sample_count = (float32_t)CALIBRATION_SAMPLE_COUNT;

    for (uint32_t axis = 0U; axis < GYRO_AXIS_COUNT; ++axis)
    {
        float32_t mean     = sum[axis] / sample_count;
        float32_t variance = (sum_squared[axis] / sample_count) - (mean * mean);

        if ((!isfinite(variance)) || (variance > g_variance_limit))
        {
            /* The axis index is 0 through 2 and fits the %d argument. */
            tm_printf((UB const *)
                      "[IMU] Gyro calibration too noisy on axis %d\n",
                      (INT)axis);
            return E_IO;
        }
    }

    *p_z_bias = sum[GYRO_Z_AXIS] / sample_count;
    return E_OK;
}

/** Wake the sampling task. All I2C traffic runs in task context. */
static void sample_tick (void * p_extension)
{
    (void)p_extension;
    (void)tk_set_flg(g_h_sample_flag, SAMPLE_FLAG);
}

/** Schedule samples on the same processor as the heading task. */
static ER start_sample_cycle (void)
{
    /* FP is the kernel's generic callback type. Entry takes void *. */
    T_CCYC cycle =
    {
        .cycatr = TA_HLNG | TA_STA,
        .cychdr = (FP)sample_tick,
        .cyctim = SAMPLE_PERIOD_MS,
        .cycphs = SAMPLE_PERIOD_MS
    };

#if TK_SUPPORT_SMP
    cycle.cycatr |= TA_ASSPRC;
    cycle.assprc  = TP_PRC1;
#endif

    g_h_sample_cycle = tk_cre_cyc(&cycle);

    if (g_h_sample_cycle <= E_OK)
    {
        return ((g_h_sample_cycle < E_OK) ? g_h_sample_cycle : E_IO);
    }

    return E_OK;
}

/** Stop sampling and tell the mission why the heading is unavailable. */
static void publish_fault (imu_heading_reading_t * p_reading, ER error)
{
    if (g_h_sample_cycle > E_OK)
    {
        (void)tk_stp_cyc(g_h_sample_cycle);
    }

    p_reading->ready = FALSE;
    p_reading->valid = FALSE;
    p_reading->fault = error;
    publish_reading(p_reading);

    /* Kernel log messages use UB byte strings. */
    tm_printf((UB const *)"[IMU] Fault %d; reset to retry\n", error);
}

/** Sample on each tick and integrate Z rate using the hardware timer. */
static ER sample_gyro (imu_heading_reading_t * p_reading, float32_t z_bias)
{
    uint32_t previous_us     = 0U;
    uint32_t rate_window_us  = heading_time_us();
    uint32_t sample_count    = 0U;
    bool     b_have_previous = false;

    for (;;)
    {
        UINT flags = 0U;
        ER error = tk_wai_flg(g_h_sample_flag, SAMPLE_FLAG,
                              TWF_ORW | TWF_BITCLR, &flags, TMO_FEVR);

        if (error < E_OK)
        {
            return error;
        }

        uint32_t now_us = heading_time_us();

        if (b_have_previous &&
            ((now_us - previous_us) < SAMPLE_MIN_INTERVAL_US))
        {
            continue;
        }

        float32_t rates[GYRO_AXIS_COUNT] = {0.0f};

        if (!read_gyro(rates))
        {
            return E_IO;
        }

        now_us = heading_time_us();
        uint32_t elapsed_us = now_us - previous_us;

        if (b_have_previous && (elapsed_us > SAMPLE_STALE_US))
        {
            return E_TMOUT;
        }

        p_reading->gyro_z_dps = rates[GYRO_Z_AXIS] - z_bias;

        if (b_have_previous)
        {
            if (elapsed_us > SAMPLE_LATE_INTERVAL_US)
            {
                ++p_reading->late;
            }

            /* The stale check bounds this conversion to 50,000 us. */
            float32_t elapsed_seconds =
                (float32_t)elapsed_us * g_seconds_per_us;
            p_reading->relative_deg += p_reading->gyro_z_dps * elapsed_seconds;
        }

        if ((!isfinite(p_reading->gyro_z_dps)) ||
            (!isfinite(p_reading->relative_deg)))
        {
            return E_IO;
        }

        previous_us     = now_us;
        b_have_previous = true;
        ++sample_count;

        uint32_t window_us = now_us - rate_window_us;

        if (window_us >= SAMPLE_RATE_WINDOW_US)
        {
            /* Convert timer ticks and count to tenths of a hertz. */
            float32_t seconds = (float32_t)window_us * g_seconds_per_us;
            float32_t rate = ((float32_t)sample_count * 10.0f / seconds) + 0.5f;

            if (!isfinite(rate))
            {
                return E_IO;
            }

            /* Sample spacing limits the rate to under 2,001 tenths of Hz. */
            p_reading->gyro_hz_x10 = (UW)rate;
            sample_count   = 0U;
            rate_window_us = now_us;
        }

        p_reading->ready     = TRUE;
        p_reading->valid     = TRUE;
        p_reading->sample_us = now_us;
        publish_reading(p_reading);
    }
}

/** Own gyro startup, calibration, sampling, and fault publication. */
static void imu_heading_task (INT start_code, void * p_extension)
{
    (void)start_code;
    (void)p_extension;

    imu_heading_reading_t reading = {0};
    float32_t z_bias = 0.0f;

    /* Kernel log messages use UB byte strings. */
    tm_printf((UB const *)"[IMU] MPU gyro-only yaw\n");
    ER error = initialise_gyro();

    if (error == E_OK)
    {
        error = calibrate_gyro(&z_bias);
    }

    if (error == E_OK)
    {
        error = start_sample_cycle();
    }

    if (error == E_OK)
    {
        /* Kernel log messages use UB byte strings. */
        tm_printf((UB const *)
                  "[IMU] Calibration complete; relative yaw starts at "
                  "0 degrees\n");
        error = sample_gyro(&reading, z_bias);
    }

    publish_fault(&reading, error);
    tk_ext_tsk();
}

/* End of file. */

