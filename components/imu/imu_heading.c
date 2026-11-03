#include <stddef.h>

#define PROHIBIT_DEF_SIZE_T

#include "imu_heading.h"
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <sys/sysdef.h>
#include "../../third_party/mtk3smp-rp2040/device/include/dev_i2c.h"

#include <stdint.h>

#define MPU_ADDR        0x68U
#define SAMPLE_FLAG     1U
#define SAMPLE_MS       10U
#define GYRO_STALE_US   50000U

static ID mpu_device;
static ID sample_flag;
static ID sample_cycle;
static ID heading_task;

static T_SPLOCK reading_lock;
static imu_heading_reading_t published;
static float gyro_z_bias;

static UW heading_time_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

static void publish(const imu_heading_reading_t *reading)
{
    UINT intsts;

    (void)ISpinLock(&reading_lock, &intsts);
    published = *reading;
    (void)ISpinUnlock(&reading_lock, intsts);
}

void imu_heading_read(imu_heading_reading_t *reading)
{
    UINT intsts;

    if (reading == NULL)
        return;

    (void)ISpinLock(&reading_lock, &intsts);
    *reading = published;
    (void)ISpinUnlock(&reading_lock, intsts);

    if (!reading->ready ||
        (UW)(heading_time_us() - reading->sample_us) > GYRO_STALE_US) {
        reading->valid = FALSE;
    }
}

static BOOL read_regs(UB reg, UB *data, SZ length)
{
    T_I2C_EXEC request = {
        .sadr = MPU_ADDR,
        .snd_size = 1,
        .snd_data = &reg,
        .rcv_size = length,
        .rcv_data = data
    };
    SZ done = 0;

    ER er = tk_swri_dev(
        mpu_device,
        TDN_I2C_EXEC,
        &request,
        sizeof(request),
        &done
    );

    return er == E_OK && done == (SZ)sizeof(request);
}

static BOOL write_reg(UB reg, UB value)
{
    UB data[2] = {reg, value};
    SZ done = 0;

    ER er = tk_swri_dev(
        mpu_device, MPU_ADDR, data, sizeof(data), &done
    );

    return er == E_OK && done == (SZ)sizeof(data);
}

static ER initialise_gyro(void)
{
    /* I2C1 uses GP2 for SDA and GP3 for SCL. */
    ER er = dev_init_i2c(1);

    if (er < E_OK)
        return er;

    mpu_device = tk_opn_dev((const UB *)"iicb", TD_READ | TD_WRITE);

    if (mpu_device <= E_OK)
        return mpu_device < E_OK ? mpu_device : E_IO;

    UB id = 0;

    if (!read_regs(0x75U, &id, 1)) {
        tm_printf((UB *)"[MPU] WHO_AM_I read failed\n");
        return E_IO;
    }

    tm_printf((UB *)"[MPU] WHO_AM_I=0x%02X\n", (UINT)id);

    if (id != 0x70U && id != 0x71U) {
        tm_printf((UB *)"[MPU] Unsupported device ID\n");
        return E_IO;
    }

    if (!write_reg(0x6BU, 0x80U))
        return E_IO;

    (void)tk_dly_tsk(100);

    if (!write_reg(0x6BU, 0x01U))
        return E_IO;

    (void)tk_dly_tsk(100);

    /* Existing gyro settings: 100 Hz, low-pass filter, +/-250 dps. */
    if (!write_reg(0x6CU, 0x00U) ||
        !write_reg(0x1AU, 0x03U) ||
        !write_reg(0x19U, 9U) ||
        !write_reg(0x1BU, 0x00U)) {
        tm_printf((UB *)"[MPU] Gyro configuration failed\n");
        return E_IO;
    }

    (void)tk_dly_tsk(100);
    return E_OK;
}

static BOOL read_gyro(float rates[3])
{
    UB data[6];

    if (!read_regs(0x43U, data, sizeof(data)))
        return FALSE;

    for (INT axis = 0; axis < 3; ++axis) {
        int16_t raw = (int16_t)(
            ((uint16_t)data[2 * axis] << 8) | data[2 * axis + 1]
        );

        rates[axis] = (float)raw / 131.0f;

        if (rates[axis] < -245.0f || rates[axis] > 245.0f)
            return FALSE;
    }

    return TRUE;
}

static ER calibrate_gyro(void)
{
    float sum[3] = {0};
    float sum_squared[3] = {0};

    tm_printf((UB *)"[IMU] Keep still; gyro calibration starts in 3 seconds\n");
    (void)tk_dly_tsk(3000);

    for (INT n = 0; n < 200; ++n) {
        float rates[3];

        if (!read_gyro(rates))
            return E_IO;

        for (INT axis = 0; axis < 3; ++axis) {
            sum[axis] += rates[axis];
            sum_squared[axis] += rates[axis] * rates[axis];
        }

        (void)tk_dly_tsk(SAMPLE_MS);
    }

    for (INT axis = 0; axis < 3; ++axis) {
        float mean = sum[axis] / 200.0f;
        float variance = sum_squared[axis] / 200.0f - mean * mean;

        if (variance > 1.0f) {
            tm_printf((UB *)"[IMU] Gyro calibration too noisy on axis %d\n",
                      axis);
            return E_IO;
        }
    }

    gyro_z_bias = sum[2] / 200.0f;
    return E_OK;
}

static void sample_tick(void *exinf)
{
    (void)exinf;
    (void)tk_set_flg(sample_flag, SAMPLE_FLAG);
}

static void fail(imu_heading_reading_t *reading, ER error)
{
    if (sample_cycle > E_OK)
        (void)tk_stp_cyc(sample_cycle);

    reading->ready = FALSE;
    reading->valid = FALSE;
    reading->fault = error;
    publish(reading);

    tm_printf((UB *)"[IMU] Fault %d; reset to retry\n", error);
}

static void task_imu_heading(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    imu_heading_reading_t reading = {0};

    tm_printf((UB *)"[IMU] MPU gyro-only yaw\n");

    ER er = initialise_gyro();

    if (er == E_OK)
        er = calibrate_gyro();

    if (er < E_OK) {
        fail(&reading, er);
        tk_ext_tsk();
        return;
    }

    T_CCYC cycle = {0};
    cycle.cycatr = TA_HLNG | TA_STA;
    cycle.cychdr = (FP)sample_tick;
    cycle.cyctim = SAMPLE_MS;
    cycle.cycphs = SAMPLE_MS;

#if TK_SUPPORT_SMP
    cycle.cycatr |= TA_ASSPRC;
    cycle.assprc = TP_PRC1;
#endif

    sample_cycle = tk_cre_cyc(&cycle);

    if (sample_cycle <= E_OK) {
        fail(&reading, sample_cycle < E_OK ? sample_cycle : E_IO);
        tk_ext_tsk();
        return;
    }

    tm_printf((UB *)"[IMU] Calibration complete; relative yaw starts at 0 degrees\n");

    UW previous = 0;
    UW window = heading_time_us();
    UW sample_count = 0;
    BOOL have_previous = FALSE;

    for (;;) {
        UINT flags;

        er = tk_wai_flg(
            sample_flag, SAMPLE_FLAG, TWF_ORW | TWF_BITCLR, &flags, TMO_FEVR
        );

        if (er < E_OK)
            break;

        UW now = heading_time_us();

        if (have_previous && (UW)(now - previous) < 5000U)
            continue;

        float rates[3];

        if (!read_gyro(rates)) {
            er = E_IO;
            break;
        }

        now = heading_time_us();
        UW elapsed = now - previous;

        if (have_previous && elapsed > GYRO_STALE_US) {
            er = E_TMOUT;
            break;
        }

        reading.gyro_z_dps = rates[2] - gyro_z_bias;

        if (have_previous) {
            if (elapsed > 15000U)
                ++reading.late;

            reading.relative_deg +=
                reading.gyro_z_dps * ((float)elapsed * 0.000001f);
        }

        previous = now;
        have_previous = TRUE;
        ++sample_count;

        if ((UW)(now - window) >= 1000000U) {
            float seconds = (float)(now - window) * 0.000001f;

            reading.gyro_hz_x10 =
                (UW)((float)sample_count * 10.0f / seconds + 0.5f);

            sample_count = 0;
            window = now;
        }

        reading.ready = TRUE;
        reading.valid = TRUE;
        reading.sample_us = now;
        publish(&reading);
    }

    fail(&reading, er);
    tk_ext_tsk();
}

ER imu_heading_start(void)
{
    if (heading_task > E_OK)
        return E_OBJ;

    (void)InitSpinLock(&reading_lock);

    T_CFLG flag = {0};
    flag.flgatr = TA_TFIFO;

    sample_flag = tk_cre_flg(&flag);

    if (sample_flag <= E_OK)
        return sample_flag < E_OK ? sample_flag : E_IO;

    T_CTSK task = {0};
    task.tskatr = TA_HLNG | TA_RNG3;

#if TK_SUPPORT_SMP
    task.tskatr |= TA_ASSPRC;
    task.assprc = TP_PRC1;
#endif

    task.task = (FP)task_imu_heading;
    task.itskpri = 4;
    task.stksz = 4096;

    heading_task = tk_cre_tsk(&task);

    if (heading_task <= E_OK) {
        (void)tk_del_flg(sample_flag);
        return heading_task < E_OK ? heading_task : E_IO;
    }

    ER er = tk_sta_tsk(heading_task, 0);

    if (er < E_OK) {
        (void)tk_del_tsk(heading_task);
        heading_task = 0;
        (void)tk_del_flg(sample_flag);
    }

    return er;
}