#include <stddef.h>
/* Use the standard size_t in this translation unit for <math.h>. */
#define PROHIBIT_DEF_SIZE_T

#include "imu_heading.h"
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <sys/sysdef.h>
#include "../../third_party/mtk3smp-rp2040/device/include/dev_i2c.h"
#include <stdint.h>
#include <math.h>

#define MPU_ADDR    0x68
#define MAG_ADDR    0x1E
#define SAMPLE_FLAG 1u

static ID mpu_device;
static ID mag_device;
static ID sample_flag;
static ID sample_cycle;
static ID heading_task;

static T_SPLOCK reading_lock;
static imu_heading_reading_t published;
static float bias[3];

static UW heading_time_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

static void publish(const imu_heading_reading_t *s)
{
    UINT intsts;
    (void)ISpinLock(&reading_lock, &intsts);
    published = *s;
    (void)ISpinUnlock(&reading_lock, intsts);
}

void imu_heading_read(imu_heading_reading_t *s)
{
    UINT intsts;
    (void)ISpinLock(&reading_lock, &intsts);
    *s = published;
    (void)ISpinUnlock(&reading_lock, intsts);

    if (!s->ready || heading_time_us() - s->sample_us > 50000u)
        s->valid = FALSE;
}

static BOOL read_regs(ID device, UW address, UB reg,
                      UB *data, SZ length)
{
    T_I2C_EXEC request = {
        .sadr = address,
        .snd_size = 1,
        .snd_data = &reg,
        .rcv_size = length,
        .rcv_data = data
    };
    SZ done = 0;
    ER er = tk_swri_dev(device, TDN_I2C_EXEC, &request,
                       sizeof(request), &done);
    return er == E_OK && done == (SZ)sizeof(request);
}

static BOOL write_reg(ID device, UW address, UB reg, UB value)
{
    UB data[2] = {reg, value};
    SZ done = 0;
    ER er = tk_swri_dev(device, address, data, sizeof(data), &done);
    return er == E_OK && done == (SZ)sizeof(data);
}

static BOOL read_gyro(float rates[3])
{
    UB data[6];
    if (!read_regs(mpu_device, MPU_ADDR, 0x43, data, 6))
        return FALSE;

    for (INT i = 0; i < 3; ++i) {
        int16_t raw = (int16_t)(
            ((uint16_t)data[2 * i] << 8) | data[2 * i + 1]);
        rates[i] = (float)raw / 131.0f;
    }
    return TRUE;
}

static ER initialise_sensors(void)
{
    /* I2C0 is registered by kernel startup. Register I2C1 here once. */
    ER er = dev_init_i2c(1);
    if (er < E_OK) return er;

    mpu_device = tk_opn_dev((const UB*)"iicb", TD_READ | TD_WRITE);
    if (mpu_device <= E_OK) return mpu_device;

    mag_device = tk_opn_dev((const UB*)"iica", TD_READ | TD_WRITE);
    if (mag_device <= E_OK) return mag_device;

    UB id;
    if (!read_regs(mpu_device, MPU_ADDR, 0x75, &id, 1) ||
        id != 0x70)
        return E_IO;

    if (!read_regs(mag_device, MAG_ADDR, 0x0F, &id, 1) ||
        id != 0x49)
        return E_IO;

    if (!write_reg(mpu_device, MPU_ADDR, 0x6B, 0x80))
        return E_IO;
    tk_dly_tsk(100);

    if (!write_reg(mpu_device, MPU_ADDR, 0x6B, 0x01))
        return E_IO;
    tk_dly_tsk(100);

    if (!write_reg(mpu_device, MPU_ADDR, 0x6C, 0x00) ||
        !write_reg(mpu_device, MPU_ADDR, 0x1A, 0x03) ||
        !write_reg(mpu_device, MPU_ADDR, 0x19, 9) ||
        !write_reg(mpu_device, MPU_ADDR, 0x1B, 0x00) ||
        !write_reg(mag_device, MAG_ADDR, 0x20, 0x08) ||
        !write_reg(mag_device, MAG_ADDR, 0x24, 0x6C) ||
        !write_reg(mag_device, MAG_ADDR, 0x25, 0x00) ||
        !write_reg(mag_device, MAG_ADDR, 0x26, 0x00))
        return E_IO;

    tk_dly_tsk(100);
    return E_OK;
}

static ER calibrate_gyro(void)
{
    float sum[3] = {0};
    float sum_squared[3] = {0};

    tm_printf((UB*)
        "[IMU] keep completely still; calibration starts in 3 seconds\n");
    tk_dly_tsk(3000);

    for (INT n = 0; n < 200; ++n) {
        float rates[3];
        if (!read_gyro(rates)) return E_IO;

        for (INT i = 0; i < 3; ++i) {
            if (fabsf(rates[i]) > 245.0f) return E_IO;
            sum[i] += rates[i];
            sum_squared[i] += rates[i] * rates[i];
        }
        tk_dly_tsk(10);
    }

    for (INT i = 0; i < 3; ++i) {
        bias[i] = sum[i] / 200.0f;
        float variance =
            sum_squared[i] / 200.0f - bias[i] * bias[i];

        if (variance > 1.0f) {
            tm_printf((UB*)
                "[IMU] calibration too noisy on axis %d\n", i);
            return E_IO;
        }
    }
    return E_OK;
}

/* -1=I2C error, 0=no new sample, 1=fresh, 2=fresh with overrun. */
static INT read_mag(int16_t raw[3])
{
    UB status, data[6];

    if (!read_regs(mag_device, MAG_ADDR, 0x07, &status, 1))
        return -1;
    if ((status & 0x08) == 0)
        return 0;
    if (!read_regs(mag_device, MAG_ADDR, 0x88, data, 6))
        return -1;

    for (INT i = 0; i < 3; ++i) {
        raw[i] = (int16_t)((uint16_t)data[2 * i] |
                          ((uint16_t)data[2 * i + 1] << 8));
    }
    return (status & 0x80) ? 2 : 1;
}

static float wrap360(float angle)
{
    angle = fmodf(angle, 360.0f);
    return angle < 0.0f ? angle + 360.0f : angle;
}

static float wrap180(float angle)
{
    return wrap360(angle + 180.0f) - 180.0f;
}

static BOOL magnetic_heading(const int16_t raw[3],
                             float *angle, float *field)
{
    float x = (float)raw[0] - (-2091.9877f);
    float y = (float)raw[1] - (-1432.6338f);

    float cx = 0.00025042909f * x - 0.00000296088f * y;
    float cy = -0.00000296088f * x + 0.00026967533f * y;
    float squared = cx * cx + cy * cy;

    if (!isfinite(squared) || squared < 0.000001f)
        return FALSE;

    *field = sqrtf(squared);
    *angle = wrap360(atan2f(-cy, cx) * 57.2957795f);
    return TRUE;
}

/* Handler only wakes the task. I2C and fusion run in task context. */
static void sample_tick(void *exinf)
{
    (void)exinf;
    (void)tk_set_flg(sample_flag, SAMPLE_FLAG);
}

static void fail(imu_heading_reading_t *s, ER error)
{
    if (sample_cycle > E_OK)
        (void)tk_stp_cyc(sample_cycle);

    s->ready = s->valid = s->mag_valid = FALSE;
    s->fault = error;
    publish(s);

    tm_printf((UB*)
        "[IMU] fault %d; reset before restarting test\n", error);
}

static void task_imu_heading(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    imu_heading_reading_t s = {0};

    ER er = initialise_sensors();
    if (er == E_OK)
        er = calibrate_gyro();

    if (er < E_OK) {
        fail(&s, er);
        tk_ext_tsk();
        return;
    }

    T_CCYC cycle = {0};
    cycle.cycatr = TA_HLNG | TA_STA;
    cycle.cychdr = (FP)sample_tick;
    cycle.cyctim = 10;
    cycle.cycphs = 10;
#if TK_SUPPORT_SMP
    cycle.cycatr |= TA_ASSPRC;
    cycle.assprc = TP_PRC1;
#endif

    sample_cycle = tk_cre_cyc(&cycle);
    if (sample_cycle <= E_OK) {
        fail(&s, sample_cycle);
        tk_ext_tsk();
        return;
    }

    tm_printf((UB*)
        "[IMU] calibration complete; fusion running, keep level\n");

    UW previous = 0;
    UW last_mag = 0;
    UW window = heading_time_us();
    UW gyro_count = 0;
    UW mag_count = 0;
    BOOL have_previous = FALSE;

    for (;;) {
        UINT flags;
        er = tk_wai_flg(sample_flag, SAMPLE_FLAG,
                        TWF_ORW | TWF_BITCLR, &flags, TMO_FEVR);
        if (er < E_OK) break;

        UW now = heading_time_us();
        if (have_previous && now - previous < 5000u)
            continue;

        float rates[3];
        if (!read_gyro(rates)) {
            er = E_IO;
            break;
        }

        for (INT i = 0; i < 3; ++i) {
            if (fabsf(rates[i]) > 245.0f)
                er = E_IO;
        }
        if (er < E_OK) break;

        now = heading_time_us();
        UW elapsed = now - previous;

        if (have_previous && elapsed > 50000u) {
            er = E_TMOUT;
            break;
        }
        if (have_previous && elapsed > 15000u)
            ++s.late;

        s.gyro_z_dps = rates[2] - bias[2];

        if (s.ready && have_previous) {
            float delta =
                s.gyro_z_dps * ((float)elapsed * 0.000001f);
            s.heading_deg = wrap360(s.heading_deg + delta);
            s.relative_deg += delta;
            s.gyro_relative_deg += delta;
        }

        previous = now;
        have_previous = TRUE;
        ++gyro_count;

        int16_t raw[3];
        INT result = read_mag(raw);

        if (result < 0) {
            ++s.mag_errors;
        } else if (result > 0) {
            ++mag_count;
            if (result == 2)
                ++s.overruns;

            BOOL usable =
                magnetic_heading(raw, &s.magnetic_deg, &s.field_xy);
            usable = usable &&
                     s.field_xy >= 0.75f && s.field_xy <= 1.25f;

            float error = s.ready
                ? wrap180(s.magnetic_deg - s.heading_deg) : 0;

            if (usable && (!s.ready || fabsf(error) <= 45.0f)) {
                if (!s.ready) {
                    s.heading_deg = s.magnetic_deg;
                    s.relative_deg = s.gyro_relative_deg = 0;
                    s.ready = TRUE;
                } else {
                    float dt =
                        (float)(now - last_mag) * 0.000001f;
                    if (dt > 0.1f) dt = 0.1f;

                    float correction =
                        dt / (2.0f + dt) * error;
                    s.heading_deg =
                        wrap360(s.heading_deg + correction);
                    s.relative_deg += correction;
                }
                last_mag = now;
            } else {
                ++s.rejected;
            }
        }

        now = heading_time_us();
        if (now - window >= 1000000u) {
            float seconds =
                (float)(now - window) * 0.000001f;

            s.gyro_hz_x10 = (UW)(
                (float)gyro_count * 10.0f / seconds + 0.5f);
            s.mag_hz_x10 = (UW)(
                (float)mag_count * 10.0f / seconds + 0.5f);

            gyro_count = mag_count = 0;
            window = now;
        }

        s.mag_valid = s.ready && now - last_mag <= 300000u;
        s.valid = s.ready;
        s.sample_us = previous;
        publish(&s);
    }

    fail(&s, er);
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
        return sample_flag;

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
        return heading_task;
    }

    ER er = tk_sta_tsk(heading_task, 0);
    if (er < E_OK) {
        (void)tk_del_tsk(heading_task);
        heading_task = 0;
        (void)tk_del_flg(sample_flag);
    }
    return er;
}