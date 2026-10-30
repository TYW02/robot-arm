#include <stddef.h>

/* Use the standard size_t for <math.h>. */
#define PROHIBIT_DEF_SIZE_T

#include "imu_heading.h"
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <sys/sysdef.h>
#include "../../third_party/mtk3smp-rp2040/device/include/dev_i2c.h"

#include <stdint.h>
#include <math.h>

#define MPU_ADDR       0x68U
#define SAMPLE_FLAG    1U

/*
 * Selected during startup after checking WHO_AM_I.
 * Every subsequent magnetic read/write uses this address.
 */
static UW mag_address = 0x1EU;

static ID mpu_device;
static ID mag_device;
static ID sample_flag;
static ID sample_cycle;
static ID heading_task;

static T_SPLOCK reading_lock;
static imu_heading_reading_t published;
static float bias[3];

/* ---------- Time and published readings ---------- */

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

    if (s == NULL)
        return;

    (void)ISpinLock(&reading_lock, &intsts);
    *s = published;
    (void)ISpinUnlock(&reading_lock, intsts);

    if (!s->ready ||
        (UW)(heading_time_us() - s->sample_us) > 50000U) {
        s->valid = FALSE;
    }
}

/* ---------- I2C register access ---------- */

static BOOL read_regs(
    ID device,
    UW address,
    UB reg,
    UB *data,
    SZ length
)
{
    T_I2C_EXEC request = {
        .sadr = address,
        .snd_size = 1,
        .snd_data = &reg,
        .rcv_size = length,
        .rcv_data = data
    };

    SZ done = 0;

    ER er = tk_swri_dev(
        device,
        TDN_I2C_EXEC,
        &request,
        sizeof(request),
        &done
    );

    BOOL ok =
        er == E_OK &&
        done == (SZ)sizeof(request);

    if (!ok &&
        device == mag_device &&
        (address == 0x1DU || address == 0x1EU)) {

        static UW errors = 0;
        ++errors;

        if (errors <= 4U || (errors % 16U) == 0U) {
            tm_printf(
                (UB *)"[MAG READ FAIL] addr=0x%02X "
                       "reg=0x%02X length=%d "
                       "error=%d actual=%d count=%u\n",
                (UINT)address,
                (UINT)reg,
                (INT)length,
                er,
                (INT)done,
                errors
            );
        }
    }

    return ok;
}

static BOOL write_reg(
    ID device,
    UW address,
    UB reg,
    UB value
)
{
    UB data[2] = {reg, value};
    SZ done = 0;

    ER er = tk_swri_dev(
        device,
        address,
        data,
        sizeof(data),
        &done
    );

    return
        er == E_OK &&
        done == (SZ)sizeof(data);
}

/* ---------- Raw gyro ---------- */

static BOOL read_gyro(float rates[3])
{
    UB data[6];

    if (!read_regs(
            mpu_device,
            MPU_ADDR,
            0x43U,
            data,
            sizeof(data))) {
        return FALSE;
    }

    for (INT i = 0; i < 3; ++i) {
        int16_t raw = (int16_t)(
            ((uint16_t)data[2 * i] << 8) |
            data[2 * i + 1]
        );

        /* MPU gyro range: +/-250 degrees/second. */
        rates[i] = (float)raw / 131.0f;
    }

    return TRUE;
}

/* ---------- Magnetometer address diagnostic ---------- */

static BOOL detect_mag_address(void)
{
    const UW addresses[2] = {0x1DU, 0x1EU};
    UW started = heading_time_us();

    tm_printf(
        (UB *)"\n[MAG PROBE] Checking 0x1D and 0x1E.\n"
               "Keep the wiring unchanged during this check.\n"
               "An unused address may report an I2C error.\n"
    );

    do {
        UW stable_mask = 0;
        UW levels = in_w(GPIO_IN);

        /*
         * Function 3 means hardware I2C.
         * Idle SDA and SCL would normally both read high.
         */
        tm_printf(
            (UB *)"[MAG PINS] GP16_func=%u GP17_func=%u "
                   "SDA=%u SCL=%u\n",
            (UINT)(in_w(GPIO_CTRL(16)) & 0x1FU),
            (UINT)(in_w(GPIO_CTRL(17)) & 0x1FU),
            (UINT)((levels >> 16) & 1U),
            (UINT)((levels >> 17) & 1U)
        );

        for (INT address_index = 0;
             address_index < 2;
             ++address_index) {

            UW passes = 0;

            for (INT attempt = 1; attempt <= 3; ++attempt) {
                UB identity = 0;

                BOOL ok = read_regs(
                    mag_device,
                    addresses[address_index],
                    0x0FU,
                    &identity,
                    1
                );

                tm_printf(
                    (UB *)"[MAG PROBE] addr=0x%02X "
                           "try=%d read=%s WHO=0x%02X\n",
                    (UINT)addresses[address_index],
                    attempt,
                    (UB *)(ok ? "OK" : "FAILED"),
                    (UINT)identity
                );

                if (ok && identity == 0x49U)
                    ++passes;

                (void)tk_dly_tsk(20);
            }

            if (passes == 3U)
                stable_mask |= 1U << address_index;
        }

        if (stable_mask == 1U || stable_mask == 2U) {
            mag_address =
                stable_mask == 1U ? 0x1DU : 0x1EU;

            tm_printf(
                (UB *)"[MAG FOUND] address=0x%02X "
                       "WHO_AM_I=0x49 passed 3/3 checks\n",
                (UINT)mag_address
            );

            return TRUE;
        }

        if (stable_mask == 3U) {
            tm_printf(
                (UB *)"[MAG PROBE] Both addresses returned 0x49. "
                       "Stopping because the result is ambiguous.\n"
            );

            return FALSE;
        }

        tm_printf(
            (UB *)"[MAG PROBE] No stable identity yet; "
                   "repeating in 2 seconds.\n"
        );

        (void)tk_dly_tsk(2000);

    } while (
        (UW)(heading_time_us() - started) < 20000000U
    );

    tm_printf(
        (UB *)"[MAG PROBE FAILED] Neither address passed "
               "three identity reads. Mission remains held.\n"
    );

    return FALSE;
}

/* ---------- Sensor initialisation ---------- */

static ER initialise_sensors(void)
{
    /*
     * Kernel startup registers I2C0 as "iica".
     * Register I2C1 as "iicb" here once.
     *
     * Pins are configured in the RP2040 I2C driver:
     * MPU: I2C1, SDA GP2 / SCL GP3.
     * MAG: I2C0, SDA GP16 / SCL GP17.
     */
    ER er = dev_init_i2c(1);

    if (er < E_OK)
        return er;

    mpu_device = tk_opn_dev(
        (const UB *)"iicb",
        TD_READ | TD_WRITE
    );

    if (mpu_device <= E_OK)
        return mpu_device;

    mag_device = tk_opn_dev(
        (const UB *)"iica",
        TD_READ | TD_WRITE
    );

    if (mag_device <= E_OK)
        return mag_device;

    UB id = 0;

    if (!read_regs(
            mpu_device,
            MPU_ADDR,
            0x75U,
            &id,
            1)) {
        tm_printf((UB *)"[MPU] WHO_AM_I read failed.\n");
        return E_IO;
    }

    tm_printf(
        (UB *)"[MPU] WHO_AM_I=0x%02X expected=0x70\n",
        (UINT)id
    );

    if (id != 0x70U)
        return E_IO;

    if (!detect_mag_address())
        return E_IO;

    /* Reset MPU, then select the gyro PLL clock. */
    if (!write_reg(
            mpu_device,
            MPU_ADDR,
            0x6BU,
            0x80U)) {
        return E_IO;
    }

    (void)tk_dly_tsk(100);

    if (!write_reg(
            mpu_device,
            MPU_ADDR,
            0x6BU,
            0x01U)) {
        return E_IO;
    }

    (void)tk_dly_tsk(100);

    /*
     * Preserve the settings used with the existing calibration:
     *
     * MPU:
     *   All axes enabled.
     *   Gyro low-pass filter setting 3.
     *   100 Hz sensor output.
     *   +/-250 dps, 131 counts per dps.
     *
     * LSM303D:
     *   Accelerometer off, block data update enabled.
     *   High-resolution magnetic output at 25 Hz.
     *   +/-2 gauss.
     *   Continuous magnetic conversion.
     */
    if (!write_reg(
            mpu_device, MPU_ADDR, 0x6CU, 0x00U) ||
        !write_reg(
            mpu_device, MPU_ADDR, 0x1AU, 0x03U) ||
        !write_reg(
            mpu_device, MPU_ADDR, 0x19U, 9U) ||
        !write_reg(
            mpu_device, MPU_ADDR, 0x1BU, 0x00U) ||
        !write_reg(
            mag_device, mag_address, 0x20U, 0x08U) ||
        !write_reg(
            mag_device, mag_address, 0x24U, 0x6CU) ||
        !write_reg(
            mag_device, mag_address, 0x25U, 0x00U) ||
        !write_reg(
            mag_device, mag_address, 0x26U, 0x00U)) {

        tm_printf(
            (UB *)"[IMU] Sensor configuration write failed.\n"
        );

        return E_IO;
    }

    (void)tk_dly_tsk(100);

    return E_OK;
}

/* ---------- Stationary gyro calibration ---------- */

static ER calibrate_gyro(void)
{
    float sum[3] = {0};
    float sum_squared[3] = {0};

    tm_printf(
        (UB *)"[IMU] keep completely still; "
               "calibration starts in 3 seconds\n"
    );

    (void)tk_dly_tsk(3000);

    for (INT n = 0; n < 200; ++n) {
        float rates[3];

        if (!read_gyro(rates))
            return E_IO;

        for (INT i = 0; i < 3; ++i) {
            if (fabsf(rates[i]) > 245.0f)
                return E_IO;

            sum[i] += rates[i];
            sum_squared[i] += rates[i] * rates[i];
        }

        (void)tk_dly_tsk(10);
    }

    for (INT i = 0; i < 3; ++i) {
        bias[i] = sum[i] / 200.0f;

        float variance =
            sum_squared[i] / 200.0f -
            bias[i] * bias[i];

        if (variance > 1.0f) {
            tm_printf(
                (UB *)"[IMU] calibration too noisy "
                       "on axis %d\n",
                i
            );

            return E_IO;
        }
    }

    return E_OK;
}

/* ---------- Magnetic data ---------- */

/*
 * Return values:
 * -1: I2C read error.
 *  0: No fresh sample.
 *  1: Fresh sample.
 *  2: Fresh sample with overrun.
 */
static INT read_mag(int16_t raw[3])
{
    UB status = 0;
    UB data[6];

    if (!read_regs(
            mag_device,
            mag_address,
            0x07U,
            &status,
            1)) {
        return -1;
    }

    if ((status & 0x08U) == 0U)
        return 0;

    /*
     * LSM303D magnetic data starts at 0x08.
     * Bit 7 enables address auto-increment.
     */
    if (!read_regs(
            mag_device,
            mag_address,
            0x88U,
            data,
            sizeof(data))) {
        return -1;
    }

    for (INT i = 0; i < 3; ++i) {
        raw[i] = (int16_t)(
            (uint16_t)data[2 * i] |
            ((uint16_t)data[2 * i + 1] << 8)
        );
    }

    return (status & 0x80U) != 0U ? 2 : 1;
}

static float wrap360(float angle)
{
    angle = fmodf(angle, 360.0f);

    return angle < 0.0f
        ? angle + 360.0f
        : angle;
}

static float wrap180(float angle)
{
    return wrap360(angle + 180.0f) - 180.0f;
}

static BOOL magnetic_heading(
    const int16_t raw[3],
    float *angle,
    float *field
)
{
    /*
     * Existing mounted-car level-only calibration.
     * Preserve the sensor range and mounting for these values.
     */
    float x = (float)raw[0] - (-2091.9877f);
    float y = (float)raw[1] - (-1432.6338f);

    float cx =
        0.00025042909f * x -
        0.00000296088f * y;

    float cy =
        -0.00000296088f * x +
        0.00026967533f * y;

    float squared = cx * cx + cy * cy;

    if (!isfinite(squared) || squared < 0.000001f)
        return FALSE;

    *field = sqrtf(squared);

    *angle = wrap360(
        atan2f(-cy, cx) * 57.2957795f
    );

    return TRUE;
}

/* ---------- Task timing and fault handling ---------- */

/* Handler only wakes the task; sensor access runs in task context. */
static void sample_tick(void *exinf)
{
    (void)exinf;

    (void)tk_set_flg(
        sample_flag,
        SAMPLE_FLAG
    );
}

static void fail(
    imu_heading_reading_t *s,
    ER error
)
{
    if (sample_cycle > E_OK)
        (void)tk_stp_cyc(sample_cycle);

    s->ready = FALSE;
    s->valid = FALSE;
    s->mag_valid = FALSE;
    s->fault = error;

    publish(s);

    tm_printf(
        (UB *)"[IMU] fault %d; "
               "reset before restarting test\n",
        error
    );
}

/* ---------- IMU owner task ---------- */

static void task_imu_heading(
    INT stacd,
    void *exinf
)
{
    (void)stacd;
    (void)exinf;

    imu_heading_reading_t s = {0};

    tm_printf(
        (UB *)"[IMU] Address-probe diagnostic v1\n"
    );

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

    tm_printf(
        (UB *)"[IMU] calibration complete; "
               "fusion running, keep level\n"
    );

    UW previous = 0;
    UW last_mag = 0;
    UW window = heading_time_us();

    UW gyro_count = 0;
    UW mag_count = 0;

    BOOL have_previous = FALSE;

    for (;;) {
        UINT flags;

        er = tk_wai_flg(
            sample_flag,
            SAMPLE_FLAG,
            TWF_ORW | TWF_BITCLR,
            &flags,
            TMO_FEVR
        );

        if (er < E_OK)
            break;

        UW now = heading_time_us();

        if (have_previous &&
            (UW)(now - previous) < 5000U) {
            continue;
        }

        float rates[3];

        if (!read_gyro(rates)) {
            er = E_IO;
            break;
        }

        for (INT i = 0; i < 3; ++i) {
            if (fabsf(rates[i]) > 245.0f)
                er = E_IO;
        }

        if (er < E_OK)
            break;

        now = heading_time_us();

        UW elapsed = now - previous;

        if (have_previous && elapsed > 50000U) {
            er = E_TMOUT;
            break;
        }

        if (have_previous && elapsed > 15000U)
            ++s.late;

        s.gyro_z_dps = rates[2] - bias[2];

        if (s.ready && have_previous) {
            float delta =
                s.gyro_z_dps *
                ((float)elapsed * 0.000001f);

            s.heading_deg = wrap360(
                s.heading_deg + delta
            );

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

            BOOL usable = magnetic_heading(
                raw,
                &s.magnetic_deg,
                &s.field_xy
            );

            usable =
                usable &&
                s.field_xy >= 0.75f &&
                s.field_xy <= 1.25f;

            float error = s.ready
                ? wrap180(
                    s.magnetic_deg - s.heading_deg
                )
                : 0.0f;

            if (usable &&
                (!s.ready || fabsf(error) <= 45.0f)) {

                if (!s.ready) {
                    s.heading_deg = s.magnetic_deg;
                    s.relative_deg = 0.0f;
                    s.gyro_relative_deg = 0.0f;
                    s.ready = TRUE;
                } else {
                    float dt =
                        (float)(now - last_mag) *
                        0.000001f;

                    if (dt > 0.1f)
                        dt = 0.1f;

                    float correction =
                        dt / (2.0f + dt) * error;

                    s.heading_deg = wrap360(
                        s.heading_deg + correction
                    );

                    s.relative_deg += correction;
                }

                last_mag = now;
            } else {
                ++s.rejected;
            }
        }

        now = heading_time_us();

        if ((UW)(now - window) >= 1000000U) {
            float seconds =
                (float)(now - window) *
                0.000001f;

            s.gyro_hz_x10 = (UW)(
                (float)gyro_count *
                10.0f / seconds + 0.5f
            );

            s.mag_hz_x10 = (UW)(
                (float)mag_count *
                10.0f / seconds + 0.5f
            );

            gyro_count = 0;
            mag_count = 0;
            window = now;
        }

        s.mag_valid =
            s.ready &&
            (UW)(now - last_mag) <= 300000U;

        s.valid = s.ready;
        s.sample_us = previous;

        publish(&s);
    }

    fail(&s, er);
    tk_ext_tsk();
}

/* ---------- Public startup function ---------- */

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