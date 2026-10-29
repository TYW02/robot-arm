#include <stddef.h>

/* Prevent µT-Kernel from redefining size_t. */
#define PROHIBIT_DEF_SIZE_T

#include "imu_mtk3.h"
#include <tm/tmonitor.h>
#include <tk/device.h>
#include "../device/include/dev_i2c.h"

#include <math.h>
#include <string.h>

/* MPU-6500 address and registers. */
#define MPU_ADDR        0x68U
#define WHO_AM_I_REG    0x75U
#define PWR_MGMT_1      0x6BU
#define PWR_MGMT_2      0x6CU
#define SMPLRT_DIV      0x19U
#define CONFIG          0x1AU
#define GYRO_CONFIG     0x1BU
#define INT_ENABLE      0x38U
#define INT_STATUS      0x3AU
#define GYRO_OUT        0x43U

/* +/-500 degrees/second: GYRO_CONFIG FS_SEL = 1. */
#define GYRO_SCALE              65.5f

/* Mutex acquisition timeout, not the I2C transfer timeout. */
#define BUS_LOCK_TIMEOUT_MS     10

#define CALIBRATION_SAMPLES     200
#define CALIBRATION_MAX_DPS     10.0f
#define CALIBRATION_MAX_VAR     0.25f
#define LOW_RATE_DPS            1.0f
#define RAW_SATURATION_LIMIT    32700

/*
 * Startup stages:
 *  0: Not started
 *  1: Open I2C device
 *  2: Check sensor identity
 *  3: Reset sensor
 *  4: Configure sensor
 *  5: Calibration running
 *  6: Establish time reference
 *  7: Startup completed
 *
 * Calibration failure stages:
 * 51: Status-register access failed
 * 52: Fresh-data polling expired
 * 53: Gyro-register access failed
 * 54: Calibration angular rate exceeded limit
 * 55: Calibration variance exceeded limit
 */
#define INIT_FAIL_STATUS_READ   51U
#define INIT_FAIL_DATA_WAIT     52U
#define INIT_FAIL_GYRO_READ     53U
#define INIT_FAIL_RATE_LIMIT    54U
#define INIT_FAIL_VARIANCE      55U

#if IMU_YAW_AXIS < 0 || IMU_YAW_AXIS > 2
#error "IMU_YAW_AXIS must be 0, 1 or 2"
#endif

#if IMU_YAW_SIGN != 1 && IMU_YAW_SIGN != -1
#error "IMU_YAW_SIGN must be 1 or -1"
#endif

static ID device_id = 0;
static ID shared_mutex = 0;

static float gyro_bias[3] = {0};
static float yaw_deg = 0.0f;
static float previous_yaw_rate_dps = 0.0f;

static UW last_ms = 0;
static UW sequence = 0;
static UW reference_id = 0;

static INT calibrated = 0;
static INT heading_valid = 0;

/*
 * Diagnostic scalars only.
 * These getters do not provide an atomic multi-field snapshot.
 */
static volatile UW init_stage = 0;
static volatile UW calibration_count = 0;

static imu_sample_t latest_sample = {0};

UW imu_mtk3_init_stage(void) {
    return init_stage;
}

UW imu_mtk3_calibration_count(void) {
    return calibration_count;
}

/* Time */

static INT get_time_ms(UW *out) {
    SYSTIM time;

    if (tk_get_otm(&time) != E_OK) {
        return 0;
    }

    *out = time.lo;
    return 1;
}

UW imu_mtk3_now_ms(void) {
    UW now = 0;
    (void)get_time_ms(&now);
    return now;
}

/* I2C register access */

static INT get_registers(UB reg, UB *data, SZ count) {
    T_I2C_EXEC transfer = {0};
    SZ actual = 0;

    if (device_id <= 0 || shared_mutex <= 0) {
        return 0;
    }

    if (tk_loc_mtx(shared_mutex, BUS_LOCK_TIMEOUT_MS) != E_OK) {
        return 0;
    }

    transfer.sadr = MPU_ADDR;
    transfer.snd_size = 1;
    transfer.snd_data = &reg;
    transfer.rcv_size = count;
    transfer.rcv_data = data;

    ER er = tk_swri_dev(
        device_id,
        TDN_I2C_EXEC,
        &transfer,
        sizeof(transfer),
        &actual
    );

    (void)tk_unl_mtx(shared_mutex);

    return er == E_OK &&
           actual == (SZ)sizeof(transfer);
}

static INT put_register(UB reg, UB value) {
    UB bytes[2] = {reg, value};
    SZ actual = 0;

    if (device_id <= 0 || shared_mutex <= 0) {
        return 0;
    }

    if (tk_loc_mtx(shared_mutex, BUS_LOCK_TIMEOUT_MS) != E_OK) {
        return 0;
    }

    ER er = tk_swri_dev(
        device_id,
        MPU_ADDR,
        bytes,
        sizeof(bytes),
        &actual
    );

    (void)tk_unl_mtx(shared_mutex);

    return er == E_OK &&
           actual == (SZ)sizeof(bytes);
}

/* Write a configuration register and verify its value. */
static INT configure_register(UB reg, UB value) {
    UB actual = 0;

    return put_register(reg, value) &&
           get_registers(reg, &actual, 1) &&
           actual == value;
}

/* Raw gyroscope readings */

static INT get_raw_gyro(H raw[3]) {
    UB data[6];

    if (!get_registers(GYRO_OUT, data, sizeof(data))) {
        return 0;
    }

    for (INT axis = 0; axis < 3; axis++) {
        raw[axis] = (H)(
            ((UH)data[axis * 2] << 8) |
            data[axis * 2 + 1]
        );
    }

    return 1;
}

/*
 * Near-limit readings may indicate actual saturation or
 * abnormal sensor/communication data.
 */
static INT raw_near_full_scale(const H raw[3]) {
    for (INT axis = 0; axis < 3; axis++) {
        if (raw[axis] >= RAW_SATURATION_LIMIT ||
            raw[axis] <= -RAW_SATURATION_LIMIT) {
            return 1;
        }
    }

    return 0;
}

static INT raw_has_multiple_saturated_axes(const H raw[3]) {
    INT count = 0;

    for (INT axis = 0; axis < 3; axis++) {
        if (raw[axis] >= RAW_SATURATION_LIMIT ||
            raw[axis] <= -RAW_SATURATION_LIMIT) {
            count++;
        }
    }

    return count >= 2;
}

/*
 * Used during calibration only.
 *
 * Returns:
 *  1: Fresh gyro data is available.
 *  0: Fresh-data polling expired.
 * -1: Status-register access failed.
 *
 * The polling limit is not an exact wall-clock timeout:
 * kernel delays and I2C operations also take time.
 */
static INT wait_for_gyro(void) {
    for (INT attempt = 0; attempt < 25; attempt++) {
        UB status = 0;

        if (!get_registers(INT_STATUS, &status, 1)) {
            return -1;
        }

        if ((status & 0x01U) != 0) {
            return 1;
        }

        (void)tk_dly_tsk(2);
    }

    return 0;
}

/* Convert an angle to [-180, 180). */
static float wrap_yaw(float angle) {
    float wrapped = remainderf(angle, 360.0f);

    if (wrapped >= 180.0f) {
        wrapped -= 360.0f;
    }

    return wrapped;
}

/* Initialisation and calibration */

ER imu_mtk3_init(ID shared_i2c_mutex) {
    calibrated = 0;
    init_stage = 1;
    calibration_count = 0;
    heading_valid = 0;
    yaw_deg = 0.0f;
    previous_yaw_rate_dps = 0.0f;

    memset(&latest_sample, 0, sizeof(latest_sample));
    latest_sample.status = IMU_STATUS_NOT_READY;

    shared_mutex = shared_i2c_mutex;

    if (shared_mutex <= 0) {
        return E_PAR;
    }

    /*
     * I2C1 is registered as "iicb".
     * GP6 = SDA, GP7 = SCL.
     */
    if (device_id <= 0) {
        device_id = tk_opn_dev(
            (CONST UB *)"iicb",
            TD_UPDATE
        );

        if (device_id <= 0) {
            return E_IO;
        }
    }

    init_stage = 2;

    UB who = 0;

    if (!get_registers(WHO_AM_I_REG, &who, 1) ||
        who != 0x70U) {
        return E_IO;
    }

    init_stage = 3;

    /* Reset the sensor to known settings. */
    if (!put_register(PWR_MGMT_1, 0x80U)) {
        return E_IO;
    }

    (void)tk_dly_tsk(100);

    /*
     * Wake sensor and select automatic PLL clock.
     * Disable accelerometers; keep all gyro axes enabled.
     *
     * Gyro range: +/-500 dps.
     * Gyro low-pass bandwidth: 20 Hz.
     * Sensor output rate: 1000 / (1 + 9) = 100 Hz.
     */
    init_stage = 4;

    if (!configure_register(PWR_MGMT_1, 0x01U) ||
        !configure_register(PWR_MGMT_2, 0x38U) ||
        !configure_register(GYRO_CONFIG, 0x08U) ||
        !configure_register(CONFIG, 0x04U) ||
        !configure_register(SMPLRT_DIV, 9U) ||
        !configure_register(INT_ENABLE, 0x01U)) {
        return E_IO;
    }

     /*
     * Initial settling period before measuring gyro bias.
     * Keep the board stationary during this wait and calibration.
     * This delay does not guarantee thermal stability.
     */
    (void)tk_dly_tsk(5000);

    /*
     * Keep the car stopped throughout calibration.
     *
     * Mean and variance checks detect gross movement and
     * unstable readings. They cannot distinguish constant
     * slow rotation from sensor bias.
     */
    init_stage = 5;

    float mean[3] = {0};
    float variance_sum[3] = {0};

    for (INT n = 1; n <= CALIBRATION_SAMPLES; n++) {
        /* Current attempted sample, not completed samples. */
        calibration_count = (UW)n;

        H raw[3];

        INT ready = wait_for_gyro();

        if (ready < 0) {
            init_stage = INIT_FAIL_STATUS_READ;
            return E_IO;
        }

        if (ready == 0) {
            init_stage = INIT_FAIL_DATA_WAIT;
            return E_TMOUT;
        }

        if (!get_raw_gyro(raw)) {
            init_stage = INIT_FAIL_GYRO_READ;
            return E_IO;
        }

        for (INT axis = 0; axis < 3; axis++) {
            float rate = (float)raw[axis] / GYRO_SCALE;

            if (fabsf(rate) > CALIBRATION_MAX_DPS) {
                init_stage = INIT_FAIL_RATE_LIMIT;
                return E_IO;
            }

            float difference = rate - mean[axis];

            mean[axis] += difference / (float)n;

            variance_sum[axis] +=
                difference * (rate - mean[axis]);
        }

        (void)tk_dly_tsk(10);
    }

        INT unstable = 0;

    /*
     * Print all three axes before deciding whether calibration
     * passed. Values use thousandths of a degree per second.
     */
    INT limit_mdps =
        (INT)(sqrtf(CALIBRATION_MAX_VAR) * 1000.0f);

    for (INT axis = 0; axis < 3; axis++) {
        float variance =
            variance_sum[axis] /
            (float)(CALIBRATION_SAMPLES - 1);

        INT failed = variance > CALIBRATION_MAX_VAR;

        INT mean_mdps =
            (INT)(mean[axis] * 1000.0f);

        INT std_mdps =
            (INT)(sqrtf(variance) * 1000.0f);

        tm_printf(
            (UB *)"[GYRO CAL] Axis=%c "
                   "| Mean=%d mdps "
                   "| StdDev=%d mdps "
                   "| Limit=%d mdps "
                   "| %s\n",
            (INT)('X' + axis),
            mean_mdps,
            std_mdps,
            limit_mdps,
            (UB *)(failed ? "FAIL" : "PASS")
        );

        if (failed) {
            unstable = 1;
        }
    }

    if (unstable) {
        init_stage = INIT_FAIL_VARIANCE;
        return E_IO;
    }

    /* Accept offsets only after every axis passes. */
    for (INT axis = 0; axis < 3; axis++) {
        gyro_bias[axis] = mean[axis];
    }

    init_stage = 6;

    if (!get_time_ms(&last_ms)) {
        return E_IO;
    }

    reference_id++;
    calibrated = 1;
    heading_valid = 1;

    init_stage = 7;
    return E_OK;
}

/* Invalid-sample handling */

static INT reject_sample(
    imu_sample_t *out,
    imu_status_t status
) {
    /*
     * Missing rotation cannot be recovered from a later
     * gyro reading. Require an explicit new reference.
     */
    heading_valid = 0;
    previous_yaw_rate_dps = 0.0f;

    out->rate_valid = 0;
    out->heading_valid = 0;
    out->low_rate = 0;
    out->status = status;

    latest_sample = *out;
    return 0;
}

/* Read gyro and update relative yaw */

INT imu_mtk3_read(imu_sample_t *out) {
    if (out == NULL) {
        return 0;
    }

    memset(out, 0, sizeof(*out));

    out->sequence = ++sequence;
    out->reference_id = reference_id;
    out->calibrated = calibrated;
    out->yaw_deg = yaw_deg;
    out->yaw_wrapped_deg = wrap_yaw(yaw_deg);

    if (!get_time_ms(&out->timestamp_ms)) {
        return reject_sample(out, IMU_STATUS_TIMING);
    }

    if (!calibrated) {
        return reject_sample(out, IMU_STATUS_NOT_READY);
    }

    UB status = 0;

    if (!get_registers(INT_STATUS, &status, 1)) {
        return reject_sample(out, IMU_STATUS_IO);
    }

    if ((status & 0x01U) == 0) {
        return reject_sample(out, IMU_STATUS_NO_DATA);
    }

    H raw[3];

    if (!get_raw_gyro(raw)) {
        return reject_sample(out, IMU_STATUS_IO);
    }

    /*
     * Preserve the existing multi-axis saturation retry.
     * A second normal reading does not prove the first
     * reading was corrupt; this behaviour still needs
     * investigation during hardware diagnosis.
     */
    if (raw_has_multiple_saturated_axes(raw)) {
        H check[3];

        if (!get_raw_gyro(check)) {
            return reject_sample(out, IMU_STATUS_IO);
        }

        if (!raw_near_full_scale(check)) {
            memcpy(raw, check, sizeof(raw));
        }
    }

    UW now;

    if (!get_time_ms(&now)) {
        return reject_sample(out, IMU_STATUS_TIMING);
    }

    /* Unsigned subtraction handles timer wraparound. */
    UW interval = now - last_ms;
    last_ms = now;

    out->timestamp_ms = now;
    out->interval_ms = interval;

    out->raw_gx = raw[0];
    out->raw_gy = raw[1];
    out->raw_gz = raw[2];

    float rate[3];

    for (INT axis = 0; axis < 3; axis++) {
        rate[axis] =
            (float)raw[axis] / GYRO_SCALE -
            gyro_bias[axis];
    }

    out->gyro_x_dps = rate[0];
    out->gyro_y_dps = rate[1];
    out->gyro_z_dps = rate[2];

    out->yaw_rate_dps =
        (float)IMU_YAW_SIGN * rate[IMU_YAW_AXIS];

    if (raw_near_full_scale(raw)) {
        return reject_sample(out, IMU_STATUS_SATURATED);
    }

    if (interval == 0 || interval > IMU_MAX_GAP_MS) {
        return reject_sample(out, IMU_STATUS_TIMING);
    }

    out->rate_valid = 1;

    out->low_rate =
        fabsf(rate[0]) <= LOW_RATE_DPS &&
        fabsf(rate[1]) <= LOW_RATE_DPS &&
        fabsf(rate[2]) <= LOW_RATE_DPS;

    /*
     * Integrate only while the heading reference is valid.
     * Small rates are retained so slow turns still count.
     */
    if (heading_valid) {
        float elapsed_seconds = (float)interval * 0.001f;

        yaw_deg += 0.5f *
            (previous_yaw_rate_dps + out->yaw_rate_dps) *
            elapsed_seconds;
    }

    previous_yaw_rate_dps = out->yaw_rate_dps;

    out->yaw_deg = yaw_deg;
    out->yaw_wrapped_deg = wrap_yaw(yaw_deg);
    out->heading_valid = heading_valid;
    out->status = IMU_STATUS_OK;

    latest_sample = *out;

    /*
     * A valid rate does not guarantee a valid heading.
     * Consumers must check heading_valid before using yaw.
     */
    return 1;
}

/* Establish a new relative-yaw reference */

ER imu_mtk3_set_yaw(float new_yaw_deg) {
    UW now;

    if (!isfinite(new_yaw_deg) ||
        fabsf(new_yaw_deg) > 1000000.0f) {
        return E_PAR;
    }

    if (!get_time_ms(&now)) {
        return E_IO;
    }

    /*
     * The caller must ensure the car is stopped.
     * A fresh low-rate sample is an additional check,
     * not proof that the car is stationary.
     */
    if (!calibrated ||
        !latest_sample.rate_valid ||
        !latest_sample.low_rate ||
        (UW)(now - latest_sample.timestamp_ms) >
            IMU_MAX_GAP_MS) {
        return E_OBJ;
    }

    yaw_deg = new_yaw_deg;
    previous_yaw_rate_dps = latest_sample.yaw_rate_dps;
    last_ms = now;
    reference_id++;
    heading_valid = 1;

    /* Require a fresh sample before another reference reset. */
    latest_sample.rate_valid = 0;
    latest_sample.heading_valid = 0;

    return E_OK;
}