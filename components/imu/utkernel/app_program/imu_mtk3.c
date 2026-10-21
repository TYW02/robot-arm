#include <stddef.h>

// Use the standard library's size_t definition.
// Prevent µT-Kernel from redefining it.
#define PROHIBIT_DEF_SIZE_T

#include "imu_mtk3.h"
#include <tk/device.h>
#include "../device/include/dev_i2c.h"
#include <math.h>
#include <string.h>


#define MPU_ADDR        0x68U
#define WHO_AM_I_REG    0x75U
#define PWR_MGMT_1      0x6BU
#define ACCEL_CONFIG    0x1CU
#define GYRO_CONFIG     0x1BU
#define SENSOR_BLOCK    0x3BU

#define DEG_PER_RAD     57.2957795f
#define GYRO_SCALE      131.0f
#define ACCEL_SCALE     16384.0f

#define MAX_GAP_MS      50U
#define BUS_TIMEOUT_MS  10

static ID device_id = 0;
static ID shared_mutex = 0;

static float gx_bias = 0;
static float gy_bias = 0;
static float gz_bias = 0;

static float pitch_deg = 0;
static float roll_deg = 0;

static UW last_ms = 0;
static INT filter_valid = 0;

/* Detect repeated identical sensor readings. */
static H previous_raw[6];
static INT previous_raw_available = 0;
static UINT identical_raw_count = 0;

/* -------------------------------------
 * Time
 * ------------------------------------- */

UW imu_mtk3_now_ms(void) {
    SYSTIM t;

    if (tk_get_otm(&t) != E_OK) {
        return 0;
    }

    return t.lo;
}

/* -------------------------------------
 * I2C register read
 * ------------------------------------- */

static INT get_registers(
    UB reg,
    UB *data,
    SZ count
) {
    UB address = reg;
    T_I2C_EXEC transfer = {0};
    SZ actual = 0;

    if (device_id <= 0 ||
        shared_mutex <= 0) {
        return 0;
    }

    if (tk_loc_mtx(
            shared_mutex,
            BUS_TIMEOUT_MS
        ) != E_OK) {
        return 0;
    }

    transfer.sadr = MPU_ADDR;
    transfer.snd_size = 1;
    transfer.snd_data = &address;
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

/* -------------------------------------
 * I2C register write
 * ------------------------------------- */

static INT put_register(
    UB reg,
    UB value
) {
    UB bytes[2] = {reg, value};
    SZ actual = 0;

    if (device_id <= 0 ||
        shared_mutex <= 0) {
        return 0;
    }

    if (tk_loc_mtx(
            shared_mutex,
            BUS_TIMEOUT_MS
        ) != E_OK) {
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

/* -------------------------------------
 * Raw sensor data
 * ------------------------------------- */

static H signed_word(
    UB high,
    UB low
) {
    return (H)(((UH)high << 8) | low);
}

static INT get_raw(
    H *ax, H *ay, H *az,
    H *gx, H *gy, H *gz
) {
    UB data[14];

    if (!get_registers(
            SENSOR_BLOCK,
            data,
            sizeof(data))) {
        return 0;
    }

    *ax = signed_word(data[0], data[1]);
    *ay = signed_word(data[2], data[3]);
    *az = signed_word(data[4], data[5]);

    *gx = signed_word(data[8], data[9]);
    *gy = signed_word(data[10], data[11]);
    *gz = signed_word(data[12], data[13]);

    return 1;
}

/* -------------------------------------
 * Calculate accelerometer angles
 * ------------------------------------- */

static INT accel_angles(
    H ax, H ay, H az,
    float *new_pitch,
    float *new_roll,
    float *magnitude
) {
    float x = (float)ax / ACCEL_SCALE;
    float y = (float)ay / ACCEL_SCALE;
    float z = (float)az / ACCEL_SCALE;

    float m2 = x*x + y*y + z*z;

    /*
     * Reject implausible acceleration.
     * Normal stationary magnitude is ~1g.
     */
    if (!isfinite(m2) ||
        m2 < 0.64f ||
        m2 > 1.44f) {
        return 0;
    }

    *magnitude = sqrtf(m2);

    *new_roll =
        atan2f(y, z) * DEG_PER_RAD;

    *new_pitch =
        atan2f(
            -x,
            sqrtf(y*y + z*z)
        ) * DEG_PER_RAD;

    return isfinite(*new_pitch) &&
           isfinite(*new_roll);
}

/* -------------------------------------
 * Initialise MPU-6500
 * ------------------------------------- */

ER imu_mtk3_init(
    ID shared_i2c_mutex
) {
    UB who = 0;

    H ax, ay, az;
    H gx, gy, gz;

    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    float magnitude;

    filter_valid = 0;
    previous_raw_available = 0;
    identical_raw_count = 0;
    shared_mutex = shared_i2c_mutex;

    if (shared_mutex <= 0) {
        return E_PAR;
    }

    /*
     * The repository registers I2C1 as "iicb".
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

    /*
     * MPU-6500 WHO_AM_I should be 0x70.
     */
    if (!get_registers(
            WHO_AM_I_REG,
            &who,
            1
        ) || who != 0x70) {
        return E_IO;
    }

    /* Wake up the MPU-6500. */
    if (!put_register(
            PWR_MGMT_1,
            0x01
        )) {
        return E_IO;
    }

    /* Give the sensor time to wake up. */
    (void)tk_dly_tsk(100);

    /* Read back the power management register. */
    UB power_state = 0xFF;

    if (!get_registers(
            PWR_MGMT_1,
            &power_state,
            1
        )) {
        return E_IO;
    }

    /* Check that sleep mode is disabled. */
    if ((power_state & 0x40U) != 0) {
        return E_IO;
    }

    /* Check that the clock source is set correctly. */
    if ((power_state & 0x07U) != 0x01U) {
        return E_IO;
    }

    /* +/-2g accelerometer, +/-250 dps gyro. */
    if (!put_register(ACCEL_CONFIG, 0x00) ||
        !put_register(GYRO_CONFIG, 0x00)) {
        return E_IO;
    }

    /*
     * Calibrate gyroscope offsets.
     * IMU must remain stationary.
     */
    for (INT i = 0; i < 100; i++) {
        if (!get_raw(
                &ax, &ay, &az,
                &gx, &gy, &gz)) {
            return E_IO;
        }

        sum_x += (float)gx / GYRO_SCALE;
        sum_y += (float)gy / GYRO_SCALE;
        sum_z += (float)gz / GYRO_SCALE;

        (void)tk_dly_tsk(5);
    }

    gx_bias = sum_x / 100.0f;
    gy_bias = sum_y / 100.0f;
    gz_bias = sum_z / 100.0f;

    /*
     * Establish initial pitch/roll.
     */
    if (!get_raw(
            &ax, &ay, &az,
            &gx, &gy, &gz)) {
        return E_IO;
    }

    if (!accel_angles(
            ax, ay, az,
            &pitch_deg,
            &roll_deg,
            &magnitude)) {
        return E_IO;
    }

    last_ms = imu_mtk3_now_ms();
    filter_valid = 1;

    return E_OK;
}

/* -------------------------------------
 * Read and update orientation
 * ------------------------------------- */

INT imu_mtk3_read(
    imu_sample_t *out
) {
    H ax, ay, az;
    H gx, gy, gz;

    float accel_pitch;
    float accel_roll;
    float magnitude;

    if (out == NULL) {
        return 0;
    }

    memset(out, 0, sizeof(*out));

    
    if (!get_raw(
            &ax, &ay, &az,
            &gx, &gy, &gz)) {
        filter_valid = 0;
        return 0;
    }

    // Store the raw sensor values
    out->raw_ax = ax;
    out->raw_ay = ay;
    out->raw_az = az;

    out->raw_gx = gx;
    out->raw_gy = gy;
    out->raw_gz = gz;

    /* Compare all six raw sensor readings. */
    H current_raw[6] = {
        ax, ay, az, gx, gy, gz
    };

    if (previous_raw_available &&
        memcmp(previous_raw,
            current_raw,
            sizeof(current_raw)) == 0) {

        identical_raw_count++;

    } else {
        identical_raw_count = 0;
    }

    /* Save the current readings for next time. */
    memcpy(previous_raw,
        current_raw,
        sizeof(current_raw));

    previous_raw_available = 1;

    /* Reject potentially frozen sensor data. */
    if (identical_raw_count >= 10U) {
        filter_valid = 0;
        out->valid = 0;
        return 0;
    }

    // Calculate pitch and roll
    if (!accel_angles(
            ax, ay, az,
            &accel_pitch,
            &accel_roll,
            &magnitude)) {
        filter_valid = 0;
        return 0;
    }


    UW now = imu_mtk3_now_ms();
    UW gap = now - last_ms;

    last_ms = now;

    out->timestamp_ms = now;
    out->interval_ms = gap;
    out->accel_g = magnitude;

    INT on_time =
        gap > 0 &&
        gap <= MAX_GAP_MS;

    float gyro_x =
        (float)gx / GYRO_SCALE - gx_bias;

    float gyro_y =
        (float)gy / GYRO_SCALE - gy_bias;

    float gyro_z =
        (float)gz / GYRO_SCALE - gz_bias;

    out->gyro_rate_dps = sqrtf(
        gyro_x*gyro_x +
        gyro_y*gyro_y +
        gyro_z*gyro_z
    );

    /*
     * Stationary threshold is preliminary.
     * Validate this on the actual robotic arm.
     */
    out->stationary =
        out->gyro_rate_dps <= 3.0f &&
        magnitude >= 0.85f &&
        magnitude <= 1.15f;

    /*
     * Complementary filter.
     *
     * After a delayed sample, recover the
     * filter but reject that reading.
     */
    if (!filter_valid || !on_time) {
        roll_deg = accel_roll;
        pitch_deg = accel_pitch;
    } else {
        float dt =
            (float)gap * 0.001f;

        float predicted_roll =
            roll_deg + gyro_x * dt;

        float roll_error =
            remainderf(
                accel_roll - predicted_roll,
                360.0f
            );

        roll_deg = remainderf(
            predicted_roll +
            0.02f * roll_error,
            360.0f
        );

        pitch_deg =
            0.98f * (
                pitch_deg + gyro_y * dt
            ) +
            0.02f * accel_pitch;
    }

    filter_valid =
        on_time &&
        isfinite(pitch_deg) &&
        isfinite(roll_deg);

    out->pitch_deg = pitch_deg;
    out->roll_deg = roll_deg;
    out->valid = filter_valid;

    return out->valid;
}
