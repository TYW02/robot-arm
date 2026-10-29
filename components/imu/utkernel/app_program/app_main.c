#include <stddef.h>
#define PROHIBIT_DEF_SIZE_T

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <math.h>
#include <string.h>

#include "imu_service.h"
#include "mag_mtk3.h"
#include "usb_console_compat.h"

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#define MONITOR_STACK          8192
#define MAG_POLL_MS            100U
#define DISPLAY_MS             1000U

#define COUNTDOWN_MS           10000U
#define COLLECTION_MS          60000U
#define POSITIONING_MS         15000U
#define REFERENCE_TIMEOUT_MS   30000U

#define MAX_CAL_SAMPLES        800U
#define MIN_CAL_SAMPLES        200U
#define CAL_SECTORS            16
#define MIN_SECTOR_SAMPLES     3U

/*
 * Initial diagnostic acceptance limits.
 * These are engineering checks, not accuracy guarantees.
 */
#define MIN_AXIS_SPAN          200.0f
#define MAX_AXIS_RATIO         2.0f
#define MAX_RADIUS_RMS         0.10f
#define MAX_RADIUS_ERROR       0.25f
#define MAX_Z_SPAN_RATIO       0.35f

#define MAG_FRESH_MS           500U
#define REFERENCE_SAMPLES      20U
#define REFERENCE_RATE_DPS     0.50f
#define REFERENCE_ANGLE_LIMIT  3.0f
#define REFERENCE_GYRO_LIMIT   0.50f

#define RAD_TO_DEG             57.2957795131f
#define DEG_TO_RAD             0.01745329252f

/*
 * Provisional mapping for the described mounting:
 * MPU component side up; GY-511 component side down;
 * both printed +X arrows point in the same direction.
 *
 * Verify the resulting heading direction before fusion.
 */
#define MAG_BODY_X_SIGN        1.0f
#define MAG_BODY_Y_SIGN       -1.0f

typedef enum {
    PHASE_WAIT_GYRO = 0,
    PHASE_COUNTDOWN,
    PHASE_COLLECT,
    PHASE_POSITION,
    PHASE_REFERENCE,
    PHASE_RUN,
    PHASE_FAILED
} phase_t;

typedef struct {
    H x;
    H y;
    H z;
} calibration_sample_t;

typedef struct {
    float bias_x;
    float bias_y;
    float radius_x;
    float radius_y;
    float mean_z;
    float z_tolerance;
    INT valid;
} planar_calibration_t;

/* Static storage keeps the calibration buffer off the task stack. */
static calibration_sample_t cal_samples[MAX_CAL_SAMPLES];
static UW cal_count = 0;
static planar_calibration_t calibration;

static ID i2c_shared_mutex = 0;
static phase_t phase = PHASE_WAIT_GYRO;
static UW phase_start = 0;
static UW last_mag_ms = 0;

static mag_sample_t latest_mag;
static INT have_mag = 0;
static INT mag_read_result = 0;

static UW reference_count = 0;
static UW reference_last_ms = 0;
static UW reference_id = 0;
static float reference_first_mag = 0.0f;
static float reference_first_gyro = 0.0f;
static float reference_sin_sum = 0.0f;
static float reference_cos_sum = 0.0f;
static float reference_gyro_sum = 0.0f;

static float magnetic_reference = 0.0f;
static float gyro_reference = 0.0f;

/* ---------- Formatting ---------- */

static void print_value(const char *name, float value)
{
    if (!isfinite(value) || fabsf(value) > 1000000.0f) {
        tm_printf((UB *)"%s=unavailable ", (UB *)name);
        return;
    }

    INT scaled = (INT)(value * 100.0f);
    UINT magnitude = scaled < 0
        ? (UINT)(-scaled)
        : (UINT)scaled;

    tm_printf(
        (UB *)"%s=%s%u.%02u ",
        (UB *)name,
        (UB *)(scaled < 0 ? "-" : ""),
        magnitude / 100U,
        magnitude % 100U
    );
}

static float wrap_degrees(float angle)
{
    float result = remainderf(angle, 360.0f);

    if (result >= 180.0f) {
        result -= 360.0f;
    }

    return result;
}

static const char *phase_name(void)
{
    switch (phase) {
        case PHASE_WAIT_GYRO: return "WAIT_GYRO";
        case PHASE_COUNTDOWN: return "COUNTDOWN";
        case PHASE_COLLECT: return "COLLECTING";
        case PHASE_POSITION: return "POSITION_FOR_REFERENCE";
        case PHASE_REFERENCE: return "HOLD_STILL";
        case PHASE_RUN: return "COMPARISON";
        default: return "FAILED";
    }
}

static void fail_calibration(const char *reason)
{
    calibration.valid = 0;
    phase = PHASE_FAILED;

    tm_printf(
        (UB *)"\n[CAL FAILED] %s\n"
               "Save this output. Correct the cause before resetting.\n"
               "Magnetic yaw correction remains OFF.\n\n",
        (UB *)reason
    );
}

/* ---------- Existing gyro service ---------- */

static void display_events(void)
{
    for (INT i = 0; i < 8; i++) {
        imu_result_t event;
        INT received = imu_service_poll(&event);

        if (received == 0) {
            break;
        }

        if (received < 0) {
            tm_printf(
                (UB *)"[EVENT] Read error=%d\n",
                received
            );
            break;
        }

        if (event.event == IMU_EVT_READY) {
            tm_printf(
                (UB *)"[GYRO] Startup calibration passed.\n"
            );
        } else if (event.event == IMU_EVT_REFERENCE_SET) {
            tm_printf(
                (UB *)"[GYRO] Reference changed; request=%u\n",
                event.request_id
            );
        } else if (event.event == IMU_EVT_FAULT) {
            tm_printf(
                (UB *)"[GYRO FAULT] error=%d stage=%u "
                       "sample=%u/200 request=%u\n",
                event.error,
                imu_mtk3_init_stage(),
                imu_mtk3_calibration_count(),
                event.request_id
            );
        }
    }
}

static INT gyro_is_still(const imu_sample_t *gyro)
{
    /*
     * This is an additional check only.
     * The user must physically keep the assembly stationary.
     */
    return
        fabsf(gyro->gyro_x_dps) <= REFERENCE_RATE_DPS &&
        fabsf(gyro->gyro_y_dps) <= REFERENCE_RATE_DPS &&
        fabsf(gyro->gyro_z_dps) <= REFERENCE_RATE_DPS;
}

/* ---------- Planar magnetic calibration ---------- */

static INT finish_calibration(void)
{
    if (cal_count < MIN_CAL_SAMPLES) {
        fail_calibration("Too few magnetic samples.");
        return 0;
    }

    float min_x = cal_samples[0].x;
    float max_x = min_x;
    float min_y = cal_samples[0].y;
    float max_y = min_y;
    float min_z = cal_samples[0].z;
    float max_z = min_z;
    float sum_z = 0.0f;

    for (UW i = 0; i < cal_count; i++) {
        float x = cal_samples[i].x;
        float y = cal_samples[i].y;
        float z = cal_samples[i].z;

        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;

        sum_z += z;
    }

    float span_x = max_x - min_x;
    float span_y = max_y - min_y;
    float span_z = max_z - min_z;

    tm_printf((UB *)"\n[CAL RANGE] ");
    print_value("Xspan", span_x);
    print_value("Yspan", span_y);
    print_value("Zspan", span_z);
    tm_printf((UB *)"\n");

    if (span_x < MIN_AXIS_SPAN || span_y < MIN_AXIS_SPAN) {
        fail_calibration(
            "Insufficient horizontal field variation."
        );
        return 0;
    }

    calibration.bias_x = 0.5f * (max_x + min_x);
    calibration.bias_y = 0.5f * (max_y + min_y);
    calibration.radius_x = 0.5f * span_x;
    calibration.radius_y = 0.5f * span_y;
    calibration.mean_z = sum_z / (float)cal_count;

    float small_radius = fminf(
        calibration.radius_x,
        calibration.radius_y
    );

    float large_radius = fmaxf(
        calibration.radius_x,
        calibration.radius_y
    );

    if (large_radius / small_radius > MAX_AXIS_RATIO) {
        fail_calibration(
            "Axis ranges differ too much for this simple calibration."
        );
        return 0;
    }

    UW sector_count[CAL_SECTORS] = {0};
    float squared_error_sum = 0.0f;
    float maximum_error = 0.0f;

    for (UW i = 0; i < cal_count; i++) {
        float x =
            ((float)cal_samples[i].x - calibration.bias_x) /
            calibration.radius_x;

        float y =
            ((float)cal_samples[i].y - calibration.bias_y) /
            calibration.radius_y;

        float radius = sqrtf(x * x + y * y);
        float error = fabsf(radius - 1.0f);

        squared_error_sum += error * error;

        if (error > maximum_error) {
            maximum_error = error;
        }

        float angle = atan2f(y, x) * RAD_TO_DEG;

        if (angle < 0.0f) {
            angle += 360.0f;
        }

        INT sector = (INT)(
            angle * (float)CAL_SECTORS / 360.0f
        );

        if (sector >= CAL_SECTORS) {
            sector = CAL_SECTORS - 1;
        }

        if (sector < 0) {
            sector = 0;
        }

        sector_count[sector]++;
    }

    UW covered = 0;

    for (INT i = 0; i < CAL_SECTORS; i++) {
        if (sector_count[i] >= MIN_SECTOR_SAMPLES) {
            covered++;
        }
    }

    float rms = sqrtf(
        squared_error_sum / (float)cal_count
    );

    tm_printf(
        (UB *)"[CAL QUALITY] Samples=%u | Sectors=%u/%u | ",
        cal_count,
        covered,
        (UINT)CAL_SECTORS
    );

    print_value("RadiusRMS_pct", rms * 100.0f);
    print_value("RadiusMax_pct", maximum_error * 100.0f);
    print_value("Zspan_pct", span_z * 100.0f / small_radius);
    tm_printf((UB *)"\n");

    if (covered != CAL_SECTORS) {
        fail_calibration(
            "Incomplete coverage. Every direction needs samples."
        );
        return 0;
    }

    if (rms > MAX_RADIUS_RMS ||
        maximum_error > MAX_RADIUS_ERROR) {
        fail_calibration(
            "Readings do not fit this planar offset/scale model."
        );
        return 0;
    }

    if (span_z > MAX_Z_SPAN_RATIO * small_radius) {
        fail_calibration(
            "Z changed too much: check tilt, mounting and magnetic surroundings."
        );
        return 0;
    }

    calibration.z_tolerance = fmaxf(
        50.0f,
        0.20f * small_radius
    );

    calibration.valid = 1;

    tm_printf((UB *)"[CAL COEFFICIENTS] ");
    print_value("BiasX", calibration.bias_x);
    print_value("BiasY", calibration.bias_y);
    print_value("RadiusX", calibration.radius_x);
    print_value("RadiusY", calibration.radius_y);
    print_value("MeanZ", calibration.mean_z);
    tm_printf(
        (UB *)"\n[CAL PASS] Planar coverage/consistency checks passed.\n"
               "This is not a heading-accuracy certification.\n"
               "Calibration is in RAM only; reset clears it.\n"
    );

    return 1;
}

/*
 * Produce a provisional CCW-positive magnetic angle.
 * It is not true north; the comparison uses a local reference.
 *
 * For body X forward and Y left:
 * heading = atan2(-body magnetic Y, body magnetic X).
 */
static INT magnetic_angle(
    const mag_sample_t *sample,
    UW now,
    float *angle
)
{
    if (!calibration.valid ||
        !sample->valid ||
        sample->status != MAG_STATUS_OK ||
        (UW)(now - sample->timestamp_ms) > MAG_FRESH_MS) {
        return 0;
    }

    float x = MAG_BODY_X_SIGN *
        ((float)sample->raw_x - calibration.bias_x) /
        calibration.radius_x;

    float y = MAG_BODY_Y_SIGN *
        ((float)sample->raw_y - calibration.bias_y) /
        calibration.radius_y;

    float radius = sqrtf(x * x + y * y);

    if (!isfinite(radius) ||
        radius < 0.75f ||
        radius > 1.25f ||
        fabsf((float)sample->raw_z - calibration.mean_z) >
            calibration.z_tolerance) {
        return 0;
    }

    *angle = wrap_degrees(
        atan2f(-y, x) * RAD_TO_DEG
    );

    return isfinite(*angle);
}

/* ---------- Establish a common comparison reference ---------- */

static void clear_reference_collection(void)
{
    reference_count = 0;
    reference_sin_sum = 0.0f;
    reference_cos_sum = 0.0f;
    reference_gyro_sum = 0.0f;
    reference_last_ms = 0;
}

static void collect_reference(
    const mag_sample_t *mag,
    const imu_sample_t *gyro,
    INT gyro_ok,
    UW now
)
{
    float angle;

    if (!gyro_ok ||
        !gyro_is_still(gyro) ||
        !magnetic_angle(mag, now, &angle)) {
        clear_reference_collection();
        return;
    }

    if (reference_count > 0 &&
        ((UW)(now - reference_last_ms) > MAG_FRESH_MS ||
         gyro->reference_id != reference_id ||
         fabsf(wrap_degrees(angle - reference_first_mag)) >
             REFERENCE_ANGLE_LIMIT ||
         fabsf(gyro->yaw_deg - reference_first_gyro) >
             REFERENCE_GYRO_LIMIT)) {
        clear_reference_collection();
    }

    if (reference_count == 0) {
        reference_first_mag = angle;
        reference_first_gyro = gyro->yaw_deg;
        reference_id = gyro->reference_id;
    }

    reference_last_ms = now;
    reference_sin_sum += sinf(angle * DEG_TO_RAD);
    reference_cos_sum += cosf(angle * DEG_TO_RAD);
    reference_gyro_sum += gyro->yaw_deg;
    reference_count++;

    if (reference_count < REFERENCE_SAMPLES) {
        return;
    }

    magnetic_reference = atan2f(
        reference_sin_sum,
        reference_cos_sum
    ) * RAD_TO_DEG;

    gyro_reference =
        reference_gyro_sum / (float)reference_count;

    phase = PHASE_RUN;

    tm_printf(
        (UB *)"\n[REFERENCE READY]\n"
               "Both displayed relative angles now refer to this position.\n"
               "Test clockwise 90 degrees, then return.\n"
               "Keep the assembly level and in the same area.\n"
               "Magnetic yaw correction remains OFF.\n\n"
    );
}

/* ---------- Calibration workflow ---------- */

static void advance_phase(
    UW now,
    const imu_sample_t *gyro,
    INT gyro_ok
)
{
    UW elapsed = now - phase_start;

    if (phase == PHASE_WAIT_GYRO) {
        UW stage = imu_mtk3_init_stage();

        if (stage >= 51U) {
            fail_calibration(
                "Gyro startup failed. Save the GYRO CAL/FAULT messages."
            );
        } else if (gyro_ok && gyro_is_still(gyro)) {
            phase = PHASE_COUNTDOWN;
            phase_start = now;

            tm_printf(
                (UB *)"\n[PREPARE] Magnetic calibration starts in 10 seconds.\n"
                       "Keep both boards level and rigidly joined.\n"
                       "At GO, rotate slowly through all directions.\n"
            );
        }

        return;
    }

    if (phase == PHASE_COUNTDOWN &&
        elapsed >= COUNTDOWN_MS) {
        cal_count = 0;
        calibration.valid = 0;
        last_mag_ms = now;
        phase = PHASE_COLLECT;
        phase_start = now;

        tm_printf(
            (UB *)"\n[GO] Collecting for 60 seconds.\n"
                   "Make at least two slow complete level rotations.\n"
                   "You may reverse direction to avoid twisting wires.\n"
                   "Do not tilt or move to another location.\n"
        );
        return;
    }

    if (phase == PHASE_COLLECT) {
        if ((UW)(now - last_mag_ms) > MAG_FRESH_MS) {
            fail_calibration(
                "Magnetic samples stopped arriving during collection."
            );
        } else if (elapsed >= COLLECTION_MS) {
            if (finish_calibration()) {
                phase = PHASE_POSITION;
                phase_start = imu_mtk3_now_ms();

                tm_printf(
                    (UB *)"\n[STOP] You have 15 seconds to return "
                           "to your chosen starting direction.\n"
                           "Then keep the assembly completely still.\n"
                );
            }
        }

        return;
    }

    if (phase == PHASE_POSITION &&
        elapsed >= POSITIONING_MS) {
        clear_reference_collection();
        phase = PHASE_REFERENCE;
        phase_start = now;

        tm_printf(
            (UB *)"\n[HOLD STILL] Collecting a common heading reference.\n"
        );
        return;
    }

    if (phase == PHASE_REFERENCE &&
        elapsed >= REFERENCE_TIMEOUT_MS) {
        fail_calibration(
            "Could not obtain a stable, valid gyro/magnetic reference."
        );
    }
}

static void process_magnetic_sample(
    const mag_sample_t *sample,
    const imu_sample_t *gyro,
    INT gyro_ok,
    UW now
)
{
    if (phase == PHASE_COLLECT) {
        if (cal_count >= MAX_CAL_SAMPLES) {
            fail_calibration("Calibration sample buffer is full.");
            return;
        }

        cal_samples[cal_count].x = sample->raw_x;
        cal_samples[cal_count].y = sample->raw_y;
        cal_samples[cal_count].z = sample->raw_z;
        cal_count++;
    } else if (phase == PHASE_REFERENCE) {
        collect_reference(sample, gyro, gyro_ok, now);
    }
}

/* ---------- Diagnostics ---------- */

static void display_read_rate(
    const imu_sample_t *gyro,
    INT gyro_ok
)
{
    static INT started = 0;
    static UW first_sequence = 0;
    static UW first_timestamp = 0;
    static UW first_reference = 0;

    if (!gyro_ok) {
        started = 0;
        return;
    }

    if (!started || gyro->reference_id != first_reference) {
        first_sequence = gyro->sequence;
        first_timestamp = gyro->timestamp_ms;
        first_reference = gyro->reference_id;
        started = 1;
        return;
    }

    UW elapsed = gyro->timestamp_ms - first_timestamp;

    if (elapsed < 5000U) {
        return;
    }

    UW readings = gyro->sequence - first_sequence;

    tm_printf((UB *)"[TIMING] ");
    print_value(
        "GyroReadsPerSecond",
        (float)readings * 1000.0f / (float)elapsed
    );

    tm_printf(
        (UB *)"| LatestGap=%ums | sensor configured at 100Hz\n",
        gyro->interval_ms
    );

    first_sequence = gyro->sequence;
    first_timestamp = gyro->timestamp_ms;
}

static void display_status(
    UW now,
    const imu_sample_t *gyro,
    ER gyro_error,
    INT gyro_ok
)
{
    tm_printf(
        (UB *)"[STATE] %s",
        (UB *)phase_name()
    );

    UW duration = 0;

    if (phase == PHASE_COUNTDOWN) duration = COUNTDOWN_MS;
    if (phase == PHASE_COLLECT) duration = COLLECTION_MS;
    if (phase == PHASE_POSITION) duration = POSITIONING_MS;

    if (duration > 0) {
        UW elapsed = now - phase_start;
        UW remaining = elapsed < duration
            ? (duration - elapsed + 999U) / 1000U
            : 0;

        tm_printf((UB *)" | remaining=%us", remaining);
    }

    if (phase == PHASE_COLLECT) {
        tm_printf((UB *)" | samples=%u", cal_count);
    }

    if (phase == PHASE_REFERENCE) {
        tm_printf(
            (UB *)" | stable_samples=%u/%u",
            reference_count,
            REFERENCE_SAMPLES
        );
    }

    tm_printf((UB *)"\n");

    if (gyro_ok) {
        tm_printf((UB *)"[GYRO] ");
        print_value("Yaw", gyro->yaw_deg);
        print_value("Rate", gyro->yaw_rate_dps);

        tm_printf(
            (UB *)"| Gap=%ums | t=%ums\n",
            gyro->interval_ms,
            gyro->timestamp_ms
        );
    } else {
        tm_printf(
            (UB *)"[GYRO] UNAVAILABLE error=%d status=%d "
                   "heading_valid=%d stage=%u sample=%u/200\n",
            gyro_error,
            gyro->status,
            gyro->heading_valid,
            imu_mtk3_init_stage(),
            imu_mtk3_calibration_count()
        );
    }

    if (have_mag) {
        tm_printf(
            (UB *)"[MAG RAW] X=%d Y=%d Z=%d "
                   "| age=%ums | last_result=%d\n",
            latest_mag.raw_x,
            latest_mag.raw_y,
            latest_mag.raw_z,
            (UW)(now - latest_mag.timestamp_ms),
            mag_read_result
        );
    }

    if (phase == PHASE_RUN) {
        float angle;
        INT mag_ok = have_mag &&
            magnetic_angle(&latest_mag, now, &angle);

        INT reference_matches =
            gyro_ok && gyro->reference_id == reference_id;

        tm_printf((UB *)"[COMPARE] ");

        if (reference_matches) {
            print_value(
                "GyroRel",
                gyro->yaw_deg - gyro_reference
            );
        } else {
            tm_printf((UB *)"GyroRel=INVALID ");
        }

        if (mag_ok) {
            float magnetic_relative = wrap_degrees(
                angle - magnetic_reference
            );

            print_value("MagRel", magnetic_relative);

            if (reference_matches) {
                print_value(
                    "MagMinusGyro",
                    wrap_degrees(
                        magnetic_relative -
                        (gyro->yaw_deg - gyro_reference)
                    )
                );
            }

            tm_printf(
                (UB *)"| MAG_CALIBRATED_2D | FUSION_OFF\n"
            );
        } else {
            tm_printf(
                (UB *)"MagRel=REJECTED "
                       "| stale, field mismatch or tilt "
                       "| FUSION_OFF\n"
            );
        }
    }

    display_read_rate(gyro, gyro_ok);
}

/* ---------- Monitor owner task ---------- */

static void monitor_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    /*
     * Wait for USB before starting the visible calibration workflow.
     * Gyro startup is already running on its own task.
     */
    while (tm_usb_state() < 2U) {
        (void)tk_dly_tsk(100);
    }

    tm_printf(
        (UB *)"\n========================================\n"
               " MPU-6500 + LSM303D PLANAR CALIBRATION\n"
               "========================================\n"
               "MPU: Grove 4, GP6 SDA / GP7 SCL\n"
               "MAG: Grove 5, GP8 SDA / GP9 SCL\n"
               "Keep still until gyro startup passes.\n"
               "Watch for the calibration countdown and GO.\n"
               "Magnetic correction of gyro yaw is OFF.\n"
               "Reset clears this calibration.\n\n"
    );

    ER mag_error = mag_mtk3_init();
    UB identity[3] = {0};

    mag_mtk3_identity(identity);

    tm_printf(
        (UB *)"[MAG] WHO_AM_I=%02X %02X %02X "
               "| expected=49 49 49\n",
        (UINT)identity[0],
        (UINT)identity[1],
        (UINT)identity[2]
    );

    INT mag_online = mag_error == E_OK;

    if (!mag_online) {
        tm_printf(
            (UB *)"[MAG FAULT] error=%d step=%s\n",
            mag_error,
            (UB *)mag_mtk3_init_step()
        );

        fail_calibration("LSM303D initialisation failed.");
    } else {
        tm_printf((UB *)"[MAG] LSM303D ready.\n");
    }

    UW now = imu_mtk3_now_ms();
    UW next_mag = now;
    UW next_display = now;
    phase_start = now;

    while (1) {
        display_events();

        imu_sample_t gyro = {0};
        ER gyro_error = imu_service_latest(&gyro);

        INT gyro_ok =
            gyro_error == E_OK &&
            gyro.rate_valid &&
            gyro.heading_valid;

        now = imu_mtk3_now_ms();
        advance_phase(now, &gyro, gyro_ok);

        if (mag_online && (W)(now - next_mag) >= 0) {
            mag_sample_t sample = {0};
            mag_read_result = mag_mtk3_read(&sample);
            now = imu_mtk3_now_ms();

            /*
             * Obtain a fresh gyro snapshot after the magnetic I2C read.
             * These remain host-timestamped, asynchronous sensors.
             */
            gyro_error = imu_service_latest(&gyro);
            gyro_ok =
                gyro_error == E_OK &&
                gyro.rate_valid &&
                gyro.heading_valid;

            advance_phase(now, &gyro, gyro_ok);

            if (mag_read_result == 1 && sample.valid) {
                latest_mag = sample;
                have_mag = 1;
                last_mag_ms = sample.timestamp_ms;

                process_magnetic_sample(
                    &sample,
                    &gyro,
                    gyro_ok,
                    now
                );
            } else {
                if (phase == PHASE_REFERENCE) {
                    clear_reference_collection();
                }

                if (mag_read_result < 0) {
                    tm_printf(
                        (UB *)"[MAG READ ERROR] status=%d error=%d\n",
                        sample.status,
                        sample.error
                    );

                    if (phase == PHASE_COLLECT) {
                        fail_calibration(
                            "Magnetic I2C/data error during calibration."
                        );
                    }
                }
            }

            next_mag = imu_mtk3_now_ms() + MAG_POLL_MS;
        }

        now = imu_mtk3_now_ms();

        if ((W)(now - next_display) >= 0) {
            gyro_error = imu_service_latest(&gyro);
            gyro_ok =
                gyro_error == E_OK &&
                gyro.rate_valid &&
                gyro.heading_valid;

            display_status(
                now,
                &gyro,
                gyro_error,
                gyro_ok
            );

            next_display =
                imu_mtk3_now_ms() + DISPLAY_MS;
        }

        (void)tk_dly_tsk(10);
    }
}

/* ---------- Application entry ---------- */

EXPORT INT usermain(void)
{
    T_CMTX mutex = {0};
    mutex.mtxatr = TA_TFIFO | TA_INHERIT;

    i2c_shared_mutex = tk_cre_mtx(&mutex);

    if (i2c_shared_mutex <= 0) {
        tm_printf(
            (UB *)"[STARTUP] Could not create I2C mutex.\n"
        );
        return 1;
    }

    ER er = imu_service_start(i2c_shared_mutex);

    if (er != E_OK) {
        tm_printf(
            (UB *)"[STARTUP] IMU service error=%d\n",
            er
        );

        (void)tk_del_mtx(i2c_shared_mutex);
        i2c_shared_mutex = 0;
        return 2;
    }

    T_CTSK task = {0};
    task.task = monitor_task;
    task.itskpri = 7;
    task.stksz = MONITOR_STACK;
    task.tskatr = TA_HLNG | TA_RNG3;

#if TK_SUPPORT_SMP
    task.tskatr |= TA_ASSPRC;
    task.assprc = TP_PRC2;
#endif

    ID monitor_id = tk_cre_tsk(&task);

    if (monitor_id <= 0) {
        tm_printf(
            (UB *)"[STARTUP] Could not create monitor task.\n"
        );
        return 3;
    }

    er = tk_sta_tsk(monitor_id, 0);

    if (er != E_OK) {
        tm_printf(
            (UB *)"[STARTUP] Monitor start error=%d\n",
            er
        );

        (void)tk_del_tsk(monitor_id);
        return 3;
    }

    while (1) {
        (void)tk_slp_tsk(TMO_FEVR);
    }

    return 0;
}