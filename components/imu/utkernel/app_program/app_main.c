#include <stddef.h>

/* Use the C library's size_t definition. */
#define PROHIBIT_DEF_SIZE_T

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <math.h>

#include "imu_service.h"
#include "usb_console_compat.h"

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#define CURRENT_CORE() tk_get_prc()
#else
#define CURRENT_CORE() 1
#endif

#define MONITOR_STACK      4096
#define MONITOR_PERIOD_MS  500U
#define IMU_SHOW_RAW       0

static ID i2c_shared_mutex = 0;

/* Console formatting */

/* Print two decimal places without requiring printf %f. */
static void print_value(const char *name, float value) {
    if (!isfinite(value) || fabsf(value) > 1000000.0f) {
        tm_printf(
            (UB *)"%s=unavailable ",
            (UB *)name
        );
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

static const char *status_name(imu_status_t status) {
    switch (status) {
        case IMU_STATUS_OK:
            return "OK";

        case IMU_STATUS_NOT_READY:
            return "NOT_READY";

        case IMU_STATUS_IO:
            return "I2C_ERROR";

        case IMU_STATUS_NO_DATA:
            return "NO_NEW_DATA";

        case IMU_STATUS_TIMING:
            return "TIMING_ERROR";

        case IMU_STATUS_SATURATED:
            return "GYRO_SATURATED";

        default:
            return "UNKNOWN";
    }
}

/*
 * Stages 1-6 identify the current or last startup operation.
 * Stages 51-55 explicitly identify calibration failures.
 */
static const char *stage_name(UW stage) {
    switch (stage) {
        case 0:
            return "Not started";

        case 1:
            return "Open I2C device";

        case 2:
            return "Read/check WHO_AM_I";

        case 3:
            return "Reset sensor";

        case 4:
            return "Write/verify sensor configuration";

        case 5:
            return "Calibration running";

        case 6:
            return "Establish time reference";

        case 7:
            return "Startup completed";

        case 51:
            return "Status-register access failed";

        case 52:
            return "Fresh gyro data did not arrive";

        case 53:
            return "Gyro-register access failed";

        case 54:
            return "Calibration rate exceeded limit";

        case 55:
            return "Calibration readings too unstable";

        default:
            return "Unknown startup stage";
    }
}

/* Display the latest reading */

static void display_sample(void) {
    static INT previous_mode = -1;
    static INT previous_status = -1;
    static UW previous_stage = (UW)-1;
    static UW previous_cal_bucket = (UW)-1;

    imu_sample_t sample = {0};
    ER er = imu_service_latest(&sample);

    if (er != E_OK) {
        UW stage = imu_mtk3_init_stage();
        UW count = imu_mtk3_calibration_count();

        /*
         * Only stage 5 means calibration is running.
         * Calibration failures now use stages 51-55.
         */
        if (!sample.calibrated && stage == 5U) {
            UW bucket = count / 50U;

            if (previous_mode != 0 ||
                bucket != previous_cal_bucket) {
                tm_printf(
                    (UB *)"[IMU] Calibrating: sample %u/200\n",
                    count
                );
            }

            previous_mode = 0;
            previous_cal_bucket = bucket;
            previous_stage = stage;
        } else {
            /*
             * Also print when the stage changes, even if
             * the sample status remains NOT_READY.
             */
            if (previous_mode != 1 ||
                previous_status != (INT)sample.status ||
                previous_stage != stage) {
                tm_printf(
                    (UB *)"[IMU] READING UNAVAILABLE | %s "
                           "| error=%d | stage=%u\n",
                    (UB *)status_name(sample.status),
                    er,
                    stage
                );

                tm_printf(
                    (UB *)"      %s | sample=%u/200\n",
                    (UB *)stage_name(stage),
                    count
                );
            }

            previous_mode = 1;
            previous_status = (INT)sample.status;
            previous_stage = stage;
        }

        return;
    }

    /* Gyro rate exists, but the yaw reference was lost. */
    if (!sample.heading_valid) {
        if (previous_mode != 2) {
            tm_printf(
                (UB *)"\n[IMU] HEADING LOST\n"
                       "      Stop car and establish "
                       "a new reference.\n\n"
            );
        }

        previous_mode = 2;
        return;
    }

    if (previous_mode != 3) {
        tm_printf(
            (UB *)"\n[IMU] HEADING TRACKING OK\n"
        );
    }

    previous_mode = 3;
    previous_status = -1;
    previous_stage = (UW)-1;
    previous_cal_bucket = (UW)-1;

    tm_printf((UB *)"[IMU] ");

    print_value("Yaw", sample.yaw_deg);
    print_value("Rate", sample.yaw_rate_dps);

    tm_printf(
        (UB *)"| Gap=%ums | OK\n",
        sample.interval_ms
    );

#if IMU_SHOW_RAW
    tm_printf(
        (UB *)"      Raw: GX=%d GY=%d GZ=%d "
               "| Seq=%u | Ref=%u\n",
        sample.raw_gx,
        sample.raw_gy,
        sample.raw_gz,
        sample.sequence,
        sample.reference_id
    );
#endif
}

/* Diagnostic monitor task */

static void monitor_task(INT stacd, void *exinf) {
    (void)stacd;
    (void)exinf;

    /*
     * Allow time for a USB serial connection.
     * The IMU service continues running during this wait.
     */
    for (
        INT i = 0;
        i < 200 && tm_usb_state() < 2;
        i++
    ) {
        (void)tk_dly_tsk(100);
    }

    tm_printf(
        (UB *)"\n========================================\n"
               "  MPU-6500 GYROSCOPE / RELATIVE YAW\n"
               "========================================\n"
               "Startup diagnostics: stages 51-55 enabled.\n"
               "Keep the car still from power-on "
               "until calibration completes.\n"
               "Yaw is relative to the startup reference.\n"
               "GY-511 magnetic correction is not added yet.\n\n"
    );

#if TK_SUPPORT_SMP
    tm_printf(
        (UB *)"IMU: processor 1 (core 0) "
               "| Monitor: processor %d\n",
        CURRENT_CORE()
    );
#else
    tm_printf((UB *)"Running in single-core mode.\n");
#endif

    UW next_print = imu_mtk3_now_ms();

    while (1) {
        /*
         * This monitor is the sole event consumer.
         * The coordinator takes over event handling
         * when integrated with the car.
         */
        for (INT i = 0; i < 8; i++) {
            imu_result_t result;

            INT received = imu_service_poll(&result);

            if (received == 0) {
                break;
            }

            if (received < 0) {
                tm_printf(
                    (UB *)"[MONITOR] Event receive error=%d\n",
                    received
                );
                break;
            }

            if (result.event == IMU_EVT_READY) {
                tm_printf(
                    (UB *)"\n[READY] Gyro calibration successful. "
                           "Starting yaw reference: 0 deg\n"
                );
            } else if (result.event == IMU_EVT_REFERENCE_SET) {
                tm_printf(
                    (UB *)"\n[READY] New yaw reference saved "
                           "(request=%u)\n",
                    result.request_id
                );
            } else if (result.event == IMU_EVT_FAULT) {
                tm_printf(
                    (UB *)"\n[FAULT] IMU service error=%d "
                           "| request=%u\n",
                    result.error,
                    result.request_id
                );

                UW stage = imu_mtk3_init_stage();

                /*
                 * Stage 7 is the only successful startup stage.
                 * Failure codes 51-55 must also be displayed.
                 */
                if (stage != 7U) {
                    tm_printf(
                        (UB *)"        Calibration/startup failed "
                               "at stage=%u, sample=%u/200\n",
                        stage,
                        imu_mtk3_calibration_count()
                    );

                    tm_printf(
                        (UB *)"        Reason: %s\n",
                        (UB *)stage_name(stage)
                    );
                }
            }
        }

        UW now = imu_mtk3_now_ms();

        if ((W)(now - next_print) >= 0) {
            display_sample();
            next_print =
                imu_mtk3_now_ms() + MONITOR_PERIOD_MS;
        }

        (void)tk_dly_tsk(10);
    }
}

/* µT-Kernel application entry */

EXPORT INT usermain(void) {
    /* Shared by every task accessing I2C1. */
    T_CMTX mutex = {0};
    mutex.mtxatr = TA_TFIFO | TA_INHERIT;

    i2c_shared_mutex = tk_cre_mtx(&mutex);

    if (i2c_shared_mutex <= 0) {
        tm_printf(
            (UB *)"[STARTUP] Could not create I2C mutex.\n"
        );
        return 1;
    }

    /*
     * Start asynchronous gyro calibration.
     * The car must already be stopped.
     */
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