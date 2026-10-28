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

/* -------------------------------------
 * Console formatting
 * ------------------------------------- */

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

static const char *event_name(UW event) {
    switch (event) {
        case IMU_EVT_READY:
            return "READY";

        case IMU_EVT_REFERENCE_SET:
            return "REFERENCE_SET";

        case IMU_EVT_FAULT:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}

/* -------------------------------------
 * Display the latest reading
 * ------------------------------------- */

static void display_sample(void) {
    imu_sample_t sample;

    ER er = imu_service_latest(&sample);

    if (er != E_OK) {
        tm_printf(
            (UB *)"[DATA] Reading unavailable: %s "
                   "| error=%d | calibrated=%d\n",
            (UB *)status_name(sample.status),
            er,
            sample.calibrated
        );
        return;
    }

    UW now = imu_mtk3_now_ms();

    tm_printf(
        (UB *)"[DATA] Seq=%u Ref=%u ",
        sample.sequence,
        sample.reference_id
    );

    print_value("Rate(dps)", sample.yaw_rate_dps);

    if (sample.heading_valid) {
        print_value("Yaw(deg)", sample.yaw_deg);
        print_value("Wrapped(deg)", sample.yaw_wrapped_deg);
    } else {
        tm_printf(
            (UB *)"Heading unavailable "
                   "(new stopped-car reference required) "
        );
    }

    tm_printf(
        (UB *)"| interval=%ums | age=%ums\n",
        sample.interval_ms,
        (UW)(now - sample.timestamp_ms)
    );

#if IMU_SHOW_RAW
    tm_printf(
        (UB *)"       RAW GX=%d GY=%d GZ=%d\n",
        sample.raw_gx,
        sample.raw_gy,
        sample.raw_gz
    );
#endif
}

/* -------------------------------------
 * Diagnostic monitor task
 * ------------------------------------- */

static void monitor_task(INT stacd, void *exinf) {
    (void)stacd;
    (void)exinf;

    /*
     * Allow time for a USB serial connection.
     * The IMU service continues running during this wait.
     * UART builds use the compatibility helper's ready state.
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
         * This monitor is currently the sole event consumer.
         * During car integration, the coordinator takes over
         * event handling.
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

            tm_printf(
                (UB *)"[EVENT] %s | request=%u "
                       "| error=%d | producer=%d "
                       "| dropped=%u\n",
                (UB *)event_name(result.event),
                result.request_id,
                result.error,
                result.producer_core,
                result.dropped_events
            );
        }

        UW now = imu_mtk3_now_ms();

        if ((W)(now - next_print) >= 0) {
            display_sample();
            next_print = imu_mtk3_now_ms() + MONITOR_PERIOD_MS;
        }

        (void)tk_dly_tsk(10);
    }
}

/* -------------------------------------
 * µT-Kernel application entry
 * ------------------------------------- */

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
     * Starts asynchronous gyro calibration.
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