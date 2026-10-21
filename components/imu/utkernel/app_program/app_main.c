
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "usb_console_compat.h"
#include "imu_service.h"

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#define CURRENT_CORE() tk_get_prc()
#else
#define CURRENT_CORE() 1
#endif

#define MONITOR_STACK 4096
#define IMU_SHOW_RAW 0

static ID i2c_shared_mutex = 0;

/* Print two decimal places without %f. */
static void print_angle(
    const char *name,
    float value
) {
    INT scaled = (INT)(value * 100.0f);

    INT whole = scaled / 100;
    INT fraction = scaled % 100;

    if (fraction < 0) {
        fraction = -fraction;
    }

    if (scaled < 0 && whole == 0) {
        tm_printf(
            (UB *)"%s=-0.%02d ",
            (UB *)name,
            fraction
        );
    } else {
        tm_printf(
            (UB *)"%s=%d.%02d ",
            (UB *)name,
            whole,
            fraction
        );
    }
}

/* -------------------------------------
 * Monitor / simulated Coordinator Task
 * ------------------------------------- */

static void monitor_task(
    INT stacd,
    void *exinf
) {
    (void)stacd;
    (void)exinf;

    /* Wait for USB serial connection. */
    for (
        INT i = 0;
        i < 200 && tm_usb_state() < 2;
        i++
    ) {
        (void)tk_dly_tsk(100);
    }

    (void)tk_dly_tsk(300);

    
    tm_printf(
        (UB *)"\n========================================\n"
            "  AAI17 | MPU-6500 ORIENTATION MONITOR\n"
            "========================================\n"
            "Tolerance: +/- 5 degrees\n\n"
    );

#if TK_SUPPORT_SMP
    tm_printf(
        (UB *)"IMU=prc1 (core0), Monitor=prc%d\n",
        CURRENT_CORE()
    );
#else
    tm_printf(
        (UB *)"WARNING: Not an SMP build!\n"
    );
#endif

    tm_printf(
        (UB *)"Keep IMU still during calibration.\n"
    );

    INT ready = 0;
    INT has_reference = 0;
    INT verification_pending = 0;

    UW next_snapshot = 0;
    UW next_verify = 0;
    UW verification_sent_at = 0;

    UW request_id = 1;
    UW expected_request = 0;

    float reference_pitch = 0;
    float reference_roll = 0;

    while (1) {

        /* -----------------------------
         * 1. Receive IMU results
         * ----------------------------- */

        imu_result_t result;
        INT received;

        while (
            (received =
                imu_service_poll(&result)) == 1
        ) {

            UW now = imu_mtk3_now_ms();

            if (result.event ==
                IMU_EVT_READY) {

                ready = 1;

                tm_printf(
                    (UB *)"[IMU] MPU-6500 ready on prc%d\n",
                    result.producer_core
                );
            }

            
            else if (result.event == IMU_EVT_FAULT) {

                UW gap = result.sample.interval_ms;

                /* Fault during orientation verification */
                if (verification_pending &&
                    result.request_id == expected_request) {

                    tm_printf(
                        (UB *)"[CHECK #%u] FAULT - "
                            "VERIFICATION BLOCKED\n"
                            "           Sensor data unreliable. "
                            "Orientation NOT approved.\n",
                        result.request_id
                    );

                    verification_pending = 0;
                }

                /* Invalid requested snapshot */
                else if (result.request_id != 0) {

                    tm_printf(
                        (UB *)"[IMU] WARNING - "
                            "Snapshot #%u unavailable "
                            "(invalid reading)\n",
                        result.request_id
                    );
                }

                /* Sampling deadline exceeded */
                else if (gap > 50U) {

                    tm_printf(
                        (UB *)"[IMU] WARNING - "
                            "Sampling delay: %ums "
                            "(limit 50ms)\n",
                        gap
                    );
                }

                /* Other invalid sensor readings */
                else {

                    tm_printf(
                        (UB *)"[IMU] WARNING - "
                            "Invalid sensor reading\n"
                            "      Possible rapid movement "
                            "or I2C communication issue.\n"
                    );
                }
            }

            else if (
                result.event == IMU_EVT_SAMPLE
            ) {

                /*
                 * For this standalone demo,
                 * save the first stationary pose
                 * as the reference.
                 *
                 * The real Coordinator will
                 * provide its own target pose.
                 */
                if (result.sample.valid &&
                    result.sample.stationary &&
                    !has_reference) {

                    reference_pitch =
                        result.sample.pitch_deg;

                    reference_roll =
                        result.sample.roll_deg;

                    has_reference = 1;
                    
                    tm_printf(
                        (UB *)"\n[TARGET] "
                    );

                    print_angle(
                        "Pitch",
                        reference_pitch
                    );

                    print_angle(
                        "Roll",
                        reference_roll
                    );

                    tm_printf(
                        (UB *)"(reference saved)\n"
                    );
                }
                                    
                    /* Clean orientation output */
                    tm_printf(
                        (UB *)"\n[DATA] "
                    );

                    print_angle(
                        "Pitch",
                        result.sample.pitch_deg
                    );

                    print_angle(
                        "Roll",
                        result.sample.roll_deg
                    );

                    tm_printf(
                        (UB *)"\n       %s | %s | %ums | Time=%ums\n",
                        result.sample.stationary
                            ? (UB *)"STILL "
                            : (UB *)"MOVING",
                        result.sample.valid
                            ? (UB *)"Valid"
                            : (UB *)"INVALID",
                        result.sample.interval_ms,
                        result.sample.timestamp_ms
                    );

                    /* Optional raw debugging output */
                    #if IMU_SHOW_RAW
                    tm_printf(
                        (UB *)"       RAW: AX=%d AY=%d AZ=%d "
                            "GX=%d GY=%d GZ=%d\n",
                        result.sample.raw_ax,
                        result.sample.raw_ay,
                        result.sample.raw_az,
                        result.sample.raw_gx,
                        result.sample.raw_gy,
                        result.sample.raw_gz
                    );
                    #endif
            }

            else if (
                result.event == IMU_EVT_VERIFIED ||
                result.event ==
                    IMU_EVT_OUT_OF_TOLERANCE
            ) {

                if (
                    result.request_id ==
                        expected_request &&
                    verification_pending
                ) {

                    INT fresh =
                        result.sample.valid &&
                        (UW)(
                            now -
                            result.sample.timestamp_ms
                        ) <= 50U;

                    const char *status;

                    if (
                        result.event ==
                            IMU_EVT_VERIFIED &&
                        fresh
                    ) {
                        status = "PASS - WITHIN TOLERANCE";
                    }

                    else if (
                        result.event ==
                            IMU_EVT_OUT_OF_TOLERANCE
                    ) {
                        status = "FAIL - OUT OF TOLERANCE";
                    }

                    else {
                        status = "REJECTED (STALE)";
                    }

                    
                    tm_printf(
                        (UB *)"\n[CHECK #%u] %s | %s\n",
                        result.request_id,
                        (UB *)status,
                        fresh ? (UB *)"FRESH" : (UB *)"STALE"
                    );

                    verification_pending = 0;
                }
            }

            else if (
                result.event == IMU_EVT_CANCELLED
            ) {
                verification_pending = 0;
            }
        }

        if (received < 0) {
            tm_printf(
                (UB *)"[MONITOR] Receive error %d\n",
                received
            );
        }

        /* -----------------------------
         * 2. Request readings
         * ----------------------------- */

        UW now = imu_mtk3_now_ms();

        if (
            ready &&
            (W)(now - next_snapshot) >= 0
        ) {

            (void)imu_service_snapshot(
                ++request_id
            );

            next_snapshot = now + 500U;
        }

        /* -----------------------------
         * 3. Request verification
         * ----------------------------- */

        if (
            ready &&
            has_reference &&
            !verification_pending &&
            (W)(now - next_verify) >= 0
        ) {

            expected_request =
                ++request_id;

            if (
                imu_service_verify(
                    expected_request,
                    reference_pitch,
                    reference_roll
                ) == E_OK
            ) {
                verification_pending = 1;
                verification_sent_at = now;
            }

            next_verify = now + 2000U;
        }

        /* -----------------------------
         * 4. Missing-result timeout
         * ----------------------------- */

        if (
            verification_pending &&
            (UW)(
                now - verification_sent_at
            ) > 1300U
        ) {

            tm_printf(
                (UB *)"[VERIFY #%u] "
                       "NO RESULT => REJECTED\n",
                expected_request
            );

            verification_pending = 0;
        }

        (void)tk_dly_tsk(10);
    }
}

/* -------------------------------------
 * µT-Kernel application entry
 * ------------------------------------- */

EXPORT INT usermain(void) {

    /* Create shared I2C mutex. */
    T_CMTX mutex = {0};

    mutex.mtxatr =
        TA_TFIFO | TA_INHERIT;

    i2c_shared_mutex =
        tk_cre_mtx(&mutex);

    if (i2c_shared_mutex <= 0) {
        return 1;
    }

    /* Start IMU Task. */
    if (
        imu_service_start(
            i2c_shared_mutex
        ) != E_OK
    ) {
        return 2;
    }

    /* Create monitoring task. */
    T_CTSK task = {0};

    task.task = monitor_task;
    task.itskpri = 7;
    task.stksz = MONITOR_STACK;

#if TK_SUPPORT_SMP
    task.tskatr =
        TA_HLNG | TA_RNG3 | TA_ASSPRC;

    task.assprc = TP_PRC2;
#else
    task.tskatr =
        TA_HLNG | TA_RNG3;
#endif

    ID id = tk_cre_tsk(&task);

    if (
        id <= 0 ||
        tk_sta_tsk(id, 0) != E_OK
    ) {
        return 3;
    }

    /* Kernel keeps running. */
    (void)tk_slp_tsk(TMO_FEVR);

    return 0;
}
