#include <stddef.h>

#define PROHIBIT_DEF_SIZE_T

#include "imu_service.h"
#include <math.h>
#include <string.h>


#if TK_SUPPORT_SMP
#include <tk/smp.h>
#define IMU_CPU() tk_get_prc()
#else
#define IMU_CPU() 1
#endif

#define IMU_PERIOD_MS       20U
#define IMU_VERIFY_MS       1000U
#define IMU_TOLERANCE_DEG   5.0f
#define IMU_TASK_STACK      4096

/*
 * Verification timeout is currently
 * a proposed value, not a Week 6 NFR.
 */

enum {
    CMD_VERIFY = 1,
    CMD_CANCEL,
    CMD_SNAPSHOT,
    CMD_REINIT
};

typedef struct {
    UW type;
    UW request_id;

    float pitch_deg;
    float roll_deg;
} imu_command_t;

/*
 * Persistent storage for kernel
 * message buffers.
 */
static UW command_store[128];
static UW result_store[256];

static ID command_mbf = 0;
static ID result_mbf = 0;
static ID imu_task_id = 0;
static ID i2c_mutex = 0;

/* -------------------------------------
 * Send IMU result
 * ------------------------------------- */

static ER publish(
    UW event,
    UW request,
    const imu_sample_t *sample
) {
    imu_result_t result = {0};

    result.event = event;
    result.request_id = request;
    result.producer_core = IMU_CPU();

    if (sample != NULL) {
        result.sample = *sample;
    }

    /*
     * Nonblocking send:
     * Do not delay sensor sampling.
     *
     * Coordinator must treat missing
     * responses as failed verification.
     */
    return tk_snd_mbf(
        result_mbf,
        &result,
        sizeof(result),
        TMO_POL
    );
}

/* -------------------------------------
 * IMU Task
 * ------------------------------------- */

static void imu_task(
    INT stacd,
    void *exinf
) {
    (void)stacd;
    (void)exinf;

    imu_sample_t sample = {0};

    UW active_request = 0;
    UW start_ms = 0;
    UW previous_ms = 0;
    UW next_ms = 0;

    INT good_count = 0;

    float desired_pitch = 0;
    float desired_roll = 0;

    /*
     * Initialise and calibrate.
     * Arm must be stationary.
     */
    INT online =
        imu_mtk3_init(i2c_mutex) == E_OK;

    INT fault_announced = !online;

    (void)publish(
        online ? IMU_EVT_READY : IMU_EVT_FAULT,
        0,
        NULL
    );

    next_ms =
        imu_mtk3_now_ms() + IMU_PERIOD_MS;

    while (1) {

        /* -----------------------------
         * 1. Process commands
         * ----------------------------- */

        imu_command_t command;

        /*
         * Process no more than four
         * commands per sampling cycle.
         */
        for (INT i = 0; i < 4; i++) {

            INT n = tk_rcv_mbf(
                command_mbf,
                &command,
                TMO_POL
            );

            if (n == E_TMOUT) {
                break;
            }

            if (n != (INT)sizeof(command)) {
                active_request = 0;
                break;
            }

            /* Cancel verification. */
            if (command.type == CMD_CANCEL) {

                UW previous_request =
                    active_request;

                active_request = 0;
                good_count = 0;

                (void)publish(
                    IMU_EVT_CANCELLED,
                    previous_request,
                    NULL
                );
            }

            /* Reinitialise IMU. */
            else if (
                command.type == CMD_REINIT
            ) {
                active_request = 0;
                good_count = 0;

                memset(
                    &sample,
                    0,
                    sizeof(sample)
                );

                /*
                 * Must be stationary during
                 * recalibration.
                 */
                online =
                    imu_mtk3_init(i2c_mutex) == E_OK;

                fault_announced = !online;

                (void)publish(
                    online
                        ? IMU_EVT_READY
                        : IMU_EVT_FAULT,
                    0,
                    NULL
                );

                next_ms =
                    imu_mtk3_now_ms() +
                    IMU_PERIOD_MS;
            }

            /* Request latest reading. */
            else if (
                command.type == CMD_SNAPSHOT
            ) {
                (void)publish(
                    sample.valid
                        ? IMU_EVT_SAMPLE
                        : IMU_EVT_FAULT,
                    command.request_id,
                    &sample
                );
            }

            /* Start verification. */
            else if (
                command.type == CMD_VERIFY
            ) {
                active_request = 0;
                good_count = 0;
                previous_ms = 0;

                if (!online ||
                    command.request_id == 0 ||
                    !isfinite(command.pitch_deg) ||
                    !isfinite(command.roll_deg)) {

                    (void)publish(
                        IMU_EVT_FAULT,
                        command.request_id,
                        &sample
                    );

                } else {
                    active_request =
                        command.request_id;

                    start_ms =
                        imu_mtk3_now_ms();

                    desired_pitch =
                        command.pitch_deg;

                    desired_roll =
                        command.roll_deg;
                }
            }
        }

        /* -----------------------------
         * 2. Sample MPU-6500
         * ----------------------------- */

        if (online) {

            INT updated =
                imu_mtk3_read(&sample);

            UW now = imu_mtk3_now_ms();

            if (updated) {
                fault_announced = 0;
            }
            else if (
                active_request == 0 &&
                !fault_announced
            ) {
                /*
                 * Report sensor failure
                 * even if no verification
                 * is currently active.
                 */
                (void)publish(
                    IMU_EVT_FAULT,
                    0,
                    &sample
                );

                fault_announced = 1;
            }

            /* -------------------------
             * 3. Orientation verification
             * ------------------------- */

            if (active_request != 0) {

                UW request =
                    active_request;

                /*
                 * Check timeout first.
                 * Never approve after timeout.
                 */
                if ((UW)(now - start_ms) >=
                    IMU_VERIFY_MS) {

                    active_request = 0;

                    (void)publish(
                        IMU_EVT_OUT_OF_TOLERANCE,
                        request,
                        &sample
                    );
                }

                /*
                 * Reject invalid, delayed
                 * or duplicated samples.
                 */
                else if (
                    !updated ||
                    !sample.valid ||
                    sample.timestamp_ms < start_ms ||
                    sample.interval_ms == 0 ||
                    sample.interval_ms > 50U ||
                    (UW)(
                        now - sample.timestamp_ms
                    ) > 50U ||
                    (
                        previous_ms != 0 &&
                        (
                            sample.timestamp_ms
                                <= previous_ms ||
                            (UW)(
                                sample.timestamp_ms -
                                previous_ms
                            ) > 50U
                        )
                    )
                ) {
                    active_request = 0;

                    (void)publish(
                        IMU_EVT_FAULT,
                        request,
                        &sample
                    );
                }

                else {
                    previous_ms =
                        sample.timestamp_ms;

                    float pitch_error =
                        fabsf(
                            sample.pitch_deg -
                            desired_pitch
                        );

                    float roll_error =
                        fabsf(
                            remainderf(
                                sample.roll_deg -
                                desired_roll,
                                360.0f
                            )
                        );

                    INT within_tolerance =
                        pitch_error <=
                            IMU_TOLERANCE_DEG &&
                        roll_error <=
                            IMU_TOLERANCE_DEG &&
                        sample.stationary;

                    /*
                     * NFR4:
                     * Require two consecutive
                     * valid samples.
                     */
                    if (within_tolerance) {
                        good_count++;
                    } else {
                        good_count = 0;
                    }

                    if (good_count >= 2) {

                        active_request = 0;

                        (void)publish(
                            IMU_EVT_VERIFIED,
                            request,
                            &sample
                        );
                    }
                }
            }
        }

        else {
            /*
             * After failed initialisation,
             * no approval is possible.
             */
            active_request = 0;
        }

        /* -----------------------------
         * 4. Periodic task scheduling
         * ----------------------------- */

        UW now = imu_mtk3_now_ms();

        if ((W)(next_ms - now) > 0) {

            (void)tk_dly_tsk(
                (RELTIM)(next_ms - now)
            );

            next_ms += IMU_PERIOD_MS;

        } else {

            next_ms =
                now + IMU_PERIOD_MS;

            (void)tk_dly_tsk(1);
        }
    }
}

/* -------------------------------------
 * Start IMU subsystem
 * ------------------------------------- */

ER imu_service_start(
    ID shared_i2c_mutex
) {
    if (shared_i2c_mutex <= 0 ||
        imu_task_id > 0) {
        return E_PAR;
    }

    i2c_mutex = shared_i2c_mutex;

    /* Coordinator -> IMU buffer */
    T_CMBF commands = {0};

    commands.mbfatr = TA_TFIFO;
    commands.bufsz =
        sizeof(command_store);

    commands.maxmsz =
        sizeof(imu_command_t);

    commands.bufptr = command_store;

    command_mbf =
        tk_cre_mbf(&commands);

    if (command_mbf <= 0) {
        return command_mbf;
    }

    /* IMU -> Coordinator buffer */
    T_CMBF results = {0};

    results.mbfatr = TA_TFIFO;
    results.bufsz =
        sizeof(result_store);

    results.maxmsz =
        sizeof(imu_result_t);

    results.bufptr = result_store;

    result_mbf =
        tk_cre_mbf(&results);

    if (result_mbf <= 0) {
        return result_mbf;
    }

    /* Create the IMU Task. */
    T_CTSK task = {0};

    task.task = imu_task;
    task.itskpri = 5;
    task.stksz = IMU_TASK_STACK;

#if TK_SUPPORT_SMP
    task.tskatr =
        TA_HLNG | TA_RNG3 | TA_ASSPRC;

    /*
     * Processor 1 = RP2040 core 0.
     * Can be changed during integration.
     */
    task.assprc = TP_PRC1;
#else
    task.tskatr =
        TA_HLNG | TA_RNG3;
#endif

    ID id = tk_cre_tsk(&task);

    if (id <= 0) {
        return id;
    }

    ER er = tk_sta_tsk(id, 0);

    if (er != E_OK) {
        (void)tk_del_tsk(id);
        return er;
    }

    imu_task_id = id;

    return E_OK;
}

/* -------------------------------------
 * Send command to IMU Task
 * ------------------------------------- */

static ER send_command(
    UW type,
    UW request,
    float pitch,
    float roll
) {
    imu_command_t command = {
        type,
        request,
        pitch,
        roll
    };

    if (command_mbf <= 0) {
        return E_OBJ;
    }

    return tk_snd_mbf(
        command_mbf,
        &command,
        sizeof(command),
        TMO_POL
    );
}

/* -------------------------------------
 * Public Coordinator interfaces
 * ------------------------------------- */

ER imu_service_verify(
    UW request_id,
    float pitch,
    float roll
) {
    if (request_id == 0 ||
        !isfinite(pitch) ||
        !isfinite(roll)) {
        return E_PAR;
    }

    return send_command(
        CMD_VERIFY,
        request_id,
        pitch,
        roll
    );
}

ER imu_service_snapshot(
    UW request_id
) {
    return send_command(
        CMD_SNAPSHOT,
        request_id,
        0,
        0
    );
}

ER imu_service_cancel(void) {
    return send_command(
        CMD_CANCEL,
        0,
        0,
        0
    );
}

ER imu_service_reinit(void) {
    return send_command(
        CMD_REINIT,
        0,
        0,
        0
    );
}

INT imu_service_poll(
    imu_result_t *out
) {
    if (out == NULL ||
        result_mbf <= 0) {
        return E_PAR;
    }

    INT n = tk_rcv_mbf(
        result_mbf,
        out,
        TMO_POL
    );

    if (n == E_TMOUT) {
        return 0;
    }

    if (n < 0) {
        return n;
    }

    return n == (INT)sizeof(*out)
        ? 1
        : E_IO;
}
