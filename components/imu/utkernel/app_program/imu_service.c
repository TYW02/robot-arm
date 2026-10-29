#include <stddef.h>

/* Prevent µT-Kernel from redefining size_t. */
#define PROHIBIT_DEF_SIZE_T

#include "imu_service.h"
#include <tm/tmonitor.h>
#include <math.h>
#include <string.h>

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#define IMU_CPU() tk_get_prc()
#else
#define IMU_CPU() 1
#endif

#define IMU_PERIOD_MS   20U
#define IMU_TASK_STACK  4096

enum {
    CMD_SET_YAW = 1,
    CMD_REINIT
};

typedef struct {
    UW type;
    UW request_id;
    float yaw_deg;
} imu_command_t;

/* Persistent storage for kernel message buffers. */
static UW command_store[128];
static UW result_store[256];

static ID command_mbf = 0;
static ID result_mbf = 0;
static ID imu_task_id = 0;

static ID i2c_mutex = 0;
static ID sample_mutex = 0;

static imu_sample_t latest_sample = {0};
static UW dropped_events = 0;

/* -------------------------------------
 * Service events
 * ------------------------------------- */

static void publish(
    UW event,
    UW request_id,
    ER error
) {
    imu_result_t result = {0};

    result.event = event;
    result.request_id = request_id;
    result.error = error;
    result.dropped_events = dropped_events;
    result.producer_core = IMU_CPU();

    /*
     * Never wait for room in the event queue.
     * The coordinator must time out missing replies.
     */
    ER er = tk_snd_mbf(
        result_mbf,
        &result,
        sizeof(result),
        TMO_POL
    );

    if (er != E_OK) {
        dropped_events++;
    }
}

/* -------------------------------------
 * Latest-sample storage
 * ------------------------------------- */

static ER save_sample(const imu_sample_t *sample) {
    ER er = tk_loc_mtx(sample_mutex, TMO_FEVR);

    if (er != E_OK) {
        return er;
    }

    latest_sample = *sample;

    return tk_unl_mtx(sample_mutex);
}

static ER clear_sample(void) {
    imu_sample_t empty = {0};

    empty.status = IMU_STATUS_NOT_READY;

    return save_sample(&empty);
}

/* -------------------------------------
 * IMU owner task
 * ------------------------------------- */

static void imu_task(INT stacd, void *exinf) {
    (void)stacd;
    (void)exinf;

    /*
     * The coordinator must keep the car stopped
     * throughout startup calibration.
     */
    ER er = imu_mtk3_init(i2c_mutex);

    INT online = er == E_OK;
    INT fault_announced = !online;

    publish(
        online ? IMU_EVT_READY : IMU_EVT_FAULT,
        0,
        er
    );

    /*
     * Wait one sampling period before the first read.
     * The driver established its time reference during init.
     */
    UW next_ms = imu_mtk3_now_ms() + IMU_PERIOD_MS;

    while (1) {
        UW now = imu_mtk3_now_ms();

        if ((W)(next_ms - now) > 0) {
            (void)tk_dly_tsk(
                (RELTIM)(next_ms - now)
            );
        }

        next_ms += IMU_PERIOD_MS;

        /* -----------------------------
         * 1. Sample the gyroscope
         * ----------------------------- */

        if (online) {
            imu_sample_t sample;

            (void)imu_mtk3_read(&sample);

            ER stored = save_sample(&sample);

            INT healthy =
                stored == E_OK &&
                sample.rate_valid &&
                sample.heading_valid;

            if (healthy) {
                fault_announced = 0;
            } else if (!fault_announced) {

                /* Print the first fault that invalidated heading. */
                tm_printf(
                    (UB *)"\n[IMU FIRST FAULT] "
                        "status=%d | interval=%ums "
                        "| GX=%d GY=%d GZ=%d\n",
                    sample.status,
                    sample.interval_ms,
                    sample.raw_gx,
                    sample.raw_gy,
                    sample.raw_gz
                );

                publish(
                    IMU_EVT_FAULT,
                    0,
                    stored != E_OK ? stored : E_IO
                );

                fault_announced = 1;
            }
        }

        /* -----------------------------
         * 2. Process one queued command
         * ----------------------------- */

        imu_command_t command;

        INT n = tk_rcv_mbf(
            command_mbf,
            &command,
            TMO_POL
        );

        if (n == (INT)sizeof(command)) {
            if (command.request_id == 0) {
                publish(IMU_EVT_FAULT, 0, E_PAR);
            }

            else if (command.type == CMD_REINIT) {
                /*
                 * Hide the previous sample before calibration.
                 * The caller must keep the car stopped.
                 */
                er = clear_sample();

                if (er == E_OK) {
                    er = imu_mtk3_init(i2c_mutex);

                    online = er == E_OK;
                    fault_announced = !online;
                }

                publish(
                    er == E_OK
                        ? IMU_EVT_READY
                        : IMU_EVT_FAULT,
                    command.request_id,
                    er
                );

                /*
                 * Calibration takes time. Establish a new
                 * schedule instead of catching up old samples.
                 */
                next_ms =
                    imu_mtk3_now_ms() + IMU_PERIOD_MS;
            }

            else if (command.type == CMD_SET_YAW) {
                /*
                 * Process this after a sensor read so the
                 * driver's low-rate check uses current data.
                 */
                er = online
                    ? imu_mtk3_set_yaw(command.yaw_deg)
                    : E_OBJ;

                if (er == E_OK) {
                    /*
                     * Do not expose the old reference after
                     * reporting that a new one was established.
                     */
                    er = clear_sample();
                    fault_announced = 0;

                    next_ms =
                        imu_mtk3_now_ms() + IMU_PERIOD_MS;
                }

                publish(
                    er == E_OK
                        ? IMU_EVT_REFERENCE_SET
                        : IMU_EVT_FAULT,
                    command.request_id,
                    er
                );
            }

            else {
                publish(
                    IMU_EVT_FAULT,
                    command.request_id,
                    E_PAR
                );
            }
        }

        else if (n != E_TMOUT) {
            publish(
                IMU_EVT_FAULT,
                0,
                n < 0 ? n : E_IO
            );
        }

        /* -----------------------------
         * 3. Avoid rapid catch-up reads
         * ----------------------------- */

        now = imu_mtk3_now_ms();

        if ((W)(now - next_ms) >= 0) {
            next_ms = now + IMU_PERIOD_MS;
        }
    }
}

/* -------------------------------------
 * Start the service
 * ------------------------------------- */

ER imu_service_start(ID shared_i2c_mutex) {
    if (shared_i2c_mutex <= 0 || imu_task_id > 0) {
        return E_PAR;
    }

    i2c_mutex = shared_i2c_mutex;
    dropped_events = 0;

    memset(&latest_sample, 0, sizeof(latest_sample));
    latest_sample.status = IMU_STATUS_NOT_READY;

    /*
     * This mutex protects only the latest-sample copy.
     * It is separate from the shared I2C mutex.
     */
    T_CMTX mutex = {0};

    mutex.mtxatr = TA_TFIFO | TA_INHERIT;

    sample_mutex = tk_cre_mtx(&mutex);

    if (sample_mutex <= 0) {
        ER error = sample_mutex;
        sample_mutex = 0;
        return error;
    }

    ER er;

    /* Coordinator -> IMU command buffer. */
    T_CMBF commands = {0};

    commands.mbfatr = TA_TFIFO;
    commands.bufsz = sizeof(command_store);
    commands.maxmsz = sizeof(imu_command_t);
    commands.bufptr = command_store;

    command_mbf = tk_cre_mbf(&commands);

    if (command_mbf <= 0) {
        er = command_mbf;
        goto cleanup;
    }

    /* IMU -> coordinator event buffer. */
    T_CMBF results = {0};

    results.mbfatr = TA_TFIFO;
    results.bufsz = sizeof(result_store);
    results.maxmsz = sizeof(imu_result_t);
    results.bufptr = result_store;

    result_mbf = tk_cre_mbf(&results);

    if (result_mbf <= 0) {
        er = result_mbf;
        goto cleanup;
    }

    T_CTSK task = {0};

    task.task = imu_task;
    task.itskpri = 5;
    task.stksz = IMU_TASK_STACK;
    task.tskatr = TA_HLNG | TA_RNG3;

#if TK_SUPPORT_SMP
    task.tskatr |= TA_ASSPRC;

    /* Kernel processor 1 = RP2040 physical core 0. */
    task.assprc = TP_PRC1;
#endif

    imu_task_id = tk_cre_tsk(&task);

    if (imu_task_id <= 0) {
        er = imu_task_id;
        imu_task_id = 0;
        goto cleanup;
    }

    er = tk_sta_tsk(imu_task_id, 0);

    if (er == E_OK) {
        return E_OK;
    }

    (void)tk_del_tsk(imu_task_id);
    imu_task_id = 0;

cleanup:
    /*
     * Release objects created before a startup failure,
     * allowing the coordinator to try starting again.
     */
    if (result_mbf > 0) {
        (void)tk_del_mbf(result_mbf);
    }

    if (command_mbf > 0) {
        (void)tk_del_mbf(command_mbf);
    }

    if (sample_mutex > 0) {
        (void)tk_del_mtx(sample_mutex);
    }

    result_mbf = 0;
    command_mbf = 0;
    sample_mutex = 0;

    return er;
}

/* -------------------------------------
 * Read the latest sample
 * ------------------------------------- */

ER imu_service_latest(imu_sample_t *out) {
    if (out == NULL) {
        return E_PAR;
    }

    memset(out, 0, sizeof(*out));
    out->status = IMU_STATUS_NOT_READY;

    if (imu_task_id <= 0 || sample_mutex <= 0) {
        return E_OBJ;
    }

    /*
     * Do not block motion control while obtaining a sample.
     * Failure leaves the caller with an invalid result.
     */
    ER er = tk_loc_mtx(sample_mutex, TMO_POL);

    if (er != E_OK) {
        return er;
    }

    *out = latest_sample;

    er = tk_unl_mtx(sample_mutex);

    if (er != E_OK) {
        out->rate_valid = 0;
        out->heading_valid = 0;
        out->low_rate = 0;
        return er;
    }

    if (!out->rate_valid) {
        return E_OBJ;
    }

    SYSTIM time;

    if (tk_get_otm(&time) != E_OK ||
        (UW)(time.lo - out->timestamp_ms) >
            IMU_MAX_GAP_MS) {

        out->rate_valid = 0;
        out->heading_valid = 0;
        out->low_rate = 0;
        out->status = IMU_STATUS_TIMING;

        return E_OBJ;
    }

    /*
     * A valid rate does not guarantee a valid heading.
     * The caller must also check out->heading_valid.
     */
    return E_OK;
}

/* -------------------------------------
 * Queue coordinator commands
 * ------------------------------------- */

static ER send_command(
    UW type,
    UW request_id,
    float yaw_deg
) {
    if (request_id == 0 ||
        !isfinite(yaw_deg) ||
        fabsf(yaw_deg) > 1000000.0f) {
        return E_PAR;
    }

    if (imu_task_id <= 0 || command_mbf <= 0) {
        return E_OBJ;
    }

    imu_command_t command = {
        .type = type,
        .request_id = request_id,
        .yaw_deg = yaw_deg
    };

    return tk_snd_mbf(
        command_mbf,
        &command,
        sizeof(command),
        TMO_POL
    );
}

ER imu_service_set_yaw(
    UW request_id,
    float yaw_deg
) {
    return send_command(
        CMD_SET_YAW,
        request_id,
        yaw_deg
    );
}

ER imu_service_reinit(UW request_id) {
    return send_command(
        CMD_REINIT,
        request_id,
        0.0f
    );
}

/* -------------------------------------
 * Receive service events
 * ------------------------------------- */

INT imu_service_poll(imu_result_t *out) {
    if (out == NULL || result_mbf <= 0) {
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