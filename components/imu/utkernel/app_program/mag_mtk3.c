#include <stddef.h>
#define PROHIBIT_DEF_SIZE_T

#include "mag_mtk3.h"
#include <tk/device.h>
#include "../device/include/dev_i2c.h"

#include <string.h>

/* LSM303D on Grove 5: I2C0, GP8 SDA / GP9 SCL. */
#define MAG_ADDR            0x1EU

#define STATUS_M            0x07U
#define OUT_X_L_M           0x08U
#define WHO_AM_I            0x0FU
#define OFFSET_X_L_M        0x16U
#define CTRL1               0x20U
#define CTRL5               0x24U
#define CTRL6               0x25U
#define CTRL7               0x26U

#define EXPECTED_ID         0x49U
#define AUTO_INCREMENT      0x80U
#define MAG_DATA_READY      0x08U

/* Conservative rejection near the signed 16-bit limits. */
#define RAW_RAIL_LIMIT      32760

static ID device_id = 0;
static INT initialised = 0;
static INT needs_drain = 0;
static UW sequence = 0;

/*
 * Retain the existing header API.
 * These are three repeated WHO_AM_I reads, not three
 * different identification registers.
 */
static UB observed_id[3] = {0};
static const char *init_step = "Not started";

/*
 * Single owner: the monitor task.
 * Future concurrent I2C0 users require shared arbitration.
 */

const char *mag_mtk3_init_step(void) {
    return init_step;
}

void mag_mtk3_identity(UB out[3]) {
    if (out != NULL) {
        memcpy(out, observed_id, sizeof(observed_id));
    }
}

/* Register access */

static ER read_registers(UB reg, UB *data, SZ count) {
    if (device_id <= 0) {
        return E_OBJ;
    }

    if (data == NULL || count <= 0) {
        return E_PAR;
    }

    /*
     * Unlike the previous DLHC driver, LSM303D requires
     * bit 7 in the sub-address for multi-byte transfers.
     */
    UB sub_address = reg;

    if (count > 1) {
        sub_address |= AUTO_INCREMENT;
    }

    T_I2C_EXEC transfer = {0};
    SZ actual = 0;

    transfer.sadr = MAG_ADDR;
    transfer.snd_size = 1;
    transfer.snd_data = &sub_address;
    transfer.rcv_size = count;
    transfer.rcv_data = data;

    ER er = tk_swri_dev(
        device_id,
        TDN_I2C_EXEC,
        &transfer,
        sizeof(transfer),
        &actual
    );

    if (er != E_OK) {
        return er;
    }

    return actual == (SZ)sizeof(transfer)
        ? E_OK
        : E_IO;
}

static ER write_register(UB reg, UB value) {
    if (device_id <= 0) {
        return E_OBJ;
    }

    UB bytes[2] = {reg, value};
    SZ actual = 0;

    ER er = tk_swri_dev(
        device_id,
        MAG_ADDR,
        bytes,
        sizeof(bytes),
        &actual
    );

    if (er != E_OK) {
        return er;
    }

    return actual == (SZ)sizeof(bytes)
        ? E_OK
        : E_IO;
}

static ER configure_register(UB reg, UB value) {
    ER er = write_register(reg, value);

    if (er != E_OK) {
        return er;
    }

    UB actual = 0;

    er = read_registers(reg, &actual, 1);

    if (er != E_OK) {
        return er;
    }

    return actual == value ? E_OK : E_IO;
}

/* LSM303D outputs low byte first. */
static H signed_le16(const UB *data) {
    W value =
        (W)data[0] |
        ((W)data[1] << 8);

    if (value >= 32768) {
        value -= 65536;
    }

    return (H)value;
}

/* Initialisation */

ER mag_mtk3_init(void) {
    initialised = 0;
    needs_drain = 0;
    sequence = 0;

    memset(observed_id, 0, sizeof(observed_id));

    init_step = "Open iica: Grove 5, GP8/GP9";

    if (device_id <= 0) {
        device_id = tk_opn_dev(
            (CONST UB *)"iica",
            TD_UPDATE
        );

        if (device_id <= 0) {
            return device_id < 0
                ? (ER)device_id
                : E_IO;
        }
    }

    ER er;

    init_step = "Read LSM303D WHO_AM_I three times";

    for (INT i = 0; i < 3; i++) {
        er = read_registers(
            WHO_AM_I,
            &observed_id[i],
            1
        );

        if (er != E_OK) {
            return er;
        }

        if (observed_id[i] != EXPECTED_ID) {
            init_step = "WHO_AM_I mismatch: expected 0x49";
            return E_NOEXS;
        }

        er = tk_dly_tsk(10);

        if (er != E_OK) {
            return er;
        }
    }

    init_step = "CTRL7: magnetic power-down";

    er = configure_register(CTRL7, 0x02U);

    if (er != E_OK) {
        return er;
    }

    /*
     * Accelerometer powered down and axes disabled.
     * Block data update enabled for consistent byte pairs.
     */
    init_step = "CTRL1: accelerometer off, BDU enabled";

    er = configure_register(CTRL1, 0x08U);

    if (er != E_OK) {
        return er;
    }

    /*
     * Temperature disabled.
     * High-resolution magnetic measurements at 50 Hz.
     */
    init_step = "CTRL5: magnetic high resolution, 50 Hz";

    er = configure_register(CTRL5, 0x70U);

    if (er != E_OK) {
        return er;
    }

    init_step = "CTRL6: magnetic range +/-4 gauss";

    er = configure_register(CTRL6, 0x20U);

    if (er != E_OK) {
        return er;
    }

    /*
     * No magnetic calibration has been performed yet.
     * Clear any hardware offsets left by previous firmware.
     */
    init_step = "Clear hardware magnetic offsets";

    for (INT i = 0; i < 6; i++) {
        er = configure_register(
            (UB)(OFFSET_X_L_M + i),
            0x00U
        );

        if (er != E_OK) {
            return er;
        }
    }

    /* Finish any previous partial output read. */
    init_step = "Drain previous magnetic output";

    UB discard[6];

    er = read_registers(
        OUT_X_L_M,
        discard,
        sizeof(discard)
    );

    if (er != E_OK) {
        return er;
    }

    /*
     * Continuous magnetic conversion.
     * Magnetic low-power and temperature-only modes disabled.
     */
    init_step = "CTRL7: continuous magnetic conversion";

    er = configure_register(CTRL7, 0x00U);

    if (er != E_OK) {
        return er;
    }

    init_step = "Wait for magnetic conversion";

    er = tk_dly_tsk(100);

    if (er != E_OK) {
        return er;
    }

    initialised = 1;
    init_step = "Ready: LSM303D";

    return E_OK;
}

/* Sample acquisition */

static INT sample_error(
    mag_sample_t *out,
    mag_status_t status,
    ER error
) {
    out->valid = 0;
    out->status = status;
    out->error = error;

    return error;
}

INT mag_mtk3_read(mag_sample_t *out) {
    if (out == NULL) {
        return E_PAR;
    }

    memset(out, 0, sizeof(*out));
    out->status = MAG_STATUS_NOT_READY;

    if (!initialised) {
        return sample_error(
            out,
            MAG_STATUS_NOT_READY,
            E_OBJ
        );
    }

    /*
     * Recover from a failed burst that may have consumed
     * only part of a sample while BDU was enabled.
     */
    if (needs_drain) {
        UB discard[6];

        ER er = read_registers(
            OUT_X_L_M,
            discard,
            sizeof(discard)
        );

        if (er != E_OK) {
            return sample_error(out, MAG_STATUS_IO, er);
        }

        needs_drain = 0;
        out->status = MAG_STATUS_LOCK_CLEARED;
        return 0;
    }

    UB status = 0;

    ER er = read_registers(STATUS_M, &status, 1);

    if (er != E_OK) {
        return sample_error(out, MAG_STATUS_IO, er);
    }

    if ((status & MAG_DATA_READY) == 0) {
        out->status = MAG_STATUS_NO_DATA;
        return 0;
    }

    UB data[6];

    er = read_registers(
        OUT_X_L_M,
        data,
        sizeof(data)
    );

    if (er != E_OK) {
        needs_drain = 1;
        return sample_error(out, MAG_STATUS_IO, er);
    }

    /* LSM303D order: X low/high, Y low/high, Z low/high. */
    out->raw_x = signed_le16(&data[0]);
    out->raw_y = signed_le16(&data[2]);
    out->raw_z = signed_le16(&data[4]);

    /*
     * LSM303D uses signed 16-bit measurements.
     * The old DLHC +/-2048 check does not apply.
     */
    if (out->raw_x <= -RAW_RAIL_LIMIT ||
        out->raw_x >= RAW_RAIL_LIMIT ||
        out->raw_y <= -RAW_RAIL_LIMIT ||
        out->raw_y >= RAW_RAIL_LIMIT ||
        out->raw_z <= -RAW_RAIL_LIMIT ||
        out->raw_z >= RAW_RAIL_LIMIT) {
        return sample_error(out, MAG_STATUS_RANGE, E_IO);
    }

    SYSTIM time;

    er = tk_get_otm(&time);

    if (er != E_OK) {
        return sample_error(out, MAG_STATUS_TIMING, er);
    }

    /*
     * Host read-completion time, not the exact conversion time.
     * This slow diagnostic reader does not collect every
     * magnetic conversion.
     */
    out->timestamp_ms = time.lo;
    out->sequence = ++sequence;
    out->valid = 1;
    out->status = MAG_STATUS_OK;
    out->error = E_OK;

    return 1;
}