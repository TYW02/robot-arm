#ifndef AAI17_MAG_MTK3_H
#define AAI17_MAG_MTK3_H

#include <tk/tkernel.h>

typedef enum {
    MAG_STATUS_NOT_READY = 0,
    MAG_STATUS_OK,
    MAG_STATUS_NO_DATA,
    MAG_STATUS_IO,
    MAG_STATUS_LOCK_CLEARED,
    MAG_STATUS_RANGE,
    MAG_STATUS_TIMING
} mag_status_t;

typedef struct {
    /* Raw magnetic readings in sensor coordinates. */
    H raw_x;
    H raw_y;
    H raw_z;

    /* Time the host finished reading this sample. */
    UW timestamp_ms;
    UW sequence;

    INT valid;
    mag_status_t status;
    ER error;
} mag_sample_t;

/*
 * Initialise the magnetic interface expected on a GY-511.
 *
 * Grove 5:
 *   GP8 = SDA
 *   GP9 = SCL
 *   I2C0 device name = "iica"
 *
 * This diagnostic driver has one owner: the monitor task.
 * Do not call it concurrently from another task.
 * Any future additional I2C0 users need shared bus arbitration.
 */
ER mag_mtk3_init(void);

/*
 * Read the latest available magnetic sample.
 *
 * Returns:
 *   1: Valid new sample.
 *   0: No usable sample this time; inspect status.
 *  <0: Error; inspect status and error.
 *
 * Raw readings are not calibrated heading measurements.
 */
INT mag_mtk3_read(mag_sample_t *out);

/* Last startup operation, for diagnostic messages. */
const char *mag_mtk3_init_step(void);

/* Copy the identification bytes observed during startup. */
void mag_mtk3_identity(UB out[3]);

#endif