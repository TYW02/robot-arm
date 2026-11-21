/*
 *----------------------------------------------------------------------
 *    Device Driver for μT-Kernel 3.0
 *
 *    Copyright (C) 2020-2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2023/03.
 *
 *----------------------------------------------------------------------
 */

#include <sys/machine.h>
#include <tm/tmonitor.h>
#ifdef CPU_RP2040

#include <tk/tkernel.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif
#include "../../i2c.h"
#include "../../../include/dev_def.h"
#if DEV_IIC_ENABLE
/*
 *	i2c_rp2040.c
  *	I2C device driver
 *	System-dependent definition for RP2040
 */

/*----------------------------------------------------------------------
 * Device register base address
*/
const LOCAL UW ba[DEV_I2C_UNITNM] = { I2C0_BASE, I2C1_BASE };

/*----------------------------------------------------------------------
 * Device data
 **** This data is not used in beta version
*/
const LOCAL struct {
	UINT	intno;		// Interrupt number
	PRI	intpri;		// Interrupt priority
	TMO	timout;		// I2C timeout time
} ll_devdat[DEV_I2C_UNITNM] = {
	{	/* I2C0 */
		.intno		= INTNO_I2C0,
		.intpri		= DEVCNF_I2C0_INTPRI,
		.timout		= DEVCNF_I2C0_TMO,
	},
	{	/* I2C1 */
		.intno		= INTNO_I2C1,
		.intpri		= DEVCNF_I2C1_INTPRI,
		.timout		= DEVCNF_I2C1_TMO,
	},
};

/*----------------------------------------------------------------------
 * Device control data
*/
typedef struct {
	ID	done_flgid;	// Transaction phase completion flag
	UW	state;		// Operating state
	UW	sadr;		// Slave address
	ER	ioerr;		// Communication error
	UW	err_reason;	// Error reason
	UW	sdat_num;	// Number of send data
	UW	rdat_num;	// Number of receive data
	UB	*sbuf;		// Send buffer
	UB	*rbuf;		// Receive buffer
} T_I2C_LLDCB;
LOCAL T_I2C_LLDCB	ll_devcb[DEV_I2C_UNITNM];

#if TK_SUPPORT_SMP
LOCAL T_SPLOCK ll_lock[DEV_I2C_UNITNM];
#define I2C_LOCK(unit, sts)   ((void)ISpinLock(&ll_lock[(unit)], &(sts)))
#define I2C_UNLOCK(unit, sts) ((void)ISpinUnlock(&ll_lock[(unit)], (sts)))
#else
#define I2C_LOCK(unit, sts)   DI(sts)
#define I2C_UNLOCK(unit, sts) EI(sts)
#endif

#define I2C_INT_ERROR	(I2C_INT_TX_ABRT|I2C_INT_TX_OVER|I2C_INT_RX_OVER|I2C_INT_RX_UNDER)

/*-------------------------------------------------------
 * Interrupt handler
 */
/*
 * Wait for the bus STOP before declaring a transaction complete.
 * This value is unused by the existing state definitions.
 */
#define I2C_STS_WAIT_STOP 0x0006U

/*
 * Disabling the controller is asynchronous.
 * Bound this short hardware wait using the RP2040 microsecond timer.
 * Called from task context, without holding the spinlock.
 */
LOCAL ER i2c_wait_disabled(INT unit)
{
    UW started = in_w(TIMER_TIMERAWL);

    while ((in_w(I2C_ENABLE_STATUS(unit)) & 1U) != 0U) {
        if ((UW)(in_w(TIMER_TIMERAWL) - started) >= 2000U)
            return E_TMOUT;
    }

    return E_OK;
}

/* Interrupt handler: advance one transaction, then wake its owner. */
LOCAL void i2c_inthdr(UINT intno)
{
    INT unit;
    UINT intsts;
    UW status;
    UW cmd;
    BOOL abort = FALSE;
    BOOL wake = FALSE;
    T_I2C_LLDCB *p_cb;

    for (unit = 0; unit < DEV_I2C_UNITNM; ++unit) {
        if (ll_devdat[unit].intno == intno)
            break;
    }

    if (unit >= DEV_I2C_UNITNM) {
        ClearInt(intno);
        return;
    }

    p_cb = &ll_devcb[unit];

    I2C_LOCK(unit, intsts);
    status = in_w(I2C_INTR_STAT(unit));

    if ((status & I2C_INT_ERROR) != 0U) {
        abort = TRUE;
    } else {
        switch (p_cb->state) {
        case I2C_STS_SEND:
            if ((status & I2C_INT_TX_EMPTY) != 0U) {
                if (p_cb->sdat_num == 0U) {
                    abort = TRUE;
                    break;
                }

                cmd = (UW)(*p_cb->sbuf++) & I2C_DATA_CMD_DAT;
                --p_cb->sdat_num;

                if (p_cb->sdat_num != 0U) {
                    out_w(I2C_DATA_CMD(unit), cmd);
                } else if (p_cb->rdat_num != 0U) {
                    /*
                     * Combined register read:
                     * queue the final write without STOP, then put
                     * RESTART on the FIRST READ command.
                     */
                    out_w(I2C_DATA_CMD(unit), cmd);

                    cmd = I2C_DATA_CMD_CMD_READ |
                          I2C_DATA_CMD_RESTART;

                    if (p_cb->rdat_num == 1U)
                        cmd |= I2C_DATA_CMD_STOP;

                    p_cb->state = I2C_STS_RECV;

                    out_w(
                        I2C_INTR_MASK(unit),
                        I2C_INT_RX_FULL |
                        I2C_INT_STOP_DET |
                        I2C_INT_ERROR
                    );

                    out_w(I2C_DATA_CMD(unit), cmd);
                } else {
                    /*
                     * Final write byte: request STOP, but do not
                     * report completion until STOP is detected.
                     */
                    p_cb->state = I2C_STS_WAIT_STOP;

                    out_w(
                        I2C_INTR_MASK(unit),
                        I2C_INT_STOP_DET | I2C_INT_ERROR
                    );

                    out_w(
                        I2C_DATA_CMD(unit),
                        cmd | I2C_DATA_CMD_STOP
                    );
                }
            }
            break;

        case I2C_STS_RECV:
            if ((status & I2C_INT_RX_FULL) != 0U) {
                if (p_cb->rdat_num == 0U) {
                    abort = TRUE;
                    break;
                }

                *p_cb->rbuf++ = (UB)(
                    in_w(I2C_DATA_CMD(unit)) &
                    I2C_DATA_CMD_DAT
                );

                --p_cb->rdat_num;

                if (p_cb->rdat_num != 0U) {
                    cmd = I2C_DATA_CMD_CMD_READ;

                    if (p_cb->rdat_num == 1U)
                        cmd |= I2C_DATA_CMD_STOP;

                    out_w(I2C_DATA_CMD(unit), cmd);
                } else {
                    p_cb->state = I2C_STS_WAIT_STOP;

                    out_w(
                        I2C_INTR_MASK(unit),
                        I2C_INT_STOP_DET | I2C_INT_ERROR
                    );
                }
            }
            break;

        case I2C_STS_WAIT_STOP:
        case I2C_STS_STOP:
            break;

        default:
            if (status != 0U)
                abort = TRUE;
            break;
        }

        if (!abort && (status & I2C_INT_STOP_DET) != 0U) {
            (void)in_w(I2C_CLR_STOP_DET(unit));

            if (p_cb->state == I2C_STS_WAIT_STOP &&
                p_cb->sdat_num == 0U &&
                p_cb->rdat_num == 0U) {
                p_cb->state = I2C_STS_STOP;
                wake = TRUE;
            } else {
                /* STOP occurred before all requested data completed. */
                abort = TRUE;
            }
        }
    }

    if (abort) {
        /*
         * Preserve the hardware abort source BEFORE clearing it.
         * Zero means this was another interrupt/state error.
         */
        p_cb->err_reason =
            (status & I2C_INT_TX_ABRT) != 0U
                ? in_w(I2C_TX_ABRT_SOURCE(unit))
                : 0U;

        p_cb->ioerr = E_IO;
        p_cb->state = I2C_STS_STOP;
        wake = TRUE;

        (void)in_w(I2C_CLR_INTR(unit));
    }

    /*
     * Do not indiscriminately clear all interrupts on the normal
     * path: that can erase a newly arriving STOP indication.
     */
    if (wake)
        out_w(I2C_INTR_MASK(unit), 0);

    ClearInt(intno);
    I2C_UNLOCK(unit, intsts);

    /* Keep kernel wake-up outside the interrupt spinlock. */
    if (wake)
        (void)tk_set_flg(p_cb->done_flgid, 1);
}

/* Execute one complete transaction. */
LOCAL ER i2c_trans(INT unit, T_I2C_LLDCB *p_cb)
{
    UINT intsts;
    UINT flags;
    ER err;
    ER disabled;
    UW cmd;
    static UW error_count[DEV_I2C_UNITNM];

    out_w(I2C_INTR_MASK(unit), 0);
    out_w(I2C_ENABLE(unit), 0);

    err = i2c_wait_disabled(unit);
    if (err != E_OK)
        return err;

    I2C_LOCK(unit, intsts);

    p_cb->ioerr = E_OK;
    p_cb->err_reason = 0;

    (void)tk_clr_flg(p_cb->done_flgid, 0);
    (void)in_w(I2C_CLR_INTR(unit));
    ClearInt(ll_devdat[unit].intno);

    out_w(
        I2C_TAR(unit),
        p_cb->sadr & I2C_TAR_7BIT_ADR
    );

    if (p_cb->sdat_num != 0U) {
        p_cb->state = I2C_STS_SEND;

        out_w(
            I2C_INTR_MASK(unit),
            I2C_INT_TX_EMPTY | I2C_INT_ERROR
        );

        out_w(I2C_ENABLE(unit), I2C_ENABLE_ENABLE);
    } else if (p_cb->rdat_num != 0U) {
        p_cb->state = I2C_STS_RECV;

        cmd = I2C_DATA_CMD_CMD_READ;
        if (p_cb->rdat_num == 1U)
            cmd |= I2C_DATA_CMD_STOP;

        out_w(
            I2C_INTR_MASK(unit),
            I2C_INT_RX_FULL |
            I2C_INT_STOP_DET |
            I2C_INT_ERROR
        );

        out_w(I2C_ENABLE(unit), I2C_ENABLE_ENABLE);
        out_w(I2C_DATA_CMD(unit), cmd);
    } else {
        I2C_UNLOCK(unit, intsts);
        return E_PAR;
    }

    I2C_UNLOCK(unit, intsts);

    /*
     * The interrupt handler now handles the write-to-read
     * transition itself. Wake once at transaction completion.
     */
    err = tk_wai_flg(
        p_cb->done_flgid,
        1,
        TWF_ORW | TWF_BITCLR,
        &flags,
        ll_devdat[unit].timout
    );

    I2C_LOCK(unit, intsts);
    out_w(I2C_INTR_MASK(unit), 0);

    if (err == E_OK) {
        err = p_cb->ioerr;

        if (err == E_OK &&
            (p_cb->state != I2C_STS_STOP ||
             p_cb->sdat_num != 0U ||
             p_cb->rdat_num != 0U)) {
            err = E_IO;
        }
    }

    p_cb->state = I2C_STS_STOP;
    out_w(I2C_ENABLE(unit), 0);
    I2C_UNLOCK(unit, intsts);

    disabled = i2c_wait_disabled(unit);
    if (err == E_OK)
        err = disabled;

    if (err != E_OK) {
        UW count = ++error_count[unit];

        /* Repeat enough to capture without flooding the terminal. */
        if (count <= 4U || (count % 16U) == 0U) {
            tm_printf(
                (UB *)"[I2C ERROR] bus=%d addr=0x%02X "
                       "error=%d abort=0x%08X count=%u\n",
                unit,
                (UINT)p_cb->sadr,
                err,
                (UINT)p_cb->err_reason,
                count
            );
        }
    }

    return err;
}

/*----------------------------------------------------------------------
 * Set communication parameters
 */
Inline void set_com_start(UW unit, UW sadr, UW sdat_num, UW rdat_num, UB *sbuf, UB *rbuf)
{
	ll_devcb[unit].state	= I2C_STS_START;
	ll_devcb[unit].sadr	= sadr;		/* Slave address */
	ll_devcb[unit].sdat_num	= sdat_num;	/* Number of send data */
	ll_devcb[unit].rdat_num	= rdat_num;	/* Number of receive data */
	ll_devcb[unit].sbuf	= sbuf;		/* Send buffer */
	ll_devcb[unit].rbuf	= rbuf;		/* Receive buffer */
}


/*----------------------------------------------------------------------
 * Low level device control
 */
EXPORT W dev_i2c_llctl( UW unit, INT cmd, UW p1, UW p2, UW *pp)
{
	T_I2C_EXEC	*p_ex;
	ER		err	= E_OK;

	p_ex = (T_I2C_EXEC*)pp;

	switch(cmd) {
	case LLD_I2C_OPEN:
		/* Initial Configuration */
		out_w(I2C_ENABLE(unit), 0);						// Disable I2C

		out_w(I2C_CON(unit), I2C_CON_SLAVE_DISABLE | I2C_CON_MASTER_MODE	// Master Mode
			| I2C_CON_RESTART_EN | I2C_CON_TX_EMPTY_CTRL
			| I2C_CON_7BIT_ADR | I2C_CON_SPEED_FAST);			// 7bit Address & Fast Speed

		out_w(I2C_RX_TL(unit), 0);
		out_w(I2C_TX_TL(unit), 0);
		out_w(I2C_DMA_CR(unit), I2C_DMA_CR_RDMAE | I2C_DMA_CR_TDMAE);

		        /*
         * I2C0: magnetometer, approximately 100 kHz.
         * I2C1: MPU, retain the existing approximately 400 kHz.
         * Counts assume the configured 125 MHz system clock.
         */
        if (unit == DEV_I2C_0) {
            out_w(I2C_FS_SCL_HCNT(unit), 500);
            out_w(I2C_FS_SCL_LCNT(unit), 750);
            out_w(I2C_FS_SPKLEN(unit), 750 / 16);
        } else {
            out_w(I2C_FS_SCL_HCNT(unit), 126);
            out_w(I2C_FS_SCL_LCNT(unit), 187);
            out_w(I2C_FS_SPKLEN(unit), 187 / 16);
        }

        out_w(I2C_SDA_HOLD(unit), 38);

		/* I2C interrupt enable */
		break;

	case LLD_I2C_CLOSE:
		/* I2C interrupt disable */
		out_w(I2C_INTR_MASK(unit), 0);
		out_w(I2C_ENABLE(unit), 0);
		break;

	case LLD_I2C_READ:
		set_com_start( unit, p1, 0, p2, NULL, (UB*)pp);
		err = i2c_trans(unit, &ll_devcb[unit]);
		if(err >= E_OK) err = p2 - ll_devcb[unit].rdat_num;
		break;

	case LLD_I2C_WRITE:
		set_com_start( unit, p1, p2, 0, (UB*)pp, NULL);
		err = i2c_trans(unit, &ll_devcb[unit]);
		if(err >= E_OK) err = p2 - ll_devcb[unit].sdat_num;
		break;

	case LLD_I2C_EXEC:
		set_com_start( unit, p_ex->sadr, p_ex->snd_size, p_ex->rcv_size, p_ex->snd_data, p_ex->rcv_data);
		err = i2c_trans(unit, &ll_devcb[unit]);
		if(err >= E_OK) err = p_ex->snd_size + p_ex->rcv_size;
		break;
	}
	
	return (W)err;
}

/*----------------------------------------------------------------------
 * Device initialization
 */
EXPORT ER dev_i2c_llinit( T_I2C_DCB *p_dcb)
{
	T_DINT	dint;
	T_CFLG	cflg = {0};
	UW	unit;
	ER	err	= E_OK;

	unit = p_dcb->unit;

#if TK_SUPPORT_SMP
	(void)InitSpinLock(&ll_lock[unit]);
#endif
	cflg.flgatr = TA_TFIFO;
	ll_devcb[unit].done_flgid = tk_cre_flg(&cflg);
	if(ll_devcb[unit].done_flgid < E_OK) {
		return ll_devcb[unit].done_flgid;
	}

	switch(unit) {
	case DEV_I2C_0:

#if DEVCNF_I2C_RESET
		/* Reset I2C module*/
		set_w( RESETS_RESET, RESETS_RESET_I2C0);
		clr_w( RESETS_RESET, RESETS_RESET_I2C0);
		while((in_w(RESETS_RESET_DONE)&(RESETS_RESET_I2C0))==0);
#endif

#if DEVCNF_I2C_SETPINFUNC
		out_w(GPIO_CTRL(16), GPIO_CTRL_FUNCSEL_I2C);
		out_w(GPIO(16), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
		out_w(GPIO_CTRL(17), GPIO_CTRL_FUNCSEL_I2C);
		out_w(GPIO(17), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
#endif
		break;
	case DEV_I2C_1:
#if DEVCNF_I2C_RESET
		/* Reset I2C module*/
		set_w( RESETS_RESET, RESETS_RESET_I2C1);
		clr_w( RESETS_RESET, RESETS_RESET_I2C1);
		while((in_w(RESETS_RESET_DONE)&(RESETS_RESET_I2C1))==0);
#endif

#if DEVCNF_I2C_SETPINFUNC
		out_w(GPIO_CTRL(2), GPIO_CTRL_FUNCSEL_I2C);
		out_w(GPIO(2), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
		out_w(GPIO_CTRL(3), GPIO_CTRL_FUNCSEL_I2C);
		out_w(GPIO(3), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
		break;
#endif
	default:
		(void)tk_del_flg(ll_devcb[unit].done_flgid);
		return E_PAR;
	}

	out_w(I2C_ENABLE(unit), 0);			// Disable I2C

	/* Interrupt handler definition */
	dint.intatr	= TA_HLNG;
	dint.inthdr	= i2c_inthdr;
	
	err = tk_def_int(ll_devdat[unit].intno, &dint);
	if(err == E_OK) {
		ClearInt(ll_devdat[unit].intno);
		EnableInt(ll_devdat[unit].intno, ll_devdat[unit].intpri);
	} else {
		(void)tk_del_flg(ll_devcb[unit].done_flgid);
	}

	return err;
}

#endif		/* DEV_IIC_ENABLE */
#endif		/* CPU_RP2040 */
