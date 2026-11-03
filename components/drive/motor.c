/*
 * motor.c - two-wheel dual-input motor driver for the µT-Kernel 3 port.
 *
 * Each wheel uses an A/B pair from its motor driver. Forward drives A with
 * PWM, reverse drives B with PWM, and zero stops both inputs.
 */
#include "motor.h"
#include <bsp/libbsp.h>
#include <sys/sysdef.h>   /* RESETS_* registers */

static const UINT pin_a[2] = { MOTOR_L_IN_A, MOTOR_R_IN_A };
static const UINT pin_b[2] = { MOTOR_L_IN_B, MOTOR_R_IN_B };
static const INT invert[2] = { MOTOR_L_INVERT, MOTOR_R_INVERT };

static UW pct_to_cc(INT pct)
{
    if (pct < 0) pct = -pct;
    if (pct > 100) pct = 100;
    return (UW)((MOTOR_PWM_WRAP + 1U) * (UW)pct / 100U);
}

/* Release PWM from reset if the selected kernel profile left it disabled. */
static void pwm_release_reset(void)
{
    if (in_w(RESETS_RESET) & RESETS_RESET_PWM) {
        clr_w(RESETS_RESET, RESETS_RESET_PWM);
        while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0)
            ;
    }
}

ER motor_init(void)
{
    static const UINT pins[4] = {
        MOTOR_L_IN_A, MOTOR_L_IN_B, MOTOR_R_IN_A, MOTOR_R_IN_B
    };
    ER er;

    pwm_release_reset();
    for (INT i = 0; i < 4; ++i) {
        if ((er = pwm_set_pin(pins[i])) < E_OK) return er;
        if ((er = pwm_set_wrap(pins[i], MOTOR_PWM_WRAP)) < E_OK) return er;
        if ((er = pwm_set_cc(pins[i], 0)) < E_OK) return er;
    }
    for (INT i = 0; i < 4; ++i) {
        if ((er = pwm_set_enabled(pins[i], TRUE)) < E_OK) return er;
    }
    return E_OK;
}

void motor_set(motor_id_t m, INT pct)
{
    if ((INT)m < 0 || (INT)m > 1) return;
    if (invert[m]) pct = -pct;
    UW cc = pct_to_cc(pct);

    if (pct > 0) {
        (void)pwm_set_cc(pin_b[m], 0);
        (void)pwm_set_cc(pin_a[m], cc);
    } else if (pct < 0) {
        (void)pwm_set_cc(pin_a[m], 0);
        (void)pwm_set_cc(pin_b[m], cc);
    } else {
        (void)pwm_set_cc(pin_a[m], 0);
        (void)pwm_set_cc(pin_b[m], 0);
    }
}

void motor_drive(INT left_pct, INT right_pct)
{
    motor_set(MOTOR_LEFT, left_pct);
    motor_set(MOTOR_RIGHT, right_pct);
}

void motor_coast(void)
{
    motor_drive(0, 0);
}

void motor_brake(void)
{
    /* Both driver inputs high requests electrical braking. */
    for (INT m = 0; m < 2; ++m) {
        (void)pwm_set_cc(pin_a[m], MOTOR_PWM_WRAP + 1U);
        (void)pwm_set_cc(pin_b[m], MOTOR_PWM_WRAP + 1U);
    }
}
