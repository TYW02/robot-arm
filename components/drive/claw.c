/*
 * claw.c - claw servo driver (µT-Kernel 3 port, no Pico SDK)
 *
 * 125 MHz / divider 125 = 1 MHz counter, so one count = 1 us.
 * wrap 19999 -> 20 ms period (50 Hz); compare value = pulse width in us.
 */
#include "claw.h"
#include <bsp/libbsp.h>
#include <sys/sysdef.h>

#define CLAW_SLICE      ((CLAW_PIN >> 1) & 7)
#define CLAW_DIV_REG    (PWM_BASE + PWM_CHx_DIV + CLAW_SLICE * 0x14)
#define CLAW_DIV_INT    125            /* integer part, bits 11:4 */
#define CLAW_WRAP       19999

LOCAL INT claw_deg = -1;

/* Same reason as motor.c: the port leaves PWM in reset when USE_PTMR = 0. */
LOCAL void pwm_release_reset(void)
{
    if (in_w(RESETS_RESET) & RESETS_RESET_PWM) {
        clr_w(RESETS_RESET, RESETS_RESET_PWM);
        while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0)
            ;
    }
}

LOCAL UW deg_to_us(INT deg)
{
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    return (UW)(CLAW_MIN_US + (CLAW_MAX_US - CLAW_MIN_US) * deg / 180);
}

ER claw_init(void)
{
    ER er;

    pwm_release_reset();
    out_w(CLAW_DIV_REG, CLAW_DIV_INT << 4);
    if ((er = pwm_set_pin(CLAW_PIN)) < E_OK)              return er;
    if ((er = pwm_set_wrap(CLAW_PIN, CLAW_WRAP)) < E_OK)  return er;
    /* Keep the servo signal low at boot. The mission opens only after it has
       accepted an object marker, avoiding motion while vision is searching. */
    if ((er = pwm_set_cc(CLAW_PIN, 0)) < E_OK) return er;
    if ((er = pwm_set_enabled(CLAW_PIN, TRUE)) < E_OK)    return er;
    claw_deg = -1;
    return E_OK;
}

void claw_angle(INT deg)
{
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    pwm_set_cc(CLAW_PIN, deg_to_us(deg));
    claw_deg = deg;
}

void claw_open(void)  { claw_angle(CLAW_OPEN_DEG); }
void claw_close(void) { claw_angle(CLAW_CLOSE_DEG); }

void claw_release(void)
{
    pwm_set_cc(CLAW_PIN, 0);   /* no pulse: servo stops holding */
    claw_deg = -1;
}

INT claw_get_angle(void) { return claw_deg; }
