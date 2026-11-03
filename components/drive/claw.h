/*
 * claw.h - claw servo driver (µT-Kernel 3 port, no Pico SDK)
 *
 * Standard hobby servo on GP13 (Robo Pico servo port): 50 Hz, 0.5-2.5 ms pulse.
 * GP13 = PWM slice 6 channel B; the wheel driver uses slices 4 and 5, so no clash.
 *
 * Calibrate: use 'claw <angle>' in the drive test to find the angles where the
 * claw is fully open and where it just grips an object, then set them below.
 * Do not set CLOSE past the point where the jaws touch: a stalled servo draws
 * high current and heats up.
 */
#ifndef CLAW_H
#define CLAW_H

#include <tk/tkernel.h>

#define CLAW_PIN        13

#define CLAW_OPEN_DEG   30    /* <- calibrate */
#define CLAW_CLOSE_DEG  50    /* calibrated for the 3 cm block with clearance */

/* Pulse range of the servo (SG90/MG90S style). Narrow it if the servo
   buzzes or hits its end stop at 0 or 180 degrees. */
#define CLAW_MIN_US     500   /* 0 degrees */
#define CLAW_MAX_US     2500  /* 180 degrees */

ER   claw_init(void);           /* call once; PWM ready, output held low */
void claw_angle(INT deg);       /* 0..180 */
void claw_open(void);
void claw_close(void);
void claw_release(void);        /* stop pulses: servo goes limp */
INT  claw_get_angle(void);      /* last commanded angle, -1 if released */

#endif /* CLAW_H */
