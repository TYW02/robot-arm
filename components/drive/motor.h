/*
 * motor.h - two-wheel dual-input motor driver for the µT-Kernel 3 port.
 *
 * Each wheel uses an A/B pair from the Robo Pico's onboard H-bridge. The
 * driver maps MOTOR 1 to GP8/GP9 and MOTOR 2 to GP10/GP11.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <tk/tkernel.h>

/* Robo Pico onboard motor driver inputs (motor 1 = left, motor 2 = right). */
#define MOTOR_L_IN_A   8    /* MOTOR 1: M1A */
#define MOTOR_L_IN_B   9    /* MOTOR 1: M1B */
#define MOTOR_R_IN_A   10   /* MOTOR 2: M2A */
#define MOTOR_R_IN_B   11   /* MOTOR 2: M2B */

/* Flip a motor if it runs backwards for the requested direction. */
#define MOTOR_L_INVERT 0
#define MOTOR_R_INVERT 0

/* 125 MHz / (6249 + 1) = 20 kHz PWM for the Robo Pico H-bridge. */
#define MOTOR_PWM_WRAP 6249

typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 } motor_id_t;

ER   motor_init(void);                       /* call once before use */
void motor_set(motor_id_t m, INT percent);   /* -100..100; sign sets direction */
void motor_drive(INT left_pct, INT right_pct);
void motor_coast(void);
void motor_brake(void);

#endif /* MOTOR_H */
