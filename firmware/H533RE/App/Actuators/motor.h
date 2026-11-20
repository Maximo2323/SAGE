#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "main.h"

/* BTS7960: ch_fwd is the PWM for RPWM, ch_rev is the PWM for LPWM.
 * The R_EN and L_EN pins of the driver connect to the MOTOR_EN output. */
typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t           ch_fwd;
  uint32_t           ch_rev;
} Motor;

void motor_init(Motor *m, TIM_HandleTypeDef *htim, uint32_t ch_fwd, uint32_t ch_rev);
/* command_pct: -100 to 100 %. Negative = reverse. The deadband is applied here. */
void motor_set(const Motor *m, float command_pct);

#endif /* MOTOR_H */
