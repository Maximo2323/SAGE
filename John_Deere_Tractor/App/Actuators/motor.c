#include "motor.h"
#include "constants.h"
#include "math_utils.h"
#include <math.h>

void motor_init(Motor *m, TIM_HandleTypeDef *htim, uint32_t ch_fwd, uint32_t ch_rev)
{
  m->htim   = htim;
  m->ch_fwd = ch_fwd;
  m->ch_rev = ch_rev;

  motor_set(m, 0.0f);
  HAL_TIM_PWM_Start(htim, ch_fwd);
  HAL_TIM_PWM_Start(htim, ch_rev);
}

void motor_set(const Motor *m, float command_pct)
{
  const float duty = deadband_map(command_pct, MOTOR_DEADBAND_PCT, (float)MOTOR_DUTY_MAX);
  uint32_t ccr = (uint32_t)(fabsf(duty) * (float)(__HAL_TIM_GET_AUTORELOAD(m->htim) + 1U) / 100.0f);

  /* Only one input has PWM. The other input stays low. */
  __HAL_TIM_SET_COMPARE(m->htim, m->ch_fwd, (duty > 0.0f) ? ccr : 0U);
  __HAL_TIM_SET_COMPARE(m->htim, m->ch_rev, (duty < 0.0f) ? ccr : 0U);
}
