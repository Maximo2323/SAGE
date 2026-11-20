#include "servo.h"
#include "constants.h"
#include "math_utils.h"

void servo_init(Servo *s, TIM_HandleTypeDef *htim, uint32_t channel)
{
  s->htim    = htim;
  s->channel = channel;
  servo_set_us(s, SERVO_CENTER_US);
  HAL_TIM_PWM_Start(htim, channel);
}

void servo_set_us(const Servo *s, uint32_t us)
{
  if (us < SERVO_MIN_US) { us = SERVO_MIN_US; }
  if (us > SERVO_MAX_US) { us = SERVO_MAX_US; }
  __HAL_TIM_SET_COMPARE(s->htim, s->channel, us);
}

void servo_set_deg(const Servo *s, float deg)
{
  deg = clampf(deg * SERVO_DIR, -SERVO_MAX_STEER_DEG, SERVO_MAX_STEER_DEG);
  float span = (deg >= 0.0f) ? (float)(SERVO_MAX_US - SERVO_CENTER_US)
                             : (float)(SERVO_CENTER_US - SERVO_MIN_US);
  servo_set_us(s, (uint32_t)((float)SERVO_CENTER_US + deg / SERVO_MAX_STEER_DEG * span + 0.5f));
}
