#include "hcsr04.h"
#include "constants.h"

static uint32_t elapsed_us(TIM_HandleTypeDef *htim, uint32_t from)
{
  return (__HAL_TIM_GET_COUNTER(htim) - from) & __HAL_TIM_GET_AUTORELOAD(htim);
}

void hcsr04_init(Hcsr04 *h, GPIO_TypeDef *trig_port, uint16_t trig_pin,
                 GPIO_TypeDef *echo_port, uint16_t echo_pin, TIM_HandleTypeDef *htim_us)
{
  h->trig_port   = trig_port;
  h->trig_pin    = trig_pin;
  h->echo_port   = echo_port;
  h->echo_pin    = echo_pin;
  h->htim_us     = htim_us;
  h->rise_us     = 0;
  h->pulse_us    = 0;
  h->new_pulse   = false;
  h->distance_mm = HCSR04_INVALID_MM;

  HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
  HAL_TIM_Base_Start(htim_us);
}

void hcsr04_trigger(Hcsr04 *h)
{
  HAL_GPIO_WritePin(h->trig_port, h->trig_pin, GPIO_PIN_SET);
  uint32_t start = __HAL_TIM_GET_COUNTER(h->htim_us);
  while (elapsed_us(h->htim_us, start) < HCSR04_TRIGGER_US) { }
  HAL_GPIO_WritePin(h->trig_port, h->trig_pin, GPIO_PIN_RESET);
}

void hcsr04_echo_isr(Hcsr04 *h)
{
  if (HAL_GPIO_ReadPin(h->echo_port, h->echo_pin) == GPIO_PIN_SET)
  {
    h->rise_us = __HAL_TIM_GET_COUNTER(h->htim_us);
  }
  else
  {
    h->pulse_us  = elapsed_us(h->htim_us, h->rise_us);
    h->new_pulse = true;
  }
}

bool hcsr04_update(Hcsr04 *h)
{
  if (!h->new_pulse) { return false; }
  h->new_pulse = false;

  uint32_t pulse = h->pulse_us;
  if (pulse >= HCSR04_TIMEOUT_US)
  {
    h->distance_mm = HCSR04_INVALID_MM;
    return true;
  }

  uint32_t mm = (uint32_t)((float)pulse * HCSR04_MM_PER_US);
  h->distance_mm = (mm > HCSR04_MAX_RANGE_MM) ? HCSR04_MAX_RANGE_MM : (uint16_t)mm;
  return true;
}
