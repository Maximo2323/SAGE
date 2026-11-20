#ifndef HCSR04_H
#define HCSR04_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#define HCSR04_INVALID_MM 0xFFFFU

/* htim_us: free-running timer at 1 MHz with ARR at its maximum value */
typedef struct
{
  GPIO_TypeDef      *trig_port;
  uint16_t           trig_pin;
  GPIO_TypeDef      *echo_port;
  uint16_t           echo_pin;
  TIM_HandleTypeDef *htim_us;
  volatile uint32_t  rise_us;
  volatile uint32_t  pulse_us;
  volatile bool      new_pulse;
  uint16_t           distance_mm;
} Hcsr04;

void hcsr04_init(Hcsr04 *h, GPIO_TypeDef *trig_port, uint16_t trig_pin,
                 GPIO_TypeDef *echo_port, uint16_t echo_pin, TIM_HandleTypeDef *htim_us);
void hcsr04_trigger(Hcsr04 *h);
void hcsr04_echo_isr(Hcsr04 *h);
bool hcsr04_update(Hcsr04 *h);

#endif /* HCSR04_H */
