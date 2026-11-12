#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "main.h"

/* htim: 1 MHz tick (CCR = pulse in us), 20 ms period */
typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t           channel;
} Servo;

void servo_init(Servo *s, TIM_HandleTypeDef *htim, uint32_t channel);
void servo_set_us(const Servo *s, uint32_t us);
/* deg: steering angle. Positive = left. */
void servo_set_deg(const Servo *s, float deg);

#endif /* SERVO_H */
