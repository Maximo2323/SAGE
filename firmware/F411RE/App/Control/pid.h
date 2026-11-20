#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct
{
  float kp, ki, kd;
  float d_tau_s;          /* time constant of the D low-pass filter, seconds. 0 = no filter */
  float out_min, out_max;
  float integral;
  float deriv;            /* filtered rate of change of the error, without setpoint changes */
} Pid;

void  pid_init(Pid *p, float kp, float ki, float kd, float d_tau_s, float out_limit);
void  pid_set_output_limits(Pid *p, float min_out, float max_out);
void  pid_reset(Pid *p);

/* error      = setpoint - measurement.
 * meas_delta = measurement - previous measurement. The caller calculates it, so that the heading
 *              controller can use the short angle difference. The D term uses only this value:
 *              a setpoint step gives no D kick. After a reset the D filter starts from 0.
 * dt_s       = real time since the previous update. dt_s <= 0 (or NaN) gives 0 and changes nothing. */
float pid_update(Pid *p, float error, float meas_delta, float dt_s);

#endif /* PID_H */
