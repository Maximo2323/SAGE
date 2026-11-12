#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct
{
  float kp, ki, kd;
  float out_min, out_max;
  float integral;
  float prev_error;
  bool  has_prev;
} Pid;

void  pid_init(Pid *p, float kp, float ki, float kd);
void  pid_set_gains(Pid *p, float kp, float ki, float kd);
void  pid_set_output_limits(Pid *p, float min_out, float max_out);
void  pid_reset(Pid *p);
float pid_update(Pid *p, float setpoint, float measurement, float dt_s);

#endif /* PID_H */
