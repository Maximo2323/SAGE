#include "pid.h"
#include "math_utils.h"
#include "constants.h"

void pid_init(Pid *p, float kp, float ki, float kd)
{
  pid_set_gains(p, kp, ki, kd);
  pid_set_output_limits(p, -PID_DEFAULT_OUT_LIMIT, PID_DEFAULT_OUT_LIMIT);
  pid_reset(p);
}

void pid_set_gains(Pid *p, float kp, float ki, float kd)
{
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
}

void pid_set_output_limits(Pid *p, float min_out, float max_out)
{
  p->out_min = min_out;
  p->out_max = max_out;
}

void pid_reset(Pid *p)
{
  p->integral   = 0.0f;
  p->prev_error = 0.0f;
  p->has_prev   = false;
}

float pid_update(Pid *p, float setpoint, float measurement, float dt_s)
{
  if (dt_s <= 0.0f) { return 0.0f; }

  const float error = setpoint - measurement;

  float deriv = 0.0f;
  if (p->has_prev) { deriv = (error - p->prev_error) / dt_s; }

  p->prev_error = error;
  p->has_prev   = true;

  const float integral = p->integral + error * dt_s;
  const float raw      = p->kp * error + p->ki * integral + p->kd * deriv;
  const float out      = clampf(raw, p->out_min, p->out_max);

  /* Anti-windup: do not integrate when the output is at a limit and the error pushes it
   * farther past that limit. Integration starts again when the error changes sign. */
  const bool pushes_past_limit = (raw > out && error > 0.0f) || (raw < out && error < 0.0f);
  if (!pushes_past_limit) { p->integral = integral; }

  return out;
}
