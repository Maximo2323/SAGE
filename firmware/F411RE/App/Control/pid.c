#include "pid.h"
#include "math_utils.h"

void pid_init(Pid *p, float kp, float ki, float kd, float d_tau_s, float out_limit)
{
  p->kp      = kp;
  p->ki      = ki;
  p->kd      = kd;
  p->d_tau_s = (d_tau_s > 0.0f) ? d_tau_s : 0.0f;
  pid_set_output_limits(p, -out_limit, out_limit);
  pid_reset(p);
}

void pid_set_output_limits(Pid *p, float min_out, float max_out)
{
  p->out_min = min_out;
  p->out_max = max_out;
}

void pid_reset(Pid *p)
{
  p->integral = 0.0f;
  p->deriv    = 0.0f;
}

float pid_update(Pid *p, float error, float meas_delta, float dt_s)
{
  if (!(dt_s > 0.0f)) { return 0.0f; }

  /* d(error)/dt without the setpoint part = -d(measurement)/dt.
   * First-order low-pass filter with the real dt: alpha = dt / (tau + dt). */
  const float d_raw = -meas_delta / dt_s;
  p->deriv += dt_s / (p->d_tau_s + dt_s) * (d_raw - p->deriv);

  const float integral = p->integral + error * dt_s;
  const float raw      = p->kp * error + p->ki * integral + p->kd * p->deriv;
  const float out      = clampf(raw, p->out_min, p->out_max);

  /* Anti-windup: do not integrate when the output is at a limit and the error pushes it
   * farther past that limit. Integration starts again when the error changes sign. */
  const bool pushes_past_limit = (raw > out && error > 0.0f) || (raw < out && error < 0.0f);
  if (!pushes_past_limit) { p->integral = integral; }

  return out;
}
