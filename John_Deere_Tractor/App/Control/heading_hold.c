#include "heading_hold.h"
#include "math_utils.h"
#include "constants.h"
#include "main.h"
#include <math.h>

void heading_hold_init(HeadingHold *h, Bno085 *imu, Pid *pid)
{
  h->imu               = imu;
  h->pid               = pid;
  h->yaw_zero          = 0.0f;
  h->target            = 0.0f;
  h->current_yaw       = 0.0f;
  h->error             = 0.0f;
  h->output            = 0.0f;
  h->last_count        = 0;
  h->last_t_ms         = 0;
  h->reset_count       = 0;
  h->have_prev         = false;
}

bool heading_hold_calibrate(HeadingHold *h, int samples, uint32_t warmup_ms)
{
  if (samples <= 0 || samples > HEADING_MAX_SAMPLES) { return false; }

  uint32_t warm_start = HAL_GetTick();
  while (HAL_GetTick() - warm_start < warmup_ms)
  {
    bno085_update(h->imu);
    HAL_Delay(HEADING_SAMPLE_DELAY_MS);
  }

  float buf[HEADING_MAX_SAMPLES];
  for (int i = 0; i < samples; i++)
  {
    bno085_update(h->imu);
    buf[i] = wrap360(h->imu->yaw_deg[HEADING_SOURCE]);
    HAL_Delay(HEADING_SAMPLE_DELAY_MS);
  }
  if (!bno085_fresh(h->imu, HEADING_SOURCE, HAL_GetTick())) { return false; }

  h->yaw_zero          = circular_mean_deg(buf, samples);
  h->have_prev         = false;
  h->reset_count       = h->imu->reset_count;
  h->target            = 0.0f;
  pid_reset(h->pid);
  return true;
}

bool heading_hold_update(HeadingHold *h, uint32_t now_ms, bool is_moving, float *steer_deg)
{
  const Bno085 *imu = h->imu;

  /* After a sensor reset the heading has a new zero. It is not valid until a new calibration. */
  if (!bno085_fresh(imu, HEADING_SOURCE, now_ms) || imu->reset_count != h->reset_count)
  {
    pid_reset(h->pid);
    h->have_prev         = false;
    h->output            = 0.0f;
    return false;
  }

  /* No new report: keep the last command. Do not use the same measurement again. */
  const uint32_t count = imu->count[HEADING_SOURCE];
  if (h->have_prev && count == h->last_count)
  {
    *steer_deg = h->output;
    return true;
  }

  const float    yaw_raw = wrap360(imu->yaw_deg[HEADING_SOURCE]);
  const uint32_t t_ms    = imu->t_ms[HEADING_SOURCE];
  float          dt_s    = 0.0f;

  if (h->have_prev) { dt_s = (float)(t_ms - h->last_t_ms) / 1000.0f; }  /* real time between the two reports */
  h->have_prev  = true;
  h->last_count = count;
  h->last_t_ms  = t_ms;

  h->current_yaw = wrap180(yaw_raw - h->yaw_zero);
  h->error       = wrap180(h->target - h->current_yaw);

  /* The integral must not increase while the vehicle is stopped. */
  if (!is_moving) { pid_reset(h->pid); }

  /* The wrapped error goes in as the setpoint, with a measurement of 0. Thus the PID error
   * is h->error and it stays correct across the 0/360 degree limit.
   * Positive output = steer left. */
  if (dt_s > 0.0f) { h->output = pid_update(h->pid, h->error, 0.0f, dt_s); }

  *steer_deg = h->output;
  return true;
}

void heading_hold_add_to_target(HeadingHold *h, float delta_deg)
{
  h->target = wrap180(h->target + delta_deg);
}
