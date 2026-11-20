#include "heading_hold.h"
#include "math_utils.h"
#include "constants.h"
#include "main.h"

void heading_hold_init(HeadingHold *h, Bno085 *imu)
{
  h->imu         = imu;
  h->yaw_zero    = 0.0f;
  h->reset_count = 0;
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

  h->yaw_zero    = circular_mean_deg(buf, samples);
  h->reset_count = h->imu->reset_count;
  return true;
}

HeadingSample heading_hold_sample(const HeadingHold *h, uint32_t now_ms)
{
  const Bno085 *imu = h->imu;
  return (HeadingSample){
    .yaw_deg = heading_relative(imu->yaw_deg[HEADING_SOURCE], h->yaw_zero),
    .count   = imu->count[HEADING_SOURCE],
    .t_ms    = imu->t_ms[HEADING_SOURCE],
    .valid   = bno085_fresh(imu, HEADING_SOURCE, now_ms) && imu->reset_count == h->reset_count,
  };
}
