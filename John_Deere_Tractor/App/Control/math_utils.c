#include "math_utils.h"
#include <math.h>

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f

float clampf(float v, float lo, float hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

float wrap360(float deg)
{
  deg = fmodf(deg, 360.0f);
  if (deg < 0.0f)    { deg += 360.0f; }
  if (deg >= 360.0f) { deg -= 360.0f; }  /* -1e-7 + 360 rounds to 360 */
  return deg;
}

float wrap180(float deg)
{
  deg = wrap360(deg);
  if (deg > 180.0f) { deg -= 360.0f; }
  return deg;
}

float circular_mean_deg(const float *deg, int n)
{
  float s = 0.0f;
  float c = 0.0f;
  for (int i = 0; i < n; i++)
  {
    s += sinf(deg[i] * DEG_TO_RAD);
    c += cosf(deg[i] * DEG_TO_RAD);
  }
  return wrap360(atan2f(s, c) * RAD_TO_DEG);
}

float rpm_from_counts(int32_t delta_counts, float counts_per_wheel_rev, float dt_s)
{
  if (dt_s <= 0.0f || counts_per_wheel_rev <= 0.0f) { return 0.0f; }
  return (float)delta_counts * 60.0f / (counts_per_wheel_rev * dt_s);
}

float deadband_map(float command_pct, float deadband_pct, float max_pct)
{
  if (command_pct == 0.0f) { return 0.0f; }
  const float mag  = clampf(fabsf(command_pct), 0.0f, 100.0f);
  const float duty = deadband_pct + mag * (max_pct - deadband_pct) / 100.0f;
  return (command_pct < 0.0f) ? -duty : duty;
}

float rear_wheel_ratio(float steer_deg, float track_width, float wheelbase)
{
  if (wheelbase <= 0.0f) { return 0.0f; }
  return 0.5f * track_width * tanf(steer_deg * DEG_TO_RAD) / wheelbase;
}
