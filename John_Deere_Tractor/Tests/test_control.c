/* PC test: gcc -std=c11 -I../App/Control -I../App/Config test_control.c ../App/Control/math_utils.c ../App/Control/pid.c -lm */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include "math_utils.h"
#include "pid.h"

#define NEAR(a, b) (fabsf((a) - (b)) < 1e-3f)

int main(void)
{
  assert(NEAR(wrap180(190.0f), -170.0f));
  assert(NEAR(wrap180(-190.0f), 170.0f));
  assert(NEAR(wrap180(180.0f), 180.0f));
  assert(NEAR(wrap360(-10.0f), 350.0f));
  assert(wrap360(-1e-7f) < 360.0f);
  assert(NEAR(wrap360(720.0f), 0.0f));

  float around_zero[] = {350.0f, 10.0f, 355.0f, 5.0f};
  float m = circular_mean_deg(around_zero, 4);
  assert(m < 0.01f || m > 359.99f);
  float around_90[] = {80.0f, 100.0f};
  assert(NEAR(circular_mean_deg(around_90, 2), 90.0f));

  assert(NEAR(clampf(5.0f, 0.0f, 1.0f), 1.0f));

  Pid p;
  pid_init(&p, 2.0f, 1.0f, 0.5f);
  assert(pid_update(&p, 1.0f, 0.0f, 0.0f) == 0.0f);
  assert(NEAR(pid_update(&p, 1.0f, 0.0f, 0.1f), 2.0f + 0.1f));
  assert(NEAR(pid_update(&p, 1.0f, 0.5f, 0.1f), 1.0f + 0.15f + 0.5f * (-5.0f)));
  pid_set_output_limits(&p, -1.0f, 1.0f);
  assert(NEAR(pid_update(&p, 100.0f, 0.0f, 0.1f), 1.0f));
  pid_reset(&p);
  assert(p.integral == 0.0f && !p.has_prev);

  /* Anti-windup: a long saturation does not fill the integral, so the output recovers at once. */
  Pid pi;
  pid_init(&pi, 1.0f, 1.0f, 0.0f);
  pid_set_output_limits(&pi, -1.0f, 1.0f);
  for (int i = 0; i < 100; i++) { assert(NEAR(pid_update(&pi, 10.0f, 0.0f, 0.1f), 1.0f)); }
  assert(pi.integral == 0.0f);
  assert(NEAR(pid_update(&pi, 0.0f, 0.5f, 0.1f), -0.5f - 0.05f));

  /* Synthetic values, not the motor data: 100 counts in 10 ms at 6000 counts/rev = 100 RPM. */
  assert(NEAR(rpm_from_counts(100, 6000.0f, 0.01f), 100.0f));
  assert(NEAR(rpm_from_counts(-100, 6000.0f, 0.01f), -100.0f));
  assert(rpm_from_counts(100, 6000.0f, 0.0f) == 0.0f);
  /* The encoder count difference stays correct when the counter wraps. */
  assert((int32_t)(5U - 0xFFFFFFFBU) == 10);
  assert((int16_t)(uint16_t)(0xFFF6U - 0x0000U) == -10);

  assert(rear_wheel_ratio(20.0f, 200.0f, 0.0f) == 0.0f);
  assert(rear_wheel_ratio(0.0f, 200.0f, 100.0f) == 0.0f);
  assert(NEAR(rear_wheel_ratio(45.0f, 200.0f, 100.0f), 1.0f));
  assert(NEAR(rear_wheel_ratio(-45.0f, 200.0f, 100.0f), -1.0f));

  assert(deadband_map(0.0f, 20.0f, 90.0f) == 0.0f);
  assert(NEAR(deadband_map(100.0f, 20.0f, 90.0f), 90.0f));
  assert(NEAR(deadband_map(-50.0f, 20.0f, 100.0f), -60.0f));
  assert(NEAR(deadband_map(37.0f, 0.0f, 100.0f), 37.0f));
  return 0;
}
