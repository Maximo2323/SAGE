/* PC test for the control core (control.c). Run it from the John_Deere_Tractor folder:
 * gcc -std=c11 -IApp/Control -IApp/Config Tests/test_core.c App/Control/control.c \
 *     App/Control/pid.c App/Control/math_utils.c -lm
 * The heading tests set their own gains, so they do not depend on the tuning in constants.h. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "control.h"
#include "constants.h"
#include "math_utils.h"

#define NEAR(a, b) (fabsf((a) - (b)) < 1e-3f)

static uint32_t reports;

static void sample(Control *c, float yaw, uint32_t t_ms)
{
  const HeadingSample s = { yaw, ++reports, t_ms, true };
  control_steering(c, &s);
}

/* Heading hold on, vehicle moving forward, given gains. The first sample sets the reference. */
static void setup(Control *c, float kp, float ki, float kd, float tau)
{
  control_init(c, true, 0);
  c->heading_active = true;
  pid_init(&c->pid_heading, kp, ki, kd, tau, SERVO_MAX_STEER_DEG);
  c->target_rpm = 50.0f;
  sample(c, 0.0f, 0);
  assert(c->heading_out == 0.0f);  /* startup: the first sample only sets the reference */
}

/* Noiseless closed loop: kinematic bicycle at a fixed speed, ideal servo, 20 ms samples.
 * Placeholder plant for the discrete behavior of the controller only, not the tractor.
 * Returns the number of sign changes of the steering command. */
static int steering_flips(float kd, float tau)
{
  Control c;
  setup(&c, 0.6f, 0.5f, kd, tau);
  const float v_mm_s = 340.0f, wheelbase_mm = 300.0f, dt = 0.02f;
  float yaw = 0.0f, last = 0.0f;
  int flips = 0;
  c.heading_target = 30.0f;
  for (uint32_t i = 1; i <= 500; i++)
  {
    yaw += v_mm_s * tanf(c.steer_goal * 0.017453292f) / wheelbase_mm * 57.29578f * dt;
    sample(&c, wrap180(yaw), i * 20U);
    if (c.steer_goal * last < 0.0f) { flips++; }
    if (c.steer_goal != 0.0f) { last = c.steer_goal; }
  }
  return flips;
}

int main(void)
{
  Control c;

  /* 1. Saturation recovery: 60 degrees of error for 10 s. The PID limit is the real steering
   *    limit, so the integral does not grow. When the target is passed, the command reverses
   *    at once (with a 255 degree internal limit, ~438 degree-seconds of integral kept it at +30). */
  setup(&c, 0.6f, 0.5f, 0.0f, 0.0f);
  c.heading_target = 60.0f;
  for (uint32_t t = 20; t <= 10000; t += 20) { sample(&c, 0.0f, t); }
  assert(c.steer_goal == SERVO_MAX_STEER_DEG && c.pid_heading.integral == 0.0f);
  sample(&c, 65.0f, 10020);
  assert(NEAR(c.steer_goal, -0.6f * 5.0f - 0.5f * 5.0f * 0.02f));

  /* 2. Target step: no derivative kick (the D term uses the measurement only). */
  setup(&c, 0.6f, 0.0f, 0.5f, 0.0f);
  sample(&c, 0.0f, 20);
  control_key(&c, 'a', 25);               /* goal + HEADING_STEP_DEG; the target ramps to it */
  control_tick(&c, 125);
  const float ramp = (HEADING_TARGET_RATE_DEG_S > 0.0f) ? fminf(HEADING_STEP_DEG, HEADING_TARGET_RATE_DEG_S * 0.125f)
                                                         : HEADING_STEP_DEG;
  assert(c.heading_goal == HEADING_STEP_DEG && NEAR(c.heading_target, ramp));
  sample(&c, 0.0f, 140);
  assert(c.pid_heading.deriv == 0.0f);
  assert(NEAR(c.steer_goal, clampf(0.6f * ramp, -SERVO_MAX_STEER_DEG, SERVO_MAX_STEER_DEG)));

  /* 3. Angle boundaries. The heading passes +180 -> -180: the measurement change is +0.2
   *    degrees, not -359.8. The error passes its +-180 branch: no D spike. Exactly 180 = -180. */
  setup(&c, 0.0f, 0.0f, 1.0f, 0.0f);
  sample(&c, 179.9f, 20);
  sample(&c, 179.9f, 40);
  sample(&c, -179.9f, 60);
  assert(fabsf(c.pid_heading.deriv + 10.0f) < 0.05f);
  assert(fabsf(c.steer_goal + 10.0f) < 0.05f);
  setup(&c, 0.1f, 0.0f, 0.0f, 0.0f);
  c.heading_target = 180.0f;
  assert(wrap180(c.heading_target) == -180.0f);
  sample(&c, 0.0f, 20);
  assert(c.heading_error == -180.0f && NEAR(c.steer_goal, -18.0f));

  /* 4. Variable dt: the integral and the derivative use the real time between reports. */
  setup(&c, 0.0f, 1.0f, 0.0f, 0.0f);
  c.heading_target = 10.0f;
  sample(&c, 0.0f, 10); sample(&c, 0.0f, 40); sample(&c, 0.0f, 60);
  assert(NEAR(c.pid_heading.integral, 10.0f * (0.01f + 0.03f + 0.02f)));
  setup(&c, 0.0f, 0.0f, 0.1f, 0.0f);
  sample(&c, 1.0f, 10);
  assert(NEAR(c.steer_goal, -10.0f));      /* 1 degree in 10 ms */
  sample(&c, 2.0f, 40);
  assert(NEAR(c.steer_goal, -10.0f / 3.0f)); /* 1 degree in 30 ms */
  setup(&c, 0.0f, 0.0f, 0.1f, 0.05f);
  sample(&c, 1.0f, 10);
  assert(NEAR(c.steer_goal, -10.0f * 0.01f / 0.06f));  /* filtered: alpha = dt / (tau + dt) */

  /* 5. Repeated, same-millisecond, long-gap and invalid samples. */
  setup(&c, 0.6f, 0.5f, 0.0f, 0.0f);
  c.heading_target = 10.0f;
  sample(&c, 0.0f, 20);
  const float out = c.steer_goal, integral = c.pid_heading.integral;
  HeadingSample same = { 5.0f, reports, 40, true };   /* the same report number again */
  control_steering(&c, &same);
  assert(c.steer_goal == out && c.pid_heading.integral == integral);
  sample(&c, 5.0f, 20);                   /* new report, same ms: no dt, command kept */
  assert(c.steer_goal == out && c.pid_heading.integral == integral);
  sample(&c, 0.0f, 20 + BNO_STALE_TIMEOUT_MS + 1U);  /* long gap: restart, centered */
  assert(c.steer_goal == 0.0f && c.pid_heading.integral == 0.0f && c.have_prev);
  sample(&c, 0.0f, 40 + BNO_STALE_TIMEOUT_MS + 1U);  /* normal again, dt = 20 ms */
  assert(NEAR(c.pid_heading.integral, 10.0f * 0.02f));
  sample(&c, 0.0f, 10U);                  /* report time before the reference: also a restart */
  assert(c.pid_heading.integral == 0.0f && c.heading_out == 0.0f);
  c.drive_dir = 1;
  const float steer_before = c.steer_goal;
  HeadingSample bad = { 0.0f, ++reports, 60, false };
  control_steering(&c, &bad);
  assert(c.drive_dir == 0 && !c.have_prev && c.heading_out == 0.0f && c.steer_goal == steer_before);

  /* 6. Stop and restart. Stopped: the integral never grows past one step. Space holds the
   *    present heading. Keys W, digit, Space and the command timeout. */
  setup(&c, 0.6f, 0.5f, 0.0f, 0.0f);
  c.target_rpm = 0.0f;
  c.heading_target = 20.0f;
  for (uint32_t t = 20; t <= 2000; t += 20) { sample(&c, 0.0f, t); }
  assert(NEAR(c.pid_heading.integral, 20.0f * 0.02f));
  control_key(&c, ' ', 2001);
  assert(c.heading_target == c.current_yaw && c.heading_goal == c.current_yaw);
  assert(c.drive_dir == 0 && c.target_rpm == 0.0f && c.steer_goal == 0.0f);
  control_key(&c, '5', 2002);
  control_key(&c, 'w', 2002);
  assert(c.drive_dir == 1 && c.speed_pct == 50);
  control_tick(&c, 2002 + COMMAND_TIMEOUT_MS - 1U);
  assert(c.drive_dir == 1);
  control_tick(&c, 2002 + COMMAND_TIMEOUT_MS);
  assert(c.drive_dir == 0);
  control_key(&c, 'x', 2400);             /* other keys do nothing */
  assert(c.speed_pct == 50);

  /* 7. Reverse: the same heading error gives the opposite steering angle. */
  setup(&c, 0.6f, 0.0f, 0.0f, 0.0f);
  c.heading_target = 10.0f;
  c.target_rpm = -50.0f;
  sample(&c, 0.0f, 20);
  assert(NEAR(c.heading_out, 6.0f) && NEAR(c.steer_goal, -6.0f));

  /* 8. High D, no noise (placeholder plant). Without the filter, Kd = 2 gives steering that
   *    alternates sign at each sample; the D filter in constants.h changes that. This shows the
   *    sampled loop only: servo lag and the real vehicle are not in this test. */
  const int flips_raw = steering_flips(2.0f, 0.0f);
  const int flips_filtered = steering_flips(2.0f, 0.05f);
  const int flips_no_d = steering_flips(0.0f, 0.0f);
  printf("steering sign changes in 500 samples: Kd=0 %d, Kd=2 unfiltered %d, Kd=2 tau=0.05 s %d\n",
         flips_no_d, flips_raw, flips_filtered);
  assert(flips_raw > 400);
  assert(flips_no_d < 10 && flips_filtered < 10);

  /* 9. Wheels: encoder counter wrap and direction, ramp, feedforward + PI, overruns, stall. */
  control_init(&c, true, 0);
  control_key(&c, '5', 0);
  control_key(&c, 'w', 0);
  uint16_t count = 0;
  control_wheels(&c, 1, count, count, 10);
  const float step = WHEEL_ACCEL_RPM_PER_S * WHEEL_CONTROL_DT_S;
  assert(NEAR(c.target_rpm, step));
  assert(NEAR(c.cmd_l, WHEEL_FF_PCT_PER_RPM * step + WHEEL_KP * step + WHEEL_KI * step * WHEEL_CONTROL_DT_S));
  control_wheels(&c, 1, 5, 5, 20);        /* same sample number: nothing changes */
  assert(NEAR(c.target_rpm, step));
  c.last_count_l = c.last_count_r = 0xFFF0U;
  control_wheels(&c, 3, 0x0010U, 0x0010U, 30);  /* wrap: +32 counts in 2 samples */
  assert(c.overruns == 1U);
  assert(NEAR(c.rpm_l, ENC_DIR_LEFT * rpm_from_counts(32, (float)COUNTS_PER_WHEEL_REV, 2.0f * WHEEL_CONTROL_DT_S)));
  /* Stall: a high command and no counts for STALL_TIMEOUT_MS. */
  control_init(&c, true, 0);
  control_key(&c, '0', 0);
  uint32_t seq = 0, t = 0;
  while (!c.stalled && t < 5000U) { t += 10U; control_key(&c, 'w', t); control_wheels(&c, ++seq, 0, 0, t); }
  assert(c.stalled && !c.motor_enable && c.cmd_l == 0.0f && c.drive_dir == 0);
  control_key(&c, ' ', t);
  assert(!c.stalled && c.motor_enable);

  /* 11. Smooth steering: the servo command moves at STEER_SLEW_DEG_S toward the request, and the
   *     heading target ramps toward the key target the short way across +-180. */
  control_init(&c, true, 1000);
  c.steer_goal = 30.0f;
  control_tick(&c, 1100);
  assert(NEAR(c.steer_deg, (STEER_SLEW_DEG_S > 0.0f) ? fminf(30.0f, STEER_SLEW_DEG_S * 0.1f) : 30.0f));
  control_tick(&c, 11100);
  assert(c.steer_deg == 30.0f);
  c.heading_target = 170.0f;
  c.heading_goal = -170.0f;               /* 20 degrees to the left, across +180 */
  control_tick(&c, 11110);
  assert(HEADING_TARGET_RATE_DEG_S <= 0.0f || c.heading_target > 170.0f || c.heading_target < -170.0f);
  control_tick(&c, 21110);
  assert(NEAR(c.heading_target, -170.0f));

  /* 10. IMU did not start: heading hold off, no driving when heading hold is selected. */
  control_init(&c, false, 0);
  assert(!c.heading_active);
  control_key(&c, 'w', 0);
  control_tick(&c, 1);
  assert(c.drive_dir == (HEADING_HOLD_ENABLE ? 0 : 1));
  return 0;
}
