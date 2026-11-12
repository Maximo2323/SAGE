/* PC test for the heading logic. Run it from the John_Deere_Tractor folder:
 * gcc -std=c11 -ITests/stubs -IApp/Control -IApp/Config -IApp/Sensors -IApp/Sensors/sh2 \
 *     Tests/test_heading.c App/Control/heading_hold.c App/Control/pid.c App/Control/math_utils.c -lm */
#include <assert.h>
#include <math.h>
#include "heading_hold.h"

#define NEAR(a, b) (fabsf((a) - (b)) < 1e-3f)
#define SRC HEADING_SOURCE

/* Replacements for the HAL and the sensor driver */
static uint32_t tick;
uint32_t HAL_GetTick(void)        { return tick; }
void     HAL_Delay(uint32_t ms)   { tick += ms; }
void     bno085_update(Bno085 *b) { (void)b; }

static Bno085 imu;

static void report(float yaw_deg, uint32_t t_ms)
{
  imu.yaw_deg[SRC] = yaw_deg;
  imu.t_ms[SRC]    = t_ms;
  imu.count[SRC]++;
}

int main(void)
{
  Pid         pid;
  HeadingHold h;
  float       out = 0.0f;

  imu.initialized = true;
  pid_init(&pid, 1.0f, 0.0f, 0.0f);  /* output = heading error */
  heading_hold_init(&h, &imu, &pid);

  /* A report that is newer than "now" is fresh (the main loop reads the time first). */
  report(0.0f, 1002);
  assert(bno085_fresh(&imu, SRC, 1000));
  assert(heading_hold_update(&h, 1000, true, &out));

  /* Wrapped error: target 175, heading -175. The short way is -10 degrees, not +350. */
  h.target = 175.0f;
  report(-175.0f, 1012);
  assert(heading_hold_update(&h, 1012, true, &out));
  assert(NEAR(h.error, -10.0f) && NEAR(out, -10.0f));

  /* No new report: the command stays, and the PID does not run again. */
  h.target = 0.0f;
  assert(heading_hold_update(&h, 1020, true, &out));
  assert(NEAR(out, -10.0f));

  /* A new report: the command changes. */
  report(-175.0f, 1022);
  assert(heading_hold_update(&h, 1022, true, &out));
  assert(NEAR(out, 175.0f));

  /* A report older than BNO_STALE_TIMEOUT_MS is not valid. */
  assert(!bno085_fresh(&imu, SRC, 1022 + BNO_STALE_TIMEOUT_MS));
  assert(!heading_hold_update(&h, 1022 + BNO_STALE_TIMEOUT_MS, true, &out));

  /* A sensor reset after the calibration makes the heading not valid, also with fresh data. */
  report(0.0f, 2000);
  assert(heading_hold_update(&h, 2000, true, &out));
  imu.reset_count++;
  report(0.0f, 2010);
  assert(!heading_hold_update(&h, 2010, true, &out));

  /* The calibration accepts the new sensor state. */
  tick = 2010;
  assert(heading_hold_calibrate(&h, 4, 0));
  report(0.0f, tick);
  assert(heading_hold_update(&h, tick, true, &out));
  return 0;
}
