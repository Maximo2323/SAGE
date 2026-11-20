/* PC test for the firmware side of the heading (heading_hold.c). Run it from the John_Deere_Tractor folder:
 * gcc -std=c11 -ITests/stubs -IApp/Control -IApp/Config -IApp/Sensors -IApp/Sensors/sh2 \
 *     Tests/test_heading.c App/Control/heading_hold.c App/Control/control.c App/Control/pid.c \
 *     App/Control/math_utils.c -lm */
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
  HeadingHold h;
  heading_hold_init(&h, &imu);
  imu.initialized = true;

  /* No report yet: not valid. */
  assert(!heading_hold_sample(&h, 0).valid);

  /* A report that is newer than "now" is fresh (the main loop reads the time first). */
  report(10.0f, 1002);
  HeadingSample s = heading_hold_sample(&h, 1000);
  assert(s.valid && s.count == 1 && s.t_ms == 1002 && NEAR(s.yaw_deg, 10.0f));

  /* The yaw is relative to the zero, in [-180, 180). */
  h.yaw_zero = 350.0f;
  report(-175.0f, 1010);
  s = heading_hold_sample(&h, 1010);
  assert(NEAR(s.yaw_deg, -165.0f));
  report(170.0f, 1020);
  assert(heading_hold_sample(&h, 1020).yaw_deg == -180.0f);

  /* A report older than BNO_STALE_TIMEOUT_MS is not valid. */
  assert(!heading_hold_sample(&h, 1020 + BNO_STALE_TIMEOUT_MS).valid);

  /* A sensor reset after the calibration makes the heading not valid, also with fresh data. */
  imu.reset_count++;
  report(0.0f, 2010);
  assert(!heading_hold_sample(&h, 2010).valid);

  /* The calibration accepts the new sensor state and sets the zero (circular mean). */
  tick = 2010;
  imu.yaw_deg[SRC] = -5.0f;  /* bno085_update is a stub: the same value for each sample */
  imu.t_ms[SRC] = 2010 + 40 + 2000;
  assert(heading_hold_calibrate(&h, 4, 2000));
  assert(NEAR(h.yaw_zero, 355.0f) && h.reset_count == imu.reset_count);
  s = heading_hold_sample(&h, tick);
  assert(s.valid && NEAR(s.yaw_deg, 0.0f));
  return 0;
}
