#ifndef HEADING_HOLD_H
#define HEADING_HOLD_H

#include <stdbool.h>
#include <stdint.h>
#include "bno085.h"
#include "control.h"

/* Firmware side of the heading: the BNO085 calibration and the sample for control.c.
 * The heading controller itself is in control.c. */
typedef struct
{
  Bno085  *imu;
  float    yaw_zero;      /* sensor heading at the calibration: the zero of this run */
  uint32_t reset_count;   /* sensor reset count at the calibration                    */
} HeadingHold;

void heading_hold_init(HeadingHold *h, Bno085 *imu);
/* The vehicle must not move during the calibration. Returns false when the sensor gives no data. */
bool heading_hold_calibrate(HeadingHold *h, int samples, uint32_t warmup_ms);
/* The newest report of HEADING_SOURCE. Not valid when it is stale, or when the sensor did a
 * reset after the calibration (the heading then has a new zero). */
HeadingSample heading_hold_sample(const HeadingHold *h, uint32_t now_ms);

#endif /* HEADING_HOLD_H */
