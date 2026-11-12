#ifndef HEADING_HOLD_H
#define HEADING_HOLD_H

#include <stdbool.h>
#include <stdint.h>
#include "bno085.h"
#include "pid.h"

typedef struct
{
  Bno085  *imu;
  Pid     *pid;
  float    yaw_zero;      /* sensor heading at the calibration: the zero of this run */
  float    prev_yaw_raw;
  float    target;        /* heading target, relative to yaw_zero                     */
  float    current_yaw;   /* heading, relative to yaw_zero                            */
  float    error;         /* wrapped: target - current_yaw, -180 to 180 degrees       */
  float    output;        /* newest steering angle command                            */
  uint32_t last_count;    /* report number used by the last PID update                */
  uint32_t last_t_ms;     /* time of that report                                      */
  bool     have_prev_yaw_raw;
} HeadingHold;

void heading_hold_init(HeadingHold *h, Bno085 *imu, Pid *pid);
/* The vehicle must not move during the calibration. Returns false when the sensor gives no data. */
bool heading_hold_calibrate(HeadingHold *h, int samples, uint32_t warmup_ms);
/* Gives the steering angle command (positive = left) for forward movement.
 * Returns false when the heading data is stale. Then do not use the command. */
bool heading_hold_update(HeadingHold *h, uint32_t now_ms, bool is_moving, float *steer_deg);
void heading_hold_add_to_target(HeadingHold *h, float delta_deg);

#endif /* HEADING_HOLD_H */
