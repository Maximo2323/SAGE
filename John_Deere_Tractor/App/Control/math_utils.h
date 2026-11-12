#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>

float clampf(float v, float lo, float hi);
float wrap180(float deg);
float wrap360(float deg);
float circular_mean_deg(const float *deg, int n);

/* Wheel speed in RPM. dt_s is the real time between the two encoder count samples. */
float rpm_from_counts(int32_t delta_counts, float counts_per_wheel_rev, float dt_s);

/* Speed difference of the rear wheels in a turn, as a fraction of the vehicle speed.
 * Inner wheel speed = v x (1 - ratio). Outer wheel speed = v x (1 + ratio).
 * The result is 0 when the wheelbase is not set. */
float rear_wheel_ratio(float steer_deg, float track_width, float wheelbase);

/* Change a command of -100 to 100 % into a duty cycle between deadband_pct and max_pct.
 * A command of 0 gives a duty cycle of 0. */
float deadband_map(float command_pct, float deadband_pct, float max_pct);

#endif /* MATH_UTILS_H */
