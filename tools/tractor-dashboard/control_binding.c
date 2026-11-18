/* Host library for the dashboard sim: the firmware control core (control.c, pid.c, math_utils.c)
 * compiled for the PC with the same constants.h. dashboard.py builds and loads it (ctypes).
 * This file only adds access for Python; it has no control logic. */
#include <stddef.h>
#include "control.h"
#include "constants.h"
#include "math_utils.h"

size_t control_size(void) { return sizeof(Control); }

/* The firmware steps heading_hold_sample() + control_steering(), from a simulated BNO085 report. */
void sim_steering(Control *c, float raw_yaw, float yaw_zero, uint32_t count, uint32_t t_ms, int valid)
{
  const HeadingSample s = { heading_relative(raw_yaw, yaw_zero), count, t_ms, valid != 0 };
  control_steering(c, &s);
}

/* constants.h values as compiled, for the dashboard display and the sim plant. */
#define CONFIG(X) \
  X(WHEEL_CONTROL_PERIOD_MS) X(STEERING_CONTROL_PERIOD_MS) X(TELEMETRY_PERIOD_MS) X(COMMAND_TIMEOUT_MS) \
  X(BNO_STALE_TIMEOUT_MS) X(BNO_READ_PERIOD_MS) X(BNO_GAME_RV_INTERVAL_US) X(BT_BAUD) \
  X(HEADING_HOLD_ENABLE) X(HEADING_STEP_DEG) X(HEADING_KP) X(HEADING_KI) X(HEADING_KD) \
  X(HEADING_D_FILTER_TAU_S) X(SERVO_MAX_STEER_DEG) X(SERVO_DIR) X(STEER_STEP_DEG) X(STEER_SLEW_DEG_S) X(HEADING_TARGET_RATE_DEG_S) \
  X(WHEEL_KP) X(WHEEL_KI) X(WHEEL_KD) X(WHEEL_D_FILTER_TAU_S) X(WHEEL_FF_PCT_PER_RPM) \
  X(WHEEL_SPEED_MAX_RPM) X(WHEEL_ACCEL_RPM_PER_S) X(WHEEL_DECEL_RPM_PER_S) X(MOTOR_DEADBAND_PCT) \
  X(MOTOR_DUTY_MAX) X(STALL_MIN_DUTY) X(STALL_TIMEOUT_MS) X(COUNTS_PER_WHEEL_REV) X(ENC_DIR_LEFT) \
  X(ENC_DIR_RIGHT) X(MOTOR_NO_LOAD_RPM_12V) X(WHEEL_DIAMETER_MM) X(TRACK_WIDTH_MM) X(WHEELBASE_MM) \
  X(SIM_MOTOR_GAIN) X(SIM_MOTOR_TAU_S) X(SIM_SERVO_RATE_DEG_S) X(SIM_SERVO_TAU_S) X(SIM_WHEELBASE_MM) \
  X(SIM_TRACK_WIDTH_MM) X(SIM_IMU_PHASE_MS) X(TOF_MAX_RANGE_MM) \
  X(SIM_TOF_FRONT_HEIGHT_MM) X(SIM_TOF_SIDE_LOW_HEIGHT_MM) X(SIM_TOF_SIDE_HIGH_HEIGHT_MM)
#define CONFIG_NAME(n)  #n " "
#define CONFIG_VALUE(n) (double)(n),
const char   config_names[]  = CONFIG(CONFIG_NAME);
const double config_values[] = { CONFIG(CONFIG_VALUE) };

/* The full controller state, for the display and the replay test. */
#define VIEW(X) \
  X(drive_dir) X(speed_pct) X(t_last_key) X(t_tick) X(steer_deg) X(steer_goal) X(cmd_l) X(cmd_r) X(motor_enable) \
  X(target_rpm) X(rpm_l) X(rpm_r) X(last_seq) X(last_count_l) X(last_count_r) X(t_motion_l) \
  X(t_motion_r) X(stalled) X(overruns) X(pid_l.integral) X(pid_l.deriv) X(pid_r.integral) \
  X(pid_r.deriv) X(heading_active) X(imu_ok) X(heading_goal) X(heading_target) X(current_yaw) X(heading_error) \
  X(heading_out) X(prev_yaw) X(last_count) X(last_t_ms) X(have_prev) X(pid_heading.integral) \
  X(pid_heading.deriv)
#define VIEW_NAME(f)  #f " "
#define VIEW_VALUE(f) out[i++] = (double)c->f;
const char view_names[] = VIEW(VIEW_NAME);
void control_view(const Control *c, double *out) { int i = 0; VIEW(VIEW_VALUE) }
