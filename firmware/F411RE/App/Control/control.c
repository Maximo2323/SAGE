#include "control.h"
#include "constants.h"
#include "math_utils.h"
#include <math.h>

/* Immediate stop with no ramp. The PI controllers start again from zero. */
static void motors_stop(Control *c)
{
  c->target_rpm = 0.0f;
  c->cmd_l      = 0.0f;
  c->cmd_r      = 0.0f;
  pid_reset(&c->pid_l);
  pid_reset(&c->pid_r);
}

/* Heading controller state reset: the next valid sample only sets the reference. */
static void heading_reset(Control *c)
{
  pid_reset(&c->pid_heading);
  c->have_prev   = false;
  c->heading_out = 0.0f;
}

void control_init(Control *c, bool imu_ok, uint32_t now_ms)
{
  *c = (Control){0};
  c->speed_pct      = 10;
  c->motor_enable   = true;
  c->t_motion_l     = now_ms;
  c->t_motion_r     = now_ms;
  c->t_tick         = now_ms;
  c->imu_ok         = imu_ok;
  c->heading_active = (HEADING_HOLD_ENABLE != 0) && imu_ok;

  pid_init(&c->pid_l, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_D_FILTER_TAU_S, 100.0f);
  pid_init(&c->pid_r, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_D_FILTER_TAU_S, 100.0f);
  /* The real steering limit: the anti-windup then sees the same saturation as the servo. */
  pid_init(&c->pid_heading, HEADING_KP, HEADING_KI, HEADING_KD, HEADING_D_FILTER_TAU_S,
           SERVO_MAX_STEER_DEG);
}

void control_key(Control *c, char key, uint32_t now_ms)
{
  switch (key)
  {
    /* Hold-to-drive: only the two drive keys start the command timeout again. */
    case 'w': case 'W': c->drive_dir = 1;  c->t_last_key = now_ms; break;
    case 's': case 'S': c->drive_dir = -1; c->t_last_key = now_ms; break;
    case 'a': case 'A':
      if (c->heading_active) { c->heading_goal = wrap180(c->heading_goal + HEADING_STEP_DEG); }
      else                   { c->steer_goal += STEER_STEP_DEG; }
      break;
    case 'd': case 'D':
      if (c->heading_active) { c->heading_goal = wrap180(c->heading_goal - HEADING_STEP_DEG); }
      else                   { c->steer_goal -= STEER_STEP_DEG; }
      break;
    case ' ':
      c->drive_dir      = 0;
      motors_stop(c);
      c->steer_goal     = 0.0f;
      c->heading_goal   = c->current_yaw;  /* hold the present heading */
      c->heading_target = c->current_yaw;
      c->stalled        = false;
      c->motor_enable   = true;
      break;
    case '0': c->speed_pct = 100; break;
    default:
      if (key < '1' || key > '9') { return; }
      c->speed_pct = (int32_t)(key - '0') * 10;
      break;
  }
  c->steer_goal = clampf(c->steer_goal, -SERVO_MAX_STEER_DEG, SERVO_MAX_STEER_DEG);
}

/* Move by delta, but at most rate x dt. rate <= 0 = no limit. */
static float rate_limit(float delta, float rate, float dt_s)
{
  return (rate > 0.0f) ? clampf(delta, -rate * dt_s, rate * dt_s) : delta;
}

void control_tick(Control *c, uint32_t now_ms)
{
  const float dt_s = (float)(now_ms - c->t_tick) / 1000.0f;
  c->t_tick = now_ms;
  /* The target takes the short way around +-180. */
  c->heading_target = wrap180(c->heading_target +
      rate_limit(wrap180(c->heading_goal - c->heading_target), HEADING_TARGET_RATE_DEG_S, dt_s));
  /* shortcut: the heading PID does not see this slew limit in its anti-windup; the target ramp
   * keeps the error small. Feed the slewed range into the PID limits if the integral winds up. */
  c->steer_deg += rate_limit(c->steer_goal - c->steer_deg, STEER_SLEW_DEG_S, dt_s);

  /* Hold-to-drive: stop when no key arrives for COMMAND_TIMEOUT_MS. */
  if (c->drive_dir != 0 && (now_ms - c->t_last_key) >= COMMAND_TIMEOUT_MS) { c->drive_dir = 0; }
  /* Heading hold is selected but the sensor did not start: do not move. */
  if (HEADING_HOLD_ENABLE && !c->imu_ok) { c->drive_dir = 0; }
}

/* Feedforward and PI for one wheel. Returns the motor command in percent. */
static float wheel_command(Pid *pid, float target, float measured, float meas_delta, float dt_s)
{
  if (target == 0.0f)
  {
    pid_reset(pid);
    return 0.0f;
  }
  /* The PI gets only the part of the command range that the feedforward term leaves.
   * Thus its anti-windup operates on the total command. */
  const float ff = WHEEL_FF_PCT_PER_RPM * target;
  pid_set_output_limits(pid, -100.0f - ff, 100.0f - ff);
  return clampf(ff + pid_update(pid, target - measured, meas_delta, dt_s), -100.0f, 100.0f);
}

/* Stall condition: a high command and no encoder counts for STALL_TIMEOUT_MS. */
static bool wheel_stalled(float cmd, int32_t delta_counts, uint32_t *t_motion, uint32_t now_ms)
{
  if (fabsf(cmd) < (float)STALL_MIN_DUTY || delta_counts != 0) { *t_motion = now_ms; }
  return (now_ms - *t_motion) >= STALL_TIMEOUT_MS;
}

void control_wheels(Control *c, uint32_t seq, uint16_t count_l, uint16_t count_r, uint32_t now_ms)
{
  const uint32_t samples = seq - c->last_seq;
  if (samples == 0U) { return; }              /* no new measurement: do not use the old one again */
  if (samples > 1U)  { c->overruns += samples - 1U; }

  /* dt is the real time between the two encoder counts.
   * The 16-bit subtraction gives the correct signed difference when the counter wraps. */
  const float   dt_s    = (float)samples * WHEEL_CONTROL_DT_S;
  const int32_t delta_l = ENC_DIR_LEFT  * (int32_t)(int16_t)(uint16_t)(count_l - c->last_count_l);
  const int32_t delta_r = ENC_DIR_RIGHT * (int32_t)(int16_t)(uint16_t)(count_r - c->last_count_r);
  c->last_seq     = seq;
  c->last_count_l = count_l;
  c->last_count_r = count_r;

  const float old_l = c->rpm_l;
  const float old_r = c->rpm_r;
  c->rpm_l = rpm_from_counts(delta_l, (float)COUNTS_PER_WHEEL_REV, dt_s);
  c->rpm_r = rpm_from_counts(delta_r, (float)COUNTS_PER_WHEEL_REV, dt_s);

  if (c->stalled) { return; }

  /* Ramp the speed target toward the operator command. */
  const float goal = (float)c->drive_dir * (float)c->speed_pct / 100.0f * WHEEL_SPEED_MAX_RPM;
  const float rate = (fabsf(goal) < fabsf(c->target_rpm)) ? WHEEL_DECEL_RPM_PER_S : WHEEL_ACCEL_RPM_PER_S;
  const float step = rate * dt_s;
  c->target_rpm += clampf(goal - c->target_rpm, -step, step);

  /* In a left turn (positive steering angle) the left wheel is the inner wheel. */
  const float ratio = rear_wheel_ratio(c->steer_deg, TRACK_WIDTH_MM, WHEELBASE_MM);
  c->cmd_l = wheel_command(&c->pid_l, c->target_rpm * (1.0f - ratio), c->rpm_l, c->rpm_l - old_l, dt_s);
  c->cmd_r = wheel_command(&c->pid_r, c->target_rpm * (1.0f + ratio), c->rpm_r, c->rpm_r - old_r, dt_s);

  const bool stall_l = wheel_stalled(c->cmd_l, delta_l, &c->t_motion_l, now_ms);
  const bool stall_r = wheel_stalled(c->cmd_r, delta_r, &c->t_motion_r, now_ms);
  if (stall_l || stall_r)
  {
    c->stalled      = true;
    c->drive_dir    = 0;
    c->motor_enable = false;
    motors_stop(c);
  }
}

float heading_relative(float raw_deg, float zero_deg)
{
  return wrap180(wrap360(raw_deg) - zero_deg);
}

/* Returns false when the heading is not valid. Then the vehicle must not move. */
static bool heading_update(Control *c, const HeadingSample *s, bool is_moving)
{
  if (!s->valid)
  {
    heading_reset(c);
    return false;
  }
  /* No new report: keep the last command. Do not use the same measurement again. */
  if (c->have_prev && s->count == c->last_count) { return true; }
  c->last_count = s->count;

  const float yaw    = wrap180(s->yaw_deg);
  c->current_yaw     = yaw;
  c->heading_error   = wrap180(c->heading_target - yaw);

  /* dt is the real time between two reports. The unsigned difference makes a report with an
   * older time a very long gap. */
  const uint32_t gap_ms = s->t_ms - c->last_t_ms;
  if (!c->have_prev || gap_ms > BNO_STALE_TIMEOUT_MS)
  {
    /* First sample, or a gap longer than the stale timeout: the old integral and derivative do
     * not describe the vehicle now. Start again, as after a reset; this sample is the reference. */
    heading_reset(c);
    c->have_prev = true;
    c->prev_yaw  = yaw;
    c->last_t_ms = s->t_ms;
    return true;
  }
  /* Two reports in the same millisecond: no dt. Keep the older reference and the command. */
  if (gap_ms == 0U) { return true; }

  const float dt_s       = (float)gap_ms / 1000.0f;
  const float meas_delta = wrap180(yaw - c->prev_yaw);  /* short way across the +-180 limit */
  c->prev_yaw  = yaw;
  c->last_t_ms = s->t_ms;

  /* The integral must not increase while the vehicle is stopped. */
  if (!is_moving) { pid_reset(&c->pid_heading); }
  c->heading_out = pid_update(&c->pid_heading, c->heading_error, meas_delta, dt_s);
  return true;
}

void control_steering(Control *c, const HeadingSample *s)
{
  if (!c->heading_active) { return; }
  if (!heading_update(c, s, c->target_rpm != 0.0f))
  {
    c->drive_dir = 0;  /* no valid heading: do not move */
    return;
  }
  /* In reverse, the same steering angle turns the heading in the opposite direction. */
  c->steer_goal = (c->target_rpm < 0.0f) ? -c->heading_out : c->heading_out;
}
