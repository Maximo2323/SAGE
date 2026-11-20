/* Route runner. See route.h. */
#include "route.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "constants.h"
#include "math_utils.h"

#define DEG2RAD(d) ((d) * 0.017453292519943295f)

static const char *const STEP_NAME[] = { "drive", "turn", "find", "checkpoint", "align" };
static const char *const SENSOR_NAME[] = { "front", "right_low", "right_high" };

void route_config_default(RouteConfig *cfg)
{
  /* 0 in constants.h = not measured yet: use the sim values, as the dashboard does. */
  cfg->wheelbase_mm        = (WHEELBASE_MM > 0.0f) ? WHEELBASE_MM : SIM_WHEELBASE_MM;
  cfg->track_mm            = (TRACK_WIDTH_MM > 0.0f) ? TRACK_WIDTH_MM : SIM_TRACK_WIDTH_MM;
  cfg->servo_max_steer_deg = SERVO_MAX_STEER_DEG;
  cfg->avoid_stop_mm       = AVOID_STOP_MM;
  cfg->avoid_wait_s        = AVOID_WAIT_S;
  cfg->avoid_back_mm       = AVOID_BACK_MM;
  cfg->avoid_swerve_deg    = AVOID_SWERVE_DEG;
  cfg->avoid_margin_mm     = AVOID_MARGIN_MM;
  cfg->avoid_max_offset_mm = AVOID_MAX_OFFSET_MM;
  cfg->avoid_look_mm       = AVOID_LOOK_MM;
  cfg->avoid_pct           = AVOID_PCT;
}

void route_start(Route *r, const RouteStep *steps, int n, int first, bool loop,
                 const RouteConfig *cfg, RouteReadTof read_tof, void *ctx)
{
  memset(r, 0, sizeof *r);
  r->steps = steps; r->n = n; r->i = first; r->lap = 1;
  r->loop = loop && n > 0;
  r->running = true;
  r->cfg = *cfg;
  r->read_tof = read_tof; r->ctx = ctx;
  r->mm[0] = r->mm[1] = r->mm[2] = -1;
  r->reading = -1;
}

void route_stop(Route *r) { r->running = false; }

static void say(Route *r, int report, const char *fmt, ...)
{
  if (r->nmsg >= ROUTE_MSG_MAX) { return; }
  r->report[r->nmsg] = (int16_t)report;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(r->msg[r->nmsg], ROUTE_MSG_LEN, fmt, ap);
  va_end(ap);
  r->nmsg++;
}

static void key(Route *r, char k)
{
  if (r->nkeys < sizeof r->keys) { r->keys[r->nkeys++] = k; }
}

/* Speed digit when it changes, then W or S. W/S again each 100 ms, before COMMAND_TIMEOUT_MS. */
static void go(Route *r, uint32_t now, int direction, float pct)
{
  const char k = (char)('0' + ((int)pct / 10) % 10);
  if (k != r->speed_key) { key(r, k); r->speed_key = k; }
  if (direction != r->dir || now - r->t_key >= 100U)
  {
    key(r, direction > 0 ? 'w' : 's');
    r->dir = direction; r->t_key = now;
  }
}

/* End of a step. Space: zero output now. Space also sets the heading goal to the yaw; the next
 * route_step() sets it again. */
static void finish(Route *r, const char *text)
{
  say(r, -1, "%d %s: %s", r->i + 1, STEP_NAME[r->steps[r->i].type], text);
  key(r, ' ');
  r->dir = 0; r->i++; r->begun = false;
}

static void fail(Route *r, const char *text)
{
  char buf[ROUTE_MSG_LEN];
  snprintf(buf, sizeof buf, "FAILED, %s", text);
  finish(r, buf);
  r->running = false;
}

/* Obstacle ahead. Stop and wait avoid_wait_s: gone = go on. Still there = go around on the left.
 * All positions are in the frame of the step line: fwd along it, off to its left, from the wheel
 * distance and the IMU.
 * reverse: straight back until the car front is avoid_back_mm from the obstacle face (an Ackermann
 *          car needs room to move sideways).
 * out:     swerve +avoid_swerve_deg. Each front hit is a point of the obstacle. Keep the leftmost
 *          (edge) and the nearest (face). Go on until the front is clear and off >= edge + half car
 *          + margin. Right front corner within the margin of the face = no room: fail.
 * pass:    parallel, until the right sensors lose it + rear overhang + margin.
 * back:    swerve -avoid_swerve_deg to the line. straight: on until the heading is within 3 deg
 *          of the line.
 * A new obstacle in pass or back: stop, wait; still there = new face, reverse again. The edge keeps
 * the leftmost point of both. The return is to the first line. */
static void avoiding(Route *r, uint32_t now, float a)
{
  const RouteConfig *c = &r->cfg;
  const int32_t front = r->mm[0];
  const bool    sample = r->fresh;
  const float   L = c->wheelbase_mm, W = ROUTE_HALF_W * c->track_mm;
  const float   hit_r = ROUTE_FRONT_X * L + (float)front;  /* rear axle to the hit point, along the car */
  char buf[ROUTE_MSG_LEN];

  if (r->avoid == AV_NONE)
  {
    r->avoid = AV_WAIT; r->av_t = now; r->av_lat0 = r->lat; r->offset = 0.0f;
    r->av_face = r->fwd + hit_r * cosf(a); r->av_edge = 0.0f;
    key(r, ' ');
    r->dir = 0;
    say(r, -1, "%d obstacle at %ld mm: stop and wait", r->i + 1, (long)front);
    return;
  }
  const float off        = r->lat - r->av_lat0;  /* mm left of the line where the obstacle was met */
  const float corner_fwd = r->fwd + ROUTE_BODY_FRONT * L * cosf(a) + W * sinf(a);  /* right front corner */
  const float corner_off = off + ROUTE_BODY_FRONT * L * sinf(a) - W * cosf(a);
  const float gap        = r->av_face - corner_fwd;

  if (r->avoid == AV_WAIT)
  {
    if (sample && !(front >= 0 && (float)front < c->avoid_stop_mm + 100.0f))
    {
      r->avoid = AV_NONE;
      say(r, -1, "%d obstacle gone", r->i + 1);
      return;
    }
    if ((float)(now - r->av_t) >= c->avoid_wait_s * 1000.0f)
    {
      r->avoid = AV_REVERSE;
      say(r, -1, "%d obstacle still there: back up, then go around on the left", r->i + 1);
    }
    return;
  }
  if (r->avoid == AV_REVERSE)
  {
    if (gap < c->avoid_back_mm) { go(r, now, -1, c->avoid_pct); return; }
    r->avoid = AV_OUT;
  }
  if (r->avoid == AV_OUT)
  {
    r->avoid_off = c->avoid_swerve_deg;
    const bool hit = front >= 0 && (float)front <= c->avoid_look_mm;
    if (sample && hit)
    {
      r->av_edge = fmaxf(r->av_edge, off + hit_r * sinf(a));
      r->av_face = fminf(r->av_face, r->fwd + hit_r * cosf(a));
    }
    const float target = r->av_edge + W + c->avoid_margin_mm;
    if (target > c->avoid_max_offset_mm)
    {
      snprintf(buf, sizeof buf, "obstacle needs more than %g m to the left", c->avoid_max_offset_mm / 1000.0f);
      fail(r, buf);
      return;
    }
    if (!hit && off >= target)
    {
      r->avoid = AV_PASS; r->avoid_off = 0.0f; r->av_mark = r->fwd; r->av_seen = false;
      say(r, -1, "%d %.0f mm left of the line: passing", r->i + 1, off);
      return;
    }
    if (gap < c->avoid_margin_mm && corner_off < r->av_edge + c->avoid_margin_mm)
    {
      snprintf(buf, sizeof buf, "no room to go around: edge %.0f mm left of the line", r->av_edge);
      fail(r, buf);
      return;
    }
    go(r, now, 1, c->avoid_pct);
    return;
  }
  if ((r->avoid == AV_PASS || r->avoid == AV_BACK || r->avoid == AV_STRAIGHT)
      && sample && front >= 0 && (float)front < c->avoid_stop_mm)
  {
    r->av_resume = r->avoid; r->av_t = now; r->avoid = AV_WAIT2;
    key(r, ' ');
    r->dir = 0;
    say(r, -1, "%d new obstacle at %ld mm during the detour: stop and wait", r->i + 1, (long)front);
    return;
  }
  if (r->avoid == AV_WAIT2)
  {
    if (sample && !(front >= 0 && (float)front < c->avoid_stop_mm + 100.0f))
    {
      r->avoid = r->av_resume;
      say(r, -1, "%d it is gone: continue the detour", r->i + 1);
      return;
    }
    if ((float)(now - r->av_t) >= c->avoid_wait_s * 1000.0f)  /* part of the same obstacle: same detour */
    {
      r->av_face = r->fwd + hit_r * cosf(a); r->avoid = AV_REVERSE;
      say(r, -1, "%d still there: back up and go around both", r->i + 1);
    }
    return;
  }
  if (r->avoid == AV_PASS)
  {
    if (r->fwd + ROUTE_SIDE_X * L < r->av_face) { r->av_mark = r->fwd; }  /* side sensors not beside it yet */
    for (int k = 1; k < 3; k++)
    {
      if (r->mm[k] >= 0 && (float)r->mm[k] <= off) { r->av_mark = r->fwd; r->av_seen = true; }
    }
    /* The car rear is behind the side sensors. Never seen = 1 m past the face. */
    const float past = r->av_seen ? (ROUTE_SIDE_X + ROUTE_BODY_REAR) * L + c->avoid_margin_mm : 1000.0f;
    if (r->fwd - r->av_mark < past) { go(r, now, 1, c->avoid_pct); return; }
    r->avoid = AV_BACK;
    say(r, -1, "%d passed: back to the line", r->i + 1);
  }
  if (r->avoid == AV_BACK)
  {
    r->avoid_off = -c->avoid_swerve_deg;
    /* Straighten early: the turn back from the swerve at full lock still moves R (1 - cos) sideways. */
    const float lead = L / tanf(DEG2RAD(c->servo_max_steer_deg)) * (1.0f - cosf(DEG2RAD(c->avoid_swerve_deg)));
    if (off > lead) { go(r, now, 1, c->avoid_pct); return; }
    r->avoid = AV_STRAIGHT; r->avoid_off = 0.0f;
  }
  if (fabsf(a) > DEG2RAD(3.0f)) { go(r, now, 1, c->avoid_pct); return; }  /* straight: back along the line */
  r->avoid = AV_NONE;
  say(r, -1, "%d back on the line", r->i + 1);
}

/* Drive until the reading comes into (seen) or goes out of (lost) mm +- tol. Then, when faster than
 * ROUTE_ALIGN_PCT, back over that edge and come up to it again at ROUTE_ALIGN_PCT: the stop is at the
 * edge, not after the coast. The reading is corrected to the distance square to the object:
 * reading x cos(yaw - route heading). This assumes that the object is parallel to the route heading.
 * hold > 0: while the object is seen, P control on the heading keeps the corrected reading at hold.
 * after > 0: no back-and-forward; drive on `after` m from the edge, then stop.
 * max limits only the search for the edge. */
static void find(Route *r, uint32_t now, const RouteStep *s, float odo, float moved, float a)
{
  char buf[ROUTE_MSG_LEN];
  if (r->phase != PH_AFTER && fabsf(moved) > s->max * 1000.0f)
  {
    snprintf(buf, sizeof buf, "no \"%s\" edge within %g m", s->until_seen ? "seen" : "lost", s->max);
    fail(r, buf);
    return;
  }
  if (r->fresh)
  {
    const int32_t raw = r->mm[s->sensor];
    r->reading = (raw >= 0) ? (int32_t)lroundf((float)raw * cosf(a)) : -1;
    r->inside  = r->reading >= 0 && fabsf((float)r->reading - s->mm) <= s->tol;
    const bool following = s->hold > 0.0f && r->inside
                        && (r->phase == PH_ARM || r->phase == PH_SEEK || r->phase == PH_AFTER);
    /* Right-side sensor: too far (reading > hold) = turn right = negative degrees. */
    r->offset = following ? clampf(-s->kp * ((float)r->reading - s->hold), -ROUTE_HOLD_MAX_DEG, ROUTE_HOLD_MAX_DEG)
                          : 0.0f;
  }
  const bool edge = r->inside == (s->until_seen != 0);
  if (r->phase == PH_ARM)  /* lost: see the object first */
  {
    if (!r->inside) { go(r, now, 1, s->speed); return; }
    r->phase = PH_SEEK;
  }
  if (r->phase == PH_SEEK)
  {
    if (!edge) { go(r, now, 1, s->speed); return; }
    /* shortcut: read each ROUTE_TOF_PERIOD_MS, so up to speed x 50 ms late. Measure the lag on the car. */
    r->edge = odo; r->have_edge = true;
    r->phase = (s->after > 0.0f) ? PH_AFTER : (s->speed > ROUTE_ALIGN_PCT) ? PH_BACK : PH_CREEP;
  }
  if (r->phase == PH_AFTER)
  {
    const float left = s->after * 1000.0f - (odo - r->edge);
    if (left > 0.0f) { go(r, now, 1, left > 100.0f ? s->speed : ROUTE_ALIGN_PCT); return; }  /* last 100 mm slow */
    snprintf(buf, sizeof buf, "%g m after the edge, %s %ld mm", s->after, SENSOR_NAME[s->sensor], (long)r->reading);
    finish(r, buf);
    return;
  }
  if (r->phase == PH_BACK)
  {
    if (edge) { go(r, now, -1, ROUTE_ALIGN_PCT); return; }
    r->phase = PH_CREEP;
  }
  if (!edge) { go(r, now, 1, ROUTE_ALIGN_PCT); return; }
  r->edge = odo; r->have_edge = true;  /* align measures from here, not from where the car stops */
  snprintf(buf, sizeof buf, "%s %ld mm, %.2f m", SENSOR_NAME[s->sensor], (long)r->reading, moved / 1000.0f);
  finish(r, buf);
}

void route_step(Route *r, const RouteInput *in)
{
  r->set_goal = false; r->nkeys = 0; r->nmsg = 0;
  if (!r->running) { return; }
  const uint32_t now = in->now_ms;
  char buf[ROUTE_MSG_LEN];

  if (!r->have_heading)
  {
    r->heading = in->heading_goal; r->t_start = now; r->have_heading = true;
  }
  r->set_goal = true;
  r->goal_deg = wrap180(r->heading + r->offset + r->avoid_off);

  if (!r->begun)
  {
    if (in->max_abs_rpm > 0.5f) { return; }  /* each step starts with the car stopped */
    if (r->i >= r->n)
    {
      if (r->loop)  /* again from step 1, from where the car is now */
      {
        say(r, -1, "lap %d done at %.1f s", r->lap, (float)(now - r->t_start) / 1000.0f);
        r->i = 0; r->lap++;
        return;
      }
      r->running = false;
      say(r, -1, "route done in %.1f s", (float)(now - r->t_start) / 1000.0f);
      return;
    }
    const RouteStep *s = &r->steps[r->i];
    r->begun = true; r->s0 = in->odo_mm; r->t0 = now; r->offset = 0.0f;
    r->lat = r->fwd = 0.0f;  /* mm left of / along the line of this step */
    r->last = r->s0; r->avoid = AV_NONE; r->avoid_off = 0.0f;
    r->t_mm = now - ROUTE_TOF_PERIOD_MS;  /* read the sensors at once */
    if (r->i == 0 || r->steps[r->i - 1].type != STEP_FIND) { r->have_edge = false; }  /* align: edge of its find only */
    r->phase = (s->type == STEP_FIND && !s->until_seen) ? PH_ARM : PH_SEEK;
    if (s->type == STEP_TURN) { r->heading = wrap180(r->heading + s->deg); }
    if (s->type == STEP_CHECKPOINT) { say(r, r->i, "%s", ""); }
  }

  const RouteStep *s = &r->steps[r->i];
  const float odo = in->odo_mm;
  const float a = DEG2RAD(wrap180(in->current_yaw - r->heading));
  r->lat += (odo - r->last) * sinf(a);
  r->fwd += (odo - r->last) * cosf(a);
  r->last = odo;
  r->fresh = now - r->t_mm >= ROUTE_TOF_PERIOD_MS;
  if (r->fresh)
  {
    r->t_mm = now;
    r->read_tof(r->ctx, r->mm);
  }
  if (r->avoid != AV_NONE
      || (r->dir > 0 && (s->type == STEP_DRIVE || (s->type == STEP_FIND && s->sensor != SENSOR_FRONT))
          && r->mm[0] >= 0 && (float)r->mm[0] < r->cfg.avoid_stop_mm))
  {
    avoiding(r, now, a);
    return;
  }
  /* drive/find: progress along the line */
  float moved = (s->type == STEP_DRIVE || s->type == STEP_FIND) ? r->fwd : odo - r->s0;
  switch (s->type)
  {
    case STEP_DRIVE:
      if (fabsf(moved) >= fabsf(s->m) * 1000.0f)
      {
        snprintf(buf, sizeof buf, "%.2f m", moved / 1000.0f);
        finish(r, buf);
        return;
      }
      go(r, now, s->m > 0.0f ? 1 : -1, s->speed);
      return;
    case STEP_TURN:
      if (fabsf(wrap180(r->heading - in->current_yaw)) < 2.0f)
      {
        snprintf(buf, sizeof buf, "heading %.1f deg after %.2f m", in->current_yaw, moved / 1000.0f);
        finish(r, buf);
        return;
      }
      if (fabsf(moved) > 20000.0f) { fail(r, "no turn after 20 m (IMU?)"); return; }
      go(r, now, 1, s->speed);  /* Ackermann: the car must roll to turn */
      return;
    case STEP_ALIGN:  /* drive from the edge to the checkpoint, stop, then report the error */
      moved = odo - (r->have_edge ? r->edge : r->s0);
      if (r->phase == PH_SEEK)
      {
        const float left = fabsf(s->m) * 1000.0f - fabsf(moved);
        if (left > 0.0f)  /* the last 100 mm at ROUTE_ALIGN_PCT: less coasting after the stop */
        {
          go(r, now, s->m > 0.0f ? 1 : -1, left > 100.0f ? s->speed : ROUTE_ALIGN_PCT);
          return;
        }
        key(r, ' ');
        r->dir = 0; r->phase = PH_SETTLE;
      }
      else if (in->max_abs_rpm <= 0.5f)
      {
        say(r, r->i, "%s", "");
        snprintf(buf, sizeof buf, "%.2f m from the edge", moved / 1000.0f);
        finish(r, buf);
      }
      return;
    case STEP_CHECKPOINT:
      if ((float)(now - r->t0) >= s->s * 1000.0f)
      {
        snprintf(buf, sizeof buf, "waited %g s", s->s);
        finish(r, buf);
      }
      return;
    default:
      find(r, now, s, odo, moved, a);
      return;
  }
}
