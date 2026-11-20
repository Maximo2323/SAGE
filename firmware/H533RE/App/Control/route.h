/* Route runner: runs the route steps one after the other. Each step gives operator keys
 * (W/S, speed digits, Space) and a heading goal to the control core.
 * The firmware (main.c) and the dashboard sim run this same code.
 * It decides from the wheel distance, the IMU heading and the ToF readings only.
 *
 * The dashboard edits the steps (tools/tractor-dashboard/route.json). When you save the
 * route there, it writes App/Config/route_steps.h for each firmware folder. Build and flash
 * to put the new route on the car. */
#ifndef ROUTE_H
#define ROUTE_H

#include <stdbool.h>
#include <stdint.h>

#define ROUTE_TOF_PERIOD_MS  50U    /* TOF_INTER_MEASUREMENT_MS: a new ToF reading this often     */
#define ROUTE_ALIGN_PCT      10.0f  /* speed of the back-and-forward align at the end of a find   */
#define ROUTE_HOLD_MAX_DEG   20.0f  /* the most that wall following turns away from the heading */
#define ROUTE_MSG_MAX        4      /* log messages from one route_step() call                   */
#define ROUTE_MSG_LEN        100

/* Car body and ToF mounts, as fractions of the wheelbase L (forward from the rear axle) and
 * of the track T. ASSUMED: measure them on the car. The dashboard draws the same body. */
#define ROUTE_FRONT_X     1.18f   /* front sensor, on the center line                    */
#define ROUTE_SIDE_X      0.5f    /* the two right sensors                               */
#define ROUTE_SIDE_Y      0.34f   /* right sensors: this x T to the right of the center  */
#define ROUTE_BODY_FRONT  1.184f  /* body front                                          */
#define ROUTE_BODY_REAR   0.183f  /* body rear, behind the rear axle                     */
#define ROUTE_HALF_W      0.34f   /* half body width                                     */

typedef enum { STEP_DRIVE, STEP_TURN, STEP_FIND, STEP_CHECKPOINT, STEP_ALIGN } StepType;
typedef enum { SENSOR_FRONT, SENSOR_RIGHT_LOW, SENSOR_RIGHT_HIGH } RouteSensor;  /* = DistId order */

/* One step. The fields that a step type does not use stay 0. Units: m, %, deg, mm, s. */
typedef struct
{
  uint8_t type;        /* StepType                                                */
  uint8_t sensor;      /* find: RouteSensor                                       */
  uint8_t until_seen;  /* find: 1 = until the reading comes in, 0 = until it goes */
  float   m, speed, deg, mm, tol, max, hold, kp, after, s;
  char    label[25];   /* checkpoint, align                                       */
} RouteStep;

/* Car geometry and obstacle avoidance settings. route_config_default() reads constants.h. */
typedef struct
{
  float wheelbase_mm, track_mm, servo_max_steer_deg;
  float avoid_stop_mm, avoid_wait_s, avoid_back_mm, avoid_swerve_deg, avoid_margin_mm;
  float avoid_max_offset_mm, avoid_look_mm, avoid_pct;
} RouteConfig;

/* Inputs for each route_step() call. */
typedef struct
{
  uint32_t now_ms;
  float    odo_mm;        /* wheel distance since the start, + = forward         */
  float    current_yaw;   /* Control.current_yaw                                 */
  float    heading_goal;  /* Control.heading_goal                                */
  float    max_abs_rpm;   /* the larger measured wheel speed, without sign       */
} RouteInput;

/* Reads the 3 ToF sensors in RouteSensor order: mm, or -1 = no target or not valid. */
typedef void (*RouteReadTof)(void *ctx, int32_t mm[3]);

typedef enum { AV_NONE, AV_WAIT, AV_REVERSE, AV_OUT, AV_PASS, AV_BACK, AV_STRAIGHT, AV_WAIT2 } AvoidPhase;
typedef enum { PH_ARM, PH_SEEK, PH_AFTER, PH_BACK, PH_CREEP, PH_SETTLE } StepPhase;

typedef struct
{
  /* Outputs of the last route_step(). Give the goal and the keys to the control core. */
  bool     set_goal;
  float    goal_deg;
  char     keys[8];
  uint8_t  nkeys;
  char     msg[ROUTE_MSG_MAX][ROUTE_MSG_LEN];  /* log text without the time               */
  int16_t  report[ROUTE_MSG_MAX];  /* >= 0: this message is a checkpoint report of that step.
                                      The text is empty: the caller writes the report.   */
  uint8_t  nmsg;

  /* State */
  const RouteStep *steps;
  int          n, i, lap;
  bool         running, begun, loop;
  RouteConfig  cfg;
  RouteReadTof read_tof;
  void        *ctx;
  bool         have_heading, have_edge, fresh, inside, av_seen;
  float        heading, offset, avoid_off, edge, s0, lat, fwd, last;
  float        av_lat0, av_face, av_edge, av_mark;
  uint32_t     t_start, t_key, t0, t_mm, av_t;
  int          dir;
  char         speed_key;
  int32_t      mm[3], reading;
  AvoidPhase   avoid, av_resume;
  StepPhase    phase;
} Route;

void route_config_default(RouteConfig *cfg);
/* Start at step `first`. loop: after the last step, start again at step 1. */
void route_start(Route *r, const RouteStep *steps, int n, int first, bool loop,
                 const RouteConfig *cfg, RouteReadTof read_tof, void *ctx);
void route_stop(Route *r);
/* Each main loop, before the keys are read. Does nothing when the route does not run. */
void route_step(Route *r, const RouteInput *in);

#endif /* ROUTE_H */
