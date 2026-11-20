/* Control core of the tractor: operator keys, wheel speed control and heading hold.
 * No HAL, no sensor driver and no actuator calls. The firmware (main.c) and the dashboard sim
 * (tools/tractor-dashboard) call the same functions with the same constants.h:
 *   - inputs:  keys, encoder counter values, heading samples, time in ms.
 *   - outputs: the fields marked "output" below. The caller sends them to the motors and the servo.
 * The order of the calls is the order of the main loop in main.c. */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "pid.h"

/* One heading measurement, already relative to the calibration zero (see heading_relative). */
typedef struct
{
  float    yaw_deg;
  uint32_t count;   /* report number. The same number = the same report again       */
  uint32_t t_ms;    /* time of the report, ms                                         */
  bool     valid;   /* the report is fresh and the sensor did no reset after the zero */
} HeadingSample;

typedef struct
{
  /* Operator command */
  int32_t  drive_dir;       /* +1 = forward, -1 = reverse, 0 = stop                       */
  int32_t  speed_pct;       /* 10 to 100 % of WHEEL_SPEED_MAX_RPM                         */
  uint32_t t_last_key;      /* time of the last W or S key                                */
  uint32_t t_tick;          /* time of the last control_tick                              */

  /* Outputs */
  float    steer_deg;       /* steering command after the slew limit, positive = left.
                               Send it to the servo.                                      */
  float    steer_goal;      /* requested steering: A/D keys, or the heading PID           */
  float    cmd_l, cmd_r;    /* motor commands, -100 to 100 %                              */
  bool     motor_enable;    /* level of the MOTOR_EN pin                                  */

  /* Wheel speed control */
  Pid      pid_l, pid_r;
  float    target_rpm;      /* vehicle speed target after the ramp                        */
  float    rpm_l, rpm_r;    /* measured wheel speeds                                      */
  uint32_t last_seq;        /* encoder sample number used last                            */
  uint16_t last_count_l, last_count_r;
  uint32_t t_motion_l, t_motion_r;  /* last time without a stall condition                */
  bool     stalled;         /* the Space key clears it                                    */
  uint32_t overruns;        /* encoder samples that the main loop missed                  */

  /* Heading hold */
  bool     heading_active;  /* HEADING_HOLD_ENABLE and the IMU started                     */
  bool     imu_ok;
  Pid      pid_heading;
  float    heading_goal;    /* A/D key target, [-180, 180)                                */
  float    heading_target;  /* PID target: moves toward heading_goal at the ramp rate     */
  float    current_yaw;
  float    heading_error;   /* wrap180(target - current_yaw)                              */
  float    heading_out;     /* newest PID output, for forward movement                    */
  float    prev_yaw;        /* yaw of the reference sample for dt and the D term          */
  uint32_t last_count, last_t_ms;
  bool     have_prev;       /* false: the next valid sample only sets the reference       */
} Control;

/* Start state. imu_ok = the BNO085 started and the heading zero is set. */
void  control_init(Control *c, bool imu_ok, uint32_t now_ms);
/* One key from the serial link: W S A D Space 0-9. Other keys do nothing. */
void  control_key(Control *c, char key, uint32_t now_ms);
/* Each main loop: command timeout, IMU interlock, heading target ramp and steering slew limit. */
void  control_tick(Control *c, uint32_t now_ms);
/* Each new encoder sample. seq counts the samples; count_l and count_r are the raw 16-bit
 * timer counters at that sample. Several missed samples are allowed (dt = samples x period). */
void  control_wheels(Control *c, uint32_t seq, uint16_t count_l, uint16_t count_r, uint32_t now_ms);
/* Each steering period (STEERING_CONTROL_PERIOD_MS). Does nothing when heading hold is off. */
void  control_steering(Control *c, const HeadingSample *s);

/* Heading relative to the calibration zero, from the raw sensor heading. */
float heading_relative(float raw_deg, float zero_deg);

#endif /* CONTROL_H */
