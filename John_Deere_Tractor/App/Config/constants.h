/* Tuning values and physical constants of the tractor.
 * Change the values here. Do not write numbers directly in the code.
 *
 * ===========================================================================
 * TBD = placeholder. The value is NOT measured. Measure it on the car first.
 * Search for "TBD" to find all placeholders.
 * ===========================================================================
 */
#ifndef CONSTANTS_H
#define CONSTANTS_H

/* --- Timing --- */
#define WHEEL_CONTROL_PERIOD_MS     10U    /* encoder sample and wheel PI: 100 Hz.
                                              main.c sets the TIM6 period from this value. */
#define WHEEL_CONTROL_DT_S          ((float)WHEEL_CONTROL_PERIOD_MS / 1000.0f)
#define STEERING_CONTROL_PERIOD_MS  20U    /* heading and steering control: 50 Hz          */
#define TELEMETRY_PERIOD_MS         100U   /* telemetry: 10 Hz                             */
#define MOTOR_PWM_HZ                20000U /* main.c sets the TIM8 period from this value.
                                              The BTS7960 data sheet permits 25 kHz maximum. */

/* --- Timeouts, in elapsed milliseconds --- */
#define STALL_TIMEOUT_MS            300U   /* high duty cycle and no encoder counts        */
#define COMMAND_TIMEOUT_MS          300U   /* hold-to-drive: no key in this time = stop.
                                              TBD: the first key repeat of the PC can be slower. */
#define BNO_STALE_TIMEOUT_MS        300U   /* no new heading report in this time = invalid */

/* --- Motor: 37D gearmotor 50:1 with 64 CPR encoder (seller data) --- */
#define GEAR_RATIO                  50U
#define MOTOR_NO_LOAD_RPM_12V       200.0f /* seller data. 100 RPM at 6 V.                 */

/* --- Encoder ---
 * ASSUMPTION: "64 CPR" = 64 edges for each motor shaft revolution
 * (2 channels x 2 edges x 16 pulses). This is the usual meaning for 37D motors.
 * ===== TBD: turn the wheel 10 revolutions, read the encoder count, divide by 10. =====
 * ASSUMPTION: the wheel is attached directly to the gearbox output shaft. */
#define ENC_PULSES_PER_MOTOR_REV    16U
#define ENC_EDGES_COUNTED           4U     /* timer encoder mode counts all edges of A and B */
#define COUNTS_PER_WHEEL_REV        (ENC_PULSES_PER_MOTOR_REV * ENC_EDGES_COUNTED * GEAR_RATIO) /* 3200 */
#define ENC_DIR_LEFT                1      /* TBD: +1 or -1. Forward movement must give positive counts. */
#define ENC_DIR_RIGHT               1      /* TBD */

/* --- Motor command ---
 * The command is -100 to 100 %. motor_set() changes it into a duty cycle between
 * MOTOR_DEADBAND_PCT and MOTOR_DUTY_MAX. Do not add the deadband in other code. */
#define MOTOR_DEADBAND_PCT          0.0f   /* ===== TBD: smallest duty cycle that turns the wheel ===== */
#define MOTOR_DUTY_MAX              100    /* TBD: decrease it to limit the motor voltage  */
#define STALL_MIN_DUTY              30     /* TBD: at this duty cycle the wheel must turn  */

/* --- Wheel speed control: one PI controller for each motor ---
 * With WHEEL_KP = 0 and WHEEL_KI = 0, only the feedforward term drives the motors.
 * command % = WHEEL_FF_PCT_PER_RPM x target RPM + PI correction */
#define WHEEL_SPEED_MAX_RPM         MOTOR_NO_LOAD_RPM_12V /* ===== TBD: speed for key 0 (100 %). Must be less than
                                                             the no-load speed at the battery voltage. ===== */
#define WHEEL_FF_PCT_PER_RPM        (100.0f / MOTOR_NO_LOAD_RPM_12V) /* TBD: estimate for a 12 V supply only */
#define WHEEL_KP                    0.0f   /* ===== TBD: from the motor model ===== */
#define WHEEL_KI                    0.0f   /* ===== TBD: from the motor model ===== */

/* --- Vehicle geometry: necessary for the rear wheel speeds in a turn --- */
#define WHEEL_DIAMETER_MM           0.0f   /* ===== TBD: measure ===== */
#define TRACK_WIDTH_MM              0.0f   /* ===== TBD: distance between the rear wheels ===== */
#define WHEELBASE_MM                0.0f   /* ===== TBD: distance between front and rear axles ===== */

/* --- Keyboard control --- */
#define WHEEL_ACCEL_RPM_PER_S       200.0f /* ===== TBD: ramp when the speed target increases ===== */
#define WHEEL_DECEL_RPM_PER_S       1000.0f /* ===== TBD: ramp when the speed target decreases (key release,
                                               lost link). The Space key stops with no ramp. ===== */
#define STEER_STEP_DEG              5.0f   /* TBD: steering angle change for each A or D key */

#define PID_DEFAULT_OUT_LIMIT       255.0f

/* --- Heading ---
 * Sign convention: positive heading = counterclockwise, seen from above.
 * Positive steering angle = left. A positive heading error gives a positive steering angle.
 * ===== TBD: confirm the two directions on the car. =====
 *
 * Heading source for control. The other report is for telemetry only.
 * The two reports have different zero references. Compare changes, not absolute values.
 * To change the source, copy one of these lines:
 *   #define HEADING_SOURCE  HEADING_SOURCE_GAME_RV   (no magnetometer: no magnetic jumps, slow drift)
 *   #define HEADING_SOURCE  HEADING_SOURCE_RV        (magnetometer: no drift, motors can disturb it)
 * bno085.c enables the two reports. The telemetry shows the two headings. */
#define HEADING_SOURCE_GAME_RV      0
#define HEADING_SOURCE_RV           1
#define HEADING_SOURCE              HEADING_SOURCE_GAME_RV
#define BNO_GAME_RV_INTERVAL_US     10000U /* 100 Hz. TBD: verify the real report rate     */
#define BNO_RV_INTERVAL_US          20000U /* 50 Hz.  TBD: verify the real report rate     */

/* Heading hold. 0 = the A and D keys move the servo directly. 1 = the A and D keys change the
 * heading target and the heading PID moves the servo.
 * ===== TBD: keep 0 until the heading direction, SERVO_DIR and the gains are confirmed. ===== */
#define HEADING_HOLD_ENABLE         0
#define HEADING_STEP_DEG            5.0f   /* TBD: heading target change for each A or D key */
#define HEADING_KP                  0.0f   /* ===== TBD: degrees of steering for each degree of error ===== */
#define HEADING_KI                  0.0f   /* ===== TBD ===== */
#define HEADING_KD                  0.0f   /* ===== TBD ===== */

#define HEADING_CAL_SAMPLES       80
#define HEADING_CAL_WARMUP_MS     2000U
#define HEADING_MAX_SAMPLES       128
#define HEADING_SAMPLE_DELAY_MS   10U

#define BNO_I2C_TIMEOUT_MS        15U     /* the largest sensor packet (384 bytes) takes 9 ms at 400 kHz */
#define BNO_READ_PERIOD_MS        5U      /* time between two reads of the sensor */
#define BNO_BEGIN_ATTEMPTS        5U
#define BNO_BOOT_DELAY_MS         250U
#define BNO_REPORT_DELAY_MS       120U
#define BNO_SOFTRESET_ATTEMPTS    5U
#define BNO_SOFTRESET_RETRY_MS    30U
#define BNO_SOFTRESET_WAIT_MS     300U

/* --- Distance sensors: 3 x VL53L1X (TOF400C module) on I2C1 ---
 * 0 = the firmware does not start or read the distance sensors.
 * ===== TBD: set to 1 when the sensors are connected and the XSHUT pins in pins.h are correct. ===== */
#define DISTANCE_ENABLE           0
#define DIST_STALE_MS             200U    /* TBD: a measurement older than this is not valid */
#define TOF_DISTANCE_MODE_SHORT   1U
#define TOF_DISTANCE_MODE_LONG    2U
#define TOF_DISTANCE_MODE         TOF_DISTANCE_MODE_LONG   /* TBD */
#define TOF_TIMING_BUDGET_MS      50U     /* TBD: 15 (short only), 20, 33, 50, 100, 200, 500 */
#define TOF_INTER_MEASUREMENT_MS  50U     /* TBD: >= TOF_TIMING_BUDGET_MS */
#define TOF_MAX_RANGE_MM          4000U   /* TBD: a larger reading is "no target". The sensor limit is 4 m in the dark. */
#define TOF_I2C_TIMEOUT_MS        10U     /* TBD */
#define TOF_BOOT_TIMEOUT_MS       100U    /* TBD */
#define TOF_BOOT_POLL_MS          2U
#define TOF_XSHUT_RESET_MS        10U     /* TBD */

#define HCSR04_TRIGGER_US         10U
#define HCSR04_TIMEOUT_US         38000U
#define HCSR04_MAX_RANGE_MM       4000U   /* TBD */
#define HCSR04_MM_PER_US          0.1715f

/* 9 kg servo. TBD: measure the pulse limits and the center on the car. */
#define SERVO_MIN_US              1000U   /* TBD */
#define SERVO_CENTER_US           1500U   /* TBD */
#define SERVO_MAX_US              2000U   /* TBD */
#define SERVO_MAX_STEER_DEG       30.0f   /* TBD */
#define SERVO_DIR                 1.0f    /* TBD: +1 or -1. A positive angle must steer left. */

#endif /* CONSTANTS_H */
