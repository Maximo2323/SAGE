/* Hardware assignments: which timer and channel each function uses.
 *
 * GPIO pins (ENC_A, ENC_B, MOTOR_IN1, MOTOR_IN2, ...) are NOT here.
 * CubeMX owns them: change them in John_Deere_Tractor.ioc, then generate code.
 * CubeMX then writes the new pins in Core/Inc/main.h.
 *
 * After a timer change here, make the same change in CubeMX. The two must agree. */
#ifndef PINS_H
#define PINS_H

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;

/* Motor PWM: TIM8 at MOTOR_PWM_HZ. Left = PC6 (RPWM), PC7 (LPWM). Right = PC8 (RPWM), PC9 (LPWM). */
#define MOTOR_PWM_TIM       htim8
#define MOTOR_L_FWD_CH      TIM_CHANNEL_1
#define MOTOR_L_REV_CH      TIM_CHANNEL_2
#define MOTOR_R_FWD_CH      TIM_CHANNEL_3
#define MOTOR_R_REV_CH      TIM_CHANNEL_4

/* Encoders, timer encoder mode x4. Left = TIM2 on PA0, PA1. Right = TIM3 on PA6, PA7.
 * Input filter = 8 in CubeMX: the timer ignores pulses shorter than 1.5 us.
 * TBD: adjust it after a check of the encoder signals with the motors on. */
#define ENC_L_TIM           htim2
#define ENC_R_TIM           htim3

/* Encoder sample: one TIM6 interrupt each WHEEL_CONTROL_PERIOD_MS */
#define ENC_SAMPLE_TIM      htim6

/* Steering servo: TIM1 channel 1 on PA8, 1 MHz tick, SERVO_PWM_HZ */
#define SERVO_TIM           htim1
#define SERVO_CHANNEL       TIM_CHANNEL_1

/* BNO085: I2C1 on PB6 (SCL), PB7 (SDA), 400 kHz */
extern I2C_HandleTypeDef hi2c1;
#define BNO_I2C             hi2c1

/* I2C addresses. BNO085: 7-bit. VL53L1X: 8-bit, as the ST driver uses them. */
#define BNO085_I2C_ADDR     0x4BU
#define TOF_DEFAULT_ADDR    0x52U
#define TOF_ADDR_1          0x54U
#define TOF_ADDR_2          0x56U
#define TOF_ADDR_3          0x58U

/* Distance sensors: same I2C bus as the BNO085.
 * ===== TBD: PLACEHOLDER XSHUT pins. They are not assigned in CubeMX and not checked on the board.
 * Each VL53L1X needs one XSHUT pin, because all sensors start with the same I2C address. ===== */
#define TOF_I2C                    hi2c1
#define TOF_XSHUT_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define TOF_XSHUT_FRONT_PORT       GPIOB
#define TOF_XSHUT_FRONT_PIN        GPIO_PIN_0
#define TOF_XSHUT_RIGHT_LOW_PORT   GPIOB
#define TOF_XSHUT_RIGHT_LOW_PIN    GPIO_PIN_1
#define TOF_XSHUT_RIGHT_HIGH_PORT  GPIOB
#define TOF_XSHUT_RIGHT_HIGH_PIN   GPIO_PIN_2

#endif /* PINS_H */
