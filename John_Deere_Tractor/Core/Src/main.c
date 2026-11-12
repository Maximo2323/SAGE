/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "constants.h"
#include "pins.h"
#include "motor.h"
#include "servo.h"
#include "pid.h"
#include "bno085.h"
#include "heading_hold.h"
#include "distance.h"
#include "math_utils.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
/* Written by the TIM6 interrupt, read by the main loop */
static volatile uint32_t sample_seq     = 0;  /* number of encoder samples since the start */
static volatile uint16_t sample_count_l = 0;  /* left encoder count at the last sample     */
static volatile uint16_t sample_count_r = 0;  /* right encoder count at the last sample    */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static Motor motor_l, motor_r;
static Servo servo;
static Pid   pid_l, pid_r;

static Bno085      imu;
static HeadingHold heading;
static Pid         pid_heading;
static uint8_t     imu_ok = 0;      /* 1 = the BNO085 started and the heading zero is set */

/* Operator command */
static int32_t  drive_dir  = 0;     /* +1 = forward, -1 = reverse, 0 = stop       */
static int32_t  speed_pct  = 10;    /* 10 to 100 % of WHEEL_SPEED_MAX_RPM         */
static float    steer_deg  = 0.0f;  /* positive = left                            */
static uint32_t t_last_key = 0;     /* time of the last W or S key                */

/* Wheel control state */
static float    target_rpm = 0.0f;  /* vehicle speed target after the ramp        */
static float    rpm_l, rpm_r;       /* measured wheel speeds                      */
static float    cmd_l, cmd_r;       /* motor commands, -100 to 100 %              */
static uint32_t t_motion_l, t_motion_r;  /* last time without a stall condition   */
static uint8_t  stalled    = 0;     /* 1 = stall detected. The Space key clears it. */
static uint32_t overruns   = 0;     /* encoder samples that the main loop missed  */

/* Distance for the telemetry: millimeters, -1 = no target, -2 = not valid. */
static int distance_for_telemetry(DistId id)
{
  uint16_t mm = 0;
  switch (distance_read(id, &mm))
  {
    case DIST_OK:        return (int)mm;
    case DIST_NO_TARGET: return -1;
    default:             return -2;
  }
}

/* True when the heading PID controls the servo. False when the A and D keys control it. */
static uint8_t heading_hold_active(void)
{
  return (HEADING_HOLD_ENABLE != 0) && imu_ok;
}

static void motors_stop(void)
{
  target_rpm = 0.0f;
  cmd_l = 0.0f;
  cmd_r = 0.0f;
  motor_set(&motor_l, 0.0f);
  motor_set(&motor_r, 0.0f);
  pid_reset(&pid_l);
  pid_reset(&pid_r);
}

/* Read one key from the serial port. This function does not wait. */
static void read_key(uint32_t now)
{
  USART_TypeDef *uart = hcom_uart[COM1].Instance;

  /* An overrun error stops the receiver until the flag is cleared. */
  if ((uart->ISR & USART_ISR_ORE) != 0U) { uart->ICR = USART_ICR_ORECF; }
  if ((uart->ISR & USART_ISR_RXNE_RXFNE) == 0U) { return; }

  const char key = (char)uart->RDR;
  switch (key)
  {
    /* Hold-to-drive: only the two drive keys start the command timeout again. */
    case 'w': case 'W': drive_dir = 1;  t_last_key = now; break;
    case 's': case 'S': drive_dir = -1; t_last_key = now; break;
    case 'a': case 'A':
      if (heading_hold_active()) { heading_hold_add_to_target(&heading, HEADING_STEP_DEG); }
      else                       { steer_deg += STEER_STEP_DEG; }
      break;
    case 'd': case 'D':
      if (heading_hold_active()) { heading_hold_add_to_target(&heading, -HEADING_STEP_DEG); }
      else                       { steer_deg -= STEER_STEP_DEG; }
      break;
    case ' ':
      drive_dir = 0;
      motors_stop();  /* immediate stop, no ramp */
      steer_deg = 0.0f;
      heading.target = heading.current_yaw;  /* hold the present heading */
      stalled   = 0;
      HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
      break;
    case '0': speed_pct = 100; break;
    default:
      if (key < '1' || key > '9') { return; }
      speed_pct = (int32_t)(key - '0') * 10;
      break;
  }

  steer_deg  = clampf(steer_deg, -SERVO_MAX_STEER_DEG, SERVO_MAX_STEER_DEG);
  servo_set_deg(&servo, steer_deg);
}

/* Heading hold: the heading PID gives the steering angle. */
static void steering_control(uint32_t now)
{
  if (!heading_hold_active()) { return; }

  float out;
  if (!heading_hold_update(&heading, now, target_rpm != 0.0f, &out))
  {
    drive_dir = 0;  /* no valid heading: do not move */
    return;
  }

  /* In reverse, the same steering angle turns the heading in the opposite direction. */
  steer_deg = (target_rpm < 0.0f) ? -out : out;
  servo_set_deg(&servo, steer_deg);
}

/* Feedforward and PI for one wheel. Returns the motor command in percent. */
static float wheel_command(Pid *pid, float target, float measured, float dt_s)
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
  return clampf(ff + pid_update(pid, target, measured, dt_s), -100.0f, 100.0f);
}

/* Stall condition: a high command and no encoder counts for STALL_TIMEOUT_MS. */
static uint8_t wheel_stalled(float cmd, int32_t delta_counts, uint32_t *t_motion, uint32_t now)
{
  if (fabsf(cmd) < (float)STALL_MIN_DUTY || delta_counts != 0) { *t_motion = now; }
  return (now - *t_motion) >= STALL_TIMEOUT_MS;
}

/* Run the two wheel controllers one time for each new encoder sample. */
static void wheel_control(uint32_t now)
{
  static uint32_t last_seq = 0;
  static uint16_t last_l = 0, last_r = 0;

  __disable_irq();
  const uint32_t seq     = sample_seq;
  const uint16_t count_l = sample_count_l;
  const uint16_t count_r = sample_count_r;
  __enable_irq();

  const uint32_t samples = seq - last_seq;
  if (samples == 0U) { return; }              /* no new measurement: do not use the old one again */
  if (samples > 1U)  { overruns += samples - 1U; }

  /* dt is the real time between the two encoder counts.
   * The 16-bit subtraction gives the correct signed difference when the counter wraps. */
  const float   dt_s    = (float)samples * WHEEL_CONTROL_DT_S;
  const int32_t delta_l = ENC_DIR_LEFT  * (int32_t)(int16_t)(uint16_t)(count_l - last_l);
  const int32_t delta_r = ENC_DIR_RIGHT * (int32_t)(int16_t)(uint16_t)(count_r - last_r);
  last_seq = seq;
  last_l   = count_l;
  last_r   = count_r;

  rpm_l = rpm_from_counts(delta_l, (float)COUNTS_PER_WHEEL_REV, dt_s);
  rpm_r = rpm_from_counts(delta_r, (float)COUNTS_PER_WHEEL_REV, dt_s);

  if (stalled) { return; }

  /* Ramp the speed target toward the operator command. */
  const float goal = (float)drive_dir * (float)speed_pct / 100.0f * WHEEL_SPEED_MAX_RPM;
  const float rate = (fabsf(goal) < fabsf(target_rpm)) ? WHEEL_DECEL_RPM_PER_S : WHEEL_ACCEL_RPM_PER_S;
  const float step = rate * dt_s;
  target_rpm += clampf(goal - target_rpm, -step, step);

  /* In a left turn (positive steering angle) the left wheel is the inner wheel. */
  const float ratio = rear_wheel_ratio(steer_deg, TRACK_WIDTH_MM, WHEELBASE_MM);
  cmd_l = wheel_command(&pid_l, target_rpm * (1.0f - ratio), rpm_l, dt_s);
  cmd_r = wheel_command(&pid_r, target_rpm * (1.0f + ratio), rpm_r, dt_s);
  motor_set(&motor_l, cmd_l);
  motor_set(&motor_r, cmd_r);

  const uint8_t stall_l = wheel_stalled(cmd_l, delta_l, &t_motion_l, now);
  const uint8_t stall_r = wheel_stalled(cmd_r, delta_r, &t_motion_r, now);
  if (stall_l || stall_r)
  {
    stalled   = 1;
    drive_dir = 0;
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
    motors_stop();
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  /* MOTOR_PWM_HZ sets the motor PWM frequency. This replaces the period value from CubeMX.
   * shortcut: correct only while the APB2 clock divider is 1 (timer clock = PCLK2). */
  __HAL_TIM_SET_AUTORELOAD(&MOTOR_PWM_TIM,
      HAL_RCC_GetPCLK2Freq() / (MOTOR_PWM_TIM.Init.Prescaler + 1U) / MOTOR_PWM_HZ - 1U);
  motor_init(&motor_l, &MOTOR_PWM_TIM, MOTOR_L_FWD_CH, MOTOR_L_REV_CH);
  motor_init(&motor_r, &MOTOR_PWM_TIM, MOTOR_R_FWD_CH, MOTOR_R_REV_CH);
  servo_init(&servo, &SERVO_TIM, SERVO_CHANNEL);

  pid_init(&pid_l, WHEEL_KP, WHEEL_KI, 0.0f);
  pid_init(&pid_r, WHEEL_KP, WHEEL_KI, 0.0f);
  pid_set_output_limits(&pid_l, -100.0f, 100.0f);
  pid_set_output_limits(&pid_r, -100.0f, 100.0f);

  /* BNO085 start and heading zero: about 3 s. The vehicle must not move during this time.
   * If the sensor does not answer, the firmware continues without heading data. */
  pid_init(&pid_heading, HEADING_KP, HEADING_KI, HEADING_KD);
  pid_set_output_limits(&pid_heading, -SERVO_MAX_STEER_DEG, SERVO_MAX_STEER_DEG);
  heading_hold_init(&heading, &imu, &pid_heading);
  imu_ok = bno085_begin(&imu, &BNO_I2C, BNO085_I2C_ADDR)
        && heading_hold_calibrate(&heading, HEADING_CAL_SAMPLES, HEADING_CAL_WARMUP_MS);

  /* A sensor that does not answer gives DIST_NOT_VALID. The other sensors continue to operate. */
  if (DISTANCE_ENABLE) { distance_init(&TOF_I2C); }

  HAL_TIM_Encoder_Start(&ENC_L_TIM, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&ENC_R_TIM, TIM_CHANNEL_ALL);
  /* WHEEL_CONTROL_PERIOD_MS sets the TIM6 period. This replaces the period value from CubeMX,
   * so the sample time and the RPM calculation always agree.
   * shortcut: correct only while the APB1 clock divider is 1 (timer clock = PCLK1). */
  __HAL_TIM_SET_AUTORELOAD(&ENC_SAMPLE_TIM,
      HAL_RCC_GetPCLK1Freq() / (ENC_SAMPLE_TIM.Init.Prescaler + 1U) / 1000U * WHEEL_CONTROL_PERIOD_MS - 1U);
  HAL_TIM_Base_Start_IT(&ENC_SAMPLE_TIM);

  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);

  uint32_t t_telemetry = HAL_GetTick();
  uint32_t t_steering  = t_telemetry;
  uint32_t t_imu_read  = t_telemetry;
  t_motion_l = t_telemetry;
  t_motion_r = t_telemetry;
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    read_key(now);

    /* Hold-to-drive: stop when no key arrives for COMMAND_TIMEOUT_MS. */
    if (drive_dir != 0 && (now - t_last_key) >= COMMAND_TIMEOUT_MS) { drive_dir = 0; }

    /* Heading hold is selected but the sensor did not start: do not move. */
    if (HEADING_HOLD_ENABLE && !imu_ok) { drive_dir = 0; }

    wheel_control(now);

    /* The BNO085 read uses I2C and can take some milliseconds. It stays in the main loop. */
    if (now - t_imu_read >= BNO_READ_PERIOD_MS)
    {
      t_imu_read = now;
      bno085_update(&imu);
    }
    if (DISTANCE_ENABLE) { distance_update(); }
    if (now - t_steering >= STEERING_CONTROL_PERIOD_MS)
    {
      t_steering = now;
      steering_control(now);
    }

    /* shortcut: printf stops the loop for some milliseconds. If the overrun count ("ov")
     * increases, use interrupt or DMA transmission. */
    if (now - t_telemetry >= TELEMETRY_PERIOD_MS)
    {
      t_telemetry = now;
      /* g and r: heading change since the start, from the Game Rotation Vector and the
       * Rotation Vector. Each report has its own zero reference. */
      const char *imu_state = !imu_ok ? " IMU-FAIL"
                            : !bno085_fresh(&imu, HEADING_SOURCE, now) ? " IMU-STALE" : "";
      printf("tgt %d rpm %d %d cmd %d %d steer %d g %d r %d rst %lu ov %lu%s%s\r\n",
             (int)target_rpm, (int)rpm_l, (int)rpm_r, (int)cmd_l, (int)cmd_r, (int)steer_deg,
             (int)bno085_relative_yaw(&imu, HEADING_SOURCE_GAME_RV),
             (int)bno085_relative_yaw(&imu, HEADING_SOURCE_RV),
             (unsigned long)imu.reset_count, (unsigned long)overruns,
             stalled ? " STALL" : "", imu_state);
      /* A second line, so that the format of the first line does not change. */
      if (DISTANCE_ENABLE)
      {
        printf("dist %d %d %d\r\n", distance_for_telemetry(DIST_FRONT),
               distance_for_telemetry(DIST_RIGHT_LOW), distance_for_telemetry(DIST_RIGHT_HIGH));
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1599;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MOTOR_EN_Pin */
  GPIO_InitStruct.Pin = MOTOR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ---- TIM6: record the two encoder counts. The main loop does the calculations. ---- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &ENC_SAMPLE_TIM)
  {
    sample_count_l = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_L_TIM);
    sample_count_r = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_R_TIM);
    sample_seq++;
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region 0 and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x08FFF000;
  MPU_InitStruct.LimitAddress = 0x08FFFFFF;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Attribute 0 and the memory to be protected
  */
  MPU_AttributesInit.Number = MPU_ATTRIBUTES_NUMBER0;
  MPU_AttributesInit.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  MOTOR_EN_GPIO_Port->BRR = MOTOR_EN_Pin;  /* fault: disable the two motor drivers */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
