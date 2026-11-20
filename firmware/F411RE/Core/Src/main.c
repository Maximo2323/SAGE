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
#include <stdio.h>
#include "constants.h"
#include "pins.h"
#include "motor.h"
#include "servo.h"
#include "control.h"
#include "bno085.h"
#include "heading_hold.h"
#include "distance.h"
#include "math_utils.h"
#include <math.h>
#include "route.h"
#include "route_steps.h"

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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Written by the TIM5 interrupt, read by the main loop */
static volatile uint32_t sample_seq     = 0;  /* number of encoder samples since the start */
static volatile uint16_t sample_count_l = 0;  /* left encoder count at the last sample     */
static volatile uint16_t sample_count_r = 0;  /* right encoder count at the last sample    */
UART_HandleTypeDef huart_bt;                  /* Bluetooth module link (USART1). The IRQ handler uses it. */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static Motor motor_l, motor_r;
static Servo servo;

static Bno085      imu;
static HeadingHold heading;
static Control     ctl;   /* control core (control.c): the same code runs in the dashboard sim */

/* Route runner (route.c, the same code as the dashboard sim). Key R runs the route of route_steps.h.
 * Any other operator key stops it. */
static Route   route;
static int32_t odo_counts_l = 0, odo_counts_r = 0;  /* encoder counts since the start, + = forward */
static int     route_step_sent = -1;                 /* step number last sent on the links          */

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

/* Bluetooth module (HC-06) on USART1, BT_BAUD, 8 data bits, no parity, 1 stop bit.
 * shortcut: set up here, not in CubeMX. If you add USART1 in CubeMX, remove this function. */
static void bt_init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin       = BT_TX_PIN | BT_RX_PIN;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;  /* RX stays high (idle) when no module is connected: no false keys */
  gpio.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = BT_GPIO_AF;
  HAL_GPIO_Init(BT_GPIO_PORT, &gpio);

  huart_bt.Instance            = BT_UART;
  huart_bt.Init.BaudRate       = BT_BAUD;
  huart_bt.Init.WordLength     = UART_WORDLENGTH_8B;
  huart_bt.Init.StopBits       = UART_STOPBITS_1;
  huart_bt.Init.Parity         = UART_PARITY_NONE;
  huart_bt.Init.Mode           = UART_MODE_TX_RX;
  huart_bt.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart_bt.Init.OverSampling   = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart_bt) != HAL_OK) { Error_Handler(); }
  HAL_NVIC_SetPriority(BT_UART_IRQn, 5, 0);  /* transmission only; the keys are polled */
  HAL_NVIC_EnableIRQ(BT_UART_IRQn);
}

/* Read one received character. Returns -1 when there is none. This function does not wait.
 * A character with a framing, noise or parity error is not used.
 * F4 USART: read SR, then DR. This sequence also clears ORE, FE, NE and PE. */
static int read_char(USART_TypeDef *uart)
{
  const uint32_t sr = uart->SR;
  if ((sr & USART_SR_RXNE) == 0U)
  {
    if ((sr & USART_SR_ORE) != 0U) { (void)uart->DR; }  /* overrun and no data: clear the flag */
    return -1;
  }
  const int ch = (int)(uart->DR & 0xFFU);
  if ((sr & (USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U) { return -1; }
  return ch;
}

/* printf (syscalls.c, _write) sends each character here: USART2, the ST-LINK virtual COM port. */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  (void)HAL_UART_Transmit(&huart2, &c, 1U, HAL_MAX_DELAY);
  return ch;
}

/* Send a telemetry line on the two links. The USB link (printf) waits; the Bluetooth link uses
 * the interrupt and does not wait. When the previous Bluetooth line is not sent yet, this line
 * is not sent on Bluetooth (9600 baud is slow). */
static void send_line(const char *line, int len)
{
  static char bt_tx[160];
  if (len <= 0) { return; }
  if (len >= (int)sizeof bt_tx) { len = (int)sizeof bt_tx - 1; }
  printf("%s", line);
  if (huart_bt.gState == HAL_UART_STATE_READY)
  {
    for (int i = 0; i < len; i++) { bt_tx[i] = line[i]; }
    (void)HAL_UART_Transmit_IT(&huart_bt, (uint8_t *)bt_tx, (uint16_t)len);
  }
}

/* Send the outputs of the control core to the motors, the servo and the enable pin. */
static void apply_outputs(void)
{
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, ctl.motor_enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
  motor_set(&motor_l, ctl.cmd_l);
  motor_set(&motor_r, ctl.cmd_r);
  servo_set_deg(&servo, ctl.steer_deg);
}

/* The TIM5 interrupt gives the encoder samples. Copy them without an interrupt in between. */
static void wheel_control(uint32_t now)
{
  __disable_irq();
  const uint32_t seq     = sample_seq;
  const uint16_t count_l = sample_count_l;
  const uint16_t count_r = sample_count_r;
  __enable_irq();
  /* Wheel distance for the route runner. The 16-bit difference also works over the counter wrap. */
  static uint32_t odo_seq = 0;
  static uint16_t odo_prev_l = 0, odo_prev_r = 0;
  if (seq != odo_seq)
  {
    odo_counts_l += ENC_DIR_LEFT * (int16_t)(uint16_t)(count_l - odo_prev_l);
    odo_counts_r += ENC_DIR_RIGHT * (int16_t)(uint16_t)(count_r - odo_prev_r);
    odo_prev_l = count_l; odo_prev_r = count_r; odo_seq = seq;
  }
  control_wheels(&ctl, seq, count_l, count_r, now);
}

/* ToF readings for the route runner, in RouteSensor order. -1 = no target, or not valid. */
static void route_read_tof(void *ctx, int32_t mm[3])
{
  (void)ctx;
  for (int k = 0; k < 3; k++)
  {
    uint16_t v = 0;
    mm[k] = (DISTANCE_ENABLE && distance_read((DistId)k, &v) == DIST_OK) ? (int32_t)v : -1;
  }
}

/* One route line on the two links: "route " + text. The dashboard reads these lines. */
static void route_line(const char *text)
{
  char line[ROUTE_MSG_LEN + 80];
  send_line(line, snprintf(line, sizeof line, "route %s\r\n", text));
}

/* Id of the flashed route and its step count. The dashboard compares the id with its saved route. */
static void route_send_id(void)
{
  char t[48];
  snprintf(t, sizeof t, "id %s %d", ROUTE_ID, ROUTE_STEP_COUNT);
  route_line(t);
}

/* One key from an operator link (USB or Bluetooth). R: run the route. ?: send the route id.
 * Other keys: stop the route, then give the key to the control core. */
static void operator_key(char k, uint32_t now)
{
  if (k == 'r' || k == 'R')
  {
    if (!route.running)
    {
      RouteConfig cfg;
      route_config_default(&cfg);
      route_start(&route, ROUTE_STEPS, ROUTE_STEP_COUNT, 0, false, &cfg, route_read_tof, NULL);
      route_step_sent = -1;
      route_send_id();
      route_line("run");
    }
    return;
  }
  if (k == '?') { route_send_id(); return; }
  if (route.running) { route_stop(&route); route_line("stop"); }
  control_key(&ctl, k, now);
}

/* Each main loop, before the operator keys: one step of the route runner, as in the dashboard sim. */
static void route_tick(uint32_t now)
{
  if (!route.running) { return; }
  const RouteInput in = {
    now,
    (float)(odo_counts_l + odo_counts_r) * 0.5f / (float)COUNTS_PER_WHEEL_REV * 3.14159265f * WHEEL_DIAMETER_MM,
    ctl.current_yaw, ctl.heading_goal, fmaxf(fabsf(ctl.rpm_l), fabsf(ctl.rpm_r)) };
  route_step(&route, &in);
  if (route.set_goal) { ctl.heading_goal = route.goal_deg; }
  for (int k = 0; k < route.nkeys; k++) { control_key(&ctl, route.keys[k], now); }
  for (int k = 0; k < route.nmsg; k++)
  {
    char t[ROUTE_MSG_LEN + 64];
    const unsigned long s = now / 1000U, ds = now % 1000U / 100U;
    if (route.report[k] >= 0)  /* checkpoint: the car cannot know its true error, so send the readings */
    {
      int32_t mm[3];
      route_read_tof(NULL, mm);
      snprintf(t, sizeof t, "log %lu.%lu %d checkpoint %s: ToF F/RL/RH %ld / %ld / %ld mm", s, ds,
               route.report[k] + 1, ROUTE_STEPS[route.report[k]].label, (long)mm[0], (long)mm[1], (long)mm[2]);
    }
    else
    {
      snprintf(t, sizeof t, "log %lu.%lu %s", s, ds, route.msg[k]);
    }
    route_line(t);
  }
  if (route.i != route_step_sent)
  {
    char t[24];
    route_step_sent = route.i;
    snprintf(t, sizeof t, "step %d", route.i);
    route_line(t);
  }
  if (!route.running) { route_line("stop"); }
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* MOTOR_PWM_HZ sets the motor PWM frequency. This replaces the period value from CubeMX.
   * F411: the motor timer is TIM3, on APB1.
   * shortcut: correct only while the APB1 clock divider is 1 (timer clock = PCLK1). */
  __HAL_TIM_SET_AUTORELOAD(&MOTOR_PWM_TIM,
      HAL_RCC_GetPCLK1Freq() / (MOTOR_PWM_TIM.Init.Prescaler + 1U) / MOTOR_PWM_HZ - 1U);
  motor_init(&motor_l, &MOTOR_PWM_TIM, MOTOR_L_FWD_CH, MOTOR_L_REV_CH);
  motor_init(&motor_r, &MOTOR_PWM_TIM, MOTOR_R_FWD_CH, MOTOR_R_REV_CH);
  servo_init(&servo, &SERVO_TIM, SERVO_CHANNEL);

  /* BNO085 start and heading zero: about 3 s. The vehicle must not move during this time.
   * If the sensor does not answer, the firmware continues without heading data. */
  heading_hold_init(&heading, &imu);
  const bool imu_ok = bno085_begin(&imu, &BNO_I2C, BNO085_I2C_ADDR)
                   && heading_hold_calibrate(&heading, HEADING_CAL_SAMPLES, HEADING_CAL_WARMUP_MS);

  /* A sensor that does not answer gives DIST_NOT_VALID. The other sensors continue to operate. */
  if (DISTANCE_ENABLE) { distance_init(&TOF_I2C); }

  HAL_TIM_Encoder_Start(&ENC_L_TIM, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&ENC_R_TIM, TIM_CHANNEL_ALL);
  /* WHEEL_CONTROL_PERIOD_MS sets the TIM5 period. This replaces the period value from CubeMX,
   * so the sample time and the RPM calculation always agree.
   * shortcut: correct only while the APB1 clock divider is 1 (timer clock = PCLK1). */
  __HAL_TIM_SET_AUTORELOAD(&ENC_SAMPLE_TIM,
      HAL_RCC_GetPCLK1Freq() / (ENC_SAMPLE_TIM.Init.Prescaler + 1U) / 1000U * WHEEL_CONTROL_PERIOD_MS - 1U);
  HAL_TIM_Base_Start_IT(&ENC_SAMPLE_TIM);

  uint32_t t_telemetry = HAL_GetTick();
  uint32_t t_steering  = t_telemetry;
  uint32_t t_imu_read  = t_telemetry;
  control_init(&ctl, imu_ok, t_telemetry);
  apply_outputs();
  bt_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    route_tick(now);

    /* One key from each link: USB (ST-LINK, USART2) and Bluetooth (USART1). Same keys. */
    int key = read_char(huart2.Instance);
    if (key >= 0) { operator_key((char)key, now); }
    key = read_char(huart_bt.Instance);
    if (key >= 0) { operator_key((char)key, now); }

    control_tick(&ctl, now);
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
      const HeadingSample sample = heading_hold_sample(&heading, now);
      control_steering(&ctl, &sample);
    }
    apply_outputs();

    /* shortcut: printf stops the loop for some milliseconds. If the overrun count ("ov")
     * increases, use interrupt or DMA transmission. */
    if (now - t_telemetry >= TELEMETRY_PERIOD_MS)
    {
      t_telemetry = now;
      /* g and r: heading change since the start, from the Game Rotation Vector and the
       * Rotation Vector. Each report has its own zero reference. */
      const char *imu_state = !imu_ok ? " IMU-FAIL"
                            : !bno085_fresh(&imu, HEADING_SOURCE, now) ? " IMU-STALE" : "";
      char line[160];
      int  len = snprintf(line, sizeof line,
             "tgt %d rpm %d %d cmd %d %d steer %d g %d r %d rst %lu ov %lu%s%s\r\n",
             (int)ctl.target_rpm, (int)ctl.rpm_l, (int)ctl.rpm_r, (int)ctl.cmd_l, (int)ctl.cmd_r,
             (int)ctl.steer_deg,
             (int)bno085_relative_yaw(&imu, HEADING_SOURCE_GAME_RV),
             (int)bno085_relative_yaw(&imu, HEADING_SOURCE_RV),
             (unsigned long)imu.reset_count, (unsigned long)ctl.overruns,
             ctl.stalled ? " STALL" : "", imu_state);
      send_line(line, len);
      /* A second line, so that the format of the first line does not change. */
      if (DISTANCE_ENABLE)
      {
        len = snprintf(line, sizeof line, "dist %d %d %d\r\n", distance_for_telemetry(DIST_FRONT),
                       distance_for_telemetry(DIST_RIGHT_LOW), distance_for_telemetry(DIST_RIGHT_HIGH));
        send_line(line, len);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *        HSI 16 MHz -> PLL -> SYSCLK 32 MHz. AHB, APB1, APB2 and all timer clocks = 32 MHz.
  *        This is the same clock as the H533RE version, so the timer settings do not change.
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  * VCO in = 16 / 8 = 2 MHz. VCO out = 2 * 128 = 256 MHz. SYSCLK = 256 / 8 = 32 MHz.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 3199;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  *        ST-LINK virtual COM port: 115200 baud, 8 data bits, no parity, 1 stop bit.
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
/* ---- TIM5: record the two encoder counts. The main loop does the calculations. ---- */
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


/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  MOTOR_EN_GPIO_Port->BSRR = (uint32_t)MOTOR_EN_Pin << 16U;  /* fault: disable the two motor drivers */
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
