# Pin assignment proposal

Status: configured in STM32CubeMX and used by the firmware. It is not tested on the hardware.

The signal and alternate function data comes from the ST pin database for the STM32H533RETx
(`STM32_open_pin_data`). The Arduino and morpho connector positions are NOT verified.
Find each pin on the board with the NUCLEO-H533RE user manual before a wire is connected.

| Function | Pin | Signal | Alternate function |
| --- | --- | --- | --- |
| Left motor, RPWM (forward) | PC6 | TIM8_CH1 | AF3 |
| Left motor, LPWM (reverse) | PC7 | TIM8_CH2 | AF3 |
| Right motor, RPWM (forward) | PC8 | TIM8_CH3 | AF3 |
| Right motor, LPWM (reverse) | PC9 | TIM8_CH4 | AF3 |
| Motor enable (R_EN and L_EN of the two drivers) | PB10 | GPIO output, label `MOTOR_EN` | - |
| Left encoder, channel A | PA0 | TIM2_CH1 | AF1 |
| Left encoder, channel B | PA1 | TIM2_CH2 | AF1 |
| Right encoder, channel A | PA6 | TIM3_CH1 | AF2 |
| Right encoder, channel B | PA7 | TIM3_CH2 | AF2 |
| Steering servo | PA8 | TIM1_CH1 | AF1 |
| BNO085 SCL | PB6 | I2C1_SCL | AF4 |
| BNO085 SDA | PB7 | I2C1_SDA | AF4 |
| Serial port (no change) | PA2, PA3 | USART2 | - |
| Distance sensors SCL / SDA (3 x VL53L1X) | PB6 / PB7 | I2C1, same bus as the BNO085 | AF4 |
| Distance sensor XSHUT: front | PB0 | GPIO output, PLACEHOLDER | - |
| Distance sensor XSHUT: right, low | PB1 | GPIO output, PLACEHOLDER | - |
| Distance sensor XSHUT: right, high | PB2 | GPIO output, PLACEHOLDER | - |

The three XSHUT pins are placeholders. They are not assigned in STM32CubeMX and not checked on the board.
`App/Sensors/distance.c` configures them. Change them in `App/Config/pins.h`.

## Motor and encoder cable (see `motor_pinout.png`, seller data)

| Wire color | Function | Connect to |
| --- | --- | --- |
| Red, black | Motor power | M+ and M- of the BTS7960 module |
| Blue | Encoder supply, 3.3 V to 24 V | 3.3 V of the Nucleo board |
| Gray | Encoder ground | GND |
| Yellow | Encoder channel A | PA0 (left) or PA6 (right) |
| White | Encoder channel B | PA1 (left) or PA7 (right) |

Pins that become free: PB5 (old encoder channel B). PC6 and PC7 change from GPIO to PWM.

## STM32CubeMX settings

All timers have a 32 MHz clock.

| Peripheral | Setting |
| --- | --- |
| TIM8 | PWM Generation on CH1, CH2, CH3, CH4. Prescaler 0. Counter Period 1599 (20 kHz). |
| TIM2 | Combined Channels: Encoder Mode. Encoder Mode: TI1 and TI2 (x4). Counter Period: maximum. Input Filter 8 on the two channels. |
| TIM3 | Combined Channels: Encoder Mode. Encoder Mode: TI1 and TI2 (x4). Counter Period 65535. Input Filter 8 on the two channels. |
| TIM1 | PWM Generation on CH1. Prescaler 31 (1 MHz tick). Counter Period 19999 (50 Hz). |
| TIM6 | Prescaler 3199. Counter Period 99 (10 ms). Make sure that the value shows 99. |
| I2C1 | I2C mode. Speed: Fast Mode, 400 kHz. |
| PB10 | GPIO_Output, initial level Low, label `MOTOR_EN`. |
| PB10, PB5, PC6, PC7 | Remove the old `ENC_A`, `ENC_B`, `MOTOR_IN1`, `MOTOR_IN2` assignments and the EXTI10 interrupt. |

## Electrical checks before connection

- Encoder: use the 3.3 V supply. The seller data permits 3.3 V to 24 V, and the outputs then stay at
  3.3 V. Do not use 5 V unless PA0, PA1, PA6 and PA7 are confirmed as 5 V tolerant.
- BTS7960 module: its input buffer has a 5 V supply. Make sure that it reads the 3.3 V PWM signals
  of the STM32 as high. Test one driver without a motor first.
- Motor enable: PB10 has no defined level during a reset of the microcontroller. If the BTS7960 module
  has no pull-down resistor on R_EN and L_EN, add a 10 kOhm resistor from `MOTOR_EN` to ground.
- Servo: supply it from a separate 5 V to 6 V regulator, not from the Nucleo board. Connect the grounds.
- Connect the grounds of the battery, the two drivers and the Nucleo board.
