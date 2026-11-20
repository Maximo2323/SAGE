# Pin assignment: NUCLEO-F411RE

Status: ported from the NUCLEO-H533RE version. It compiles. It is not tested on the hardware.

The STM32F411RE has no TIM6 and no TIM8. Thus three functions use a different timer:

- Motor PWM: TIM8 -> **TIM3**, on the same pins PC6 to PC9.
- Right encoder: TIM3 -> **TIM4**. The pins change from PA6/PA7 to **PB6/PB7**.
- 10 ms encoder sample: TIM6 -> **TIM5** (no pin).

TIM4 needs PB6/PB7. Thus the I2C bus moves from PB6/PB7 to **PB8/PB9** (Arduino D15/D14).
All other pins are the same as on the H533RE.

Board positions: ST user manual UM1724 (NUCLEO-64, MB1136). Before you connect a wire, find each pin on the board.

## Changes from the H533RE (4 wires move)

| Signal | H533RE | F411RE |
| --- | --- | --- |
| Right encoder, channel A (yellow) | PA6 | **PB6** (D10, CN10-17) |
| Right encoder, channel B (white) | PA7 | **PB7** (CN7-21) |
| I2C SCL (BNO085 + VL53L1X) | PB6 | **PB8** (D15, CN10-3) |
| I2C SDA (BNO085 + VL53L1X) | PB7 | **PB9** (D14, CN10-5) |

## Full pin map

| Function | Pin | Board position | Signal | AF |
| --- | --- | --- | --- | --- |
| Left motor, RPWM (forward) | PC6 | CN10-4 | TIM3_CH1 | AF2 |
| Left motor, LPWM (reverse) | PC7 | D9, CN10-19 | TIM3_CH2 | AF2 |
| Right motor, RPWM (forward) | PC8 | CN10-2 | TIM3_CH3 | AF2 |
| Right motor, LPWM (reverse) | PC9 | CN10-1 | TIM3_CH4 | AF2 |
| Motor enable (R_EN and L_EN of the two drivers) | PB10 | D6, CN10-25 | GPIO output, `MOTOR_EN` | - |
| Left encoder, channel A | PA0 | A0, CN7-28 | TIM2_CH1 | AF1 |
| Left encoder, channel B | PA1 | A1, CN7-30 | TIM2_CH2 | AF1 |
| Right encoder, channel A | PB6 | D10, CN10-17 | TIM4_CH1 | AF2 |
| Right encoder, channel B | PB7 | CN7-21 | TIM4_CH2 | AF2 |
| Steering servo | PA8 | D7, CN10-23 | TIM1_CH1 | AF1 |
| BNO085 + VL53L1X SCL | PB8 | D15, CN10-3 | I2C1_SCL | AF4 |
| BNO085 + VL53L1X SDA | PB9 | D14, CN10-5 | I2C1_SDA | AF4 |
| USB serial (ST-LINK virtual COM port) | PA2, PA3 | D1, D0 (do not use) | USART2 | AF7 |
| Bluetooth HC-06: STM32 TX -> module RXD | PA9 | D8, CN10-21 | USART1_TX, set up in main.c | AF7 |
| Bluetooth HC-06: STM32 RX <- module TXD | PA10 | D2, CN10-33 | USART1_RX, pull-up | AF7 |
| Bluetooth HC-06: VCC / GND | 5V / GND | CN7-18 / CN7-20 | 3.6 V to 6 V in, 3.3 V logic | - |
| Distance sensor XSHUT: front | PB0 | A3, CN7-34 | GPIO output, PLACEHOLDER | - |
| Distance sensor XSHUT: right, low | PB1 | CN10-24 | GPIO output, PLACEHOLDER | - |
| Distance sensor XSHUT: right, high | PB2 | CN10-22 | GPIO output, PLACEHOLDER | - |
| Green LED LD2 | PA5 | D13 | GPIO output (off) | - |
| User button B1 | PC13 | - | GPIO input (not used) | - |
| SWD (on-board ST-LINK) | PA13, PA14 | CN7-13, CN7-15 | SWDIO, SWCLK | - |

The XSHUT pins are placeholders. `App/Sensors/distance.c` sets them up. Change them in `App/Config/pins.h`.
PB2 is also BOOT1. The microcontroller reads BOOT1 only at reset and only when BOOT0 = 1, so it is safe as an output.

## Timer settings (identical to the H533RE)

The clock is HSI 16 MHz -> PLL -> 32 MHz. AHB, APB1, APB2 and all timer clocks = 32 MHz, the same as on the H533RE.
Thus all prescaler and period values stay the same.

| Timer | Function | Setting |
| --- | --- | --- |
| TIM3 | Motor PWM, CH1 to CH4 | Prescaler 0. Period 1599 (20 kHz). main.c sets the period from `MOTOR_PWM_HZ`. |
| TIM2 | Left encoder, 32 bit | Encoder mode TI1 and TI2 (x4). Period maximum. Input filter 8. |
| TIM4 | Right encoder, 16 bit | Encoder mode TI1 and TI2 (x4). Period 65535. Input filter 8. |
| TIM1 | Servo, CH1 | Prescaler 31 (1 MHz tick). Period 19999 (50 Hz). |
| TIM5 | 10 ms sample, interrupt | Prescaler 3199. Period 99. main.c sets the period from `WHEEL_CONTROL_PERIOD_MS`. |
| I2C1 | BNO085, VL53L1X | Fast mode, 400 kHz. |
| USART2 | ST-LINK serial, printf | 115200 baud, 8N1. |

## Motor and encoder cable (see `motor_pinout.png`, seller data)

| Wire color | Function | Connect to |
| --- | --- | --- |
| Red, black | Motor power | M+ and M- of the BTS7960 module |
| Blue | Encoder supply, 3.3 V to 24 V | 3.3 V of the Nucleo board |
| Gray | Encoder ground | GND |
| Yellow | Encoder channel A | PA0 (left) or PB6 (right) |
| White | Encoder channel B | PA1 (left) or PB7 (right) |

## Electrical checks before connection

- Encoder: use the 3.3 V supply. The outputs then stay at 3.3 V.
- BTS7960 module: its input buffer has a 5 V supply. Make sure that it reads the 3.3 V PWM signals of the STM32 as high. Test one driver without a motor first.
- Motor enable: PB10 has no defined level during a reset. If the BTS7960 module has no pull-down resistor on R_EN and L_EN, add a 10 kOhm resistor from `MOTOR_EN` to ground.
- I2C: the BNO085 and VL53L1X boards usually have pull-up resistors. If not, add 4.7 kOhm from SCL and SDA to 3.3 V.
- Servo: supply it from a separate 5 V to 6 V regulator, not from the Nucleo board. Connect the grounds.
- Connect the grounds of the battery, the two drivers and the Nucleo board.
