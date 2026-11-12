# SAGE

SAGE is an Industrial Automation project for the John Deere challenge.
This repository contains the STM32 firmware for a vehicle with two driven rear wheels and Ackermann steering, and a PC dashboard for the telemetry.

## Firmware

The `John_Deere_Tractor/` project uses the NUCLEO-H533RE board with the STM32H533RET6 microcontroller.

The vehicle has two rear wheels, each with a motor and an encoder.
One servo operates the front Ackermann steering.
A BNO085 sensor gives the heading.

### Keyboard control (serial port, 115200 baud)

Open a serial terminal on the ST-LINK virtual COM port. The firmware reads one key at a time.

| Key | Function |
| --- | --- |
| `W` | Move forward |
| `S` | Move in reverse |
| `A` | Steer 5 degrees more to the left |
| `D` | Steer 5 degrees more to the right |
| Space | Stop immediately, center the steering, and clear a stall |
| `1` to `9` | Set the wheel speed to 10 to 90 percent |
| `0` | Set the wheel speed to 100 percent |

Hold `W` or `S` to move. The vehicle stops when no key arrives for 300 ms.
The steering angle stays until a new steering key or the Space key.

### Update intervals

| Function | Interval |
| --- | --- |
| Encoder sample and wheel speed control | 10 ms |
| Heading and steering control | 20 ms |
| Telemetry | 100 ms |
| BNO085 Game Rotation Vector | 10 ms |
| BNO085 Rotation Vector | 20 ms |

### Present firmware state

- Each rear wheel has a speed controller: a feedforward term and a PI controller. The PI gains are 0 until the motor model is measured.
- The motors stop when a command is 30 percent or more and an encoder gives no counts for 300 ms. The Space key clears this stall condition.
- The telemetry line shows the speed target, the two wheel speeds, the two motor commands, the steering angle, and the count of missed encoder samples.
- The firmware reads two heading reports from the BNO085: the Game Rotation Vector each 10 ms and the Rotation Vector each 20 ms. `HEADING_SOURCE` selects the report for control. The telemetry shows the two headings as `g` and `r`.
- Heading hold is off (`HEADING_HOLD_ENABLE` is 0). When it is on, the `A` and `D` keys change the heading target by 5 degrees, and the heading PID moves the servo.
- The vehicle must not move for 3 seconds after power-on. The firmware sets the heading zero in this time.
- When the heading data is older than 300 ms and heading hold is on, the vehicle does not move. The telemetry shows `IMU-STALE`.
- `App/Config/constants.h` contains all tuning values. The mark `TBD` shows each value that is not measured.
- `docs/PINOUT.md` shows the pin assignment and the motor cable colors.
- Three VL53L1X distance sensors (front, right low, right high) have a driver and one interface, `distance_read()`. They are off (`DISTANCE_ENABLE` is 0) and their XSHUT pins are placeholders.
- `App/StateMachine/state_machine.h` is the provisional design of the automatic mode. It contains comments only.
- `tools/tractor-dashboard/` is a PC dashboard for the telemetry and the keyboard control. It has its own README.

## Project files

All paths below are relative to `John_Deere_Tractor/`.

| Path | Contents |
| --- | --- |
| `Core/Src/main.c` | Main loop, encoder measurement, and speed calculation |
| `App/` | Controllers, actuators, sensors, and constants |
| `Core/Inc/` | Headers that STM32CubeMX generates |
| `Tests/test_control.c` | PC test for the control calculations |
| `Drivers/` | STM32 HAL, CMSIS, and board drivers |
| `John_Deere_Tractor.ioc` | STM32CubeMX configuration |
| `CMakePresets.json` | Debug and Release build presets |
| `.vscode/`, `.settings/` | Editor settings and STM32 tool configuration |

## Open the project

1. Open VS Code.
2. Select **File > Open Folder**.
3. Select the `John_Deere_Tractor` folder in this repository.

The supplied configuration requires STM32 tools for compilation and debugging.
Git excludes the generated files in `John_Deere_Tractor/build/`.

## Make a change

1. From the repository folder, create a branch:

   ```sh
   git switch -c feature/change-name
   ```

2. Make the necessary changes.
3. Examine the changes:

   ```sh
   git status
   git diff
   ```

4. Stage the applicable files. Replace `<file-path>` with each file path:

   ```sh
   git add <file-path>
   ```

5. Commit the changes:

   ```sh
   git commit -m "Describe the change"
   ```

6. Push the branch to GitHub:

   ```sh
   git push -u origin feature/change-name
   ```

7. Open a pull request for review before the merge of the branch into `main`.

## Licenses

The `LICENSE` file applies to the project code in `App/`, `Core/Src/main.c`, `Tests/` and `tools/`.
These parts keep the licenses of their owners: `Drivers/` and the generated files (STMicroelectronics), `App/Sensors/sh2/` (see its `NOTICE.txt`), and `App/Sensors/vl53l1x_uld/` (see the file headers).

## Verification status

The firmware compiles and links for the STM32H533 with the GCC ARM compiler. It is not tested on the hardware.
The PC test in `John_Deere_Tractor/Tests/test_control.c` passes. It tests the calculations only.
