# SAGE

SAGE is an Industrial Automation project for the John Deere challenge.
This repository contains the STM32 firmware for a vehicle with two driven rear wheels and Ackermann steering, and a PC dashboard for the telemetry.

## Firmware

The `firmware/H533RE/` project uses the NUCLEO-H533RE board with the STM32H533RET6 microcontroller.
The `firmware/F411RE/` project is the same firmware for the NUCLEO-F411RE board (STM32F411RET6). Four wires move: see `docs/PINOUT_F411RE.md`.
Open one of the two folders in VS Code. Each folder has its own copy of `App/`: make a fix in both.

The vehicle has two rear wheels, each with a motor and an encoder.
One servo operates the front Ackermann steering.
A BNO085 sensor gives the heading.

### Keyboard control (USB serial port, 115200 baud, or Bluetooth)

Open a serial terminal on the ST-LINK virtual COM port, or on the Bluetooth port (see below). The firmware reads one key at a time from each link. The two links accept the same keys and send the same telemetry.

| Key | Function |
| --- | --- |
| `W` | Move forward |
| `S` | Move in reverse |
| `A` | Heading target 30 degrees to the left (heading hold), or steer 10 degrees more to the left |
| `D` | Heading target 30 degrees to the right (heading hold), or steer 10 degrees more to the right |
| Space | Stop immediately, center the steering, and clear a stall |
| `1` to `9` | Set the wheel speed to 10 to 90 percent |
| `0` | Set the wheel speed to 100 percent |

Hold `W` or `S` to move. The vehicle stops when no `W` or `S` key arrives for 300 ms. Other keys do not restart this time.
The steering angle stays until a new steering key or the Space key.

### Bluetooth (HC-06, not tested)

The firmware also reads keys and sends telemetry on USART1: PA9 = STM32 TX to the module RXD, PA10 = STM32 RX from the module TXD, 9600 baud (`BT_BAUD`, the HC-06 factory setting). Connect the module VCC to 5 V and GND to GND. Module: HC-06 (confirmed). PA9 is morpho CN10-2 and PA10 is CN10-4.

1. Pair the module in the Bluetooth settings of the PC (the usual PIN is `1234`).
2. Double-click `Start BT.command` (macOS) or `Start BT.bat` (Windows) in `tools/tractor-dashboard`. It finds the port.
3. If it finds no port, give the name: `--port /dev/cu.HC-06` on macOS. On Windows, look in Device Manager > Ports for the "Outgoing" Bluetooth COM port, for example `--port COM6`.

At 9600 baud a telemetry line takes about 80 ms. When the previous line is not sent yet, the firmware does not send the next one on Bluetooth (the USB link gets all lines). The USB link stays on.

### Update intervals

| Function | Interval |
| --- | --- |
| Encoder sample and wheel speed control | 10 ms |
| Heading and steering control | 20 ms |
| Telemetry | 100 ms |
| BNO085 Game Rotation Vector | 10 ms |
| BNO085 Rotation Vector | 20 ms |

### Present firmware state

- `App/Control/control.c` is the control core: keys, wheel speed control and heading hold. It has no hardware calls. The firmware and the dashboard sim run this same code with the same `constants.h`.
- Each rear wheel has a speed controller: a feedforward term and a PI controller (Kp 0.4, Ki 6.5, Kd 0). The gains come from a model of the left motor, with the wheel in the air. Change them after a test on the ground.
- The motors stop when a command is 30 percent or more and an encoder gives no counts for 300 ms. The Space key clears this stall condition.
- The telemetry line shows the speed target, the two wheel speeds, the two motor commands, the steering angle, and the count of missed encoder samples.
- The firmware reads two heading reports from the BNO085: the Game Rotation Vector each 10 ms and the Rotation Vector each 20 ms. `HEADING_SOURCE` selects the report for control. The telemetry shows the two headings as `g` and `r`.
- Heading hold is on (`HEADING_HOLD_ENABLE` is 1, Kp 0.6, Ki 0.5, Kd 0.5, not tested on the car). The `A` and `D` keys change the heading target by 30 degrees, and the heading PID moves the servo. The PID output limit is the steering limit (±30 degrees). The D term uses the filtered change of the measured heading (`HEADING_D_FILTER_TAU_S`), so a target change gives no D step. A heading gap longer than 300 ms restarts the heading controller.
- The vehicle must not move for 3 seconds after power-on. The firmware sets the heading zero in this time.
- When heading hold is on, the vehicle does not move in these conditions: the BNO085 did not start (`IMU-FAIL`), the heading data is older than 300 ms (`IMU-STALE`), or the BNO085 did a reset after power-on (`rst` is more than 0). A reset is cleared only by a new power-on.
- `App/Config/constants.h` contains all tuning values. The mark `TBD` shows each value that is not measured.
- `docs/PINOUT_H533RE.md` shows the pin assignment and the motor cable colors.
- Three VL53L1X distance sensors (front, right low, right high) have a driver and one interface, `distance_read()`. They are off (`DISTANCE_ENABLE` is 0) and their XSHUT pins are placeholders.
- `App/StateMachine/state_machine.h` is the provisional design of the automatic mode. It contains comments only.
- `tools/tractor-dashboard/` is a PC dashboard for the telemetry and the keyboard control. It has its own README.

## Project files

All paths below are relative to `firmware/H533RE/`.

| Path | Contents |
| --- | --- |
| `Core/Src/main.c` | Main loop, encoder measurement, and speed calculation |
| `App/` | Controllers, actuators, sensors, and constants |
| `Core/Inc/` | Headers that STM32CubeMX generates |
| `App/Control/control.c` | Control core, shared with the dashboard sim |
| `Tests/test_control.c` | PC test for the PID and the calculations |
| `Tests/test_core.c` | PC test for the control core (heading and wheel behavior) |
| `Tests/test_heading.c` | PC test for the BNO085 side of the heading |
| `Tests/replay.c` | Firmware-side replay, used by the dashboard equivalence test |
| `Drivers/` | STM32 HAL, CMSIS, and board drivers |
| `H533RE.ioc` | STM32CubeMX configuration |
| `CMakePresets.json` | Debug and Release build presets |
| `.vscode/`, `.settings/` | Editor settings and STM32 tool configuration |

## Open the project

1. Open VS Code.
2. Select **File > Open Folder**.
3. Select the `firmware/H533RE` folder in this repository.

The supplied configuration requires STM32 tools for compilation and debugging.
Git excludes the generated files in `firmware/*/build/`.

## Use another PC (macOS, Windows or Linux)

No file in the repository has a path that is specific to one PC. Clone the repository anywhere.

Firmware (build and flash):

1. Install VS Code and the STM32Cube for VS Code extension. It installs the ARM compiler, CMake and Ninja.
2. Windows: install the ST-LINK USB driver (STSW-LINK009) or STM32CubeProgrammer, which includes it.
3. Open `firmware/H533RE` or `firmware/F411RE` (**File > Open Folder**), build, then flash through the ST-LINK USB connector.

Dashboard:

1. Install Python 3.9 or newer. Windows: python.org installer, with "Add python.exe to PATH".
2. Install a C compiler for the PC (the sim compiles the firmware control core):
   - macOS: `xcode-select --install`
   - Windows: MSYS2, then `pacman -S mingw-w64-ucrt-x86_64-gcc` in the MSYS2 UCRT64 shell. Add `C:\msys64\ucrt64\bin` to the PATH.
   - Linux: `gcc` from the package manager.
3. Windows only, for Live mode: `py -m pip install pyserial`
4. Double-click a `Start` file in `tools/tractor-dashboard`.

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
The PC tests in `firmware/H533RE/Tests/` pass. They test the calculations and the control logic only. The command to run each test is at the top of its file.
