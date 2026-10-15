# SAGE

SAGE is an Industrial Automation project for the John Deere challenge.
This repository contains STM32 firmware for DC motor control and encoder measurement.

## Firmware

The `Interrup_econder/` project uses the NUCLEO-H533RE board with the STM32H533RET6 microcontroller.
The firmware provides these functions:

- Motor speed and direction control through pulse-width modulation (PWM) and two digital outputs.
- Encoder pulse counts through interrupts on both edges of channel A, with channel B for direction detection.
- Output shaft speed calculation in revolutions per minute (RPM), with a nominal sample interval of 100 ms.
- A duty cycle sequence of 30, 60, 100, 0, −60, and 0 percent, with 10 seconds per step.
- A duty cycle ramp of one percentage point per 20 ms.
- Motor stop after three samples without encoder counts, when the duty cycle magnitude is at least 30 percent.

## Project files

All paths below are relative to `Interrup_econder/`.

| Path | Contents |
| --- | --- |
| `Core/Src/main.c` | Motor control, encoder measurement, and speed calculation |
| `Core/Inc/` | Application headers |
| `Drivers/` | STM32 HAL, CMSIS, and board drivers |
| `Interrup_econder.ioc` | STM32CubeMX configuration |
| `CMakePresets.json` | Debug and Release build presets |
| `.vscode/`, `.settings/` | Editor settings and STM32 tool configuration |

## Open the project

1. Open VS Code.
2. Select **File > Open Folder**.
3. Select the `Interrup_econder` folder in this repository.

The supplied configuration requires STM32 tools for compilation and debugging.
Git excludes the generated files in `Interrup_econder/build/`.

## Make a change

1. From the repository folder, create a branch:

   ```sh
   git switch -c feature/your-change
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
   git push -u origin feature/your-change
   ```

7. Open a pull request for review before you merge the branch into `main`.

## Verification status

The imported firmware files match the supplied archive.
All source files and include directories referenced by the generated CMake configuration are present.
Compilation and hardware operation remain unverified.
