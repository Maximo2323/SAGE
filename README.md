# SAGE

Industrial Automation project for the John Deere challenge.

## DC motor control and encoder reading

The initial firmware project is in `Interrup_econder/`, imported from the team's original project archive. It targets the **NUCLEO-H533RE (STM32H533RET6)** and includes motor PWM control, encoder edge counting, RPM calculation, a duty-cycle ramp, and stall detection.

## Project files

- `Interrup_econder/Core/Src/main.c`: application logic.
- `Interrup_econder/Core/Inc/`: application headers.
- `Interrup_econder/Drivers/`: STM32 HAL, CMSIS, and board drivers.
- `Interrup_econder/Interrup_econder.ioc`: STM32CubeMX configuration.
- `Interrup_econder/CMakePresets.json`: Debug and Release build presets.
- `Interrup_econder/.vscode/` and `.settings/`: supplied editor and STM32 tool configuration.

## Open the firmware

In VS Code, choose **File > Open Folder** and select `Interrup_econder` inside this repository. This folder contains the CMake project and supplied VS Code configuration. Building and debugging require the STM32 tools referenced in that configuration. Build output is generated locally under `Interrup_econder/build/` and is excluded from Git.

## Work with branches

From the repository folder, create a branch before making a change:

```sh
git switch -c feature/your-change
```

After editing, review and commit your changes, then upload the branch:

```sh
git status
git diff
git add <files-you-changed>
git commit -m "Describe your change"
git push -u origin feature/your-change
```

Open a pull request on GitHub to review and merge the branch into `main`.

## Initial import validation

The firmware source is preserved from the supplied archive, and the imported CMake source and include paths were checked for missing files. Compilation and hardware operation have not been verified as part of this import.
