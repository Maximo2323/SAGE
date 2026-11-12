# Tractor dashboard

Python standard library + one local HTML canvas page. No packages, cloud service, build step, or firmware changes. Serial support is macOS/Linux. Run only one control tab and close other serial terminals.

## Run

From this folder:

```sh
python3 dashboard.py --demo
```

Open http://127.0.0.1:8766. The demo is explicitly synthetic; it uses illustrative diameter/wheelbase/track of 100/300/220 mm, ideal wheel speed response and a steering model. It is not a digital twin or a controller validation model.

For the board, replace the device with its actual serial port:

```sh
python3 dashboard.py --port /dev/cu.usbmodemXXXX
```

There was no USB STM32 serial device visible during development. Bluetooth ports were not opened. No board was driven or flashed. A connected project folder does not establish a connected or identically flashed robot.

Optional estimated travel, after measuring the robot:

```sh
python3 dashboard.py --port /dev/cu.usbmodemXXXX --geometry-mm DIAMETER WHEELBASE TRACK
```

Replace all three words with positive dimensions in millimetres. These arguments affect the dashboard only; they do not change firmware. Without them serial mode displays a stationary schematic, valid Game yaw and commanded steering. With them it integrates average measured RPM and commanded steering as a bicycle model, labels it **estimated odometry**, and draws dotted rear-wheel trails. It does not measure X/Y, correct wheel slip, or know actual steering angles. Gaps over 300 ms are not integrated. Stale data is marked after 500 ms. Pose starts at an arbitrary origin and is lost on restart. Demo and estimated trails keep the latest 400 samples.

W/S drive while held; updates are sent every 80 ms without relying on operating-system key repeat. A/D change steering once per press. Digits set 10–100% (0 = 100%). Drive release, Space, focus loss and hidden page send the existing Space command. The backend also sends it once if drive updates cease for 200 ms. This is best effort, not a hard real-time stop; the firmware's 300 ms fallback remains. On server shutdown Space is attempted. Space also clears a stall latch and centers steering, exactly as firmware does. An output command of zero does not guarantee an instantaneous mechanical stop.

Only a loopback server is opened. Control requests require a per-run token and local Host header. The local `/state` and `/control` routes belong to this new dashboard, not to the MCU. No configuration-upload protocol is invented.

## Exact firmware interface inspected

Source: `John_Deere_Tractor/` in this repository, inspected November 11, 2026.

- MCU: STM32H533RET6; COM1 = USART2, PA2 TX / PA3 RX for `USE_NUCLEO_H533RE`.
- 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control. The comment mentioning “7-bit data” is misleading; the configured word length is 8B with no parity.
- Input is one raw ASCII character, no newline needed: W/S direction; A/D ±5°; 1…9 = 10…90%; 0 = 100%; Space stops and centers. Both letter cases accepted.
- Every valid character refreshes `t_last_key`. Thus steering or digit traffic can keep an earlier drive command alive. Invalid characters do not refresh it.
- 10 Hz CRLF text output, parsed exactly, including optional `STALL`, `IMU-FAIL` or `IMU-STALE`:

```text
tgt 20 rpm 19 21 cmd 10 10 steer 5 g -12 r 13 rst 0 ov 0
```

`tgt`: ramped central RPM target. `rpm`: measured left/right wheel RPM. `cmd`: left/right requested motor percentages before deadband mapping. `steer`: commanded degrees, positive left. `g`/`r`: relative Game Rotation Vector / Rotation Vector yaw in degrees, each with its own startup zero. `rst`: IMU resets. `ov`: encoder samples missed by the main loop. Numeric telemetry is integer-truncated. `IMU-STALE` applies to the selected heading source, not independent freshness for both reports. These yaw fields are not fused into dashboard odometry.

There are no timestamps, raw cumulative counts, measured steering angle, X/Y position, runtime configuration messages, or left/right distances in this format. When `DISTANCE_ENABLE` is 1, the firmware sends a second line, `dist <front> <right low> <right high>`, in millimetres (-1 = no target, -2 = not valid). The dashboard does not read that line. The UI reserves a labeled distance area; it fabricates no range dots. Adding real dots later requires sensor polling, units/validity in telemetry and measured sensor positions/directions.

## Timeout and geometry

The MCU cannot observe keyboard release: a terminal sends characters, not key-up events. After 300 ms without **any valid key**, `drive_dir` becomes zero. The next 100 Hz wheel updates ramp the target toward zero at 1000 RPM/s: from 200 RPM this takes about another 200 ms, or about 500 ms from the last key, plus loop delays. At the default 20 RPM target the ramp takes about 20 ms. Space calls `motors_stop()` immediately when read, zeroing target/output and resetting PI, without the ramp. The main loop can still be delayed by blocking IMU reads/printing. Physical braking/coasting depends on mechanics and the driver.

For wheelbase L, rear track T, equivalent center steering angle δ:

- Turning radius at rear axle center: R = L / tan(δ).
- Firmware ratio q = T × tan(δ) / (2L).
- Rear targets: left = central RPM × (1−q); right = central RPM × (1+q).
- Positive δ turns left: left is the inner/slower wheel. Reverse reverses heading rate.
- Ideal front Ackermann angles for a left turn: inner = atan(L/(R−Tfront/2)), outer = atan(L/(R+Tfront/2)). Front track is a separate measurement if different from rear track. The drawing assumes equal tracks when dimensions are supplied.

Firmware drives **one servo**, not two independent front-wheel servos. Mechanical linkage determines actual inner/outer angles. Mapping ±30° linearly to servo pulse width does not calibrate actual tire angles. The dashboard's individual front-wheel angles are an illustration, not sensor measurements.

Because L and T are both zero, `rear_wheel_ratio()` currently returns zero: both rear wheels get the same target even when steering. `WHEEL_DIAMETER_MM` is defined but unused in current wheel-speed control; it is needed to convert RPM to ground speed for odometry.

## Constants/configuration to change

All values below are current values in `App/Config/constants.h`; replace TBD values with measurements, not the demo dimensions.

| Configuration | Current | Action |
|---|---|---|
| `WHEEL_DIAMETER_MM`, `TRACK_WIDTH_MM`, `WHEELBASE_MM` | 0 / 0 / 0 | Measure loaded wheel diameter, rear wheel-center track, axle spacing. Required for useful physical geometry. |
| `SERVO_MIN_US`, `SERVO_CENTER_US`, `SERVO_MAX_US` | 1000 / 1500 / 2000 | Calibrate center and mechanical-safe pulse limits. |
| `SERVO_MAX_STEER_DEG`, `SERVO_DIR` | 30°, +1 | Measure actual steering vs pulse and confirm positive means left. |
| `ENC_PULSES_PER_MOTOR_REV`, `ENC_EDGES_COUNTED`, `GEAR_RATIO` | 16 / 4 / 50 | Assumed 3200 counts/wheel revolution; verify with ten turns and confirm `ENC_DIR_LEFT/RIGHT` (both +1). |
| `WHEEL_KP`, `WHEEL_KI` | 0 / 0 | Wheel PI is presently inactive; tune from measurements. |
| `WHEEL_FF_PCT_PER_RPM` | 0.5 | Estimate based on 200 RPM no-load at 12 V; calibrate loaded response and actual supply. |
| `WHEEL_SPEED_MAX_RPM` | 200 | Set a verified loaded limit below no-load speed; update dashboard speed labels if changed. |
| `MOTOR_DEADBAND_PCT`, `MOTOR_DUTY_MAX` | 0 / 100 | Measure minimum useful duty and safe maximum. |
| `WHEEL_ACCEL_RPM_PER_S`, `WHEEL_DECEL_RPM_PER_S` | 200 / 1000 | Tune ramp behavior; release in this dashboard uses Space and bypasses it. |
| `COMMAND_TIMEOUT_MS` | 300 | Keep as link-loss fallback; shortening it does not provide key-up detection. |
| `STALL_TIMEOUT_MS`, `STALL_MIN_DUTY` | 300 ms / 30 | Verify against loaded motor/encoder behavior. Low commands below 30 do not trigger this stall detection. |
| `STEER_STEP_DEG` | 5° | Per A/D byte; dashboard uses one byte per press. |
| `HEADING_HOLD_ENABLE`, `HEADING_SOURCE` | 0 / Game RV | Direct steering now. Keep disabled until yaw/servo signs and heading gains are calibrated; gains are all zero. If enabled, A/D change heading target, so update UI instructions. |
| `WHEEL_CONTROL_PERIOD_MS`, `STEERING_CONTROL_PERIOD_MS`, `TELEMETRY_PERIOD_MS` | 10 / 20 / 100 ms | TIM6 supplies wheel sampling; changing wheel period requires matching `.ioc`/timer setup. |

Encoder migration is already represented in this source: TIM2 PA0/PA1 and TIM3 PA6/PA7, x4, sampled by TIM6. At 3200 counts/rev and 10 ms, one count = 1.875 RPM before integer telemetry truncation. Timer counting avoids per-edge software interrupts but still depends on electrical signal quality/filter configuration. Motor PWM is TIM8 PC6–PC9 at 20 kHz; servo is TIM1 PA8 at 50 Hz. No pin edits or code regeneration were necessary for the dashboard.

For future firmware work, consider a separate stop-only command if release should preserve steering and never clear a stall. That would be a protocol change, so it was not added here. At high turning targets, independent ±100% command clamps can distort the intended wheel-speed ratio; allow headroom when setting maximum speed. Monitor `ov` before replacing blocking telemetry with DMA.

## Verification

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s . -v
```

Tests cover exact parsing and flags, malformed inputs, command sequencing/release, watchdog stop, estimated distance and stale gaps, real POSIX pseudo-terminal fragmented serial reads/writes, and demo stopping. Browser verification covers the responsive display, a 5° steering step, holding drive, rear trails and zero output/center on release. Hardware behavior remains unverified.

## Bare-board test with placeholder dimensions

```sh
python3 dashboard.py --port /dev/cu.usbmodemXXXX --placeholder-geometry
```

Open http://127.0.0.1:8766. This uses explicitly uncalibrated 100/300/220 mm dimensions in the dashboard only. It does not fabricate sensor readings or alter firmware constants. The serial port name may change after reconnecting.

A bare board must still have the current `John_Deere_Tractor` firmware flashed and running. With no IMU the expected flag is `IMU-FAIL`; telemetry should still arrive. Encoder inputs without encoders are not valid wheel-speed measurements. No distance trails should be interpreted as real motion.

To check the receive-and-return path on a bare board: click A once and look for `steer 5`; click Space and look for `steer 0` and zero target/commands. This checks command handling, not a servo. W/S can change target and output commands, but cannot validate a motor or controller without hardware feedback. On a fully wired vehicle those keys can cause movement.

If the dashboard stays `SERIAL · STALE` with no frame, verify the flashed firmware, resume/exit a paused debugger, press the board RESET button, and ensure the USB cable is on the ST-LINK connector. Close any other serial terminal. Missing sensors do not by themselves explain indefinitely silent UART output in the inspected firmware.

## Sim / Live selector and keyboard fixes

Start with both modes available, initially in simulation:

```sh
python3 dashboard.py --demo --port /dev/cu.usbmodemXXXX --placeholder-geometry
```

Or run `python3 dashboard.py` for simulation only. The existing `--port` command still starts in Live. Use the Sim / Live buttons at the top to switch. Sim never opens/writes the serial port. Leaving Live attempts Space, releases the device, and resets the display. Live opens the configured port; connection errors appear in the dashboard and clicking Live retries. Mode changes reset speed to 10%, inputs and pose. Requests from the old mode are rejected.

A small blinking green **LIVE** dot appears only when valid serial frames are less than 500 ms old and there is no serial error. It becomes **NO DATA** when stale, or **OFFLINE** when the browser loses the server. Sim is always labeled **SIM**, with simulated telemetry.

WASD, Space and digits now work even when the speed dropdown is focused. Digits 1–9 select 10–90%; only 0 selects 100%. Space sends a one-time stop and releases its visual highlight on key-up. It no longer stays latched and blocks later speed updates. Held keys show a highlight. A/D remain 5° per press, matching the existing firmware protocol; W/S are held to drive.

After updating, stop an older server with Ctrl+C, start the new one, and reload the browser. Do not run two instances against the same serial device.
