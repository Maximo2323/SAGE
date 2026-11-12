# Vehicle and project objectives

Build an STM32-controlled vehicle to complete an approximately 50–70 m track, passing targets/checkpoints with a desired error below roughly 1 cm. The user reports that the roughly 1 cm performance goal is verified. Preserve this as user-reported verification; the test conditions and measurement reference are not recorded here.

## Vehicle layout (user description)
- Two independently driven wheels/motors, without suspension.
- Front Ackermann steering, believed to be true Ackermann, actuated by one servo; verify the linkage geometry.
- Coordinate left/right drive speeds with steering angle. Do not assume a pure differential-drive model with a fixed center of rotation: the turning center depends on steering geometry.
- Intended BNO085 mounting: centered between the wheels and away from the motors. The user intends this as the vehicle reference/rotation-center location; confirm which axle and measure the position. That midpoint is not generally the instantaneous turning center during Ackermann steering.

## Keep in mind during implementation
- Measure wheelbase, track width, effective wheel radii, encoder counts per wheel revolution, servo-angle mapping and sensor offsets.
- Steering-servo support: `John_Deere_Tractor/App/Actuators/servo.c` (pulse range and steering limits are placeholders in `constants.h`).
- Heading hold alone does not establish centimeter-level position accuracy. Define checkpoint sensing/localization and correction strategy; validate accumulated error over the full course, including slip, steering backlash and sensor calibration.
- Confirm course layout, checkpoint definition, permitted external references and speed requirements before choosing navigation/control methods.

This is a compact statement of user intent and user-reported performance verification; it does not independently validate the vehicle model.

## Future localization: repeatable sensor landmarks
Planned for later, not implemented now: place or identify track markers/features whose sensor readings or short reading patterns repeat at known checkpoints. During a run, look for the expected landmark using the designated sensor and use a confirmed match to correct accumulated localization error.

A matching reading is evidence of a checkpoint, not proof of an exact position: the same distance can occur elsewhere and depends on vehicle angle, sensor offset, noise and target geometry. Associate each landmark with its sensor, expected reading/pattern, tolerance, approach direction and expected route segment/order. Require a stable match within a plausible location window before correcting only the position/heading components that the landmark actually constrains. Handle missed or ambiguous matches without blindly resetting position.

First record repeated passes to measure repeatability and false matches; choose marker geometry and tolerances from those measurements. Keep encoder/heading navigation between landmarks. The user reports the roughly 1 cm performance goal as verified. Record the supporting test conditions and reference point when available; recognition of a reading alone does not establish that accuracy for every future run.
