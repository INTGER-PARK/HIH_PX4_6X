# Validation report

## Passed checks

- `git diff --check`: no whitespace or malformed patch errors.
- `palletrone_params.yaml`: parsed successfully with PyYAML.
- Active custom allocator references `force_to_dshot_control`; no active call to the old `force_to_pwm_scale` remains.
- Active mass value is shared from `PalletroneConfig.hpp` as 4.0 kg.
- DOB, CoM estimator and L1 controller use the shared half-inertia values.
- MAIN/PX4IO output parameters are not set or reset by the new board-default block.
- AUX/FM​​U 1–4 are assigned Motor 1–4 with DShot600.
- No XL430/XC330/Dynamixel model-specific control-table setting exists in the custom PX4 modules, so no servo-model change was made.
- The DShot conversion translation unit compiled successfully as an ARM EABI5 object with Clang during source validation.

## Final firmware build status in this execution environment

A new uploadable `.px4` is **not included**. The container does not contain the standard GNU Arm Embedded runtime/toolchain required for a trustworthy full PX4 link, and its current Python environment is also missing PX4's `empy` dependency. The old `.px4` that was present in the uploaded build cache predates these changes and was deliberately excluded from all deliverables.

Use `./BUILD_PIXHAWK6X.sh` in a normal PX4 development environment. The script deletes the old build cache first and only reports success when a new non-empty `px4_fmu-v6x_default.px4` is created.
