# Palletrone second-order MOB implementation report

## Repository audit

- `HIH_PX4`: branch `main`, commit `54f2f992` (`parameter added`), no matching version tag (`PX4_GIT_TAG` falls back to `v0.0.0`). Actual target is `px4_fmu-v6x_default`; its board config enables control allocator and uXRCE-DDS.
- `PX4_Rasp_Pi`: branch `main`, commit `3f5ff03`; ROS 2 Humble ament/colcon workspace. Pre-existing user state includes `quadrotor_controller/config/xc330_m288t.yaml` modification and tracked `build/`/`install/` deletion. It was preserved.
- Servo flow: `quadrotor_controller/src/dynamixel_fix.cpp` publishes actual radians on `servo_angle`; `listen_and_speak_ros.cpp` copies IDs 1–4 into `px4_msgs/ServoAngle` and publishes `/fmu/in/servo_angle`; `dds_topics.yaml` converts this to `servo_angle` uORB; `ControlAllocator.cpp` consumes it.
- Actuator flow: active allocator computes and clamps `_actuator_sp[0..3]` in N, publishes the same values as `thrust_command`, converts them through the identified lookup table to normalized `actuator_motors.control`, then DShot maps control zero to `DSHOT_MIN`. MOB uses `thrust_command`, not normalized vehicle/actuator setpoints.
- Estimator flow: EKF2 publishes `vehicle_odometry` with quaternion, pose/velocity frame, fused velocity, angular velocity, variance, reset counter and sample timestamp. H-Flow contributes through EKF; raw optical flow is not treated as force.
- Output flow: `palletrone_mob_status` uORB -> uXRCE-DDS `/fmu/out/palletrone_mob_status` -> byte-compatible `PX4_Rasp_Pi/px4_msgs` -> rosbag directly.

### Active dynamics/geometry

| Item | Active value/source |
|---|---|
| mass | 4.0 kg, `PalletroneConfig.hpp` / default `PMOB_MASS` |
| inertia | diag(0.0192, 0.02125, 0.0282) kg·m²; products default zero |
| origin/CoM | allocator modeled CoM/body origin; active allocator forces CoM update x/y/z to zero |
| arm | 0.181 m CoM-to-motor axis, X configuration |
| vertical offset | rotor plane z = -0.005 m in FRD |
| reaction ratio | 0.01 m, signs motor 1/3 positive and 2/4 negative in the implemented `sigma*kappa*T*d` convention |
| thrust | `thrust_command[i]`, already-clamped physical N; motor model table is T5147-3 proxy for planned 5540-3 |
| DShot | N -> lookup throttle -> PX4 control; throttle capped at 0.80 and runtime `DSHOT_MIN` accounted for |

| Motor | Physical position (FRD) | PX4 index | Spin | Servo ID | positive tilt horizontal direction | zero thrust |
|---|---|---:|---|---:|---|---|
| 1 | upper-left in allocator comment | 0 | CW | 1 | +x,+y | -z |
| 2 | lower-left in allocator comment | 1 | CCW | 2 | +x,-y | -z |
| 3 | lower-right in allocator comment | 2 | CW | 3 | -x,-y | -z |
| 4 | upper-right in allocator comment | 3 | CCW | 4 | -x,+y | -z |

These signs were derived from the currently compiled custom allocator matrix, not generic PX4 rotor geometry. The source comment supplies the physical upper/lower-left/right labels but does not define how that sketch maps to FRD x/y, and the custom moment rows cannot be represented by one conventional four-position X geometry for all axes. Therefore no unverified FRD position coordinates are claimed. Shared arm/offset/tilt/reaction constants were placed in `PalletroneConfig.hpp`; allocator behavior itself was not changed.

## Observer

The estimate is at the modeled CoM/body origin in body FRD, positive when the wrench acts on the vehicle. With `p=[m v_B; J omega_B]`:

```text
b_linear  = f_act + m R_LB^T [0,0,g] - m (omega x v_B)
b_angular = tau_act - omega x (J omega)
p_hat_dot = b + w_hat + L1 (p-p_hat)
w_hat_dot = L2 (p-p_hat)
L1 = 2 zeta omega_n, L2 = omega_n^2
```

Local NED/FRD velocity is rotated with PX4 `Quatf`/`Dcmf`; BODY_FRD velocity is used directly. Unknown frames invalidate force. Each axis uses an exact algebraic Tustin update of the two observer states for variable `timestamp_sample` dt; forward Euler and acceleration measurements are not used. Force and torque have independent `wn/zeta` and validity.

Reset occurs on enable transition/input recovery, estimator reset-counter change, reversed/invalid timestamp, invalid dt (accepted range 0.5–50 ms), invalid parameters/non-finite inputs, stale odometry/thrust/servo, or arm gate loss/recovery. Reset sets `p_hat=p`, `w_hat=0`; validity remains false for `PMOB_WARMUP`. The inertia matrix is checked positive definite.

Rotor force uses actual servo angle and final commanded thrust. Roll/pitch torque follows `r_i x f_i + sigma_i kappa T_i d_i`. To remain identical to the active flight model, yaw follows the allocator's current custom yaw row: at its forced zero CoM offset it retains reaction torque but omits the yaw component of tilted-force `r_i x f_i`. A randomized numerical equivalence test is used to guard all three torque rows. This omission is an existing allocator/model limitation, not a MOB correction.

```text
f_i = T_i d_i
tau_x/y = [r_i x f_i + sigma_i kappa T_i d_i]_x/y
tau_z = -sigma_i kappa T_i cos(theta_i)   # active allocator at CoM offset zero
```

No MOB output is subscribed to or injected into any controller, allocator, setpoint, actuator, DOB or L1 path.

## Files changed

- `HIH_PX4/msg/CMakeLists.txt`, `msg/PalletroneMobStatus.msg`: new typed uORB status.
- `HIH_PX4/src/modules/palletrone_mob/*`: pure observer, PX4 wrapper, parameters, Kconfig and deterministic reference tests.
- `HIH_PX4/src/lib/palletrone/PalletroneConfig.hpp`: shared active rotor geometry/sign constants.
- `HIH_PX4/boards/px4/fmu-v6x/default.px4board`: compile module into Pixhawk 6X firmware; runtime default remains disabled.
- `HIH_PX4/src/modules/uxrce_dds_client/dds_topics.yaml`: DDS publication.
- `PX4_Rasp_Pi/px4_msgs/msg/PalletroneMobStatus.msg`: byte-compatible ROS interface.
- both flight launch files and `cf_bag/src/bag.cpp`: record the new topic beside the unchanged 92-field legacy topic.
- `PX4_Rasp_Pi/tools/plot_palletrone_mob.py`, `ROSBAG_README.md`: plotting and logging documentation.

## Topic and parameters

Topic: `/fmu/out/palletrone_mob_status`, type `px4_msgs/msg/PalletroneMobStatus`. It contains timestamps, BODY_FRD frame enum, independent validity, initialization/warm-up, status/reset flags, external and known actuator force/torque, momentum errors, dt and three input ages. Units are documented in both message definitions.

Parameters: `PMOB_EN` (false), mass, six symmetric inertia terms, force/torque `wn` and damping, odometry/actuator/servo timeout, warm-up, and `PMOB_ARM_ONLY` (true). No automatic tare, bias, learned residual, drag or ground-effect compensation exists.

## Build and test results

- Baseline `make px4_fmu-v6x_default`: failed before compilation because this repository vendors dependency files while retaining `.gitmodules`, so nested `.git` metadata was absent.
- The vendored-dependency build path was repaired centrally. Missing build inputs were restored from matching releases: pymavlink 2.4.49 generator headers, DroneCAN 1.0.16 DSDL parser, Micro-CDR 2.0.1 and the matching Micro-XRCE-DDS-Client 2.4.0 log sources. Vendored NuttX/MAVLink use explicit `unknown` version metadata fallbacks.
- Post-change full `make px4_fmu-v6x_default`: success. It generated `build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4` (about 1.7 MiB); final FLASH usage was 1,795,732 bytes of 1,920 KiB (91.34%). The MOB uORB message, DDS bridge, MOB module, XRCE-DDS client and final ELF all compiled and linked.
- Isolated real target compile: CMake configured `px4_fmu-v6x_default`, generated uORB and parameters, and ARM Cortex-M7 compiled `PalletroneMob.cpp` with PX4 `-Werror`: success.
- `python3 src/modules/palletrone_mob/test/test_palletrone_mob.py`: 9 tests passed (hover, free fall, +Fx/+Fz, +Tx/+Ty/+Tz, gyro/Coriolis signs, variable dt, reset, stale input).
- ROS `px4_msgs` and `cf_bag`: success. New interface is discoverable with `ros2 interface show`.
- ROS dependencies plus `px4_ros_com`: success (four packages); only CMake Python-policy development warnings.
- launch and plot `py_compile`: success. Legacy `raw_data` source remains 92 entries and was not modified.

## Runtime procedure

Firmware build (all required vendored dependencies are now included):

```bash
cd ~/HIH_PX4
make px4_fmu-v6x_default
```

No flashing is part of this work. PX4 NSH diagnostics:

```text
param show PMOB_*
palletrone_mob start
param set PMOB_EN 1
palletrone_mob status
listener palletrone_mob_status 10
listener vehicle_odometry
listener vehicle_local_position
listener vehicle_angular_velocity
listener sensor_optical_flow
listener thrust_command
listener servo_angle
```

ROS and logging:

```bash
ros2 topic list | grep -i mob
ros2 topic info /fmu/out/palletrone_mob_status -v
ros2 topic echo /fmu/out/palletrone_mob_status --once
ros2 topic hz /fmu/out/palletrone_mob_status
ros2 bag record /data_logging_msg /fmu/out/palletrone_mob_status
python3 tools/plot_palletrone_mob.py <bag-directory>
```

All three CM4 recording entry points (`palletrone_cm4.launch.py`, `v3_flight_launcher.launch.py`, and `cf_bag`) record both topics in the same bag. The legacy `/data_logging_msg` remains exactly 92 fields; MOB data is never appended to that array.

## Hardware-safe initial validation

Use FC USB power and CM4/DDS only, with propellers removed and no motor command. Confirm actual servo freshness, zero motor thrust source, status/ages/reset/dt and invalid gating. For bench observation only set `PMOB_ARM_ONLY=0`; table support force may correctly appear as residual and must not be auto-zeroed. Apply small hand forces in +FRD x/y/z and small roll/pitch/yaw moments to check signs. Disable `PMOB_EN` and verify existing control behavior is unchanged. Flight validation requires a separate safety procedure.

## Limitations

This is not ground-truth contact force. It is a model-based estimated external/residual wrench: mass/inertia/CoM error, the active allocator's omitted tilt-induced `r×f` yaw term, proxy thrust calibration, DShot-to-thrust and motor/propeller lag, missing RPM/current feedback, servo delay/clock mismatch, drag, rotor interaction, ground effect and H-Flow/EKF velocity quality all enter the residual. Contact location cannot be uniquely recovered from a CoM wrench. No compensation is implemented or connected.
