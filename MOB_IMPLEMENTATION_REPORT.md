# Palletrone 2차 MOB 구현 보고서

## 저장소 감사 결과

- `HIH_PX4`: 브랜치 `main`, 커밋 `54f2f992` (`parameter added`). 일치하는 version tag가 없어 `PX4_GIT_TAG`는 `v0.0.0`으로 대체된다. 실제 빌드 target은 `px4_fmu-v6x_default`이며, board 설정에서 control allocator와 uXRCE-DDS를 활성화한다.
- `PX4_Rasp_Pi`: 브랜치 `main`, 커밋 `3f5ff03`. ROS 2 Humble ament/colcon workspace이다. 작업 전부터 존재하던 `quadrotor_controller/config/xc330_m288t.yaml` 수정과 추적 중인 `build/`·`install/` 삭제 상태는 그대로 보존했다.
- Servo 흐름: `quadrotor_controller/src/dynamixel_fix.cpp`가 실제 angle을 rad 단위로 `servo_angle`에 publish한다. `listen_and_speak_ros.cpp`가 ID 1~4의 값을 `px4_msgs/ServoAngle`에 복사하여 `/fmu/in/servo_angle`로 publish하고, `dds_topics.yaml`이 이를 `servo_angle` uORB로 변환하며, `ControlAllocator.cpp`가 이 값을 사용한다.
- Actuator 흐름: 활성 allocator가 N 단위 `_actuator_sp[0..3]`를 계산하고 제한한 다음, 동일한 값을 `thrust_command`로 publish한다. 이후 식별된 lookup table을 통해 normalized `actuator_motors.control`로 변환하고, DShot은 control zero를 `DSHOT_MIN`에 대응시킨다. MOB는 normalized vehicle/actuator setpoint가 아니라 `thrust_command`를 사용한다.
- Estimator 흐름: EKF2는 quaternion, pose/velocity frame, fused velocity, angular velocity, variance, reset counter 및 sample timestamp가 포함된 `vehicle_odometry`를 publish한다. H-Flow는 EKF 융합을 통해 기여하며 raw optical flow를 force로 직접 취급하지 않는다.
- 출력 흐름: `palletrone_mob_status` uORB → uXRCE-DDS `/fmu/out/palletrone_mob_status` → byte 호환 `PX4_Rasp_Pi/px4_msgs` → rosbag 직접 기록.

### 현재 적용된 동역학 및 기체 형상

| 항목 | 현재 값/출처 |
|---|---|
| 질량 | 4.0 kg, `PalletroneConfig.hpp` / 기본 `PMOB_MASS` |
| 관성 | diag(0.0192, 0.02125, 0.0282) kg·m², 관성곱 기본값은 0 |
| 원점/CoM | allocator가 모델링한 CoM/body origin, 활성 allocator는 CoM update x/y/z를 0으로 강제 |
| Arm | CoM에서 motor 축까지 0.181 m, X 형상 |
| 수직 offset | FRD에서 rotor plane z = -0.005 m |
| Reaction torque 비율 | 0.01 m, 구현된 `sigma*kappa*T*d` 규약에서 motor 1/3은 양수, 2/4는 음수 |
| 추력 | `thrust_command[i]`, 이미 제한된 물리 추력 N. Motor model table은 예정된 5540-3 대신 T5147-3 데이터를 초기 proxy로 사용 |
| DShot | N → lookup throttle → PX4 control. Throttle은 0.80으로 제한하며 runtime `DSHOT_MIN`을 반영 |

| Motor | 물리 위치(FRD) | PX4 index | 회전 방향 | Servo ID | 양의 tilt 수평 방향 | Zero thrust 방향 |
|---|---|---:|---|---:|---|---|
| 1 | allocator 주석의 upper-left: +x,-y | 0 | CW | 1 | +x,-y | -z |
| 2 | allocator 주석의 lower-left: -x,-y | 1 | CCW | 2 | -x,-y | -z |
| 3 | allocator 주석의 lower-right: -x,+y | 2 | CW | 3 | -x,+y | -z |
| 4 | allocator 주석의 upper-right: +x,+y | 3 | CCW | 4 | +x,+y | -z |

이 FRD x/y 부호는 기체 소유자가 확인한 값이며, 이전 allocator 주석 해석을 대체한다. 위치, arm, offset, tilt 및 reaction 상수는 `PalletroneConfig.hpp`에 공통으로 정의했다. 기존 allocator/control 경로와 motor command는 MOB에 의해 변경되지 않는다.

## Observer

추정 결과는 modeled CoM/body origin 기준 body FRD로 표현하며, wrench가 기체에 작용하는 방향을 양수로 정의한다. `p=[m v_B; J omega_B]`일 때 수식은 다음과 같다.

```text
b_linear  = f_act + m R_LB^T [0,0,g] - m (omega x v_B)
b_angular = tau_act - omega x (J omega)
p_hat_dot = b + w_hat + L1 (p-p_hat)
w_hat_dot = L2 (p-p_hat)
L1 = 2 zeta omega_n, L2 = omega_n^2
```

Local NED/FRD velocity는 PX4 `Quatf`/`Dcmf`로 회전 변환하고, BODY_FRD velocity는 그대로 사용한다. 알 수 없는 frame이면 force를 invalid 처리한다. 각 축은 가변 `timestamp_sample` dt에 대해 두 observer state의 정확한 대수적 Tustin update를 사용한다. Forward Euler와 acceleration measurement는 사용하지 않는다. Force와 torque는 독립적인 `wn/zeta` 및 validity를 가진다.

Enable 전환 또는 입력 복구, estimator reset counter 변경, 역행하거나 잘못된 timestamp, 잘못된 dt(허용 범위 0.5~50 ms), 잘못된 parameter 또는 non-finite 입력, 오래된 odometry/thrust/servo, arm gate 상실 또는 복구 시 observer를 reset한다. Reset은 `p_hat=p`, `w_hat=0`으로 설정하며 `PMOB_WARMUP` 동안 validity는 false로 유지된다. Inertia matrix는 positive definite인지 검사한다.

Rotor force 계산에는 actual servo angle과 최종 command thrust를 사용한다. 세 torque 축 모두 기체 소유자가 확인한 물리 rotor 위치를 사용하며 body FRD에서 `r_i x f_i + sigma_i kappa T_i d_i`를 따른다.

```text
f_i = T_i d_i
tau_i = r_i x f_i + sigma_i kappa T_i d_i
```

MOB 출력은 controller, allocator, setpoint, actuator, DOB 또는 L1 경로에서 subscribe하거나 주입하지 않는다.

## 변경 파일

- `HIH_PX4/msg/CMakeLists.txt`, `msg/PalletroneMobStatus.msg`: 새로운 typed uORB status 추가.
- `HIH_PX4/src/modules/palletrone_mob/*`: 순수 observer, PX4 wrapper, parameter, Kconfig 및 deterministic reference test.
- `HIH_PX4/src/lib/palletrone/PalletroneConfig.hpp`: 현재 사용하는 rotor geometry/sign 공통 상수.
- `HIH_PX4/boards/px4/fmu-v6x/default.px4board`: module을 Pixhawk 6X firmware에 포함. Runtime 기본값은 disabled 상태를 유지.
- `HIH_PX4/src/modules/uxrce_dds_client/dds_topics.yaml`: DDS publication 추가.
- `PX4_Rasp_Pi/px4_msgs/msg/PalletroneMobStatus.msg`: byte 호환 ROS interface.
- 두 flight launch 파일과 `cf_bag/src/bag.cpp`: 기존 92-field legacy topic을 변경하지 않고 새 topic을 함께 기록.
- `PX4_Rasp_Pi/tools/plot_palletrone_mob.py`, `ROSBAG_README.md`: plotting 도구 및 logging 문서.

## Topic 및 parameter

Topic은 `/fmu/out/palletrone_mob_status`, type은 `px4_msgs/msg/PalletroneMobStatus`이다. Timestamp, BODY_FRD frame enum, force/torque 독립 validity, initialization/warm-up 상태, status/reset flag, external 및 known actuator force/torque, momentum error, dt와 세 입력의 age를 포함한다. 단위는 PX4와 ROS 양쪽 message 정의에 명시했다.

Parameter는 `PMOB_EN`(false), 질량, 대칭 inertia matrix의 6개 항, force/torque `wn`과 damping ratio, odometry/actuator/servo timeout, warm-up 및 `PMOB_ARM_ONLY`(true)를 포함한다. Automatic tare, bias subtraction, learned residual, drag 또는 ground-effect compensation은 구현하지 않았다.

## Build 및 test 결과

- Baseline `make px4_fmu-v6x_default`: 이 저장소가 `.gitmodules`를 유지하면서 dependency 파일을 vendoring하여 nested `.git` metadata가 없었기 때문에 compile 전에 실패했다.
- Vendored dependency build 경로를 공통으로 수정했다. 일치하는 release에서 누락된 build 입력인 pymavlink 2.4.49 generator header, DroneCAN 1.0.16 DSDL parser, Micro-CDR 2.0.1 및 이에 맞는 Micro-XRCE-DDS-Client 2.4.0 log source를 복원했다. Vendored NuttX/MAVLink는 명시적인 `unknown` version metadata fallback을 사용한다.
- 수정 후 전체 `make px4_fmu-v6x_default`: 성공. `build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`(약 1.7 MiB)를 생성했다. 최종 FLASH 사용량은 1,795,596 bytes / 1,920 KiB(91.33%)이다. MOB uORB message, DDS bridge, MOB module, XRCE-DDS client 및 최종 ELF가 모두 compile 및 link됐다.
- 실제 target 단독 compile: CMake가 `px4_fmu-v6x_default`를 구성하고 uORB와 parameter를 생성했으며, ARM Cortex-M7 compiler가 PX4 `-Werror` 조건으로 `PalletroneMob.cpp`를 compile했다. 결과는 성공이다.
- `python3 src/modules/palletrone_mob/test/test_palletrone_mob.py`: 10개 test 통과. Hover, free fall, +Fx/+Fz, +Tx/+Ty/+Tz, gyro/Coriolis 부호, 가변 dt, reset, stale input 및 확인된 FRD rotor geometry를 검증한다.
- ROS `px4_msgs`와 `cf_bag`: 성공. `ros2 interface show`로 새 interface를 확인할 수 있다.
- ROS dependency와 `px4_ros_com`: 4개 package build 성공. CMake Python-policy 개발 경고만 발생했다.
- Launch와 plot script `py_compile`: 성공. Legacy `raw_data` source는 92개 항목을 그대로 유지하며 수정하지 않았다.

## 실행 절차

필요한 vendored dependency가 모두 포함된 firmware build 명령은 다음과 같다.

```bash
cd ~/HIH_PX4
make px4_fmu-v6x_default
```

이 작업에서는 flashing을 수행하지 않았다. PX4 NSH 진단 명령은 다음과 같다.

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

ROS 및 logging 명령은 다음과 같다.

```bash
ros2 topic list | grep -i mob
ros2 topic info /fmu/out/palletrone_mob_status -v
ros2 topic echo /fmu/out/palletrone_mob_status --once
ros2 topic hz /fmu/out/palletrone_mob_status
ros2 bag record /data_logging_msg /fmu/out/palletrone_mob_status
python3 tools/plot_palletrone_mob.py <bag-directory>
```

CM4의 세 recording 진입점(`palletrone_cm4.launch.py`, `v3_flight_launcher.launch.py`, `cf_bag`)은 모두 동일한 bag에 두 topic을 함께 기록한다. Legacy `/data_logging_msg`는 정확히 92개 field를 유지하며 MOB data를 이 배열에 추가하지 않는다.

## Hardware-safe 초기 검증 절차

Propeller를 제거하고 motor command가 없는 상태에서 FC USB 전원과 CM4/DDS만 사용한다. Actual servo freshness, motor thrust source가 0인지, status/age/reset/dt 및 invalid gating을 확인한다. Bench 관찰에 한해서만 `PMOB_ARM_ONLY=0`으로 설정한다. 이때 table support force가 residual로 나타나는 것은 정상일 수 있으며 자동으로 zeroing하면 안 된다. +FRD x/y/z 방향으로 작은 손 힘을 가하고 작은 roll/pitch/yaw moment를 가하여 부호를 확인한다. `PMOB_EN`을 disable한 후 기존 control behavior가 완전히 동일한지 확인한다. 비행 검증에는 별도의 안전 절차가 필요하다.

## 한계

이 결과는 ground-truth contact force가 아니다. Mass/inertia/CoM 오차, proxy thrust calibration, DShot-to-thrust 및 motor/propeller lag, RPM/current feedback 부재, servo delay/clock mismatch, aerodynamic drag, rotor interaction, ground effect와 H-Flow/EKF velocity 품질이 모두 residual에 포함되는 model-based estimated external/residual wrench이다. Contact 위치를 알 수 없으면 CoM wrench만으로 접촉점을 유일하게 복원할 수 없다. Compensation은 구현하거나 control 경로에 연결하지 않았다.
