# Palletrone 4 kg / Pixhawk 6X / DShot 변경 안내

## 적용 대상

- Flight controller: Pixhawk 6X
- Build target: `px4_fmu-v6x_default`
- Frame: 기존과 동일한 X형 4모터 틸트 구조
- 기체 질량: **4.0 kg**
- CoG–모터축 arm 길이: **0.181 m**
- 모터: **T-Motor F60 PRO V-LV 2020KV**
- 전원/초기 모델 기준: **6S, 25.2 V**
- 예정 프로펠러: **5540 3엽**
- 모터 사용 상한: **최대 추력/명령의 80%**

## 변경된 파일

| 경로 | 변경 내용 |
|---|---|
| `src/lib/palletrone/PalletroneConfig.hpp` | 질량, arm, 관성, F60 추력표, 80% 제한을 한곳에서 관리하는 신규 헤더 |
| `src/modules/mc_pos_control/PositionControl/PositionControl.cpp` | 제어기 질량을 공통 4.0 kg 값으로 연결 |
| `src/modules/control_allocator/ControlAllocator.cpp` | arm 0.181 m, 모터당 힘 제한, DShot 변환 및 최종 80% 제한 |
| `src/modules/control_allocator/ControlAllocator.hpp` | 실제 `DSHOT_MIN` 파라미터를 allocator가 읽도록 추가 |
| `src/modules/control_allocator/ControlAllocationUtils.cpp` | 구형 PWM 역모델을 F60 force-to-DShot lookup 변환으로 교체 |
| `src/modules/control_allocator/ControlAllocationUtils.hpp` | 새 DShot 변환 함수 선언 |
| `src/modules/mc_rate_control/torque_disturbance_observer.cpp` | nominal inertia를 기존 값의 절반으로 변경 |
| `src/modules/mc_rate_control/dob_based_com_estimator.cpp` | CoM estimator inertia를 기존 값의 절반으로 변경 |
| `src/modules/mc_rate_control/l1_adaptive_controller.cpp` | L1 nominal inertia를 기존 값의 절반으로 변경 |
| `src/modules/mc_rate_control/palletrone_params.yaml` | custom attitude D 기본값 0, 일회성 migration marker 추가 |
| `boards/px4/fmu-v6x/init/rc.board_defaults` | AUX 1–4 DShot600, gain/AUX 설정 일회성 migration |
| `BUILD_PIXHAWK6X.sh` | clean build 및 결과 파일 검증 스크립트 |

## 기체 물리 모델

```cpp
// 이전 대형 기체: vehicle_mass = 8.0f kg
// 변경 미니 기체: mini_vehicle_mass = 4.0f kg
kVehicleMassKg = 4.0f;

// 이전 대형 기체: arm_length = 0.230f m
// 변경 미니 기체: mini_arm_length = 0.181f m
kArmLengthM = 0.181f;
```

관성은 요청대로 기존 값의 정확히 절반이다.

```text
old: [0.07680, 0.08710, 0.11300] kg·m²
new: [0.03840, 0.04355, 0.05650] kg·m²
```

## BLDC 추력 모델과 80% 제한

제공된 F60 PRO V-LV 자료 중 T5147-3, 25.2 V 데이터를 초기 proxy로 사용했다.

| ESC throttle | 추력/모터 |
|---:|---:|
| 0% | 0.0000 N |
| 20% | 2.7674 N |
| 40% | 6.8303 N |
| 60% | 10.5529 N |
| 80% | 16.3702 N |
| 100% | 20.4322 N |

안전 제한은 두 단계다.

1. allocator 요구 힘: `0.80 × 20.4322 = 16.3457 N/모터` 이하
2. 실제 DShot throttle: `0.80` 이하

`DSHOT_MIN=0.055`일 때 PX4의 `actuator_motors.control` 상한은 단순 0.8이 아니라 다음과 같다.

```text
(0.80 - 0.055) / (1.0 - 0.055) = 0.7883598
```

이 변환은 코드에서 현재 `DSHOT_MIN` 파라미터를 읽어 계산하므로, 추후 `DSHOT_MIN`을 바꾸더라도 실제 DShot 80% 상한은 유지된다.

> **주의:** 제공 자료에는 예정된 5540 3엽의 정확한 수치표가 없다. 현재 모델은 초기 비행용 proxy이며, 실제 5540 3엽·ESC·6S 조합을 추력 스탠드에서 측정한 뒤 `PalletroneConfig.hpp`의 표를 교체해야 한다.

4 kg 기체의 정지 호버 요구량은 약 `9.81 N/모터`이고, 현재 proxy에서 약 56% throttle이다. 80% 제한 시 총 정적 추력은 약 `65.38 N`, 이상적인 정적 추력비는 약 1.67이다. 배터리 전압 강하와 실제 5540 모델 오차는 별도 고려해야 한다.

## DShot 출력 설정

다음 기본값과 일회성 실제값 적용이 포함된다.

```text
PWM_AUX_TIM0  = -3    # DShot600
PWM_AUX_FUNC1 = 101   # Motor 1
PWM_AUX_FUNC2 = 102   # Motor 2
PWM_AUX_FUNC3 = 103   # Motor 3
PWM_AUX_FUNC4 = 104   # Motor 4
DSHOT_MIN     = 0.055
DSHOT_3D_ENABLE = 0
THR_MDL_FAC   = 0.0
```

**MAIN/PX4IO 파라미터는 수정하거나 비활성화하지 않았다.** ESC 신호선은 AUX/FM​​U 1–4에 연결한다. MAIN에 장치가 연결되지 않았다면 MAIN의 기존 신호는 실제 장치를 구동하지 않는다.

## gain 초기화

기존 Pixhawk의 parameter 저장소에 구형 대형 기체 gain이 남을 수 있으므로 `ZZZ_PAL_CFG_VER=20260805`를 사용한다. 첫 부팅 한 번만 다음 gain을 현재 소스 트리의 기본값으로 reset한다.

- `MC_*RATE_*` rate gain
- `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P` 및 custom attitude D
- `MPC_*` position/velocity gain

센서, 보드, 가속도계, 자이로, 나침반, RC 등의 보정 파라미터는 전체 reset하지 않는다. AUX DShot 설정만 실제값으로 한 번 적용한다.

## 서보 확인 결과

PX4 소스에서 XL430-250T 또는 XC330-M288-T에 종속된 Dynamixel control-table, model number, baud, position-unit 설정은 발견되지 않았다. 따라서 펌웨어의 servo 처리에는 불필요한 변경을 하지 않았다. 해당 모델별 설정은 ROS 2/Dynamixel SDK 노드에서 관리해야 한다.

## 빌드

정식 GNU Arm Embedded toolchain과 PX4 빌드 의존성이 설치된 Ubuntu 환경에서 다음을 실행한다.

```bash
cd PX4-Optimized
./BUILD_PIXHAWK6X.sh
```

직접 명령을 사용할 수도 있다.

```bash
make distclean
make -j"$(nproc)" px4_fmu-v6x_default
```

생성 파일:

```text
build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4
```

업로드:

```bash
make px4_fmu-v6x_default upload
```

또는 QGroundControl의 Firmware 메뉴에서 생성된 `.px4`를 선택한다.

## 업로드 후 필수 확인

프로펠러를 제거하고 다음을 먼저 확인한다.

```text
param show ZZZ_PAL_CFG_VER       # 20260805
param show PWM_AUX_TIM0          # -3
param show PWM_AUX_FUNC1         # 101
param show PWM_AUX_FUNC2         # 102
param show PWM_AUX_FUNC3         # 103
param show PWM_AUX_FUNC4         # 104
param show DSHOT_MIN             # 0.055
param show THR_MDL_FAC           # 0.0
```

그다음 AUX 1–4의 모터 번호, 회전 방향, ESC DShot 인식, 각 servo 방향을 확인한다. 첫 비행은 DOB/CoM/L1 부가기능을 비활성화할 수 있는 상태에서 낮은 고도 또는 계류 조건으로 진행하고 rate → attitude → velocity → position 순서로 재튜닝한다.
