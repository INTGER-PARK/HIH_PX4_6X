# Palletrone Momentum Observer (MOB)

`palletrone_mob`는 Palletrone에 작용하는 외력과 외부 토크를 추정하는 PX4용 6축 2차 운동량 관측기다. 이 모듈은 추정 결과만 발행하며 비행 제어기나 actuator 명령을 직접 변경하지 않는다.

## 좌표계와 부호

모든 힘, 토크 및 운동량 벡터는 PX4 `BODY_FRD` 좌표계를 사용한다.

- `+X`: 기체 전방
- `+Y`: 기체 우측
- `+Z`: 기체 하방
- 양의 외력/토크: 기체에 작용하는 방향
- 기준점: 모델에 정의된 기체 무게중심(CoM)/body origin

로터 위치 순서는 `(전방 우측, 후방 우측, 후방 좌측, 전방 좌측)`이며, 틸트 방향은 각 로터 위치에 대한 접선 방향 `(-y, x)`으로 정의한다.

## 관측기 모델

선형 및 각운동량은 다음과 같이 계산한다.

```text
p = m v_body
h = J omega
```

각 축의 2차 관측기는 다음 연속시간 모델을 Tustin 방식으로 이산화해 갱신한다.

```text
p_hat_dot = b + w_hat + 2 zeta wn (p - p_hat)
w_hat_dot = wn^2 (p - p_hat)
```

`w_hat`은 추정 외력 또는 외부 토크다. 수치 미분은 사용하지 않는다. 힘과 토크의 각 축은 서로 독립적인 `wn`과 `zeta`를 갖는다.

## 입력과 출력

주요 입력은 다음과 같다.

- `vehicle_odometry`: 속도, 자세, 각속도 및 sample timestamp
- `thrust_command`: 각 로터의 실제 힘 명령
- `servo_angle`: 각 틸트 서보의 실제 각도
- `vehicle_status`, `vehicle_land_detected`: arming 및 착륙 상태

결과는 `palletrone_mob_status`로 발행한다.

| 필드 | 의미 |
|---|---|
| `external_force[3]` | raw 외력 추정값 `[N]` |
| `external_torque[3]` | raw 외부 토크 추정값 `[N m]` |
| `external_force_corrected[3]` | force bias를 뺀 외력 `[N]` |
| `external_torque_corrected[3]` | torque bias를 뺀 토크 `[N m]` |
| `known_actuator_force[3]` | 재구성한 actuator 힘 |
| `known_actuator_torque[3]` | 재구성한 actuator 토크 |
| `linear_momentum_error[3]` | 선형 운동량 오차 |
| `angular_momentum_error[3]` | 각운동량 오차 |
| `force_valid`, `torque_valid` | 해당 추정값의 사용 가능 여부 |
| `status_flags` | 입력, 파라미터, warm-up 및 reset 상태 |

## 현재 기본 파라미터

### 기체 모델

| 파라미터 | 기본값 | 단위 |
|---|---:|---|
| `PMOB_EN` | `true` | - |
| `PMOB_MASS` | `1.6` | kg |
| `PMOB_IXX` | `0.0192` | kg m^2 |
| `PMOB_IYY` | `0.02125` | kg m^2 |
| `PMOB_IZZ` | `0.0282` | kg m^2 |
| `PMOB_IXY`, `PMOB_IXZ`, `PMOB_IYZ` | `0.0` | kg m^2 |

관성 행렬은 양의 정부호여야 한다. 질량이나 관성값을 변경하면 관측기 상태가 재초기화된다.

### 축별 관측기 게인

| 축 | 자연주파수 `wn` | 감쇠비 `zeta` |
|---|---:|---:|
| Force X | `PMOB_FX_WN = 6.0` | `PMOB_FX_ZETA = 0.9` |
| Force Y | `PMOB_FY_WN = 6.0` | `PMOB_FY_ZETA = 0.9` |
| Force Z | `PMOB_FZ_WN = 6.0` | `PMOB_FZ_ZETA = 0.9` |
| Torque X | `PMOB_TX_WN = 10.0` | `PMOB_TX_ZETA = 0.9` |
| Torque Y | `PMOB_TY_WN = 10.0` | `PMOB_TY_ZETA = 0.9` |
| Torque Z | `PMOB_TZ_WN = 10.0` | `PMOB_TZ_ZETA = 0.9` |

- `wn`을 높이면 응답이 빨라지지만 센서 노이즈와 모델 오차에 민감해진다.
- `zeta`를 높이면 overshoot가 줄어들지만 응답이 완만해질 수 있다.
- 한 번에 한 축만 조정하고 raw 출력과 momentum error를 함께 확인하는 것을 권장한다.

### 6축 출력 바이어스

| 파라미터 | 범위 | 기본값 |
|---|---:|---:|
| `PMOB_BIAS_FX`, `PMOB_BIAS_FY`, `PMOB_BIAS_FZ` | `-50 .. 50 N` | `0.0` |
| `PMOB_BIAS_TX`, `PMOB_BIAS_TY`, `PMOB_BIAS_TZ` | `-10 .. 10 N m` | `0.0` |

보정값은 다음 식으로 계산한다.

```text
external_force_corrected  = external_force  - PMOB_BIAS_F*
external_torque_corrected = external_torque - PMOB_BIAS_T*
```

바이어스는 발행 직전 출력에만 적용한다. 관측기 내부 상태, 동역학 또는 raw 출력은 변경하지 않는다. 따라서 운용 중 바이어스를 변경해도 관측기가 reset되지 않는다. 파라미터가 유한하지 않으면 보정 출력은 `NaN`이 되고 parameter-invalid 상태가 설정된다.

예를 들어 raw `Fx = -3 N`이고 `PMOB_BIAS_FX = -1 N`이면 corrected `Fx = -2 N`이다.

### 유효성 및 타이밍

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `PMOB_ODOM_TO` | `0.1 s` | odometry timeout |
| `PMOB_ACT_TO` | `0.1 s` | thrust command timeout |
| `PMOB_SERVO_TO` | `0.1 s` | servo angle timeout |
| `PMOB_WARMUP` | `1.0 s` | reset 후 출력 invalid 시간 |
| `PMOB_ARM_ONLY` | `true` | armed 상태에서만 추정값 valid |

관측기 sample 간격은 `vehicle_odometry.timestamp_sample`에서 계산하며 `0.5 ms <= dt <= 50 ms`일 때만 허용한다. 입력 timeout, timestamp 역행, estimator reset 또는 잘못된 `dt`가 감지되면 관측기를 재초기화하고 warm-up을 다시 시작한다.

`external_force`와 `external_torque`는 진단을 위한 raw 값이므로, 실제 사용자는 `force_valid`/`torque_valid`와 `warmup_complete`를 확인한 뒤 corrected 출력을 사용해야 한다.

## NSH 사용법

```sh
palletrone_mob start
palletrone_mob status
palletrone_mob stop
listener palletrone_mob_status
```

파라미터 변경 예시:

```sh
param set PMOB_BIAS_FX -1.0
param set PMOB_FZ_WN 5.0
param set PMOB_FZ_ZETA 1.0
```

## 관련 파일

- `PalletroneMob.cpp/.hpp`: 모듈 실행, 입력 검증, actuator wrench 계산 및 상태 발행
- `MomentumObserver.hpp`: 축별 2차 운동량 관측기
- `palletrone_mob_physical_parms.hpp`: MOB 전용 기체/추진 형상 상수
- `module.yaml`: PX4 파라미터 정의
- `msg/PalletroneMobStatus.msg`: uORB 출력 메시지 정의

