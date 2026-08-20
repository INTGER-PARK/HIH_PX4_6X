# Palletrone Admittance Activation and Setpoint Ownership Change

## 목적

이 문서는 Palletrone Admittance의 활성 조건과 position/velocity setpoint ownership 상태 전이를 수정한 내용을 기록한다.

변경 전에는 QGC Position mode가 활성화되어 있어도 legacy ROS2 `/control_mode_flags` publisher가 없으면 `custom_mode_flag=false`가 유지되어 Admittance가 `WAIT_VALID`에 머물렀다. 또한 유효성 검사를 통과하기 전에 setpoint를 latch하여 `WAIT_VALID`에서도 upstream setpoint를 덮어썼고, 이전 `_effective_velocity`가 남을 수 있었다.

이번 변경의 목표는 다음과 같다.

- Admittance를 legacy `custom_mode_flag`와 독립적으로 활성화한다.
- `WAIT_VALID`에서는 upstream QGC position/velocity setpoint를 그대로 통과시킨다.
- 유효성 검사를 통과한 시점에만 setpoint ownership을 획득한다.
- Request OFF 후 안전한 handover가 끝나면 ownership을 자동 반환한다.
- reset 및 재진입 시 이전 velocity setpoint가 남지 않게 한다.

## 변경 파일

- `src/modules/mc_pos_control/MulticopterPositionControl.cpp`
- `src/modules/mc_pos_control/palletrone_admittance/AdmittanceManager.hpp`
- `src/modules/mc_pos_control/palletrone_admittance/test/test_admittance.py`

기존 `custom_control_mode`, `/control_mode_flags`, DOB, custom trajectory, payload 및 L1 adaptive 관련 로직은 삭제하거나 의미를 변경하지 않았다.

## Admittance Position mode 조건 분리

변경 전에는 Admittance의 `position_mode` 인자가 다음 조건이었다.

```cpp
_vehicle_control_mode.flag_control_position_enabled &&
_custom_control_mode.custom_mode_flag
```

따라서 QGC Position mode가 활성화되어도 `/control_mode_flags` publisher가 없으면 `custom_mode_flag=false` 때문에 `basic_valid=false`가 되었다.

변경 후에는 Admittance 전용 조건을 사용한다.

```cpp
const bool admittance_position_mode =
	_vehicle_control_mode.flag_control_position_enabled;
```

이 값만 `AdmittanceManager::update()`의 `position_mode` 인자로 전달한다. `custom_mode_flag` 자체와 다른 legacy controller에서의 사용은 그대로 유지한다. `trajectory_flag`도 기존 호환성을 위해 계속 전달하지만 ownership 반환의 필수 조건으로 사용하지 않는다.

## 유지되는 안전 조건

`basic_valid`의 다음 조건은 그대로 유지된다.

```cpp
basic_valid =
	PADM_EN &&
	request &&
	admittance_position_mode &&
	attitude_valid &&
	(!PADM_ARM_ONLY || armed) &&
	dynamics_valid &&
	force_valid &&
	yaw_valid;
```

다음 로직도 변경하지 않았다.

- command freshness와 Request enable
- `PADM_ARM_ONLY` 및 armed 검사
- attitude와 parameter validity
- MOB freshness와 force/yaw validity
- contact hysteresis와 dwell/grace time
- M-D-K 수식, force error 부호 및 deadzone
- 좌표계 변환과 MOB wrench
- axis gain과 input/acceleration/velocity/position limit
- handover 감속과 fault hold

`PADM_ARM_ONLY` 기본값은 계속 `true`이다. Disarmed 벤치 시험에서는 사용자가 명시적으로 `PADM_ARM_ONLY=0`을 설정해야 한다.

## 상태별 Setpoint Ownership

| 상태 | Ownership | Position 출력 | Velocity 출력 |
|---|---:|---|---|
| `DISABLED_HOLD` | 없음 | upstream 통과 | upstream 통과 |
| `WAIT_VALID` | 없음 | upstream 통과 | upstream 통과, Admittance 내부 출력은 0 |
| `WAIT_CONTACT` | 있음 | validity 통과 시 latch한 위치 hold | 0 |
| `BLEND_IN` | 있음 | Admittance offset 적용 | blend된 M-D-K velocity |
| `ACTIVE` | 있음 | Admittance offset 적용 | M-D-K velocity |
| `HANDOVER_DECEL` | 있음 | 감속 중 offset 적용 | 기존 damping 기반 감속 velocity |
| `FAULT_HOLD` | 있음 | 안전 위치 hold | 0 |

실제 setpoint 덮어쓰기는 `_hold_latched=true`일 때만 수행한다.

```cpp
if (_hold_latched) {
	applyEffective(position_yaw_sp, velocity_yawrate_sp);
}
```

따라서 단순히 상태가 `WAIT_VALID`이라는 이유만으로 upstream setpoint를 덮어쓰지 않는다.

## 변경 전후 상태 전이

### 변경 전

```text
Request ON
  -> basic_valid 확인 전에 latchBase()
  -> hold_latched=true
  -> basic_valid=false
  -> WAIT_VALID
  -> M-D-K 계산 정지
  -> position/velocity setpoint는 계속 덮어씀
```

### 변경 후

```text
Request ON
  -> basic_valid=false
  -> WAIT_VALID
  -> hold_latched=false
  -> Admittance velocity=0
  -> upstream QGC position/velocity setpoint 통과

basic_valid=true
  -> 현재 upstream setpoint를 latchBase()
  -> hold_latched=true
  -> contact=false: WAIT_CONTACT
  -> contact=true: BLEND_IN -> ACTIVE
```

`WAIT_CONTACT`에서 validity가 다시 깨지면 ownership을 해제하고 `WAIT_VALID`로 돌아간다. validity가 회복되면 그 시점의 최신 upstream setpoint를 다시 latch한다.

## Request OFF와 Ownership 반환

### WAIT_VALID 또는 WAIT_CONTACT

M-D-K 기반 velocity가 실행 중이지 않으므로 handover 없이 즉시 반환한다.

```text
Request OFF
  -> releaseOwnership()
  -> DISABLED_HOLD
  -> hold_latched=false
  -> upstream setpoint 통과
```

### BLEND_IN 또는 ACTIVE

기존 안전 감속을 유지한다.

```text
Request OFF 또는 validity/contact 상실
  -> HANDOVER_DECEL
  -> zero input + damping 기반 감속
  -> 정지 threshold 충족 또는 timeout
  -> atomicRebase()
  -> releaseOwnership()
  -> DISABLED_HOLD
  -> upstream controller에 ownership 반환
```

기존에는 handover 종료 후 `_hold_latched=true`가 남고 legacy `trajectory_flag` rising edge가 있어야 해제될 수 있었다. 변경 후 handover 종료 자체가 ownership을 자동 반환한다. legacy rising-edge 해제 경로는 호환성을 위해 남겨 두었다.

## Stale Velocity 제거

이전에는 `latchBase()`가 `_offset`과 M-D-K core만 초기화하고 `_effective_velocity`를 초기화하지 않았다. 이 때문에 이전 세션의 `+0.060 m/s` 같은 feed-forward가 새 요청에서 남을 수 있었다.

다음 경로에서 `_effective_velocity.zero()`를 보장한다.

- `latchBase()`
- `resetAt()`
- `severeFault()`
- `releaseOwnership()`
- `atomicRebase()`는 `releaseOwnership()`을 통해 초기화

`WAIT_CONTACT`, `DISABLED_HOLD`, `FAULT_HOLD`에서 Admittance가 ownership을 가진 경우에도 외부로 적용하는 velocity는 명시적으로 0으로 만든다. `HANDOVER_DECEL`의 감속 velocity는 유지한다.

## 시나리오별 예상 동작

### A. QGC Position, Armed 조건 불충족

```text
flag_control_position_enabled=true
custom_mode_flag=false
PADM_ARM_ONLY=1
armed=false
```

예상 결과:

```text
WAIT_VALID
hold_latched=false
Admittance effective velocity=0
upstream setpoint 통과
```

### B. Disarmed 벤치 시험

```text
flag_control_position_enabled=true
custom_mode_flag=false
PADM_ARM_ONLY=0
나머지 validity 만족
```

예상 결과:

```text
contact=false: WAIT_CONTACT
contact=true: BLEND_IN -> ACTIVE
```

`custom_mode_flag`는 진입 여부에 영향을 주지 않는다.

### C. Manual mode

```text
flag_control_position_enabled=false
```

신규 ACTIVE 진입은 금지된다. `WAIT_CONTACT`라면 즉시 ownership을 반환하고 `WAIT_VALID`로 간다. 이미 `BLEND_IN` 또는 `ACTIVE`라면 갑작스러운 velocity 절단 대신 `HANDOVER_DECEL`을 거쳐 ownership을 반환한다.

### D. WAIT_VALID에서 Request OFF

```text
WAIT_VALID -> DISABLED_HOLD
hold_latched=false
```

### E. ACTIVE에서 Request OFF

```text
ACTIVE -> HANDOVER_DECEL -> DISABLED_HOLD
hold_latched: true -> false
```

### F. 다시 Request ON

현재 upstream setpoint를 새로 latch하고 `_offset`, M-D-K core 및 `_effective_velocity`를 모두 초기화한다. 이전 세션 velocity가 재사용되지 않으며 0에서 새 응답을 시작한다.

## 검증 결과

Host 회귀 테스트:

```bash
python3 src/modules/mc_pos_control/palletrone_admittance/test/test_admittance.py
```

결과:

```text
PASS: Admittance4D dynamics/sign/invalid/atomic/axis and reference/contact/handover invariants
```

FMUv6X 빌드:

```bash
make px4_fmu-v6x_default -j1
```

결과:

```text
Build success
FLASH: 1808480 B / 1920 KB (91.98%)
```

생성된 펌웨어:

```text
build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4
size: 1692813 bytes
build time: 2026-08-20 21:48:24 KST
```

## QGC 및 PX4 콘솔 확인 순서

먼저 다음 항목을 확인한다.

```sh
listener vehicle_control_mode
listener custom_control_mode
listener palletrone_admittance_status
listener palletrone_mob_status
```

정상적인 실제 비행 진입 순서는 다음과 같다.

```text
1. Request OFF
   DISABLED_HOLD, hold_latched=false

2. QGC Position mode 선택
   flag_control_position_enabled=true
   custom_mode_flag 값은 Admittance 진입에 무관

3. Request ON, 일부 validity 불충족
   WAIT_VALID, hold_latched=false, velocity=0

4. 모든 basic validity 만족, contact 미충족
   WAIT_CONTACT, hold_latched=true, velocity=0

5. contact dwell 만족
   BLEND_IN -> ACTIVE

6. Request OFF
   HANDOVER_DECEL -> DISABLED_HOLD
   hold_latched=false
```

QGC에서 특히 다음 필드를 함께 확인한다.

- `state`
- `enable_requested`
- `command_fresh`
- `active`
- `contact`
- `hold_latched`
- `velocity[4]`
- `effective_velocity_setpoint[3]`
- `status_flags`

