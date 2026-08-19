# Palletrone MOB Axis-Specific Gain Update

## Repository Audit

작업 저장소는 `~/HIH_PX4_6X`이다. 작업 전 audit 결과는 다음과 같다.

```bash
pwd
/home/jeongsu/HIH_PX4_6X

git status --short
git branch --show-current
git log -1 --oneline

rg -n \
"PMOB_F_WN|PMOB_F_ZETA|PMOB_T_WN|PMOB_T_ZETA|MomentumObserver|palletrone_mob" \
src MOB_IMPLEMENTATION_REPORT.md
```

작업 전부터 존재하던 unrelated dirty file은 `.vscode/*`, `boards/px4/fmu-v6x/init/rc.board_defaults`,
`src/modules/uxrce_dds_client/dds_topics.yaml`, `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`,
`src/modules/palletrone_mob/CMakeLists.txt`, `src/modules/palletrone_mob/PalletroneMob.cpp`,
`src/modules/palletrone_mob/module.yaml`, `src/modules/palletrone_mob/palletrone_mob_physical_parms.hpp` 등이었다.
이번 작업에서는 MOB observer gain 확장과 직접 관계있는 파일만 수정했다.

## Before

변경 전 observer gain 구조는 다음과 같았다.

```text
Force group shared:
    PMOB_F_WN
    PMOB_F_ZETA

Torque group shared:
    PMOB_T_WN
    PMOB_T_ZETA
```

기존 `MomentumObserver::update(...)`는 `float natural_frequency, float damping_ratio`
를 받아 세 축 전체에 동일한 scalar gain을 재사용했다.

```text
L1 = 2 zeta wn
L2 = wn^2
```

이 scalar gain이 각 loop iteration 안에서 재계산되지 않고 x/y/z에 공통 적용되는 구조였다.

## After

변경 후 gain parameter는 축별 12개다.

```text
Fx:
    PMOB_FX_WN
    PMOB_FX_ZETA

Fy:
    PMOB_FY_WN
    PMOB_FY_ZETA

Fz:
    PMOB_FZ_WN
    PMOB_FZ_ZETA

Tx:
    PMOB_TX_WN
    PMOB_TX_ZETA

Ty:
    PMOB_TY_WN
    PMOB_TY_ZETA

Tz:
    PMOB_TZ_WN
    PMOB_TZ_ZETA
```

축 정의는 body FRD이다.

```text
Fx: body +x forward force
Fy: body +y right force
Fz: body +z downward force

Tx: body +x roll torque
Ty: body +y pitch torque
Tz: body +z yaw torque
```

기본값은 기존 수치 응답을 보존하도록 설정했다.

```text
PMOB_FX_WN   = 6.0
PMOB_FX_ZETA = 0.9
PMOB_FY_WN   = 6.0
PMOB_FY_ZETA = 0.9
PMOB_FZ_WN   = 6.0
PMOB_FZ_ZETA = 0.9

PMOB_TX_WN   = 10.0
PMOB_TX_ZETA = 0.9
PMOB_TY_WN   = 10.0
PMOB_TY_ZETA = 0.9
PMOB_TZ_WN   = 10.0
PMOB_TZ_ZETA = 0.9
```

즉 새 parameter를 따로 바꾸지 않으면 기존 shared-gain observer와 수치적으로 동일하게 동작한다.

## Observer Math

이제 각 축은 자신의 `wn(i)`와 `zeta(i)`를 사용한다.

```text
L1,i = 2 zeta_i wn_i
L2,i = wn_i^2
```

예를 들면:

```text
L1,Fx = 2 zeta_Fx wn_Fx
L2,Fx = wn_Fx^2
```

나머지 `Fy`, `Fz`, `Tx`, `Ty`, `Tz`도 같은 방식이다.

Tustin update 역시 축별 coefficient로 분리했다.

```text
h    = dt / 2
D_i  = 1 + h L1,i + h^2 L2,i
r_p,i = (1 - h L1,i) p_hat_i + h w_hat_i + dt (b_i + L1,i p_i)
r_w,i = -h L2,i p_hat_i + w_hat_i + dt L2,i p_i

p_hat_i(k+1) = (r_p,i + h r_w,i) / D_i
w_hat_i(k+1) = (-h L2,i r_p,i + (1 + h L1,i) r_w,i) / D_i
e_i(k+1)     = p_i(k) - p_hat_i(k+1)
```

기존처럼 아래 scalar를 세 축에 재사용하는 코드는 제거했다.

```text
const float l1 = 2 * zeta * wn;
const float l2 = wn * wn;
```

## Atomic State Update

Observer state 구조는 그대로 유지했다.

```text
p_hat
w_hat
momentum error
initialized
```

다만 한 축 계산 후 즉시 member를 갱신하지 않고, 세 축 모두 local temporary에 계산한 뒤
모든 결과가 finite일 때만 commit한다.

```text
next_p_hat
next_w_hat
next_error
```

다음 경우 update는 실패하고 기존 state는 그대로 유지된다.

```text
dt <= 0
dt nonfinite
momentum nonfinite
known dynamics nonfinite
wn(i) <= 0
zeta(i) <= 0
gain NaN/Inf
D_i <= 0
computed next state NaN/Inf
```

## Parameter Validity Split

parameter validity는 다음처럼 동작한다.

```text
common model parameters valid
force gains valid
torque gains valid
```

의미는 다음과 같다.

```text
mass/inertia/common model invalid:
    force와 torque 둘 다 invalid

Fx/Fy/Fz gain 중 하나 invalid:
    force observer 전체 invalid
    torque observer는 torque gain이 valid이면 계속 동작 가능

Tx/Ty/Tz gain 중 하나 invalid:
    torque observer 전체 invalid
    force observer는 force gain이 valid이면 계속 동작 가능
```

message schema는 변경하지 않았다. 따라서 축별 validity flag는 추가하지 않았고,
기존 `FLAG_PARAMETER_INVALID` 의미는 유지했다.

## Runtime Parameter Change

runtime gain 변경 시 새 gain은 다음 observer update부터 바로 적용된다.

```text
gain parameter 변경:
    observer state 유지
    p_hat, w_hat, error, initialized reset 안 함
```

반대로 momentum 정의 자체가 바뀌는 parameter는 observer reset을 유도한다.

```text
mass 또는 inertia 변경:
    _active_last_cycle = false
    다음 valid sample에서 resetObservers()
```

timeout, warm-up, arm gate, enable parameter의 기존 validity gating은 유지했다.

## Migration Note

기존 4개 parameter는 production code와 metadata에서 제거했다.
기존 parameter file이나 QGC setup을 쓰는 경우 새 이름으로 옮겨야 한다.

예:

```text
PMOB_F_WN = 6
    ->
PMOB_FX_WN = 6
PMOB_FY_WN = 6
PMOB_FZ_WN = 6
```

```text
PMOB_F_ZETA = 0.9
    ->
PMOB_FX_ZETA = 0.9
PMOB_FY_ZETA = 0.9
PMOB_FZ_ZETA = 0.9
```

```text
PMOB_T_WN = 10
    ->
PMOB_TX_WN = 10
PMOB_TY_WN = 10
PMOB_TZ_WN = 10
```

```text
PMOB_T_ZETA = 0.9
    ->
PMOB_TX_ZETA = 0.9
PMOB_TY_ZETA = 0.9
PMOB_TZ_ZETA = 0.9
```

## Test Coverage

synthetic test는 다음 항목을 유지하거나 추가 검증한다.

```text
hover
free fall
+Fx
+Fy
+Fz
+Tx
+Ty
+Tz
gyroscopic sign
omega cross v sign
variable dt
reset
stale input
FRD geometry
equal-gain regression vs legacy scalar reference
force axis-specific natural frequency ordering
torque axis-specific natural frequency ordering
axis isolation
damping-ratio effect
invalid gain rejection without partial update
equal-gain symmetry
```

## Parameter Metadata Check

PX4 generator 스크립트로 `generated_params/module_params.c`, `parameters.xml`,
`parameters.json`을 재생성해 다음 항목을 확인했다.

```text
PMOB_FX_WN
PMOB_FX_ZETA
PMOB_FY_WN
PMOB_FY_ZETA
PMOB_FZ_WN
PMOB_FZ_ZETA
PMOB_TX_WN
PMOB_TX_ZETA
PMOB_TY_WN
PMOB_TY_ZETA
PMOB_TZ_WN
PMOB_TZ_ZETA
```

그리고 legacy 이름은 `src`와 generated metadata에서 제거했다.
이 보고서 안에서는 migration 설명 때문에 legacy 이름을 유지한다.

## Runtime Usage Example

현재 gain 확인:

```bash
param show PMOB_*_WN
param show PMOB_*_ZETA
```

Fx observer만 상대적으로 빠르게:

```bash
param set PMOB_FX_WN 8.0
param set PMOB_FX_ZETA 0.9

param set PMOB_FY_WN 4.0
param set PMOB_FY_ZETA 1.0

param set PMOB_FZ_WN 4.0
param set PMOB_FZ_ZETA 1.0
```

yaw torque observer를 느리게:

```bash
param set PMOB_TZ_WN 6.0
param set PMOB_TZ_ZETA 1.0

param set PMOB_TX_WN 10.0
param set PMOB_TX_ZETA 0.9

param set PMOB_TY_WN 10.0
param set PMOB_TY_ZETA 0.9
```

diagnostics-only 사용 명령:

```bash
palletrone_mob start
palletrone_mob status
listener palletrone_mob_status 10
```

parameter 저장과 reboot 후 유지 여부는 PX4 parameter system 기존 동작을 따른다.

## Raw and bias-corrected MOB output

The second-order observer dynamics remain unchanged. `external_force` and
`external_torque` are the raw BODY_FRD residual at the modeled CoM/body origin.
Immediately before status publication, the output layer computes
`external_force_corrected = external_force - force_bias` and
`external_torque_corrected = external_torque - torque_bias`. The six manual
parameters are `PMOB_BIAS_FX/FY/FZ` [N] and `PMOB_BIAS_TX/TY/TZ` [N m]. A finite
runtime change applies to the next published sample and does not reset observer
state. Nonfinite bias values set parameter-invalid and are not applied.

Manual tuning procedure: keep Admittance OFF, hover without contact, wait for
MOB warm-up, average 5--10 seconds of each raw force/torque axis, and enter those
means as the corresponding `PMOB_BIAS_*` values. Verify that corrected wrench is
near zero, then select Admittance deadbands around three standard deviations of
the corrected residual noise. Bias removes a systematic/DC offset; deadband
rejects remaining noise. No automatic tare is implemented.

Corrected MOB wrench is not ground-truth contact wrench. Thrust-model and
voltage variation, aerodynamics, servo dynamics, EKF/H-Flow errors, ground
effect, and rotor interaction can remain after a fixed bias is subtracted.

## Build Note

이 작업 환경에서는 `make`, `cmake`, `ninja`, `arm-none-eabi-g++` binary가 현재 실행 가능한 PATH에 존재하지 않아
전체 firmware 재빌드는 수행되지 못했다. 대신 다음은 수행했다.

```text
python3 src/modules/palletrone_mob/test/test_palletrone_mob.py
PX4 공식 generator 스크립트로 generated_params/module_params.c 재생성
PX4 공식 generator 스크립트로 parameters.xml / parameters.json 재생성
기존 firmware artifact 경로와 크기 확인
```

기존 firmware artifact는 다음 위치에 남아 있다.

```text
build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4
```
