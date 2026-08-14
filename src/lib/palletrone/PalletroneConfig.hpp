/****************************************************************************
 *
 * Palletrone 4 kg mini-airframe physical and propulsion configuration.
 *
 * Vehicle-dependent constants are kept in one header so that the position
 * controller, allocator, DOB, CoM estimator, and L1 controller use the same
 * mass, geometry, inertia, and motor model.
 *
 ****************************************************************************/

#pragma once

namespace palletrone
{

// --------------------------------------------------------------------------
// Airframe configuration
// --------------------------------------------------------------------------

// 이전 대형 기체: vehicle_mass = 8.0f kg
// 변경 미니 기체: mini_vehicle_mass = 4.0f kg
static constexpr float kVehicleMassKg = 4.0f;

// 이전 대형 기체: arm_length = 0.230f m
// 변경 미니 기체: mini_arm_length = 0.181f m
// Assumption: X-frame CoG-to-motor-axis distance.
static constexpr float kArmLengthM = 0.181f;

// Rotor hubs are an X configuration about the modeled CoM/body origin.
// FRD order matches allocator/PX4 motor indices: front-left, rear-left,
// rear-right, front-right. The rotor plane is 5 mm above the origin.
static constexpr float kRotorVerticalOffsetM = -0.005f;
static constexpr float kRotorReactionTorqueRatioM = 0.01f;
static constexpr float kInvSqrt2 = 0.7071067811865475f;
// Positive actual servo angle horizontal thrust signs in FRD. At zero angle
// every rotor thrust direction is [0, 0, -1] (upward).
static constexpr float kTiltDirectionXY[4][2] = {
	{ 1.f,  1.f}, { 1.f, -1.f}, {-1.f, -1.f}, {-1.f,  1.f}
};

// Reaction torque is sigma * kappa * T * thrust_direction.
static constexpr float kRotorReactionSign[4] = {1.f, -1.f, 1.f, -1.f};

// 이전 8 kg 기체 관성: [0.0768, 0.0871, 0.1130] kg*m^2
// 변경 4 kg 기체: 요청대로 질량 비례 근사로 각 축을 정확히 1/4 적용.
static constexpr float kInertiaXxKgM2 = 0.0192f;
static constexpr float kInertiaYyKgM2 = 0.02125f;
static constexpr float kInertiaZzKgM2 = 0.0282f;

// 이전 allocator의 0.5 N 하한은 사용자가 '나머지는 그대로'를 요청하여 유지.
static constexpr float kMotorMinimumForceN = 1.00f;

// --------------------------------------------------------------------------
// T-Motor F60 PRO V-LV 2020KV + 6S propulsion model
// --------------------------------------------------------------------------

// 예정 프로펠러는 5540 3엽이지만 제공된 제조사 자료에는 해당 조합의 정확한
// 수치표가 없으므로, 제공 자료의 T5147-3 @ 25.2 V 데이터를 초기 proxy로 사용.
// 실제 5540 3엽 장착 후 추력 스탠드 측정값으로 아래 표를 교체해야 한다.
static constexpr int kMotorModelPointCount = 6;

static constexpr float kMotorThrottleTable[kMotorModelPointCount] = {
	0.00f,
	0.20f,
	0.40f,
	0.60f,
	0.80f,
	1.00f
};

// 제조사 gf 데이터를 9.80665 m/s^2로 N 단위 변환.
static constexpr float kMotorThrustTableN[kMotorModelPointCount] = {
	0.0f,
	2.76743663f,   // 282.2 gf @ 20%
	6.83033173f,   // 696.5 gf @ 40%
	10.55293606f,  // 1076.1 gf @ 60%
	16.37024084f,  // 1669.3 gf @ 80%
	20.43215527f   // 2083.5 gf @ 100%
};

// 이전 모델: maximum_thrust_limit = 0.85f
// 변경 미니 모델: mini_model_max_throttle = 0.80f
static constexpr float kMotorThrottleLimit = 0.80f;

static constexpr float kMotorFullThrustN = kMotorThrustTableN[5];

// 요청 사항: 확인된 최대 정적 추력의 80%까지만 allocator가 요구하도록 제한.
// DShot 실제 명령도 별도로 0.80에 제한하여 두 단계로 보호한다.
static constexpr float kMotorMaxUsableThrustN = kMotorThrottleLimit * kMotorFullThrustN;
static constexpr float kTotalMaxUsableThrustN = 4.0f * kMotorMaxUsableThrustN;

// 이 소스 트리의 PX4 DShot 기본값. 실제 변환에는 런타임 DSHOT_MIN을 읽는다.
static constexpr float kDshotMinDefault = 0.055f;

static_assert(kVehicleMassKg > 0.0f, "Vehicle mass must be positive");
static_assert(kArmLengthM > 0.0f, "Arm length must be positive");
static_assert(kMotorThrottleLimit > kDshotMinDefault && kMotorThrottleLimit <= 1.0f,
	      "DShot throttle limit must be above DSHOT_MIN and no greater than 1");

} // namespace palletrone
