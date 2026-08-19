/****************************************************************************
 *
 * Palletrone MOB-local physical and propulsion configuration.
 *
 * This header intentionally duplicates the constants that the momentum
 * observer needs so that `palletrone_mob` does not depend on the shared
 * `src/lib/palletrone` configuration.
 *
 ****************************************************************************/

#pragma once

namespace palletrone_mob::physical_parms
{

static constexpr float kDroneMassKg = 1.6f;
static constexpr float kBatteryMassKg = 0.0f;
static constexpr float kVehicleMassKg = kDroneMassKg + kBatteryMassKg;

static constexpr float kInertiaXxKgM2 = 0.0192f;
static constexpr float kInertiaYyKgM2 = 0.02125f;
static constexpr float kInertiaZzKgM2 = 0.0282f;
static constexpr float kInertiaXyKgM2 = 0.0f;
static constexpr float kInertiaXzKgM2 = 0.0f;
static constexpr float kInertiaYzKgM2 = 0.0f;

static constexpr float kArmLengthM = 0.181f;
static constexpr float kRotorVerticalOffsetM = 0.0f;
static constexpr float kRotorReactionTorqueRatioM = 0.01f;
static constexpr float kInvSqrt2 = 0.7071067811865475f;

static constexpr float kRotorPositionXY[4][2] = {
	{ 1.f, -1.f}, {-1.f, -1.f}, {-1.f,  1.f}, { 1.f,  1.f}
}; //rotor position임

static constexpr float kTiltDirectionXY[4][2] = {
	{ 1.f,  1.f}, { 1.f, -1.f}, {-1.f, -1.f}, {-1.f,  1.f}
};

static constexpr float kRotorReactionSign[4] = {1.f, -1.f, 1.f, -1.f};

static_assert(kVehicleMassKg > 0.0f, "Vehicle mass must be positive");
static_assert(kArmLengthM > 0.0f, "Arm length must be positive");

} // namespace palletrone_mob::physical_parms
