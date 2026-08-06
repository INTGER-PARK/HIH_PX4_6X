#include "ControlAllocationUtils.hpp"

#include <lib/palletrone/PalletroneConfig.hpp>
#include <px4_platform_common/defines.h>

namespace
{

float clampf(float value, float lower, float upper)
{
	return value < lower ? lower : (value > upper ? upper : value);
}

float validated_dshot_min(float dshot_min_norm)
{
	if (!PX4_ISFINITE(dshot_min_norm)) {
		return palletrone::kDshotMinDefault;
	}

	// Keep a valid denominator and preserve room below the 80% safety ceiling.
	return clampf(dshot_min_norm, 0.0f, palletrone::kMotorThrottleLimit - 0.001f);
}

} // namespace

float dshot_throttle_to_actuator_control(float esc_throttle, float dshot_min_norm)
{
	const float dshot_min = validated_dshot_min(dshot_min_norm);
	const float throttle = clampf(esc_throttle, dshot_min, palletrone::kMotorThrottleLimit);

	// PX4 non-reversible DShot mapping:
	// actual_throttle = DSHOT_MIN + actuator_control * (1 - DSHOT_MIN)
	return clampf((throttle - dshot_min) / (1.0f - dshot_min), 0.0f, 1.0f);
}

float force_to_dshot_control(float force_n, float dshot_min_norm)
{
	// NaN/non-positive requests map to actuator control 0. Disarming remains
	// handled by the PX4 output driver rather than this conversion function.
	if (!PX4_ISFINITE(force_n) || force_n <= 0.0f) {
		return 0.0f;
	}

	// 이전 analog PWM 모델:
	//   force -> inverse quadratic PWM pulse width -> normalized 1100~1900 us
	// 변경 DShot 모델:
	//   force -> F60 lookup-table throttle -> PX4 DShot normalized command
	const float limited_force = clampf(force_n, 0.0f, palletrone::kMotorMaxUsableThrustN);
	float esc_throttle = palletrone::kMotorThrottleLimit;

	for (int i = 0; i < palletrone::kMotorModelPointCount - 1; ++i) {
		const float force_low = palletrone::kMotorThrustTableN[i];
		const float force_high = palletrone::kMotorThrustTableN[i + 1];

		if (limited_force <= force_high) {
			const float interpolation = (limited_force - force_low) / (force_high - force_low);
			esc_throttle = palletrone::kMotorThrottleTable[i]
			       + interpolation * (palletrone::kMotorThrottleTable[i + 1]
						  - palletrone::kMotorThrottleTable[i]);
			break;
		}
	}

	// Final physical ESC throttle is independently capped at 80%.
	esc_throttle = clampf(esc_throttle, validated_dshot_min(dshot_min_norm),
			      palletrone::kMotorThrottleLimit);
	return dshot_throttle_to_actuator_control(esc_throttle, dshot_min_norm);
}
