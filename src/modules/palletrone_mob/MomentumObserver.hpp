#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

namespace palletrone_mob
{

// Per-axis second-order extended momentum observer. The bilinear (Tustin)
// update avoids the numerical-differentiation and forward-Euler paths.
class MomentumObserver
{
public:
	/**
	 * Initialize one three-axis observer from the currently measured momentum.
	 * The external wrench state is deliberately cleared so estimator startup or
	 * reset cannot inject a stale residual into the published result.
	 * @param momentum measured linear [kg m/s] or angular [kg m^2/s] momentum
	 */
	void reset(const matrix::Vector3f &momentum)
	{
		_p_hat = momentum;
		_w_hat.setZero();
		_error.setZero();
		_initialized = true;
	}

	/**
	 * Mark the observer unusable and clear all internal states.
	 * A subsequent update is rejected until reset() supplies a valid momentum.
	 */
	void invalidate()
	{
		_initialized = false;
		_p_hat.setZero();
		_w_hat.setZero();
		_error.setZero();
	}

	/**
	 * Advance the extended momentum observer by one variable-duration sample.
	 * This implements a bilinear/Tustin discretization independently on x/y/z:
	 *   p_dot = b + w_ext
	 *   e_p = p - p_hat
	 *   p_hat_dot = b + w_hat + 2*zeta*wn*(p-p_hat)
	 *   w_hat_dot = wn^2*(p-p_hat)
	 * Here w_hat is the estimated residual/external wrench; ideally
	 * w_hat -> w_ext = p_dot - b.
	 * It performs no numerical differentiation and rejects invalid inputs.
	 * @param momentum current measured generalized momentum p
	 * @param known_dynamics modeled momentum derivative b without external wrench
	 * @param dt accepted sample interval in seconds
	 * @return true only when every updated state is finite
	 */
	bool update(const matrix::Vector3f &momentum, const matrix::Vector3f &known_dynamics,
		    float dt, float natural_frequency, float damping_ratio)
	{
		if (!_initialized || !momentum.isAllFinite() || !known_dynamics.isAllFinite()
		    || !PX4_ISFINITE(dt) || !PX4_ISFINITE(natural_frequency) || !PX4_ISFINITE(damping_ratio)
		    || dt <= 0.f || natural_frequency <= 0.f || damping_ratio <= 0.f) {
			return false;
		}

		// Observer gains: l1 = 2*zeta*wn, l2 = wn^2.
		const float l1 = 2.f * damping_ratio * natural_frequency;
		const float l2 = natural_frequency * natural_frequency;
		const float h = 0.5f * dt;
		const float determinant = 1.f + h * l1 + h * h * l2;

		if (!PX4_ISFINITE(determinant) || determinant <= 0.f) {
			return false;
		}

		for (int axis = 0; axis < 3; ++axis) {
			const float rhs_p = (1.f - h * l1) * _p_hat(axis) + h * _w_hat(axis)
					    + dt * (known_dynamics(axis) + l1 * momentum(axis));
			const float rhs_w = -h * l2 * _p_hat(axis) + _w_hat(axis) + dt * l2 * momentum(axis);
			const float p_next = (rhs_p + h * rhs_w) / determinant;
			const float w_next = (-h * l2 * rhs_p + (1.f + h * l1) * rhs_w) / determinant;

			if (!PX4_ISFINITE(p_next) || !PX4_ISFINITE(w_next)) {
				return false;
			}

			_p_hat(axis) = p_next;
			_w_hat(axis) = w_next;
		}

		// Momentum estimation error: e_p = p - p_hat.
		_error = momentum - _p_hat;
		return _error.isAllFinite() && _w_hat.isAllFinite();
	}

	/** Return whether reset() has supplied an initial measured momentum. */
	bool initialized() const { return _initialized; }

	/** Return the current external force/torque estimate for the three axes. */
	const matrix::Vector3f &estimate() const { return _w_hat; }

	/** Return measured minus estimated momentum, p - p_hat. */
	const matrix::Vector3f &error() const { return _error; }

private:
	matrix::Vector3f _p_hat{};
	matrix::Vector3f _w_hat{};
	matrix::Vector3f _error{};
	bool _initialized{false};
};

} // namespace palletrone_mob
