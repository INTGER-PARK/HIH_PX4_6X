#pragma once

#include <cmath>
#include <cstdint>

namespace palletrone
{

class Admittance4D
{
public:
	static constexpr unsigned AXES = 4;
	struct AxisParams { float mass; float damping; float stiffness; };
	struct State { float q; float dq; float ddq; };

	void reset()
	{
		for (auto &s : _state) { s = {}; }
	}

	const State &state(unsigned axis) const { return _state[axis]; }
	void setState(unsigned axis, const State &state) { if (axis < AXES) { _state[axis] = state; } }

	bool update(float dt, const float input[AXES], const AxisParams params[AXES], float dt_min = 0.001f,
		    float dt_max = 0.05f)
	{
		if (!std::isfinite(dt) || dt < dt_min || dt > dt_max) { return false; }
		State next[AXES]{};

		for (unsigned i = 0; i < AXES; ++i) {
			const AxisParams &p = params[i];
			const State &s = _state[i];
			if (!finite(p.mass) || !finite(p.damping) || !finite(p.stiffness) || !finite(input[i]) ||
			    !finite(s.q) || !finite(s.dq) || !finite(s.ddq) || p.mass <= 0.f || p.damping <= 0.f ||
			    p.stiffness < 0.f) { return false; }

			// Trapezoidal integration of q_dot=dq and dq_dot=c-a*dq-b*q:
			// (I-hA)x[k+1]=(I+hA)x[k]+dt*B*u, h=dt/2.
			const float h = 0.5f * dt;
			const float a = p.damping / p.mass;
			const float b = p.stiffness / p.mass;
			const float c = input[i] / p.mass;
			const float det = 1.f + h * a + h * h * b;
			if (!finite(det) || det <= 1e-7f) { return false; }
			const float rq = s.q + h * s.dq;
			const float rv = -h * b * s.q + (1.f - h * a) * s.dq + dt * c;
			next[i].q = ((1.f + h * a) * rq + h * rv) / det;
			next[i].dq = (-h * b * rq + rv) / det;
			next[i].ddq = (input[i] - p.damping * next[i].dq - p.stiffness * next[i].q) / p.mass;
			if (!finite(next[i].q) || !finite(next[i].dq) || !finite(next[i].ddq)) { return false; }
		}

		for (unsigned i = 0; i < AXES; ++i) { _state[i] = next[i]; } // atomic four-axis commit
		return true;
	}

	static float deadzone(float value, float epsilon)
	{
		if (!finite(value) || !finite(epsilon) || epsilon < 0.f) { return NAN; }
		if (value > epsilon) { return value - epsilon; }
		if (value < -epsilon) { return value + epsilon; }
		return 0.f;
	}

private:
	static bool finite(float value) { return std::isfinite(value); }
	State _state[AXES]{};
};

} // namespace palletrone
