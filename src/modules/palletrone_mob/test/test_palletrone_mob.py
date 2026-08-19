#!/usr/bin/env python3
"""Deterministic reference tests for PalletroneMob's per-axis Tustin update."""

import math
import unittest


DEFAULT_FORCE_WN = (6.0, 6.0, 6.0)
DEFAULT_FORCE_ZETA = (0.9, 0.9, 0.9)
DEFAULT_TORQUE_WN = (10.0, 10.0, 10.0)
DEFAULT_TORQUE_ZETA = (0.9, 0.9, 0.9)
STEP_TOLERANCE = 0.05


def scalar_step(p_hat, w_hat, momentum, known, dt, wn, zeta):
    l1 = 2.0 * zeta * wn
    l2 = wn * wn
    h = 0.5 * dt
    determinant = 1.0 + h * l1 + h * h * l2
    rhs_p = (1.0 - h * l1) * p_hat + h * w_hat + dt * (known + l1 * momentum)
    rhs_w = -h * l2 * p_hat + w_hat + dt * l2 * momentum
    next_p = (rhs_p + h * rhs_w) / determinant
    next_w = (-h * l2 * rhs_p + (1.0 + h * l1) * rhs_w) / determinant
    return next_p, next_w, momentum - next_p


class Observer3D:
    def __init__(self):
        self.p_hat = [0.0, 0.0, 0.0]
        self.w_hat = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]
        self.initialized = False

    def reset(self, momentum):
        self.p_hat = list(momentum)
        self.w_hat = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]
        self.initialized = True

    def invalidate(self):
        self.p_hat = [0.0, 0.0, 0.0]
        self.w_hat = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]
        self.initialized = False

    def update(self, momentum, known, dt, wn, zeta):
        if (not self.initialized or dt <= 0.0 or not math.isfinite(dt)
                or not all(math.isfinite(value) for value in momentum + known + wn + zeta)):
            return False

        h = 0.5 * dt
        next_p_hat = [0.0, 0.0, 0.0]
        next_w_hat = [0.0, 0.0, 0.0]
        next_error = [0.0, 0.0, 0.0]

        for axis in range(3):
            if wn[axis] <= 0.0 or zeta[axis] <= 0.0:
                return False

            l1 = 2.0 * zeta[axis] * wn[axis]
            l2 = wn[axis] * wn[axis]
            determinant = 1.0 + h * l1 + h * h * l2

            if not math.isfinite(l1) or not math.isfinite(l2) or not math.isfinite(determinant) or determinant <= 0.0:
                return False

            rhs_p = ((1.0 - h * l1) * self.p_hat[axis] + h * self.w_hat[axis]
                     + dt * (known[axis] + l1 * momentum[axis]))
            rhs_w = -h * l2 * self.p_hat[axis] + self.w_hat[axis] + dt * l2 * momentum[axis]
            p_next = (rhs_p + h * rhs_w) / determinant
            w_next = (-h * l2 * rhs_p + (1.0 + h * l1) * rhs_w) / determinant

            if not math.isfinite(p_next) or not math.isfinite(w_next):
                return False

            next_p_hat[axis] = p_next
            next_w_hat[axis] = w_next
            next_error[axis] = momentum[axis] - p_next

        self.p_hat = next_p_hat
        self.w_hat = next_w_hat
        self.error = next_error
        return True


def simulate_vector(external, known, wn, zeta, duration=4.0, variable_dt=False):
    observer = Observer3D()
    momentum = [0.0, 0.0, 0.0]
    time_series = []
    elapsed = 0.0
    index = 0
    observer.reset(momentum)

    while elapsed < duration:
        dt = (0.006, 0.011, 0.014, 0.009)[index % 4] if variable_dt else 0.01
        for axis in range(3):
            momentum[axis] += (known[axis] + external[axis]) * dt
        assert observer.update(tuple(momentum), known, dt, wn, zeta)
        elapsed += dt
        index += 1
        time_series.append((elapsed, tuple(observer.p_hat), tuple(observer.w_hat), tuple(observer.error)))

    return observer, time_series


def simulate_scalar(external, known, wn, zeta, duration=4.0, variable_dt=False):
    p_hat = 0.0
    w_hat = 0.0
    momentum = 0.0
    time_series = []
    elapsed = 0.0
    index = 0

    while elapsed < duration:
        dt = (0.006, 0.011, 0.014, 0.009)[index % 4] if variable_dt else 0.01
        momentum += (known + external) * dt
        p_hat, w_hat, error = scalar_step(p_hat, w_hat, momentum, known, dt, wn, zeta)
        elapsed += dt
        index += 1
        time_series.append((elapsed, p_hat, w_hat, error))

    return (p_hat, w_hat, error), time_series


def time_to_threshold(time_series, axis, threshold):
    for elapsed, _p_hat, w_hat, _error in time_series:
        if w_hat[axis] >= threshold:
            return elapsed
    return float("inf")


def max_response(time_series, axis):
    return max(w_hat[axis] for _elapsed, _p_hat, w_hat, _error in time_series)


def correct_wrench(raw, bias):
    if not all(math.isfinite(value) for value in bias):
        raise ValueError("MOB output bias must be finite")
    return tuple(raw_value - bias_value for raw_value, bias_value in zip(raw, bias))


class PalletroneMobReferenceTest(unittest.TestCase):
    def test_zero_bias_preserves_raw_output(self):
        raw = (2.0, -1.0, 8.0)
        self.assertEqual(correct_wrench(raw, (0.0, 0.0, 0.0)), raw)

    def test_force_bias_examples(self):
        corrected_fx = correct_wrench((-3.0, 0.0, 0.0), (-1.0, 0.0, 0.0))[0]
        self.assertEqual(corrected_fx, -2.0)
        self.assertEqual(max(0.0, -corrected_fx), 2.0)
        self.assertEqual(correct_wrench((0.0, 0.0, 4.0), (0.0, 0.0, 1.5))[2], 2.5)

    def test_torque_bias_example(self):
        corrected = correct_wrench((0.0, 0.0, -0.4), (0.0, 0.0, -0.1))
        self.assertAlmostEqual(corrected[2], -0.3)

    def test_runtime_bias_update_changes_only_corrected_output(self):
        observer, _ = simulate_vector((2.0, 1.0, -3.0), (0.0, 0.0, 0.0),
                                      DEFAULT_FORCE_WN, DEFAULT_FORCE_ZETA, duration=1.0)
        raw_before = tuple(observer.w_hat)
        reset_count = 1
        corrected_before = correct_wrench(raw_before, (0.0, 0.0, 0.0))
        corrected_after = correct_wrench(raw_before, (0.5, -0.25, 1.0))
        self.assertEqual(tuple(observer.w_hat), raw_before)
        self.assertEqual(reset_count, 1)
        self.assertEqual(corrected_before, raw_before)
        self.assertNotEqual(corrected_after, corrected_before)

    def test_nonfinite_bias_is_rejected(self):
        for invalid in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                correct_wrench((1.0, 2.0, 3.0), (0.0, invalid, 0.0))

    def test_confirmed_frd_rotor_geometry(self):
        position_signs = ((1, -1), (-1, -1), (-1, 1), (1, 1))
        positive_tilt_signs = ((1, 1), (1, -1), (-1, -1), (-1, 1))
        self.assertEqual(positive_tilt_signs,
                         tuple((-y, x) for x, y in position_signs))

        zero_tilt_moment_signs = tuple((-y, x) for x, y in position_signs)
        self.assertEqual(zero_tilt_moment_signs,
                         ((1, 1), (1, -1), (-1, -1), (-1, 1)))

    def test_hover_no_external(self):
        result, _ = simulate_vector((0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                    DEFAULT_FORCE_WN, DEFAULT_FORCE_ZETA)
        self.assertAlmostEqual(result.w_hat[0], 0.0, places=6)
        self.assertAlmostEqual(result.w_hat[1], 0.0, places=6)
        self.assertAlmostEqual(result.w_hat[2], 0.0, places=6)

    def test_free_fall_no_external(self):
        result, _ = simulate_vector((0.0, 0.0, 0.0), (4.0 * 9.80665, 4.0 * 9.80665, 4.0 * 9.80665),
                                    DEFAULT_FORCE_WN, DEFAULT_FORCE_ZETA)
        for axis in range(3):
            self.assertAlmostEqual(result.w_hat[axis], 0.0, delta=0.02)

    def test_positive_force_axes(self):
        external = (3.0, 5.0, 4.0)
        result, _ = simulate_vector(external, (0.0, 0.0, 0.0), DEFAULT_FORCE_WN, DEFAULT_FORCE_ZETA)
        for axis, target in enumerate(external):
            self.assertAlmostEqual(result.w_hat[axis], target, delta=0.03)

    def test_positive_torque_axes(self):
        external = (0.4, 0.6, 0.2)
        result, _ = simulate_vector(external, (0.0, 0.0, 0.0), DEFAULT_TORQUE_WN, DEFAULT_TORQUE_ZETA)
        for axis, target in enumerate(external):
            self.assertAlmostEqual(result.w_hat[axis], target, delta=0.01)

    def test_gyroscopic_sign(self):
        omega = (2.0, -3.0, 4.0)
        angular_momentum = (0.0192 * omega[0], 0.02125 * omega[1], 0.0282 * omega[2])
        cross = (
            omega[1] * angular_momentum[2] - omega[2] * angular_momentum[1],
            omega[2] * angular_momentum[0] - omega[0] * angular_momentum[2],
            omega[0] * angular_momentum[1] - omega[1] * angular_momentum[0],
        )
        known_angular = tuple(-value for value in cross)
        self.assertEqual(math.copysign(1, known_angular[0]), 1)
        self.assertEqual(math.copysign(1, known_angular[1]), 1)
        self.assertEqual(math.copysign(1, known_angular[2]), 1)

    def test_translational_coriolis_sign(self):
        omega = (0.0, 0.0, 2.0)
        velocity = (3.0, 0.0, 0.0)
        cross = (
            omega[1] * velocity[2] - omega[2] * velocity[1],
            omega[2] * velocity[0] - omega[0] * velocity[2],
            omega[0] * velocity[1] - omega[1] * velocity[0],
        )
        known_linear = tuple(-4.0 * value for value in cross)
        self.assertEqual(known_linear, (0.0, -24.0, 0.0))

    def test_variable_dt_is_finite_and_stable(self):
        result, _ = simulate_vector((2.5, 2.5, 2.5), (-1.0, -1.0, -1.0),
                                    DEFAULT_FORCE_WN, DEFAULT_FORCE_ZETA, variable_dt=True)
        for axis in range(3):
            self.assertTrue(math.isfinite(result.p_hat[axis]))
            self.assertTrue(math.isfinite(result.w_hat[axis]))
            self.assertAlmostEqual(result.w_hat[axis], 2.5, delta=0.04)

    def test_estimator_reset_has_no_impulse(self):
        observer, _ = simulate_vector((2.0, 2.0, 2.0), (0.0, 0.0, 0.0),
                                      DEFAULT_FORCE_WN, DEFAULT_FORCE_ZETA, duration=1.0)
        observer.reset((7.0, 7.0, 7.0))
        self.assertEqual(observer.p_hat, [7.0, 7.0, 7.0])
        self.assertEqual(observer.w_hat, [0.0, 0.0, 0.0])
        self.assertEqual(observer.error, [0.0, 0.0, 0.0])

    def test_stale_inputs_invalidate(self):
        now = 1_000_000
        timeout_us = 100_000
        servo_received = now - timeout_us - 1
        thrust_received = now - timeout_us - 1
        self.assertFalse(now - servo_received <= timeout_us)
        self.assertFalse(now - thrust_received <= timeout_us)

    def test_equal_gain_regression_matches_scalar_reference(self):
        external = (2.0, 2.0, 2.0)
        known = (-0.4, -0.4, -0.4)
        vector_result, vector_series = simulate_vector(external, known, DEFAULT_FORCE_WN, DEFAULT_FORCE_ZETA)
        scalar_result, scalar_series = simulate_scalar(2.0, -0.4, 6.0, 0.9)

        for axis in range(3):
            self.assertAlmostEqual(vector_result.p_hat[axis], scalar_result[0], places=6)
            self.assertAlmostEqual(vector_result.w_hat[axis], scalar_result[1], places=6)
            self.assertAlmostEqual(vector_result.error[axis], scalar_result[2], places=6)

        for step_index, scalar_step_state in enumerate(scalar_series):
            elapsed, p_hat, w_hat, error = scalar_step_state
            self.assertAlmostEqual(vector_series[step_index][0], elapsed, places=9)

            for axis in range(3):
                self.assertAlmostEqual(vector_series[step_index][1][axis], p_hat, places=6)
                self.assertAlmostEqual(vector_series[step_index][2][axis], w_hat, places=6)
                self.assertAlmostEqual(vector_series[step_index][3][axis], error, places=6)

        vector_variable, vector_variable_series = simulate_vector(external, known, DEFAULT_FORCE_WN,
                                                                  DEFAULT_FORCE_ZETA, variable_dt=True)
        scalar_variable, scalar_variable_series = simulate_scalar(2.0, -0.4, 6.0, 0.9, variable_dt=True)

        for axis in range(3):
            self.assertAlmostEqual(vector_variable.p_hat[axis], scalar_variable[0], places=6)
            self.assertAlmostEqual(vector_variable.w_hat[axis], scalar_variable[1], places=6)
            self.assertAlmostEqual(vector_variable.error[axis], scalar_variable[2], places=6)

        for step_index, scalar_step_state in enumerate(scalar_variable_series):
            elapsed, p_hat, w_hat, error = scalar_step_state
            self.assertAlmostEqual(vector_variable_series[step_index][0], elapsed, places=9)

            for axis in range(3):
                self.assertAlmostEqual(vector_variable_series[step_index][1][axis], p_hat, places=6)
                self.assertAlmostEqual(vector_variable_series[step_index][2][axis], w_hat, places=6)
                self.assertAlmostEqual(vector_variable_series[step_index][3][axis], error, places=6)

    def test_force_axis_specific_natural_frequency(self):
        external = (4.0, 4.0, 4.0)
        _, series = simulate_vector(external, (0.0, 0.0, 0.0), (3.0, 6.0, 12.0), (0.9, 0.9, 0.9))
        threshold = 0.9 * external[0]
        rise_x = time_to_threshold(series, 0, threshold)
        rise_y = time_to_threshold(series, 1, threshold)
        rise_z = time_to_threshold(series, 2, threshold)

        self.assertLess(rise_z, rise_y)
        self.assertLess(rise_y, rise_x)

        final = series[-1][2]
        for axis in range(3):
            self.assertAlmostEqual(final[axis], external[axis], delta=STEP_TOLERANCE)

    def test_torque_axis_specific_natural_frequency(self):
        external = (0.5, 0.5, 0.5)
        _, series = simulate_vector(external, (0.0, 0.0, 0.0), (4.0, 8.0, 16.0), (0.9, 0.9, 0.9))
        threshold = 0.9 * external[0]
        rise_x = time_to_threshold(series, 0, threshold)
        rise_y = time_to_threshold(series, 1, threshold)
        rise_z = time_to_threshold(series, 2, threshold)

        self.assertLess(rise_z, rise_y)
        self.assertLess(rise_y, rise_x)

        final = series[-1][2]
        for axis in range(3):
            self.assertAlmostEqual(final[axis], external[axis], delta=0.01)

    def test_axis_isolation(self):
        external = (3.0, 3.0, 3.0)
        _, baseline_force = simulate_vector(external, (0.0, 0.0, 0.0), (6.0, 6.0, 6.0), (0.9, 0.9, 0.9),
                                            duration=1.5)
        _, modified_force = simulate_vector(external, (0.0, 0.0, 0.0), (12.0, 6.0, 6.0), (0.9, 0.9, 0.9),
                                            duration=1.5)
        self.assertNotAlmostEqual(modified_force[-1][2][0], baseline_force[-1][2][0], places=4)
        self.assertAlmostEqual(modified_force[-1][2][1], baseline_force[-1][2][1], places=6)
        self.assertAlmostEqual(modified_force[-1][2][2], baseline_force[-1][2][2], places=6)

        _, baseline_torque = simulate_vector((0.4, 0.4, 0.4), (0.0, 0.0, 0.0), (10.0, 10.0, 10.0), (0.9, 0.9, 0.9),
                                             duration=0.25)
        _, modified_torque = simulate_vector((0.4, 0.4, 0.4), (0.0, 0.0, 0.0), (10.0, 14.0, 10.0), (0.9, 0.7, 0.9),
                                             duration=0.25)
        self.assertAlmostEqual(modified_torque[-1][2][0], baseline_torque[-1][2][0], places=6)
        self.assertGreater(abs(modified_torque[-1][2][1] - baseline_torque[-1][2][1]), 1e-3)
        self.assertAlmostEqual(modified_torque[-1][2][2], baseline_torque[-1][2][2], places=6)

    def test_damping_ratio_changes_overshoot(self):
        external = (4.0, 4.0, 4.0)
        _, series = simulate_vector(external, (0.0, 0.0, 0.0), (8.0, 8.0, 8.0), (0.6, 0.9, 1.2), duration=3.0)
        overshoot = tuple(max_response(series, axis) - external[axis] for axis in range(3))

        self.assertGreater(overshoot[0], overshoot[1] - 1e-4)
        self.assertGreater(overshoot[1], overshoot[2] - 1e-4)

        final = series[-1][2]
        for axis in range(3):
            self.assertAlmostEqual(final[axis], external[axis], delta=STEP_TOLERANCE)

    def test_invalid_gain_rejects_without_partial_state_update(self):
        invalid_cases = (
            ((0.0, 6.0, 6.0), (0.9, 0.9, 0.9)),
            ((-1.0, 6.0, 6.0), (0.9, 0.9, 0.9)),
            ((6.0, 6.0, 6.0), (0.0, 0.9, 0.9)),
            ((6.0, 6.0, 6.0), (-0.1, 0.9, 0.9)),
            ((float("nan"), 6.0, 6.0), (0.9, 0.9, 0.9)),
            ((6.0, 6.0, 6.0), (float("inf"), 0.9, 0.9)),
        )

        for wn, zeta in invalid_cases:
            observer = Observer3D()
            observer.reset((1.0, 2.0, 3.0))
            self.assertTrue(observer.update((1.2, 2.2, 3.2), (0.1, 0.2, 0.3), 0.01,
                                            (6.0, 6.0, 6.0), (0.9, 0.9, 0.9)))
            state_before = (tuple(observer.p_hat), tuple(observer.w_hat), tuple(observer.error), observer.initialized)
            self.assertFalse(observer.update((1.3, 2.3, 3.3), (0.1, 0.2, 0.3), 0.01, wn, zeta))
            self.assertEqual(state_before,
                             (tuple(observer.p_hat), tuple(observer.w_hat), tuple(observer.error), observer.initialized))

    def test_equal_gain_symmetry(self):
        result, series = simulate_vector((2.5, 2.5, 2.5), (-0.2, -0.2, -0.2),
                                         DEFAULT_FORCE_WN, DEFAULT_FORCE_ZETA, duration=2.0)
        self.assertEqual(result.p_hat[0], result.p_hat[1])
        self.assertEqual(result.p_hat[1], result.p_hat[2])
        self.assertEqual(result.w_hat[0], result.w_hat[1])
        self.assertEqual(result.w_hat[1], result.w_hat[2])
        self.assertEqual(result.error[0], result.error[1])
        self.assertEqual(result.error[1], result.error[2])

        for _elapsed, p_hat, w_hat, error in series:
            self.assertEqual(p_hat[0], p_hat[1])
            self.assertEqual(p_hat[1], p_hat[2])
            self.assertEqual(w_hat[0], w_hat[1])
            self.assertEqual(w_hat[1], w_hat[2])
            self.assertEqual(error[0], error[1])
            self.assertEqual(error[1], error[2])


if __name__ == "__main__":
    unittest.main()
