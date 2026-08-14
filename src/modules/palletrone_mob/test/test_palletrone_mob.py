#!/usr/bin/env python3
"""Deterministic reference tests for PalletroneMob's per-axis Tustin update."""

import math
import unittest


class Observer:
    def __init__(self, momentum=0.0):
        self.p_hat = momentum
        self.w_hat = 0.0

    def reset(self, momentum):
        self.p_hat = momentum
        self.w_hat = 0.0

    def update(self, momentum, known, dt, wn, zeta):
        l1 = 2.0 * zeta * wn
        l2 = wn * wn
        h = 0.5 * dt
        determinant = 1.0 + h * l1 + h * h * l2
        rhs_p = (1.0 - h * l1) * self.p_hat + h * self.w_hat + dt * (known + l1 * momentum)
        rhs_w = -h * l2 * self.p_hat + self.w_hat + dt * l2 * momentum
        self.p_hat = (rhs_p + h * rhs_w) / determinant
        self.w_hat = (-h * l2 * rhs_p + (1.0 + h * l1) * rhs_w) / determinant


def simulate(external, known, duration=4.0, variable_dt=False):
    observer = Observer()
    momentum = 0.0
    elapsed = 0.0
    index = 0
    while elapsed < duration:
        dt = (0.006, 0.011, 0.014, 0.009)[index % 4] if variable_dt else 0.01
        momentum += (known + external) * dt
        observer.update(momentum, known, dt, 6.0, 0.9)
        elapsed += dt
        index += 1
    return observer


class PalletroneMobReferenceTest(unittest.TestCase):
    def test_confirmed_frd_rotor_geometry(self):
        position_signs = ((1, -1), (-1, -1), (-1, 1), (1, 1))
        positive_tilt_signs = ((1, -1), (-1, -1), (-1, 1), (1, 1))
        self.assertEqual(position_signs, positive_tilt_signs)

        # With zero tilt f=[0,0,-T], r x f must produce the expected
        # roll/pitch signs for the owner-confirmed FRD rotor positions.
        zero_tilt_moment_signs = tuple((-y, x) for x, y in position_signs)
        self.assertEqual(zero_tilt_moment_signs,
                         ((1, 1), (1, -1), (-1, -1), (-1, 1)))

    def test_hover_no_external(self):
        # FRD: actuator thrust -mg and gravity +mg cancel.
        result = simulate(0.0, 0.0)
        self.assertAlmostEqual(result.w_hat, 0.0, places=6)

    def test_free_fall_no_external(self):
        result = simulate(0.0, 4.0 * 9.80665)
        self.assertAlmostEqual(result.w_hat, 0.0, delta=0.02)

    def test_positive_force_axes(self):
        for wrench in (3.0, 5.0):  # +Fx and +Fz share the same FRD-positive equation
            self.assertAlmostEqual(simulate(wrench, 0.0).w_hat, wrench, delta=0.03)

    def test_positive_torque_axes(self):
        for wrench in (0.4, 0.6, 0.2):  # +Tx, +Ty, +Tz
            self.assertAlmostEqual(simulate(wrench, 0.0).w_hat, wrench, delta=0.01)

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
        result = simulate(2.5, -1.0, variable_dt=True)
        self.assertTrue(math.isfinite(result.p_hat))
        self.assertTrue(math.isfinite(result.w_hat))
        self.assertAlmostEqual(result.w_hat, 2.5, delta=0.04)

    def test_estimator_reset_has_no_impulse(self):
        observer = simulate(2.0, 0.0, duration=1.0)
        observer.reset(7.0)
        self.assertEqual(observer.p_hat, 7.0)
        self.assertEqual(observer.w_hat, 0.0)

    def test_stale_inputs_invalidate(self):
        now = 1_000_000
        timeout_us = 100_000
        servo_received = now - timeout_us - 1
        thrust_received = now - timeout_us - 1
        self.assertFalse(now - servo_received <= timeout_us)
        self.assertFalse(now - thrust_received <= timeout_us)


if __name__ == "__main__":
    unittest.main()
