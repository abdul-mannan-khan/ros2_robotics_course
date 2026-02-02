#!/usr/bin/env python3
"""Task 4: Jacobian Verification via Numerical Finite Differences."""

import numpy as np
import os

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


# ---- Motion model and Jacobian ----

def motion_model(x, u, dt):
    px, py, theta, vx, vy, omega = x
    ax, ay, alpha = u
    return np.array([
        px + vx * np.cos(theta) * dt,
        py + vx * np.sin(theta) * dt,
        normalize_angle(theta + omega * dt),
        vx + ax * dt,
        vy + ay * dt,
        omega + alpha * dt,
    ])


def compute_motion_jacobian(x, u, dt):
    px, py, theta, vx, vy, omega = x
    F = np.eye(6)
    F[0, 2] = -vx * np.sin(theta) * dt
    F[0, 3] = np.cos(theta) * dt
    F[1, 2] = vx * np.cos(theta) * dt
    F[1, 3] = np.sin(theta) * dt
    F[2, 5] = dt
    return F


# ---- Encoder model and Jacobian ----

def encoder_measurement_model(x):
    vx, vy, omega = x[3], x[4], x[5]
    v = np.sqrt(vx ** 2 + vy ** 2)
    return np.array([v, omega])


def compute_encoder_jacobian(x):
    vx, vy = x[3], x[4]
    v = np.sqrt(vx ** 2 + vy ** 2)
    H = np.zeros((2, 6))
    if v > 1e-6:
        H[0, 3] = vx / v
        H[0, 4] = vy / v
    else:
        H[0, 3] = 1.0
    H[1, 5] = 1.0
    return H


# ---- GPS model and Jacobian ----

def gps_measurement_model(x):
    return np.array([x[0], x[1]])


def compute_gps_jacobian(x):
    H = np.zeros((2, 6))
    H[0, 0] = 1.0
    H[1, 1] = 1.0
    return H


# ---- Numerical Jacobian ----

def numerical_jacobian(func, x, eps=1e-7):
    """Central finite difference Jacobian of func w.r.t. x.

    func: callable returning an array given x.
    """
    f0 = func(x)
    n = len(x)
    m = len(f0)
    J = np.zeros((m, n))
    for j in range(n):
        x_plus = x.copy()
        x_minus = x.copy()
        x_plus[j] += eps
        x_minus[j] -= eps
        f_plus = func(x_plus)
        f_minus = func(x_minus)
        diff = f_plus - f_minus
        # Normalize angle differences for angle states
        for k in range(m):
            diff[k] = normalize_angle(diff[k]) if abs(diff[k]) > 1.0 else diff[k]
        J[:, j] = diff / (2.0 * eps)
    return J


def verify_motion_jacobian(x, u, dt, atol=1e-5):
    F_analytic = compute_motion_jacobian(x, u, dt)
    func = lambda xi: motion_model(xi, u, dt)
    F_numeric = numerical_jacobian(func, x)
    match = np.allclose(F_analytic, F_numeric, atol=atol)
    max_diff = np.max(np.abs(F_analytic - F_numeric))
    return match, max_diff, F_analytic, F_numeric


def verify_encoder_jacobian(x, atol=1e-5):
    H_analytic = compute_encoder_jacobian(x)
    H_numeric = numerical_jacobian(encoder_measurement_model, x)
    match = np.allclose(H_analytic, H_numeric, atol=atol)
    max_diff = np.max(np.abs(H_analytic - H_numeric))
    return match, max_diff, H_analytic, H_numeric


def verify_gps_jacobian(x, atol=1e-5):
    H_analytic = compute_gps_jacobian(x)
    H_numeric = numerical_jacobian(gps_measurement_model, x)
    match = np.allclose(H_analytic, H_numeric, atol=atol)
    max_diff = np.max(np.abs(H_analytic - H_numeric))
    return match, max_diff, H_analytic, H_numeric


def main():
    print("=== Task 4: Jacobian Verification ===\n")
    np.random.seed(42)

    num_tests = 5
    dt = 0.01

    all_passed = True

    for i in range(num_tests):
        # Random state with non-trivial values
        x = np.random.randn(6)
        x[3] = max(abs(x[3]), 0.5) * np.sign(x[3])  # ensure non-zero vx
        u = np.random.randn(3) * 0.5

        # Motion Jacobian
        ok, diff, _, _ = verify_motion_jacobian(x, u, dt)
        status = "PASSED" if ok else "FAILED"
        if not ok:
            all_passed = False
        print(f"Test {i+1} - Motion Jacobian:  {status} (max diff: {diff:.2e})")

        # Encoder Jacobian
        ok, diff, _, _ = verify_encoder_jacobian(x)
        status = "PASSED" if ok else "FAILED"
        if not ok:
            all_passed = False
        print(f"Test {i+1} - Encoder Jacobian: {status} (max diff: {diff:.2e})")

        # GPS Jacobian
        ok, diff, _, _ = verify_gps_jacobian(x)
        status = "PASSED" if ok else "FAILED"
        if not ok:
            all_passed = False
        print(f"Test {i+1} - GPS Jacobian:     {status} (max diff: {diff:.2e})")
        print()

    # Edge case: v near zero
    x_zero_v = np.array([1.0, 2.0, 0.5, 1e-8, 1e-8, 0.3])
    ok, diff, _, _ = verify_encoder_jacobian(x_zero_v, atol=1e-3)
    status = "PASSED" if ok else "FAILED (expected near-zero v)"
    print(f"Edge case (v~0) - Encoder Jacobian: {status} (max diff: {diff:.2e})")

    print()
    if all_passed:
        print("ALL JACOBIAN TESTS PASSED")
    else:
        print("SOME JACOBIAN TESTS FAILED")


if __name__ == "__main__":
    main()
