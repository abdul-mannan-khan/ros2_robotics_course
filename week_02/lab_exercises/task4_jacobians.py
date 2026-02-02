#!/usr/bin/env python3
"""
Task 4: Jacobian Verification

Verify your analytic Jacobians (from Tasks 1-3) against numerical Jacobians
computed via finite differences. This is a critical debugging technique for
any EKF implementation.

State vector: x = [px, py, theta, vx, vy, omega]^T
"""

import numpy as np
import os
import sys


def numerical_jacobian(func, x, eps=1e-7):
    """
    Compute the Jacobian of func at x using central finite differences.

    J[i, j] = (f_i(x + eps*e_j) - f_i(x - eps*e_j)) / (2 * eps)

    Parameters
    ----------
    func : callable
        Function mapping np.ndarray of shape (n,) to np.ndarray of shape (m,).
    x : np.ndarray, shape (n,)
        Point at which to evaluate the Jacobian.
    eps : float, optional
        Perturbation size for finite differences.

    Returns
    -------
    np.ndarray, shape (m, n)
        Numerically computed Jacobian matrix.
    """
    raise NotImplementedError("TODO: Implement numerical Jacobian via finite differences")


def verify_motion_jacobian(state, dt, tol=1e-5):
    """
    Compare the analytic motion Jacobian against the numerical Jacobian.

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        State at which to verify.
    dt : float
        Time step.
    tol : float, optional
        Tolerance for element-wise comparison.

    Returns
    -------
    bool
        True if analytic and numerical Jacobians agree within tolerance.
    """
    raise NotImplementedError(
        "TODO: Compare compute_motion_jacobian() vs numerical_jacobian()"
    )


def verify_encoder_jacobian(state, tol=1e-5):
    """
    Compare the analytic encoder Jacobian against the numerical Jacobian.

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        State at which to verify.
    tol : float, optional
        Tolerance for element-wise comparison.

    Returns
    -------
    bool
        True if analytic and numerical Jacobians agree within tolerance.
    """
    raise NotImplementedError(
        "TODO: Compare compute_encoder_jacobian() vs numerical_jacobian()"
    )


def verify_gps_jacobian(state, tol=1e-5):
    """
    Compare the analytic GPS Jacobian against the numerical Jacobian.

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        State at which to verify.
    tol : float, optional
        Tolerance for element-wise comparison.

    Returns
    -------
    bool
        True if analytic and numerical Jacobians agree within tolerance.
    """
    raise NotImplementedError(
        "TODO: Compare compute_gps_jacobian() vs numerical_jacobian()"
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Task 4: Jacobian Verification")
    print("=" * 60)

    print("\nTODO: Import Jacobian functions from tasks 1-3")
    print("TODO: Test at multiple state values (zeros, random, edge cases)")
    print("TODO: Print pass/fail for each Jacobian verification")
    print("TODO: Print max absolute difference when a test fails")

    # ---- Starter scaffold (replace with your implementation) ----
    # from task1_prediction import compute_motion_jacobian
    # from task2_encoder_update import compute_encoder_jacobian
    # from task3_gps_update import compute_gps_jacobian
    #
    # test_states = [
    #     np.zeros(6),
    #     np.array([1.0, 2.0, np.pi / 4, 0.5, -0.3, 0.1]),
    #     np.random.randn(6),
    # ]
    # dt = 0.02
    #
    # print("\n--- Motion Jacobian ---")
    # for i, s in enumerate(test_states):
    #     result = verify_motion_jacobian(s, dt)
    #     status = "PASS" if result else "FAIL"
    #     print(f"  Test state {i}: {status}")
    #
    # print("\n--- Encoder Jacobian ---")
    # for i, s in enumerate(test_states):
    #     result = verify_encoder_jacobian(s)
    #     status = "PASS" if result else "FAIL"
    #     print(f"  Test state {i}: {status}")
    #
    # print("\n--- GPS Jacobian ---")
    # for i, s in enumerate(test_states):
    #     result = verify_gps_jacobian(s)
    #     status = "PASS" if result else "FAIL"
    #     print(f"  Test state {i}: {status}")

    print("\nImplement the functions above and uncomment the scaffold to run.")
