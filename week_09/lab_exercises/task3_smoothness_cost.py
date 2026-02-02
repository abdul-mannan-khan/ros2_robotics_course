#!/usr/bin/env python3
"""
Task 3: Trajectory Smoothness Optimization
===========================================
Minimize jerk or snap along a B-spline trajectory to produce
smooth drone motions.

Exercises:
1. Compute smoothness cost (integral of squared jerk or snap)
2. Compute gradient of smoothness cost w.r.t. control points
3. Gradient descent optimization
4. Compare jerk vs snap minimization

Saves: task3_smoothness_cost.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bspline_utils import (
    uniform_bspline_acceleration, uniform_bspline_jerk,
    evaluate_uniform_bspline, waypoints_to_uniform_bspline
)


def compute_smoothness_cost(control_points, dt, order=3):
    """
    Compute smoothness cost based on finite differences of control points.

    For order=2 (acceleration): sum ||a_i||^2, where a_i = (Q_{i+2} - 2Q_{i+1} + Q_i)/dt^2
    For order=3 (jerk):         sum ||j_i||^2, where j_i = (Q_{i+3} - 3Q_{i+2} + 3Q_{i+1} - Q_i)/dt^3

    Parameters
    ----------
    control_points : ndarray, shape (n, d)
    dt : float
    order : int (2=acceleration, 3=jerk)

    Returns
    -------
    float
        Smoothness cost.
    """
    # TODO: Compute finite differences of the appropriate order
    # TODO: Return sum of squared norms
    raise NotImplementedError("Implement compute_smoothness_cost")


def compute_smoothness_gradient(control_points, dt, order=3):
    """
    Compute gradient of smoothness cost w.r.t. control points.

    Parameters
    ----------
    control_points : ndarray, shape (n, d)
    dt : float
    order : int

    Returns
    -------
    ndarray, shape (n, d)
        Gradient for each control point.
    """
    # TODO: Analytically derive the gradient
    # HINT: For jerk cost J = sum ||j_i||^2 where j_i depends on Q_i..Q_{i+3},
    #   dJ/dQ_k involves all j_i that depend on Q_k
    # Alternative: use finite differences for verification
    raise NotImplementedError("Implement compute_smoothness_gradient")


def optimize_smoothness(control_points, dt, fixed_endpoints=True, max_iter=200):
    """
    Optimize control points for smoothness using gradient descent.

    Parameters
    ----------
    control_points : ndarray, shape (n, d)
    dt : float
    fixed_endpoints : bool
        If True, keep first 2 and last 2 control points fixed (preserve start/end).
    max_iter : int

    Returns
    -------
    optimized_cp : ndarray
    cost_history : list
    """
    # TODO: Copy control points
    # TODO: For each iteration:
    #   - Compute gradient
    #   - Update free control points (not fixed endpoints)
    #   - Record cost
    # TODO: Return optimized control points and cost history
    raise NotImplementedError("Implement optimize_smoothness")


def compare_smoothness_orders(waypoints):
    """
    Compare jerk minimization vs acceleration minimization.

    Parameters
    ----------
    waypoints : ndarray, shape (m, d)

    Returns
    -------
    cp_jerk : ndarray
    cp_accel : ndarray
    """
    # TODO: Fit B-spline to waypoints
    # TODO: Optimize with order=3 (jerk)
    # TODO: Optimize with order=2 (acceleration)
    # TODO: Return both results
    raise NotImplementedError("Implement compare_smoothness_orders")


def main():
    """Main function: smoothness optimization demo."""
    print("=" * 60)
    print("Task 3: Trajectory Smoothness Optimization")
    print("=" * 60)

    # TODO: Create waypoints and fit B-spline
    # TODO: Optimize for smoothness
    # TODO: Plot before/after trajectories
    # TODO: Plot cost convergence
    # TODO: Compare jerk vs accel minimization
    # TODO: Save as 'task3_smoothness_cost.png'

    print("Task 3 not yet implemented. Complete the TODOs!")


if __name__ == '__main__':
    main()
