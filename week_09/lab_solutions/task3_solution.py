#!/usr/bin/env python3
"""
Task 3 Solution: Trajectory Smoothness Optimization
====================================================
Saves: task3_smoothness_cost.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from bspline_utils import (
    uniform_bspline_acceleration, uniform_bspline_jerk,
    evaluate_uniform_bspline, waypoints_to_uniform_bspline
)


def compute_smoothness_cost(control_points, dt, order=3):
    """Compute smoothness cost (sum of squared finite differences)."""
    cp = np.asarray(control_points, dtype=float)
    if order == 2:
        # Acceleration: a_i = (Q_{i+2} - 2*Q_{i+1} + Q_i) / dt^2
        diffs = (cp[2:] - 2*cp[1:-1] + cp[:-2]) / (dt**2)
    elif order == 3:
        # Jerk: j_i = (Q_{i+3} - 3*Q_{i+2} + 3*Q_{i+1} - Q_i) / dt^3
        diffs = (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / (dt**3)
    else:
        raise ValueError(f"Order {order} not supported")
    return np.sum(diffs**2)


def compute_smoothness_gradient(control_points, dt, order=3):
    """Analytical gradient of smoothness cost w.r.t. control points."""
    cp = np.asarray(control_points, dtype=float)
    n, d = cp.shape
    grad = np.zeros_like(cp)

    if order == 3:
        # Jerk: j_i = (Q_{i+3} - 3Q_{i+2} + 3Q_{i+1} - Q_i)/dt^3
        # dJ/dQ_k = 2/dt^6 * sum_i (dj_i/dQ_k * j_i)
        # j_i depends on Q_i, Q_{i+1}, Q_{i+2}, Q_{i+3}
        # dj_i/dQ_i = -1/dt^3, dj_i/dQ_{i+1} = 3/dt^3,
        # dj_i/dQ_{i+2} = -3/dt^3, dj_i/dQ_{i+3} = 1/dt^3
        jerks = (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / (dt**3)
        n_j = len(jerks)
        for i in range(n_j):
            j = jerks[i] * 2.0 / (dt**3)
            grad[i]     += -j
            grad[i + 1] +=  3*j
            grad[i + 2] += -3*j
            grad[i + 3] +=  j
    elif order == 2:
        accs = (cp[2:] - 2*cp[1:-1] + cp[:-2]) / (dt**2)
        n_a = len(accs)
        for i in range(n_a):
            a = accs[i] * 2.0 / (dt**2)
            grad[i]     += a
            grad[i + 1] += -2*a
            grad[i + 2] += a

    return grad


def optimize_smoothness(control_points, dt, fixed_endpoints=True, max_iter=200,
                        lr=0.001, order=3):
    """Gradient descent for smoothness."""
    cp = control_points.copy()
    n = len(cp)
    cost_history = []

    for it in range(max_iter):
        cost = compute_smoothness_cost(cp, dt, order)
        cost_history.append(cost)
        grad = compute_smoothness_gradient(cp, dt, order)

        if fixed_endpoints:
            grad[:2] = 0.0
            grad[-2:] = 0.0

        cp -= lr * grad

    return cp, cost_history


def compare_smoothness_orders(waypoints):
    cp_init, dt = waypoints_to_uniform_bspline(waypoints, n_control_points=len(waypoints)+6, dt=1.0)
    cp_jerk, hist_jerk = optimize_smoothness(cp_init.copy(), dt, order=3, max_iter=300, lr=0.0005)
    cp_accel, hist_accel = optimize_smoothness(cp_init.copy(), dt, order=2, max_iter=300, lr=0.0005)
    return cp_init, cp_jerk, cp_accel, hist_jerk, hist_accel, dt


def main():
    print("=" * 60)
    print("Task 3 Solution: Trajectory Smoothness Optimization")
    print("=" * 60)

    waypoints = np.array([
        [0, 0], [2, 3], [4, -1], [6, 4], [8, 0], [10, 2]
    ], dtype=float)

    cp_init, cp_jerk, cp_accel, hist_jerk, hist_accel, dt = compare_smoothness_orders(waypoints)

    # Evaluate curves
    n_seg = len(cp_init) - 3
    t_eval = np.linspace(0, n_seg * dt - 1e-10, 500)
    curve_init = evaluate_uniform_bspline(cp_init, dt, t_eval)
    curve_jerk = evaluate_uniform_bspline(cp_jerk, dt, t_eval)
    curve_accel = evaluate_uniform_bspline(cp_accel, dt, t_eval)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Before/after jerk
    ax = axes[0, 0]
    ax.plot(curve_init[:, 0], curve_init[:, 1], 'r--', label='Initial', alpha=0.6)
    ax.plot(curve_jerk[:, 0], curve_jerk[:, 1], 'b-', linewidth=2, label='Jerk-optimized')
    ax.plot(cp_init[:, 0], cp_init[:, 1], 'r.', alpha=0.3)
    ax.plot(cp_jerk[:, 0], cp_jerk[:, 1], 'b.', alpha=0.5)
    ax.plot(waypoints[:, 0], waypoints[:, 1], 'g^', markersize=10, label='Waypoints')
    ax.set_title('Jerk Minimization')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Before/after acceleration
    ax = axes[0, 1]
    ax.plot(curve_init[:, 0], curve_init[:, 1], 'r--', label='Initial', alpha=0.6)
    ax.plot(curve_accel[:, 0], curve_accel[:, 1], 'b-', linewidth=2, label='Accel-optimized')
    ax.plot(waypoints[:, 0], waypoints[:, 1], 'g^', markersize=10, label='Waypoints')
    ax.set_title('Acceleration Minimization')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Cost convergence
    ax = axes[1, 0]
    ax.semilogy(hist_jerk, 'b-', label='Jerk cost')
    ax.semilogy(hist_accel, 'r-', label='Accel cost')
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Cost (log)')
    ax.set_title('Cost Convergence')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Comparison of jerk values
    ax = axes[1, 1]
    # Compute jerk for both
    jerk_init = (cp_init[3:] - 3*cp_init[2:-1] + 3*cp_init[1:-2] - cp_init[:-3]) / dt**3
    jerk_opt = (cp_jerk[3:] - 3*cp_jerk[2:-1] + 3*cp_jerk[1:-2] - cp_jerk[:-3]) / dt**3
    jerk_init_mag = np.linalg.norm(jerk_init, axis=1)
    jerk_opt_mag = np.linalg.norm(jerk_opt, axis=1)
    ax.bar(np.arange(len(jerk_init_mag)) - 0.15, jerk_init_mag, 0.3, label='Initial', alpha=0.7)
    ax.bar(np.arange(len(jerk_opt_mag)) + 0.15, jerk_opt_mag, 0.3, label='Optimized', alpha=0.7)
    ax.set_xlabel('Segment')
    ax.set_ylabel('|Jerk|')
    ax.set_title('Jerk Magnitude per Segment')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('task3_smoothness_cost.png', dpi=150)
    print("Saved: task3_smoothness_cost.png")


if __name__ == '__main__':
    main()
