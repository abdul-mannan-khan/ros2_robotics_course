#!/usr/bin/env python3
"""
Task 1 Solution: B-Spline Basics
=================================
Demonstrates basis functions, curve evaluation, local support, and convex hull.
Saves: task1_bspline_basics.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from bspline_utils import bspline_basis, create_uniform_knots, evaluate_bspline


def compute_basis_functions(degree, knots, t_values):
    n_basis = len(knots) - degree - 1
    result = np.zeros((n_basis, len(t_values)))
    for i in range(n_basis):
        for j, t in enumerate(t_values):
            result[i, j] = bspline_basis(i, degree, t, knots)
    return result


def create_uniform_knots_clamped(n_control_points, degree):
    return create_uniform_knots(n_control_points, degree)


def evaluate_curve(control_points, degree, knots, t_values):
    return evaluate_bspline(control_points, knots, degree, t_values)


def demonstrate_local_support(control_points):
    cp = np.asarray(control_points, dtype=float)
    n = len(cp)
    degree = 3
    knots = create_uniform_knots(n, degree)
    t_values = np.linspace(0, 1 - 1e-10, 500)

    original_curve = evaluate_bspline(cp, knots, degree, t_values)

    modified_cp = cp.copy()
    mid = n // 2
    modified_cp[mid] += np.array([0.0, 1.5])

    modified_curve = evaluate_bspline(modified_cp, knots, degree, t_values)

    return original_curve, modified_curve, modified_cp, mid


def demonstrate_convex_hull(control_points):
    cp = np.asarray(control_points, dtype=float)
    degree = 3
    knots = create_uniform_knots(len(cp), degree)
    t_values = np.linspace(0, 1 - 1e-10, 500)
    curve = evaluate_bspline(cp, knots, degree, t_values)
    hull = ConvexHull(cp)
    return curve, hull


def main():
    print("=" * 60)
    print("Task 1 Solution: B-Spline Basics")
    print("=" * 60)

    fig = plt.figure(figsize=(16, 12))

    # --- Panel 1: Basis functions ---
    ax1 = fig.add_subplot(2, 2, 1)
    n_cp = 7
    degree = 3
    knots = create_uniform_knots(n_cp, degree)
    t_values = np.linspace(0, 1 - 1e-10, 500)
    basis = compute_basis_functions(degree, knots, t_values)
    for i in range(n_cp):
        ax1.plot(t_values, basis[i], label=f'N_{{{i},{degree}}}')
    ax1.set_xlabel('t')
    ax1.set_ylabel('N(t)')
    ax1.set_title(f'Cubic B-Spline Basis Functions (n={n_cp})')
    ax1.legend(fontsize=7, ncol=2)
    ax1.grid(True, alpha=0.3)

    # --- Panel 2: 2D curve with control polygon ---
    ax2 = fig.add_subplot(2, 2, 2)
    cp_2d = np.array([
        [0, 0], [1, 2], [2, -1], [4, 3], [5, 0], [6, 2], [7, 1]
    ], dtype=float)
    knots2 = create_uniform_knots(len(cp_2d), degree)
    curve_2d = evaluate_curve(cp_2d, degree, knots2, t_values)
    ax2.plot(cp_2d[:, 0], cp_2d[:, 1], 'ro--', markersize=8, label='Control Points')
    ax2.plot(curve_2d[:, 0], curve_2d[:, 1], 'b-', linewidth=2, label='B-Spline Curve')
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.set_title('2D Cubic B-Spline with Control Polygon')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')

    # --- Panel 3: Local support ---
    ax3 = fig.add_subplot(2, 2, 3)
    orig, mod, mod_cp, mid_idx = demonstrate_local_support(cp_2d)
    ax3.plot(orig[:, 0], orig[:, 1], 'b-', linewidth=2, label='Original')
    ax3.plot(mod[:, 0], mod[:, 1], 'r--', linewidth=2, label='Modified')
    ax3.plot(cp_2d[:, 0], cp_2d[:, 1], 'bo', markersize=6)
    ax3.plot(mod_cp[:, 0], mod_cp[:, 1], 'rs', markersize=8)
    ax3.annotate('Moved CP', xy=(mod_cp[mid_idx, 0], mod_cp[mid_idx, 1]),
                 fontsize=9, color='red',
                 xytext=(mod_cp[mid_idx, 0]+0.3, mod_cp[mid_idx, 1]+0.3),
                 arrowprops=dict(arrowstyle='->', color='red'))
    ax3.set_title('Local Support Property')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # --- Panel 4: 3D curve ---
    ax4 = fig.add_subplot(2, 2, 4, projection='3d')
    cp_3d = np.array([
        [0, 0, 0], [1, 2, 1], [2, -1, 2], [4, 3, 1.5],
        [5, 0, 3], [6, 2, 2], [7, 1, 0.5]
    ], dtype=float)
    knots3 = create_uniform_knots(len(cp_3d), degree)
    curve_3d = evaluate_bspline(cp_3d, knots3, degree, t_values)
    ax4.plot(cp_3d[:, 0], cp_3d[:, 1], cp_3d[:, 2], 'ro--', markersize=8, label='Control Points')
    ax4.plot(curve_3d[:, 0], curve_3d[:, 1], curve_3d[:, 2], 'b-', linewidth=2, label='B-Spline')
    ax4.set_xlabel('X')
    ax4.set_ylabel('Y')
    ax4.set_zlabel('Z')
    ax4.set_title('3D Cubic B-Spline')
    ax4.legend()

    plt.tight_layout()
    plt.savefig('task1_bspline_basics.png', dpi=150)
    print("Saved: task1_bspline_basics.png")


if __name__ == '__main__':
    main()
