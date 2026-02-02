#!/usr/bin/env python3
"""
Task 2 Solution: B-Spline Trajectory Representation
====================================================
Saves: task2_trajectory_representation.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from bspline_utils import (
    waypoints_to_uniform_bspline, evaluate_uniform_bspline,
    uniform_bspline_velocity, uniform_bspline_acceleration,
    evaluate_uniform_bspline_derivative
)


def waypoints_to_bspline(waypoints, n_segments=None):
    waypoints = np.asarray(waypoints, dtype=float)
    m = len(waypoints)
    if n_segments is None:
        n_segments = m + 3
    n_cp = n_segments + 3
    control_points, dt = waypoints_to_uniform_bspline(waypoints, n_control_points=n_cp, dt=1.0)
    return control_points, dt


def compute_velocity_profile(control_points, dt):
    cp = np.asarray(control_points, dtype=float)
    n_seg = len(cp) - 3
    t_max = n_seg * dt
    t_eval = np.linspace(0, t_max - 1e-10, 300)
    velocities = evaluate_uniform_bspline_derivative(cp, dt, t_eval, order=1)
    speeds = np.linalg.norm(velocities, axis=1)
    return t_eval, speeds, velocities


def compute_acceleration_profile(control_points, dt):
    cp = np.asarray(control_points, dtype=float)
    n_seg = len(cp) - 3
    t_max = n_seg * dt
    # Acceleration B-spline has n-2 CPs and degree 1 => n-3 segments
    t_eval = np.linspace(0, max((len(cp) - 2 - 1) * dt - 1e-10, 0.01), 300)
    accelerations = evaluate_uniform_bspline_derivative(cp, dt, t_eval, order=2)
    acc_mag = np.linalg.norm(accelerations, axis=1)
    return t_eval, acc_mag, accelerations


def check_dynamic_feasibility(control_points, dt, v_max=2.0, a_max=4.0):
    cp = np.asarray(control_points, dtype=float)
    vel_cp = uniform_bspline_velocity(cp, dt)
    acc_cp = uniform_bspline_acceleration(cp, dt)
    max_v = np.max(np.linalg.norm(vel_cp, axis=1))
    max_a = np.max(np.linalg.norm(acc_cp, axis=1))
    feasible = (max_v <= v_max) and (max_a <= a_max)
    return feasible, max_v, max_a


def time_parameterize(control_points, v_max=2.0, a_max=4.0):
    cp = np.asarray(control_points, dtype=float)
    diffs1 = np.linalg.norm(np.diff(cp, axis=0), axis=1)
    dt_v = np.max(diffs1) / v_max if np.max(diffs1) > 0 else 0.1

    diffs2 = np.linalg.norm(cp[2:] - 2*cp[1:-1] + cp[:-2], axis=1)
    dt_a = np.sqrt(np.max(diffs2) / a_max) if np.max(diffs2) > 0 else 0.1

    return max(dt_v, dt_a, 0.1)


def main():
    print("=" * 60)
    print("Task 2 Solution: B-Spline Trajectory Representation")
    print("=" * 60)

    # 3D waypoints (drone flight)
    waypoints = np.array([
        [0, 0, 1],
        [2, 3, 2],
        [5, 2, 3],
        [7, 5, 2],
        [10, 4, 1.5],
        [12, 1, 2.5]
    ], dtype=float)

    cp, dt_init = waypoints_to_bspline(waypoints)
    print(f"Control points: {len(cp)}, initial dt: {dt_init:.2f}")

    # Check feasibility with initial dt
    v_max, a_max = 3.0, 5.0
    feasible, max_v, max_a = check_dynamic_feasibility(cp, dt_init, v_max, a_max)
    print(f"Initial: feasible={feasible}, max_v={max_v:.2f}, max_a={max_a:.2f}")

    if not feasible:
        dt = time_parameterize(cp, v_max, a_max)
        print(f"Adjusted dt: {dt:.3f}")
    else:
        dt = dt_init

    feasible2, max_v2, max_a2 = check_dynamic_feasibility(cp, dt, v_max, a_max)
    print(f"After adjustment: feasible={feasible2}, max_v={max_v2:.2f}, max_a={max_a2:.2f}")

    # Evaluate trajectory
    n_seg = len(cp) - 3
    t_traj = np.linspace(0, n_seg * dt - 1e-10, 500)
    traj = evaluate_uniform_bspline(cp, dt, t_traj)

    t_vel, speeds, velocities = compute_velocity_profile(cp, dt)
    t_acc, acc_mag, accelerations = compute_acceleration_profile(cp, dt)

    # Plot
    fig = plt.figure(figsize=(16, 12))

    # 3D trajectory
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'b-', linewidth=2, label='Trajectory')
    ax1.plot(cp[:, 0], cp[:, 1], cp[:, 2], 'r.--', markersize=6, alpha=0.5, label='Control Points')
    ax1.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], 'g^', markersize=10, label='Waypoints')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D Drone Trajectory')
    ax1.legend()

    # Position vs time
    ax2 = fig.add_subplot(2, 2, 2)
    labels = ['x', 'y', 'z']
    for i in range(3):
        ax2.plot(t_traj, traj[:, i], label=labels[i])
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Position [m]')
    ax2.set_title('Position Profiles')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Velocity
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(t_vel, speeds, 'b-', linewidth=2, label='Speed')
    ax3.axhline(y=v_max, color='r', linestyle='--', label=f'v_max={v_max}')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Speed [m/s]')
    ax3.set_title('Velocity Profile')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Acceleration
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(t_acc, acc_mag, 'b-', linewidth=2, label='|a|')
    ax4.axhline(y=a_max, color='r', linestyle='--', label=f'a_max={a_max}')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Acceleration [m/s^2]')
    ax4.set_title('Acceleration Profile')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('task2_trajectory_representation.png', dpi=150)
    print("Saved: task2_trajectory_representation.png")


if __name__ == '__main__':
    main()
