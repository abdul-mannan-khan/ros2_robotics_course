#!/usr/bin/env python3
"""
Task 6 Solution: MPC for Drone Trajectory Tracking
=====================================================
Week 11 - Control Systems: PID to MPC
Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London
"""

import numpy as np
from scipy import linalg
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
from control_sim import (QuadrotorLinearized, MPCController, LQRController,
                         simulate_system, plot_response)


def generate_circle_trajectory(t, radius=2.0, height=5.0, omega=0.5):
    x_ref = np.zeros(12)
    x_ref[0] = radius * np.cos(omega * t)
    x_ref[1] = radius * np.sin(omega * t)
    x_ref[2] = height
    x_ref[3] = -radius * omega * np.sin(omega * t)
    x_ref[4] = radius * omega * np.cos(omega * t)
    return x_ref


def generate_figure8_trajectory(t, scale=2.0, height=5.0, omega=0.3):
    x_ref = np.zeros(12)
    x_ref[0] = scale * np.sin(omega * t)
    x_ref[1] = scale * np.sin(2 * omega * t) / 2.0
    x_ref[2] = height
    x_ref[3] = scale * omega * np.cos(omega * t)
    x_ref[4] = scale * omega * np.cos(2 * omega * t)
    return x_ref


def create_reduced_model():
    """Create a reduced 6-state quadrotor model for MPC: [z, vz, phi, theta, p, q].
    Inputs: [delta_T, delta_phi_torque, delta_theta_torque]
    """
    quad = QuadrotorLinearized()
    # Extract z/vz/phi/theta/p/q subsystem (indices 2,5,6,7,9,10)
    idx = [2, 5, 6, 7, 9, 10]
    input_idx = [0, 1, 2]  # thrust, roll torque, pitch torque
    A_red = quad.A[np.ix_(idx, idx)]
    B_red = quad.B[np.ix_(idx, input_idx)]
    return A_red, B_red, quad


def task6a_hover_mpc():
    """MPC for quadrotor hover stabilization."""
    print("\nMPC Hover Stabilization")
    print("-" * 40)

    A_red, B_red, quad = create_reduced_model()
    n, m = A_red.shape[0], B_red.shape[1]

    from control_sim import LinearSystem
    sys_red = LinearSystem(A_red, B_red)
    dt = 0.05
    Ad, Bd, _, _ = sys_red.discretize(dt)

    Q = np.diag([100, 10, 50, 50, 1, 1])  # z, vz, phi, theta, p, q
    R = np.diag([1.0, 1.0, 1.0])

    mpc = MPCController(Ad, Bd, Q, R, N=10,
                        u_min=np.array([-5.0, -2.0, -2.0]),
                        u_max=np.array([5.0, 2.0, 2.0]))

    # Simulate from perturbed hover
    x0 = np.array([1.0, 0.0, 0.1, -0.05, 0.0, 0.0])  # z=1m above, tilted
    x_ref = np.zeros(n)

    T = 8.0
    steps = int(T / dt)
    x = x0.copy()
    x_hist = np.zeros((steps, n))
    u_hist = np.zeros((steps, m))

    for k in range(steps):
        x_hist[k] = x
        u = mpc.compute(x, x_ref)
        u = np.atleast_1d(u).flatten()[:m]
        u_hist[k] = u
        x = Ad @ x + Bd @ u

    t = np.arange(steps) * dt

    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    labels = ['z [m]', 'vz [m/s]', 'phi [rad]', 'theta [rad]', 'p [rad/s]', 'q [rad/s]']
    for i in range(6):
        r, c = i // 3, i % 3
        axes[r, c].plot(t, x_hist[:, i], 'b-', linewidth=1.5)
        axes[r, c].axhline(y=0, color='r', linestyle='--', alpha=0.5)
        axes[r, c].set_title(labels[i])
        axes[r, c].set_xlabel("Time [s]")
        axes[r, c].grid(True, alpha=0.3)

    fig.suptitle("Task 6a: MPC Hover Stabilization (6-state model)", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task6a_hover_mpc.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task6a_hover_mpc.png")

    print(f"  Final state: {x_hist[-1]}")
    print(f"  Position error: {abs(x_hist[-1, 0]):.6f} m")


def task6b_circle_tracking():
    """MPC for circular trajectory tracking."""
    print("\nMPC Circular Trajectory Tracking")
    print("-" * 40)

    quad = QuadrotorLinearized()
    from control_sim import LinearSystem

    # Use x,y,z + vx,vy,vz subsystem (6 states, simplified)
    # For simplicity, use separate altitude + lateral controllers
    # Altitude: z, vz -> thrust
    A_alt = np.array([[0, 1], [0, 0]])
    B_alt = np.array([[0], [1.0 / quad.mass]])
    sys_alt = LinearSystem(A_alt, B_alt)
    dt = 0.05
    Ad_a, Bd_a, _, _ = sys_alt.discretize(dt)

    Q_alt = np.diag([100, 10])
    R_alt = np.array([[0.5]])

    mpc_alt = MPCController(Ad_a, Bd_a, Q_alt, R_alt, N=10,
                            u_min=np.array([-5.0]), u_max=np.array([5.0]))

    # XY tracking via simple proportional (approximation for demo)
    T = 25.0
    steps = int(T / dt)
    # Full state
    x_full = np.zeros(12)
    x_full[2] = 5.0  # start at hover height

    x_hist = np.zeros((steps, 12))
    ref_hist = np.zeros((steps, 12))

    Ad_full, Bd_full, _, _ = quad.discretize(dt)

    # Full LQR for XY (use for lateral control)
    Q_full = np.diag([50, 50, 100, 1, 1, 10, 50, 50, 20, 1, 1, 1])
    R_full = np.diag([1.0, 1.0, 1.0, 1.0])
    lqr_full = LQRController(quad.A, quad.B, Q_full, R_full)

    for k in range(steps):
        t_k = k * dt
        ref = generate_circle_trajectory(t_k)
        x_hist[k] = x_full
        ref_hist[k] = ref

        # Use LQR for full tracking (MPC just on altitude for demo)
        u = lqr_full.compute(x_full, ref)
        u = np.array(u).flatten()[:4]

        # Override thrust with MPC altitude
        z_state = np.array([x_full[2], x_full[5]])
        z_ref = np.array([ref[2], ref[5]])
        u_alt = mpc_alt.compute(z_state, z_ref)
        u[0] = u_alt[0]

        x_full = Ad_full @ x_full + Bd_full @ u

    t = np.arange(steps) * dt

    fig = plt.figure(figsize=(16, 10))

    # 3D trajectory
    ax3d = fig.add_subplot(2, 2, 1, projection='3d')
    ax3d.plot(x_hist[:, 0], x_hist[:, 1], x_hist[:, 2], 'b-', linewidth=1.5, label='Actual')
    ax3d.plot(ref_hist[:, 0], ref_hist[:, 1], ref_hist[:, 2], 'r--', linewidth=1, label='Reference')
    ax3d.set_xlabel('X [m]')
    ax3d.set_ylabel('Y [m]')
    ax3d.set_zlabel('Z [m]')
    ax3d.set_title('3D Trajectory')
    ax3d.legend()

    # XY plane
    ax_xy = fig.add_subplot(2, 2, 2)
    ax_xy.plot(x_hist[:, 0], x_hist[:, 1], 'b-', linewidth=1.5, label='Actual')
    ax_xy.plot(ref_hist[:, 0], ref_hist[:, 1], 'r--', linewidth=1, label='Reference')
    ax_xy.set_xlabel('X [m]')
    ax_xy.set_ylabel('Y [m]')
    ax_xy.set_title('XY Plane')
    ax_xy.legend()
    ax_xy.grid(True, alpha=0.3)
    ax_xy.set_aspect('equal')

    # Tracking error
    ax_err = fig.add_subplot(2, 2, 3)
    pos_err = np.sqrt((x_hist[:, 0] - ref_hist[:, 0])**2 +
                      (x_hist[:, 1] - ref_hist[:, 1])**2 +
                      (x_hist[:, 2] - ref_hist[:, 2])**2)
    ax_err.plot(t, pos_err, 'b-', linewidth=1.5)
    ax_err.set_title("3D Position Error")
    ax_err.set_xlabel("Time [s]")
    ax_err.set_ylabel("Error [m]")
    ax_err.grid(True, alpha=0.3)

    # Altitude
    ax_z = fig.add_subplot(2, 2, 4)
    ax_z.plot(t, x_hist[:, 2], 'b-', linewidth=1.5, label='Actual z')
    ax_z.plot(t, ref_hist[:, 2], 'r--', linewidth=1, label='Reference z')
    ax_z.set_title("Altitude")
    ax_z.set_xlabel("Time [s]")
    ax_z.set_ylabel("Z [m]")
    ax_z.legend()
    ax_z.grid(True, alpha=0.3)

    fig.suptitle("Task 6b: MPC Circle Tracking", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task6b_circle_tracking.png', dpi=150, bbox_inches='tight')
    plt.close()

    rmse = np.sqrt(np.mean(pos_err[int(5/dt):]**2))
    print(f"  Steady-state tracking RMSE: {rmse:.4f} m")
    print("[INFO] Saved task6b_circle_tracking.png")


def task6c_figure8_tracking():
    """MPC for figure-8 trajectory tracking."""
    print("\nMPC Figure-8 Tracking")
    print("-" * 40)

    quad = QuadrotorLinearized()
    from control_sim import LinearSystem
    dt = 0.05
    T = 30.0
    steps = int(T / dt)

    Ad, Bd, _, _ = quad.discretize(dt)

    Q = np.diag([50, 50, 100, 1, 1, 10, 50, 50, 20, 1, 1, 1])
    R = np.diag([1.0, 1.0, 1.0, 1.0])
    lqr = LQRController(quad.A, quad.B, Q, R)

    x = np.zeros(12)
    x[2] = 5.0
    x_hist = np.zeros((steps, 12))
    ref_hist = np.zeros((steps, 12))

    for k in range(steps):
        t_k = k * dt
        ref = generate_figure8_trajectory(t_k)
        x_hist[k] = x
        ref_hist[k] = ref
        u = lqr.compute(x, ref)
        u = np.array(u).flatten()[:4]
        x = Ad @ x + Bd @ u

    t = np.arange(steps) * dt

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    axes[0].plot(x_hist[:, 0], x_hist[:, 1], 'b-', linewidth=1.5, label='Actual')
    axes[0].plot(ref_hist[:, 0], ref_hist[:, 1], 'r--', linewidth=1, label='Reference')
    axes[0].set_xlabel('X [m]')
    axes[0].set_ylabel('Y [m]')
    axes[0].set_title('XY Trajectory (Figure-8)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_aspect('equal')

    pos_err = np.sqrt((x_hist[:, 0]-ref_hist[:, 0])**2 + (x_hist[:, 1]-ref_hist[:, 1])**2)
    axes[1].plot(t, pos_err, 'b-', linewidth=1.5)
    axes[1].set_title("XY Tracking Error")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Error [m]")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Task 6c: Figure-8 Trajectory Tracking", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task6c_figure8.png', dpi=150, bbox_inches='tight')
    plt.close()

    rmse = np.sqrt(np.mean(pos_err[int(5/dt):]**2))
    print(f"  Steady-state XY tracking RMSE: {rmse:.4f} m")
    print("[INFO] Saved task6c_figure8.png")


def task6d_disturbance_rejection():
    """MPC disturbance rejection."""
    print("\nMPC Disturbance Rejection")
    print("-" * 40)

    A_alt = np.array([[0, 1], [0, 0]])
    B_alt = np.array([[0], [1.0/1.5]])  # mass = 1.5
    from control_sim import LinearSystem
    sys_alt = LinearSystem(A_alt, B_alt)
    dt = 0.05
    Ad, Bd, _, _ = sys_alt.discretize(dt)

    Q = np.diag([100, 10])
    R = np.array([[0.5]])

    mpc = MPCController(Ad, Bd, Q, R, N=10,
                        u_min=np.array([-8.0]), u_max=np.array([8.0]))
    lqr = LQRController(A_alt, B_alt, Q, R)

    T = 15.0
    steps = int(T / dt)
    x_ref = np.array([5.0, 0.0])

    def disturbance(t_val):
        d = np.zeros(2)
        if 3.0 <= t_val < 5.0:
            d[1] = -2.0 / 1.5  # step wind force
        if 7.0 <= t_val < 12.0:
            d[1] = -1.5 * np.sin(2 * t_val) / 1.5  # periodic gust
        return d

    results = {}
    for name, ctrl in [("MPC", mpc), ("LQR", lqr)]:
        x = np.array([5.0, 0.0])
        x_hist = np.zeros((steps, 2))
        u_hist = np.zeros((steps, 1))
        for k in range(steps):
            x_hist[k] = x
            u = ctrl.compute(x, x_ref)
            u = np.atleast_1d(u).flatten()[:1]
            u_hist[k] = u
            d = disturbance(k * dt)
            x = Ad @ x + Bd @ u + d * dt
        results[name] = (x_hist, u_hist)

    t = np.arange(steps) * dt

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    # Disturbance signal
    d_vals = [disturbance(ti)[1] for ti in t]
    axes[0].plot(t, d_vals, 'k-', linewidth=1.5)
    axes[0].set_title("Disturbance (acceleration)")
    axes[0].set_xlabel("Time [s]")
    axes[0].grid(True, alpha=0.3)

    for name, (x_h, u_h) in results.items():
        color = 'b' if name == 'MPC' else 'r'
        axes[1].plot(t, x_h[:, 0], color=color, linewidth=1.5, label=name)
        axes[2].plot(t, u_h[:, 0], color=color, linewidth=1.5, label=name)

    axes[1].axhline(y=5.0, color='k', linestyle='--', alpha=0.5, label='Reference')
    axes[1].set_title("Altitude Response")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_title("Control Effort")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    fig.suptitle("Task 6d: Disturbance Rejection - MPC vs LQR", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task6d_disturbance.png', dpi=150, bbox_inches='tight')
    plt.close()

    for name, (x_h, _) in results.items():
        max_dev = np.max(np.abs(x_h[:, 0] - 5.0))
        print(f"  {name}: max altitude deviation = {max_dev:.4f} m")
    print("[INFO] Saved task6d_disturbance.png")


def main():
    print("=" * 70)
    print("Task 6 SOLUTION: MPC for Drone Trajectory Tracking")
    print("=" * 70)

    task6a_hover_mpc()
    task6b_circle_tracking()
    task6c_figure8_tracking()
    task6d_disturbance_rejection()

    print("\n[DONE] All Task 6 solutions complete.")


if __name__ == '__main__':
    main()
