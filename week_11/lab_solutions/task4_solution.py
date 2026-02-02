#!/usr/bin/env python3
"""
Task 4 Solution: Linear Quadratic Regulator (LQR)
===================================================
Week 11 - Control Systems: PID to MPC
Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London
"""

import numpy as np
from scipy import linalg
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
from control_sim import (LinearSystem, QuadrotorLinearized, LQRController,
                         simulate_system, plot_response, pole_placement)


class StateFeedbackController:
    def __init__(self, K):
        self.K = np.atleast_2d(np.array(K, dtype=float))
    def compute(self, x, x_ref=None):
        x = np.array(x, dtype=float).flatten()
        ref = np.array(x_ref, dtype=float).flatten() if x_ref is not None else np.zeros_like(x)
        return (-self.K @ (x - ref)).flatten()


def task4a_lqr_basics():
    """Basic LQR design for double integrator."""
    print("\nLQR Basics - Double Integrator")
    print("-" * 40)

    A = np.array([[0, 1], [0, 0]])
    B = np.array([[0], [1]])

    Q = np.diag([10.0, 1.0])
    R = np.array([[1.0]])

    lqr = LQRController(A, B, Q, R)

    print(f"  Q = diag({np.diag(Q)})")
    print(f"  R = {R.flatten()}")
    print(f"  Riccati solution P:\n    {lqr.P}")
    print(f"  Optimal gain K: {lqr.K}")
    print(f"  Closed-loop eigenvalues: {lqr.closed_loop_eigenvalues()}")

    # Simulate from x0=[1, 0]
    sys = LinearSystem(A, B)
    x0 = np.array([1, 0])
    dt = 0.01
    T = 5.0
    t, x, u, y = simulate_system(sys, lqr, x0, [0, 0], dt, T)

    # Compute actual cost
    J_actual = 0.0
    for k in range(len(t)):
        xk = x[k]
        uk = u[k]
        J_actual += (xk @ Q @ xk + uk @ R @ uk) * dt
    J_theory = x0 @ lqr.P @ x0

    print(f"\n  Cost J (numerical): {J_actual:.4f}")
    print(f"  Cost J (x0'Px0):   {J_theory:.4f}")
    print(f"  Match: {abs(J_actual - J_theory) / J_theory * 100:.1f}% error")

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    axes[0].plot(t, x[:, 0], 'b-', linewidth=1.5)
    axes[0].set_title("Position")
    axes[0].axhline(y=0, color='r', linestyle='--')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlabel("Time [s]")

    axes[1].plot(t, x[:, 1], 'g-', linewidth=1.5)
    axes[1].set_title("Velocity")
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlabel("Time [s]")

    axes[2].plot(t, u[:, 0], 'r-', linewidth=1.5)
    axes[2].set_title("Control Input")
    axes[2].grid(True, alpha=0.3)
    axes[2].set_xlabel("Time [s]")

    fig.suptitle(f"Task 4a: LQR (J_theory={J_theory:.3f}, J_actual={J_actual:.3f})",
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task4a_lqr_basics.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task4a_lqr_basics.png")


def task4b_weight_tuning():
    """Q/R weight tuning study."""
    print("\nLQR Weight Tuning")
    print("-" * 40)

    A = np.array([[0, 1], [0, 0]])
    B = np.array([[0], [1]])
    sys = LinearSystem(A, B)
    Q = np.diag([10.0, 1.0])

    R_values = [0.01, 0.1, 1.0, 10.0, 100.0]
    colors = plt.cm.viridis(np.linspace(0, 1, len(R_values)))

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    dt = 0.01
    T = 8.0

    print(f"  {'R':>6} {'K':>20} {'CL Eigs':>30} {'Peak |u|':>10}")
    print("  " + "-" * 70)

    for R_val, color in zip(R_values, colors):
        R = np.array([[R_val]])
        lqr = LQRController(A, B, Q, R)
        t, x, u, y = simulate_system(sys, lqr, [1, 0], [0, 0], dt, T)

        label = f"R={R_val}"
        axes[0].plot(t, x[:, 0], color=color, linewidth=1.5, label=label)
        axes[1].plot(t, u[:, 0], color=color, linewidth=1.5, label=label)

        eigs = lqr.closed_loop_eigenvalues()
        peak_u = np.max(np.abs(u))
        print(f"  {R_val:>6.2f} {str(lqr.K.flatten()):>20} {str(np.round(eigs, 3)):>30} {peak_u:>10.3f}")

    # Trade-off plot: total control effort vs settling time
    efforts = []
    settle_times = []
    for R_val in np.logspace(-2, 2, 20):
        R = np.array([[R_val]])
        lqr = LQRController(A, B, Q, R)
        t, x, u, y = simulate_system(sys, lqr, [1, 0], [0, 0], dt, T)
        effort = np.sum(u**2) * dt
        settled = np.where(np.abs(x[:, 0]) > 0.02)[0]
        st = t[settled[-1]] if len(settled) > 0 else 0
        efforts.append(effort)
        settle_times.append(st)

    axes[2].plot(efforts, settle_times, 'ko-', markersize=3)
    axes[2].set_xlabel("Total Control Effort")
    axes[2].set_ylabel("Settling Time [s]")
    axes[2].set_title("Effort vs Speed Trade-off")
    axes[2].grid(True, alpha=0.3)

    axes[0].set_title("Position Response")
    axes[0].set_xlabel("Time [s]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[1].set_title("Control Input")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Task 4b: LQR Weight Tuning (Q fixed, R varies)", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task4b_weight_tuning.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task4b_weight_tuning.png")


def task4c_quadrotor_lqr():
    """LQR for quadrotor hover stabilization."""
    print("\nQuadrotor LQR Hover Stabilization")
    print("-" * 40)

    quad = QuadrotorLinearized()

    # Q: penalize positions and angles heavily
    Q = np.diag([50, 50, 100,   # x, y, z position
                  1, 1, 10,      # vx, vy, vz velocity
                  50, 50, 20,    # phi, theta, psi angles
                  1, 1, 1])      # p, q, r angular rates

    R = np.diag([1.0, 1.0, 1.0, 1.0])  # input costs

    lqr = LQRController(quad.A, quad.B, Q, R)
    print(f"  LQR gain K shape: {lqr.K.shape}")
    print(f"  Closed-loop eigenvalues (real parts):")
    cl_eigs = lqr.closed_loop_eigenvalues()
    for i, e in enumerate(np.sort(np.real(cl_eigs))):
        print(f"    lambda_{i}: Re = {e:.4f}")
    print(f"  All stable: {np.all(np.real(cl_eigs) < 0)}")

    # Simulate from perturbed hover
    x0 = np.zeros(12)
    x0[0] = 0.5   # x offset
    x0[1] = -0.3  # y offset
    x0[2] = 1.0   # z offset (1m above hover)
    x0[6] = 0.1   # phi = ~6 deg
    x0[7] = -0.05 # theta = ~3 deg

    dt = 0.01
    T = 10.0
    t, x, u, y = simulate_system(quad, lqr, x0, np.zeros(12), dt, T)

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # Position
    for i, (idx, label) in enumerate([(0, 'x'), (1, 'y'), (2, 'z')]):
        ax = axes[0, 0] if i < 2 else axes[0, 1]
        ax.plot(t, x[:, idx], linewidth=1.5, label=label)

    axes[0, 0].set_title("XY Position [m]")
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 1].set_title("Z Position [m]")
    axes[0, 1].plot(t, x[:, 2], 'b-', linewidth=1.5)
    axes[0, 1].axhline(y=0, color='r', linestyle='--')
    axes[0, 1].grid(True, alpha=0.3)

    # Angles
    for i, (idx, label) in enumerate([(6, 'phi'), (7, 'theta'), (8, 'psi')]):
        axes[1, 0].plot(t, np.degrees(x[:, idx]), linewidth=1.5, label=label)
    axes[1, 0].set_title("Angles [deg]")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Velocities
    for i, (idx, label) in enumerate([(3, 'vx'), (4, 'vy'), (5, 'vz')]):
        axes[1, 1].plot(t, x[:, idx], linewidth=1.5, label=label)
    axes[1, 1].set_title("Velocities [m/s]")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    # Control inputs
    input_labels = ['Thrust', 'Roll torque', 'Pitch torque', 'Yaw torque']
    for j in range(4):
        axes[2, 0 if j < 2 else 1].plot(t, u[:, j], linewidth=1.5, label=input_labels[j])
    axes[2, 0].set_title("Thrust & Roll Torque")
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].set_xlabel("Time [s]")
    axes[2, 1].set_title("Pitch & Yaw Torque")
    axes[2, 1].legend()
    axes[2, 1].grid(True, alpha=0.3)
    axes[2, 1].set_xlabel("Time [s]")

    fig.suptitle("Task 4c: Quadrotor LQR Hover Stabilization", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task4c_quadrotor_lqr.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task4c_quadrotor_lqr.png")


def task4d_lqr_vs_pole_placement():
    """Compare LQR with pole placement."""
    print("\nLQR vs Pole Placement Comparison")
    print("-" * 40)

    A = np.array([[0, 1, 0], [0, 0, 1], [-6, -11, -6]])
    B = np.array([[0], [0], [1]])
    sys = LinearSystem(A, B)

    # Pole placement: place at -3, -4, -5
    K_pp = pole_placement(A, B, [-3, -4, -5])
    ctrl_pp = StateFeedbackController(K_pp)

    # LQR
    Q = np.diag([10, 1, 1])
    R = np.array([[1.0]])
    lqr = LQRController(A, B, Q, R)

    print(f"  Pole Placement K: {K_pp.flatten()}")
    print(f"  LQR K: {lqr.K.flatten()}")

    dt = 0.01
    T = 5.0
    x0 = np.array([1, 0, 0])

    t1, x1, u1, _ = simulate_system(sys, ctrl_pp, x0, [0, 0, 0], dt, T)
    t2, x2, u2, _ = simulate_system(sys, lqr, x0, [0, 0, 0], dt, T)

    # Robustness test: 20% parameter perturbation
    A_perturbed = A * 1.2
    sys_p = LinearSystem(A_perturbed, B)
    t3, x3, u3, _ = simulate_system(sys_p, ctrl_pp, x0, [0, 0, 0], dt, T)
    t4, x4, u4, _ = simulate_system(sys_p, lqr, x0, [0, 0, 0], dt, T)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    axes[0, 0].plot(t1, x1[:, 0], 'b-', linewidth=1.5, label='Pole Placement')
    axes[0, 0].plot(t2, x2[:, 0], 'r-', linewidth=1.5, label='LQR')
    axes[0, 0].set_title("Nominal: Position x[0]")
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(t1, u1[:, 0], 'b-', linewidth=1.5, label='Pole Placement')
    axes[0, 1].plot(t2, u2[:, 0], 'r-', linewidth=1.5, label='LQR')
    axes[0, 1].set_title("Nominal: Control Input")
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(t3, x3[:, 0], 'b-', linewidth=1.5, label='Pole Placement')
    axes[1, 0].plot(t4, x4[:, 0], 'r-', linewidth=1.5, label='LQR')
    axes[1, 0].set_title("Perturbed (+20%): Position x[0]")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_xlabel("Time [s]")

    axes[1, 1].plot(t3, u3[:, 0], 'b-', linewidth=1.5, label='Pole Placement')
    axes[1, 1].plot(t4, u4[:, 0], 'r-', linewidth=1.5, label='LQR')
    axes[1, 1].set_title("Perturbed (+20%): Control Input")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_xlabel("Time [s]")

    effort_pp = np.sum(u1**2) * dt
    effort_lqr = np.sum(u2**2) * dt
    print(f"  Control effort (nominal) - PP: {effort_pp:.3f}, LQR: {effort_lqr:.3f}")
    print(f"  LQR uses {(effort_lqr/effort_pp - 1)*100:+.1f}% effort vs PP")
    print(f"  LQR guarantees gain margin >= 6dB and phase margin >= 60 deg (for SISO)")

    fig.suptitle("Task 4d: LQR vs Pole Placement", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task4d_comparison.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task4d_comparison.png")


def main():
    print("=" * 70)
    print("Task 4 SOLUTION: Linear Quadratic Regulator (LQR)")
    print("=" * 70)

    task4a_lqr_basics()
    task4b_weight_tuning()
    task4c_quadrotor_lqr()
    task4d_lqr_vs_pole_placement()

    print("\n[DONE] All Task 4 solutions complete.")


if __name__ == '__main__':
    main()
