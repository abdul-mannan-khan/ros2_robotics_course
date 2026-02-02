#!/usr/bin/env python3
"""
Task 3 Solution: Pole Placement & Eigenvalue Assignment
=========================================================
Week 11 - Control Systems: PID to MPC
Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London
"""

import numpy as np
from scipy import signal, linalg
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
from control_sim import LinearSystem, pole_placement, simulate_system, plot_response


class StateFeedbackController:
    """u = -K*(x - x_ref) state feedback controller."""
    def __init__(self, K):
        self.K = np.atleast_2d(np.array(K, dtype=float))
    def compute(self, x, x_ref=None):
        x = np.array(x, dtype=float).flatten()
        if x_ref is not None:
            x_ref = np.array(x_ref, dtype=float).flatten()
            return (-self.K @ (x - x_ref)).flatten()
        return (-self.K @ x).flatten()


def task3a_basic_pole_placement():
    """Basic pole placement for double integrator."""
    print("\nBasic Pole Placement - Double Integrator")
    print("-" * 40)

    A = np.array([[0, 1], [0, 0]])
    B = np.array([[0], [1]])
    sys = LinearSystem(A, B)

    print(f"  Open-loop eigenvalues: {sys.eigenvalues()}")
    print(f"  Controllable: {sys.is_controllable()}")

    # Desired poles: s = -2 +/- 2j (zeta ~ 0.707, wn ~ 2.83)
    desired = np.array([-2 + 2j, -2 - 2j])
    K = pole_placement(A, B, desired)

    A_cl = A - B @ K
    cl_eigs = np.linalg.eigvals(A_cl)

    print(f"  Desired poles: {desired}")
    print(f"  Gain K: {K}")
    print(f"  Closed-loop eigenvalues: {cl_eigs}")
    print(f"  Poles match: {np.allclose(np.sort(cl_eigs), np.sort(desired), atol=1e-6)}")

    # Simulate
    ctrl = StateFeedbackController(K)
    t, x, u, y = simulate_system(sys, ctrl, [1, 0], [0, 0], 0.01, 5.0)

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

    fig.suptitle("Task 3a: Pole Placement (Double Integrator)", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task3a_pole_placement.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task3a_pole_placement.png")


def task3b_specification_design():
    """Design from time-domain specifications."""
    print("\nSpecification-Based Pole Placement")
    print("-" * 40)

    # Specs: settling time < 2s, overshoot < 10%
    # Mp = exp(-pi*zeta/sqrt(1-zeta^2)) < 0.10
    # -> zeta > 0.59 (use 0.7)
    # ts ~ 4/(zeta*wn) < 2 -> wn > 4/(0.7*2) ~ 2.86 (use 3.0)
    zeta = 0.7
    wn = 3.0
    sigma = zeta * wn
    wd = wn * np.sqrt(1 - zeta**2)
    desired = np.array([-sigma + 1j*wd, -sigma - 1j*wd])

    print(f"  Target: ts < 2s, Mp < 10%")
    print(f"  Design: zeta={zeta}, wn={wn}")
    print(f"  Desired poles: {desired}")

    # Mass-spring-damper plant
    m, c, k = 1.0, 0.5, 2.0
    A = np.array([[0, 1], [-k/m, -c/m]])
    B = np.array([[0], [1.0/m]])
    sys = LinearSystem(A, B)

    K = pole_placement(A, B, desired)
    ctrl = StateFeedbackController(K)

    print(f"  Gain K: {K}")

    # Simulate step to 1.0 (use reference tracking: u = -K(x - x_ref) + feedforward)
    # For step tracking, add integral or precompensation
    # Simple: x_ref = [1, 0], track with state feedback
    t, x, u, y = simulate_system(sys, ctrl, [0, 0], [1, 0], 0.01, 5.0)

    # Compute metrics
    pos = x[:, 0]
    peak = np.max(pos)
    overshoot = max(0, (peak - 1.0) / 1.0 * 100) if peak > 1.0 else 0
    settled = np.abs(pos - 1.0) <= 0.02
    not_settled = np.where(~settled)[0]
    settling_time = t[not_settled[-1]] if len(not_settled) > 0 else 0

    print(f"  Actual overshoot: {overshoot:.1f}% (spec: <10%)")
    print(f"  Actual settling time: {settling_time:.2f}s (spec: <2s)")

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(t, pos, 'b-', linewidth=2, label='Response')
    ax.axhline(y=1.0, color='k', linestyle='--', alpha=0.5, label='Reference')
    ax.axhline(y=1.1, color='r', linestyle=':', alpha=0.5, label='10% overshoot')
    ax.axvline(x=2.0, color='orange', linestyle=':', alpha=0.5, label='2s settling spec')
    ax.fill_between(t, 0.98, 1.02, alpha=0.2, color='green', label='2% band')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position")
    ax.set_title(f"Spec Design: zeta={zeta}, wn={wn}, K={K.flatten()}")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle("Task 3b: Specification-Based Design", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task3b_specs.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task3b_specs.png")


def task3c_stability_regions():
    """Visualize s-plane and z-plane stability regions."""
    print("\nStability Regions Visualization")
    print("-" * 40)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # S-plane
    ax = axes[0]
    ax.fill_betweenx([-8, 8], -10, 0, alpha=0.1, color='green', label='Stable')

    # Lines of constant damping ratio
    for zeta in [0.2, 0.4, 0.6, 0.8]:
        angle = np.arccos(zeta)
        r = np.linspace(0, 8, 100)
        ax.plot(-r * np.cos(angle), r * np.sin(angle), 'gray', alpha=0.3, linewidth=0.8)
        ax.plot(-r * np.cos(angle), -r * np.sin(angle), 'gray', alpha=0.3, linewidth=0.8)
        ax.annotate(f'z={zeta}', (-2*np.cos(angle), 2*np.sin(angle)+0.2), fontsize=7, color='gray')

    # Lines of constant wn
    for wn in [2, 4, 6]:
        theta = np.linspace(np.pi/2, np.pi, 50)
        ax.plot(wn * np.cos(theta), wn * np.sin(theta), 'gray', alpha=0.3, linewidth=0.8, linestyle='--')
        ax.plot(wn * np.cos(theta), -wn * np.sin(theta), 'gray', alpha=0.3, linewidth=0.8, linestyle='--')

    # Place 5 example pole pairs
    pole_sets = [
        ([-1+1j, -1-1j], 'blue', 'Underdamped slow'),
        ([-3+1j, -3-1j], 'red', 'Underdamped fast'),
        ([-2, -2], 'green', 'Critically damped'),
        ([-1, -5], 'purple', 'Overdamped'),
        ([0.5+2j, 0.5-2j], 'orange', 'UNSTABLE'),
    ]

    for poles, color, label in pole_sets:
        poles = np.array(poles)
        ax.scatter(np.real(poles), np.imag(poles), c=color, s=80, marker='x',
                   linewidths=2, zorder=5, label=label)

    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.axvline(x=0, color='k', linewidth=0.5)
    ax.set_xlim([-7, 2])
    ax.set_ylim([-5, 5])
    ax.set_xlabel("Real (sigma)")
    ax.set_ylabel("Imaginary (jw)")
    ax.set_title("S-Plane")
    ax.legend(fontsize=7, loc='lower left')
    ax.grid(True, alpha=0.3)

    # Z-plane
    ax = axes[1]
    theta = np.linspace(0, 2*np.pi, 200)
    ax.plot(np.cos(theta), np.sin(theta), 'k-', linewidth=1.5)
    ax.fill(np.cos(theta), np.sin(theta), alpha=0.1, color='green', label='Stable')

    dt = 0.1
    for poles, color, label in pole_sets:
        poles_z = np.exp(np.array(poles, dtype=complex) * dt)
        ax.scatter(np.real(poles_z), np.imag(poles_z), c=color, s=80, marker='x',
                   linewidths=2, zorder=5, label=label)

    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.axvline(x=0, color='k', linewidth=0.5)
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_xlabel("Real")
    ax.set_ylabel("Imaginary")
    ax.set_title(f"Z-Plane (dt={dt}s)")
    ax.legend(fontsize=7, loc='lower left')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    fig.suptitle("Task 3c: Stability Regions", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task3c_stability_regions.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task3c_stability_regions.png")


def task3d_ackermann_formula():
    """Implement Ackermann's formula manually."""
    print("\nAckermann's Formula Implementation")
    print("-" * 40)

    # 3rd order system
    A = np.array([[0, 1, 0], [0, 0, 1], [-6, -11, -6]])
    B = np.array([[0], [0], [1]])
    n = A.shape[0]

    desired = np.array([-3, -4, -5])
    print(f"  System order: {n}")
    print(f"  Open-loop eigenvalues: {np.linalg.eigvals(A)}")
    print(f"  Desired poles: {desired}")

    # Ackermann's formula: K = e_n^T * C^{-1} * phi_d(A)
    # where e_n = [0, 0, ..., 1], C = controllability matrix
    # phi_d(A) = A^n + a_{n-1}*A^{n-1} + ... + a_0*I (desired char poly evaluated at A)

    # Controllability matrix
    C_mat = B.copy()
    Ak = np.eye(n)
    for i in range(1, n):
        Ak = Ak @ A
        C_mat = np.hstack([C_mat, Ak @ B])

    print(f"  Controllability matrix rank: {np.linalg.matrix_rank(C_mat)}/{n}")

    # Desired characteristic polynomial: (s - p1)(s - p2)(s - p3)
    # coefficients from np.poly
    char_poly = np.real(np.poly(desired))  # [1, a1, a2, a3]
    print(f"  Desired char. poly coefficients: {char_poly}")

    # Evaluate phi_d(A) = A^3 + a1*A^2 + a2*A + a3*I
    phi_d = np.zeros((n, n))
    Ak = np.eye(n)
    for i in range(n + 1):
        phi_d += char_poly[n - i] * Ak
        if i < n:
            Ak = Ak @ A

    # K = e_n^T * C^{-1} * phi_d
    e_n = np.zeros(n)
    e_n[-1] = 1.0
    K_ackermann = (e_n @ np.linalg.inv(C_mat) @ phi_d).reshape(1, -1)

    # Compare with scipy
    K_scipy = pole_placement(A, B, desired)

    print(f"  K (Ackermann):  {K_ackermann.flatten()}")
    print(f"  K (scipy):      {K_scipy.flatten()}")
    print(f"  Match: {np.allclose(K_ackermann, K_scipy, atol=1e-6)}")

    # Verify closed-loop eigenvalues
    A_cl = A - B @ K_ackermann
    cl_eigs = np.linalg.eigvals(A_cl)
    print(f"  Closed-loop eigenvalues: {np.sort(np.real(cl_eigs))}")


def main():
    print("=" * 70)
    print("Task 3 SOLUTION: Pole Placement & Eigenvalue Assignment")
    print("=" * 70)

    task3a_basic_pole_placement()
    task3b_specification_design()
    task3c_stability_regions()
    task3d_ackermann_formula()

    print("\n[DONE] All Task 3 solutions complete.")


if __name__ == '__main__':
    main()
