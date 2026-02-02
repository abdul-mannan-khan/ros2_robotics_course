#!/usr/bin/env python3
"""
Task 2 Solution: State-Space Modeling
=======================================
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
from control_sim import LinearSystem, QuadrotorLinearized, simulate_system, plot_response, rk4_step


class ConstantController:
    """Apply constant input."""
    def __init__(self, u_const):
        self.u = np.array(u_const, dtype=float).flatten()
    def compute(self, x, x_ref=None):
        return self.u


def task2a_mass_spring_damper():
    """Mass-spring-damper state-space model."""
    print("\nMass-Spring-Damper System")
    print("-" * 40)
    m, c, k = 1.0, 0.5, 2.0

    # m*x'' + c*x' + k*x = F
    # x1 = x, x2 = x'
    # x1' = x2
    # x2' = -k/m * x1 - c/m * x2 + F/m
    A = np.array([[0, 1], [-k/m, -c/m]])
    B = np.array([[0], [1.0/m]])
    C = np.array([[1, 0]])  # observe position only

    sys = LinearSystem(A, B, C)
    eigs = sys.eigenvalues()

    print(f"  A = {A.tolist()}")
    print(f"  B = {B.flatten().tolist()}")
    print(f"  Eigenvalues: {eigs}")
    print(f"  Stable: {sys.is_stable()}")
    print(f"  Controllable: {sys.is_controllable()}")
    print(f"  Observable: {sys.is_observable()}")
    print(f"  Natural freq wn = {np.sqrt(k/m):.3f} rad/s")
    print(f"  Damping ratio zeta = {c/(2*np.sqrt(k*m)):.3f}")

    # Simulate step response
    ctrl = ConstantController([1.0])
    t, x, u, y = simulate_system(sys, ctrl, [0, 0], [0, 0], 0.01, 15.0)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(t, x[:, 0], 'b-', linewidth=1.5)
    axes[0].set_title("Position Response to Step Force")
    axes[0].set_xlabel("Time [s]")
    axes[0].set_ylabel("Position [m]")
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=1.0/k, color='r', linestyle='--', label=f'Steady state = {1.0/k:.2f}')
    axes[0].legend()

    axes[1].plot(t, x[:, 1], 'g-', linewidth=1.5)
    axes[1].set_title("Velocity Response")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Velocity [m/s]")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Task 2a: Mass-Spring-Damper", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task2a_mass_spring_damper.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task2a_mass_spring_damper.png")
    return sys


def task2b_dc_motor():
    """DC motor state-space model."""
    print("\nDC Motor Model")
    print("-" * 40)

    R = 1.0     # resistance
    L = 0.01    # inductance
    J = 0.01    # moment of inertia
    b = 0.1     # friction
    Kt = 0.01   # torque constant
    Ke = 0.01   # back-EMF constant

    # States: [theta, omega, i]
    # theta' = omega
    # omega' = (Kt*i - b*omega) / J
    # i' = (V - Ke*omega - R*i) / L
    A = np.array([
        [0, 1, 0],
        [0, -b/J, Kt/J],
        [0, -Ke/L, -R/L]
    ])
    B = np.array([[0], [0], [1.0/L]])
    C = np.array([[1, 0, 0], [0, 1, 0]])  # observe position and velocity

    sys = LinearSystem(A, B, C)
    eigs = sys.eigenvalues()

    print(f"  States: [theta, omega, current]")
    print(f"  Eigenvalues: {eigs}")
    print(f"  Stable: {sys.is_stable()}")
    print(f"  Controllable: {sys.is_controllable()}")
    print(f"  Observable: {sys.is_observable()}")
    print(f"  Controllability rank: {np.linalg.matrix_rank(sys.controllability_matrix())}/{sys.n}")

    ctrl = ConstantController([1.0])
    t, x, u, y = simulate_system(sys, ctrl, [0, 0, 0], [0, 0, 0], 0.001, 2.0)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    labels = ['Angular Position [rad]', 'Angular Velocity [rad/s]', 'Current [A]']
    colors = ['b', 'g', 'r']
    for i in range(3):
        axes[i].plot(t, x[:, i], colors[i], linewidth=1.5)
        axes[i].set_title(labels[i])
        axes[i].set_xlabel("Time [s]")
        axes[i].grid(True, alpha=0.3)

    fig.suptitle("Task 2b: DC Motor Step Response (V=1V)", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task2b_dc_motor.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task2b_dc_motor.png")
    return sys


def task2c_quadrotor_analysis():
    """Analyze linearized quadrotor model."""
    print("\nQuadrotor Linearized Model Analysis")
    print("-" * 40)

    quad = QuadrotorLinearized()
    eigs = quad.eigenvalues()

    print(f"  Number of states: {quad.n}")
    print(f"  Number of inputs: {quad.m}")
    print(f"  Controllable: {quad.is_controllable()}")
    print(f"  Observable: {quad.is_observable()}")

    print("\n  Eigenvalues and stability classification:")
    for i, e in enumerate(eigs):
        re = np.real(e)
        im = np.imag(e)
        if abs(re) < 1e-10:
            stability = "Marginally stable"
        elif re < 0:
            stability = "Stable"
        else:
            stability = "UNSTABLE"
        print(f"    lambda_{i} = {re:+.4f} {'+' if im >= 0 else ''}{im:.4f}j  -> {stability}")

    # Discretize
    dt = 0.01
    Ad, Bd, Cd, Dd = quad.discretize(dt)
    eigs_d = np.linalg.eigvals(Ad)

    # Plot eigenvalue map
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # S-plane
    ax = axes[0]
    ax.scatter(np.real(eigs), np.imag(eigs), c='red', s=100, marker='x', linewidths=2, zorder=5)
    ax.axvline(x=0, color='k', linewidth=0.5)
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.fill_betweenx([-10, 10], -20, 0, alpha=0.1, color='green', label='Stable region')
    ax.set_xlim([-5, 1])
    ax.set_ylim([-3, 3])
    ax.set_xlabel("Real")
    ax.set_ylabel("Imaginary")
    ax.set_title("S-Plane (Continuous)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Z-plane
    ax = axes[1]
    theta = np.linspace(0, 2*np.pi, 100)
    ax.plot(np.cos(theta), np.sin(theta), 'k-', linewidth=1)
    ax.scatter(np.real(eigs_d), np.imag(eigs_d), c='red', s=100, marker='x', linewidths=2, zorder=5)
    ax.fill(np.cos(theta), np.sin(theta), alpha=0.1, color='green', label='Stable region')
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_xlabel("Real")
    ax.set_ylabel("Imaginary")
    ax.set_title(f"Z-Plane (Discrete, dt={dt}s)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    fig.suptitle("Task 2c: Quadrotor Eigenvalue Map", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task2c_quadrotor_eigenvalues.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task2c_quadrotor_eigenvalues.png")


def task2d_controllability_observability():
    """Demonstrate controllability and observability edge cases."""
    print("\nControllability & Observability Study")
    print("-" * 40)

    # System 1: Controllable but NOT observable
    # Two states, one observed, but second state not visible
    A1 = np.array([[0, 1], [0, -1]])
    B1 = np.array([[1], [0]])
    C1 = np.array([[0, 1]])  # Only observe x2
    sys1 = LinearSystem(A1, B1, C1)

    print(f"\n  System 1 (designed to be controllable, not observable):")
    print(f"    Controllable: {sys1.is_controllable()}")
    print(f"    Observable: {sys1.is_observable()}")
    print(f"    Controllability rank: {np.linalg.matrix_rank(sys1.controllability_matrix())}/{sys1.n}")
    print(f"    Observability rank: {np.linalg.matrix_rank(sys1.observability_matrix())}/{sys1.n}")

    # System 2: Observable but NOT controllable
    A2 = np.array([[-1, 0], [0, -2]])
    B2 = np.array([[1], [0]])  # Can only drive first state
    C2 = np.array([[1, 0], [0, 1]])  # Observe both
    sys2 = LinearSystem(A2, B2, C2)

    print(f"\n  System 2 (designed to be observable, not fully controllable from single input):")
    print(f"    Controllable: {sys2.is_controllable()}")
    print(f"    Observable: {sys2.is_observable()}")
    print(f"    Controllability rank: {np.linalg.matrix_rank(sys2.controllability_matrix())}/{sys2.n}")
    print(f"    Observability rank: {np.linalg.matrix_rank(sys2.observability_matrix())}/{sys2.n}")

    # System 3: Both controllable and observable
    A3 = np.array([[-1, 1], [0, -2]])
    B3 = np.array([[0], [1]])
    C3 = np.array([[1, 0]])
    sys3 = LinearSystem(A3, B3, C3)

    print(f"\n  System 3 (controllable AND observable):")
    print(f"    Controllable: {sys3.is_controllable()}")
    print(f"    Observable: {sys3.is_observable()}")

    print("\n  KEY INSIGHT: A system must be controllable for state feedback to")
    print("  place all poles. It must be observable for a state estimator (e.g.,")
    print("  Kalman filter) to reconstruct all states from outputs.")


def main():
    print("=" * 70)
    print("Task 2 SOLUTION: State-Space Modeling")
    print("=" * 70)

    task2a_mass_spring_damper()
    task2b_dc_motor()
    task2c_quadrotor_analysis()
    task2d_controllability_observability()

    print("\n[DONE] All Task 2 solutions complete.")


if __name__ == '__main__':
    main()
