#!/usr/bin/env python3
"""
Task 5 Solution: MPC Basics
==============================
Week 11 - Control Systems: PID to MPC
Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London
"""

import numpy as np
from scipy import linalg, optimize
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
from control_sim import (LinearSystem, MPCController, LQRController,
                         simulate_system, plot_response)


def task5a_mpc_formulation():
    """Basic MPC formulation and one-step solve."""
    print("\nMPC Formulation - One Step Solve")
    print("-" * 40)

    # Double integrator
    A = np.array([[0, 1], [0, 0]])
    B = np.array([[0], [1]])
    sys = LinearSystem(A, B)
    dt = 0.1

    # Discretize
    Ad, Bd, Cd, Dd = sys.discretize(dt)
    print(f"  Ad = {Ad.tolist()}")
    print(f"  Bd = {Bd.flatten().tolist()}")

    Q = np.diag([10.0, 1.0])
    R = np.array([[0.1]])
    N = 10

    mpc = MPCController(Ad, Bd, Q, R, N=N)

    x0 = np.array([2.0, 0.0])
    x_ref = np.array([0.0, 0.0])

    # Solve one step
    u_mpc = mpc.compute(x0, x_ref)

    # Compare with LQR
    lqr = LQRController(A, B, Q, R)
    u_lqr = lqr.compute(x0, x_ref)

    print(f"  x0 = {x0}")
    print(f"  MPC first input: u = {u_mpc}")
    print(f"  LQR input:       u = {u_lqr}")
    print(f"  (Without constraints, MPC ~ LQR as N -> inf)")

    # Show predicted trajectory
    states = mpc._predict(x0, mpc.u_prev)
    states = np.array(states)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    time_steps = np.arange(N + 1) * dt

    axes[0].plot(time_steps, states[:, 0], 'bo-', linewidth=1.5, label='MPC prediction')
    axes[0].axhline(y=0, color='r', linestyle='--', label='Reference')
    axes[0].set_title("Predicted Position")
    axes[0].set_xlabel("Time [s]")
    axes[0].set_ylabel("Position")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    u_seq = mpc.u_prev.reshape(N, -1)
    axes[1].step(np.arange(N) * dt, u_seq[:, 0], 'g-', linewidth=1.5, where='post')
    axes[1].set_title("Predicted Control Sequence")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Input")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Task 5a: MPC One-Step Prediction", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task5a_mpc_formulation.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task5a_mpc_formulation.png")


def task5b_receding_horizon():
    """Receding horizon simulation."""
    print("\nReceding Horizon Simulation")
    print("-" * 40)

    A = np.array([[0, 1], [0, 0]])
    B = np.array([[0], [1]])
    sys = LinearSystem(A, B)
    dt = 0.1
    Ad, Bd, _, _ = sys.discretize(dt)

    Q = np.diag([10.0, 1.0])
    R = np.array([[0.1]])
    N = 10

    mpc = MPCController(Ad, Bd, Q, R, N=N)
    lqr = LQRController(A, B, Q, R)

    # Simulate closed-loop
    T = 5.0
    steps = int(T / dt)
    x_mpc = np.array([2.0, 0.0])
    x_lqr = np.array([2.0, 0.0])
    x_ref = np.array([0.0, 0.0])

    t_hist = np.zeros(steps)
    x_mpc_hist = np.zeros((steps, 2))
    x_lqr_hist = np.zeros((steps, 2))
    u_mpc_hist = np.zeros((steps, 1))
    u_lqr_hist = np.zeros((steps, 1))

    # Store predictions at a few time steps
    prediction_times = [0, 5, 10, 20]
    predictions = {}

    for k in range(steps):
        t_hist[k] = k * dt
        x_mpc_hist[k] = x_mpc
        x_lqr_hist[k] = x_lqr

        # MPC
        u_m = mpc.compute(x_mpc, x_ref)
        u_mpc_hist[k] = u_m

        if k in prediction_times:
            states_pred = mpc._predict(x_mpc, mpc.u_prev)
            predictions[k] = np.array(states_pred)

        # LQR
        u_l = lqr.compute(x_lqr, x_ref)
        u_lqr_hist[k] = u_l

        # Step both systems
        x_mpc = Ad @ x_mpc + Bd @ u_m.flatten()
        x_lqr = Ad @ x_lqr + Bd @ u_l.flatten()

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    axes[0].plot(t_hist, x_mpc_hist[:, 0], 'b-', linewidth=2, label='MPC')
    axes[0].plot(t_hist, x_lqr_hist[:, 0], 'r--', linewidth=2, label='LQR')
    axes[0].axhline(y=0, color='k', linestyle=':', alpha=0.5)

    # Show predictions
    for k, pred in predictions.items():
        t_pred = (k + np.arange(N + 1)) * dt
        axes[0].plot(t_pred, pred[:, 0], 'c-', alpha=0.4, linewidth=1)

    axes[0].set_title("Position (with MPC predictions in cyan)")
    axes[0].set_xlabel("Time [s]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_hist, u_mpc_hist[:, 0], 'b-', linewidth=1.5, label='MPC')
    axes[1].plot(t_hist, u_lqr_hist[:, 0], 'r--', linewidth=1.5, label='LQR')
    axes[1].set_title("Control Input")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Task 5b: Receding Horizon Simulation", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task5b_receding_horizon.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task5b_receding_horizon.png")

    rmse_mpc = np.sqrt(np.mean(x_mpc_hist[:, 0]**2))
    rmse_lqr = np.sqrt(np.mean(x_lqr_hist[:, 0]**2))
    print(f"  Position RMSE - MPC: {rmse_mpc:.4f}, LQR: {rmse_lqr:.4f}")


def task5c_constrained_mpc():
    """MPC with input and state constraints."""
    print("\nConstrained MPC")
    print("-" * 40)

    A = np.array([[0, 1], [0, 0]])
    B = np.array([[0], [1]])
    sys = LinearSystem(A, B)
    dt = 0.1
    Ad, Bd, _, _ = sys.discretize(dt)

    Q = np.diag([10.0, 1.0])
    R = np.array([[0.1]])
    N = 15

    # Constrained MPC: |u| <= 1.0, |x[0]| <= 3.0
    mpc_con = MPCController(Ad, Bd, Q, R, N=N,
                            u_min=np.array([-1.0]),
                            u_max=np.array([1.0]),
                            x_min=np.array([-3.0, -10.0]),
                            x_max=np.array([3.0, 10.0]))

    # Unconstrained MPC
    mpc_uncon = MPCController(Ad, Bd, Q, R, N=N)

    # LQR
    lqr = LQRController(A, B, Q, R)

    T = 8.0
    steps = int(T / dt)
    x0 = np.array([2.5, 0.0])
    x_ref = np.array([0.0, 0.0])

    results = {}
    for name, ctrl in [("Constrained MPC", mpc_con), ("Unconstrained MPC", mpc_uncon), ("LQR", lqr)]:
        x = x0.copy()
        x_hist = np.zeros((steps, 2))
        u_hist = np.zeros((steps, 1))
        for k in range(steps):
            x_hist[k] = x
            if name == "LQR":
                u = np.clip(lqr.compute(x, x_ref), -1.0, 1.0) if "LQR" in name else lqr.compute(x, x_ref)
                # For fair comparison, clip LQR too
                u = lqr.compute(x, x_ref)
            else:
                u = ctrl.compute(x, x_ref)
            u = np.atleast_1d(u).flatten()[:1]
            u_hist[k] = u
            x = Ad @ x + Bd @ u
        results[name] = (x_hist, u_hist)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    colors = {'Constrained MPC': 'b', 'Unconstrained MPC': 'g', 'LQR': 'r'}

    for name, (x_h, u_h) in results.items():
        t = np.arange(steps) * dt
        axes[0].plot(t, x_h[:, 0], color=colors[name], linewidth=1.5, label=name)
        axes[1].plot(t, x_h[:, 1], color=colors[name], linewidth=1.5, label=name)
        axes[2].plot(t, u_h[:, 0], color=colors[name], linewidth=1.5, label=name)

    # Constraint lines
    axes[0].axhline(y=3.0, color='orange', linestyle=':', linewidth=2, label='State constraint')
    axes[0].axhline(y=-3.0, color='orange', linestyle=':', linewidth=2)
    axes[2].axhline(y=1.0, color='orange', linestyle=':', linewidth=2, label='Input constraint')
    axes[2].axhline(y=-1.0, color='orange', linestyle=':', linewidth=2)

    axes[0].set_title("Position")
    axes[1].set_title("Velocity")
    axes[2].set_title("Control Input")
    for ax in axes:
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("Time [s]")

    fig.suptitle("Task 5c: Constrained vs Unconstrained MPC", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task5c_constrained.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task5c_constrained.png")


def task5d_horizon_effect():
    """Effect of prediction horizon length."""
    print("\nHorizon Length Effect")
    print("-" * 40)

    A = np.array([[0, 1], [0, 0]])
    B = np.array([[0], [1]])
    sys = LinearSystem(A, B)
    dt = 0.1
    Ad, Bd, _, _ = sys.discretize(dt)

    Q = np.diag([10.0, 1.0])
    R = np.array([[0.1]])
    x0 = np.array([2.5, 0.0])
    x_ref = np.array([0.0, 0.0])

    N_values = [3, 5, 10, 20, 30]
    T = 6.0
    steps = int(T / dt)

    settling_times = []
    total_costs = []
    comp_times = []

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    colors = plt.cm.plasma(np.linspace(0, 1, len(N_values)))

    print(f"  {'N':>4} {'Settle [s]':>12} {'Total Cost':>12} {'Comp Time [ms]':>16}")
    print("  " + "-" * 48)

    for N_val, color in zip(N_values, colors):
        mpc = MPCController(Ad, Bd, Q, R, N=N_val,
                            u_min=np.array([-1.0]), u_max=np.array([1.0]))
        x = x0.copy()
        x_hist = np.zeros((steps, 2))
        u_hist = np.zeros((steps, 1))
        total_cost = 0.0

        t_start = time.time()
        for k in range(steps):
            x_hist[k] = x
            u = mpc.compute(x, x_ref)
            u = np.atleast_1d(u).flatten()[:1]
            u_hist[k] = u
            total_cost += (x @ Q @ x + u @ R @ u) * dt
            x = Ad @ x + Bd @ u
        comp_time = (time.time() - t_start) / steps * 1000  # ms per step

        t = np.arange(steps) * dt
        axes[0].plot(t, x_hist[:, 0], color=color, linewidth=1.5, label=f'N={N_val}')

        # Settling time
        settled = np.where(np.abs(x_hist[:, 0]) > 0.05)[0]
        st = t[settled[-1]] if len(settled) > 0 else 0

        settling_times.append(st)
        total_costs.append(total_cost)
        comp_times.append(comp_time)

        print(f"  {N_val:>4} {st:>12.2f} {total_cost:>12.3f} {comp_time:>16.3f}")

    axes[0].set_title("Position Response vs Horizon N")
    axes[0].set_xlabel("Time [s]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    ax2 = axes[1]
    ax2.plot(N_values, total_costs, 'bo-', linewidth=2, label='Total Cost')
    ax2.set_xlabel("Horizon N")
    ax2.set_ylabel("Total Cost", color='b')
    ax2_twin = ax2.twinx()
    ax2_twin.plot(N_values, comp_times, 'rs-', linewidth=2, label='Comp Time')
    ax2_twin.set_ylabel("Computation Time [ms/step]", color='r')
    ax2.set_title("Cost & Computation vs Horizon")
    ax2.grid(True, alpha=0.3)

    fig.suptitle("Task 5d: Effect of Prediction Horizon", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task5d_horizon.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task5d_horizon.png")


def main():
    print("=" * 70)
    print("Task 5 SOLUTION: MPC Basics")
    print("=" * 70)

    task5a_mpc_formulation()
    task5b_receding_horizon()
    task5c_constrained_mpc()
    task5d_horizon_effect()

    print("\n[DONE] All Task 5 solutions complete.")


if __name__ == '__main__':
    main()
