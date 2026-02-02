#!/usr/bin/env python3
"""
Task 7 Solution: Full Controller Comparison - PID vs LQR vs MPC
=================================================================
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
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
from control_sim import (LinearSystem, PIDController, LQRController, MPCController,
                         simulate_system, plot_response)


# Altitude model: z, vz (2 states, 1 input = thrust/mass)
A_ALT = np.array([[0, 1], [0, 0]])
B_ALT = np.array([[0], [1.0/1.5]])  # mass = 1.5 kg
DT = 0.05


class PIDWrapper:
    def __init__(self, pid, state_idx=0):
        self.pid = pid
        self.idx = state_idx
    def compute(self, x, x_ref):
        error = x_ref[self.idx] - x[self.idx]
        return np.array([self.pid.compute(error)])


def setup_controllers(constrained=False):
    """Create PID, LQR, MPC for altitude control."""
    sys_alt = LinearSystem(A_ALT, B_ALT)
    Ad, Bd, _, _ = sys_alt.discretize(DT)

    pid = PIDController(kp=8.0, ki=3.0, kd=4.0, dt=DT,
                        output_limits=(-15, 15), integrator_limits=(-20, 20))

    Q = np.diag([100, 10])
    R = np.array([[0.5]])
    lqr = LQRController(A_ALT, B_ALT, Q, R)

    u_lim = 3.0 if constrained else 50.0
    mpc = MPCController(Ad, Bd, Q, R, N=15,
                        u_min=np.array([-u_lim]), u_max=np.array([u_lim]))

    return pid, lqr, mpc, Ad, Bd


def run_simulation(controller, Ad, Bd, x0, x_ref_fn, T, name=""):
    """Run discrete-time simulation."""
    steps = int(T / DT)
    x = np.array(x0, dtype=float).flatten()
    n, m = Ad.shape[0], Bd.shape[1]

    t_hist = np.zeros(steps)
    x_hist = np.zeros((steps, n))
    u_hist = np.zeros((steps, m))

    t_start = time.time()
    for k in range(steps):
        t_k = k * DT
        t_hist[k] = t_k
        x_hist[k] = x

        if callable(x_ref_fn):
            ref = x_ref_fn(t_k)
        else:
            ref = np.array(x_ref_fn, dtype=float).flatten()

        u = controller.compute(x, ref)
        u = np.atleast_1d(u).flatten()[:m]
        u_hist[k] = u
        x = Ad @ x + Bd @ u

    comp_time = (time.time() - t_start) / steps * 1000
    return t_hist, x_hist, u_hist, comp_time


def compute_metrics(t, x, u, ref_val=5.0):
    """Compute comprehensive performance metrics."""
    pos = x[:, 0]

    # Rise time (10% to 90%)
    target = ref_val
    try:
        t10 = t[np.where(pos >= 0.1 * target)[0][0]] if target > 0 else np.inf
        t90 = t[np.where(pos >= 0.9 * target)[0][0]] if target > 0 else np.inf
        rise_time = t90 - t10
    except IndexError:
        rise_time = np.inf

    # Overshoot
    peak = np.max(pos)
    overshoot = max(0, (peak - target) / target * 100) if target != 0 else 0

    # Settling time (2% band)
    settled = np.abs(pos - target) <= 0.02 * abs(target)
    not_settled = np.where(~settled)[0]
    settling_time = t[not_settled[-1]] if len(not_settled) > 0 and not_settled[-1] < len(t)-1 else 0

    # RMSE (after transient)
    steady_idx = int(len(t) * 0.3)
    rmse = np.sqrt(np.mean((pos[steady_idx:] - target)**2))

    # Control effort
    effort = np.sum(u[:, 0]**2) * DT

    return {
        'rise_time': rise_time,
        'overshoot': overshoot,
        'settling_time': settling_time,
        'rmse': rmse,
        'effort': effort
    }


def task7a_hover_comparison():
    """Compare PID, LQR, MPC for hover."""
    print("\nHover Stabilization Comparison")
    print("-" * 40)

    pid, lqr, mpc, Ad, Bd = setup_controllers()

    x0 = np.array([0, 0])
    x_ref = np.array([5.0, 0.0])
    T = 10.0

    controllers = {
        'PID': PIDWrapper(pid),
        'LQR': lqr,
        'MPC': mpc,
    }
    colors = {'PID': 'b', 'LQR': 'r', 'MPC': 'g'}

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    results = {}

    for name, ctrl in controllers.items():
        if hasattr(ctrl, 'pid'):
            ctrl.pid.reset()
        t, x, u, comp = run_simulation(ctrl, Ad, Bd, x0, x_ref, T, name)
        metrics = compute_metrics(t, x, u, 5.0)
        metrics['comp_time'] = comp
        results[name] = metrics

        axes[0].plot(t, x[:, 0], color=colors[name], linewidth=1.5, label=name)
        axes[1].plot(t, x_ref[0] - x[:, 0], color=colors[name], linewidth=1.5, label=name)
        axes[2].plot(t, u[:, 0], color=colors[name], linewidth=1.5, label=name)

    axes[0].axhline(y=5.0, color='k', linestyle='--', alpha=0.5)
    axes[0].set_title("Altitude [m]")
    axes[0].set_xlabel("Time [s]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_title("Tracking Error [m]")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_title("Thrust Command [N]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    fig.suptitle("Task 7a: Hover Comparison - PID vs LQR vs MPC", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task7a_hover_comparison.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task7a_hover_comparison.png")

    print(f"\n  {'Metric':<20} {'PID':>10} {'LQR':>10} {'MPC':>10}")
    print("  " + "-" * 52)
    for key in ['rise_time', 'overshoot', 'settling_time', 'rmse', 'effort', 'comp_time']:
        unit = 's' if 'time' in key else ('%' if key == 'overshoot' else ('ms' if 'comp' in key else ''))
        vals = [f"{results[c][key]:.3f}{unit}" for c in ['PID', 'LQR', 'MPC']]
        print(f"  {key:<20} {vals[0]:>10} {vals[1]:>10} {vals[2]:>10}")

    return results


def task7b_tracking_comparison():
    """Compare tracking performance."""
    print("\nSinusoidal Tracking Comparison")
    print("-" * 40)

    pid, lqr, mpc, Ad, Bd = setup_controllers()

    def ref_fn(t_val):
        return np.array([5.0 + 2.0 * np.sin(0.5 * t_val), 2.0 * 0.5 * np.cos(0.5 * t_val)])

    T = 20.0
    controllers = {'PID': PIDWrapper(pid), 'LQR': lqr, 'MPC': mpc}
    colors = {'PID': 'b', 'LQR': 'r', 'MPC': 'g'}

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    print(f"  {'Controller':<10} {'Tracking RMSE':>15} {'Max Error':>12}")
    print("  " + "-" * 40)

    for name, ctrl in controllers.items():
        if hasattr(ctrl, 'pid'):
            ctrl.pid.reset()
        t, x, u, comp = run_simulation(ctrl, Ad, Bd, [5, 0], ref_fn, T, name)

        ref_vals = np.array([ref_fn(ti)[0] for ti in t])
        error = ref_vals - x[:, 0]

        axes[0].plot(t, x[:, 0], color=colors[name], linewidth=1.5, label=name)
        axes[1].plot(t, np.abs(error), color=colors[name], linewidth=1.5, label=name)

        # Skip transient
        ss_idx = int(5.0 / DT)
        rmse = np.sqrt(np.mean(error[ss_idx:]**2))
        max_err = np.max(np.abs(error[ss_idx:]))
        print(f"  {name:<10} {rmse:>15.4f} m {max_err:>12.4f} m")

    ref_t = np.arange(int(T/DT)) * DT
    ref_line = [ref_fn(ti)[0] for ti in ref_t]
    axes[0].plot(ref_t, ref_line, 'k--', linewidth=1, alpha=0.5, label='Reference')
    axes[0].set_title("Altitude Tracking")
    axes[0].set_xlabel("Time [s]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_title("|Tracking Error|")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Task 7b: Tracking Comparison", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task7b_tracking_comparison.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task7b_tracking_comparison.png")


def task7c_constrained_comparison():
    """Compare under tight constraints."""
    print("\nConstrained Comparison (|u| <= 3 N)")
    print("-" * 40)

    pid_raw, lqr, mpc, Ad, Bd = setup_controllers(constrained=True)
    pid = PIDController(kp=8.0, ki=3.0, kd=4.0, dt=DT,
                        output_limits=(-3, 3), integrator_limits=(-5, 5))

    x0 = np.array([0, 0])
    x_ref = np.array([10.0, 0.0])
    T = 15.0

    class LQRClipped:
        def __init__(self, lqr_ctrl, clip_val):
            self.lqr = lqr_ctrl
            self.clip = clip_val
        def compute(self, x, x_ref):
            u = self.lqr.compute(x, x_ref)
            return np.clip(u, -self.clip, self.clip)

    controllers = {
        'PID (clipped)': PIDWrapper(pid),
        'LQR (clipped)': LQRClipped(lqr, 3.0),
        'MPC (constrained)': mpc,
    }
    colors = {'PID (clipped)': 'b', 'LQR (clipped)': 'r', 'MPC (constrained)': 'g'}

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    for name, ctrl in controllers.items():
        if hasattr(ctrl, 'pid'):
            ctrl.pid.reset()
        t, x, u, comp = run_simulation(ctrl, Ad, Bd, x0, x_ref, T, name)

        axes[0].plot(t, x[:, 0], color=colors[name], linewidth=1.5, label=name)
        axes[1].plot(t, u[:, 0], color=colors[name], linewidth=1.5, label=name)

        ss_idx = int(len(t) * 0.5)
        rmse = np.sqrt(np.mean((x[ss_idx:, 0] - 10.0)**2))
        print(f"  {name:<22} Steady-state RMSE: {rmse:.4f} m, Peak |u|: {np.max(np.abs(u)):.3f}")

    axes[0].axhline(y=10.0, color='k', linestyle='--', alpha=0.5, label='Reference')
    axes[0].set_title("Altitude [m]")
    axes[0].set_xlabel("Time [s]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].axhline(y=3.0, color='orange', linestyle=':', linewidth=2, label='Constraint')
    axes[1].axhline(y=-3.0, color='orange', linestyle=':', linewidth=2)
    axes[1].set_title("Control Input [N]")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Task 7c: Constrained Comparison (|u| <= 3 N)", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task7c_constrained.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task7c_constrained.png")
    print("  NOTE: MPC anticipates constraints and plans smoother trajectory.")


def task7d_disturbance_comparison():
    """Compare disturbance rejection."""
    print("\nDisturbance Rejection Comparison")
    print("-" * 40)

    pid_raw, lqr, mpc, Ad, Bd = setup_controllers()
    pid = PIDController(kp=8.0, ki=3.0, kd=4.0, dt=DT,
                        output_limits=(-15, 15), integrator_limits=(-20, 20))

    x_ref = np.array([5.0, 0.0])
    T = 15.0
    steps = int(T / DT)

    def disturbance(t_val):
        d = np.zeros(2)
        if 5.0 <= t_val < 7.0:
            d[1] = -3.0 / 1.5  # step gust
        return d

    controllers = {
        'PID': PIDWrapper(pid),
        'LQR': lqr,
        'MPC': mpc,
    }
    colors = {'PID': 'b', 'LQR': 'r', 'MPC': 'g'}

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    # Disturbance
    t_arr = np.arange(steps) * DT
    d_vals = [disturbance(ti)[1] for ti in t_arr]
    axes[0].plot(t_arr, d_vals, 'k-', linewidth=1.5)
    axes[0].set_title("Disturbance")
    axes[0].set_xlabel("Time [s]")
    axes[0].grid(True, alpha=0.3)

    for name, ctrl in controllers.items():
        if hasattr(ctrl, 'pid'):
            ctrl.pid.reset()
        x = np.array([5.0, 0.0])
        x_hist = np.zeros((steps, 2))
        u_hist = np.zeros((steps, 1))

        for k in range(steps):
            x_hist[k] = x
            u = ctrl.compute(x, x_ref)
            u = np.atleast_1d(u).flatten()[:1]
            u_hist[k] = u
            d = disturbance(k * DT)
            x = Ad @ x + Bd @ u + d * DT

        axes[1].plot(t_arr, x_hist[:, 0], color=colors[name], linewidth=1.5, label=name)
        axes[2].plot(t_arr, u_hist[:, 0], color=colors[name], linewidth=1.5, label=name)

        max_dev = np.max(np.abs(x_hist[:, 0] - 5.0))
        # Recovery time
        dist_end = int(7.0 / DT)
        recovered = np.where(np.abs(x_hist[dist_end:, 0] - 5.0) > 0.1)[0]
        rec_time = recovered[-1] * DT if len(recovered) > 0 else 0
        print(f"  {name:<10} Max deviation: {max_dev:.4f} m, Recovery time: {rec_time:.2f} s")

    axes[1].axhline(y=5.0, color='k', linestyle='--', alpha=0.5)
    axes[1].set_title("Altitude Response")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_title("Control Input")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    fig.suptitle("Task 7d: Disturbance Rejection Comparison", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task7d_disturbance.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task7d_disturbance.png")


def task7e_summary_table():
    """Generate comprehensive summary."""
    print("\nComprehensive Summary")
    print("-" * 40)

    summary = """
====================================================================
         CONTROLLER COMPARISON SUMMARY - Week 11
====================================================================

Scenario         | PID       | LQR       | MPC
-----------------+-----------+-----------+-----------
Hover            |           |           |
  Rise Time      | Medium    | Fast      | Fast
  Overshoot      | Possible  | Minimal   | Minimal
  Tuning         | 3 params  | Q, R mat  | Q, R, N

Tracking         |           |           |
  Sine RMSE      | Highest   | Medium    | Lowest*
  Phase Lag      | Yes       | Small     | Predictive

Constrained      |           |           |
  Handles well?  | Clip only | Clip only | Native
  Performance    | Worst     | Medium    | Best

Disturbance      |           |           |
  Recovery       | Slow      | Fast      | Fast
  Max Deviation  | Largest   | Medium    | Smallest*

Computation      |           |           |
  Per step       | ~0.001 ms | ~0.01 ms  | ~1-10 ms
  Real-time OK?  | Always    | Always    | Depends on N

* MPC advantage is most pronounced with constraints.
  Without constraints, LQR ~ MPC as horizon -> infinity.

RECOMMENDATION:
- Simple hover/setpoint: PID is sufficient
- Moderate performance: LQR (optimal, guaranteed margins)
- Constraints + trajectory: MPC (best performance)
====================================================================
"""
    print(summary)

    with open('task7e_summary.txt', 'w') as f:
        f.write(summary)
    print("[INFO] Saved task7e_summary.txt")


def main():
    print("=" * 70)
    print("Task 7 SOLUTION: Full Controller Comparison")
    print("=" * 70)

    task7a_hover_comparison()
    task7b_tracking_comparison()
    task7c_constrained_comparison()
    task7d_disturbance_comparison()
    task7e_summary_table()

    print("\n[DONE] All Task 7 solutions complete.")
    print("  MPC > LQR > PID for constrained trajectory tracking (as expected).")


if __name__ == '__main__':
    main()
