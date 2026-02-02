#!/usr/bin/env python3
"""
Task 1 Solution: PID Control Analysis
=======================================
Week 11 - Control Systems: PID to MPC
Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
from control_sim import LinearSystem, PIDController, simulate_system, plot_response, rk4_step


def create_test_plant():
    """Second-order underdamped plant."""
    wn = 2.0
    zeta = 0.3
    A = np.array([[0, 1], [-wn**2, -2*zeta*wn]])
    B = np.array([[0], [wn**2]])
    return LinearSystem(A, B), wn, zeta


class PIDWrapper:
    """Wraps PIDController for use with simulate_system."""
    def __init__(self, pid, state_idx=0):
        self.pid = pid
        self.idx = state_idx
    def compute(self, x, x_ref):
        error = x_ref[self.idx] - x[self.idx]
        return np.array([self.pid.compute(error)])


def compute_metrics(t, y, y_ref=1.0):
    """Compute step response metrics."""
    y_pos = y[:, 0] if y.ndim > 1 else y
    # Rise time (10% to 90%)
    try:
        t10 = t[np.where(y_pos >= 0.1 * y_ref)[0][0]]
        t90 = t[np.where(y_pos >= 0.9 * y_ref)[0][0]]
        rise_time = t90 - t10
    except IndexError:
        rise_time = np.inf

    # Overshoot
    peak = np.max(y_pos)
    overshoot = max(0, (peak - y_ref) / y_ref * 100)

    # Settling time (within 2%)
    settled = np.abs(y_pos - y_ref) <= 0.02 * abs(y_ref)
    if np.any(settled):
        # Find last time it was NOT settled
        not_settled = np.where(~settled)[0]
        if len(not_settled) > 0:
            settling_time = t[not_settled[-1]] if not_settled[-1] < len(t) - 1 else t[-1]
        else:
            settling_time = t[0]
    else:
        settling_time = np.inf

    return rise_time, overshoot, settling_time


def task1a_step_response():
    """PID step response analysis with three controller configurations."""
    print("Analyzing PID step responses...")
    plant, wn, zeta = create_test_plant()
    dt = 0.01
    T = 10.0

    configs = [
        ("P-only (kp=1.0)", PIDController(kp=1.0, ki=0.0, kd=0.0, dt=dt)),
        ("PI (kp=1.0, ki=0.5)", PIDController(kp=1.0, ki=0.5, kd=0.0, dt=dt)),
        ("PID (kp=1.5, ki=1.0, kd=0.3)", PIDController(kp=1.5, ki=1.0, kd=0.3, dt=dt)),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    print(f"\n{'Controller':<30} {'Rise Time':>10} {'Overshoot':>10} {'Settling':>10}")
    print("-" * 65)

    for name, pid in configs:
        pid.reset()
        ctrl = PIDWrapper(pid)
        t, x, u, y = simulate_system(plant, ctrl, [0, 0], [1, 0], dt, T)

        axes[0, 0].plot(t, x[:, 0], linewidth=1.5, label=name)
        axes[0, 1].plot(t, x[:, 1], linewidth=1.5, label=name)
        axes[1, 0].plot(t, u[:, 0], linewidth=1.5, label=name)

        rt, os_pct, st = compute_metrics(t, x[:, 0], 1.0)
        print(f"{name:<30} {rt:>10.3f}s {os_pct:>9.1f}% {st:>10.3f}s")

        # Error plot
        axes[1, 1].plot(t, 1.0 - x[:, 0], linewidth=1.5, label=name)

    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    axes[0, 0].set_title("Position (Output)")
    axes[0, 0].axhline(y=1.0, color='k', linestyle='--', alpha=0.5)
    axes[0, 0].set_ylabel("Position")
    axes[0, 1].set_title("Velocity")
    axes[0, 1].set_ylabel("Velocity")
    axes[1, 0].set_title("Control Input")
    axes[1, 0].set_ylabel("Force")
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 1].set_title("Tracking Error")
    axes[1, 1].set_ylabel("Error")
    axes[1, 1].set_xlabel("Time [s]")

    fig.suptitle("Task 1a: PID Step Response Comparison", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task1a_step_response.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task1a_step_response.png")


def task1b_ziegler_nichols():
    """Ziegler-Nichols tuning method."""
    print("\nApplying Ziegler-Nichols tuning...")
    plant, wn, zeta = create_test_plant()
    dt = 0.01
    T = 10.0

    # For the plant G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
    # Closed-loop with P-only: characteristic eq = s^2 + 2*zeta*wn*s + wn^2*(1+Ku) = 0
    # For sustained oscillation: real part = 0 -> 2*zeta*wn = 0 (not possible with P-only on 2nd order)
    # Use approximation: increase Ku until oscillation, measure period

    # Analytical: for this plant, Ku where marginal stability occurs
    # char eq: s^2 + 2*zeta*wn*s + wn^2 + Ku*wn^2 = 0
    # Routh criterion: marginal when 2*zeta*wn = 0 -> never happens for this system
    # So we approximate numerically

    # Search for Ku by finding gain where overshoot is very large
    best_ku = None
    for ku_test in np.arange(0.1, 50.0, 0.1):
        pid = PIDController(kp=ku_test, ki=0.0, kd=0.0, dt=dt)
        pid.reset()
        ctrl = PIDWrapper(pid)
        t, x, u, y = simulate_system(plant, ctrl, [0, 0], [1, 0], dt, 5.0)

        # Check if oscillating (not diverging, not converging)
        last_quarter = x[int(len(x)*0.75):, 0]
        if np.std(last_quarter) > 0.1 and np.max(np.abs(last_quarter)) < 100:
            best_ku = ku_test
            # Estimate period from zero crossings of error
            error = 1.0 - x[:, 0]
            crossings = np.where(np.diff(np.sign(error)))[0]
            if len(crossings) >= 4:
                Tu = np.mean(np.diff(t[crossings[-4:]])) * 2
                break

    if best_ku is None:
        # Fallback: use a reasonable estimate
        best_ku = 8.0
        Tu = 2 * np.pi / wn
        print(f"  Using estimated Ku={best_ku:.1f}, Tu={Tu:.3f}s")
    else:
        print(f"  Found Ku={best_ku:.1f}, Tu={Tu:.3f}s")

    # Ziegler-Nichols PID rules
    kp_zn = 0.6 * best_ku
    ki_zn = kp_zn / (0.5 * Tu)
    kd_zn = kp_zn * 0.125 * Tu

    print(f"  Z-N PID: kp={kp_zn:.3f}, ki={ki_zn:.3f}, kd={kd_zn:.3f}")

    # Compare Z-N with manual tuning
    pid_zn = PIDController(kp=kp_zn, ki=ki_zn, kd=kd_zn, dt=dt)
    pid_manual = PIDController(kp=1.5, ki=1.0, kd=0.3, dt=dt)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    for name, pid in [("Ziegler-Nichols", pid_zn), ("Manual Tuning", pid_manual)]:
        pid.reset()
        ctrl = PIDWrapper(pid)
        t, x, u, y = simulate_system(plant, ctrl, [0, 0], [1, 0], dt, T)
        axes[0].plot(t, x[:, 0], linewidth=1.5, label=name)
        axes[1].plot(t, u[:, 0], linewidth=1.5, label=name)

        rt, os_pct, st = compute_metrics(t, x[:, 0], 1.0)
        print(f"  {name}: rise={rt:.3f}s, OS={os_pct:.1f}%, settle={st:.3f}s")

    axes[0].axhline(y=1.0, color='k', linestyle='--', alpha=0.5, label='Reference')
    axes[0].set_title("Step Response")
    axes[0].set_xlabel("Time [s]")
    axes[0].set_ylabel("Position")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].set_title("Control Input")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Force")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Task 1b: Ziegler-Nichols vs Manual Tuning", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task1b_ziegler_nichols.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task1b_ziegler_nichols.png")


def task1c_frequency_analysis():
    """Frequency domain analysis of PID control loop."""
    print("\nFrequency domain analysis...")
    plant, wn, zeta = create_test_plant()

    # Open-loop transfer function: C(s) * G(s)
    # G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
    # C(s) = kp + ki/s + kd*s = (kd*s^2 + kp*s + ki)/s

    configs = [
        ("P-only (kp=1.5)", 1.5, 0.0, 0.0),
        ("PI (kp=1.5, ki=1.0)", 1.5, 1.0, 0.0),
        ("PID (kp=1.5, ki=1.0, kd=0.3)", 1.5, 1.0, 0.3),
    ]

    omega = np.logspace(-2, 2, 1000)
    s = 1j * omega

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    for name, kp, ki, kd in configs:
        # Plant
        G = wn**2 / (s**2 + 2*zeta*wn*s + wn**2)
        # Controller
        C = kp + ki / (s + 1e-10) + kd * s / (1 + 0.01 * s)  # with derivative filter
        # Open loop
        L = C * G

        mag_db = 20 * np.log10(np.abs(L) + 1e-30)
        phase_deg = np.angle(L, deg=True)

        axes[0].semilogx(omega, mag_db, linewidth=1.5, label=name)
        axes[1].semilogx(omega, phase_deg, linewidth=1.5, label=name)

        # Gain margin: phase = -180 deg
        phase_crossings = np.where(np.diff(np.sign(phase_deg + 180)))[0]
        if len(phase_crossings) > 0:
            idx_pc = phase_crossings[0]
            gm = -mag_db[idx_pc]
            print(f"  {name}: Gain margin = {gm:.1f} dB at w={omega[idx_pc]:.2f} rad/s")

        # Phase margin: magnitude = 0 dB
        gain_crossings = np.where(np.diff(np.sign(mag_db)))[0]
        if len(gain_crossings) > 0:
            idx_gc = gain_crossings[0]
            pm = 180 + phase_deg[idx_gc]
            print(f"  {name}: Phase margin = {pm:.1f} deg at w={omega[idx_gc]:.2f} rad/s")

    axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    axes[0].set_ylabel("Magnitude [dB]")
    axes[0].set_title("Bode Plot - Magnitude")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3, which='both')
    axes[1].axhline(y=-180, color='k', linestyle='--', alpha=0.5)
    axes[1].set_ylabel("Phase [deg]")
    axes[1].set_xlabel("Frequency [rad/s]")
    axes[1].set_title("Bode Plot - Phase")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3, which='both')

    fig.suptitle("Task 1c: Frequency Response Analysis", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task1c_frequency.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task1c_frequency.png")


def task1d_antiwindup_comparison():
    """Compare PID with and without anti-windup."""
    print("\nAnti-windup comparison...")
    plant, wn, zeta = create_test_plant()
    dt = 0.01
    T = 15.0

    # PID with high integral gain and output saturation
    pid_windup = PIDController(kp=2.0, ki=5.0, kd=0.3, dt=dt,
                               output_limits=(-5, 5),
                               integrator_limits=(-1e6, 1e6))  # No anti-windup
    pid_antiwindup = PIDController(kp=2.0, ki=5.0, kd=0.3, dt=dt,
                                   output_limits=(-5, 5),
                                   integrator_limits=(-10, 10))  # With anti-windup

    # Simulate with a step that causes saturation, then step down
    def ref_signal(t):
        if t < 5:
            return np.array([5.0, 0.0])
        else:
            return np.array([1.0, 0.0])

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    for name, pid, color in [("Without Anti-Windup", pid_windup, 'r'),
                              ("With Anti-Windup", pid_antiwindup, 'b')]:
        pid.reset()
        ctrl = PIDWrapper(pid)
        t, x, u, y = simulate_system(plant, ctrl, [0, 0], ref_signal, dt, T)

        ref_vals = np.array([ref_signal(ti)[0] for ti in t])

        axes[0, 0].plot(t, x[:, 0], color=color, linewidth=1.5, label=name)
        axes[0, 1].plot(t, u[:, 0], color=color, linewidth=1.5, label=name)
        axes[1, 0].plot(t, ref_vals - x[:, 0], color=color, linewidth=1.5, label=name)

        # Track integrator state manually
        integral_vals = []
        pid.reset()
        err_prev = 0
        integ = 0
        for i in range(len(t)):
            err = ref_signal(t[i])[0] - x[i, 0]
            integ += err * dt
            if hasattr(pid, 'integrator_limits'):
                integ_clipped = np.clip(integ, *pid.integrator_limits)
            else:
                integ_clipped = integ
            integral_vals.append(integ_clipped)
        axes[1, 1].plot(t, integral_vals, color=color, linewidth=1.5, label=name)

    axes[0, 0].plot(t, [ref_signal(ti)[0] for ti in t], 'k--', alpha=0.5, label='Reference')
    axes[0, 0].set_title("Position Response")
    axes[0, 0].set_ylabel("Position")
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].axhline(y=5, color='gray', linestyle=':', alpha=0.5, label='Saturation')
    axes[0, 1].axhline(y=-5, color='gray', linestyle=':', alpha=0.5)
    axes[0, 1].set_title("Control Input (saturated)")
    axes[0, 1].set_ylabel("Force")
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].set_title("Tracking Error")
    axes[1, 0].set_ylabel("Error")
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].set_title("Integrator State")
    axes[1, 1].set_ylabel("Integral Value")
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True, alpha=0.3)

    fig.suptitle("Task 1d: Anti-Windup Comparison", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('task1d_antiwindup.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("[INFO] Saved task1d_antiwindup.png")

    print("\n  ANALYSIS: Without anti-windup, the integrator accumulates large values")
    print("  during saturation. When the reference drops, the large integrator causes")
    print("  overshoot and slow recovery ('integrator windup'). Anti-windup clamps")
    print("  the integrator, enabling faster response to reference changes.")


def main():
    print("=" * 70)
    print("Task 1 SOLUTION: PID Control Analysis")
    print("=" * 70)

    task1a_step_response()
    task1b_ziegler_nichols()
    task1c_frequency_analysis()
    task1d_antiwindup_comparison()

    print("\n[DONE] All Task 1 solutions complete.")


if __name__ == '__main__':
    main()
