#!/usr/bin/env python3
"""
Task 2 Solution: Quadrotor Dynamics Model
==========================================
Full implementation and verification of quadrotor equations of motion.
"""
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from quadrotor_sim import (QuadrotorParams, euler_to_rotation, motor_mixing_matrix,
                           quadrotor_dynamics, rk4_step, hover_motor_speed)


def compute_thrust_torque(motor_speeds, params):
    """Compute total thrust and torques from motor speeds using mixing matrix."""
    w_sq = np.array(motor_speeds) ** 2
    M = motor_mixing_matrix(params)
    wrench = M @ w_sq
    return wrench[0], wrench[1:]


def translational_dynamics(state, R, thrust, params):
    """Compute translational acceleration in world frame."""
    thrust_world = R @ np.array([0, 0, thrust])
    gravity = np.array([0, 0, -params.mass * params.g])
    acc = (thrust_world + gravity) / params.mass
    return acc


def rotational_dynamics(state, torques, params):
    """Compute angular acceleration using Euler's equation."""
    p, q, r = state[9], state[10], state[11]
    omega = np.array([p, q, r])
    I = np.array([params.I_xx, params.I_yy, params.I_zz])
    I_omega = I * omega
    omega_dot = (torques - np.cross(omega, I_omega)) / I
    return omega_dot


def full_dynamics(state, motor_speeds, params):
    """Compute complete state derivative."""
    phi, theta, psi = state[6], state[7], state[8]
    p, q, r = state[9], state[10], state[11]

    thrust, torques = compute_thrust_torque(motor_speeds, params)
    R = euler_to_rotation(phi, theta, psi)
    acc = translational_dynamics(state, R, thrust, params)
    omega_dot = rotational_dynamics(state, torques, params)

    # Euler rate kinematics
    cp, sp = np.cos(phi), np.sin(phi)
    ct = np.cos(theta)
    tt = np.tan(theta)
    euler_dot = np.array([
        p + sp*tt*q + cp*tt*r,
        cp*q - sp*r,
        (sp/ct)*q + (cp/ct)*r,
    ])

    state_dot = np.zeros(12)
    state_dot[0:3] = state[3:6]   # vel
    state_dot[3:6] = acc
    state_dot[6:9] = euler_dot
    state_dot[9:12] = omega_dot
    return state_dot


def main():
    print("=" * 60)
    print("Task 2 Solution: Quadrotor Dynamics Model")
    print("=" * 60)

    params = QuadrotorParams()
    state0 = np.zeros(12)
    state0[2] = 10.0  # Start at 10m height

    # Test 1: Free fall
    print("\n--- Test 1: Free Fall (all motors off) ---")
    sd = full_dynamics(state0, [0, 0, 0, 0], params)
    print(f"  Acceleration: [{sd[3]:.4f}, {sd[4]:.4f}, {sd[5]:.4f}] m/s^2")
    print(f"  Expected:     [0.0000, 0.0000, {-params.g:.4f}] m/s^2")
    err = abs(sd[5] + params.g)
    print(f"  Error: {err:.2e} {'PASS' if err < 1e-10 else 'FAIL'}")

    # Test 2: Hover
    print("\n--- Test 2: Hover (equal motor speeds) ---")
    w_hover = hover_motor_speed(params)
    print(f"  Hover motor speed: {w_hover:.2f} rad/s")
    thrust, torques = compute_thrust_torque([w_hover]*4, params)
    print(f"  Total thrust: {thrust:.4f} N (weight: {params.mass*params.g:.4f} N)")
    sd = full_dynamics(state0, [w_hover]*4, params)
    print(f"  Acceleration: [{sd[3]:.6f}, {sd[4]:.6f}, {sd[5]:.6f}] m/s^2")
    err = np.linalg.norm(sd[3:6])
    print(f"  |acceleration|: {err:.2e} {'PASS' if err < 1e-8 else 'FAIL'}")

    # Test 3: Single motor
    print("\n--- Test 3: Single Motor (motor 1 only at hover speed) ---")
    sd = full_dynamics(state0, [w_hover, 0, 0, 0], params)
    thrust1, torques1 = compute_thrust_torque([w_hover, 0, 0, 0], params)
    print(f"  Thrust: {thrust1:.4f} N (1/4 of hover: {params.mass*params.g/4:.4f} N)")
    print(f"  Torques: tau_x={torques1[0]:.6f}, tau_y={torques1[1]:.6f}, tau_z={torques1[2]:.6f}")
    print(f"  Motor 1 is front (+x, CW): should produce positive pitch torque (tau_y > 0)")
    print(f"  and negative yaw torque (tau_z < 0 for CW)")

    # Test 4: Simulate free fall for 2 seconds
    print("\n--- Test 4: Free Fall Simulation (2 seconds) ---")
    dt = 0.001
    t_final = 2.0
    N = int(t_final / dt)
    state = state0.copy()
    times = []
    heights = []
    vels = []
    for i in range(N):
        times.append(i * dt)
        heights.append(state[2])
        vels.append(state[5])
        state = rk4_step(state, np.zeros(4), params, dt)
    times.append(N * dt)
    heights.append(state[2])
    vels.append(state[5])

    expected_h = 10.0 - 0.5 * params.g * t_final**2
    expected_v = -params.g * t_final
    print(f"  Final height: {state[2]:.4f} m (expected: {expected_h:.4f} m)")
    print(f"  Final velocity: {state[5]:.4f} m/s (expected: {expected_v:.4f} m/s)")
    h_err = abs(state[2] - expected_h)
    v_err = abs(state[5] - expected_v)
    print(f"  Height error: {h_err:.2e} {'PASS' if h_err < 1e-6 else 'FAIL'}")
    print(f"  Velocity error: {v_err:.2e} {'PASS' if v_err < 1e-6 else 'FAIL'}")

    # Test 5: Hover simulation
    print("\n--- Test 5: Hover Simulation (2 seconds) ---")
    state = state0.copy()
    hover_heights = []
    hover_times = []
    for i in range(N):
        hover_times.append(i * dt)
        hover_heights.append(state[2])
        state = rk4_step(state, np.array([w_hover]*4), params, dt)
    hover_times.append(N * dt)
    hover_heights.append(state[2])
    drift = abs(state[2] - 10.0)
    print(f"  Height after 2s hover: {state[2]:.6f} m (started at 10.0 m)")
    print(f"  Drift: {drift:.2e} m {'PASS' if drift < 1e-4 else 'FAIL'}")

    # Plot results
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    ax = axes[0, 0]
    ax.plot(times, heights, 'b-', label='Simulated')
    t_arr = np.array(times)
    ax.plot(t_arr, 10.0 - 0.5*params.g*t_arr**2, 'r--', label='Analytical')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Height (m)')
    ax.set_title('Free Fall: Height vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(times, vels, 'b-', label='Simulated')
    ax.plot(t_arr, -params.g*t_arr, 'r--', label='Analytical')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Free Fall: Vertical Velocity vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(hover_times, hover_heights, 'b-')
    ax.axhline(y=10.0, color='r', linestyle='--', label='Reference')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Height (m)')
    ax.set_title('Hover: Height vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Test 6: Plot torque from asymmetric motors
    ax = axes[1, 1]
    motor_configs = [
        ([w_hover, 0, 0, 0], 'Motor 1 only'),
        ([0, w_hover, 0, 0], 'Motor 2 only'),
        ([0, 0, w_hover, 0], 'Motor 3 only'),
        ([0, 0, 0, w_hover], 'Motor 4 only'),
    ]
    labels_t = ['tau_x', 'tau_y', 'tau_z']
    x_pos = np.arange(3)
    width = 0.2
    for idx, (motors, name) in enumerate(motor_configs):
        _, torques = compute_thrust_torque(motors, params)
        ax.bar(x_pos + idx*width, torques, width, label=name)
    ax.set_xticks(x_pos + 1.5*width)
    ax.set_xticklabels(labels_t)
    ax.set_ylabel('Torque (N*m)')
    ax.set_title('Torques from Individual Motors')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')

    plt.suptitle('Task 2: Quadrotor Dynamics Verification', fontsize=14)
    plt.tight_layout()
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task2_dynamics.png')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n  Plot saved to {save_path}")
    plt.close()

    print("\n" + "=" * 60)
    print("Task 2 Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
