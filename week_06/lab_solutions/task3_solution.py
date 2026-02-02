#!/usr/bin/env python3
"""
Task 3 Solution: PID Attitude Controller
==========================================
Cascaded PID for attitude stabilization with hover and step response tests.

Note: Step responses in roll/pitch are tested for short durations because
commanding a constant roll/pitch without position control causes lateral
drift and cross-coupling effects.
"""
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from quadrotor_sim import (QuadrotorParams, rk4_step, hover_motor_speed,
                           inverse_mixing, motor_mixing_matrix, simulate)


class PIDController:
    """PID controller with anti-windup."""

    def __init__(self, kp, ki, kd, integral_limit=1.0, output_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def update(self, error, dt, derivative=None):
        P = self.kp * error
        self.integral = np.clip(self.integral + error * dt, -self.integral_limit, self.integral_limit)
        I = self.ki * self.integral
        if derivative is not None:
            D = self.kd * derivative
        elif self.initialized:
            D = self.kd * (error - self.prev_error) / dt
        else:
            D = 0.0
            self.initialized = True
        self.prev_error = error
        return np.clip(P + I + D, -self.output_limit, self.output_limit)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False


def attitude_controller(desired_angles, current_angles, current_rates, pids, dt):
    """Cascaded attitude controller: outer angle loop + inner rate loop."""
    torques = np.zeros(3)
    axes = ['roll', 'pitch', 'yaw']
    for i, axis in enumerate(axes):
        angle_pid, rate_pid = pids[axis]
        angle_error = desired_angles[i] - current_angles[i]
        if i == 2:
            angle_error = (angle_error + np.pi) % (2*np.pi) - np.pi
        desired_rate = angle_pid.update(angle_error, dt)
        rate_error = desired_rate - current_rates[i]
        torques[i] = rate_pid.update(rate_error, dt)
    return torques


def altitude_controller(desired_z, current_z, current_vz, pid, dt, params):
    """Altitude PID with gravity feedforward."""
    z_error = desired_z - current_z
    thrust_cmd = pid.update(z_error, dt, derivative=-current_vz)
    thrust = params.mass * params.g + params.mass * thrust_cmd
    return max(thrust, 0.0)


def make_pids():
    """Create fresh set of attitude PIDs."""
    return {
        'roll':  (PIDController(3.5, 0.0, 0.0, integral_limit=1.0, output_limit=4.0),
                  PIDController(0.25, 0.1, 0.0, integral_limit=0.5, output_limit=1.0)),
        'pitch': (PIDController(3.5, 0.0, 0.0, integral_limit=1.0, output_limit=4.0),
                  PIDController(0.25, 0.1, 0.0, integral_limit=0.5, output_limit=1.0)),
        'yaw':   (PIDController(2.0, 0.0, 0.0, integral_limit=1.0, output_limit=3.0),
                  PIDController(0.2, 0.05, 0.0, integral_limit=0.5, output_limit=0.5)),
    }


def run_attitude_test(params, desired_angles_func, desired_z, state0, dt, t_final):
    """Run an attitude control test and return timeseries."""
    pids = make_pids()
    alt_pid = PIDController(5.0, 1.5, 4.0, integral_limit=5.0, output_limit=10.0)
    M_inv = inverse_mixing(params)

    def controller(t, state):
        desired_ang = desired_angles_func(t)
        torques = attitude_controller(desired_ang, state[6:9], state[9:12], pids, dt)
        thrust = altitude_controller(desired_z, state[2], state[5], alt_pid, dt, params)
        wrench = np.array([thrust, torques[0], torques[1], torques[2]])
        w_sq = M_inv @ wrench
        w_sq = np.maximum(w_sq, 0.0)
        motors = np.sqrt(w_sq)
        return np.clip(motors, params.motor_min, params.motor_max)

    return simulate(state0, controller, params, dt=dt, t_final=t_final)


def main():
    print("=" * 60)
    print("Task 3 Solution: PID Attitude Controller")
    print("=" * 60)

    params = QuadrotorParams()
    dt = 0.001

    # Test 1: Hover from slight initial tilt
    print("\n--- Test 1: Stabilize from initial tilt (2 deg roll) ---")
    state0 = np.zeros(12)
    state0[2] = 5.0
    state0[6] = np.radians(2)

    times1, states1, motors1 = run_attitude_test(
        params, lambda t: np.array([0.0, 0.0, 0.0]), 5.0, state0, dt, 2.0)

    # Evaluate at t=1.0s when attitude should have settled
    idx_eval = int(1.0 / dt)
    roll_at_1s = np.degrees(states1[idx_eval, 6])
    height_at_1s = states1[idx_eval, 2]
    print(f"  Roll at t=1.0s: {roll_at_1s:.4f} deg (should be ~0)")
    print(f"  Height at t=1.0s: {height_at_1s:.4f} m (should be ~5)")
    print(f"  Roll PASS: {abs(roll_at_1s) < 1.0}")
    print(f"  Height PASS: {abs(height_at_1s - 5.0) < 0.3}")
    print(f"  Note: Without position control, lateral drift occurs over time.")

    # Test 2: Roll step response -- brief pulse test
    # We command a 5 deg roll step at t=0.2, measure at t=1.5
    print("\n--- Test 2: Roll Step Response (5 deg) ---")
    state0 = np.zeros(12)
    state0[2] = 5.0
    step_angle = np.radians(5)

    times2, states2, motors2 = run_attitude_test(
        params, lambda t: np.array([step_angle if t > 0.2 else 0.0, 0.0, 0.0]),
        5.0, state0, dt, 2.0)

    # Evaluate: find the roll value around t=1.0 (should be near 5 deg)
    idx_1s = int(1.0 / dt)
    roll_at_1s = np.degrees(states2[idx_1s, 6])
    print(f"  Roll at t=1.0s: {roll_at_1s:.2f} deg (target: {np.degrees(step_angle):.1f})")
    print(f"  PASS: {abs(roll_at_1s - np.degrees(step_angle)) < 2.0}")

    # Test 3: Pitch step response
    print("\n--- Test 3: Pitch Step Response (5 deg) ---")
    times3, states3, motors3 = run_attitude_test(
        params, lambda t: np.array([0.0, step_angle if t > 0.2 else 0.0, 0.0]),
        5.0, state0, dt, 2.0)

    pitch_at_1s = np.degrees(states3[idx_1s, 7])
    print(f"  Pitch at t=1.0s: {pitch_at_1s:.2f} deg (target: {np.degrees(step_angle):.1f})")
    print(f"  PASS: {abs(pitch_at_1s - np.degrees(step_angle)) < 2.0}")

    # Test 4: Yaw step response (yaw doesn't cause lateral drift)
    print("\n--- Test 4: Yaw Step Response (30 deg) ---")
    yaw_step = np.radians(30)
    times4, states4, motors4 = run_attitude_test(
        params, lambda t: np.array([0.0, 0.0, yaw_step if t > 0.2 else 0.0]),
        5.0, state0, dt, 4.0)

    final_yaw = np.degrees(states4[-1, 8])
    print(f"  Final yaw: {final_yaw:.2f} deg (target: {np.degrees(yaw_step):.1f})")
    print(f"  PASS: {abs(final_yaw - np.degrees(yaw_step)) < 2.0}")

    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Hover stabilization
    ax = axes[0, 0]
    ax.plot(times1, np.degrees(states1[:, 6]), label='Roll')
    ax.plot(times1, np.degrees(states1[:, 7]), label='Pitch')
    ax.plot(times1, np.degrees(states1[:, 8]), label='Yaw')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('Hover Stabilization (2 deg initial roll)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Roll step response
    ax = axes[0, 1]
    ax.plot(times2, np.degrees(states2[:, 6]), 'b-', label='Roll')
    ax.axhline(y=np.degrees(step_angle), color='r', linestyle='--', label='Reference')
    ax.axvline(x=0.2, color='g', linestyle=':', alpha=0.5, label='Step time')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Roll (deg)')
    ax.set_title('Roll Step Response (5 deg)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Pitch step response
    ax = axes[1, 0]
    ax.plot(times3, np.degrees(states3[:, 7]), 'b-', label='Pitch')
    ax.axhline(y=np.degrees(step_angle), color='r', linestyle='--', label='Reference')
    ax.axvline(x=0.2, color='g', linestyle=':', alpha=0.5, label='Step time')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch (deg)')
    ax.set_title('Pitch Step Response (5 deg)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Yaw step response
    ax = axes[1, 1]
    ax.plot(times4, np.degrees(states4[:, 8]), 'b-', label='Yaw')
    ax.axhline(y=np.degrees(yaw_step), color='r', linestyle='--', label='Reference')
    ax.axvline(x=0.2, color='g', linestyle=':', alpha=0.5, label='Step time')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw (deg)')
    ax.set_title('Yaw Step Response (30 deg)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.suptitle('Task 3: PID Attitude Controller', fontsize=14)
    plt.tight_layout()
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task3_pid_attitude.png')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n  Plot saved to {save_path}")
    plt.close()

    print("\n" + "=" * 60)
    print("Task 3 Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
