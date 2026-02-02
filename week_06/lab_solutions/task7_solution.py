#!/usr/bin/env python3
"""
Task 7 Solution: Full Quadrotor Simulation
============================================
Complete system: dynamics + cascaded control + attitude estimation + motor mixing.
Mission: takeoff -> fly square trajectory -> land.
"""
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from quadrotor_sim import (QuadrotorParams, rk4_step, hover_motor_speed,
                           inverse_mixing, euler_to_rotation)


def complementary_filter_step(gyro, accel, alpha, dt, prev_est):
    """Single step complementary filter for roll/pitch."""
    gyro_est = prev_est + gyro[:2] * dt
    ax, ay, az = accel
    accel_est = np.array([np.arctan2(ay, az), np.arctan2(-ax, np.sqrt(ay**2 + az**2))])
    return alpha * gyro_est + (1 - alpha) * accel_est


class QuadrotorSystem:
    """Complete quadrotor with dynamics, control, estimation, and mixing."""

    def __init__(self, params):
        self.params = params
        self.state = np.zeros(12)
        self.M_inv = inverse_mixing(params)
        self.g = params.g
        self.mass = params.mass
        self.att_estimate = np.zeros(2)

        # Control gains - PD on position, PD on attitude
        self.kp_pos = np.array([1.5, 1.5, 3.0])
        self.kd_pos = np.array([2.0, 2.0, 2.5])
        self.kp_att = np.array([0.5, 0.5, 0.3])
        self.kd_att = np.array([0.15, 0.15, 0.1])

    def step(self, desired_pos, desired_yaw, dt):
        state = self.state
        angles = state[6:9]
        rates = state[9:12]

        # Simulate IMU with noise
        gyro = rates + 0.005 * np.random.randn(3)
        R = euler_to_rotation(angles[0], angles[1], angles[2])
        gravity_body = R.T @ np.array([0, 0, self.g])
        accel_meas = gravity_body + 0.3 * np.random.randn(3)

        # Complementary filter
        self.att_estimate = complementary_filter_step(gyro, accel_meas, 0.98, dt, self.att_estimate)

        # Position PD -> desired acceleration
        pos_err = np.array(desired_pos) - state[:3]
        desired_acc = self.kp_pos * pos_err + self.kd_pos * (-state[3:6])
        desired_acc = np.clip(desired_acc, -4.0, 4.0)

        # Acceleration -> attitude + thrust
        cpsi, spsi = np.cos(desired_yaw), np.sin(desired_yaw)
        phi_d = np.clip((desired_acc[0]*spsi - desired_acc[1]*cpsi) / self.g, -0.4, 0.4)
        theta_d = np.clip((desired_acc[0]*cpsi + desired_acc[1]*spsi) / self.g, -0.4, 0.4)
        thrust = max(self.mass * (self.g + desired_acc[2]), 0.1)

        # Attitude PD using estimated angles for roll/pitch
        desired_angles = np.array([phi_d, theta_d, desired_yaw])
        est_angles = np.array([self.att_estimate[0], self.att_estimate[1], angles[2]])

        torques = np.zeros(3)
        for i in range(3):
            err = desired_angles[i] - est_angles[i]
            if i == 2:
                err = (err + np.pi) % (2*np.pi) - np.pi
            torques[i] = self.kp_att[i] * err - self.kd_att[i] * rates[i]

        # Motor mixing
        wrench = np.array([thrust, torques[0], torques[1], torques[2]])
        w_sq = self.M_inv @ wrench
        w_sq = np.maximum(w_sq, 0.0)
        motors = np.sqrt(w_sq)
        motors = np.clip(motors, self.params.motor_min, self.params.motor_max)

        # Dynamics (RK4)
        self.state = rk4_step(self.state, motors, self.params, dt)
        return self.state.copy(), motors.copy(), self.att_estimate.copy()


def fly_mission(system, waypoints, dt=0.002, hold_time=4.0):
    """Execute waypoint mission with smooth interpolation."""
    wp_count = len(waypoints)
    t_final = wp_count * hold_time
    N = int(t_final / dt)

    times = np.zeros(N)
    states = np.zeros((N, 12))
    motors_hist = np.zeros((N, 4))
    estimates_hist = np.zeros((N, 2))
    desired_hist = np.zeros((N, 4))

    prev_wp = np.array([0.0, 0.0, 0.0, 0.0])

    for i in range(N):
        t = i * dt
        times[i] = t

        wp_idx = min(int(t / hold_time), wp_count - 1)
        target = np.array(waypoints[wp_idx])
        if wp_idx > 0:
            prev_wp = np.array(waypoints[wp_idx - 1])
        else:
            prev_wp = np.array([0.0, 0.0, 0.0, 0.0])

        seg_start = wp_idx * hold_time
        frac = min((t - seg_start) / (hold_time * 0.5), 1.0)
        frac = 3*frac**2 - 2*frac**3
        desired = prev_wp + frac * (target - prev_wp)
        desired_hist[i] = desired

        state, motors, att_est = system.step(desired[:3], desired[3], dt)
        states[i] = state
        motors_hist[i] = motors
        estimates_hist[i] = att_est

    return {
        'times': times, 'states': states, 'motors': motors_hist,
        'estimates': estimates_hist, 'desired': desired_hist,
    }


def evaluate_performance(trajectory, waypoints):
    """Compute tracking performance metrics."""
    states = trajectory['states']
    desired = trajectory['desired']
    motors = trajectory['motors']
    times = trajectory['times']

    pos_err = np.sqrt(np.sum((states[:, :3] - desired[:, :3])**2, axis=1))
    dt = times[1] - times[0]
    energy = np.sum(motors**2) * dt

    threshold = 0.15
    not_settled = np.where(pos_err > threshold)[0]
    settling_time = times[not_settled[-1]] if len(not_settled) > 0 else 0.0

    return {
        'mean_error': np.mean(pos_err),
        'max_error': np.max(pos_err),
        'rms_error': np.sqrt(np.mean(pos_err**2)),
        'settling_time': settling_time,
        'energy': energy,
        'max_motor_speed': np.max(motors),
    }


def main():
    print("=" * 60)
    print("Task 7 Solution: Full Quadrotor Simulation")
    print("=" * 60)

    np.random.seed(42)
    params = QuadrotorParams()
    system = QuadrotorSystem(params)

    waypoints = [
        [0.0, 0.0, 2.0, 0.0],
        [0.0, 0.0, 2.0, 0.0],
        [2.0, 0.0, 2.0, 0.0],
        [2.0, 2.0, 2.0, np.pi/2],
        [0.0, 2.0, 2.0, np.pi],
        [0.0, 0.0, 2.0, 0.0],
        [0.0, 0.0, 2.0, 0.0],
        [0.0, 0.0, 0.2, 0.0],
    ]

    print("\n  Waypoints (square trajectory):")
    for i, wp in enumerate(waypoints):
        print(f"    WP{i}: x={wp[0]:.1f}, y={wp[1]:.1f}, z={wp[2]:.1f}, yaw={np.degrees(wp[3]):.0f} deg")

    print("\n  Running simulation...")
    trajectory = fly_mission(system, waypoints, dt=0.002, hold_time=4.0)

    metrics = evaluate_performance(trajectory, waypoints)
    print(f"\n  Performance Metrics:")
    print(f"    Mean tracking error: {metrics['mean_error']:.4f} m")
    print(f"    Max tracking error:  {metrics['max_error']:.4f} m")
    print(f"    RMS tracking error:  {metrics['rms_error']:.4f} m")
    print(f"    Max motor speed:     {metrics['max_motor_speed']:.1f} rad/s (limit: {params.motor_max})")
    print(f"    Total energy:        {metrics['energy']:.0f}")

    final_pos = trajectory['states'][-1, :3]
    final_desired = trajectory['desired'][-1, :3]
    final_err = np.linalg.norm(final_pos - final_desired)
    print(f"\n  Final position: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]")
    print(f"  Final desired:  [{final_desired[0]:.3f}, {final_desired[1]:.3f}, {final_desired[2]:.3f}]")
    print(f"  Final error:    {final_err:.4f} m {'PASS' if final_err < 0.5 else 'NEEDS TUNING'}")

    # 6-panel plot
    times = trajectory['times']
    states = trajectory['states']
    desired = trajectory['desired']
    motors = trajectory['motors']
    estimates = trajectory['estimates']

    fig = plt.figure(figsize=(20, 14))

    ax = fig.add_subplot(2, 3, 1, projection='3d')
    ax.plot(states[:, 0], states[:, 1], states[:, 2], 'b-', linewidth=1.2, label='Actual')
    ax.plot(desired[:, 0], desired[:, 1], desired[:, 2], 'r--', linewidth=0.8, label='Desired')
    for wp in waypoints:
        ax.scatter(*wp[:3], c='red', s=40, marker='o', zorder=5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Flight Path')
    ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 3, 2)
    for i, (label, color) in enumerate(zip(['X', 'Y', 'Z'], ['r', 'g', 'b'])):
        ax.plot(times, states[:, i], color=color, linewidth=1.2, label=f'{label} actual')
        ax.plot(times, desired[:, i], color=color, linestyle='--', alpha=0.5, label=f'{label} ref')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('Position Tracking')
    ax.legend(fontsize=6, ncol=2)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(2, 3, 3)
    for i, (label, color) in enumerate(zip(['Roll', 'Pitch', 'Yaw'], ['r', 'g', 'b'])):
        ax.plot(times, np.degrees(states[:, 6+i]), color=color, linewidth=0.8, label=label)
    ax.plot(times, np.degrees(estimates[:, 0]), 'r:', linewidth=0.5, alpha=0.7, label='Roll est.')
    ax.plot(times, np.degrees(estimates[:, 1]), 'g:', linewidth=0.5, alpha=0.7, label='Pitch est.')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('Attitude (true + estimated)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(2, 3, 4)
    for i, (label, color) in enumerate(zip(['p', 'q', 'r'], ['r', 'g', 'b'])):
        ax.plot(times, np.degrees(states[:, 9+i]), color=color, linewidth=0.8, label=label)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Rate (deg/s)')
    ax.set_title('Body Angular Rates')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(2, 3, 5)
    for i in range(4):
        ax.plot(times, motors[:, i], linewidth=0.6, label=f'Motor {i+1}')
    ax.axhline(y=hover_motor_speed(params), color='k', linestyle='--', alpha=0.3, label='Hover')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (rad/s)')
    ax.set_title('Motor Speeds')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(2, 3, 6)
    pos_err = np.sqrt(np.sum((states[:, :3] - desired[:, :3])**2, axis=1))
    ax.plot(times, pos_err, 'b-', linewidth=1)
    ax.axhline(y=0.15, color='r', linestyle='--', alpha=0.5, label='0.15m threshold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (m)')
    ax.set_title('3D Position Tracking Error')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.suptitle('Task 7: Full Quadrotor Simulation - Square Trajectory Mission', fontsize=14)
    plt.tight_layout()
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task7_full_simulation.png')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n  Plot saved to {save_path}")
    plt.close()

    print("\n" + "=" * 60)
    print("Task 7 Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
