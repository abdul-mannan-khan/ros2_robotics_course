#!/usr/bin/env python3
"""
Task 5 Solution: Cascaded Position + Attitude Control
======================================================
Full cascaded control flying a waypoint mission.
Position PID -> desired accel -> desired attitude + thrust -> PD attitude -> motor mixing.
"""
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from quadrotor_sim import (QuadrotorParams, rk4_step, hover_motor_speed,
                           inverse_mixing, simulate, euler_to_rotation)


def position_controller(desired_pos, current_pos, current_vel, gains):
    """PD position controller outputting desired acceleration."""
    pos_err = np.array(desired_pos) - np.array(current_pos)
    desired_acc = gains['kp_pos'] * pos_err + gains['kd_pos'] * (-np.array(current_vel))
    max_acc = 4.0
    return np.clip(desired_acc, -max_acc, max_acc)


def acceleration_to_attitude(desired_acc, desired_yaw, mass, g):
    """Convert desired acceleration to attitude + thrust (small angle approx)."""
    ax, ay, az = desired_acc
    cpsi = np.cos(desired_yaw)
    spsi = np.sin(desired_yaw)

    phi_d = np.clip((ax * spsi - ay * cpsi) / g, -np.radians(25), np.radians(25))
    theta_d = np.clip((ax * cpsi + ay * spsi) / g, -np.radians(25), np.radians(25))
    thrust = max(mass * (g + az), 0.1)
    return thrust, phi_d, theta_d


def attitude_pd_torques(desired_angles, current_angles, current_rates, kp_att, kd_att):
    """Simple PD attitude controller (direct, not cascaded)."""
    torques = np.zeros(3)
    for i in range(3):
        err = desired_angles[i] - current_angles[i]
        if i == 2:  # wrap yaw
            err = (err + np.pi) % (2*np.pi) - np.pi
        torques[i] = kp_att[i] * err - kd_att[i] * current_rates[i]
    return torques


def main():
    print("=" * 60)
    print("Task 5 Solution: Cascaded Position + Attitude Control")
    print("=" * 60)

    params = QuadrotorParams()
    dt = 0.002
    t_final = 20.0

    pos_gains = {
        'kp_pos': np.array([1.5, 1.5, 3.0]),
        'kd_pos': np.array([2.0, 2.0, 2.5]),
    }
    kp_att = np.array([0.5, 0.5, 0.3])
    kd_att = np.array([0.15, 0.15, 0.1])
    M_inv = inverse_mixing(params)

    # Waypoints: [x, y, z, yaw]
    waypoints = [
        [0.0, 0.0, 2.0, 0.0],
        [0.0, 0.0, 2.0, 0.0],
        [2.0, 0.0, 2.0, 0.0],
        [2.0, 2.0, 2.0, np.pi/4],
        [0.0, 0.0, 2.0, 0.0],
        [0.0, 0.0, 0.2, 0.0],
    ]
    wp_times = [0, 3, 6, 10, 14, 18]

    def get_desired(t):
        for i in range(len(wp_times) - 1):
            if t < wp_times[i+1]:
                frac = min((t - wp_times[i]) / (wp_times[i+1] - wp_times[i]), 1.0)
                frac = 3*frac**2 - 2*frac**3
                pos = (1-frac)*np.array(waypoints[i][:3]) + frac*np.array(waypoints[i+1][:3])
                yaw = (1-frac)*waypoints[i][3] + frac*waypoints[i+1][3]
                return pos, yaw
        return np.array(waypoints[-1][:3]), waypoints[-1][3]

    def controller(t, state):
        pos_d, yaw_d = get_desired(t)
        desired_acc = position_controller(pos_d, state[:3], state[3:6], pos_gains)
        thrust, phi_d, theta_d = acceleration_to_attitude(desired_acc, yaw_d, params.mass, params.g)
        desired_angles = np.array([phi_d, theta_d, yaw_d])
        torques = attitude_pd_torques(desired_angles, state[6:9], state[9:12], kp_att, kd_att)

        wrench = np.array([thrust, torques[0], torques[1], torques[2]])
        w_sq = M_inv @ wrench
        w_sq = np.maximum(w_sq, 0.0)
        motors = np.sqrt(w_sq)
        return np.clip(motors, params.motor_min, params.motor_max)

    state0 = np.zeros(12)
    print("\n  Running simulation...")
    times, states, motor_hist = simulate(state0, controller, params, dt=dt, t_final=t_final)

    # Compute desired history for plotting
    desired_hist = np.zeros((len(times), 4))
    for i, t in enumerate(times):
        p, y = get_desired(t)
        desired_hist[i] = [p[0], p[1], p[2], y]

    pos_err = np.sqrt(np.sum((states[:, :3] - desired_hist[:, :3])**2, axis=1))
    print(f"  Mean tracking error: {np.mean(pos_err):.4f} m")
    print(f"  Max tracking error:  {np.max(pos_err):.4f} m")
    print(f"  Final position: [{states[-1,0]:.3f}, {states[-1,1]:.3f}, {states[-1,2]:.3f}]")
    print(f"  Final desired:  [{desired_hist[-1,0]:.3f}, {desired_hist[-1,1]:.3f}, {desired_hist[-1,2]:.3f}]")
    final_err = np.linalg.norm(states[-1,:3] - desired_hist[-1,:3])
    print(f"  Final error: {final_err:.4f} m  {'PASS' if final_err < 0.5 else 'NEEDS TUNING'}")

    # 4-panel plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    ax = fig.add_subplot(2, 2, 1, projection='3d')
    axes[0, 0].remove()
    ax.plot(states[:, 0], states[:, 1], states[:, 2], 'b-', linewidth=1.5, label='Actual')
    ax.plot(desired_hist[:, 0], desired_hist[:, 1], desired_hist[:, 2], 'r--', linewidth=1, label='Desired')
    for wp in waypoints:
        ax.scatter(*wp[:3], c='red', s=50, marker='o')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Flight Path')
    ax.legend()

    ax = axes[0, 1]
    for i, (label, color) in enumerate(zip(['X', 'Y', 'Z'], ['r', 'g', 'b'])):
        ax.plot(times, states[:, i], color=color, linewidth=1.5, label=f'{label} actual')
        ax.plot(times, desired_hist[:, i], color=color, linestyle='--', alpha=0.6, label=f'{label} desired')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('Position Tracking')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    for i, (label, color) in enumerate(zip(['Roll', 'Pitch', 'Yaw'], ['r', 'g', 'b'])):
        ax.plot(times, np.degrees(states[:, 6+i]), color=color, linewidth=1, label=label)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('Attitude Angles')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    for i in range(4):
        ax.plot(times, motor_hist[:, i], linewidth=0.8, label=f'Motor {i+1}')
    ax.axhline(y=hover_motor_speed(params), color='k', linestyle='--', alpha=0.3, label='Hover speed')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Motor Speed (rad/s)')
    ax.set_title('Motor Speeds')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.suptitle('Task 5: Cascaded Position + Attitude Control', fontsize=14)
    plt.tight_layout()
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task5_cascaded_control.png')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n  Plot saved to {save_path}")
    plt.close()

    print("\n" + "=" * 60)
    print("Task 5 Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
