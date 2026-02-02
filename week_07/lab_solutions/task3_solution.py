#!/usr/bin/env python3
"""
Task 3 Solution: Offboard Position Control
"""

import sys
import os
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from px4_sim import (
    PX4SITL, FlightMode, FLIGHT_MODE_NAMES, OffboardController,
    spin, wait_for_position, MAVROSInterface
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def setup_offboard(vehicle):
    ctrl = OffboardController(vehicle)
    ctrl.start_setpoint_stream()
    return ctrl


def send_position_setpoint(vehicle, x, y, z, yaw=0.0):
    vehicle.send_position_setpoint(x, y, z, yaw)


def send_velocity_setpoint(vehicle, vx, vy, vz, yaw_rate=0.0):
    vehicle.send_velocity_setpoint(vx, vy, vz, yaw_rate)


def fly_to_position(vehicle, x, y, z, tolerance=0.5, timeout=30.0):
    target = np.array([x, y, z])
    vehicle.send_position_setpoint(x, y, z)
    return wait_for_position(vehicle, target, tolerance=tolerance, timeout=timeout)


def offboard_mission(vehicle, waypoints):
    positions = []
    for wp in waypoints:
        vehicle.send_position_setpoint(wp[0], wp[1], wp[2])
        start = vehicle.sim_time
        while vehicle.sim_time - start < 30.0:
            vehicle.step()
            positions.append(vehicle.position.copy())
            dist = np.linalg.norm(vehicle.position - np.array(wp))
            if dist < 0.5:
                break
    return positions


def main():
    print("=" * 60)
    print("Task 3 Solution: Offboard Position Control")
    print("=" * 60)

    vehicle = PX4SITL(dt=0.02)
    mavros = MAVROSInterface(vehicle)

    # Setup: transition to POSITION, then prepare offboard
    print("\n1. Mode setup...")
    vehicle.set_mode(FlightMode.STABILIZED)
    vehicle.set_mode(FlightMode.POSITION)

    ctrl = setup_offboard(vehicle)

    print("2. Engaging offboard and arming...")
    ok, msg = ctrl.engage_offboard()
    print(f"   Offboard: {msg}")
    ok, msg = vehicle.arm()
    print(f"   Arm: {msg}")

    # Takeoff - mark as airborne
    print("3. Taking off to 10m...")
    vehicle.landed = False
    vehicle.in_air = True
    reached = fly_to_position(vehicle, 0, 0, -10, tolerance=1.0, timeout=15.0)
    print(f"   Takeoff {'reached' if reached else 'timeout'}, alt={-vehicle.position[2]:.1f}m")

    # Fly square pattern
    waypoints = [
        (10, 0, -10),
        (10, 10, -10),
        (0, 10, -10),
        (0, 0, -10),
    ]
    print("4. Flying square pattern...")
    for i, wp in enumerate(waypoints):
        reached = fly_to_position(vehicle, wp[0], wp[1], wp[2])
        print(f"   WP{i+1} ({wp[0]},{wp[1]},{-wp[2]}m): "
              f"{'reached' if reached else 'timeout'}")

    # Land
    print("5. Landing...")
    vehicle.send_position_setpoint(0, 0, 0)
    wait_for_position(vehicle, np.array([0, 0, 0]), tolerance=0.3, timeout=20.0)
    vehicle.landed = True
    vehicle.in_air = False
    vehicle.disarm()
    print(f"   Landed and disarmed")

    # Extract trajectory from log
    times = [e['time'] for e in vehicle.log]
    positions = np.array([e['position'] for e in vehicle.log])
    setpoints = np.array([e['setpoint'] for e in vehicle.log])

    # Plot
    fig = plt.figure(figsize=(16, 10))

    # 3D trajectory
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(positions[:, 0], positions[:, 1], -positions[:, 2], 'b-', linewidth=1.5,
             label='Actual')
    for i, wp in enumerate(waypoints):
        ax1.scatter(wp[0], wp[1], -wp[2], c='r', s=80, marker='^',
                    label='Waypoint' if i == 0 else '')
    ax1.scatter(0, 0, 0, c='g', s=100, marker='*', label='Home')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Altitude (m)')
    ax1.set_title('3D Flight Trajectory')
    ax1.legend(fontsize=8)

    # XY top-down
    ax2 = fig.add_subplot(222)
    ax2.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=1.5, label='Actual')
    wp_arr = np.array(waypoints + [waypoints[0]])
    ax2.plot(wp_arr[:, 0], wp_arr[:, 1], 'r--o', linewidth=1, label='Planned')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top-Down View')
    ax2.legend()
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    # Position vs time
    ax3 = fig.add_subplot(223)
    ax3.plot(times, positions[:, 0], label='X')
    ax3.plot(times, positions[:, 1], label='Y')
    ax3.plot(times, -positions[:, 2], label='Z (alt)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (m)')
    ax3.set_title('Position vs Time')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Velocity
    velocities = np.array([e['velocity'] for e in vehicle.log])
    ax4 = fig.add_subplot(224)
    speed = np.linalg.norm(velocities, axis=1)
    ax4.plot(times, speed, 'b-', label='Speed')
    ax4.plot(times, velocities[:, 0], '--', alpha=0.5, label='Vx')
    ax4.plot(times, velocities[:, 1], '--', alpha=0.5, label='Vy')
    ax4.plot(times, -velocities[:, 2], '--', alpha=0.5, label='Vz')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Velocity (m/s)')
    ax4.set_title('Velocity Profile')
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    plt.suptitle('Task 3: Offboard Position Control - Square Pattern', fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(SCRIPT_DIR, 'task3_offboard_control.png'), dpi=150)
    print(f"\nPlot saved to {os.path.join(SCRIPT_DIR, 'task3_offboard_control.png')}")
    print("Task 3 complete.")


if __name__ == '__main__':
    main()
