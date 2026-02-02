#!/usr/bin/env python3
"""
Task 5 Solution: Sensor Integration & Obstacle Avoidance
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
    PX4SITL, FlightMode, OffboardController,
    create_simple_environment, check_obstacle_distance,
    spin, wait_for_position
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def simulate_depth_camera(vehicle_position, vehicle_yaw, environment,
                          fov=np.radians(90), resolution=(64, 48), max_range=20.0):
    """Generate simulated depth image."""
    w, h = resolution
    depth = np.full((h, w), max_range)

    # Camera rays in body frame, rotated by yaw
    for row in range(h):
        elev = (row / h - 0.5) * fov * (h / w)
        for col in range(w):
            azim = (col / w - 0.5) * fov + vehicle_yaw
            # Ray direction
            dx = np.cos(elev) * np.cos(azim)
            dy = np.cos(elev) * np.sin(azim)
            dz = np.sin(elev)
            ray_dir = np.array([dx, dy, dz])

            # Check intersections with obstacles
            for obs in environment.get('obstacles', []):
                # Ray-sphere intersection
                oc = vehicle_position - obs['center']
                a = np.dot(ray_dir, ray_dir)
                b = 2.0 * np.dot(oc, ray_dir)
                c = np.dot(oc, oc) - obs['radius'] ** 2
                disc = b * b - 4 * a * c
                if disc >= 0:
                    t = (-b - np.sqrt(disc)) / (2 * a)
                    if 0 < t < depth[row, col]:
                        depth[row, col] = t
    return depth


def simulate_lidar(vehicle_position, vehicle_yaw, environment,
                   num_rays=36, max_range=30.0):
    """Generate simulated 2D LiDAR scan."""
    ranges = np.full(num_rays, max_range)
    angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False) + vehicle_yaw

    for i, angle in enumerate(angles):
        ray_dir = np.array([np.cos(angle), np.sin(angle), 0.0])
        for obs in environment.get('obstacles', []):
            # Project obstacle to 2D (horizontal plane at vehicle altitude)
            oc = vehicle_position - obs['center']
            # Only consider horizontal distance
            oc_h = np.array([oc[0], oc[1], 0.0])
            a = np.dot(ray_dir, ray_dir)
            b = 2.0 * np.dot(oc_h, ray_dir)
            c = np.dot(oc_h, oc_h) - obs['radius'] ** 2
            # Also check vertical overlap
            vert_dist = abs(oc[2])
            if vert_dist > obs['radius']:
                continue
            disc = b * b - 4 * a * c
            if disc >= 0:
                t = (-b - np.sqrt(disc)) / (2 * a)
                if 0 < t < ranges[i]:
                    ranges[i] = t
    return ranges


def obstacle_detection(scan_data, threshold=5.0):
    """Detect nearby obstacles from LiDAR scan."""
    num_rays = len(scan_data)
    angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False)
    obstacles = []
    for i, (angle, dist) in enumerate(zip(angles, scan_data)):
        if dist < threshold:
            obstacles.append((angle, dist))
    return obstacles


def reactive_avoidance(vehicle, obstacles, goal, safety_margin=3.0):
    """Potential field obstacle avoidance."""
    pos = vehicle.position

    # Attractive force toward goal
    to_goal = goal - pos
    dist_goal = np.linalg.norm(to_goal)
    if dist_goal > 0.1:
        f_att = to_goal / dist_goal * min(dist_goal, 2.0)
    else:
        f_att = np.zeros(3)

    # Repulsive force from obstacles
    f_rep = np.zeros(3)
    for angle, dist in obstacles:
        if dist < safety_margin:
            # Direction of obstacle in world frame
            obs_dir = np.array([np.cos(angle + vehicle.attitude[2]),
                                np.sin(angle + vehicle.attitude[2]),
                                0.0])
            strength = (1.0 / dist - 1.0 / safety_margin) * 3.0
            f_rep -= obs_dir * strength

    # Combined velocity
    vel = f_att + f_rep
    speed = np.linalg.norm(vel)
    max_speed = 3.0
    if speed > max_speed:
        vel = vel / speed * max_speed

    return vel


def fly_direct(vehicle, goal, duration=40.0):
    """Fly directly to goal without avoidance."""
    positions = []
    start = vehicle.sim_time
    while vehicle.sim_time - start < duration:
        vehicle.send_position_setpoint(goal[0], goal[1], goal[2])
        vehicle.step()
        positions.append(vehicle.position.copy())
        if np.linalg.norm(vehicle.position - goal) < 0.5:
            break
    return np.array(positions)


def fly_with_avoidance(vehicle, goal, environment, duration=60.0):
    """Fly to goal with obstacle avoidance."""
    positions = []
    start = vehicle.sim_time
    while vehicle.sim_time - start < duration:
        # Simulate LiDAR
        scan = simulate_lidar(vehicle.position, vehicle.attitude[2], environment)
        obstacles = obstacle_detection(scan, threshold=6.0)

        if obstacles:
            vel = reactive_avoidance(vehicle, obstacles, goal, safety_margin=4.0)
            vehicle.send_velocity_setpoint(vel[0], vel[1], vel[2])
        else:
            vehicle.send_position_setpoint(goal[0], goal[1], goal[2])

        vehicle.step()
        positions.append(vehicle.position.copy())
        if np.linalg.norm(vehicle.position - goal) < 0.5:
            break
    return np.array(positions)


def main():
    print("=" * 60)
    print("Task 5 Solution: Sensor Integration & Obstacle Avoidance")
    print("=" * 60)

    environment = create_simple_environment([
        {'center': np.array([12.0, 3.0, -8.0]), 'radius': 2.5},
        {'center': np.array([8.0, -2.0, -9.0]), 'radius': 2.0},
        {'center': np.array([18.0, 1.0, -7.0]), 'radius': 2.0},
        {'center': np.array([15.0, -4.0, -10.0]), 'radius': 1.5},
    ])

    goal = np.array([25.0, 0.0, -10.0])

    # --- Flight 1: Direct (no avoidance) ---
    print("\nFlight 1: Direct path (no avoidance)...")
    v1 = PX4SITL(dt=0.02)
    c1 = OffboardController(v1)
    c1.arm_and_takeoff(altitude=10.0)
    for _ in range(300):
        v1.step()
    traj_direct = fly_direct(v1, goal)

    min_dist_direct = float('inf')
    for p in traj_direct:
        d, _ = check_obstacle_distance(p, environment)
        min_dist_direct = min(min_dist_direct, d)
    print(f"  Min obstacle distance: {min_dist_direct:.2f}m")
    reached_direct = np.linalg.norm(traj_direct[-1] - goal) < 1.0
    print(f"  Goal reached: {reached_direct}")

    # --- Flight 2: With avoidance ---
    print("\nFlight 2: With obstacle avoidance...")
    v2 = PX4SITL(dt=0.02)
    c2 = OffboardController(v2)
    c2.arm_and_takeoff(altitude=10.0)
    for _ in range(300):
        v2.step()
    traj_avoid = fly_with_avoidance(v2, goal, environment)

    min_dist_avoid = float('inf')
    for p in traj_avoid:
        d, _ = check_obstacle_distance(p, environment)
        min_dist_avoid = min(min_dist_avoid, d)
    print(f"  Min obstacle distance: {min_dist_avoid:.2f}m")
    reached_avoid = np.linalg.norm(traj_avoid[-1] - goal) < 1.0
    print(f"  Goal reached: {reached_avoid}")

    # --- Plot ---
    fig = plt.figure(figsize=(16, 10))

    # 3D comparison
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(traj_direct[:, 0], traj_direct[:, 1], -traj_direct[:, 2],
             'r-', label='Direct', alpha=0.7)
    ax1.plot(traj_avoid[:, 0], traj_avoid[:, 1], -traj_avoid[:, 2],
             'b-', label='Avoidance', alpha=0.7)
    # Obstacles
    for obs in environment['obstacles']:
        u, v_a = np.mgrid[0:2*np.pi:12j, 0:np.pi:8j]
        x = obs['center'][0] + obs['radius'] * np.cos(u) * np.sin(v_a)
        y = obs['center'][1] + obs['radius'] * np.sin(u) * np.sin(v_a)
        z = -obs['center'][2] + obs['radius'] * np.cos(v_a)
        ax1.plot_surface(x, y, z, alpha=0.3, color='orange')
    ax1.scatter(*goal[:2], -goal[2], c='green', s=100, marker='*', label='Goal')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Alt')
    ax1.set_title('3D Trajectory Comparison')
    ax1.legend(fontsize=8)

    # Top-down
    ax2 = fig.add_subplot(222)
    ax2.plot(traj_direct[:, 0], traj_direct[:, 1], 'r-', label='Direct', linewidth=1.5)
    ax2.plot(traj_avoid[:, 0], traj_avoid[:, 1], 'b-', label='Avoidance', linewidth=1.5)
    for obs in environment['obstacles']:
        circle = plt.Circle((obs['center'][0], obs['center'][1]),
                             obs['radius'], color='orange', alpha=0.5)
        ax2.add_patch(circle)
        # Safety margin
        circle2 = plt.Circle((obs['center'][0], obs['center'][1]),
                              obs['radius'] + 4.0, color='orange', alpha=0.1,
                              linestyle='--', fill=False)
        ax2.add_patch(circle2)
    ax2.scatter(*goal[:2], c='green', s=100, marker='*', zorder=5)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top-Down View')
    ax2.legend()
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    # LiDAR scan example
    ax3 = fig.add_subplot(223, polar=True)
    scan = simulate_lidar(np.array([10.0, 0.0, -10.0]), 0.0, environment, num_rays=72)
    angles = np.linspace(0, 2 * np.pi, 72, endpoint=False)
    ax3.plot(angles, scan, 'b-', linewidth=1.5)
    ax3.fill(angles, scan, alpha=0.1, color='blue')
    ax3.set_title('LiDAR Scan at (10,0)', pad=20)
    ax3.set_rmax(30)

    # Stats comparison
    ax4 = fig.add_subplot(224)
    labels = ['Path Length', 'Min Obs Dist', 'Goal Error']
    direct_vals = [
        np.sum(np.linalg.norm(np.diff(traj_direct, axis=0), axis=1)),
        min_dist_direct,
        np.linalg.norm(traj_direct[-1] - goal),
    ]
    avoid_vals = [
        np.sum(np.linalg.norm(np.diff(traj_avoid, axis=0), axis=1)),
        min_dist_avoid,
        np.linalg.norm(traj_avoid[-1] - goal),
    ]
    x_pos = np.arange(len(labels))
    width = 0.35
    ax4.bar(x_pos - width/2, direct_vals, width, label='Direct', color='red', alpha=0.7)
    ax4.bar(x_pos + width/2, avoid_vals, width, label='Avoidance', color='blue', alpha=0.7)
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(labels)
    ax4.set_ylabel('Value (m)')
    ax4.set_title('Performance Comparison')
    ax4.legend()
    ax4.grid(True, alpha=0.3, axis='y')

    plt.suptitle('Task 5: Sensor Integration & Obstacle Avoidance', fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(SCRIPT_DIR, 'task5_sensor_integration.png'), dpi=150)
    print(f"\nPlot saved to {os.path.join(SCRIPT_DIR, 'task5_sensor_integration.png')}")
    print("Task 5 complete.")


if __name__ == '__main__':
    main()
