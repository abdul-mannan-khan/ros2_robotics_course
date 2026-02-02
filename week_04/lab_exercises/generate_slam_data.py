#!/usr/bin/env python3
"""
Week 4 - 2D SLAM with LiDAR: Data Generator
Generates a simulated 2D SLAM dataset with environment, trajectory, LiDAR, and odometry.
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def create_environment(grid_size=500, resolution=0.1):
    """
    Create a 50x50m environment as a 500x500 occupancy grid.
    0 = free, 1 = occupied.
    Features: outer walls, internal corridors, rooms, obstacles.
    """
    grid = np.zeros((grid_size, grid_size), dtype=np.float64)

    # Helper to fill rectangle in grid coords
    def fill_rect(r0, c0, r1, c1):
        grid[r0:r1, c0:c1] = 1.0

    # Outer walls (thickness ~0.3m = 3 cells)
    fill_rect(0, 0, 3, grid_size)          # top wall
    fill_rect(grid_size-3, 0, grid_size, grid_size)  # bottom wall
    fill_rect(0, 0, grid_size, 3)          # left wall
    fill_rect(0, grid_size-3, grid_size, grid_size)  # right wall

    # Horizontal corridor wall (y=15m -> row 150), with gap at x=25m
    fill_rect(148, 3, 152, 230)
    fill_rect(148, 270, 152, grid_size-3)

    # Horizontal corridor wall (y=35m -> row 350), with gap at x=25m
    fill_rect(348, 3, 352, 220)
    fill_rect(348, 280, 352, grid_size-3)

    # Vertical corridor wall (x=30m -> col 300), with gaps at y=10m and y=40m
    fill_rect(3, 298, 80, 302)
    fill_rect(120, 298, 348, 302)
    fill_rect(352, 298, 420, 302)
    fill_rect(460, 298, grid_size-3, 302)

    # Some room partitions in top-left quadrant
    fill_rect(50, 100, 54, 200)   # horizontal wall
    fill_rect(54, 198, 148, 202)  # vertical wall with gap

    # Obstacles (small blocks)
    obstacles = [
        (200, 100, 215, 115),
        (200, 380, 215, 395),
        (420, 150, 435, 165),
        (420, 400, 435, 415),
        (250, 200, 260, 210),
        (260, 350, 270, 360),
        (170, 450, 185, 465),
        (400, 60, 415, 75),
    ]
    for r0, c0, r1, c1 in obstacles:
        fill_rect(r0, c0, r1, c1)

    return grid


def generate_trajectory(num_poses=300):
    """
    Generate a rectangular loop trajectory through the environment.
    The robot starts at (10, 10), goes right, down, left, up - forming a loop.
    Returns ground truth poses as Nx3 [x, y, theta].
    """
    poses = []

    # Define waypoints for the loop (in meters)
    waypoints = [
        (10, 10),
        (40, 10),
        (40, 25),
        (40, 40),
        (10, 40),
        (10, 25),
        (10, 10),   # loop closure
    ]

    segments = len(waypoints) - 1
    poses_per_segment = num_poses // segments

    all_x, all_y = [], []
    for i in range(segments):
        x0, y0 = waypoints[i]
        x1, y1 = waypoints[i + 1]
        n = poses_per_segment if i < segments - 1 else num_poses - len(all_x)
        xs = np.linspace(x0, x1, n, endpoint=(i == segments - 1))
        ys = np.linspace(y0, y1, n, endpoint=(i == segments - 1))
        all_x.extend(xs)
        all_y.extend(ys)

    all_x = np.array(all_x)
    all_y = np.array(all_y)

    # Compute heading from trajectory direction
    thetas = np.zeros(len(all_x))
    for i in range(len(all_x) - 1):
        thetas[i] = np.arctan2(all_y[i+1] - all_y[i], all_x[i+1] - all_x[i])
    thetas[-1] = thetas[-2]

    poses = np.column_stack([all_x, all_y, thetas])
    return poses


def simulate_lidar(pose, environment, num_rays=360, max_range=12.0, resolution=0.1, noise_std=0.02):
    """
    Simulate a LiDAR scan from a given pose in the environment.
    """
    x, y, theta = pose
    angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False) + theta
    ranges = np.full(num_rays, max_range)
    grid_h, grid_w = environment.shape

    for i, angle in enumerate(angles):
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        # Ray march
        for d_idx in range(int(max_range / resolution)):
            d = d_idx * resolution
            wx = x + d * cos_a
            wy = y + d * sin_a
            # Convert to grid
            gc = int(wx / resolution)
            gr = int(wy / resolution)
            if gr < 0 or gr >= grid_h or gc < 0 or gc >= grid_w:
                ranges[i] = d
                break
            if environment[gr, gc] > 0.5:
                ranges[i] = d + np.random.normal(0, noise_std)
                break

    ranges = np.clip(ranges, 0.1, max_range)
    return ranges


def generate_odometry(poses, trans_noise=0.08, rot_noise=0.03, drift_rate=0.003):
    """
    Generate noisy odometry from ground truth poses.
    Returns Nx3 relative [dx, dy, dtheta] with noise and drift.
    """
    N = len(poses)
    odom = np.zeros((N, 3))

    for i in range(1, N):
        dx = poses[i, 0] - poses[i-1, 0]
        dy = poses[i, 1] - poses[i-1, 1]
        dtheta = poses[i, 2] - poses[i-1, 2]
        # Wrap angle
        dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))

        dist = np.sqrt(dx**2 + dy**2)
        # Add noise proportional to motion + drift
        odom[i, 0] = dx + np.random.normal(0, trans_noise * max(dist, 0.01)) + drift_rate
        odom[i, 1] = dy + np.random.normal(0, trans_noise * max(dist, 0.01)) + drift_rate
        odom[i, 2] = dtheta + np.random.normal(0, rot_noise * max(abs(dtheta), 0.01))

    return odom


def main():
    np.random.seed(42)

    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    os.makedirs(data_dir, exist_ok=True)

    print("=== Week 4 SLAM Data Generator ===")

    # 1. Create environment
    print("Creating environment (500x500 occupancy grid, 0.1m resolution)...")
    environment = create_environment()
    np.save(os.path.join(data_dir, "environment.npy"), environment)
    print(f"  Environment shape: {environment.shape}")
    print(f"  Occupied cells: {int(environment.sum())}")

    # 2. Generate trajectory
    print("Generating ground truth trajectory (rectangular loop)...")
    poses = generate_trajectory(num_poses=300)
    np.save(os.path.join(data_dir, "ground_truth_poses.npy"), poses)
    print(f"  Number of poses: {len(poses)}")
    print(f"  Start: ({poses[0,0]:.1f}, {poses[0,1]:.1f})")
    print(f"  End:   ({poses[-1,0]:.1f}, {poses[-1,1]:.1f})")

    # 3. Generate odometry
    print("Generating noisy odometry...")
    odometry = generate_odometry(poses)
    np.save(os.path.join(data_dir, "odometry.npy"), odometry)

    # 4. Simulate LiDAR scans
    print("Simulating LiDAR scans (360 rays, 12m range)...")
    scans = np.zeros((len(poses), 360))
    for i in range(len(poses)):
        scans[i] = simulate_lidar(poses[i], environment)
        if (i + 1) % 50 == 0:
            print(f"  Scan {i+1}/{len(poses)} done")
    np.save(os.path.join(data_dir, "lidar_scans.npy"), scans)

    # 5. Timestamps
    timestamps = np.linspace(0, 30.0, len(poses))
    np.save(os.path.join(data_dir, "timestamps.npy"), timestamps)

    # 6. Visualization
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    axes[0].imshow(environment, cmap='gray_r', origin='lower')
    axes[0].set_title("Environment (Occupancy Grid)")
    axes[0].set_xlabel("x (cells)")
    axes[0].set_ylabel("y (cells)")

    axes[1].imshow(environment, cmap='gray_r', origin='lower', alpha=0.3)
    axes[1].plot(poses[:, 0] / 0.1, poses[:, 1] / 0.1, 'b-', linewidth=2, label='Ground Truth')
    # Show odometry trajectory
    odom_poses = np.zeros_like(poses)
    odom_poses[0] = poses[0]
    for i in range(1, len(poses)):
        odom_poses[i, 0] = odom_poses[i-1, 0] + odometry[i, 0]
        odom_poses[i, 1] = odom_poses[i-1, 1] + odometry[i, 1]
        odom_poses[i, 2] = odom_poses[i-1, 2] + odometry[i, 2]
    axes[1].plot(odom_poses[:, 0] / 0.1, odom_poses[:, 1] / 0.1, 'r--', linewidth=1, label='Odometry')
    axes[1].legend()
    axes[1].set_title("Trajectory")

    # Show sample scan
    scan_idx = 50
    angles = np.linspace(0, 2*np.pi, 360, endpoint=False) + poses[scan_idx, 2]
    sx = poses[scan_idx, 0] + scans[scan_idx] * np.cos(angles)
    sy = poses[scan_idx, 1] + scans[scan_idx] * np.sin(angles)
    axes[2].imshow(environment, cmap='gray_r', origin='lower', alpha=0.3)
    axes[2].scatter(sx / 0.1, sy / 0.1, s=1, c='red')
    axes[2].plot(poses[scan_idx, 0] / 0.1, poses[scan_idx, 1] / 0.1, 'bo', markersize=8)
    axes[2].set_title(f"Sample LiDAR Scan (pose {scan_idx})")

    plt.tight_layout()
    plt.savefig(os.path.join(data_dir, "slam_data_overview.png"), dpi=150)
    plt.close()

    print("\nAll data saved to:", data_dir)
    print("Files: environment.npy, ground_truth_poses.npy, odometry.npy, lidar_scans.npy, timestamps.npy")
    print("Overview plot: slam_data_overview.png")


if __name__ == "__main__":
    main()
