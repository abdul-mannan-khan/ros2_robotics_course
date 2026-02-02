#!/usr/bin/env python3
"""
Week 5 - Generate Navigation Data
Creates grid maps, laser scans, waypoints, and robot footprint data
for Nav2-style navigation exercises.
"""

import os
import numpy as np

def create_grid_map(width=100, height=100):
    """Create a 100x100 occupancy grid with walls, obstacles, narrow passages, open areas.
    0 = free, 100 = occupied, -1 = unknown
    """
    grid = np.zeros((height, width), dtype=np.int8)

    # Outer walls
    grid[0, :] = 100
    grid[-1, :] = 100
    grid[:, 0] = 100
    grid[:, -1] = 100

    # Interior walls creating rooms
    # Horizontal wall with gap (passage)
    grid[30, 10:45] = 100
    grid[30, 55:90] = 100  # gap at columns 45-55

    # Vertical wall with gap
    grid[10:40, 60] = 100
    grid[50:70, 60] = 100  # gap at rows 40-50

    # Another horizontal wall
    grid[70, 20:40] = 100
    grid[70, 50:80] = 100  # gap at 40-50

    # Narrow passage (width ~2 cells)
    grid[45:55, 25] = 100
    grid[45:55, 28] = 100  # passage between col 25-28

    # Rectangular obstacles
    grid[15:20, 15:20] = 100
    grid[50:55, 75:80] = 100
    grid[80:85, 30:35] = 100
    grid[40:43, 80:85] = 100

    # Circular obstacle
    cy, cx, r = 60, 40, 4
    for y in range(height):
        for x in range(width):
            if (x - cx)**2 + (y - cy)**2 <= r**2:
                grid[y, x] = 100

    # Small unknown region
    grid[85:95, 85:95] = -1

    return grid


def simulate_laser_scan(pose, grid_map, resolution, num_beams=360, max_range=3.5):
    """Simulate a 2D lidar scan from a given pose on the grid map."""
    px, py, theta = pose
    # Convert world coords to grid
    ranges = np.full(num_beams, max_range)
    angles = np.linspace(-np.pi, np.pi, num_beams, endpoint=False)

    height, width = grid_map.shape
    for i, angle in enumerate(angles):
        beam_angle = theta + angle
        for step in range(1, int(max_range / resolution) + 1):
            dist = step * resolution
            wx = px + dist * np.cos(beam_angle)
            wy = py + dist * np.sin(beam_angle)
            gx = int(wx / resolution)
            gy = int(wy / resolution)
            if gx < 0 or gx >= width or gy < 0 or gy >= height:
                ranges[i] = dist
                break
            if grid_map[gy, gx] == 100:
                ranges[i] = dist
                break
    return ranges


def generate_waypoints():
    """Generate a set of waypoints (in world coordinates, meters)."""
    waypoints = np.array([
        [1.0, 1.0],
        [2.5, 1.0],
        [2.5, 2.5],
        [1.0, 3.5],
        [3.5, 3.5],
        [4.0, 1.5],
        [2.0, 4.0],
    ])
    return waypoints


def generate_robot_footprint():
    """Rectangular robot footprint polygon (meters)."""
    # 0.3m x 0.3m square robot
    half = 0.15
    footprint = np.array([
        [-half, -half],
        [half, -half],
        [half, half],
        [-half, half],
    ])
    return footprint


def main():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
    os.makedirs(data_dir, exist_ok=True)

    resolution = 0.05  # 5cm per cell

    # Grid map
    grid_map = create_grid_map(100, 100)
    np.save(os.path.join(data_dir, 'grid_map.npy'), grid_map)

    # Metadata
    metadata = np.array([resolution, 0.0, 0.0, 100, 100])  # res, ox, oy, w, h
    np.save(os.path.join(data_dir, 'map_metadata.npy'), metadata)

    # Waypoints
    waypoints = generate_waypoints()
    np.save(os.path.join(data_dir, 'waypoints.npy'), waypoints)

    # Robot footprint
    footprint = generate_robot_footprint()
    np.save(os.path.join(data_dir, 'robot_footprint.npy'), footprint)

    # Laser scans at various poses
    scan_poses = np.array([
        [1.0, 1.0, 0.0],
        [2.5, 1.0, np.pi / 2],
        [2.5, 2.5, np.pi],
        [1.0, 3.5, -np.pi / 2],
        [3.5, 3.5, 0.0],
    ])
    scans = []
    for pose in scan_poses:
        scan = simulate_laser_scan(pose, grid_map, resolution)
        scans.append(scan)
    scans = np.array(scans)
    np.save(os.path.join(data_dir, 'laser_scans.npy'), scans)
    np.save(os.path.join(data_dir, 'scan_poses.npy'), scan_poses)

    print("Navigation data generated in", data_dir)
    print(f"  grid_map.npy: {grid_map.shape}, free={np.sum(grid_map==0)}, "
          f"occupied={np.sum(grid_map==100)}, unknown={np.sum(grid_map==-1)}")
    print(f"  waypoints.npy: {waypoints.shape}")
    print(f"  laser_scans.npy: {scans.shape}")
    print(f"  scan_poses.npy: {scan_poses.shape}")
    print(f"  robot_footprint.npy: {footprint.shape}")
    print(f"  map_metadata.npy: {metadata}")


if __name__ == '__main__':
    main()
