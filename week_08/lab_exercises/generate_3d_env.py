#!/usr/bin/env python3
"""
Week 8 - 3D Path Planning for Drones
Data Generator: Creates a 3D voxel environment with obstacles, no-fly zones,
waypoints, and dynamic obstacles for drone path planning exercises.
"""

import numpy as np
import os


def create_voxel_grid(nx=50, ny=50, nz=20):
    """Create an empty 3D voxel grid. 0 = free, 1 = occupied."""
    grid = np.zeros((nx, ny, nz), dtype=np.uint8)
    # Ground plane
    grid[:, :, 0] = 1
    return grid


def add_building(grid, x_range, y_range, height):
    """Add a rectangular building (prism) to the grid."""
    x_min, x_max = x_range
    y_min, y_max = y_range
    grid[x_min:x_max, y_min:y_max, 1:height + 1] = 1


def add_no_fly_zone_to_grid(grid, cx, cy, radius, z_min, z_max):
    """Mark a cylindrical no-fly zone in the grid."""
    nx, ny, nz = grid.shape
    for x in range(max(0, int(cx - radius)), min(nx, int(cx + radius) + 1)):
        for y in range(max(0, int(cy - radius)), min(ny, int(cy + radius) + 1)):
            if (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2:
                z_lo = max(1, int(z_min))
                z_hi = min(nz, int(z_max) + 1)
                grid[x, y, z_lo:z_hi] = 1


def generate_environment():
    """Generate the full 3D environment and save all data files."""
    np.random.seed(42)

    nx, ny, nz = 50, 50, 20
    resolution = 1.0  # 1 meter per voxel

    grid = create_voxel_grid(nx, ny, nz)

    # Define buildings (rectangular prisms)
    buildings = [
        # (x_range, y_range, height)
        ((5, 10), (5, 10), 8),
        ((15, 20), (8, 14), 12),
        ((25, 30), (25, 32), 10),
        ((35, 42), (10, 16), 14),
        ((10, 14), (30, 36), 6),
        ((38, 45), (35, 42), 9),
        ((20, 24), (40, 46), 11),
        ((8, 12), (18, 22), 7),
        ((30, 35), (2, 7), 13),
        ((42, 48), (22, 28), 8),
    ]

    obstacles = []
    for (xr, yr, h) in buildings:
        add_building(grid, xr, yr, h)
        cx = (xr[0] + xr[1]) / 2.0
        cy = (yr[0] + yr[1]) / 2.0
        sx = xr[1] - xr[0]
        sy = yr[1] - yr[0]
        obstacles.append([cx, cy, (h + 1) / 2.0, sx, sy, h])

    # No-fly zones (cylindrical)
    no_fly_zones = [
        [22.0, 18.0, 6.0, 1, 19],   # cx, cy, radius, z_min, z_max
        [40.0, 45.0, 4.0, 1, 15],
        [12.0, 42.0, 5.0, 1, 19],
    ]

    for nfz in no_fly_zones:
        add_no_fly_zone_to_grid(grid, nfz[0], nfz[1], nfz[2], nfz[3], nfz[4])

    # Start and goal at different altitudes
    start = np.array([2.0, 2.0, 3.0])
    goal = np.array([47.0, 47.0, 15.0])

    # Waypoints for multi-point missions
    waypoints = np.array([
        [2.0, 2.0, 3.0],
        [12.0, 2.0, 5.0],
        [12.0, 15.0, 8.0],
        [25.0, 20.0, 12.0],
        [35.0, 35.0, 10.0],
        [47.0, 47.0, 15.0],
    ])

    # Dynamic obstacles: [start_x, start_y, start_z, vx, vy, vz, radius, t_start, t_end]
    dynamic_obstacles = np.array([
        [10.0, 25.0, 8.0, 0.5, 0.0, 0.0, 2.0, 0.0, 60.0],
        [30.0, 10.0, 10.0, 0.0, 0.3, 0.1, 1.5, 5.0, 50.0],
        [20.0, 35.0, 5.0, -0.2, 0.4, 0.0, 2.5, 0.0, 40.0],
        [45.0, 20.0, 12.0, -0.3, 0.2, -0.1, 1.0, 10.0, 55.0],
    ])

    # Environment parameters
    env_params = {
        'resolution': resolution,
        'bounds': np.array([0.0, float(nx), 0.0, float(ny), 0.0, float(nz)]),
        'start': start,
        'goal': goal,
    }

    # Save all data
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
    os.makedirs(data_dir, exist_ok=True)

    np.save(os.path.join(data_dir, 'voxel_grid.npy'), grid)
    np.save(os.path.join(data_dir, 'env_params.npy'), env_params)
    np.save(os.path.join(data_dir, 'obstacles.npy'), np.array(obstacles))
    np.save(os.path.join(data_dir, 'no_fly_zones.npy'), np.array(no_fly_zones))
    np.save(os.path.join(data_dir, 'waypoints.npy'), waypoints)
    np.save(os.path.join(data_dir, 'dynamic_obstacles.npy'), dynamic_obstacles)

    print(f"Environment generated and saved to {data_dir}/")
    print(f"  Grid shape: {grid.shape}")
    print(f"  Occupied voxels: {np.sum(grid)}")
    print(f"  Buildings: {len(buildings)}")
    print(f"  No-fly zones: {len(no_fly_zones)}")
    print(f"  Start: {start}, Goal: {goal}")
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Dynamic obstacles: {len(dynamic_obstacles)}")

    return grid, env_params, obstacles, no_fly_zones, waypoints, dynamic_obstacles


if __name__ == '__main__':
    generate_environment()
