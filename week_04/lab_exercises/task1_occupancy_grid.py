#!/usr/bin/env python3
"""
Week 4 - Task 1: Occupancy Grid Mapping with Known Poses

Build an occupancy grid map using log-odds representation and known (ground truth) poses.

Functions to implement:
- initialize_grid(width, height, resolution)
- log_odds_update(prior, measurement)
- bresenham_ray(x0, y0, x1, y1)
- inverse_sensor_model(cell, endpoint, scan_range)
- update_grid_with_scan(grid, pose, scan, params)
- log_odds_to_probability(grid)
- main()
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def initialize_grid(width, height, resolution):
    """
    Create a log-odds occupancy grid.

    Args:
        width: Environment width in meters
        height: Environment height in meters
        resolution: Cell size in meters

    Returns:
        grid: 2D numpy array of log-odds values (initialized to 0 = unknown = P=0.5)
        params: dict with grid metadata (resolution, origin, shape)
    """
    # TODO: Compute grid dimensions from physical size and resolution
    # TODO: Initialize grid to 0 (log-odds of 0.5 probability)
    # TODO: Return grid and params dict
    raise NotImplementedError("Implement initialize_grid")


def log_odds_update(prior, measurement):
    """
    Bayesian update in log-odds form: l(x|z) = l(x) + l(z) - l0
    where l0 = log(0.5/0.5) = 0.

    Args:
        prior: Current log-odds value
        measurement: Log-odds of measurement (positive = occupied, negative = free)

    Returns:
        Updated log-odds value
    """
    # TODO: Implement log-odds Bayes update
    raise NotImplementedError("Implement log_odds_update")


def bresenham_ray(x0, y0, x1, y1):
    """
    Bresenham's line algorithm to trace a ray through grid cells.

    Args:
        x0, y0: Start cell coordinates
        x1, y1: End cell coordinates

    Returns:
        List of (x, y) cell coordinates along the ray
    """
    # TODO: Implement Bresenham's line algorithm
    # TODO: Return list of grid cells the ray passes through
    raise NotImplementedError("Implement bresenham_ray")


def inverse_sensor_model(cell_dist, endpoint_dist, max_range, l_occ=0.85, l_free=-0.4):
    """
    Inverse sensor model: compute log-odds update for a cell given a range measurement.

    Args:
        cell_dist: Distance from sensor to this cell
        endpoint_dist: Distance from sensor to scan endpoint
        max_range: Maximum sensor range
        l_occ: Log-odds value for occupied cells
        l_free: Log-odds value for free cells

    Returns:
        Log-odds update value
    """
    # TODO: If cell is near the endpoint -> return l_occ (occupied)
    # TODO: If cell is before the endpoint -> return l_free (free)
    # TODO: If beyond endpoint or at max range -> return 0 (unknown)
    raise NotImplementedError("Implement inverse_sensor_model")


def update_grid_with_scan(grid, pose, scan, params):
    """
    Update the occupancy grid with a full LiDAR scan from a known pose.

    Args:
        grid: Log-odds occupancy grid
        pose: Robot pose [x, y, theta]
        scan: Array of 360 range values
        params: Grid parameters (resolution, etc.)

    Returns:
        Updated grid
    """
    # TODO: For each ray in the scan:
    #   1. Compute the endpoint in world coordinates
    #   2. Convert robot position and endpoint to grid coordinates
    #   3. Use bresenham_ray to get cells along the ray
    #   4. Use inverse_sensor_model to get log-odds update for each cell
    #   5. Apply log_odds_update to the grid
    raise NotImplementedError("Implement update_grid_with_scan")


def log_odds_to_probability(grid):
    """
    Convert log-odds grid to probability grid.

    Args:
        grid: Log-odds occupancy grid

    Returns:
        Probability grid (values in [0, 1])
    """
    # TODO: Convert using P = 1 - 1/(1 + exp(l))
    raise NotImplementedError("Implement log_odds_to_probability")


def main():
    """
    Build occupancy grid map using ground truth poses and LiDAR scans.
    """
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    out_dir = os.path.dirname(os.path.abspath(__file__))

    # Load data
    poses = np.load(os.path.join(data_dir, "ground_truth_poses.npy"))
    scans = np.load(os.path.join(data_dir, "lidar_scans.npy"))
    gt_env = np.load(os.path.join(data_dir, "environment.npy"))

    print("=== Task 1: Occupancy Grid Mapping ===")
    print(f"Poses: {poses.shape}, Scans: {scans.shape}")

    # TODO: Initialize grid (50m x 50m, 0.1m resolution)
    # TODO: For each pose/scan pair, update the grid
    # TODO: Convert to probability
    # TODO: Visualize: side-by-side ground truth vs built map
    # TODO: Save plot

    print("Task 1 not yet implemented. Complete the functions above!")


if __name__ == "__main__":
    main()
