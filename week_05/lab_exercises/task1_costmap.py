#!/usr/bin/env python3
"""
Week 5 - Task 1: Costmap Layers
Implement costmap generation with static, obstacle, and inflation layers.

Nav2 uses a layered costmap approach:
  - Static layer: from the pre-built map
  - Obstacle layer: from real-time sensor data
  - Inflation layer: safety buffer around obstacles

Cost values: 0 = free, 253 = inscribed, 254 = lethal, 255 = no-info
"""

import os
import sys
import numpy as np

# Allow importing data from the data directory
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


def load_data():
    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    scans = np.load(os.path.join(DATA_DIR, 'laser_scans.npy'))
    scan_poses = np.load(os.path.join(DATA_DIR, 'scan_poses.npy'))
    return grid_map, metadata, scans, scan_poses


def create_static_layer(grid_map):
    """Convert occupancy grid to costmap values (0-254).

    Mapping:
      -1 (unknown) -> 255 (NO_INFORMATION)
      0 (free) -> 0 (FREE_SPACE)
      100 (occupied) -> 254 (LETHAL_OBSTACLE)

    Returns:
        numpy array of same shape with uint8 cost values
    """
    # TODO: Implement static layer conversion
    # Hint: Use np.where or conditional assignment
    costmap = np.zeros_like(grid_map, dtype=np.uint8)
    raise NotImplementedError("Implement create_static_layer")
    return costmap


def create_obstacle_layer(grid_map, robot_pose, scan, resolution=0.05, max_range=3.5):
    """Create obstacle layer from a laser scan at a given pose.

    For each beam in the scan:
      - If range < max_range, mark the endpoint cell as LETHAL (254)
      - Optionally, raytrace free space along the beam

    Args:
        grid_map: reference map (for shape)
        robot_pose: (x, y, theta) in world coordinates
        scan: 1D array of range values
        resolution: meters per cell
        max_range: maximum sensor range

    Returns:
        obstacle costmap (uint8)
    """
    # TODO: Implement obstacle layer from scan data
    # Hint: Convert polar scan to cartesian, then to grid coordinates
    height, width = grid_map.shape
    costmap = np.zeros((height, width), dtype=np.uint8)
    raise NotImplementedError("Implement create_obstacle_layer")
    return costmap


def inflate_costmap(costmap, inflation_radius=0.5, inscribed_radius=0.15, resolution=0.05):
    """Apply exponential decay inflation around lethal cells.

    For each lethal cell (254), set costs in surrounding cells:
      - Within inscribed_radius: cost = 253 (INSCRIBED)
      - Within inflation_radius: cost = 253 * exp(-gain * (dist - inscribed_radius))
      - Beyond inflation_radius: unchanged

    Args:
        costmap: input costmap with lethal obstacles
        inflation_radius: outer inflation radius (meters)
        inscribed_radius: robot inscribed radius (meters)
        resolution: meters per cell

    Returns:
        inflated costmap (uint8)
    """
    # TODO: Implement inflation with exponential decay
    # Hint: Find all lethal cells, iterate outward, apply decay function
    inflated = costmap.copy()
    raise NotImplementedError("Implement inflate_costmap")
    return inflated


def combine_layers(*layers):
    """Combine multiple costmap layers using element-wise maximum.

    Args:
        *layers: variable number of costmap arrays

    Returns:
        combined costmap (uint8)
    """
    # TODO: Implement layer combination
    raise NotImplementedError("Implement combine_layers")


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    grid_map, metadata, scans, scan_poses = load_data()
    resolution = metadata[0]

    # Create each layer
    static = create_static_layer(grid_map)
    obstacle = create_obstacle_layer(grid_map, scan_poses[0], scans[0], resolution)
    inflated = inflate_costmap(static, inflation_radius=0.5, inscribed_radius=0.15,
                               resolution=resolution)
    combined = combine_layers(static, obstacle, inflated)

    # Visualize
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    titles = ['Static Layer', 'Obstacle Layer', 'Inflation Layer', 'Combined Costmap']
    data = [static, obstacle, inflated, combined]
    for ax, title, d in zip(axes.flat, titles, data):
        ax.imshow(d, cmap='hot', origin='lower', vmin=0, vmax=254)
        ax.set_title(title)
        ax.set_xlabel('X (cells)')
        ax.set_ylabel('Y (cells)')

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task1_costmap.png')
    plt.savefig(out_path, dpi=100)
    print(f"Saved plot to {out_path}")


if __name__ == '__main__':
    main()
