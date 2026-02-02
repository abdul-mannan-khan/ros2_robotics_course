#!/usr/bin/env python3
"""
Week 5 - Task 1 Solution: Costmap Layers
Complete implementation of Nav2-style layered costmap.
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises', 'data')
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


def load_data():
    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    scans = np.load(os.path.join(DATA_DIR, 'laser_scans.npy'))
    scan_poses = np.load(os.path.join(DATA_DIR, 'scan_poses.npy'))
    return grid_map, metadata, scans, scan_poses


def create_static_layer(grid_map):
    """Convert occupancy grid to cost values."""
    costmap = np.zeros_like(grid_map, dtype=np.uint8)
    costmap[grid_map == 100] = 254   # LETHAL_OBSTACLE
    costmap[grid_map == -1] = 255    # NO_INFORMATION
    # grid_map == 0 stays 0 (FREE_SPACE)
    print("[Static Layer] Lethal cells:", np.sum(costmap == 254),
          "Free cells:", np.sum(costmap == 0),
          "Unknown cells:", np.sum(costmap == 255))
    return costmap


def create_obstacle_layer(grid_map, robot_pose, scan, resolution=0.05, max_range=3.5):
    """Create obstacle layer from laser scan."""
    height, width = grid_map.shape
    costmap = np.zeros((height, width), dtype=np.uint8)
    px, py, theta = robot_pose
    num_beams = len(scan)
    angles = np.linspace(-np.pi, np.pi, num_beams, endpoint=False)

    for i, (r, angle) in enumerate(zip(scan, angles)):
        if r >= max_range:
            continue
        beam_angle = theta + angle
        wx = px + r * np.cos(beam_angle)
        wy = py + r * np.sin(beam_angle)
        gx = int(wx / resolution)
        gy = int(wy / resolution)
        if 0 <= gx < width and 0 <= gy < height:
            costmap[gy, gx] = 254  # LETHAL

    print(f"[Obstacle Layer] Marked {np.sum(costmap == 254)} lethal cells from scan")
    return costmap


def inflate_costmap(costmap, inflation_radius=0.5, inscribed_radius=0.15, resolution=0.05):
    """Apply exponential decay inflation around lethal cells."""
    inflated = costmap.copy()
    infl_cells = int(inflation_radius / resolution)
    inscr_cells = int(inscribed_radius / resolution)

    # Gain for exponential decay so cost = 1 at inflation_radius
    if inflation_radius > inscribed_radius:
        gain = 1.0 / (inflation_radius - inscribed_radius)
    else:
        gain = 10.0

    lethal_positions = np.argwhere(costmap == 254)
    height, width = costmap.shape

    for ly, lx in lethal_positions:
        for dy in range(-infl_cells, infl_cells + 1):
            for dx in range(-infl_cells, infl_cells + 1):
                ny, nx = ly + dy, lx + dx
                if ny < 0 or ny >= height or nx < 0 or nx >= width:
                    continue
                dist_cells = np.sqrt(dy*dy + dx*dx)
                dist_m = dist_cells * resolution
                if dist_m <= inscribed_radius:
                    new_cost = 253  # INSCRIBED
                elif dist_m <= inflation_radius:
                    factor = np.exp(-gain * (dist_m - inscribed_radius))
                    new_cost = int(253 * factor)
                else:
                    continue
                if new_cost > inflated[ny, nx] and inflated[ny, nx] != 254:
                    inflated[ny, nx] = new_cost

    print(f"[Inflation] Inscribed cells: {np.sum(inflated == 253)}, "
          f"Inflated cells: {np.sum((inflated > 0) & (inflated < 253))}")
    return inflated


def combine_layers(*layers):
    """Combine layers using element-wise maximum."""
    result = layers[0].copy()
    for layer in layers[1:]:
        result = np.maximum(result, layer)
    print(f"[Combined] Max cost: {result.max()}, Non-zero cells: {np.sum(result > 0)}")
    return result


def main():
    print("=" * 60)
    print("Task 1: Costmap Layers")
    print("=" * 60)
    print()
    print("Nav2 uses a layered costmap approach where multiple sources")
    print("of obstacle information are combined into a single costmap.")
    print()

    grid_map, metadata, scans, scan_poses = load_data()
    resolution = metadata[0]
    print(f"Map: {grid_map.shape}, resolution: {resolution}m/cell")
    print()

    # Create layers
    print("--- Creating Static Layer ---")
    static = create_static_layer(grid_map)

    print("\n--- Creating Obstacle Layer ---")
    obstacle = create_obstacle_layer(grid_map, scan_poses[0], scans[0], resolution)

    print("\n--- Creating Inflation Layer ---")
    inflated = inflate_costmap(static, inflation_radius=0.5, inscribed_radius=0.15,
                               resolution=resolution)

    print("\n--- Combining Layers ---")
    combined = combine_layers(static, obstacle, inflated)

    # Visualize
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))

    titles = ['Static Layer', 'Obstacle Layer (from scan)', 'Inflation Layer', 'Combined Costmap']
    data = [static, obstacle, inflated, combined]

    for ax, title, d in zip(axes.flat, titles, data):
        # Mask 255 (no-info) differently
        display = d.astype(float)
        display[d == 255] = np.nan
        im = ax.imshow(display, cmap='hot', origin='lower', vmin=0, vmax=254)
        ax.set_title(title, fontsize=13)
        ax.set_xlabel('X (cells)')
        ax.set_ylabel('Y (cells)')
        plt.colorbar(im, ax=ax, shrink=0.8)

    # Mark robot position on obstacle layer
    rpose = scan_poses[0]
    gx, gy = int(rpose[0] / resolution), int(rpose[1] / resolution)
    axes[0, 1].plot(gx, gy, 'c^', markersize=10)

    plt.suptitle('Nav2 Costmap Layers', fontsize=15)
    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, 'task1_costmap.png')
    plt.savefig(out_path, dpi=100)
    print(f"\nSaved plot to {out_path}")

    print("\n--- Summary ---")
    print("The costmap combines information from multiple sources:")
    print("  Static layer: pre-built map obstacles (walls, known obstacles)")
    print("  Obstacle layer: real-time sensor detections")
    print("  Inflation layer: safety buffer for robot footprint")
    print("  Combined: maximum of all layers for conservative planning")
    print(f"\nCost values: FREE=0, INSCRIBED=253, LETHAL=254, NO_INFO=255")


if __name__ == '__main__':
    main()
