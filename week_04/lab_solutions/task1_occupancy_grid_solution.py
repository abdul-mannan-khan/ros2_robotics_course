#!/usr/bin/env python3
"""
Week 4 - Task 1 Solution: Occupancy Grid Mapping with Known Poses

Builds an occupancy grid map using log-odds Bayesian updates,
Bresenham ray casting, and an inverse sensor model.
"""

import numpy as np
import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


def initialize_grid(width, height, resolution):
    """Create a log-odds occupancy grid initialized to 0 (P=0.5 = unknown)."""
    cols = int(width / resolution)
    rows = int(height / resolution)
    grid = np.zeros((rows, cols), dtype=np.float64)
    params = {
        "resolution": resolution,
        "width": width,
        "height": height,
        "rows": rows,
        "cols": cols,
    }
    return grid, params


def log_odds_update(prior, measurement):
    """Bayesian update in log-odds: l_new = l_prior + l_measurement."""
    return prior + measurement


def bresenham_ray(x0, y0, x1, y1):
    """Bresenham's line algorithm returning list of (x, y) grid cells."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells


def inverse_sensor_model(cell_dist, endpoint_dist, max_range, l_occ=0.85, l_free=-0.4):
    """
    Inverse sensor model in log-odds.
    Cells near the endpoint are occupied; cells before it are free.
    """
    if endpoint_dist >= max_range - 0.1:
        # Max range reading: mark ray as free, endpoint unknown
        if cell_dist < endpoint_dist - 0.2:
            return l_free
        return 0.0
    if abs(cell_dist - endpoint_dist) < 0.3:
        return l_occ
    if cell_dist < endpoint_dist:
        return l_free
    return 0.0


def update_grid_with_scan(grid, pose, scan, params, max_range=12.0, num_rays=360):
    """Update the occupancy grid with a full LiDAR scan from a known pose."""
    res = params["resolution"]
    rows, cols = params["rows"], params["cols"]
    x, y, theta = pose
    gx = int(x / res)
    gy = int(y / res)

    angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False) + theta

    for i in range(num_rays):
        r = scan[i]
        ex = x + r * np.cos(angles[i])
        ey = y + r * np.sin(angles[i])
        egx = int(np.clip(ex / res, 0, cols - 1))
        egy = int(np.clip(ey / res, 0, rows - 1))

        cells = bresenham_ray(gx, gy, egx, egy)
        for cx, cy in cells:
            if 0 <= cx < cols and 0 <= cy < rows:
                cd = np.sqrt((cx * res - x)**2 + (cy * res - y)**2)
                update = inverse_sensor_model(cd, r, max_range)
                grid[cy, cx] = log_odds_update(grid[cy, cx], update)
                # Clamp to prevent overflow
                grid[cy, cx] = np.clip(grid[cy, cx], -10.0, 10.0)

    return grid


def log_odds_to_probability(grid):
    """Convert log-odds to probability: P = 1 - 1/(1 + exp(l))."""
    return 1.0 - 1.0 / (1.0 + np.exp(grid))


def main():
    print("=" * 60)
    print("Task 1: Occupancy Grid Mapping with Known Poses")
    print("=" * 60)

    # Load data
    poses = np.load(os.path.join(DATA_DIR, "ground_truth_poses.npy"))
    scans = np.load(os.path.join(DATA_DIR, "lidar_scans.npy"))
    gt_env = np.load(os.path.join(DATA_DIR, "environment.npy"))

    print(f"\nLoaded {len(poses)} poses and scans.")
    print(f"Environment ground truth: {gt_env.shape}")

    # Initialize grid
    print("\n[Step 1] Initializing 500x500 log-odds grid (50m x 50m, 0.1m resolution)...")
    grid, params = initialize_grid(50.0, 50.0, 0.1)
    print(f"  Grid shape: {grid.shape}, all values = {grid[0,0]} (log-odds of P=0.5)")

    # Update grid with all scans
    print("\n[Step 2] Integrating LiDAR scans using Bresenham ray casting...")
    print("  Using inverse sensor model: l_occ=0.85, l_free=-0.4")
    for i in range(len(poses)):
        grid = update_grid_with_scan(grid, poses[i], scans[i], params)
        if (i + 1) % 50 == 0:
            print(f"  Processed scan {i+1}/{len(poses)}")

    # Convert to probability
    print("\n[Step 3] Converting log-odds to probability...")
    prob_grid = log_odds_to_probability(grid)
    print(f"  Min probability: {prob_grid.min():.4f}")
    print(f"  Max probability: {prob_grid.max():.4f}")
    print(f"  Cells classified occupied (P>0.7): {(prob_grid > 0.7).sum()}")
    print(f"  Cells classified free (P<0.3): {(prob_grid < 0.3).sum()}")

    # Visualize
    print("\n[Step 4] Creating visualization...")
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    axes[0].imshow(gt_env, cmap='gray_r', origin='lower')
    axes[0].set_title("Ground Truth Environment")
    axes[0].set_xlabel("x (cells)")
    axes[0].set_ylabel("y (cells)")

    axes[1].imshow(prob_grid, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    axes[1].plot(poses[:, 0] / 0.1, poses[:, 1] / 0.1, 'b-', linewidth=0.5, alpha=0.5)
    axes[1].set_title("Built Occupancy Map (Known Poses)")

    # Difference
    diff = np.abs(gt_env - (prob_grid > 0.5).astype(float))
    axes[2].imshow(diff, cmap='Reds', origin='lower', vmin=0, vmax=1)
    axes[2].set_title("Difference (red = mismatch)")

    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, "task1_occupancy_grid.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"  Saved: {out_path}")

    # Accuracy metric
    explored = (prob_grid > 0.7) | (prob_grid < 0.3)
    correct = ((prob_grid > 0.7) & (gt_env > 0.5)) | ((prob_grid < 0.3) & (gt_env < 0.5))
    if explored.sum() > 0:
        acc = correct.sum() / explored.sum()
        print(f"\n[Result] Mapping accuracy (explored cells): {acc:.2%}")

    print("\nTask 1 complete.")


if __name__ == "__main__":
    main()
