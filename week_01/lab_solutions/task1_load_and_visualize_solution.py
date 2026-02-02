#!/usr/bin/env python3
"""
Task 1: Load and Visualize Point Cloud Data - SOLUTION
=======================================================
INSTRUCTOR VERSION - DO NOT DISTRIBUTE TO STUDENTS
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")


def load_point_cloud_npy(filepath: str) -> np.ndarray:
    """Load a point cloud from a .npy file."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")

    points = np.load(filepath)

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"Expected Nx3 array, got shape {points.shape}")

    return points


def load_point_cloud_pcd(filepath: str) -> np.ndarray:
    """Load a point cloud from an ASCII PCD file."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")

    points = []
    data_section = False

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if data_section:
                if line:
                    parts = line.split()
                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            elif line.startswith("DATA"):
                data_section = True

    if not points:
        raise ValueError("No point data found in PCD file")

    return np.array(points)


def compute_statistics(points: np.ndarray) -> dict:
    """Compute basic statistics of a point cloud."""
    min_bound = np.min(points, axis=0)
    max_bound = np.max(points, axis=0)

    return {
        'num_points': len(points),
        'centroid': np.mean(points, axis=0),
        'min_bound': min_bound,
        'max_bound': max_bound,
        'std': np.std(points, axis=0),
        'extent': max_bound - min_bound,
    }


def visualize_point_cloud(points: np.ndarray, title: str = "Point Cloud",
                          color_by_height: bool = True, subsample: int = None):
    """Visualize a point cloud in 3D using matplotlib."""
    display_pts = points
    if subsample is not None and len(points) > subsample:
        indices = np.random.choice(len(points), subsample, replace=False)
        display_pts = points[indices]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    if color_by_height:
        colors = display_pts[:, 2]
        sc = ax.scatter(display_pts[:, 0], display_pts[:, 1], display_pts[:, 2],
                        c=colors, cmap='viridis', s=0.5, alpha=0.6)
        plt.colorbar(sc, ax=ax, label='Height (z)', shrink=0.6)
    else:
        ax.scatter(display_pts[:, 0], display_pts[:, 1], display_pts[:, 2],
                   s=0.5, alpha=0.6, c='blue')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)

    # Equal aspect ratio
    max_range = np.max(np.ptp(display_pts, axis=0)) / 2.0
    mid = np.mean(display_pts, axis=0)
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print("=" * 60)
    print("Task 1 SOLUTION: Load and Visualize Point Cloud Data")
    print("=" * 60)

    npy_path = os.path.join(DATA_DIR, "sample_pointcloud.npy")
    print(f"\nLoading from {npy_path}")
    points_npy = load_point_cloud_npy(npy_path)
    print(f"  Loaded {len(points_npy)} points, shape: {points_npy.shape}")

    pcd_path = os.path.join(DATA_DIR, "sample_pointcloud.pcd")
    print(f"\nLoading from {pcd_path}")
    points_pcd = load_point_cloud_pcd(pcd_path)
    print(f"  Loaded {len(points_pcd)} points, shape: {points_pcd.shape}")

    stats = compute_statistics(points_npy)
    print("\nStatistics:")
    for key, val in stats.items():
        print(f"  {key}: {val}")

    visualize_point_cloud(points_npy, title="Raw LiDAR Point Cloud", subsample=5000)
    print("\nTask 1 SOLUTION complete!")
