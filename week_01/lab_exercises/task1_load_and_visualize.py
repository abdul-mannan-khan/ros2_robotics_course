#!/usr/bin/env python3
"""
Task 1: Load and Visualize Point Cloud Data
============================================

Objectives:
- Load point cloud data from .npy and .pcd files
- Visualize point clouds in 3D using matplotlib and Open3D
- Understand point cloud data structure and statistics

Instructions:
- Complete all functions marked with TODO
- Run this file to verify your implementation
- Do NOT modify function signatures
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def load_point_cloud_npy(filepath: str) -> np.ndarray:
    """Load a point cloud from a .npy file.

    Args:
        filepath: Path to .npy file containing Nx3 array of (x, y, z) points.

    Returns:
        np.ndarray: Point cloud as Nx3 array.

    Raises:
        FileNotFoundError: If file does not exist.
        ValueError: If array is not Nx3.
    """
    # TODO: Implement this function
    # 1. Load the .npy file using numpy
    # 2. Validate shape is (N, 3)
    # 3. Return the array
    raise NotImplementedError("Complete this function")


def load_point_cloud_pcd(filepath: str) -> np.ndarray:
    """Load a point cloud from an ASCII PCD file.

    Read the PCD file manually (do NOT use Open3D for this function).
    Parse the header to find where DATA begins, then read the points.

    Args:
        filepath: Path to .pcd file.

    Returns:
        np.ndarray: Point cloud as Nx3 array.
    """
    # TODO: Implement this function
    # 1. Open and read the file
    # 2. Parse header lines until you find "DATA ascii"
    # 3. Read remaining lines as x, y, z float values
    # 4. Return as Nx3 numpy array
    raise NotImplementedError("Complete this function")


def compute_statistics(points: np.ndarray) -> dict:
    """Compute basic statistics of a point cloud.

    Args:
        points: Nx3 array of (x, y, z) points.

    Returns:
        dict with keys:
            'num_points': int - total number of points
            'centroid': np.ndarray - mean (x, y, z)
            'min_bound': np.ndarray - minimum (x, y, z)
            'max_bound': np.ndarray - maximum (x, y, z)
            'std': np.ndarray - standard deviation per axis
            'extent': np.ndarray - max_bound - min_bound
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def visualize_point_cloud(points: np.ndarray, title: str = "Point Cloud",
                          color_by_height: bool = True, subsample: int = None):
    """Visualize a point cloud in 3D using matplotlib.

    Args:
        points: Nx3 array of (x, y, z) points.
        title: Plot title.
        color_by_height: If True, color points by z-value.
        subsample: If set, randomly subsample to this many points for display.
    """
    # TODO: Implement this function
    # 1. Optionally subsample points for faster rendering
    # 2. Create a 3D scatter plot
    # 3. Color by z-axis if color_by_height is True
    # 4. Add axis labels (X, Y, Z) and title
    # 5. Set equal aspect ratio for all axes
    # 6. Show the plot
    raise NotImplementedError("Complete this function")


# =============================================================================
# Main - Run this to test your implementations
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("Task 1: Load and Visualize Point Cloud Data")
    print("=" * 60)

    # Test 1: Load from .npy
    npy_path = os.path.join(DATA_DIR, "sample_pointcloud.npy")
    print(f"\n[Test 1] Loading from {npy_path}")
    points_npy = load_point_cloud_npy(npy_path)
    print(f"  Loaded {len(points_npy)} points, shape: {points_npy.shape}")

    # Test 2: Load from .pcd
    pcd_path = os.path.join(DATA_DIR, "sample_pointcloud.pcd")
    print(f"\n[Test 2] Loading from {pcd_path}")
    points_pcd = load_point_cloud_pcd(pcd_path)
    print(f"  Loaded {len(points_pcd)} points, shape: {points_pcd.shape}")

    # Verify both loaders give same data
    assert points_npy.shape == points_pcd.shape, "Shape mismatch between loaders!"
    print("  Both loaders return same shape.")

    # Test 3: Compute statistics
    print("\n[Test 3] Computing statistics...")
    stats = compute_statistics(points_npy)
    for key, val in stats.items():
        print(f"  {key}: {val}")

    # Test 4: Visualize
    print("\n[Test 4] Visualizing point cloud...")
    visualize_point_cloud(points_npy, title="Raw LiDAR Point Cloud", subsample=5000)

    print("\nTask 1 complete!")
