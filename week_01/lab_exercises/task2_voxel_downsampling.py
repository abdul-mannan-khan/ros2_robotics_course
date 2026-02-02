#!/usr/bin/env python3
"""
Task 2: Implement Voxel Grid Downsampling
==========================================

Objectives:
- Implement voxel grid downsampling from scratch (no Open3D)
- Understand the trade-off between voxel size, point count, and detail
- Compare your implementation against Open3D's built-in method

Instructions:
- Complete all functions marked with TODO
- Run this file to verify your implementation
- Do NOT modify function signatures
"""

import numpy as np
import time
import os

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def voxel_grid_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Downsample a point cloud using voxel grid filtering.

    Divide the 3D space into cubic voxels of the given size.
    Replace all points within each voxel with their centroid.

    Args:
        points: Nx3 array of (x, y, z) points.
        voxel_size: Side length of each cubic voxel in meters.

    Returns:
        np.ndarray: Mx3 downsampled point cloud (M <= N).

    Algorithm:
        1. Compute the minimum bound of the point cloud
        2. Compute voxel indices for each point:
           voxel_idx = floor((point - min_bound) / voxel_size)
        3. Group points by their voxel index
        4. Compute centroid of each group
        5. Return centroids as downsampled cloud

    Hint: Use a dictionary with tuple(voxel_idx) as key, or use
          np.unique with the return_inverse parameter.
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def compare_voxel_sizes(points: np.ndarray, voxel_sizes: list) -> dict:
    """Compare downsampling results for different voxel sizes.

    Args:
        points: Nx3 original point cloud.
        voxel_sizes: List of voxel sizes to test.

    Returns:
        dict mapping voxel_size -> {
            'num_points': int,
            'reduction_ratio': float (0 to 1),
            'time_ms': float,
            'points': np.ndarray
        }
    """
    # TODO: Implement this function
    # For each voxel size:
    #   1. Time the downsampling
    #   2. Record output point count
    #   3. Compute reduction ratio = output_points / input_points
    raise NotImplementedError("Complete this function")


# =============================================================================
# Main - Run this to test your implementations
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("Task 2: Voxel Grid Downsampling")
    print("=" * 60)

    # Load data
    points = np.load(os.path.join(DATA_DIR, "sample_pointcloud.npy"))
    print(f"Original point cloud: {len(points)} points")

    # Test 1: Basic downsampling
    print("\n[Test 1] Voxel size = 0.5m")
    downsampled = voxel_grid_downsample(points, voxel_size=0.5)
    print(f"  Result: {len(downsampled)} points ({len(downsampled)/len(points)*100:.1f}%)")

    # Test 2: Compare different voxel sizes
    print("\n[Test 2] Comparing voxel sizes...")
    sizes = [0.1, 0.2, 0.5, 1.0, 2.0]
    results = compare_voxel_sizes(points, sizes)
    print(f"  {'Voxel Size':>10} | {'Points':>8} | {'Reduction':>10} | {'Time':>8}")
    print(f"  {'-'*10} | {'-'*8} | {'-'*10} | {'-'*8}")
    for vs in sizes:
        r = results[vs]
        print(f"  {vs:>10.2f} | {r['num_points']:>8d} | {r['reduction_ratio']:>9.1%} | {r['time_ms']:>6.1f}ms")

    # Test 3: Edge cases
    print("\n[Test 3] Edge cases")
    single_point = np.array([[1.0, 2.0, 3.0]])
    result = voxel_grid_downsample(single_point, 0.5)
    assert len(result) == 1, f"Single point should return 1 point, got {len(result)}"
    print("  Single point: PASSED")

    empty = np.empty((0, 3))
    result = voxel_grid_downsample(empty, 0.5)
    assert len(result) == 0, f"Empty cloud should return 0 points, got {len(result)}"
    print("  Empty cloud: PASSED")

    print("\nTask 2 complete!")
