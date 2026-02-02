#!/usr/bin/env python3
"""
Task 2: Voxel Grid Downsampling - SOLUTION
============================================
INSTRUCTOR VERSION - DO NOT DISTRIBUTE TO STUDENTS
"""

import numpy as np
import time
import os

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")


def voxel_grid_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Downsample a point cloud using voxel grid filtering."""
    if len(points) == 0:
        return np.empty((0, 3))

    # Compute voxel indices for each point
    min_bound = np.min(points, axis=0)
    voxel_indices = np.floor((points - min_bound) / voxel_size).astype(np.int32)

    # Use a dictionary to group points by voxel
    voxel_map = {}
    for i, idx in enumerate(voxel_indices):
        key = tuple(idx)
        if key not in voxel_map:
            voxel_map[key] = []
        voxel_map[key].append(i)

    # Compute centroid for each voxel
    centroids = np.empty((len(voxel_map), 3))
    for j, indices in enumerate(voxel_map.values()):
        centroids[j] = np.mean(points[indices], axis=0)

    return centroids


def compare_voxel_sizes(points: np.ndarray, voxel_sizes: list) -> dict:
    """Compare downsampling results for different voxel sizes."""
    results = {}
    n_input = len(points)

    for vs in voxel_sizes:
        start = time.time()
        downsampled = voxel_grid_downsample(points, vs)
        elapsed_ms = (time.time() - start) * 1000

        results[vs] = {
            'num_points': len(downsampled),
            'reduction_ratio': len(downsampled) / n_input if n_input > 0 else 0,
            'time_ms': elapsed_ms,
            'points': downsampled,
        }

    return results


if __name__ == "__main__":
    print("=" * 60)
    print("Task 2 SOLUTION: Voxel Grid Downsampling")
    print("=" * 60)

    points = np.load(os.path.join(DATA_DIR, "sample_pointcloud.npy"))
    print(f"Original point cloud: {len(points)} points")

    print("\n[Test 1] Voxel size = 0.5m")
    downsampled = voxel_grid_downsample(points, voxel_size=0.5)
    print(f"  Result: {len(downsampled)} points ({len(downsampled)/len(points)*100:.1f}%)")

    print("\n[Test 2] Comparing voxel sizes...")
    sizes = [0.1, 0.2, 0.5, 1.0, 2.0]
    results = compare_voxel_sizes(points, sizes)
    print(f"  {'Voxel Size':>10} | {'Points':>8} | {'Reduction':>10} | {'Time':>8}")
    print(f"  {'-'*10} | {'-'*8} | {'-'*10} | {'-'*8}")
    for vs in sizes:
        r = results[vs]
        print(f"  {vs:>10.2f} | {r['num_points']:>8d} | {r['reduction_ratio']:>9.1%} | {r['time_ms']:>6.1f}ms")

    # Edge cases
    single_point = np.array([[1.0, 2.0, 3.0]])
    result = voxel_grid_downsample(single_point, 0.5)
    assert len(result) == 1
    print("\n  Single point: PASSED")

    empty = np.empty((0, 3))
    result = voxel_grid_downsample(empty, 0.5)
    assert len(result) == 0
    print("  Empty cloud: PASSED")

    print("\nTask 2 SOLUTION complete!")
