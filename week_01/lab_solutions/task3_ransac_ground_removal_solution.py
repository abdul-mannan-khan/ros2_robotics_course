#!/usr/bin/env python3
"""
Task 3: RANSAC Ground Plane Removal - SOLUTION
================================================
INSTRUCTOR VERSION - DO NOT DISTRIBUTE TO STUDENTS
"""

import numpy as np
import math
import os

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")


def fit_plane_from_3_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> tuple:
    """Fit a plane through exactly 3 points."""
    v1 = p2 - p1
    v2 = p3 - p1
    normal = np.cross(v1, v2)
    norm_len = np.linalg.norm(normal)

    if norm_len < 1e-10:
        return None  # collinear

    normal = normal / norm_len
    a, b, c = normal
    d = -np.dot(normal, p1)
    return (a, b, c, d)


def point_to_plane_distance(points: np.ndarray, plane: tuple) -> np.ndarray:
    """Compute distance from each point to a plane."""
    a, b, c, d = plane
    norm = math.sqrt(a * a + b * b + c * c)
    distances = np.abs(points[:, 0] * a + points[:, 1] * b + points[:, 2] * c + d) / norm
    return distances


def ransac_plane_fit(points: np.ndarray, max_iterations: int = 1000,
                     distance_threshold: float = 0.15) -> tuple:
    """Fit a plane to point cloud data using RANSAC."""
    n = len(points)
    best_plane = None
    best_count = 0
    best_mask = None

    for _ in range(max_iterations):
        # Randomly select 3 points
        idx = np.random.choice(n, 3, replace=False)
        p1, p2, p3 = points[idx[0]], points[idx[1]], points[idx[2]]

        plane = fit_plane_from_3_points(p1, p2, p3)
        if plane is None:
            continue

        distances = point_to_plane_distance(points, plane)
        inlier_mask = distances < distance_threshold
        count = np.sum(inlier_mask)

        if count > best_count:
            best_count = count
            best_plane = plane
            best_mask = inlier_mask

    return best_plane, best_mask


def segment_ground(points: np.ndarray, plane: tuple,
                   inlier_mask: np.ndarray) -> tuple:
    """Separate ground and obstacle points."""
    ground_points = points[inlier_mask]
    obstacle_points = points[~inlier_mask]
    return ground_points, obstacle_points


def calculate_ransac_iterations(desired_probability: float, inlier_ratio: float,
                                 sample_size: int = 3) -> int:
    """Calculate the required number of RANSAC iterations."""
    if inlier_ratio <= 0 or inlier_ratio >= 1:
        return 1
    numerator = math.log(1.0 - desired_probability)
    denominator = math.log(1.0 - inlier_ratio ** sample_size)
    if denominator == 0:
        return 1
    return int(math.ceil(numerator / denominator))


if __name__ == "__main__":
    print("=" * 60)
    print("Task 3 SOLUTION: RANSAC Ground Plane Removal")
    print("=" * 60)

    points = np.load(os.path.join(DATA_DIR, "sample_pointcloud.npy"))
    print(f"Loaded {len(points)} points")

    # Test plane fitting
    p1 = np.array([0, 0, 0])
    p2 = np.array([1, 0, 0])
    p3 = np.array([0, 1, 0])
    plane = fit_plane_from_3_points(p1, p2, p3)
    print(f"\nPlane from 3 points (z=0): a={plane[0]:.3f}, b={plane[1]:.3f}, c={plane[2]:.3f}, d={plane[3]:.3f}")

    # Collinear
    result = fit_plane_from_3_points(np.array([0,0,0]), np.array([1,0,0]), np.array([2,0,0]))
    assert result is None
    print("Collinear test: PASSED")

    # Distances
    test_points = np.array([[0, 0, 1], [0, 0, 2], [0, 0, 0.5]])
    distances = point_to_plane_distance(test_points, (0, 0, 1, 0))
    np.testing.assert_allclose(distances, [1.0, 2.0, 0.5], atol=1e-6)
    print("Distance test: PASSED")

    # Iterations formula
    n_iter = calculate_ransac_iterations(0.99, 0.6, 3)
    print(f"Iterations (p=0.99, w=0.6): {n_iter}")

    # RANSAC on sample data
    np.random.seed(42)
    plane, inlier_mask = ransac_plane_fit(points, max_iterations=500, distance_threshold=0.15)
    ground, obstacles = segment_ground(points, plane, inlier_mask)
    print(f"\nRANSAC result:")
    print(f"  Plane: ({plane[0]:.4f}, {plane[1]:.4f}, {plane[2]:.4f}, {plane[3]:.4f})")
    print(f"  Ground: {len(ground)} points")
    print(f"  Obstacles: {len(obstacles)} points")

    print("\nTask 3 SOLUTION complete!")
