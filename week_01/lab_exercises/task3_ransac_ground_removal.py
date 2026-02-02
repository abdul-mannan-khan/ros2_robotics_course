#!/usr/bin/env python3
"""
Task 3: Implement RANSAC for Ground Plane Removal
===================================================

Objectives:
- Implement RANSAC plane fitting from scratch
- Segment ground plane from obstacle points
- Understand parameter tuning (iterations, threshold)

Instructions:
- Complete all functions marked with TODO
- Run this file to verify your implementation
- Do NOT modify function signatures
"""

import numpy as np
import os

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def fit_plane_from_3_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> tuple:
    """Fit a plane through exactly 3 points.

    A plane is defined as: ax + by + cz + d = 0
    where (a, b, c) is the normal vector.

    Args:
        p1, p2, p3: Each a 1D array of (x, y, z).

    Returns:
        tuple: (a, b, c, d) plane coefficients with unit normal.
               Returns None if points are collinear.

    Steps:
        1. Compute vectors v1 = p2 - p1, v2 = p3 - p1
        2. Compute normal: n = v1 x v2 (cross product)
        3. If |n| ≈ 0, points are collinear -> return None
        4. Normalize n to unit length
        5. Compute d = -(n · p1)
        6. Return (a, b, c, d)
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def point_to_plane_distance(points: np.ndarray, plane: tuple) -> np.ndarray:
    """Compute distance from each point to a plane.

    Distance = |ax + by + cz + d| / sqrt(a² + b² + c²)
    (If normal is already unit length, denominator = 1)

    Args:
        points: Nx3 array of points.
        plane: (a, b, c, d) plane coefficients.

    Returns:
        np.ndarray: N distances (all non-negative).
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def ransac_plane_fit(points: np.ndarray, max_iterations: int = 1000,
                     distance_threshold: float = 0.15) -> tuple:
    """Fit a plane to point cloud data using RANSAC.

    Args:
        points: Nx3 array of points.
        max_iterations: Maximum number of RANSAC iterations.
        distance_threshold: Max distance for a point to be an inlier (meters).

    Returns:
        tuple: (plane, inlier_mask)
            - plane: (a, b, c, d) best plane coefficients
            - inlier_mask: boolean array of length N, True for inliers (ground)

    Algorithm:
        1. For each iteration:
           a. Randomly select 3 points
           b. Fit plane through them (skip if collinear)
           c. Compute distances from all points to plane
           d. Count inliers (distance < threshold)
           e. Keep track of best model (most inliers)
        2. Return best plane and its inlier mask
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def segment_ground(points: np.ndarray, plane: tuple,
                   inlier_mask: np.ndarray) -> tuple:
    """Separate ground and obstacle points.

    Args:
        points: Nx3 array of points.
        plane: (a, b, c, d) ground plane coefficients.
        inlier_mask: Boolean array, True = ground point.

    Returns:
        tuple: (ground_points, obstacle_points)
            - ground_points: Mx3 array of ground points
            - obstacle_points: Kx3 array of non-ground points
            Where M + K = N
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def calculate_ransac_iterations(desired_probability: float, inlier_ratio: float,
                                 sample_size: int = 3) -> int:
    """Calculate the required number of RANSAC iterations.

    Formula: N = log(1 - p) / log(1 - w^n)

    Args:
        desired_probability: Probability of finding a good model (e.g., 0.99).
        inlier_ratio: Estimated fraction of inliers (0 to 1).
        sample_size: Minimum points for model (3 for plane).

    Returns:
        int: Required number of iterations (ceiling).
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


# =============================================================================
# Main - Run this to test your implementations
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("Task 3: RANSAC Ground Plane Removal")
    print("=" * 60)

    # Load data
    points = np.load(os.path.join(DATA_DIR, "sample_pointcloud.npy"))
    print(f"Loaded {len(points)} points")

    # Test 1: Plane fitting from 3 points
    print("\n[Test 1] Fit plane from 3 points")
    p1 = np.array([0, 0, 0])
    p2 = np.array([1, 0, 0])
    p3 = np.array([0, 1, 0])
    plane = fit_plane_from_3_points(p1, p2, p3)
    print(f"  Plane (should be z=0): a={plane[0]:.3f}, b={plane[1]:.3f}, c={plane[2]:.3f}, d={plane[3]:.3f}")
    assert abs(abs(plane[2]) - 1.0) < 0.01, "Normal should point in z direction"
    assert abs(plane[3]) < 0.01, "d should be ~0 for plane through origin"
    print("  PASSED")

    # Test 2: Collinear points
    print("\n[Test 2] Collinear points")
    result = fit_plane_from_3_points(np.array([0,0,0]), np.array([1,0,0]), np.array([2,0,0]))
    assert result is None, "Should return None for collinear points"
    print("  PASSED")

    # Test 3: Point-to-plane distance
    print("\n[Test 3] Point-to-plane distance")
    test_points = np.array([[0, 0, 1], [0, 0, 2], [0, 0, 0.5]])
    distances = point_to_plane_distance(test_points, (0, 0, 1, 0))  # z=0 plane
    np.testing.assert_allclose(distances, [1.0, 2.0, 0.5], atol=1e-6)
    print(f"  Distances: {distances}")
    print("  PASSED")

    # Test 4: Calculate iterations
    print("\n[Test 4] Calculate RANSAC iterations")
    n_iter = calculate_ransac_iterations(0.99, 0.6, 3)
    print(f"  p=0.99, w=0.6, n=3 -> {n_iter} iterations")

    # Test 5: RANSAC on sample data
    print("\n[Test 5] RANSAC ground segmentation")
    plane, inlier_mask = ransac_plane_fit(points, max_iterations=500, distance_threshold=0.15)
    ground, obstacles = segment_ground(points, plane, inlier_mask)
    print(f"  Plane: a={plane[0]:.4f}, b={plane[1]:.4f}, c={plane[2]:.4f}, d={plane[3]:.4f}")
    print(f"  Ground points: {len(ground)}")
    print(f"  Obstacle points: {len(obstacles)}")
    print(f"  Ground normal should be ~(0, 0, 1): ({plane[0]:.3f}, {plane[1]:.3f}, {plane[2]:.3f})")

    print("\nTask 3 complete!")
