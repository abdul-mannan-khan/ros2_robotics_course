#!/usr/bin/env python3
"""
Week 10 - Task 3: Homography Estimation
Implement DLT homography, RANSAC, and homography decomposition.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def compute_homography_dlt(src_pts, dst_pts):
    """
    Compute homography using Direct Linear Transform (SVD).

    Args:
        src_pts: Nx2 source points
        dst_pts: Nx2 destination points

    Returns:
        H: 3x3 homography matrix
    """
    # TODO: Build 2Nx9 matrix A, solve Ah=0 using SVD
    raise NotImplementedError("Implement compute_homography_dlt")


def ransac_homography(src_pts, dst_pts, threshold=3.0, max_iter=1000):
    """
    RANSAC-based robust homography estimation.

    Args:
        src_pts: Nx2 source points
        dst_pts: Nx2 destination points
        threshold: inlier distance threshold in pixels
        max_iter: maximum iterations

    Returns:
        H: 3x3 best homography
        inlier_mask: boolean array of length N
    """
    # TODO: Implement RANSAC
    # 1. Randomly sample 4 points
    # 2. Compute homography from sample
    # 3. Count inliers
    # 4. Keep best model, refit on all inliers
    raise NotImplementedError("Implement ransac_homography")


def apply_homography(H, points):
    """
    Apply homography to a set of points.

    Args:
        H: 3x3 homography matrix
        points: Nx2 points

    Returns:
        Nx2 transformed points
    """
    # TODO: Convert to homogeneous, multiply by H, normalize
    raise NotImplementedError("Implement apply_homography")


def decompose_homography(H, K):
    """
    Decompose homography into rotation and translation.

    Args:
        H: 3x3 homography matrix
        K: 3x3 camera intrinsic matrix

    Returns:
        R: 3x3 rotation matrix
        t: 3x1 translation vector
        n: 3x1 normal vector of the plane
    """
    # TODO: H = K (R + t*n^T) K^{-1}
    # Compute K^{-1} H K, then decompose using SVD
    raise NotImplementedError("Implement decompose_homography")


def main():
    """Main function: estimate homography, visualize inliers/outliers."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, 'data')

    print("Week 10 - Task 3: Homography Estimation")
    print("=" * 42)

    pts1 = np.load(os.path.join(data_dir, 'feature_points_1.npy'))
    pts2 = np.load(os.path.join(data_dir, 'feature_points_2.npy'))

    print(f"Source points: {pts1.shape}")
    print(f"Destination points: {pts2.shape}")

    # TODO: Estimate homography with RANSAC
    # TODO: Separate inliers and outliers
    # TODO: Visualize warped points
    # TODO: Decompose homography

    print("\nImplement the functions above and complete main().")


if __name__ == '__main__':
    main()
