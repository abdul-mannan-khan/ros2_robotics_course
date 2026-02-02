#!/usr/bin/env python3
"""
Week 4 - Task 2: Iterative Closest Point (ICP) Scan Matching

Implement ICP to align consecutive LiDAR scans.

Functions to implement:
- scan_to_points(ranges, angles)
- find_correspondences(source, target, max_dist)
- estimate_transform(source, target)
- apply_transform(points, R, t)
- icp(source, target, max_iterations, tolerance)
- main()
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def scan_to_points(ranges, angles, max_range=11.5):
    """
    Convert polar LiDAR scan to Cartesian points.

    Args:
        ranges: Array of range values
        angles: Array of corresponding angles (radians)
        max_range: Discard points beyond this range

    Returns:
        points: Nx2 array of [x, y] points
    """
    # TODO: Convert polar to Cartesian
    # TODO: Filter out max-range readings
    raise NotImplementedError("Implement scan_to_points")


def find_correspondences(source, target, max_dist=1.0):
    """
    Find nearest-neighbor correspondences from source to target.

    Args:
        source: Nx2 source points
        target: Mx2 target points
        max_dist: Maximum allowed correspondence distance

    Returns:
        src_matched: Kx2 matched source points
        tgt_matched: Kx2 matched target points
    """
    # TODO: For each source point, find the nearest target point
    # TODO: Reject correspondences beyond max_dist
    raise NotImplementedError("Implement find_correspondences")


def estimate_transform(source, target):
    """
    Estimate 2D rigid transform (R, t) using SVD.
    Minimizes sum of ||target_i - (R @ source_i + t)||^2.

    Args:
        source: Nx2 source points
        target: Nx2 target points (correspondences)

    Returns:
        R: 2x2 rotation matrix
        t: 2x1 translation vector
    """
    # TODO: Center both point sets
    # TODO: Compute cross-covariance matrix H = src_centered.T @ tgt_centered
    # TODO: SVD of H
    # TODO: R = V @ U.T (handle reflection case)
    # TODO: t = tgt_mean - R @ src_mean
    raise NotImplementedError("Implement estimate_transform")


def apply_transform(points, R, t):
    """
    Apply rigid transform to point set.

    Args:
        points: Nx2 points
        R: 2x2 rotation matrix
        t: 2x1 translation vector

    Returns:
        Transformed Nx2 points
    """
    # TODO: Apply R and t to each point
    raise NotImplementedError("Implement apply_transform")


def icp(source, target, max_iterations=50, tolerance=1e-5, max_dist=1.0):
    """
    Full ICP algorithm.

    Args:
        source: Nx2 source points
        target: Mx2 target points
        max_iterations: Maximum number of iterations
        tolerance: Convergence threshold on transform change
        max_dist: Max correspondence distance

    Returns:
        R_total: Cumulative 2x2 rotation
        t_total: Cumulative 2x1 translation
        transformed_source: Final aligned source points
        errors: List of mean error at each iteration
    """
    # TODO: Initialize cumulative transform to identity
    # TODO: Iterate:
    #   1. Find correspondences
    #   2. Estimate transform
    #   3. Apply transform to source
    #   4. Accumulate transform
    #   5. Check convergence
    raise NotImplementedError("Implement icp")


def main():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    out_dir = os.path.dirname(os.path.abspath(__file__))

    scans = np.load(os.path.join(data_dir, "lidar_scans.npy"))
    poses = np.load(os.path.join(data_dir, "ground_truth_poses.npy"))

    print("=== Task 2: ICP Scan Matching ===")

    # TODO: Pick two consecutive scans
    # TODO: Convert to points in robot frame
    # TODO: Transform source scan by known relative pose (to test ICP)
    # TODO: Run ICP to recover the transform
    # TODO: Compare estimated vs true transform
    # TODO: Plot before/after alignment
    # TODO: Save plots

    print("Task 2 not yet implemented. Complete the functions above!")


if __name__ == "__main__":
    main()
