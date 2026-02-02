#!/usr/bin/env python3
"""
Week 10 - Task 1: Camera Projection and Calibration
Implement camera projection, undistortion, and calibration using DLT.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def project_point(K, R, t, point_3d):
    """
    Project a 3D point onto the image plane.

    Args:
        K: 3x3 camera intrinsic matrix
        R: 3x3 rotation matrix
        t: 3x1 translation vector
        point_3d: 3D point as (3,) array

    Returns:
        2D point as (2,) array in pixel coordinates
    """
    # TODO: Implement 3D to 2D projection
    # 1. Apply extrinsics: p_cam = R @ point_3d + t
    # 2. Apply intrinsics: p_img = K @ p_cam
    # 3. Normalize by z coordinate
    raise NotImplementedError("Implement project_point")


def undistort_point(point, K, dist_coeffs):
    """
    Remove radial and tangential distortion from an image point.

    Args:
        point: 2D distorted point (2,)
        K: 3x3 intrinsic matrix
        dist_coeffs: distortion coefficients [k1, k2, p1, p2, k3]

    Returns:
        2D undistorted point (2,)
    """
    # TODO: Implement undistortion
    # 1. Convert to normalized coordinates using K inverse
    # 2. Apply iterative undistortion (remove radial + tangential)
    # 3. Convert back to pixel coordinates
    raise NotImplementedError("Implement undistort_point")


def calibrate_camera(object_points, image_points):
    """
    Estimate camera intrinsic matrix from 3D-2D correspondences using DLT.

    Args:
        object_points: Nx3 array of 3D points
        image_points: Nx2 array of corresponding 2D points

    Returns:
        K: 3x3 estimated intrinsic matrix
        R: 3x3 rotation matrix
        t: 3x1 translation vector
    """
    # TODO: Implement DLT calibration
    # 1. Build the DLT matrix A from correspondences
    # 2. Solve using SVD
    # 3. Extract P = K[R|t] from the solution
    # 4. Decompose P into K, R, t using RQ decomposition
    raise NotImplementedError("Implement calibrate_camera")


def reprojection_error(K, R, t, points_3d, points_2d):
    """
    Compute mean reprojection error.

    Args:
        K: 3x3 intrinsic matrix
        R: 3x3 rotation matrix
        t: 3x1 translation vector
        points_3d: Nx3 array
        points_2d: Nx2 array

    Returns:
        Mean reprojection error in pixels
    """
    # TODO: Project all 3D points and compute mean distance to observed 2D points
    raise NotImplementedError("Implement reprojection_error")


def main():
    """Main function: project 3D points, calibrate camera, evaluate."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, 'data')

    # Load data
    K = np.load(os.path.join(data_dir, 'camera_intrinsics.npy'))
    dist_coeffs = np.load(os.path.join(data_dir, 'distortion_coeffs.npy'))

    print("Week 10 - Task 1: Camera Projection and Calibration")
    print("=" * 55)
    print(f"Camera matrix K:\n{K}")
    print(f"Distortion coefficients: {dist_coeffs}")

    # TODO: Generate 3D test points, project them, add noise
    # TODO: Calibrate from the noisy observations
    # TODO: Compute reprojection error
    # TODO: Plot projected points and reprojection errors

    print("\nImplement the functions above and complete main().")


if __name__ == '__main__':
    main()
