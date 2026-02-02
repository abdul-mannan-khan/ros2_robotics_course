#!/usr/bin/env python3
"""
Week 10 - Task 5: Stereo Depth Estimation
Implement block matching stereo, disparity-to-depth, and point cloud generation.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def compute_disparity(left, right, block_size=11, max_disparity=64):
    """
    Compute disparity map using block matching (SAD).

    Args:
        left: HxW left image
        right: HxW right image
        block_size: matching window size (odd)
        max_disparity: maximum disparity to search

    Returns:
        disparity: HxW disparity map
    """
    # TODO: For each pixel in left image, find best matching column in right
    # using Sum of Absolute Differences (SAD)
    raise NotImplementedError("Implement compute_disparity")


def disparity_to_depth(disparity, focal_length, baseline):
    """
    Convert disparity map to depth map.

    Args:
        disparity: HxW disparity map
        focal_length: camera focal length in pixels
        baseline: stereo baseline in meters

    Returns:
        depth: HxW depth map in meters
    """
    # TODO: depth = focal_length * baseline / disparity
    raise NotImplementedError("Implement disparity_to_depth")


def depth_to_pointcloud(depth, K):
    """
    Back-project depth map to 3D point cloud.

    Args:
        depth: HxW depth map
        K: 3x3 camera intrinsic matrix

    Returns:
        points: Nx3 array of 3D points
    """
    # TODO: For each pixel, compute 3D point using K inverse and depth
    raise NotImplementedError("Implement depth_to_pointcloud")


def filter_depth(depth, min_depth=0.5, max_depth=100.0):
    """
    Filter invalid depth values.

    Args:
        depth: HxW depth map
        min_depth, max_depth: valid depth range

    Returns:
        filtered_depth: HxW with invalid values set to 0
    """
    # TODO: Set out-of-range and inf/nan values to 0
    raise NotImplementedError("Implement filter_depth")


def main():
    """Main function: compute disparity, depth, point cloud, visualize."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, 'data')

    print("Week 10 - Task 5: Stereo Depth Estimation")
    print("=" * 44)

    left = np.load(os.path.join(data_dir, 'stereo_left.npy'))
    right = np.load(os.path.join(data_dir, 'stereo_right.npy'))
    stereo_params = np.load(os.path.join(data_dir, 'stereo_params.npy'))
    K = np.load(os.path.join(data_dir, 'camera_intrinsics.npy'))

    baseline, focal_length = stereo_params
    print(f"Stereo baseline: {baseline:.3f} m, focal length: {focal_length:.1f} px")
    print(f"Image shape: {left.shape}")

    # TODO: Compute disparity, depth, point cloud
    # TODO: Filter depth
    # TODO: Visualize disparity map, depth map, point cloud

    print("\nImplement the functions above and complete main().")


if __name__ == '__main__':
    main()
