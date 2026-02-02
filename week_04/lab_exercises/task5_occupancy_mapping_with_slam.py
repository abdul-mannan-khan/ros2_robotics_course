#!/usr/bin/env python3
"""
Week 4 - Task 5: Occupancy Mapping with SLAM Poses

Build occupancy grid maps using estimated (not ground truth) poses and compare.

Functions to implement:
- simple_slam_pipeline(scans, odometry)
- build_map_from_estimates(grid, poses, scans, params)
- compare_maps(estimated_map, ground_truth_map)
- main()
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def simple_slam_pipeline(scans, odometry, initial_pose):
    """
    Odometry + ICP scan matching to estimate poses.

    Args:
        scans: Nx360 LiDAR scans
        odometry: Nx3 relative odometry
        initial_pose: Starting pose

    Returns:
        Nx3 estimated poses
    """
    # TODO: Reuse scan matching pipeline from Task 4
    raise NotImplementedError("Implement simple_slam_pipeline")


def build_map_from_estimates(poses, scans, grid_size=500, resolution=0.1):
    """
    Build an occupancy grid map from estimated poses and scans.

    Args:
        poses: Nx3 estimated poses
        scans: Nx360 LiDAR scans
        grid_size: Grid dimension
        resolution: Cell size in meters

    Returns:
        Probability occupancy grid
    """
    # TODO: Reuse occupancy grid mapping from Task 1
    raise NotImplementedError("Implement build_map_from_estimates")


def compare_maps(estimated_map, ground_truth_map, threshold=0.5):
    """
    Compare estimated map against ground truth.

    Args:
        estimated_map: Probability grid from estimated poses
        ground_truth_map: Ground truth occupancy grid
        threshold: Probability threshold to classify cells

    Returns:
        accuracy: Fraction of correctly classified cells
        precision: Precision for occupied cells
        recall: Recall for occupied cells
    """
    # TODO: Threshold both maps
    # TODO: Compute accuracy, precision, recall
    raise NotImplementedError("Implement compare_maps")


def main():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    out_dir = os.path.dirname(os.path.abspath(__file__))

    poses = np.load(os.path.join(data_dir, "ground_truth_poses.npy"))
    scans = np.load(os.path.join(data_dir, "lidar_scans.npy"))
    odometry = np.load(os.path.join(data_dir, "odometry.npy"))
    environment = np.load(os.path.join(data_dir, "environment.npy"))

    print("=== Task 5: Occupancy Mapping with SLAM ===")

    # TODO: Build map with ground truth poses
    # TODO: Estimate poses with simple SLAM pipeline
    # TODO: Build map with estimated poses
    # TODO: Compare both maps to ground truth
    # TODO: Visualize side-by-side: GT map, GT-pose map, estimated-pose map
    # TODO: Save plots

    print("Task 5 not yet implemented. Complete the functions above!")


if __name__ == "__main__":
    main()
