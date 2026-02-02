#!/usr/bin/env python3
"""
Week 4 - Task 4: Scan Matching Localization

Combine ICP scan matching with odometry for improved localization.

Functions to implement:
- predict_pose(pose, odometry)
- correct_pose_icp(predicted_pose, scan, reference_scan)
- scan_matching_pipeline(scans, odometry)
- evaluate_trajectory(estimated, ground_truth)
- main()
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def predict_pose(pose, odometry):
    """
    Predict next pose using odometry (dead reckoning step).

    Args:
        pose: Current pose [x, y, theta]
        odometry: Relative motion [dx, dy, dtheta]

    Returns:
        Predicted pose [x, y, theta]
    """
    # TODO: Apply odometry to current pose
    raise NotImplementedError("Implement predict_pose")


def correct_pose_icp(predicted_pose, current_scan, reference_scan, angles):
    """
    Correct the predicted pose using ICP between current and reference scans.

    Args:
        predicted_pose: Predicted pose from odometry
        current_scan: Current LiDAR ranges
        reference_scan: Previous LiDAR ranges
        angles: Beam angles

    Returns:
        Corrected pose [x, y, theta]
    """
    # TODO: Convert scans to point clouds
    # TODO: Run ICP
    # TODO: Apply correction to predicted pose
    raise NotImplementedError("Implement correct_pose_icp")


def scan_matching_pipeline(scans, odometry, initial_pose):
    """
    Full scan matching localization pipeline.

    Args:
        scans: Nx360 LiDAR scans
        odometry: Nx3 relative odometry
        initial_pose: Starting pose

    Returns:
        Nx3 estimated poses
    """
    # TODO: For each timestep:
    #   1. Predict pose from odometry
    #   2. Correct pose with ICP
    #   3. Store corrected pose
    raise NotImplementedError("Implement scan_matching_pipeline")


def evaluate_trajectory(estimated, ground_truth):
    """
    Compute Absolute Trajectory Error (ATE).

    Args:
        estimated: Nx3 estimated poses
        ground_truth: Nx3 ground truth poses

    Returns:
        ate_rmse: Root mean square ATE
        errors: Per-pose position errors
    """
    # TODO: Compute position error at each timestep
    # TODO: Return RMSE and per-pose errors
    raise NotImplementedError("Implement evaluate_trajectory")


def main():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    out_dir = os.path.dirname(os.path.abspath(__file__))

    poses = np.load(os.path.join(data_dir, "ground_truth_poses.npy"))
    scans = np.load(os.path.join(data_dir, "lidar_scans.npy"))
    odometry = np.load(os.path.join(data_dir, "odometry.npy"))

    print("=== Task 4: Scan Matching Localization ===")

    # TODO: Run scan matching pipeline
    # TODO: Also compute pure odometry trajectory for comparison
    # TODO: Evaluate both against ground truth
    # TODO: Plot trajectories and error curves
    # TODO: Save plots

    print("Task 4 not yet implemented. Complete the functions above!")


if __name__ == "__main__":
    main()
