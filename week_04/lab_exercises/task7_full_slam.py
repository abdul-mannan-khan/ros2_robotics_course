#!/usr/bin/env python3
"""
Week 4 - Task 7: Full 2D SLAM Pipeline

Complete 2D SLAM system combining scan matching, loop closure, pose graph
optimization, and occupancy grid mapping.

Classes/Functions to implement:
- class SLAM2D: Full SLAM system
- run_slam(scans, odometry)
- evaluate_slam(estimated_poses, ground_truth, estimated_map, gt_map)
- main()
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class SLAM2D:
    """
    Full 2D SLAM system with:
    - Scan matching (ICP)
    - Loop closure detection
    - Pose graph optimization
    - Occupancy grid mapping
    """

    def __init__(self, grid_size=500, resolution=0.1, max_range=12.0):
        """
        Initialize SLAM system.

        Args:
            grid_size: Occupancy grid dimension
            resolution: Grid resolution in meters
            max_range: LiDAR max range
        """
        # TODO: Initialize occupancy grid
        # TODO: Initialize pose graph structures
        # TODO: Store parameters
        raise NotImplementedError("Implement __init__")

    def process_scan(self, scan, odometry):
        """
        Process a new scan with odometry input.

        Args:
            scan: 360 range values
            odometry: [dx, dy, dtheta] relative motion

        Returns:
            Current estimated pose
        """
        # TODO: Predict pose from odometry
        # TODO: Correct with ICP if we have a previous scan
        # TODO: Store scan and pose
        # TODO: Check for loop closures periodically
        raise NotImplementedError("Implement process_scan")

    def detect_loops(self):
        """
        Check for loop closures with recent scans.

        Returns:
            List of (i, j, relative_pose) loop closure detections
        """
        # TODO: Compare recent scan to past scans
        # TODO: Return detected loop closures
        raise NotImplementedError("Implement detect_loops")

    def optimize(self):
        """
        Run pose graph optimization.

        Returns:
            Optimized poses
        """
        # TODO: Build pose graph from odometry + loop closures
        # TODO: Optimize
        raise NotImplementedError("Implement optimize")

    def get_map(self):
        """
        Build and return the occupancy grid map.

        Returns:
            Probability occupancy grid
        """
        # TODO: Build map from current pose estimates and stored scans
        raise NotImplementedError("Implement get_map")


def run_slam(scans, odometry, initial_pose):
    """
    Run the complete SLAM pipeline.

    Args:
        scans: Nx360 LiDAR scans
        odometry: Nx3 relative odometry
        initial_pose: Starting pose

    Returns:
        slam: SLAM2D object with results
        poses: Nx3 estimated poses
        occupancy_map: Probability grid
    """
    # TODO: Create SLAM2D instance
    # TODO: Process each scan
    # TODO: Optimize at the end
    # TODO: Build final map
    raise NotImplementedError("Implement run_slam")


def evaluate_slam(estimated_poses, ground_truth, estimated_map, gt_map):
    """
    Comprehensive SLAM evaluation.

    Args:
        estimated_poses: Nx3 estimated poses
        ground_truth: Nx3 ground truth poses
        estimated_map: Probability occupancy grid
        gt_map: Ground truth occupancy grid

    Returns:
        metrics: dict with ATE, RPE, map_accuracy, etc.
    """
    # TODO: Compute ATE (Absolute Trajectory Error)
    # TODO: Compute RPE (Relative Pose Error)
    # TODO: Compute map accuracy metrics
    raise NotImplementedError("Implement evaluate_slam")


def main():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    out_dir = os.path.dirname(os.path.abspath(__file__))

    poses = np.load(os.path.join(data_dir, "ground_truth_poses.npy"))
    scans = np.load(os.path.join(data_dir, "lidar_scans.npy"))
    odometry = np.load(os.path.join(data_dir, "odometry.npy"))
    environment = np.load(os.path.join(data_dir, "environment.npy"))

    print("=== Task 7: Full 2D SLAM Pipeline ===")

    # TODO: Run full SLAM pipeline
    # TODO: Evaluate against ground truth
    # TODO: Create 4-panel figure:
    #   1. Trajectory comparison (GT, odometry, SLAM)
    #   2. Occupancy map from SLAM
    #   3. Error plots (ATE over time)
    #   4. Loop closure visualization
    # TODO: Print comprehensive metrics
    # TODO: Save plots

    print("Task 7 not yet implemented. Complete the functions above!")


if __name__ == "__main__":
    main()
