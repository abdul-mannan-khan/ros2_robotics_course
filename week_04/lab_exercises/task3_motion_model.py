#!/usr/bin/env python3
"""
Week 4 - Task 3: Probabilistic Odometry Motion Model

Implement sample-based odometry motion model and visualize uncertainty growth.

Functions to implement:
- odometry_motion_model(x, u, noise_params)
- propagate_particles(particles, u, noise_params)
- compute_pose_from_odometry(odometry_sequence)
- visualize_uncertainty(particles, true_pose)
- main()
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def odometry_motion_model(x, u, noise_params):
    """
    Sample-based odometry motion model.
    Given current pose x and control u, return a noisy new pose.

    Args:
        x: Current pose [x, y, theta]
        u: Odometry reading [dx, dy, dtheta]
        noise_params: dict with 'alpha1' to 'alpha4' noise parameters

    Returns:
        New pose [x', y', theta'] with sampled noise
    """
    # TODO: Add noise to the odometry proportional to the motion magnitude
    # TODO: Apply noisy motion to current pose
    # TODO: Return new pose
    raise NotImplementedError("Implement odometry_motion_model")


def propagate_particles(particles, u, noise_params):
    """
    Propagate a set of particles through the motion model.

    Args:
        particles: Nx3 array of particle poses
        u: Odometry reading [dx, dy, dtheta]
        noise_params: Noise parameters

    Returns:
        Updated Nx3 particle array
    """
    # TODO: Apply odometry_motion_model to each particle
    raise NotImplementedError("Implement propagate_particles")


def compute_pose_from_odometry(initial_pose, odometry_sequence):
    """
    Dead reckoning: integrate odometry to get trajectory.

    Args:
        initial_pose: Starting pose [x, y, theta]
        odometry_sequence: Nx3 relative odometry readings

    Returns:
        Nx3 array of poses from dead reckoning
    """
    # TODO: Integrate odometry sequentially
    raise NotImplementedError("Implement compute_pose_from_odometry")


def visualize_uncertainty(particles_history, true_poses, filename="task3_uncertainty.png"):
    """
    Visualize particle spread at several timesteps.

    Args:
        particles_history: List of Nx3 particle arrays at selected timesteps
        true_poses: Corresponding ground truth poses
        filename: Output filename
    """
    # TODO: Plot particles at each timestep, color-coded
    # TODO: Show ground truth pose as a marker
    # TODO: Save figure
    raise NotImplementedError("Implement visualize_uncertainty")


def main():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    out_dir = os.path.dirname(os.path.abspath(__file__))

    poses = np.load(os.path.join(data_dir, "ground_truth_poses.npy"))
    odometry = np.load(os.path.join(data_dir, "odometry.npy"))

    print("=== Task 3: Probabilistic Motion Model ===")

    # TODO: Compute dead-reckoning trajectory and compare to ground truth
    # TODO: Initialize particles at the first pose
    # TODO: Propagate particles through odometry, record snapshots
    # TODO: Visualize uncertainty growth
    # TODO: Save plots

    print("Task 3 not yet implemented. Complete the functions above!")


if __name__ == "__main__":
    main()
