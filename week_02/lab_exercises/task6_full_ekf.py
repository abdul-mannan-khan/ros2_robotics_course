#!/usr/bin/env python3
"""
Task 6: Complete Extended Kalman Filter

Combine the prediction step (IMU), encoder update, and GPS update into a
single EKF pipeline. Visualize the trajectory with uncertainty ellipses.

State vector: x = [px, py, theta, vx, vy, omega]^T
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import os
import sys


def run_full_ekf(imu_data, encoder_data, gps_data, Q, R_enc, R_gps, x0, P0):
    """
    Run the complete EKF with IMU prediction, encoder updates, and GPS updates.

    Parameters
    ----------
    imu_data : np.ndarray, shape (N, 3)
        IMU measurements [ax, ay, omega_z] at each timestep.
    encoder_data : np.ndarray, shape (N, 2)
        Encoder measurements [v, omega] at each timestep.
    gps_data : np.ndarray, shape (N, 2) or list
        GPS measurements [px, py]; entries may be None when GPS is unavailable.
    Q : np.ndarray, shape (6, 6)
        Process noise covariance.
    R_enc : np.ndarray, shape (2, 2)
        Encoder measurement noise covariance.
    R_gps : np.ndarray, shape (2, 2)
        GPS measurement noise covariance.
    x0 : np.ndarray, shape (6,)
        Initial state estimate.
    P0 : np.ndarray, shape (6, 6)
        Initial state covariance.

    Returns
    -------
    dict
        Dictionary with keys:
        - 'trajectory': np.ndarray, shape (N, 6) - state estimates
        - 'timestamps': np.ndarray, shape (N,) - time values
        - 'covariances': list of np.ndarray - P matrix at each step
    """
    raise NotImplementedError("TODO: Implement the complete EKF pipeline")


def plot_trajectory(results, ground_truth):
    """
    Plot the EKF estimated trajectory against ground truth.

    Parameters
    ----------
    results : dict
        Output of run_full_ekf().
    ground_truth : np.ndarray, shape (N, 6) or (N, 3)
        Ground truth states.

    Returns
    -------
    matplotlib.figure.Figure
        The generated figure.
    """
    raise NotImplementedError("TODO: Create trajectory comparison plot")


def plot_uncertainty_ellipse(ax, mean, cov, color="blue", n_std=2.0):
    """
    Draw a 2D covariance ellipse on the given axes.

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        Axes on which to draw.
    mean : np.ndarray, shape (2,)
        Center of the ellipse [x, y].
    cov : np.ndarray, shape (2, 2)
        2x2 covariance matrix for the position states.
    color : str, optional
        Color of the ellipse.
    n_std : float, optional
        Number of standard deviations for the ellipse radius (default 2-sigma).
    """
    raise NotImplementedError("TODO: Compute and draw the uncertainty ellipse")


if __name__ == "__main__":
    print("=" * 60)
    print("Task 6: Complete Extended Kalman Filter")
    print("=" * 60)

    data_dir = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")
    print(f"\nData directory: {os.path.abspath(data_dir)}")
    print("TODO: Load all sensor data and ground truth")
    print("TODO: Set initial state, covariance, Q, R_enc, R_gps")
    print("TODO: Run the full EKF")
    print("TODO: Plot trajectory with uncertainty ellipses at regular intervals")

    # ---- Starter scaffold (replace with your implementation) ----
    # imu_data = np.load(os.path.join(data_dir, "imu_data.npy"))
    # encoder_data = np.load(os.path.join(data_dir, "encoder_data.npy"))
    # gps_data = np.load(os.path.join(data_dir, "gps_data.npy"))
    # ground_truth = np.load(os.path.join(data_dir, "ground_truth.npy"))
    #
    # x0 = np.zeros(6)
    # P0 = np.eye(6) * 0.1
    # Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01])
    # R_enc = np.diag([0.05, 0.02])
    # R_gps = np.diag([1.0, 1.0])
    #
    # results = run_full_ekf(imu_data, encoder_data, gps_data, Q, R_enc, R_gps, x0, P0)
    #
    # fig = plot_trajectory(results, ground_truth)
    #
    # # Add uncertainty ellipses every 50 steps
    # ax = fig.gca()
    # traj = results['trajectory']
    # covs = results['covariances']
    # for k in range(0, len(traj), 50):
    #     plot_uncertainty_ellipse(ax, traj[k, :2], covs[k][:2, :2], color="red")
    #
    # plt.show()

    print("\nImplement the functions above and uncomment the scaffold to run.")
