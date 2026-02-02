#!/usr/bin/env python3
"""
Task 5: Process and Measurement Noise Covariance Tuning

Explore how the choice of Q (process noise) and R (measurement noise)
covariance matrices affects EKF performance. Use grid search to find
optimal scaling factors.

State vector: x = [px, py, theta, vx, vy, omega]^T
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys


def run_ekf_with_params(sensor_data, Q_scale, R_scales):
    """
    Run the full EKF pipeline with given noise covariance scaling factors.

    Parameters
    ----------
    sensor_data : dict
        Dictionary with keys 'imu', 'encoder', 'gps', 'timestamps',
        'ground_truth', each containing np.ndarray data.
    Q_scale : float
        Scalar multiplier for the base process noise covariance Q.
    R_scales : dict
        Dictionary with keys 'encoder' and 'gps', each a float multiplier
        for the corresponding base measurement noise covariance.

    Returns
    -------
    tuple of (np.ndarray, float)
        (trajectory as Nx6 array, total RMSE as float)
    """
    raise NotImplementedError(
        "TODO: Run EKF with scaled Q and R, return trajectory and RMSE"
    )


def compute_rmse(estimated, ground_truth):
    """
    Compute root-mean-square error for position and heading.

    Parameters
    ----------
    estimated : np.ndarray, shape (N, 6)
        Estimated state trajectory.
    ground_truth : np.ndarray, shape (N, 6) or (N, 3)
        Ground truth trajectory (at minimum [px, py, theta]).

    Returns
    -------
    tuple of (float, float)
        (position_rmse in meters, heading_rmse in radians)
    """
    raise NotImplementedError("TODO: Compute position and heading RMSE")


def grid_search_tuning(sensor_data, q_values, r_values):
    """
    Perform grid search over Q and R scaling factors to minimize RMSE.

    Parameters
    ----------
    sensor_data : dict
        Sensor data dictionary (see run_ekf_with_params).
    q_values : list of float
        List of Q scaling factors to try.
    r_values : list of float
        List of R scaling factors to try (applied to both encoder and GPS).

    Returns
    -------
    tuple of (dict, np.ndarray)
        (best_params dict with 'Q_scale' and 'R_scale',
         results as 2D array of RMSE values indexed by [q_idx, r_idx])
    """
    raise NotImplementedError(
        "TODO: Grid search over Q/R scales, return best params and RMSE surface"
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Task 5: Covariance Tuning via Grid Search")
    print("=" * 60)

    data_dir = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")
    print(f"\nData directory: {os.path.abspath(data_dir)}")
    print("TODO: Load all sensor data")
    print("TODO: Define grid of Q and R scaling factors")
    print("TODO: Run grid search")
    print("TODO: Plot RMSE surface as heatmap or 3D plot")
    print("TODO: Report best parameters and corresponding RMSE")

    # ---- Starter scaffold (replace with your implementation) ----
    # sensor_data = {
    #     'imu': np.load(os.path.join(data_dir, "imu_data.npy")),
    #     'encoder': np.load(os.path.join(data_dir, "encoder_data.npy")),
    #     'gps': np.load(os.path.join(data_dir, "gps_data.npy")),
    #     'timestamps': np.load(os.path.join(data_dir, "timestamps.npy")),
    #     'ground_truth': np.load(os.path.join(data_dir, "ground_truth.npy")),
    # }
    #
    # q_values = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0]
    # r_values = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0]
    #
    # best_params, results = grid_search_tuning(sensor_data, q_values, r_values)
    # print(f"\nBest Q scale: {best_params['Q_scale']}")
    # print(f"Best R scale: {best_params['R_scale']}")
    #
    # plt.figure()
    # plt.imshow(results, origin="lower", aspect="auto",
    #            extent=[r_values[0], r_values[-1], q_values[0], q_values[-1]])
    # plt.colorbar(label="RMSE [m]")
    # plt.xlabel("R scale")
    # plt.ylabel("Q scale")
    # plt.title("RMSE vs Q/R Scaling")
    # plt.show()

    print("\nImplement the functions above and uncomment the scaffold to run.")
