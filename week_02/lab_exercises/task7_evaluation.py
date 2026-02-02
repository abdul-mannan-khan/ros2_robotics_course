#!/usr/bin/env python3
"""
Task 7: EKF Evaluation and Sensor Dropout Analysis

Evaluate the EKF quantitatively and investigate robustness to GPS dropout.
Compare filter performance with and without GPS over a time window.

State vector: x = [px, py, theta, vx, vy, omega]^T
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys


def evaluate_ekf(results, ground_truth):
    """
    Compute quantitative evaluation metrics for the EKF output.

    Parameters
    ----------
    results : dict
        Output of run_full_ekf() containing 'trajectory' and 'covariances'.
    ground_truth : np.ndarray, shape (N, 6) or (N, 3)
        Ground truth states (at minimum [px, py, theta]).

    Returns
    -------
    dict
        Evaluation metrics:
        - 'pos_rmse': float, position RMSE in meters
        - 'heading_rmse': float, heading RMSE in radians
        - 'max_pos_error': float, maximum position error in meters
        - 'avg_trace_P': float, average trace of position covariance
        - 'nees': np.ndarray, normalized estimation error squared at each step
    """
    raise NotImplementedError(
        "TODO: Compute RMSE, max error, average covariance trace, and NEES"
    )


def test_gps_dropout(
    imu_data, encoder_data, gps_data, dropout_start, dropout_end,
    Q, R_enc, R_gps
):
    """
    Run the EKF with GPS measurements removed during [dropout_start, dropout_end].

    Parameters
    ----------
    imu_data : np.ndarray, shape (N, 3)
        IMU measurements.
    encoder_data : np.ndarray, shape (N, 2)
        Encoder measurements.
    gps_data : np.ndarray, shape (N, 2) or list
        GPS measurements.
    dropout_start : float
        Start time of GPS dropout in seconds.
    dropout_end : float
        End time of GPS dropout in seconds.
    Q : np.ndarray, shape (6, 6)
        Process noise covariance.
    R_enc : np.ndarray, shape (2, 2)
        Encoder measurement noise covariance.
    R_gps : np.ndarray, shape (2, 2)
        GPS measurement noise covariance.

    Returns
    -------
    dict
        Dictionary with keys:
        - 'results_full': EKF results with all sensors
        - 'results_dropout': EKF results with GPS dropout applied
    """
    raise NotImplementedError(
        "TODO: Run EKF twice (with/without GPS dropout) and return both results"
    )


def plot_comparison(results_full, results_dropout, ground_truth):
    """
    Plot side-by-side comparison of full-sensor vs GPS-dropout trajectories.

    Parameters
    ----------
    results_full : dict
        EKF results with all sensors available.
    results_dropout : dict
        EKF results with GPS dropout.
    ground_truth : np.ndarray
        Ground truth trajectory.

    Returns
    -------
    matplotlib.figure.Figure
        Figure with comparison plots (trajectory, error over time, covariance).
    """
    raise NotImplementedError(
        "TODO: Create multi-panel comparison figure"
    )


if __name__ == "__main__":
    print("=" * 60)
    print("Task 7: EKF Evaluation and Sensor Dropout")
    print("=" * 60)

    data_dir = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")
    print(f"\nData directory: {os.path.abspath(data_dir)}")
    print("TODO: Load all sensor data and ground truth")
    print("TODO: Run full EKF and compute evaluation metrics")
    print("TODO: Test GPS dropout from t=20s to t=40s")
    print("TODO: Plot comparison of full vs dropout results")
    print("TODO: Discuss how uncertainty grows during GPS dropout")

    # ---- Starter scaffold (replace with your implementation) ----
    # from task6_full_ekf import run_full_ekf
    #
    # imu_data = np.load(os.path.join(data_dir, "imu_data.npy"))
    # encoder_data = np.load(os.path.join(data_dir, "encoder_data.npy"))
    # gps_data = np.load(os.path.join(data_dir, "gps_data.npy"))
    # ground_truth = np.load(os.path.join(data_dir, "ground_truth.npy"))
    #
    # Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01])
    # R_enc = np.diag([0.05, 0.02])
    # R_gps = np.diag([1.0, 1.0])
    #
    # # Full evaluation
    # x0 = np.zeros(6)
    # P0 = np.eye(6) * 0.1
    # results_full = run_full_ekf(imu_data, encoder_data, gps_data,
    #                             Q, R_enc, R_gps, x0, P0)
    # metrics = evaluate_ekf(results_full, ground_truth)
    # print(f"\nPosition RMSE: {metrics['pos_rmse']:.4f} m")
    # print(f"Heading RMSE:  {metrics['heading_rmse']:.4f} rad")
    # print(f"Max pos error: {metrics['max_pos_error']:.4f} m")
    #
    # # GPS dropout test
    # dropout_results = test_gps_dropout(
    #     imu_data, encoder_data, gps_data,
    #     dropout_start=20.0, dropout_end=40.0,
    #     Q=Q, R_enc=R_enc, R_gps=R_gps
    # )
    #
    # fig = plot_comparison(
    #     dropout_results['results_full'],
    #     dropout_results['results_dropout'],
    #     ground_truth
    # )
    # plt.show()

    print("\nImplement the functions above and uncomment the scaffold to run.")
