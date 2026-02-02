#!/usr/bin/env python3
"""
Task 1: EKF Prediction Step with IMU

Implement the prediction (time-update) step of an Extended Kalman Filter
using IMU measurements (linear acceleration and angular velocity).

State vector: x = [px, py, theta, vx, vy, omega]^T
IMU data: [ax, ay, omega_z]
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys


def motion_model(state, imu_data, dt):
    """
    Nonlinear motion model: x_{k+1} = f(x_k, u_k)

    Propagate the state forward using IMU measurements and kinematic equations.

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        Current state [px, py, theta, vx, vy, omega].
    imu_data : np.ndarray, shape (3,)
        IMU reading [ax, ay, omega_z] in the body frame.
    dt : float
        Time step in seconds.

    Returns
    -------
    np.ndarray, shape (6,)
        Predicted state after applying the motion model.
    """
    raise NotImplementedError("TODO: Implement the nonlinear motion model f(x, u)")


def compute_motion_jacobian(state, dt):
    """
    Compute the Jacobian F of the motion model with respect to the state.

    F = df/dx evaluated at the current state. This is the 6x6 matrix of
    partial derivatives needed for covariance propagation.

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        Current state [px, py, theta, vx, vy, omega].
    dt : float
        Time step in seconds.

    Returns
    -------
    np.ndarray, shape (6, 6)
        Jacobian matrix F of the motion model.
    """
    raise NotImplementedError("TODO: Compute the 6x6 motion model Jacobian F")


def predict(state, P, imu_data, dt, Q):
    """
    EKF prediction step: propagate state and covariance forward in time.

    predicted_state = f(state, imu_data)
    predicted_P = F @ P @ F^T + Q

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        Current state estimate.
    P : np.ndarray, shape (6, 6)
        Current state covariance matrix.
    imu_data : np.ndarray, shape (3,)
        IMU reading [ax, ay, omega_z].
    dt : float
        Time step in seconds.
    Q : np.ndarray, shape (6, 6)
        Process noise covariance matrix.

    Returns
    -------
    tuple of (np.ndarray, np.ndarray)
        (predicted_state (6,), predicted_P (6,6))
    """
    raise NotImplementedError("TODO: Implement the full EKF prediction step")


if __name__ == "__main__":
    print("=" * 60)
    print("Task 1: EKF Prediction Step with IMU")
    print("=" * 60)

    data_dir = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")
    print(f"\nData directory: {os.path.abspath(data_dir)}")
    print("TODO: Load IMU data from the data directory")
    print("TODO: Initialize state x0 and covariance P0")
    print("TODO: Define process noise covariance Q")
    print("TODO: Run prediction-only loop over all IMU samples")
    print("TODO: Plot predicted trajectory vs ground truth")
    print("TODO: Observe drift accumulation without measurement updates")

    # ---- Starter scaffold (replace with your implementation) ----
    # imu_data = np.load(os.path.join(data_dir, "imu_data.npy"))
    # ground_truth = np.load(os.path.join(data_dir, "ground_truth.npy"))
    # timestamps = np.load(os.path.join(data_dir, "timestamps.npy"))
    #
    # x = np.zeros(6)
    # P = np.eye(6) * 0.1
    # Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01])
    #
    # trajectory = [x.copy()]
    # for k in range(1, len(timestamps)):
    #     dt = timestamps[k] - timestamps[k - 1]
    #     x, P = predict(x, P, imu_data[k], dt, Q)
    #     trajectory.append(x.copy())
    # trajectory = np.array(trajectory)
    #
    # plt.figure()
    # plt.plot(trajectory[:, 0], trajectory[:, 1], label="Predicted")
    # plt.plot(ground_truth[:, 0], ground_truth[:, 1], label="Ground Truth")
    # plt.legend()
    # plt.xlabel("x [m]")
    # plt.ylabel("y [m]")
    # plt.title("Prediction-Only Trajectory")
    # plt.axis("equal")
    # plt.grid(True)
    # plt.show()

    print("\nImplement the functions above and uncomment the scaffold to run.")
