#!/usr/bin/env python3
"""
Task 3: GPS Update Step

Implement the EKF measurement-update step using GPS position data.
GPS provides absolute position measurements [px, py] that correct
accumulated drift from dead-reckoning.

State vector: x = [px, py, theta, vx, vy, omega]^T
GPS measurement: z = [px, py]
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys


def gps_measurement_model(state):
    """
    GPS measurement model: z = h_gps(x)

    Maps the state to the expected GPS reading. GPS directly observes
    the position components of the state.

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        Current state [px, py, theta, vx, vy, omega].

    Returns
    -------
    np.ndarray, shape (2,)
        Expected GPS measurement [px, py].
    """
    raise NotImplementedError("TODO: Implement GPS measurement model h_gps(x)")


def compute_gps_jacobian(state):
    """
    Compute the Jacobian H_gps of the GPS measurement model.

    H_gps = dh_gps/dx evaluated at the current state.

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        Current state [px, py, theta, vx, vy, omega].

    Returns
    -------
    np.ndarray, shape (2, 6)
        Jacobian matrix H of the GPS measurement model.
    """
    raise NotImplementedError("TODO: Compute the 2x6 GPS Jacobian H_gps")


def gps_update(state, P, z_gps, R_gps):
    """
    EKF measurement-update step using GPS data.

    Compute the Kalman gain, innovation, and update the state and covariance:
        y = z - h(x)
        S = H @ P @ H^T + R
        K = P @ H^T @ S^{-1}
        x_new = x + K @ y
        P_new = (I - K @ H) @ P

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        Predicted state estimate.
    P : np.ndarray, shape (6, 6)
        Predicted state covariance.
    z_gps : np.ndarray, shape (2,)
        GPS measurement [px, py].
    R_gps : np.ndarray, shape (2, 2)
        GPS measurement noise covariance.

    Returns
    -------
    tuple of (np.ndarray, np.ndarray)
        (updated_state (6,), updated_P (6,6))
    """
    raise NotImplementedError("TODO: Implement GPS measurement update step")


if __name__ == "__main__":
    print("=" * 60)
    print("Task 3: GPS Update Step")
    print("=" * 60)

    data_dir = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")
    print(f"\nData directory: {os.path.abspath(data_dir)}")
    print("TODO: Load IMU and GPS data from the data directory")
    print("TODO: Initialize state, covariance, Q, and R_gps")
    print("TODO: Run predict + gps_update loop")
    print("TODO: Plot trajectory comparing prediction-only vs prediction+GPS")

    # ---- Starter scaffold (replace with your implementation) ----
    # imu_data = np.load(os.path.join(data_dir, "imu_data.npy"))
    # gps_data = np.load(os.path.join(data_dir, "gps_data.npy"))
    # ground_truth = np.load(os.path.join(data_dir, "ground_truth.npy"))
    # timestamps = np.load(os.path.join(data_dir, "timestamps.npy"))
    #
    # x = np.zeros(6)
    # P = np.eye(6) * 0.1
    # Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01])
    # R_gps = np.diag([1.0, 1.0])
    #
    # from task1_prediction import predict
    #
    # trajectory = [x.copy()]
    # for k in range(1, len(timestamps)):
    #     dt = timestamps[k] - timestamps[k - 1]
    #     x, P = predict(x, P, imu_data[k], dt, Q)
    #     if gps_data[k] is not None:  # GPS may not be available every step
    #         x, P = gps_update(x, P, gps_data[k], R_gps)
    #     trajectory.append(x.copy())
    # trajectory = np.array(trajectory)
    #
    # plt.figure()
    # plt.plot(trajectory[:, 0], trajectory[:, 1], label="Predict + GPS")
    # plt.plot(ground_truth[:, 0], ground_truth[:, 1], label="Ground Truth")
    # plt.legend()
    # plt.xlabel("x [m]")
    # plt.ylabel("y [m]")
    # plt.title("EKF with GPS Updates")
    # plt.axis("equal")
    # plt.grid(True)
    # plt.show()

    print("\nImplement the functions above and uncomment the scaffold to run.")
