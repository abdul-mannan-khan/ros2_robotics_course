#!/usr/bin/env python3
"""
Task 2: Encoder Update Step

Implement the EKF measurement-update step using wheel encoder data.
Encoders provide velocity measurements [v, omega] that can be used to
correct the predicted state.

State vector: x = [px, py, theta, vx, vy, omega]^T
Encoder measurement: z = [v, omega]
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys


def encoder_measurement_model(state):
    """
    Encoder measurement model: z = h_enc(x)

    Maps the state to the expected encoder reading. The encoder measures
    forward velocity and yaw rate.

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        Current state [px, py, theta, vx, vy, omega].

    Returns
    -------
    np.ndarray, shape (2,)
        Expected encoder measurement [v, omega].
    """
    raise NotImplementedError("TODO: Implement encoder measurement model h_enc(x)")


def compute_encoder_jacobian(state):
    """
    Compute the Jacobian H_encoder of the encoder measurement model.

    H_enc = dh_enc/dx evaluated at the current state.

    Parameters
    ----------
    state : np.ndarray, shape (6,)
        Current state [px, py, theta, vx, vy, omega].

    Returns
    -------
    np.ndarray, shape (2, 6)
        Jacobian matrix H of the encoder measurement model.
    """
    raise NotImplementedError("TODO: Compute the 2x6 encoder Jacobian H_enc")


def encoder_update(state, P, z_encoder, R_encoder):
    """
    EKF measurement-update step using encoder data.

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
    z_encoder : np.ndarray, shape (2,)
        Encoder measurement [v, omega].
    R_encoder : np.ndarray, shape (2, 2)
        Encoder measurement noise covariance.

    Returns
    -------
    tuple of (np.ndarray, np.ndarray)
        (updated_state (6,), updated_P (6,6))
    """
    raise NotImplementedError("TODO: Implement encoder measurement update step")


if __name__ == "__main__":
    print("=" * 60)
    print("Task 2: Encoder Update Step")
    print("=" * 60)

    data_dir = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")
    print(f"\nData directory: {os.path.abspath(data_dir)}")
    print("TODO: Load IMU and encoder data from the data directory")
    print("TODO: Initialize state, covariance, Q, and R_encoder")
    print("TODO: Run predict + encoder_update loop")
    print("TODO: Plot trajectory comparing prediction-only vs prediction+encoder")

    # ---- Starter scaffold (replace with your implementation) ----
    # imu_data = np.load(os.path.join(data_dir, "imu_data.npy"))
    # encoder_data = np.load(os.path.join(data_dir, "encoder_data.npy"))
    # ground_truth = np.load(os.path.join(data_dir, "ground_truth.npy"))
    # timestamps = np.load(os.path.join(data_dir, "timestamps.npy"))
    #
    # x = np.zeros(6)
    # P = np.eye(6) * 0.1
    # Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01])
    # R_enc = np.diag([0.05, 0.02])
    #
    # from task1_prediction import predict
    #
    # trajectory = [x.copy()]
    # for k in range(1, len(timestamps)):
    #     dt = timestamps[k] - timestamps[k - 1]
    #     x, P = predict(x, P, imu_data[k], dt, Q)
    #     x, P = encoder_update(x, P, encoder_data[k], R_enc)
    #     trajectory.append(x.copy())
    # trajectory = np.array(trajectory)
    #
    # plt.figure()
    # plt.plot(trajectory[:, 0], trajectory[:, 1], label="Predict + Encoder")
    # plt.plot(ground_truth[:, 0], ground_truth[:, 1], label="Ground Truth")
    # plt.legend()
    # plt.xlabel("x [m]")
    # plt.ylabel("y [m]")
    # plt.title("EKF with Encoder Updates")
    # plt.axis("equal")
    # plt.grid(True)
    # plt.show()

    print("\nImplement the functions above and uncomment the scaffold to run.")
