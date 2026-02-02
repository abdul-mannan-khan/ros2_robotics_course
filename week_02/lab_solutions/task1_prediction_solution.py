#!/usr/bin/env python3
"""Task 1: EKF Prediction Step - Full Implementation.

State vector: x = [px, py, theta, vx, vy, omega]^T
IMU input: u = [ax, ay, alpha]
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def motion_model(x, u, dt):
    """Predict next state using motion model.

    x: [px, py, theta, vx, vy, omega]
    u: [ax, ay, alpha]
    """
    px, py, theta, vx, vy, omega = x
    ax, ay, alpha = u

    px_new = px + vx * np.cos(theta) * dt
    py_new = py + vx * np.sin(theta) * dt
    theta_new = normalize_angle(theta + omega * dt)
    vx_new = vx + ax * dt
    vy_new = vy + ay * dt
    omega_new = omega + alpha * dt

    return np.array([px_new, py_new, theta_new, vx_new, vy_new, omega_new])


def compute_motion_jacobian(x, u, dt):
    """Compute 6x6 Jacobian of motion model w.r.t. state.

    F = df/dx evaluated at (x, u).
    """
    px, py, theta, vx, vy, omega = x

    F = np.eye(6)
    # d(px)/d(theta) = -vx*sin(theta)*dt
    F[0, 2] = -vx * np.sin(theta) * dt
    # d(px)/d(vx) = cos(theta)*dt
    F[0, 3] = np.cos(theta) * dt
    # d(py)/d(theta) = vx*cos(theta)*dt
    F[1, 2] = vx * np.cos(theta) * dt
    # d(py)/d(vx) = sin(theta)*dt
    F[1, 3] = np.sin(theta) * dt
    # d(theta)/d(omega) = dt
    F[2, 5] = dt

    return F


def predict(x, P, u, dt, Q):
    """EKF prediction step.

    Returns predicted state and covariance.
    """
    F = compute_motion_jacobian(x, u, dt)
    x_pred = motion_model(x, u, dt)
    P_pred = F @ P @ F.T + Q
    return x_pred, P_pred


def main():
    # Load data
    ground_truth = np.load(os.path.join(DATA_DIR, "ground_truth.npy"))
    imu_data = np.load(os.path.join(DATA_DIR, "imu_data.npy"))

    # Initial state from ground truth
    x = ground_truth[0, 1:].copy()
    P = np.diag([1.0, 1.0, 0.1, 0.5, 0.5, 0.1])
    Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01])

    # Run prediction loop
    trajectory = [x.copy()]
    times = [imu_data[0, 0]]

    for i in range(1, len(imu_data)):
        dt = imu_data[i, 0] - imu_data[i - 1, 0]
        if dt <= 0:
            continue
        u = imu_data[i, 1:]  # ax, ay, alpha
        x, P = predict(x, P, u, dt, Q)
        trajectory.append(x.copy())
        times.append(imu_data[i, 0])

    trajectory = np.array(trajectory)
    times = np.array(times)

    # Compute drift statistics
    gt_times = ground_truth[:, 0]
    gt_px = np.interp(times, gt_times, ground_truth[:, 1])
    gt_py = np.interp(times, gt_times, ground_truth[:, 2])

    pos_error = np.sqrt((trajectory[:, 0] - gt_px) ** 2 + (trajectory[:, 1] - gt_py) ** 2)
    print("=== Task 1: EKF Prediction Only ===")
    print(f"Number of IMU samples processed: {len(imu_data)}")
    print(f"Duration: {times[-1] - times[0]:.2f} s")
    print(f"Mean position error: {np.mean(pos_error):.4f} m")
    print(f"Max position error:  {np.max(pos_error):.4f} m")
    print(f"Final position error: {pos_error[-1]:.4f} m")
    print(f"Final covariance diagonal: {np.diag(P)}")

    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Task 1: EKF Prediction Only (IMU)")

    # Trajectory
    axes[0, 0].plot(ground_truth[:, 1], ground_truth[:, 2], "g-", label="Ground Truth", linewidth=2)
    axes[0, 0].plot(trajectory[:, 0], trajectory[:, 1], "r--", label="Predicted", linewidth=1)
    axes[0, 0].set_xlabel("X [m]")
    axes[0, 0].set_ylabel("Y [m]")
    axes[0, 0].set_title("Trajectory")
    axes[0, 0].legend()
    axes[0, 0].set_aspect("equal")
    axes[0, 0].grid(True)

    # Position error over time
    axes[0, 1].plot(times, pos_error, "r-")
    axes[0, 1].set_xlabel("Time [s]")
    axes[0, 1].set_ylabel("Position Error [m]")
    axes[0, 1].set_title("Position Error (Drift)")
    axes[0, 1].grid(True)

    # X position
    axes[1, 0].plot(gt_times, ground_truth[:, 1], "g-", label="GT")
    axes[1, 0].plot(times, trajectory[:, 0], "r--", label="Predicted")
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("X [m]")
    axes[1, 0].set_title("X Position")
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # Y position
    axes[1, 1].plot(gt_times, ground_truth[:, 2], "g-", label="GT")
    axes[1, 1].plot(times, trajectory[:, 1], "r--", label="Predicted")
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("Y [m]")
    axes[1, 1].set_title("Y Position")
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "task1_prediction.png")
    plt.savefig(out_path, dpi=150)
    print(f"Plot saved to {out_path}")
    plt.close()


if __name__ == "__main__":
    main()
