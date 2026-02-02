#!/usr/bin/env python3
"""Task 3: EKF with GPS Update.

GPS measures: z = [px, py].
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def motion_model(x, u, dt):
    px, py, theta, vx, vy, omega = x
    ax, ay, alpha = u
    return np.array([
        px + vx * np.cos(theta) * dt,
        py + vx * np.sin(theta) * dt,
        normalize_angle(theta + omega * dt),
        vx + ax * dt,
        vy + ay * dt,
        omega + alpha * dt,
    ])


def compute_motion_jacobian(x, u, dt):
    px, py, theta, vx, vy, omega = x
    F = np.eye(6)
    F[0, 2] = -vx * np.sin(theta) * dt
    F[0, 3] = np.cos(theta) * dt
    F[1, 2] = vx * np.cos(theta) * dt
    F[1, 3] = np.sin(theta) * dt
    F[2, 5] = dt
    return F


def predict(x, P, u, dt, Q):
    F = compute_motion_jacobian(x, u, dt)
    x_pred = motion_model(x, u, dt)
    P_pred = F @ P @ F.T + Q
    return x_pred, P_pred


def gps_measurement_model(x):
    """h(x) = [px, py]."""
    return np.array([x[0], x[1]])


def compute_gps_jacobian(x):
    """Jacobian of GPS measurement model (2x6)."""
    H = np.zeros((2, 6))
    H[0, 0] = 1.0
    H[1, 1] = 1.0
    return H


def gps_update(x, P, z, R):
    """Standard EKF update with GPS measurement."""
    z_pred = gps_measurement_model(x)
    H = compute_gps_jacobian(x)
    innovation = z - z_pred
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x_upd = x + K @ innovation
    x_upd[2] = normalize_angle(x_upd[2])
    P_upd = (np.eye(6) - K @ H) @ P
    return x_upd, P_upd


def main():
    ground_truth = np.load(os.path.join(DATA_DIR, "ground_truth.npy"))
    imu_data = np.load(os.path.join(DATA_DIR, "imu_data.npy"))
    gps_data = np.load(os.path.join(DATA_DIR, "gps_data.npy"))

    x0 = ground_truth[0, 1:].copy()
    P0 = np.diag([1.0, 1.0, 0.1, 0.5, 0.5, 0.1])
    Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01])
    R_gps = np.diag([2.0 ** 2, 2.0 ** 2])

    x = x0.copy()
    P = P0.copy()
    traj = [x.copy()]
    times = [imu_data[0, 0]]

    gps_idx = 0
    for i in range(1, len(imu_data)):
        dt = imu_data[i, 0] - imu_data[i - 1, 0]
        if dt <= 0:
            continue
        x, P = predict(x, P, imu_data[i, 1:], dt, Q)

        t_now = imu_data[i, 0]
        while gps_idx < len(gps_data) and gps_data[gps_idx, 0] <= t_now:
            z_gps = gps_data[gps_idx, 1:]
            x, P = gps_update(x, P, z_gps, R_gps)
            gps_idx += 1

        traj.append(x.copy())
        times.append(t_now)

    traj = np.array(traj)
    times = np.array(times)

    gt_t = ground_truth[:, 0]
    gt_px = np.interp(times, gt_t, ground_truth[:, 1])
    gt_py = np.interp(times, gt_t, ground_truth[:, 2])
    err = np.sqrt((traj[:, 0] - gt_px) ** 2 + (traj[:, 1] - gt_py) ** 2)

    print("=== Task 3: EKF with GPS Update ===")
    print(f"Mean position error: {np.mean(err):.4f} m")
    print(f"Max position error:  {np.max(err):.4f} m")
    print(f"Final position error: {err[-1]:.4f} m")
    print(f"GPS measurements used: {gps_idx}")

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Task 3: EKF with GPS Update")

    axes[0].plot(ground_truth[:, 1], ground_truth[:, 2], "g-", label="Ground Truth", linewidth=2)
    axes[0].plot(traj[:, 0], traj[:, 1], "b-", label="IMU+GPS", linewidth=1)
    axes[0].scatter(gps_data[:, 1], gps_data[:, 2], c="m", s=15, zorder=5, label="GPS Meas.")
    axes[0].set_xlabel("X [m]")
    axes[0].set_ylabel("Y [m]")
    axes[0].legend()
    axes[0].set_aspect("equal")
    axes[0].grid(True)
    axes[0].set_title("Trajectory")

    axes[1].plot(times, err, "b-")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Position Error [m]")
    axes[1].grid(True)
    axes[1].set_title("Position Error Over Time")

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "task3_gps_update.png")
    plt.savefig(out_path, dpi=150)
    print(f"Plot saved to {out_path}")
    plt.close()


if __name__ == "__main__":
    main()
