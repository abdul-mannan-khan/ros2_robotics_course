#!/usr/bin/env python3
"""Task 2: EKF with Encoder Update.

Encoder measures: z = [v, omega] where v = sqrt(vx^2 + vy^2).
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


def encoder_measurement_model(x):
    """h(x) = [sqrt(vx^2 + vy^2), omega]."""
    vx, vy, omega = x[3], x[4], x[5]
    v = np.sqrt(vx ** 2 + vy ** 2)
    return np.array([v, omega])


def compute_encoder_jacobian(x):
    """Jacobian of encoder measurement model (2x6)."""
    vx, vy = x[3], x[4]
    v = np.sqrt(vx ** 2 + vy ** 2)
    H = np.zeros((2, 6))
    if v > 1e-6:
        H[0, 3] = vx / v
        H[0, 4] = vy / v
    else:
        H[0, 3] = 1.0
        H[0, 4] = 0.0
    H[1, 5] = 1.0
    return H


def encoder_update(x, P, z, R):
    """Standard EKF update with encoder measurement."""
    z_pred = encoder_measurement_model(x)
    H = compute_encoder_jacobian(x)
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
    encoder_data = np.load(os.path.join(DATA_DIR, "encoder_data.npy"))

    x0 = ground_truth[0, 1:].copy()
    P0 = np.diag([1.0, 1.0, 0.1, 0.5, 0.5, 0.1])
    Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01])
    R_enc = np.diag([0.05 ** 2, 0.02 ** 2])

    # --- Prediction only ---
    x_po = x0.copy()
    P_po = P0.copy()
    traj_po = [x_po.copy()]
    times_po = [imu_data[0, 0]]
    for i in range(1, len(imu_data)):
        dt = imu_data[i, 0] - imu_data[i - 1, 0]
        if dt <= 0:
            continue
        x_po, P_po = predict(x_po, P_po, imu_data[i, 1:], dt, Q)
        traj_po.append(x_po.copy())
        times_po.append(imu_data[i, 0])
    traj_po = np.array(traj_po)
    times_po = np.array(times_po)

    # --- Prediction + Encoder update ---
    x = x0.copy()
    P = P0.copy()
    traj = [x.copy()]
    times = [imu_data[0, 0]]

    enc_idx = 0
    for i in range(1, len(imu_data)):
        dt = imu_data[i, 0] - imu_data[i - 1, 0]
        if dt <= 0:
            continue
        x, P = predict(x, P, imu_data[i, 1:], dt, Q)

        # Apply encoder updates up to current time
        t_now = imu_data[i, 0]
        while enc_idx < len(encoder_data) and encoder_data[enc_idx, 0] <= t_now:
            z_enc = encoder_data[enc_idx, 1:]
            x, P = encoder_update(x, P, z_enc, R_enc)
            enc_idx += 1

        traj.append(x.copy())
        times.append(t_now)

    traj = np.array(traj)
    times = np.array(times)

    # Compute errors
    gt_t = ground_truth[:, 0]
    gt_px = np.interp(times, gt_t, ground_truth[:, 1])
    gt_py = np.interp(times, gt_t, ground_truth[:, 2])

    err_fused = np.sqrt((traj[:, 0] - gt_px) ** 2 + (traj[:, 1] - gt_py) ** 2)

    gt_px_po = np.interp(times_po, gt_t, ground_truth[:, 1])
    gt_py_po = np.interp(times_po, gt_t, ground_truth[:, 2])
    err_po = np.sqrt((traj_po[:, 0] - gt_px_po) ** 2 + (traj_po[:, 1] - gt_py_po) ** 2)

    print("=== Task 2: EKF with Encoder Update ===")
    print(f"Prediction-only  - Mean error: {np.mean(err_po):.4f} m, Max: {np.max(err_po):.4f} m")
    print(f"IMU+Encoder      - Mean error: {np.mean(err_fused):.4f} m, Max: {np.max(err_fused):.4f} m")
    print(f"Improvement: {(1 - np.mean(err_fused) / max(np.mean(err_po), 1e-9)) * 100:.1f}%")

    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Task 2: EKF with Encoder Update")

    axes[0].plot(ground_truth[:, 1], ground_truth[:, 2], "g-", label="Ground Truth", linewidth=2)
    axes[0].plot(traj_po[:, 0], traj_po[:, 1], "r--", label="Prediction Only", alpha=0.6)
    axes[0].plot(traj[:, 0], traj[:, 1], "b-", label="IMU+Encoder", linewidth=1)
    axes[0].set_xlabel("X [m]")
    axes[0].set_ylabel("Y [m]")
    axes[0].legend()
    axes[0].set_aspect("equal")
    axes[0].grid(True)
    axes[0].set_title("Trajectory")

    axes[1].plot(times_po, err_po, "r--", label="Prediction Only", alpha=0.6)
    axes[1].plot(times, err_fused, "b-", label="IMU+Encoder")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Position Error [m]")
    axes[1].legend()
    axes[1].grid(True)
    axes[1].set_title("Position Error")

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "task2_encoder_update.png")
    plt.savefig(out_path, dpi=150)
    print(f"Plot saved to {out_path}")
    plt.close()


if __name__ == "__main__":
    main()
