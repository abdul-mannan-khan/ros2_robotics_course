#!/usr/bin/env python3
"""Task 6: Complete Integrated EKF with All Sensors.

Time-sorted event loop processing IMU, encoder, and GPS measurements chronologically.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import os

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


# ---- Models ----

def motion_model(x, u, dt):
    px, py, theta, vx, vy, omega = x
    ax, ay, alpha = u
    return np.array([
        px + vx * np.cos(theta) * dt,
        py + vx * np.sin(theta) * dt,
        normalize_angle(theta + omega * dt),
        vx + ax * dt, vy + ay * dt, omega + alpha * dt,
    ])


def compute_motion_jacobian(x, u, dt):
    theta, vx = x[2], x[3]
    F = np.eye(6)
    F[0, 2] = -vx * np.sin(theta) * dt
    F[0, 3] = np.cos(theta) * dt
    F[1, 2] = vx * np.cos(theta) * dt
    F[1, 3] = np.sin(theta) * dt
    F[2, 5] = dt
    return F


def predict(x, P, u, dt, Q):
    F = compute_motion_jacobian(x, u, dt)
    return motion_model(x, u, dt), F @ P @ F.T + Q


def ekf_update(x, P, z, h_func, H_func, R):
    z_pred = h_func(x)
    H = H_func(x)
    innovation = z - z_pred
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x_upd = x + K @ innovation
    x_upd[2] = normalize_angle(x_upd[2])
    P_upd = (np.eye(6) - K @ H) @ P
    return x_upd, P_upd


def encoder_measurement_model(x):
    return np.array([np.sqrt(x[3]**2 + x[4]**2), x[5]])


def compute_encoder_jacobian(x):
    vx, vy = x[3], x[4]
    v = np.sqrt(vx**2 + vy**2)
    H = np.zeros((2, 6))
    H[0, 3] = vx / v if v > 1e-6 else 1.0
    H[0, 4] = vy / v if v > 1e-6 else 0.0
    H[1, 5] = 1.0
    return H


def gps_measurement_model(x):
    return np.array([x[0], x[1]])


def compute_gps_jacobian(x):
    H = np.zeros((2, 6))
    H[0, 0] = 1.0
    H[1, 1] = 1.0
    return H


# ---- Full EKF ----

def run_full_ekf(imu_data, encoder_data, gps_data, x0,
                  Q=None, R_enc=None, R_gps=None, P0=None,
                  gps_filter_func=None):
    """Run full EKF processing all sensors chronologically.

    gps_filter_func: optional callable(time) -> bool; if returns False, skip GPS update.
    """
    if Q is None:
        Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01])
    if R_enc is None:
        R_enc = np.diag([0.05**2, 0.02**2])
    if R_gps is None:
        R_gps = np.diag([2.0**2, 2.0**2])
    if P0 is None:
        P0 = np.diag([1.0, 1.0, 0.1, 0.5, 0.5, 0.1])

    # Build sorted event list: (time, type, data)
    events = []
    for row in imu_data:
        events.append((row[0], "imu", row[1:]))
    for row in encoder_data:
        events.append((row[0], "enc", row[1:]))
    for row in gps_data:
        events.append((row[0], "gps", row[1:]))

    # Sort by time, with IMU first at same timestamp (predict before update)
    type_order = {"imu": 0, "enc": 1, "gps": 2}
    events.sort(key=lambda e: (e[0], type_order[e[1]]))

    x = x0.copy()
    P = P0.copy()
    prev_imu_time = None
    last_imu_u = None

    trajectory = []
    covariances = []
    times = []

    for t, etype, data in events:
        if etype == "imu":
            if prev_imu_time is not None:
                dt = t - prev_imu_time
                if dt > 0:
                    x, P = predict(x, P, data, dt, Q)
            prev_imu_time = t
            last_imu_u = data
            trajectory.append(x.copy())
            covariances.append(P.copy())
            times.append(t)

        elif etype == "enc":
            x, P = ekf_update(x, P, data, encoder_measurement_model,
                              compute_encoder_jacobian, R_enc)

        elif etype == "gps":
            if gps_filter_func is None or gps_filter_func(t):
                x, P = ekf_update(x, P, data, gps_measurement_model,
                                  compute_gps_jacobian, R_gps)

    return np.array(times), np.array(trajectory), covariances


def plot_uncertainty_ellipse(ax, mean, cov_2x2, n_std=2.0, **kwargs):
    """Draw uncertainty ellipse from 2x2 covariance matrix."""
    eigvals, eigvecs = np.linalg.eigh(cov_2x2)
    # Ensure positive
    eigvals = np.maximum(eigvals, 0)
    angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
    width = 2 * n_std * np.sqrt(eigvals[0])
    height = 2 * n_std * np.sqrt(eigvals[1])
    ellipse = Ellipse(xy=mean, width=width, height=height, angle=angle, **kwargs)
    ax.add_patch(ellipse)
    return ellipse


def plot_trajectory(times, traj, covariances, ground_truth, gps_data, save_path):
    """Comprehensive 2x2 visualization."""
    gt_t = ground_truth[:, 0]

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle("Task 6: Full EKF - Comprehensive Visualization", fontsize=14)

    # (0,0) Trajectory with uncertainty ellipses
    ax = axes[0, 0]
    ax.plot(ground_truth[:, 1], ground_truth[:, 2], "g-", label="Ground Truth", linewidth=2)
    ax.plot(traj[:, 0], traj[:, 1], "b-", label="EKF Estimate", linewidth=1)
    ax.scatter(gps_data[:, 1], gps_data[:, 2], c="m", s=10, alpha=0.5, label="GPS", zorder=3)

    # Plot ellipses at intervals
    step = max(1, len(times) // 20)
    for i in range(0, len(times), step):
        P_pos = covariances[i][:2, :2]
        plot_uncertainty_ellipse(ax, traj[i, :2], P_pos, n_std=2.0,
                                 fill=False, edgecolor="blue", alpha=0.4, linewidth=0.8)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Trajectory with 2-sigma Uncertainty")
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True)

    # (0,1) X and Y over time
    ax = axes[0, 1]
    ax.plot(gt_t, ground_truth[:, 1], "g-", label="GT X", linewidth=1.5)
    ax.plot(gt_t, ground_truth[:, 2], "g--", label="GT Y", linewidth=1.5)
    ax.plot(times, traj[:, 0], "b-", label="Est X", linewidth=0.8)
    ax.plot(times, traj[:, 1], "b--", label="Est Y", linewidth=0.8)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position [m]")
    ax.set_title("Position Over Time")
    ax.legend()
    ax.grid(True)

    # (1,0) Heading over time
    ax = axes[1, 0]
    gt_theta = np.interp(times, gt_t, ground_truth[:, 3])
    ax.plot(times, np.degrees(gt_theta), "g-", label="GT", linewidth=1.5)
    ax.plot(times, np.degrees(traj[:, 2]), "b-", label="EKF", linewidth=0.8)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Heading [deg]")
    ax.set_title("Heading Over Time")
    ax.legend()
    ax.grid(True)

    # (1,1) Speed over time
    ax = axes[1, 1]
    gt_speed = np.interp(times, gt_t, np.sqrt(ground_truth[:, 4]**2 + ground_truth[:, 5]**2))
    est_speed = np.sqrt(traj[:, 3]**2 + traj[:, 4]**2)
    ax.plot(times, gt_speed, "g-", label="GT", linewidth=1.5)
    ax.plot(times, est_speed, "b-", label="EKF", linewidth=0.8)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [m/s]")
    ax.set_title("Speed Over Time")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"Plot saved to {save_path}")
    plt.close()


def main():
    ground_truth = np.load(os.path.join(DATA_DIR, "ground_truth.npy"))
    imu_data = np.load(os.path.join(DATA_DIR, "imu_data.npy"))
    encoder_data = np.load(os.path.join(DATA_DIR, "encoder_data.npy"))
    gps_data = np.load(os.path.join(DATA_DIR, "gps_data.npy"))

    x0 = ground_truth[0, 1:].copy()

    print("=== Task 6: Full Integrated EKF ===")
    print(f"IMU samples:     {len(imu_data)}")
    print(f"Encoder samples: {len(encoder_data)}")
    print(f"GPS samples:     {len(gps_data)}")

    times, traj, covs = run_full_ekf(imu_data, encoder_data, gps_data, x0)

    # Compute errors
    gt_t = ground_truth[:, 0]
    gt_px = np.interp(times, gt_t, ground_truth[:, 1])
    gt_py = np.interp(times, gt_t, ground_truth[:, 2])
    pos_err = np.sqrt((traj[:, 0] - gt_px)**2 + (traj[:, 1] - gt_py)**2)

    gt_theta = np.interp(times, gt_t, ground_truth[:, 3])
    heading_err = np.abs(normalize_angle(traj[:, 2] - gt_theta))

    print(f"\nResults:")
    print(f"  Position RMSE:  {np.sqrt(np.mean(pos_err**2)):.4f} m")
    print(f"  Heading RMSE:   {np.degrees(np.sqrt(np.mean(heading_err**2))):.4f} deg")
    print(f"  Max pos error:  {np.max(pos_err):.4f} m")
    print(f"  Final pos error: {pos_err[-1]:.4f} m")
    print(f"  Final P diag:   {np.diag(covs[-1])}")

    out_path = os.path.join(os.path.dirname(__file__), "task6_full_ekf.png")
    plot_trajectory(times, traj, covs, ground_truth, gps_data, out_path)


if __name__ == "__main__":
    main()
