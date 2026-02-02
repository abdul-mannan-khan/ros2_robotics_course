#!/usr/bin/env python3
"""Task 7: EKF Evaluation and GPS Dropout Analysis."""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os
import sys

# Import from task6
sys.path.insert(0, os.path.dirname(__file__))
from task6_full_ekf_solution import run_full_ekf, normalize_angle

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "lab_exercises", "data")


def evaluate_ekf(times, traj, ground_truth):
    """Compute comprehensive EKF evaluation metrics.

    Returns dict with position_rmse, heading_rmse, max_error, final_error.
    """
    gt_t = ground_truth[:, 0]
    gt_px = np.interp(times, gt_t, ground_truth[:, 1])
    gt_py = np.interp(times, gt_t, ground_truth[:, 2])
    gt_theta = np.interp(times, gt_t, ground_truth[:, 3])

    pos_err = np.sqrt((traj[:, 0] - gt_px)**2 + (traj[:, 1] - gt_py)**2)
    heading_err = np.abs(normalize_angle(traj[:, 2] - gt_theta))

    return {
        "position_rmse": np.sqrt(np.mean(pos_err**2)),
        "heading_rmse_deg": np.degrees(np.sqrt(np.mean(heading_err**2))),
        "max_position_error": np.max(pos_err),
        "final_position_error": pos_err[-1],
        "max_heading_error_deg": np.degrees(np.max(heading_err)),
        "final_heading_error_deg": np.degrees(heading_err[-1]),
        "position_errors": pos_err,
        "heading_errors_deg": np.degrees(heading_err),
    }


def test_gps_dropout(imu_data, encoder_data, gps_data, x0, ground_truth,
                      dropout_start, dropout_end):
    """Run EKF but skip GPS updates between dropout_start and dropout_end."""
    def gps_filter(t):
        return t < dropout_start or t > dropout_end

    times, traj, covs = run_full_ekf(
        imu_data, encoder_data, gps_data, x0,
        gps_filter_func=gps_filter
    )
    return times, traj, covs


def plot_comparison(times_full, traj_full, times_dropout, traj_dropout,
                     ground_truth, gps_data, dropout_start, dropout_end, save_path):
    """Overlay full vs dropout trajectories with ground truth."""
    gt_t = ground_truth[:, 0]

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle("Task 7: EKF Evaluation - Full vs GPS Dropout", fontsize=14)

    # (0,0) Trajectory comparison
    ax = axes[0, 0]
    ax.plot(ground_truth[:, 1], ground_truth[:, 2], "g-", label="Ground Truth", linewidth=2)
    ax.plot(traj_full[:, 0], traj_full[:, 1], "b-", label="Full EKF", linewidth=1)
    ax.plot(traj_dropout[:, 0], traj_dropout[:, 1], "r--", label="GPS Dropout", linewidth=1)
    ax.scatter(gps_data[:, 1], gps_data[:, 2], c="m", s=10, alpha=0.3, zorder=3)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Trajectory Comparison")
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True)

    # (0,1) Position error comparison
    ax = axes[0, 1]
    gt_px_f = np.interp(times_full, gt_t, ground_truth[:, 1])
    gt_py_f = np.interp(times_full, gt_t, ground_truth[:, 2])
    err_full = np.sqrt((traj_full[:, 0] - gt_px_f)**2 + (traj_full[:, 1] - gt_py_f)**2)

    gt_px_d = np.interp(times_dropout, gt_t, ground_truth[:, 1])
    gt_py_d = np.interp(times_dropout, gt_t, ground_truth[:, 2])
    err_dropout = np.sqrt((traj_dropout[:, 0] - gt_px_d)**2 + (traj_dropout[:, 1] - gt_py_d)**2)

    ax.plot(times_full, err_full, "b-", label="Full EKF")
    ax.plot(times_dropout, err_dropout, "r-", label="GPS Dropout")
    ax.axvspan(dropout_start, dropout_end, alpha=0.15, color="red", label="Dropout Window")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position Error [m]")
    ax.set_title("Position Error Over Time")
    ax.legend()
    ax.grid(True)

    # (1,0) Heading error comparison
    ax = axes[1, 0]
    gt_th_f = np.interp(times_full, gt_t, ground_truth[:, 3])
    gt_th_d = np.interp(times_dropout, gt_t, ground_truth[:, 3])
    herr_full = np.abs(normalize_angle(traj_full[:, 2] - gt_th_f))
    herr_dropout = np.abs(normalize_angle(traj_dropout[:, 2] - gt_th_d))

    ax.plot(times_full, np.degrees(herr_full), "b-", label="Full EKF")
    ax.plot(times_dropout, np.degrees(herr_dropout), "r-", label="GPS Dropout")
    ax.axvspan(dropout_start, dropout_end, alpha=0.15, color="red", label="Dropout Window")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Heading Error [deg]")
    ax.set_title("Heading Error Over Time")
    ax.legend()
    ax.grid(True)

    # (1,1) Speed comparison
    ax = axes[1, 1]
    gt_speed = np.interp(times_full, gt_t, np.sqrt(ground_truth[:, 4]**2 + ground_truth[:, 5]**2))
    est_speed_full = np.sqrt(traj_full[:, 3]**2 + traj_full[:, 4]**2)
    est_speed_drop = np.sqrt(traj_dropout[:, 3]**2 + traj_dropout[:, 4]**2)

    ax.plot(times_full, gt_speed, "g-", label="GT", linewidth=1.5)
    ax.plot(times_full, est_speed_full, "b-", label="Full EKF", linewidth=0.8)
    gt_speed_d = np.interp(times_dropout, gt_t, np.sqrt(ground_truth[:, 4]**2 + ground_truth[:, 5]**2))
    ax.plot(times_dropout, est_speed_drop, "r--", label="GPS Dropout", linewidth=0.8)
    ax.axvspan(dropout_start, dropout_end, alpha=0.15, color="red", label="Dropout Window")
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

    print("=== Task 7: EKF Evaluation ===\n")

    # Full EKF
    print("Running full EKF...")
    times_full, traj_full, covs_full = run_full_ekf(imu_data, encoder_data, gps_data, x0)
    metrics_full = evaluate_ekf(times_full, traj_full, ground_truth)

    print("Full EKF Metrics:")
    print(f"  Position RMSE:       {metrics_full['position_rmse']:.4f} m")
    print(f"  Heading RMSE:        {metrics_full['heading_rmse_deg']:.4f} deg")
    print(f"  Max position error:  {metrics_full['max_position_error']:.4f} m")
    print(f"  Final position error: {metrics_full['final_position_error']:.4f} m")
    print(f"  Max heading error:   {metrics_full['max_heading_error_deg']:.4f} deg")
    print(f"  Final heading error: {metrics_full['final_heading_error_deg']:.4f} deg")

    # GPS dropout test
    dropout_start = 20.0
    dropout_end = 40.0
    t0 = ground_truth[0, 0]
    dropout_start_abs = t0 + dropout_start
    dropout_end_abs = t0 + dropout_end

    print(f"\nRunning GPS dropout test (t={dropout_start}s to t={dropout_end}s)...")
    times_drop, traj_drop, covs_drop = test_gps_dropout(
        imu_data, encoder_data, gps_data, x0, ground_truth,
        dropout_start_abs, dropout_end_abs
    )
    metrics_drop = evaluate_ekf(times_drop, traj_drop, ground_truth)

    print("GPS Dropout Metrics:")
    print(f"  Position RMSE:       {metrics_drop['position_rmse']:.4f} m")
    print(f"  Heading RMSE:        {metrics_drop['heading_rmse_deg']:.4f} deg")
    print(f"  Max position error:  {metrics_drop['max_position_error']:.4f} m")
    print(f"  Final position error: {metrics_drop['final_position_error']:.4f} m")
    print(f"  Max heading error:   {metrics_drop['max_heading_error_deg']:.4f} deg")
    print(f"  Final heading error: {metrics_drop['final_heading_error_deg']:.4f} deg")

    # Degradation
    print(f"\nDegradation due to GPS dropout:")
    print(f"  Position RMSE increase: "
          f"{(metrics_drop['position_rmse'] / metrics_full['position_rmse'] - 1) * 100:.1f}%")
    print(f"  Max error increase: "
          f"{(metrics_drop['max_position_error'] / metrics_full['max_position_error'] - 1) * 100:.1f}%")

    # Plot
    out_path = os.path.join(os.path.dirname(__file__), "task7_evaluation.png")
    plot_comparison(times_full, traj_full, times_drop, traj_drop,
                     ground_truth, gps_data, dropout_start_abs, dropout_end_abs, out_path)


if __name__ == "__main__":
    main()
