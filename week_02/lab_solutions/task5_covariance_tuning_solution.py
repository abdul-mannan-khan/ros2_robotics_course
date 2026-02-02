#!/usr/bin/env python3
"""Task 5: Covariance Tuning via Grid Search."""

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
        vx + ax * dt, vy + ay * dt, omega + alpha * dt,
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
    vx, vy = x[3], x[4]
    return np.array([np.sqrt(vx**2 + vy**2), x[5]])


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


def ekf_update(x, P, z, z_pred, H, R):
    innovation = z - z_pred
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x_upd = x + K @ innovation
    x_upd[2] = normalize_angle(x_upd[2])
    P_upd = (np.eye(6) - K @ H) @ P
    return x_upd, P_upd


def run_ekf_with_params(imu_data, encoder_data, gps_data, x0, Q_scale, r_enc_scale, r_gps_scale):
    """Run full EKF with scaled covariance parameters. Returns times and trajectory."""
    Q_base = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01])
    R_enc_base = np.diag([0.05**2, 0.02**2])
    R_gps_base = np.diag([2.0**2, 2.0**2])

    Q = Q_base * Q_scale
    R_enc = R_enc_base * r_enc_scale
    R_gps = R_gps_base * r_gps_scale

    x = x0.copy()
    P = np.diag([1.0, 1.0, 0.1, 0.5, 0.5, 0.1])

    traj = [x.copy()]
    times = [imu_data[0, 0]]

    enc_idx = 0
    gps_idx = 0

    for i in range(1, len(imu_data)):
        dt = imu_data[i, 0] - imu_data[i - 1, 0]
        if dt <= 0:
            continue
        x, P = predict(x, P, imu_data[i, 1:], dt, Q)
        t_now = imu_data[i, 0]

        while enc_idx < len(encoder_data) and encoder_data[enc_idx, 0] <= t_now:
            z = encoder_data[enc_idx, 1:]
            z_pred = encoder_measurement_model(x)
            H = compute_encoder_jacobian(x)
            x, P = ekf_update(x, P, z, z_pred, H, R_enc)
            enc_idx += 1

        while gps_idx < len(gps_data) and gps_data[gps_idx, 0] <= t_now:
            z = gps_data[gps_idx, 1:]
            z_pred = gps_measurement_model(x)
            H = compute_gps_jacobian(x)
            x, P = ekf_update(x, P, z, z_pred, H, R_gps)
            gps_idx += 1

        traj.append(x.copy())
        times.append(t_now)

    return np.array(times), np.array(traj)


def compute_rmse(times, traj, ground_truth):
    """Compute position RMSE by interpolating estimate to GT timestamps."""
    gt_t = ground_truth[:, 0]
    gt_px = ground_truth[:, 1]
    gt_py = ground_truth[:, 2]

    est_px = np.interp(gt_t, times, traj[:, 0])
    est_py = np.interp(gt_t, times, traj[:, 1])

    rmse = np.sqrt(np.mean((est_px - gt_px)**2 + (est_py - gt_py)**2))
    return rmse


def grid_search_tuning(imu_data, encoder_data, gps_data, ground_truth, x0,
                        q_scales, r_enc_scales, r_gps_scales):
    """Grid search over Q/R scale factors. Returns best params and results dict."""
    best_rmse = np.inf
    best_params = None
    results = {}

    total = len(q_scales) * len(r_enc_scales) * len(r_gps_scales)
    count = 0

    for qs in q_scales:
        for res in r_enc_scales:
            for rgs in r_gps_scales:
                count += 1
                times, traj = run_ekf_with_params(
                    imu_data, encoder_data, gps_data, x0, qs, res, rgs
                )
                rmse = compute_rmse(times, traj, ground_truth)
                results[(qs, res, rgs)] = rmse
                if rmse < best_rmse:
                    best_rmse = rmse
                    best_params = (qs, res, rgs)
                if count % 10 == 0:
                    print(f"  Progress: {count}/{total} combinations evaluated...")

    return best_params, best_rmse, results


def main():
    ground_truth = np.load(os.path.join(DATA_DIR, "ground_truth.npy"))
    imu_data = np.load(os.path.join(DATA_DIR, "imu_data.npy"))
    encoder_data = np.load(os.path.join(DATA_DIR, "encoder_data.npy"))
    gps_data = np.load(os.path.join(DATA_DIR, "gps_data.npy"))

    x0 = ground_truth[0, 1:].copy()

    q_scales = [0.001, 0.01, 0.1, 1.0]
    r_enc_scales = [0.1, 0.5, 1.0, 2.0]
    r_gps_scales = [1.0, 2.0, 5.0, 10.0]

    print("=== Task 5: Covariance Tuning ===")
    print(f"Grid search: {len(q_scales)}x{len(r_enc_scales)}x{len(r_gps_scales)} = "
          f"{len(q_scales)*len(r_enc_scales)*len(r_gps_scales)} combinations\n")

    best_params, best_rmse, results = grid_search_tuning(
        imu_data, encoder_data, gps_data, ground_truth, x0,
        q_scales, r_enc_scales, r_gps_scales
    )

    print(f"\nBest parameters:")
    print(f"  Q_scale:     {best_params[0]}")
    print(f"  R_enc_scale: {best_params[1]}")
    print(f"  R_gps_scale: {best_params[2]}")
    print(f"  Position RMSE: {best_rmse:.4f} m")

    # Run best and default for comparison
    times_best, traj_best = run_ekf_with_params(
        imu_data, encoder_data, gps_data, x0, *best_params
    )
    times_def, traj_def = run_ekf_with_params(
        imu_data, encoder_data, gps_data, x0, 1.0, 1.0, 1.0
    )
    rmse_def = compute_rmse(times_def, traj_def, ground_truth)
    print(f"\nDefault params RMSE: {rmse_def:.4f} m")
    print(f"Improvement: {(1 - best_rmse / rmse_def) * 100:.1f}%")

    # Top 5 results
    sorted_results = sorted(results.items(), key=lambda kv: kv[1])
    print("\nTop 5 parameter combinations:")
    for (qs, res, rgs), rmse in sorted_results[:5]:
        print(f"  Q={qs:6.3f}, R_enc={res:4.1f}, R_gps={rgs:5.1f} -> RMSE={rmse:.4f} m")

    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Task 5: Covariance Tuning")

    axes[0].plot(ground_truth[:, 1], ground_truth[:, 2], "g-", label="Ground Truth", linewidth=2)
    axes[0].plot(traj_def[:, 0], traj_def[:, 1], "r--", label="Default", alpha=0.7)
    axes[0].plot(traj_best[:, 0], traj_best[:, 1], "b-", label="Best Tuned", linewidth=1)
    axes[0].set_xlabel("X [m]")
    axes[0].set_ylabel("Y [m]")
    axes[0].legend()
    axes[0].set_aspect("equal")
    axes[0].grid(True)
    axes[0].set_title("Trajectory Comparison")

    # RMSE heatmap for one Q scale (best)
    qs_best = best_params[0]
    rmse_grid = np.zeros((len(r_enc_scales), len(r_gps_scales)))
    for ei, res in enumerate(r_enc_scales):
        for gi, rgs in enumerate(r_gps_scales):
            rmse_grid[ei, gi] = results.get((qs_best, res, rgs), np.nan)

    im = axes[1].imshow(rmse_grid, origin="lower", aspect="auto", cmap="viridis")
    axes[1].set_xticks(range(len(r_gps_scales)))
    axes[1].set_xticklabels([str(s) for s in r_gps_scales])
    axes[1].set_yticks(range(len(r_enc_scales)))
    axes[1].set_yticklabels([str(s) for s in r_enc_scales])
    axes[1].set_xlabel("R_gps scale")
    axes[1].set_ylabel("R_enc scale")
    axes[1].set_title(f"RMSE Heatmap (Q_scale={qs_best})")
    plt.colorbar(im, ax=axes[1], label="RMSE [m]")

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "task5_tuning.png")
    plt.savefig(out_path, dpi=150)
    print(f"\nPlot saved to {out_path}")
    plt.close()


if __name__ == "__main__":
    main()
