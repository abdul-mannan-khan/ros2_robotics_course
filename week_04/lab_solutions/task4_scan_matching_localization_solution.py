#!/usr/bin/env python3
"""
Week 4 - Task 4 Solution: Scan Matching Localization

Combines odometry prediction with ICP scan matching for improved localization.
Uses world-frame ICP with sanity checks to avoid divergence from bad matches.
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


# ---- ICP functions ----

def scan_to_points(ranges, angles, max_range=11.5):
    valid = ranges < max_range
    return np.column_stack([ranges[valid] * np.cos(angles[valid]),
                            ranges[valid] * np.sin(angles[valid])])


def transform_points_to_world(points, pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    return (R @ points.T).T + pose[:2]


def find_correspondences(source, target, max_dist=1.0):
    if len(source) == 0 or len(target) == 0:
        return np.zeros((0, 2)), np.zeros((0, 2))
    diff = source[:, np.newaxis, :] - target[np.newaxis, :, :]
    dists = np.sqrt((diff ** 2).sum(axis=2))
    nearest_idx = np.argmin(dists, axis=1)
    nearest_dist = dists[np.arange(len(source)), nearest_idx]
    mask = nearest_dist < max_dist
    return source[mask], target[nearest_idx[mask]]


def estimate_transform(source, target):
    src_mean = source.mean(axis=0)
    tgt_mean = target.mean(axis=0)
    H = (source - src_mean).T @ (target - tgt_mean)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = tgt_mean - R @ src_mean
    return R, t


def icp_world(source_world, target_world, max_iterations=20, max_dist=0.3):
    """ICP on world-frame point clouds. Returns R, t correction and mean error."""
    current = source_world.copy()
    R_total = np.eye(2)
    t_total = np.zeros(2)
    for it in range(max_iterations):
        src_m, tgt_m = find_correspondences(current, target_world, max_dist)
        if len(src_m) < 10:
            break
        R, t = estimate_transform(src_m, tgt_m)
        current = (R @ current.T).T + t
        R_total = R @ R_total
        t_total = R @ t_total + t
        if np.linalg.norm(t) < 1e-6:
            break
    # Compute final error
    src_m, tgt_m = find_correspondences(current, target_world, max_dist * 2)
    err = np.sqrt(((src_m - tgt_m)**2).sum(1).mean()) if len(src_m) > 10 else 999.0
    return R_total, t_total, err


def predict_pose(pose, odometry):
    new = pose.copy()
    new[0] += odometry[0]
    new[1] += odometry[1]
    new[2] += odometry[2]
    new[2] = np.arctan2(np.sin(new[2]), np.cos(new[2]))
    return new


def scan_matching_pipeline(scans, odometry, initial_pose):
    """
    Scan matching pipeline with sanity-checked ICP corrections.
    ICP correction is applied only if it is small (consistent with local refinement).
    """
    N = len(scans)
    est_poses = np.zeros((N, 3))
    est_poses[0] = initial_pose
    beam_angles = np.linspace(0, 2 * np.pi, 360, endpoint=False)

    # Max correction to accept from ICP (to avoid divergence)
    max_correction_t = 0.15  # meters
    max_correction_r = 0.05  # radians
    blend_weight = 0.3       # how much to trust ICP vs odometry

    for i in range(1, N):
        predicted = predict_pose(est_poses[i-1], odometry[i])

        pts_cur = scan_to_points(scans[i], beam_angles)
        pts_prev = scan_to_points(scans[i-1], beam_angles)

        if len(pts_cur) >= 20 and len(pts_prev) >= 20:
            cur_world = transform_points_to_world(pts_cur, predicted)
            prev_world = transform_points_to_world(pts_prev, est_poses[i-1])

            R, t, err = icp_world(cur_world, prev_world, max_iterations=15, max_dist=0.3)
            dtheta = np.arctan2(R[1, 0], R[0, 0])

            # Sanity check: only apply small corrections, blended
            if np.linalg.norm(t) < max_correction_t and abs(dtheta) < max_correction_r:
                w = blend_weight
                corrected_pos = R @ predicted[:2] + t
                est_poses[i, 0] = (1 - w) * predicted[0] + w * corrected_pos[0]
                est_poses[i, 1] = (1 - w) * predicted[1] + w * corrected_pos[1]
                blended_th = predicted[2] + w * dtheta
                est_poses[i, 2] = np.arctan2(np.sin(blended_th), np.cos(blended_th))
            else:
                est_poses[i] = predicted
        else:
            est_poses[i] = predicted

    return est_poses


def evaluate_trajectory(estimated, ground_truth):
    errors = np.sqrt((estimated[:, 0] - ground_truth[:, 0])**2 +
                     (estimated[:, 1] - ground_truth[:, 1])**2)
    ate_rmse = np.sqrt(np.mean(errors**2))
    return ate_rmse, errors


def main():
    print("=" * 60)
    print("Task 4: Scan Matching Localization")
    print("=" * 60)

    poses = np.load(os.path.join(DATA_DIR, "ground_truth_poses.npy"))
    scans = np.load(os.path.join(DATA_DIR, "lidar_scans.npy"))
    odometry = np.load(os.path.join(DATA_DIR, "odometry.npy"))

    # Dead reckoning baseline
    print("\n[Step 1] Computing dead reckoning trajectory...")
    odom_poses = np.zeros_like(poses)
    odom_poses[0] = poses[0]
    for i in range(1, len(poses)):
        odom_poses[i] = predict_pose(odom_poses[i-1], odometry[i])
    ate_odom, errors_odom = evaluate_trajectory(odom_poses, poses)
    print(f"  Odometry ATE RMSE: {ate_odom:.3f}m")

    # Scan matching pipeline
    print("\n[Step 2] Running scan matching localization (odometry + ICP)...")
    print("  Using world-frame ICP with sanity-checked corrections")
    sm_poses = scan_matching_pipeline(scans, odometry, poses[0])
    ate_sm, errors_sm = evaluate_trajectory(sm_poses, poses)
    print(f"  Scan Matching ATE RMSE: {ate_sm:.3f}m")
    improvement = (1 - ate_sm / ate_odom) * 100
    if improvement > 0:
        print(f"  Improvement over odometry: {improvement:.1f}%")
    else:
        print(f"  Note: ICP did not improve over odometry ({improvement:.1f}%)")
        print("  This happens when odometry is already good or ICP corrections are noisy.")
        print("  The real benefit of scan matching appears with loop closure (Task 6).")

    # Visualization
    print("\n[Step 3] Creating visualization...")
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    ax = axes[0]
    ax.plot(poses[:, 0], poses[:, 1], 'b-', linewidth=2, label='Ground Truth')
    ax.plot(odom_poses[:, 0], odom_poses[:, 1], 'r--', linewidth=1, label=f'Odometry (ATE={ate_odom:.2f}m)')
    ax.plot(sm_poses[:, 0], sm_poses[:, 1], 'g-', linewidth=1.5, label=f'Scan Matching (ATE={ate_sm:.2f}m)')
    ax.legend(fontsize=8)
    ax.set_title("Trajectory Comparison")
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    ax = axes[1]
    ax.plot(errors_odom, 'r-', label='Odometry')
    ax.plot(errors_sm, 'g-', label='Scan Matching')
    ax.legend()
    ax.set_title("Position Error Over Time")
    ax.set_xlabel("Timestep")
    ax.set_ylabel("Error (m)")
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.bar(['Odometry', 'Scan Matching'], [ate_odom, ate_sm], color=['red', 'green'], alpha=0.7)
    ax.set_title("ATE RMSE Comparison")
    ax.set_ylabel("RMSE (m)")
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, "task4_scan_matching_localization.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"  Saved: {out_path}")

    print("\n[Key Insight] Local scan matching provides small corrections to odometry.")
    print("  Without loop closure, drift still accumulates over the full trajectory.")
    print("  The main value of scan matching is providing relative constraints for")
    print("  pose graph optimization (Tasks 6 & 7).")
    print("\nTask 4 complete.")


if __name__ == "__main__":
    main()
