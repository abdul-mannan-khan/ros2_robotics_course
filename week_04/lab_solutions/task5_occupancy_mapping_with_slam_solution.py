#!/usr/bin/env python3
"""
Week 4 - Task 5 Solution: Occupancy Mapping with SLAM Poses

Builds occupancy grid maps using both ground truth and estimated poses, then compares.
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


# ---- ICP + scan matching pipeline (from Task 4) ----

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
    nn = np.argmin(dists, axis=1)
    nd = dists[np.arange(len(source)), nn]
    mask = nd < max_dist
    return source[mask], target[nn[mask]]

def estimate_transform_svd(source, target):
    sm, tm = source.mean(0), target.mean(0)
    H = (source - sm).T @ (target - tm)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    return R, tm - R @ sm

def icp_world(source, target, max_iter=15, max_dist=0.3):
    cur = source.copy()
    Rt, tt = np.eye(2), np.zeros(2)
    for _ in range(max_iter):
        sm, tm = find_correspondences(cur, target, max_dist)
        if len(sm) < 10:
            break
        R, t = estimate_transform_svd(sm, tm)
        cur = (R @ cur.T).T + t
        Rt, tt = R @ Rt, R @ tt + t
        if np.linalg.norm(t) < 1e-6:
            break
    return Rt, tt

def simple_slam_pipeline(scans, odometry, initial_pose):
    """Odometry + conservative ICP correction."""
    N = len(scans)
    est = np.zeros((N, 3))
    est[0] = initial_pose
    angles = np.linspace(0, 2 * np.pi, 360, endpoint=False)
    blend_w = 0.3

    for i in range(1, N):
        est[i] = est[i-1] + odometry[i]
        est[i, 2] = np.arctan2(np.sin(est[i, 2]), np.cos(est[i, 2]))

        pc = scan_to_points(scans[i], angles)
        pr = scan_to_points(scans[i-1], angles)
        if len(pc) >= 20 and len(pr) >= 20:
            cw = transform_points_to_world(pc, est[i])
            pw = transform_points_to_world(pr, est[i-1])
            R, t = icp_world(cw, pw)
            dth = np.arctan2(R[1, 0], R[0, 0])
            if np.linalg.norm(t) < 0.15 and abs(dth) < 0.05:
                cp = R @ est[i, :2] + t
                est[i, 0] = (1 - blend_w) * est[i, 0] + blend_w * cp[0]
                est[i, 1] = (1 - blend_w) * est[i, 1] + blend_w * cp[1]
                est[i, 2] += blend_w * dth
    return est


# ---- Occupancy grid mapping (from Task 1) ----

def bresenham_ray(x0, y0, x1, y1):
    cells = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells

def build_map_from_estimates(poses, scans, grid_size=500, resolution=0.1, max_range=12.0):
    grid = np.zeros((grid_size, grid_size), dtype=np.float64)
    angles_base = np.linspace(0, 2 * np.pi, 360, endpoint=False)
    l_occ, l_free = 0.85, -0.4

    for idx in range(len(poses)):
        x, y, theta = poses[idx]
        scan = scans[idx]
        gx, gy = int(x / resolution), int(y / resolution)
        angles = angles_base + theta

        for i in range(360):
            r = scan[i]
            ex = x + r * np.cos(angles[i])
            ey = y + r * np.sin(angles[i])
            egx = int(np.clip(ex / resolution, 0, grid_size - 1))
            egy = int(np.clip(ey / resolution, 0, grid_size - 1))
            cells = bresenham_ray(gx, gy, egx, egy)
            for cx, cy in cells:
                if 0 <= cx < grid_size and 0 <= cy < grid_size:
                    cd = np.sqrt((cx * resolution - x)**2 + (cy * resolution - y)**2)
                    if r < max_range - 0.1 and abs(cd - r) < 0.3:
                        grid[cy, cx] = np.clip(grid[cy, cx] + l_occ, -10, 10)
                    elif cd < r - 0.3:
                        grid[cy, cx] = np.clip(grid[cy, cx] + l_free, -10, 10)

    return 1.0 - 1.0 / (1.0 + np.exp(grid))


def compare_maps(estimated_map, ground_truth_map, threshold=0.6):
    est_occ = estimated_map > threshold
    est_free = estimated_map < (1.0 - threshold)
    gt_occ = ground_truth_map > 0.5
    explored = est_occ | est_free
    if explored.sum() == 0:
        return 0, 0, 0
    correct = (est_occ & gt_occ) | (est_free & ~gt_occ)
    accuracy = correct[explored].sum() / explored.sum()
    tp = (est_occ & gt_occ).sum()
    fp = (est_occ & ~gt_occ).sum()
    fn = (~est_occ & gt_occ & explored).sum()
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    return accuracy, precision, recall


def main():
    print("=" * 60)
    print("Task 5: Occupancy Mapping with SLAM Poses")
    print("=" * 60)

    poses = np.load(os.path.join(DATA_DIR, "ground_truth_poses.npy"))
    scans = np.load(os.path.join(DATA_DIR, "lidar_scans.npy"))
    odometry = np.load(os.path.join(DATA_DIR, "odometry.npy"))
    environment = np.load(os.path.join(DATA_DIR, "environment.npy"))

    # Map with ground truth poses
    print("\n[Step 1] Building map with ground truth poses...")
    gt_map = build_map_from_estimates(poses, scans)
    acc_gt, prec_gt, rec_gt = compare_maps(gt_map, environment)
    print(f"  Accuracy: {acc_gt:.2%}, Precision: {prec_gt:.2%}, Recall: {rec_gt:.2%}")

    # Estimate poses
    print("\n[Step 2] Estimating poses with odometry + ICP...")
    est_poses = simple_slam_pipeline(scans, odometry, poses[0])
    ate = np.sqrt(np.mean((est_poses[:, :2] - poses[:, :2])**2))
    print(f"  Pose estimation ATE: {ate:.3f}m")

    # Map with estimated poses
    print("\n[Step 3] Building map with estimated poses...")
    est_map = build_map_from_estimates(est_poses, scans)
    acc_est, prec_est, rec_est = compare_maps(est_map, environment)
    print(f"  Accuracy: {acc_est:.2%}, Precision: {prec_est:.2%}, Recall: {rec_est:.2%}")

    # Odometry-only poses for comparison
    odom_poses = np.zeros_like(poses)
    odom_poses[0] = poses[0]
    for i in range(1, len(poses)):
        odom_poses[i] = odom_poses[i-1] + odometry[i]
    odom_map = build_map_from_estimates(odom_poses, scans)
    acc_odom, _, _ = compare_maps(odom_map, environment)

    # Visualization
    print("\n[Step 4] Creating visualization...")
    fig, axes = plt.subplots(1, 4, figsize=(24, 6))

    axes[0].imshow(environment, cmap='gray_r', origin='lower')
    axes[0].set_title("Ground Truth Environment")

    axes[1].imshow(gt_map, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    axes[1].plot(poses[:, 0] / 0.1, poses[:, 1] / 0.1, 'b-', lw=0.5, alpha=0.5)
    axes[1].set_title(f"Map (GT Poses)\nAcc={acc_gt:.1%}")

    axes[2].imshow(est_map, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    axes[2].plot(est_poses[:, 0] / 0.1, est_poses[:, 1] / 0.1, 'g-', lw=0.5, alpha=0.5)
    axes[2].set_title(f"Map (SLAM Poses)\nAcc={acc_est:.1%}")

    axes[3].imshow(odom_map, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    axes[3].plot(odom_poses[:, 0] / 0.1, odom_poses[:, 1] / 0.1, 'r-', lw=0.5, alpha=0.5)
    axes[3].set_title(f"Map (Odometry Only)\nAcc={acc_odom:.1%}")

    for ax in axes:
        ax.set_xlabel("x (cells)")
        ax.set_ylabel("y (cells)")

    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, "task5_occupancy_mapping_with_slam.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"  Saved: {out_path}")

    print(f"\n[Summary]")
    print(f"  GT poses map accuracy:       {acc_gt:.2%}")
    print(f"  SLAM poses map accuracy:     {acc_est:.2%}")
    print(f"  Odometry poses map accuracy: {acc_odom:.2%}")
    print("\n[Key Insight] Pose estimation errors cause map artifacts (blurring, ghosting).")
    print("  Better pose estimates yield cleaner maps. Loop closure (Task 6) helps further.")
    print("\nTask 5 complete.")


if __name__ == "__main__":
    main()
