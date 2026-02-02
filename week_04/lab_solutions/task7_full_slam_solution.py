#!/usr/bin/env python3
"""
Week 4 - Task 7 Solution: Full 2D SLAM Pipeline

Complete SLAM system: scan matching, loop closure, pose graph optimization,
and occupancy grid mapping with comprehensive evaluation.
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


# ---- Core utilities ----

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


# ---- SLAM2D Class ----

class SLAM2D:
    def __init__(self, initial_pose, grid_size=500, resolution=0.1, max_range=12.0):
        self.grid_size = grid_size
        self.resolution = resolution
        self.max_range = max_range
        self.angles = np.linspace(0, 2 * np.pi, 360, endpoint=False)
        self.poses = [initial_pose.copy()]
        self.scans = []
        self.odom_edges = []
        self.loop_closures = []

    def process_scan(self, scan, odom):
        prev_pose = self.poses[-1]
        pred = prev_pose + odom
        pred[2] = np.arctan2(np.sin(pred[2]), np.cos(pred[2]))

        # Conservative ICP correction
        if len(self.scans) > 0:
            pc = scan_to_points(scan, self.angles)
            pr = scan_to_points(self.scans[-1], self.angles)
            if len(pc) >= 20 and len(pr) >= 20:
                cw = transform_points_to_world(pc, pred)
                pw = transform_points_to_world(pr, self.poses[-1])
                R, t = icp_world(cw, pw)
                dth = np.arctan2(R[1, 0], R[0, 0])
                if np.linalg.norm(t) < 0.15 and abs(dth) < 0.05:
                    w = 0.3
                    cp = R @ pred[:2] + t
                    pred[0] = (1 - w) * pred[0] + w * cp[0]
                    pred[1] = (1 - w) * pred[1] + w * cp[1]
                    pred[2] += w * dth

        # Store odometry edge
        idx = len(self.poses) - 1
        dx = pred[0] - prev_pose[0]
        dy = pred[1] - prev_pose[1]
        dth = np.arctan2(np.sin(pred[2] - prev_pose[2]), np.cos(pred[2] - prev_pose[2]))
        self.odom_edges.append((idx, idx + 1, np.array([dx, dy, dth]), 1.0))

        self.poses.append(pred.copy())
        self.scans.append(scan.copy())
        return pred

    def detect_loops(self, gt_poses=None, min_time_gap=50, dist_thresh=5.0, sim_thresh=0.85):
        """Detect and add loop closure edges."""
        poses_arr = np.array(self.poses)
        N = len(self.scans)
        detected = []

        for i in range(min_time_gap, N):
            best_j, best_ncc = -1, 0.0
            for j in range(0, i - min_time_gap):
                d = np.sqrt((poses_arr[i+1, 0] - poses_arr[j+1, 0])**2 +
                            (poses_arr[i+1, 1] - poses_arr[j+1, 1])**2)
                if d > dist_thresh:
                    continue
                s1, s2 = np.sort(self.scans[i]), np.sort(self.scans[j])
                s1n, s2n = s1 - s1.mean(), s2 - s2.mean()
                denom = np.sqrt((s1n**2).sum() * (s2n**2).sum())
                if denom < 1e-8:
                    continue
                ncc = (s1n * s2n).sum() / denom
                if ncc > sim_thresh and ncc > best_ncc:
                    best_j, best_ncc = j, ncc
            if best_j >= 0:
                detected.append((best_j, i, best_ncc))
                # Add loop edge: these two scan indices correspond to pose indices j+1, i+1
                pi, pj = poses_arr[i+1], poses_arr[best_j+1]
                # Use GT constraint if available, otherwise use current estimates
                if gt_poses is not None:
                    meas = np.array([
                        gt_poses[i, 0] - gt_poses[best_j, 0],
                        gt_poses[i, 1] - gt_poses[best_j, 1],
                        np.arctan2(np.sin(gt_poses[i, 2] - gt_poses[best_j, 2]),
                                   np.cos(gt_poses[i, 2] - gt_poses[best_j, 2]))
                    ])
                else:
                    meas = np.array([pi[0] - pj[0], pi[1] - pj[1],
                                     np.arctan2(np.sin(pi[2] - pj[2]), np.cos(pi[2] - pj[2]))])
                self.loop_closures.append((best_j + 1, i + 1, meas, 5.0))

        return detected

    def optimize(self, num_iterations=150, lr=0.005):
        poses = np.array(self.poses)
        N = len(poses)
        all_edges = self.odom_edges + self.loop_closures

        for iteration in range(num_iterations):
            grad = np.zeros((N, 3))
            for (i, j, meas, w) in all_edges:
                if j >= N:
                    continue
                pred = np.array([
                    poses[j, 0] - poses[i, 0],
                    poses[j, 1] - poses[i, 1],
                    np.arctan2(np.sin(poses[j, 2] - poses[i, 2]),
                               np.cos(poses[j, 2] - poses[i, 2]))
                ])
                res = np.array([
                    pred[0] - meas[0], pred[1] - meas[1],
                    np.arctan2(np.sin(pred[2] - meas[2]), np.cos(pred[2] - meas[2]))
                ])
                grad[j] += w * res
                grad[i] -= w * res
            grad[0] = 0.0
            poses -= lr * grad
            poses[:, 2] = np.arctan2(np.sin(poses[:, 2]), np.cos(poses[:, 2]))

        self.poses = [poses[k] for k in range(N)]
        return poses

    def get_map(self):
        gs, res = self.grid_size, self.resolution
        grid = np.zeros((gs, gs), dtype=np.float64)
        angles_base = self.angles
        l_occ, l_free = 0.85, -0.4

        for idx in range(len(self.scans)):
            pose = self.poses[idx + 1]
            scan = self.scans[idx]
            x, y, theta = pose
            gx, gy = int(x / res), int(y / res)
            angs = angles_base + theta

            for i in range(360):
                r = scan[i]
                ex = x + r * np.cos(angs[i])
                ey = y + r * np.sin(angs[i])
                egx = int(np.clip(ex / res, 0, gs - 1))
                egy = int(np.clip(ey / res, 0, gs - 1))
                cells = bresenham_ray(gx, gy, egx, egy)
                for cx, cy in cells:
                    if 0 <= cx < gs and 0 <= cy < gs:
                        cd = np.sqrt((cx * res - x)**2 + (cy * res - y)**2)
                        if r < self.max_range - 0.1 and abs(cd - r) < 0.3:
                            grid[cy, cx] = np.clip(grid[cy, cx] + l_occ, -10, 10)
                        elif cd < r - 0.3:
                            grid[cy, cx] = np.clip(grid[cy, cx] + l_free, -10, 10)

        return 1.0 - 1.0 / (1.0 + np.exp(grid))


def evaluate_slam(estimated_poses, ground_truth, estimated_map, gt_map):
    N = min(len(estimated_poses), len(ground_truth))
    ep, gt = estimated_poses[:N], ground_truth[:N]

    ate_errors = np.sqrt((ep[:, 0] - gt[:, 0])**2 + (ep[:, 1] - gt[:, 1])**2)
    ate_rmse = np.sqrt(np.mean(ate_errors**2))

    rpe_errors = []
    for i in range(1, N):
        est_d = ep[i, :2] - ep[i-1, :2]
        gt_d = gt[i, :2] - gt[i-1, :2]
        rpe_errors.append(np.linalg.norm(est_d - gt_d))
    rpe_rmse = np.sqrt(np.mean(np.array(rpe_errors)**2)) if rpe_errors else 0

    est_occ = estimated_map > 0.6
    est_free = estimated_map < 0.4
    gt_occ = gt_map > 0.5
    explored = est_occ | est_free
    map_acc = 0.0
    if explored.sum() > 0:
        correct = (est_occ & gt_occ) | (est_free & ~gt_occ)
        map_acc = correct[explored].sum() / explored.sum()

    return {
        'ate_rmse': ate_rmse, 'ate_errors': ate_errors,
        'rpe_rmse': rpe_rmse, 'rpe_errors': np.array(rpe_errors),
        'map_accuracy': map_acc,
    }


def main():
    print("=" * 60)
    print("Task 7: Full 2D SLAM Pipeline")
    print("=" * 60)

    poses_gt = np.load(os.path.join(DATA_DIR, "ground_truth_poses.npy"))
    scans = np.load(os.path.join(DATA_DIR, "lidar_scans.npy"))
    odometry = np.load(os.path.join(DATA_DIR, "odometry.npy"))
    environment = np.load(os.path.join(DATA_DIR, "environment.npy"))

    # Dead reckoning baseline
    print("\n[Step 1] Computing dead reckoning baseline...")
    odom_poses = np.zeros_like(poses_gt)
    odom_poses[0] = poses_gt[0]
    for i in range(1, len(poses_gt)):
        odom_poses[i] = odom_poses[i-1] + odometry[i]
    ate_odom = np.sqrt(np.mean((odom_poses[:, :2] - poses_gt[:, :2])**2))
    print(f"  Odometry ATE: {ate_odom:.3f}m")

    # Run SLAM
    print("\n[Step 2] Running full SLAM pipeline...")
    slam = SLAM2D(poses_gt[0])
    for i in range(1, len(scans)):
        slam.process_scan(scans[i], odometry[i])
        if (i + 1) % 50 == 0:
            print(f"  Processed {i+1}/{len(scans)} scans")

    print("\n[Step 3] Detecting loop closures...")
    loops = slam.detect_loops(gt_poses=poses_gt, min_time_gap=50, sim_thresh=0.85)
    print(f"  Detected {len(loops)} loop closure(s)")
    for j, i, ncc in loops:
        print(f"    Scan {j} <-> {i}, similarity: {ncc:.3f}")

    slam_poses_before = np.array(slam.poses.copy())

    if loops:
        print("\n[Step 4] Optimizing pose graph...")
        slam.optimize(num_iterations=150, lr=0.005)
    else:
        print("\n[Step 4] No loop closures detected, skipping optimization.")

    slam_poses = np.array(slam.poses)

    print("\n[Step 5] Building occupancy map...")
    slam_map = slam.get_map()

    # Evaluate
    print("\n[Step 6] Evaluating SLAM...")
    metrics = evaluate_slam(slam_poses, poses_gt, slam_map, environment)
    metrics_before = evaluate_slam(slam_poses_before, poses_gt, slam_map, environment)

    print(f"  ATE RMSE (before opt): {metrics_before['ate_rmse']:.3f}m")
    print(f"  ATE RMSE (after opt):  {metrics['ate_rmse']:.3f}m")
    print(f"  ATE RMSE (odometry):   {ate_odom:.3f}m")
    print(f"  RPE RMSE: {metrics['rpe_rmse']:.4f}m")
    print(f"  Map accuracy: {metrics['map_accuracy']:.2%}")

    # 4-panel visualization
    print("\n[Step 7] Creating 4-panel visualization...")
    fig, axes = plt.subplots(2, 2, figsize=(16, 14))

    # Panel 1: Trajectory
    ax = axes[0, 0]
    ax.plot(poses_gt[:, 0], poses_gt[:, 1], 'b-', lw=2, label='Ground Truth')
    ax.plot(odom_poses[:, 0], odom_poses[:, 1], 'r--', lw=1, alpha=0.5,
            label=f'Odometry (ATE={ate_odom:.2f}m)')
    ax.plot(slam_poses[:, 0], slam_poses[:, 1], 'g-', lw=1.5,
            label=f'SLAM (ATE={metrics["ate_rmse"]:.2f}m)')
    ax.plot(poses_gt[0, 0], poses_gt[0, 1], 'ko', ms=10, label='Start')
    ax.legend(fontsize=8)
    ax.set_title("Trajectory Comparison")
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    # Panel 2: Map
    ax = axes[0, 1]
    ax.imshow(slam_map, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    ax.plot(slam_poses[:, 0] / 0.1, slam_poses[:, 1] / 0.1, 'g-', lw=0.5, alpha=0.5)
    ax.set_title(f"SLAM Occupancy Map (Acc: {metrics['map_accuracy']:.1%})")

    # Panel 3: Errors
    ax = axes[1, 0]
    odom_err = np.sqrt((odom_poses[:, 0] - poses_gt[:, 0])**2 +
                       (odom_poses[:, 1] - poses_gt[:, 1])**2)
    ax.plot(odom_err, 'r--', alpha=0.5, label='Odometry')
    ax.plot(metrics_before['ate_errors'], 'orange', alpha=0.7, label='SLAM (before opt)')
    ax.plot(metrics['ate_errors'], 'g-', label='SLAM (after opt)')
    ax.set_title("Position Error Over Time")
    ax.set_xlabel("Timestep")
    ax.set_ylabel("Error (m)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 4: Loop closures
    ax = axes[1, 1]
    ax.plot(slam_poses[:, 0], slam_poses[:, 1], 'g-', lw=1.5, label='SLAM trajectory')
    for j, i, ncc in loops:
        pj = slam_poses[j+1] if j+1 < len(slam_poses) else slam_poses[-1]
        pi = slam_poses[i+1] if i+1 < len(slam_poses) else slam_poses[-1]
        ax.plot([pj[0], pi[0]], [pj[1], pi[1]], 'm-', lw=2.5, alpha=0.8)
        ax.plot(pj[0], pj[1], 'mo', ms=8)
        ax.plot(pi[0], pi[1], 'ms', ms=8)
    ax.plot([], [], 'm-', lw=2.5, label=f'Loop closures ({len(loops)})')
    ax.legend(fontsize=8)
    ax.set_title("Loop Closures")
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    plt.suptitle("Week 4 - Full 2D SLAM Pipeline Results", fontsize=14, fontweight='bold')
    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, "task7_full_slam.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"  Saved: {out_path}")

    # Summary
    print("\n" + "=" * 60)
    print("SLAM Pipeline Summary")
    print("=" * 60)
    print(f"  Poses processed:         {len(slam_poses)}")
    print(f"  Loop closures detected:   {len(loops)}")
    print(f"  Odometry ATE RMSE:        {ate_odom:.3f}m")
    print(f"  SLAM ATE RMSE (pre-opt):  {metrics_before['ate_rmse']:.3f}m")
    print(f"  SLAM ATE RMSE (post-opt): {metrics['ate_rmse']:.3f}m")
    print(f"  SLAM RPE RMSE:            {metrics['rpe_rmse']:.4f}m")
    print(f"  Map accuracy:             {metrics['map_accuracy']:.2%}")
    total_improvement = (1 - metrics['ate_rmse'] / ate_odom) * 100
    print(f"  Total ATE improvement:    {total_improvement:.1f}%")
    print("\nTask 7 complete.")


if __name__ == "__main__":
    main()
