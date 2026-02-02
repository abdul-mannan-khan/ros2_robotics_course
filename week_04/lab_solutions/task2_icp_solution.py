#!/usr/bin/env python3
"""
Week 4 - Task 2 Solution: Iterative Closest Point (ICP) Scan Matching

Implements ICP with SVD-based transform estimation to align consecutive LiDAR scans.
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


def scan_to_points(ranges, angles, max_range=11.5):
    """Convert polar LiDAR scan to Cartesian points, filtering max-range readings."""
    valid = ranges < max_range
    x = ranges[valid] * np.cos(angles[valid])
    y = ranges[valid] * np.sin(angles[valid])
    return np.column_stack([x, y])


def find_correspondences(source, target, max_dist=1.0):
    """Find nearest-neighbor correspondences using brute-force distance computation."""
    # Compute pairwise distances
    # For efficiency, use broadcasting: (N,1,2) - (1,M,2) -> (N,M,2)
    diff = source[:, np.newaxis, :] - target[np.newaxis, :, :]
    dists = np.sqrt((diff ** 2).sum(axis=2))  # (N, M)
    nearest_idx = np.argmin(dists, axis=1)     # (N,)
    nearest_dist = dists[np.arange(len(source)), nearest_idx]

    mask = nearest_dist < max_dist
    return source[mask], target[nearest_idx[mask]]


def estimate_transform(source, target):
    """
    Estimate 2D rigid transform (R, t) using SVD.
    source and target are Nx2 corresponding point sets.
    Finds R, t such that target ~ R @ source + t.
    """
    src_mean = source.mean(axis=0)
    tgt_mean = target.mean(axis=0)
    src_c = source - src_mean
    tgt_c = target - tgt_mean

    H = src_c.T @ tgt_c  # 2x2
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Handle reflection
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = tgt_mean - R @ src_mean
    return R, t


def apply_transform(points, R, t):
    """Apply rigid transform: p' = R @ p + t."""
    return (R @ points.T).T + t


def icp(source, target, max_iterations=50, tolerance=1e-6, max_dist=1.0):
    """
    Full ICP algorithm.
    Returns cumulative R, t, transformed source, and per-iteration errors.
    """
    current = source.copy()
    R_total = np.eye(2)
    t_total = np.zeros(2)
    errors = []

    for it in range(max_iterations):
        # Find correspondences
        src_matched, tgt_matched = find_correspondences(current, target, max_dist)

        if len(src_matched) < 3:
            print(f"  ICP: too few correspondences ({len(src_matched)}) at iteration {it}")
            break

        # Estimate transform
        R, t = estimate_transform(src_matched, tgt_matched)

        # Apply transform
        current = apply_transform(current, R, t)

        # Accumulate
        R_total = R @ R_total
        t_total = R @ t_total + t

        # Compute error
        src_m2, tgt_m2 = find_correspondences(current, target, max_dist * 2)
        if len(src_m2) > 0:
            err = np.sqrt(((src_m2 - tgt_m2) ** 2).sum(axis=1).mean())
            errors.append(err)
        else:
            errors.append(float('inf'))

        # Check convergence
        delta = np.linalg.norm(t) + abs(np.arctan2(R[1, 0], R[0, 0]))
        if delta < tolerance:
            break

    return R_total, t_total, current, errors


def main():
    print("=" * 60)
    print("Task 2: ICP Scan Matching")
    print("=" * 60)

    scans = np.load(os.path.join(DATA_DIR, "lidar_scans.npy"))
    poses = np.load(os.path.join(DATA_DIR, "ground_truth_poses.npy"))

    angles = np.linspace(0, 2 * np.pi, 360, endpoint=False)

    # Pick two scans that are a few steps apart for a visible transform
    idx_a, idx_b = 10, 15
    print(f"\n[Step 1] Converting scans {idx_a} and {idx_b} to point clouds...")
    pts_a = scan_to_points(scans[idx_a], angles)
    pts_b = scan_to_points(scans[idx_b], angles)
    print(f"  Scan A: {len(pts_a)} points, Scan B: {len(pts_b)} points")

    # Ground truth relative transform
    dx_gt = poses[idx_b, 0] - poses[idx_a, 0]
    dy_gt = poses[idx_b, 1] - poses[idx_a, 1]
    dth_gt = poses[idx_b, 2] - poses[idx_a, 2]
    print(f"\n[Step 2] Ground truth relative pose: dx={dx_gt:.3f}, dy={dy_gt:.3f}, dth={np.degrees(dth_gt):.2f} deg")

    # Transform scan A into scan B's frame using GT for reference, then use ICP
    # Actually, let's align scan B to scan A to estimate the relative motion
    print("\n[Step 3] Running ICP to align scan B -> scan A...")
    R_est, t_est, pts_b_aligned, errors = icp(pts_b, pts_a, max_iterations=50, max_dist=0.5)
    dth_est = np.arctan2(R_est[1, 0], R_est[0, 0])
    print(f"  ICP converged in {len(errors)} iterations")
    print(f"  Estimated: dx={t_est[0]:.3f}, dy={t_est[1]:.3f}, dth={np.degrees(dth_est):.2f} deg")
    print(f"  Final mean correspondence error: {errors[-1]:.4f}m" if errors else "  No error data")

    # Note: ICP in robot frame recovers relative motion between scans.
    # The sign convention depends on which scan is source/target.

    # Visualization
    print("\n[Step 4] Creating alignment visualization...")
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    axes[0].scatter(pts_a[:, 0], pts_a[:, 1], s=1, c='blue', label='Scan A')
    axes[0].scatter(pts_b[:, 0], pts_b[:, 1], s=1, c='red', label='Scan B')
    axes[0].set_title("Before ICP")
    axes[0].legend()
    axes[0].set_aspect('equal')
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")

    axes[1].scatter(pts_a[:, 0], pts_a[:, 1], s=1, c='blue', label='Scan A (target)')
    axes[1].scatter(pts_b_aligned[:, 0], pts_b_aligned[:, 1], s=1, c='green', label='Scan B (aligned)')
    axes[1].set_title("After ICP")
    axes[1].legend()
    axes[1].set_aspect('equal')

    if errors:
        axes[2].plot(errors, 'b-o', markersize=3)
        axes[2].set_title("ICP Convergence")
        axes[2].set_xlabel("Iteration")
        axes[2].set_ylabel("Mean Correspondence Error (m)")
        axes[2].grid(True)

    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, "task2_icp.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"  Saved: {out_path}")

    # Additional: test on multiple scan pairs
    print("\n[Step 5] Testing ICP on multiple consecutive scan pairs...")
    for step in [1, 5, 10, 20]:
        idx_a, idx_b = 50, 50 + step
        if idx_b >= len(scans):
            break
        pa = scan_to_points(scans[idx_a], angles)
        pb = scan_to_points(scans[idx_b], angles)
        R, t, _, errs = icp(pb, pa, max_iterations=50, max_dist=0.5)
        dth = np.arctan2(R[1, 0], R[0, 0])
        gt_dx = poses[idx_b, 0] - poses[idx_a, 0]
        gt_dy = poses[idx_b, 1] - poses[idx_a, 1]
        print(f"  Scans {idx_a}->{idx_b}: ICP t=[{t[0]:.3f},{t[1]:.3f}] "
              f"GT delta=[{-gt_dx:.3f},{-gt_dy:.3f}] "
              f"final_err={errs[-1]:.4f}m" if errs else f"  no convergence")

    print("\nTask 2 complete.")


if __name__ == "__main__":
    main()
