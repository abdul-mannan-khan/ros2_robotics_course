#!/usr/bin/env python3
"""
Week 10 - Task 3 Solution: Homography Estimation
DLT homography, RANSAC, decomposition.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def compute_homography_dlt(src_pts, dst_pts):
    """Compute homography using DLT with SVD."""
    n = len(src_pts)
    A = np.zeros((2 * n, 9))
    for i in range(n):
        x, y = src_pts[i]
        u, v = dst_pts[i]
        A[2 * i] = [-x, -y, -1, 0, 0, 0, u * x, u * y, u]
        A[2 * i + 1] = [0, 0, 0, -x, -y, -1, v * x, v * y, v]

    _, _, Vt = np.linalg.svd(A)
    H = Vt[-1].reshape(3, 3)
    return H / H[2, 2]


def apply_homography(H, points):
    """Apply homography to points."""
    ones = np.ones((len(points), 1))
    pts_h = np.hstack([points, ones])
    transformed = (H @ pts_h.T).T
    return transformed[:, :2] / transformed[:, 2:3]


def ransac_homography(src_pts, dst_pts, threshold=3.0, max_iter=1000):
    """RANSAC robust homography estimation."""
    n = len(src_pts)
    best_inliers = np.zeros(n, dtype=bool)
    best_H = None
    best_count = 0

    np.random.seed(42)
    for _ in range(max_iter):
        # Sample 4 random correspondences
        idx = np.random.choice(n, 4, replace=False)
        try:
            H = compute_homography_dlt(src_pts[idx], dst_pts[idx])
        except np.linalg.LinAlgError:
            continue

        # Count inliers
        projected = apply_homography(H, src_pts)
        errors = np.linalg.norm(projected - dst_pts, axis=1)
        inliers = errors < threshold
        count = np.sum(inliers)

        if count > best_count:
            best_count = count
            best_inliers = inliers.copy()
            best_H = H.copy()

    # Refit on all inliers
    if best_count >= 4:
        best_H = compute_homography_dlt(src_pts[best_inliers], dst_pts[best_inliers])

    return best_H, best_inliers


def decompose_homography(H, K):
    """Decompose homography into R, t, n."""
    K_inv = np.linalg.inv(K)
    H_norm = K_inv @ H @ K

    U, S, Vt = np.linalg.svd(H_norm)

    # Normalize by middle singular value
    H_norm = H_norm / S[1]

    # Simple decomposition: assume planar scene
    # H_norm = R + t * n^T
    # Use SVD-based decomposition
    U2, S2, Vt2 = np.linalg.svd(H_norm)

    R = U2 @ np.diag([1, 1, np.linalg.det(U2 @ Vt2)]) @ Vt2
    t = H_norm[:, 2] - R[:, 2]
    t = t.reshape(3, 1)

    # Estimate normal (simplified)
    n = np.array([[0], [0], [1.0]])

    return R, t, n


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, '..', 'lab_exercises', 'data')
    output_dir = script_dir

    print("Week 10 - Task 3 Solution: Homography Estimation")
    print("=" * 50)

    pts1 = np.load(os.path.join(data_dir, 'feature_points_1.npy'))
    pts2 = np.load(os.path.join(data_dir, 'feature_points_2.npy'))
    K = np.load(os.path.join(data_dir, 'camera_intrinsics.npy'))

    print(f"Points: {len(pts1)} correspondences")

    # RANSAC homography
    H, inlier_mask = ransac_homography(pts1, pts2, threshold=5.0, max_iter=2000)
    n_inliers = np.sum(inlier_mask)
    n_outliers = len(pts1) - n_inliers
    print(f"Inliers: {n_inliers}, Outliers: {n_outliers}")
    print(f"Homography H:\n{H}")

    # Apply homography
    pts1_warped = apply_homography(H, pts1)
    reproj_errors = np.linalg.norm(pts1_warped - pts2, axis=1)
    print(f"Mean reprojection error (inliers): {np.mean(reproj_errors[inlier_mask]):.3f} px")

    # Decompose
    R, t, n = decompose_homography(H, K)
    print(f"\nEstimated rotation:\n{R}")
    print(f"Estimated translation:\n{t.flatten()}")

    # Visualization
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Inliers and outliers
    axes[0, 0].scatter(pts1[inlier_mask, 0], pts1[inlier_mask, 1], c='green', s=15, label='Inliers')
    axes[0, 0].scatter(pts1[~inlier_mask, 0], pts1[~inlier_mask, 1], c='red', s=15, marker='x', label='Outliers')
    axes[0, 0].set_title(f'RANSAC: {n_inliers} inliers, {n_outliers} outliers')
    axes[0, 0].legend()
    axes[0, 0].set_xlim(0, 640)
    axes[0, 0].set_ylim(480, 0)

    # Correspondences with homography lines
    for i in range(0, len(pts1), 3):
        if inlier_mask[i]:
            axes[0, 1].plot([pts1[i, 0], pts2[i, 0]], [pts1[i, 1], pts2[i, 1]], 'g-', alpha=0.3)
    axes[0, 1].scatter(pts1[:, 0], pts1[:, 1], c='blue', s=5, label='Source')
    axes[0, 1].scatter(pts2[:, 0], pts2[:, 1], c='orange', s=5, label='Dest')
    axes[0, 1].set_title('Point Correspondences')
    axes[0, 1].legend()

    # Warped points vs actual
    axes[1, 0].scatter(pts2[inlier_mask, 0], pts2[inlier_mask, 1], c='blue', s=10, label='Actual')
    axes[1, 0].scatter(pts1_warped[inlier_mask, 0], pts1_warped[inlier_mask, 1],
                        c='red', s=5, marker='+', label='Warped')
    axes[1, 0].set_title('Warped vs Actual (Inliers)')
    axes[1, 0].legend()

    # Error histogram
    axes[1, 1].hist(reproj_errors[inlier_mask], bins=25, color='steelblue', edgecolor='black')
    axes[1, 1].axvline(np.mean(reproj_errors[inlier_mask]), color='red', linestyle='--',
                        label=f'Mean={np.mean(reproj_errors[inlier_mask]):.2f}')
    axes[1, 1].set_title('Reprojection Error (Inliers)')
    axes[1, 1].set_xlabel('Error (pixels)')
    axes[1, 1].legend()

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'task3_homography.png'), dpi=150)
    plt.close()
    print(f"\nPlot saved to {os.path.join(output_dir, 'task3_homography.png')}")


if __name__ == '__main__':
    main()
