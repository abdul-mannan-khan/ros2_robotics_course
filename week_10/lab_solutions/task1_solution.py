#!/usr/bin/env python3
"""
Week 10 - Task 1 Solution: Camera Projection and Calibration
Complete implementation of camera model, undistortion, DLT calibration.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def project_point(K, R, t, point_3d):
    """Project a 3D point onto the image plane."""
    p_cam = R @ point_3d + t.flatten()
    p_img = K @ p_cam
    return p_img[:2] / p_img[2]


def undistort_point(point, K, dist_coeffs):
    """Remove radial and tangential distortion from an image point."""
    k1, k2, p1, p2, k3 = dist_coeffs
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # Convert to normalized coordinates
    x = (point[0] - cx) / fx
    y = (point[1] - cy) / fy

    # Iterative undistortion
    x0, y0 = x, y
    for _ in range(10):
        r2 = x * x + y * y
        r4 = r2 * r2
        r6 = r4 * r2
        radial = 1 + k1 * r2 + k2 * r4 + k3 * r6
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x = (x0 - dx) / radial
        y = (y0 - dy) / radial

    return np.array([x * fx + cx, y * fy + cy])


def calibrate_camera(object_points, image_points):
    """Estimate camera projection matrix using DLT, decompose into K, R, t."""
    n = len(object_points)
    A = np.zeros((2 * n, 12))

    for i in range(n):
        X, Y, Z = object_points[i]
        u, v = image_points[i]
        A[2 * i] = [X, Y, Z, 1, 0, 0, 0, 0, -u * X, -u * Y, -u * Z, -u]
        A[2 * i + 1] = [0, 0, 0, 0, X, Y, Z, 1, -v * X, -v * Y, -v * Z, -v]

    _, _, Vt = np.linalg.svd(A)
    P = Vt[-1].reshape(3, 4)

    # Decompose P = K[R|t] using RQ decomposition of M = P[:,:3]
    M = P[:, :3]

    # RQ decomposition: M = K R where K upper triangular, R rotation
    # Use scipy for QR on M^{-T} then invert
    from scipy.linalg import rq
    K, R = rq(M)

    # Ensure positive diagonal for K
    D = np.diag(np.sign(np.diag(K)))
    K = K @ D
    R = D @ R

    # Normalize so K[2,2] = 1
    K = K / K[2, 2]

    # Extract t: P[:,3] = K t  =>  t = K^{-1} P[:,3]
    t = np.linalg.solve(K, P[:, 3])

    # Ensure proper rotation (det(R) = 1)
    if np.linalg.det(R) < 0:
        R = -R
        t = -t

    return K, R, t.reshape(3, 1)


def reprojection_error(K, R, t, points_3d, points_2d):
    """Compute mean reprojection error."""
    errors = []
    for p3d, p2d in zip(points_3d, points_2d):
        proj = project_point(K, R, t, p3d)
        errors.append(np.linalg.norm(proj - p2d))
    return np.mean(errors)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, '..', 'lab_exercises', 'data')
    output_dir = script_dir

    # Load data
    K_true = np.load(os.path.join(data_dir, 'camera_intrinsics.npy'))
    dist_coeffs = np.load(os.path.join(data_dir, 'distortion_coeffs.npy'))

    print("Week 10 - Task 1 Solution: Camera Projection and Calibration")
    print("=" * 60)
    print(f"True K:\n{K_true}")

    # Generate 3D test points (non-planar for full calibration)
    np.random.seed(7)
    obj_pts = []
    for r in range(7):
        for c in range(9):
            obj_pts.append([c * 0.03 - 0.12, r * 0.03 - 0.09, np.random.uniform(0.0, 0.05)])
    obj_pts = np.array(obj_pts)

    # True camera pose
    R_true = np.eye(3)
    angle = np.deg2rad(10)
    R_true = np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])
    t_true = np.array([[0.0], [-0.05], [0.5]])

    # Project points
    img_pts_clean = np.array([project_point(K_true, R_true, t_true, p) for p in obj_pts])
    img_pts_noisy = img_pts_clean + np.random.randn(*img_pts_clean.shape) * 0.5

    print(f"\nGenerated {len(obj_pts)} 3D-2D correspondences")

    # Calibrate
    K_est, R_est, t_est = calibrate_camera(obj_pts, img_pts_noisy)
    print(f"\nEstimated K:\n{K_est}")
    print(f"\nTrue R:\n{R_true}")
    print(f"Estimated R:\n{R_est}")

    # Reprojection error
    err = reprojection_error(K_est, R_est, t_est, obj_pts, img_pts_noisy)
    print(f"\nMean reprojection error: {err:.4f} pixels")

    # Undistortion demo
    test_pt = np.array([350.0, 260.0])
    undist = undistort_point(test_pt, K_true, dist_coeffs)
    print(f"\nDistorted point: {test_pt}")
    print(f"Undistorted point: {undist}")

    # Plot
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # Panel 1: Projected points
    axes[0].scatter(img_pts_clean[:, 0], img_pts_clean[:, 1], c='blue', s=20, label='Clean')
    axes[0].scatter(img_pts_noisy[:, 0], img_pts_noisy[:, 1], c='red', s=10, marker='x', label='Noisy')
    axes[0].set_title('Projected Points')
    axes[0].set_xlabel('u (pixels)')
    axes[0].set_ylabel('v (pixels)')
    axes[0].legend()
    axes[0].invert_yaxis()
    axes[0].set_xlim(0, 640)
    axes[0].set_ylim(480, 0)

    # Panel 2: Reprojected vs observed
    reproj = np.array([project_point(K_est, R_est, t_est, p) for p in obj_pts])
    axes[1].scatter(img_pts_noisy[:, 0], img_pts_noisy[:, 1], c='red', s=20, label='Observed')
    axes[1].scatter(reproj[:, 0], reproj[:, 1], c='green', s=10, marker='+', label='Reprojected')
    axes[1].set_title(f'Reprojection (error={err:.2f}px)')
    axes[1].set_xlabel('u (pixels)')
    axes[1].set_ylabel('v (pixels)')
    axes[1].legend()
    axes[1].invert_yaxis()

    # Panel 3: Error histogram
    errors = [np.linalg.norm(project_point(K_est, R_est, t_est, p) - q)
              for p, q in zip(obj_pts, img_pts_noisy)]
    axes[2].hist(errors, bins=20, color='steelblue', edgecolor='black')
    axes[2].set_title('Reprojection Error Distribution')
    axes[2].set_xlabel('Error (pixels)')
    axes[2].set_ylabel('Count')
    axes[2].axvline(np.mean(errors), color='red', linestyle='--', label=f'Mean={np.mean(errors):.2f}')
    axes[2].legend()

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'task1_camera_model.png'), dpi=150)
    plt.close()
    print(f"\nPlot saved to {os.path.join(output_dir, 'task1_camera_model.png')}")


if __name__ == '__main__':
    main()
