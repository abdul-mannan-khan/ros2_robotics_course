#!/usr/bin/env python3
"""
Week 10 - Task 6 Solution: Optical Flow Estimation
Lucas-Kanade sparse optical flow, motion estimation.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


def compute_image_gradients(image):
    """Compute spatial gradients using finite differences."""
    Iy, Ix = np.gradient(image.astype(np.float64))
    return Ix, Iy


def lucas_kanade(frame1, frame2, points, window_size=15):
    """Lucas-Kanade sparse optical flow solving 2x2 system per point."""
    half = window_size // 2
    h, w = frame1.shape

    # Smooth images slightly
    f1 = gaussian_filter(frame1.astype(np.float64), sigma=1.0)
    f2 = gaussian_filter(frame2.astype(np.float64), sigma=1.0)

    # Spatial gradients on frame1
    Ix, Iy = compute_image_gradients(f1)
    # Temporal gradient
    It = f2 - f1

    flow = np.zeros((len(points), 2))
    status = np.zeros(len(points), dtype=bool)

    for i, (px, py) in enumerate(points):
        x, y = int(round(px)), int(round(py))
        if y - half < 0 or y + half >= h or x - half < 0 or x + half >= w:
            continue

        # Extract window
        ix = Ix[y - half:y + half + 1, x - half:x + half + 1].flatten()
        iy = Iy[y - half:y + half + 1, x - half:x + half + 1].flatten()
        it = It[y - half:y + half + 1, x - half:x + half + 1].flatten()

        # Build A^T A and A^T b
        ATA = np.array([
            [np.sum(ix * ix), np.sum(ix * iy)],
            [np.sum(ix * iy), np.sum(iy * iy)]
        ])
        ATb = -np.array([np.sum(ix * it), np.sum(iy * it)])

        # Check eigenvalues for reliability
        eigenvalues = np.linalg.eigvalsh(ATA)
        min_eig = np.min(eigenvalues)
        if min_eig < 1e-6:
            continue

        # Solve 2x2 system
        v = np.linalg.solve(ATA, ATb)
        flow[i] = v
        status[i] = True

    return flow, status


def flow_to_motion(flow_vectors, K, depth=1.0):
    """Estimate camera translation from median flow."""
    median_flow = np.median(flow_vectors, axis=0)
    fx, fy = K[0, 0], K[1, 1]
    tx = -median_flow[0] * depth / fx
    ty = -median_flow[1] * depth / fy
    tz = 0.0  # Cannot estimate from pure translation flow without more info
    return np.array([tx, ty, tz])


def visualize_flow(image, points, flow, status=None, output_path=None):
    """Visualize optical flow as arrows."""
    fig, ax = plt.subplots(1, 1, figsize=(10, 7))
    ax.imshow(image, cmap='gray')

    valid = status if status is not None else np.ones(len(points), dtype=bool)
    pts = points[valid]
    fl = flow[valid]

    # Color by flow magnitude
    mag = np.sqrt(fl[:, 0] ** 2 + fl[:, 1] ** 2)
    ax.quiver(pts[:, 0], pts[:, 1], fl[:, 0], fl[:, 1],
              mag, cmap='jet', scale=50, width=0.003)
    ax.set_title(f'Optical Flow ({np.sum(valid)} tracked points)')

    if output_path:
        plt.savefig(output_path, dpi=150)
    plt.close()


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, '..', 'lab_exercises', 'data')
    output_dir = script_dir

    print("Week 10 - Task 6 Solution: Optical Flow Estimation")
    print("=" * 52)

    frame1 = np.load(os.path.join(data_dir, 'optical_flow_frame1.npy'))
    frame2 = np.load(os.path.join(data_dir, 'optical_flow_frame2.npy'))
    gt_flow = np.load(os.path.join(data_dir, 'optical_flow_gt.npy'))
    K = np.load(os.path.join(data_dir, 'camera_intrinsics.npy'))

    print(f"Frame shape: {frame1.shape}")

    # Generate tracking points on a grid
    step = 20
    xs = np.arange(30, frame1.shape[1] - 30, step)
    ys = np.arange(30, frame1.shape[0] - 30, step)
    xx, yy = np.meshgrid(xs, ys)
    points = np.column_stack([xx.ravel(), yy.ravel()]).astype(np.float64)
    print(f"Tracking {len(points)} points...")

    # Compute Lucas-Kanade flow
    flow, status = lucas_kanade(frame1, frame2, points, window_size=21)
    n_tracked = np.sum(status)
    print(f"Successfully tracked: {n_tracked}/{len(points)}")

    # Compare with ground truth
    gt_at_pts = np.zeros((len(points), 2))
    for i, (px, py) in enumerate(points):
        x, y = int(round(px)), int(round(py))
        if 0 <= y < gt_flow.shape[0] and 0 <= x < gt_flow.shape[1]:
            gt_at_pts[i] = gt_flow[y, x]

    valid = status
    errors = np.linalg.norm(flow[valid] - gt_at_pts[valid], axis=1)
    print(f"Mean flow error: {np.mean(errors):.4f} pixels")
    print(f"Median flow error: {np.median(errors):.4f} pixels")

    # Estimate motion
    translation = flow_to_motion(flow[valid], K)
    print(f"Estimated translation: {translation}")

    # Visualization
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    axes[0, 0].imshow(frame1, cmap='gray')
    axes[0, 0].set_title('Frame 1')

    axes[0, 1].imshow(frame2, cmap='gray')
    axes[0, 1].set_title('Frame 2')

    # Estimated flow
    pts_v = points[valid]
    fl_v = flow[valid]
    mag = np.sqrt(fl_v[:, 0] ** 2 + fl_v[:, 1] ** 2)
    axes[0, 2].imshow(frame1, cmap='gray')
    axes[0, 2].quiver(pts_v[:, 0], pts_v[:, 1], fl_v[:, 0], fl_v[:, 1],
                       mag, cmap='jet', scale=80, width=0.003)
    axes[0, 2].set_title(f'Estimated Flow ({n_tracked} pts)')

    # Ground truth flow field (subsampled)
    s = 15
    yg, xg = np.mgrid[0:frame1.shape[0]:s, 0:frame1.shape[1]:s]
    axes[1, 0].imshow(frame1, cmap='gray')
    axes[1, 0].quiver(xg, yg, gt_flow[::s, ::s, 0], gt_flow[::s, ::s, 1],
                       color='cyan', scale=80, width=0.003)
    axes[1, 0].set_title('Ground Truth Flow')

    # Error map
    error_img = np.zeros(frame1.shape)
    for i, (px, py) in enumerate(points):
        if valid[i]:
            x, y = int(round(px)), int(round(py))
            if 0 <= y < error_img.shape[0] and 0 <= x < error_img.shape[1]:
                error_img[y, x] = errors[np.sum(valid[:i + 1]) - 1] if np.sum(valid[:i + 1]) > 0 else 0
    axes[1, 1].imshow(frame1, cmap='gray', alpha=0.5)
    pts_err = points[valid]
    sc = axes[1, 1].scatter(pts_err[:, 0], pts_err[:, 1], c=errors, cmap='hot', s=10)
    plt.colorbar(sc, ax=axes[1, 1], fraction=0.046)
    axes[1, 1].set_title('Flow Error per Point')

    # Error histogram
    axes[1, 2].hist(errors, bins=30, color='steelblue', edgecolor='black')
    axes[1, 2].axvline(np.mean(errors), color='red', linestyle='--',
                        label=f'Mean={np.mean(errors):.3f}')
    axes[1, 2].set_title('Flow Error Distribution')
    axes[1, 2].set_xlabel('Error (pixels)')
    axes[1, 2].legend()

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'task6_optical_flow.png'), dpi=150)
    plt.close()
    print(f"\nPlot saved to {os.path.join(output_dir, 'task6_optical_flow.png')}")


if __name__ == '__main__':
    main()
