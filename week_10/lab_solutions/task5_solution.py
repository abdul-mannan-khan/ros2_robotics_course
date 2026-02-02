#!/usr/bin/env python3
"""
Week 10 - Task 5 Solution: Stereo Depth Estimation
Block matching, disparity-to-depth, point cloud generation.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def compute_disparity(left, right, block_size=11, max_disparity=64):
    """Compute disparity using block matching with SAD."""
    h, w = left.shape
    half = block_size // 2
    disparity = np.zeros((h, w))

    for y in range(half, h - half):
        for x in range(half + max_disparity, w - half):
            left_block = left[y - half:y + half + 1, x - half:x + half + 1]
            best_d = 0
            best_sad = float('inf')
            for d in range(max_disparity):
                rx = x - d
                if rx - half < 0:
                    break
                right_block = right[y - half:y + half + 1, rx - half:rx + half + 1]
                sad = np.sum(np.abs(left_block - right_block))
                if sad < best_sad:
                    best_sad = sad
                    best_d = d
            disparity[y, x] = best_d

    return disparity


def disparity_to_depth(disparity, focal_length, baseline):
    """Convert disparity to depth map."""
    depth = np.zeros_like(disparity)
    valid = disparity > 0
    depth[valid] = (focal_length * baseline) / disparity[valid]
    return depth


def depth_to_pointcloud(depth, K):
    """Back-project depth map to 3D point cloud."""
    h, w = depth.shape
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    u, v = np.meshgrid(np.arange(w), np.arange(h))
    valid = depth > 0

    x = (u[valid] - cx) * depth[valid] / fx
    y = (v[valid] - cy) * depth[valid] / fy
    z = depth[valid]

    return np.column_stack([x, y, z])


def filter_depth(depth, min_depth=0.5, max_depth=100.0):
    """Filter invalid depth values."""
    filtered = depth.copy()
    invalid = (depth <= min_depth) | (depth >= max_depth) | np.isnan(depth) | np.isinf(depth)
    filtered[invalid] = 0
    return filtered


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, '..', 'lab_exercises', 'data')
    output_dir = script_dir

    print("Week 10 - Task 5 Solution: Stereo Depth Estimation")
    print("=" * 52)

    left = np.load(os.path.join(data_dir, 'stereo_left.npy'))
    right = np.load(os.path.join(data_dir, 'stereo_right.npy'))
    stereo_params = np.load(os.path.join(data_dir, 'stereo_params.npy'))
    K = np.load(os.path.join(data_dir, 'camera_intrinsics.npy'))

    baseline, focal_length = stereo_params
    print(f"Baseline: {baseline:.3f} m, Focal length: {focal_length:.1f} px")
    print(f"Image shape: {left.shape}")

    # Use subsampled image for speed (every 4th pixel)
    step = 4
    left_sub = left[::step, ::step]
    right_sub = right[::step, ::step]
    print(f"Subsampled to {left_sub.shape} for block matching...")

    disparity_sub = compute_disparity(left_sub, right_sub, block_size=7, max_disparity=32)

    # Scale disparity back
    from scipy.ndimage import zoom
    disparity = zoom(disparity_sub, step, order=1) * step
    disparity = disparity[:left.shape[0], :left.shape[1]]

    print(f"Disparity range: [{disparity[disparity > 0].min():.1f}, {disparity.max():.1f}]")

    # Convert to depth
    depth = disparity_to_depth(disparity, focal_length, baseline)
    depth = filter_depth(depth, min_depth=0.1, max_depth=80.0)
    valid_depth = depth[depth > 0]
    print(f"Depth range: [{valid_depth.min():.2f}, {valid_depth.max():.2f}] m")

    # Point cloud
    pc = depth_to_pointcloud(depth, K)
    print(f"Point cloud: {len(pc)} points")

    # Visualization
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    axes[0, 0].imshow(left, cmap='gray')
    axes[0, 0].set_title('Left Image')

    axes[0, 1].imshow(right, cmap='gray')
    axes[0, 1].set_title('Right Image')

    im = axes[0, 2].imshow(disparity, cmap='jet')
    axes[0, 2].set_title('Disparity Map')
    plt.colorbar(im, ax=axes[0, 2], fraction=0.046)

    im2 = axes[1, 0].imshow(depth, cmap='plasma')
    axes[1, 0].set_title('Depth Map (m)')
    plt.colorbar(im2, ax=axes[1, 0], fraction=0.046)

    # Point cloud top view (X-Z)
    if len(pc) > 0:
        subsample = np.random.choice(len(pc), min(5000, len(pc)), replace=False)
        axes[1, 1].scatter(pc[subsample, 0], pc[subsample, 2], c=pc[subsample, 2],
                           cmap='viridis', s=1)
        axes[1, 1].set_title('Point Cloud (top view)')
        axes[1, 1].set_xlabel('X (m)')
        axes[1, 1].set_ylabel('Z (m)')

    # Depth histogram
    axes[1, 2].hist(valid_depth, bins=50, color='steelblue', edgecolor='black')
    axes[1, 2].set_title('Depth Distribution')
    axes[1, 2].set_xlabel('Depth (m)')
    axes[1, 2].set_ylabel('Count')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'task5_stereo_depth.png'), dpi=150)
    plt.close()
    print(f"\nPlot saved to {os.path.join(output_dir, 'task5_stereo_depth.png')}")


if __name__ == '__main__':
    main()
