#!/usr/bin/env python3
"""
Week 10 - Task 2 Solution: Feature Detection and Matching
Harris corner detector, patch descriptors, brute-force matching with ratio test.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


def harris_corner_response(image, k=0.04, window_size=5):
    """Compute Harris corner response using structure tensor."""
    # Compute gradients using finite differences
    Iy, Ix = np.gradient(image)

    # Structure tensor components
    Ixx = gaussian_filter(Ix * Ix, sigma=window_size / 3.0)
    Iyy = gaussian_filter(Iy * Iy, sigma=window_size / 3.0)
    Ixy = gaussian_filter(Ix * Iy, sigma=window_size / 3.0)

    # Harris response: det(M) - k * trace(M)^2
    det_M = Ixx * Iyy - Ixy * Ixy
    trace_M = Ixx + Iyy
    response = det_M - k * trace_M * trace_M

    return response


def non_maximum_suppression(response, threshold, radius=5):
    """Non-maximum suppression on response map."""
    h, w = response.shape
    corners = []
    # Threshold
    mask = response > threshold

    # Get candidates sorted by response strength
    ys, xs = np.where(mask)
    values = response[mask]
    order = np.argsort(-values)

    suppressed = np.zeros_like(response, dtype=bool)
    for idx in order:
        y, x = ys[idx], xs[idx]
        if suppressed[y, x]:
            continue
        corners.append([x, y])
        # Suppress neighborhood
        y_min = max(0, y - radius)
        y_max = min(h, y + radius + 1)
        x_min = max(0, x - radius)
        x_max = min(w, x + radius + 1)
        suppressed[y_min:y_max, x_min:x_max] = True

    return np.array(corners) if corners else np.empty((0, 2))


def compute_simple_descriptor(image, keypoint, patch_size=16):
    """Compute normalized patch descriptor."""
    x, y = int(round(keypoint[0])), int(round(keypoint[1]))
    half = patch_size // 2
    h, w = image.shape

    if y - half < 0 or y + half >= h or x - half < 0 or x + half >= w:
        return None

    patch = image[y - half:y + half, x - half:x + half].flatten().astype(np.float64)
    mean = np.mean(patch)
    std = np.std(patch)
    if std < 1e-8:
        return None
    return (patch - mean) / std


def match_features(desc1, desc2, ratio_threshold=0.75):
    """Brute-force matching with Lowe's ratio test."""
    matches = []
    for i, d1 in enumerate(desc1):
        # Compute distances to all descriptors in desc2
        dists = np.linalg.norm(desc2 - d1, axis=1)
        if len(dists) < 2:
            continue
        # Two nearest
        idx = np.argsort(dists)
        best, second = dists[idx[0]], dists[idx[1]]
        if second > 1e-8 and best / second < ratio_threshold:
            matches.append((i, idx[0]))
    return matches


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, '..', 'lab_exercises', 'data')
    output_dir = script_dir

    print("Week 10 - Task 2 Solution: Feature Detection and Matching")
    print("=" * 58)

    frame1 = np.load(os.path.join(data_dir, 'optical_flow_frame1.npy'))
    frame2 = np.load(os.path.join(data_dir, 'optical_flow_frame2.npy'))
    print(f"Image shapes: {frame1.shape}, {frame2.shape}")

    # Harris corners
    resp1 = harris_corner_response(frame1, k=0.04, window_size=5)
    resp2 = harris_corner_response(frame2, k=0.04, window_size=5)

    threshold1 = np.percentile(resp1[resp1 > 0], 95) if np.any(resp1 > 0) else 0.001
    threshold2 = np.percentile(resp2[resp2 > 0], 95) if np.any(resp2 > 0) else 0.001

    kp1 = non_maximum_suppression(resp1, threshold1, radius=10)
    kp2 = non_maximum_suppression(resp2, threshold2, radius=10)
    print(f"Detected corners: image1={len(kp1)}, image2={len(kp2)}")

    # Compute descriptors
    descs1, valid_kp1 = [], []
    for kp in kp1:
        d = compute_simple_descriptor(frame1, kp, patch_size=16)
        if d is not None:
            descs1.append(d)
            valid_kp1.append(kp)
    descs1 = np.array(descs1) if descs1 else np.empty((0, 256))
    valid_kp1 = np.array(valid_kp1) if valid_kp1 else np.empty((0, 2))

    descs2, valid_kp2 = [], []
    for kp in kp2:
        d = compute_simple_descriptor(frame2, kp, patch_size=16)
        if d is not None:
            descs2.append(d)
            valid_kp2.append(kp)
    descs2 = np.array(descs2) if descs2 else np.empty((0, 256))
    valid_kp2 = np.array(valid_kp2) if valid_kp2 else np.empty((0, 2))

    print(f"Valid descriptors: image1={len(descs1)}, image2={len(descs2)}")

    # Match
    matches = match_features(descs1, descs2, ratio_threshold=0.8)
    print(f"Matches found: {len(matches)}")

    # Visualization
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Harris response
    axes[0, 0].imshow(resp1, cmap='hot')
    axes[0, 0].set_title('Harris Response - Image 1')

    # Corners on image
    axes[0, 1].imshow(frame1, cmap='gray')
    if len(valid_kp1) > 0:
        axes[0, 1].scatter(valid_kp1[:, 0], valid_kp1[:, 1], c='lime', s=10, marker='+')
    axes[0, 1].set_title(f'Detected Corners ({len(valid_kp1)})')

    # Matches visualization - side by side
    combined_w = frame1.shape[1] + frame2.shape[1]
    combined = np.zeros((max(frame1.shape[0], frame2.shape[0]), combined_w))
    combined[:frame1.shape[0], :frame1.shape[1]] = frame1
    combined[:frame2.shape[0], frame1.shape[1]:] = frame2
    axes[1, 0].imshow(combined, cmap='gray')
    offset_x = frame1.shape[1]
    for i1, i2 in matches[:50]:
        p1 = valid_kp1[i1]
        p2 = valid_kp2[i2]
        axes[1, 0].plot([p1[0], p2[0] + offset_x], [p1[1], p2[1]], 'c-', linewidth=0.5)
    axes[1, 0].set_title(f'Feature Matches ({len(matches)})')

    # Match distance histogram
    if matches:
        dists = [np.linalg.norm(valid_kp1[i1] - valid_kp2[i2]) for i1, i2 in matches]
        axes[1, 1].hist(dists, bins=20, color='steelblue', edgecolor='black')
        axes[1, 1].set_title('Match Distance Distribution')
        axes[1, 1].set_xlabel('Distance (pixels)')
        axes[1, 1].set_ylabel('Count')
    else:
        axes[1, 1].text(0.5, 0.5, 'No matches', ha='center', va='center', transform=axes[1, 1].transAxes)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'task2_feature_detection.png'), dpi=150)
    plt.close()
    print(f"\nPlot saved to {os.path.join(output_dir, 'task2_feature_detection.png')}")


if __name__ == '__main__':
    main()
