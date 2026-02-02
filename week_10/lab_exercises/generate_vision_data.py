#!/usr/bin/env python3
"""
Week 10: Computer Vision for Drones - Synthetic Data Generator
Generates all required data files using only numpy and matplotlib.
"""

import os
import numpy as np

# matplotlib with Agg backend
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def generate_camera_intrinsics():
    """Generate synthetic camera intrinsic matrix and distortion coefficients."""
    K = np.array([
        [500.0, 0.0, 320.0],
        [0.0, 500.0, 240.0],
        [0.0, 0.0, 1.0]
    ])
    dist_coeffs = np.array([-0.1, 0.01, 0.001, -0.001, 0.0005])
    return K, dist_coeffs


def generate_checkerboard_corners(rows=7, cols=9, square_size=30.0):
    """Generate simulated checkerboard corner positions in image coordinates."""
    corners = []
    offset_x, offset_y = 100.0, 60.0
    for r in range(rows):
        for c in range(cols):
            x = offset_x + c * square_size + np.random.randn() * 0.3
            y = offset_y + r * square_size + np.random.randn() * 0.3
            corners.append([x, y])
    return np.array(corners)


def generate_scene_objects():
    """Generate simple 2D scene with geometric shapes as structured array."""
    # Each object: (type_id, cx, cy, width, height, label_id)
    # type: 0=rectangle, 1=circle, 2=line
    objects = np.array([
        [0, 150, 120, 80, 60, 0],   # rectangle - "building"
        [0, 400, 200, 100, 80, 0],   # rectangle - "building"
        [1, 300, 350, 40, 40, 1],    # circle - "tree"
        [1, 500, 100, 30, 30, 1],    # circle - "tree"
        [0, 100, 400, 60, 40, 2],    # rectangle - "car"
        [0, 450, 380, 70, 35, 2],    # rectangle - "car"
        [2, 50, 240, 590, 240, 3],   # line - "road"
        [1, 250, 150, 25, 25, 1],    # circle - "tree"
    ], dtype=np.float64)
    return objects


def generate_feature_points(n_points=150):
    """Generate matched feature point pairs with known transformation."""
    np.random.seed(42)
    pts1 = np.column_stack([
        np.random.uniform(50, 590, n_points),
        np.random.uniform(50, 430, n_points)
    ])

    # Apply known homography-like transform + noise
    angle = np.deg2rad(5)
    tx, ty = 15.0, -10.0
    scale = 1.02
    R = scale * np.array([[np.cos(angle), -np.sin(angle)],
                           [np.sin(angle),  np.cos(angle)]])
    pts2 = (R @ pts1.T).T + np.array([tx, ty])
    pts2 += np.random.randn(n_points, 2) * 0.5

    # Add some outliers (10%)
    n_outliers = n_points // 10
    outlier_idx = np.random.choice(n_points, n_outliers, replace=False)
    pts2[outlier_idx] = np.column_stack([
        np.random.uniform(50, 590, n_outliers),
        np.random.uniform(50, 430, n_outliers)
    ])

    return pts1, pts2


def generate_feature_descriptors(n_points=150, dim=128):
    """Generate synthetic feature descriptors."""
    np.random.seed(42)
    desc1 = np.random.randn(n_points, dim).astype(np.float32)
    # Normalize
    desc1 = desc1 / (np.linalg.norm(desc1, axis=1, keepdims=True) + 1e-8)

    # desc2: similar to desc1 with noise (matched), outliers get random
    desc2 = desc1 + np.random.randn(n_points, dim).astype(np.float32) * 0.1
    desc2 = desc2 / (np.linalg.norm(desc2, axis=1, keepdims=True) + 1e-8)

    n_outliers = n_points // 10
    outlier_idx = np.random.choice(n_points, n_outliers, replace=False)
    desc2[outlier_idx] = np.random.randn(n_outliers, dim).astype(np.float32)
    desc2[outlier_idx] = desc2[outlier_idx] / (np.linalg.norm(desc2[outlier_idx], axis=1, keepdims=True) + 1e-8)

    return desc1, desc2


def generate_stereo_pair(width=640, height=480):
    """Generate synthetic stereo image pair with known disparity."""
    np.random.seed(123)

    # Create a simple scene with depth layers
    left = np.zeros((height, width), dtype=np.float64)
    depth_map = np.ones((height, width)) * 50.0  # far background

    # Background gradient
    for y in range(height):
        left[y, :] = 0.3 + 0.2 * (y / height)

    # Add objects at different depths
    rects = [
        (100, 80, 180, 200, 0.8, 10.0),
        (300, 150, 420, 300, 0.6, 20.0),
        (450, 250, 550, 400, 0.9, 5.0),
        (200, 300, 350, 420, 0.5, 15.0),
    ]
    for x1, y1, x2, y2, intensity, depth in rects:
        left[y1:y2, x1:x2] = intensity
        depth_map[y1:y2, x1:x2] = depth

    # Add texture noise
    left += np.random.randn(height, width) * 0.02

    # Stereo parameters
    baseline = 0.12  # 12cm
    focal_length = 500.0

    # Compute disparity from depth
    disparity = (focal_length * baseline) / depth_map

    # Create right image by shifting pixels according to disparity
    right = np.zeros_like(left)
    for y in range(height):
        for x in range(width):
            d = int(round(disparity[y, x]))
            src_x = x + d
            if 0 <= src_x < width:
                right[y, x] = left[y, src_x]

    stereo_params = np.array([baseline, focal_length])
    return left, right, stereo_params


def generate_drone_trajectory(n_poses=50):
    """Generate smooth drone trajectory: x,y,z,roll,pitch,yaw."""
    t = np.linspace(0, 2 * np.pi, n_poses)
    x = 10.0 * np.cos(t)
    y = 10.0 * np.sin(t)
    z = 5.0 + 2.0 * np.sin(2 * t)
    roll = 0.1 * np.sin(t)
    pitch = 0.1 * np.cos(t)
    yaw = t + np.pi / 2  # heading tangent to circle
    poses = np.column_stack([x, y, z, roll, pitch, yaw])
    return poses


def generate_optical_flow_frames(width=640, height=480):
    """Generate two frames with known displacement for optical flow."""
    np.random.seed(99)

    # Frame 1: random textured image with blobs
    frame1 = np.random.rand(height, width) * 0.3

    # Add Gaussian blobs
    for _ in range(30):
        cx = np.random.randint(40, width - 40)
        cy = np.random.randint(40, height - 40)
        sigma = np.random.uniform(10, 30)
        intensity = np.random.uniform(0.3, 1.0)
        yy, xx = np.mgrid[0:height, 0:width]
        blob = intensity * np.exp(-((xx - cx)**2 + (yy - cy)**2) / (2 * sigma**2))
        frame1 += blob

    frame1 = np.clip(frame1, 0, 1)

    # Known global flow: dx=3, dy=2 with slight variation
    dx_global, dy_global = 3.0, 2.0
    gt_flow = np.zeros((height, width, 2))
    gt_flow[:, :, 0] = dx_global + 0.5 * np.sin(np.linspace(0, np.pi, width))[None, :]
    gt_flow[:, :, 1] = dy_global + 0.3 * np.cos(np.linspace(0, np.pi, height))[:, None]

    # Warp frame1 to create frame2 using integer shifts for simplicity
    frame2 = np.zeros_like(frame1)
    for y in range(height):
        for x in range(width):
            sx = int(round(x - gt_flow[y, x, 0]))
            sy = int(round(y - gt_flow[y, x, 1]))
            if 0 <= sx < width and 0 <= sy < height:
                frame2[y, x] = frame1[sy, sx]

    return frame1, frame2, gt_flow


def generate_marker_image(width=640, height=480):
    """Generate image with ArUco-like markers (simple black/white grid patterns)."""
    image = np.ones((height, width)) * 0.8  # light gray background

    marker_corners_list = []

    def draw_marker(img, cx, cy, size, marker_id):
        """Draw a simple 5x5 ArUco-like marker."""
        np.random.seed(marker_id)
        half = size // 2
        x0, y0 = cx - half, cy - half

        # Black border
        cell = size // 7
        for r in range(7):
            for c in range(7):
                px, py = x0 + c * cell, y0 + r * cell
                if r == 0 or r == 6 or c == 0 or c == 6:
                    # Border - black
                    img[py:py+cell, px:px+cell] = 0.0
                else:
                    # Inner 5x5 pattern
                    bit = np.random.randint(0, 2)
                    img[py:py+cell, px:px+cell] = float(bit)

        corners = np.array([
            [x0, y0],
            [x0 + size, y0],
            [x0 + size, y0 + size],
            [x0, y0 + size]
        ], dtype=np.float64)
        return corners

    positions = [(160, 120, 84, 0), (480, 120, 84, 1),
                 (160, 360, 84, 2), (480, 360, 84, 3)]

    for cx, cy, sz, mid in positions:
        c = draw_marker(image, cx, cy, sz, mid)
        marker_corners_list.append(c)

    marker_corners = np.array(marker_corners_list)  # shape (4, 4, 2)
    return image, marker_corners


def main():
    """Generate all synthetic vision data and save to data directory."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, 'data')
    os.makedirs(data_dir, exist_ok=True)

    print("Generating camera intrinsics...")
    K, dist_coeffs = generate_camera_intrinsics()
    np.save(os.path.join(data_dir, 'camera_intrinsics.npy'), K)
    np.save(os.path.join(data_dir, 'distortion_coeffs.npy'), dist_coeffs)

    print("Generating checkerboard corners...")
    corners = generate_checkerboard_corners()
    np.save(os.path.join(data_dir, 'checkerboard_corners.npy'), corners)

    print("Generating scene objects...")
    objects = generate_scene_objects()
    np.save(os.path.join(data_dir, 'scene_objects.npy'), objects)

    print("Generating feature points and descriptors...")
    pts1, pts2 = generate_feature_points()
    desc1, desc2 = generate_feature_descriptors()
    np.save(os.path.join(data_dir, 'feature_points_1.npy'), pts1)
    np.save(os.path.join(data_dir, 'feature_points_2.npy'), pts2)
    np.save(os.path.join(data_dir, 'feature_descriptors_1.npy'), desc1)
    np.save(os.path.join(data_dir, 'feature_descriptors_2.npy'), desc2)

    print("Generating stereo pair...")
    left, right, stereo_params = generate_stereo_pair()
    np.save(os.path.join(data_dir, 'stereo_left.npy'), left)
    np.save(os.path.join(data_dir, 'stereo_right.npy'), right)
    np.save(os.path.join(data_dir, 'stereo_params.npy'), stereo_params)

    print("Generating drone trajectory...")
    poses = generate_drone_trajectory()
    np.save(os.path.join(data_dir, 'drone_poses.npy'), poses)

    print("Generating optical flow frames...")
    f1, f2, gt_flow = generate_optical_flow_frames()
    np.save(os.path.join(data_dir, 'optical_flow_frame1.npy'), f1)
    np.save(os.path.join(data_dir, 'optical_flow_frame2.npy'), f2)
    np.save(os.path.join(data_dir, 'optical_flow_gt.npy'), gt_flow)

    print("Generating marker image...")
    marker_img, marker_corners = generate_marker_image()
    np.save(os.path.join(data_dir, 'marker_image.npy'), marker_img)
    np.save(os.path.join(data_dir, 'marker_corners_gt.npy'), marker_corners)

    print(f"All data saved to {data_dir}")
    print("Files generated:")
    for f in sorted(os.listdir(data_dir)):
        fpath = os.path.join(data_dir, f)
        size = os.path.getsize(fpath)
        print(f"  {f}: {size:,} bytes")


if __name__ == '__main__':
    main()
