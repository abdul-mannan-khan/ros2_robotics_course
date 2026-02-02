#!/usr/bin/env python3
"""
Generate sample 3D point cloud data for the LiDAR processing lab.

Creates a synthetic scene with:
- A ground plane
- Several box-shaped obstacles at known positions
- Noise points
- Saves as .npy and .pcd files

Usage:
    python3 generate_sample_data.py
"""

import numpy as np
import os


def generate_ground_plane(x_range=(-20, 40), y_range=(-10, 10), density=5.0, noise_std=0.02):
    """Generate ground plane points with slight noise."""
    num_x = int((x_range[1] - x_range[0]) * density)
    num_y = int((y_range[1] - y_range[0]) * density)
    x = np.random.uniform(x_range[0], x_range[1], num_x * num_y)
    y = np.random.uniform(y_range[0], y_range[1], num_x * num_y)
    z = np.random.normal(0.0, noise_std, num_x * num_y)  # ground at z=0 with noise
    return np.column_stack([x, y, z])


def generate_box_obstacle(center, size, num_points=500):
    """Generate points on the surface of a box obstacle.

    Args:
        center: (x, y, z) center of the box
        size: (sx, sy, sz) half-extents
        num_points: number of surface points
    """
    cx, cy, cz = center
    sx, sy, sz = size
    points = []

    pts_per_face = num_points // 6

    # Six faces of the box
    for axis in range(3):
        for sign in [-1, 1]:
            face_pts = np.random.uniform(-1, 1, (pts_per_face, 3))
            face_pts[:, axis] = sign
            face_pts[:, 0] *= sx
            face_pts[:, 1] *= sy
            face_pts[:, 2] *= sz
            face_pts[:, 0] += cx
            face_pts[:, 1] += cy
            face_pts[:, 2] += cz
            points.append(face_pts)

    return np.vstack(points)


def generate_noise(num_points=200, bounds=(-20, 40, -10, 10, -2, 5)):
    """Generate random noise points within bounds."""
    x = np.random.uniform(bounds[0], bounds[1], num_points)
    y = np.random.uniform(bounds[2], bounds[3], num_points)
    z = np.random.uniform(bounds[4], bounds[5], num_points)
    return np.column_stack([x, y, z])


def save_pcd(filename, points):
    """Save points as ASCII PCD file."""
    n = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {n}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n}
DATA ascii
"""
    with open(filename, 'w') as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def main():
    np.random.seed(42)

    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    os.makedirs(output_dir, exist_ok=True)

    # Generate ground
    ground = generate_ground_plane()
    print(f"Ground points: {len(ground)}")

    # Generate obstacles at known positions (ground truth)
    obstacles_config = [
        {"center": (5, 0, 1.0), "size": (1.0, 0.5, 1.0), "label": "car_1"},
        {"center": (10, 3, 0.75), "size": (0.5, 0.5, 0.75), "label": "pedestrian_1"},
        {"center": (15, -2, 1.5), "size": (2.0, 1.0, 1.5), "label": "truck_1"},
        {"center": (8, -5, 0.5), "size": (0.3, 0.3, 0.5), "label": "pole_1"},
        {"center": (20, 1, 1.0), "size": (1.0, 0.5, 1.0), "label": "car_2"},
    ]

    obstacle_points = []
    ground_truth = []

    for obs in obstacles_config:
        pts = generate_box_obstacle(obs["center"], obs["size"])
        obstacle_points.append(pts)
        gt = {
            "label": obs["label"],
            "center": obs["center"],
            "size": tuple(2 * s for s in obs["size"]),  # full extents
            "num_points": len(pts),
        }
        ground_truth.append(gt)
        print(f"Obstacle '{obs['label']}': {len(pts)} points at {obs['center']}")

    obstacles = np.vstack(obstacle_points)

    # Add noise
    noise = generate_noise(num_points=300)
    print(f"Noise points: {len(noise)}")

    # Combine all points
    all_points = np.vstack([ground, obstacles, noise])
    print(f"Total points: {len(all_points)}")

    # Shuffle points (students shouldn't rely on ordering)
    np.random.shuffle(all_points)

    # Save files
    np.save(os.path.join(output_dir, "sample_pointcloud.npy"), all_points)
    save_pcd(os.path.join(output_dir, "sample_pointcloud.pcd"), all_points)

    # Save ground truth for evaluation
    np.save(os.path.join(output_dir, "ground_truth.npy"), ground_truth, allow_pickle=True)

    # Save ground truth as readable text
    with open(os.path.join(output_dir, "ground_truth.txt"), 'w') as f:
        f.write("# Ground Truth Obstacles\n")
        f.write("# label, cx, cy, cz, sx, sy, sz\n")
        for gt in ground_truth:
            c = gt["center"]
            s = gt["size"]
            f.write(f"{gt['label']}, {c[0]}, {c[1]}, {c[2]}, {s[0]}, {s[1]}, {s[2]}\n")

    print(f"\nFiles saved to {output_dir}/")
    print("  - sample_pointcloud.npy")
    print("  - sample_pointcloud.pcd")
    print("  - ground_truth.npy")
    print("  - ground_truth.txt")


if __name__ == "__main__":
    main()
