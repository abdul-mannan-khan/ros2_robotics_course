#!/usr/bin/env python3
"""
Task 6: Full Pipeline - SOLUTION
==================================
INSTRUCTOR VERSION - DO NOT DISTRIBUTE TO STUDENTS
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os
import sys

# Add solutions directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from task1_load_and_visualize_solution import load_point_cloud_npy
from task2_voxel_downsampling_solution import voxel_grid_downsample
from task3_ransac_ground_removal_solution import ransac_plane_fit, segment_ground
from task4_euclidean_clustering_solution import euclidean_clustering, get_cluster_properties
from task5_bounding_boxes_solution import compute_aabb, compute_obb_pca, compute_iou_aabb, match_detections_to_ground_truth

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")


def run_pipeline(points: np.ndarray, config: dict) -> dict:
    """Run the complete LiDAR obstacle detection pipeline."""
    # Step 1: Voxel grid downsampling
    downsampled = voxel_grid_downsample(points, config['voxel_size'])

    # Step 2: RANSAC ground removal
    plane, inlier_mask = ransac_plane_fit(
        downsampled,
        max_iterations=config['ransac_iterations'],
        distance_threshold=config['ransac_threshold']
    )
    ground, obstacles = segment_ground(downsampled, plane, inlier_mask)

    # Step 3: Euclidean clustering
    clusters = euclidean_clustering(
        obstacles,
        distance_threshold=config['cluster_distance'],
        min_cluster_size=config['min_cluster_size'],
        max_cluster_size=config['max_cluster_size']
    )

    # Step 4: Bounding boxes
    bboxes_aabb = [compute_aabb(c) for c in clusters]
    bboxes_obb = [compute_obb_pca(c) for c in clusters]

    return {
        'raw_points': points,
        'downsampled': downsampled,
        'ground': ground,
        'obstacles': obstacles,
        'plane': plane,
        'clusters': clusters,
        'bounding_boxes_aabb': bboxes_aabb,
        'bounding_boxes_obb': bboxes_obb,
    }


def evaluate_pipeline(bounding_boxes: list, ground_truth_path: str,
                      iou_threshold: float = 0.3) -> dict:
    """Evaluate pipeline detections against ground truth."""
    # Load ground truth
    gt_list = []
    with open(ground_truth_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split(',')
            label = parts[0].strip()
            cx, cy, cz = float(parts[1]), float(parts[2]), float(parts[3])
            sx, sy, sz = float(parts[4]), float(parts[5]), float(parts[6])
            gt_list.append({
                'label': label,
                'center': (cx, cy, cz),
                'size': (sx, sy, sz),
            })

    matches = match_detections_to_ground_truth(bounding_boxes, gt_list, iou_threshold)

    tp = sum(1 for m in matches if m['matched'])
    fp = sum(1 for m in matches if not m['matched'])
    fn = len(gt_list) - tp

    matched_ious = [m['iou'] for m in matches if m['matched']]
    mean_iou = np.mean(matched_ious) if matched_ious else 0.0

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0

    return {
        'num_detections': len(bounding_boxes),
        'num_ground_truth': len(gt_list),
        'true_positives': tp,
        'false_positives': fp,
        'false_negatives': fn,
        'precision': precision,
        'recall': recall,
        'mean_iou': mean_iou,
    }


def draw_aabb(ax, aabb, color='red', alpha=0.15):
    """Draw an AABB on a 3D axes."""
    mn = aabb['min_corner']
    mx = aabb['max_corner']

    # 8 corners
    corners = np.array([
        [mn[0], mn[1], mn[2]], [mx[0], mn[1], mn[2]],
        [mx[0], mx[1], mn[2]], [mn[0], mx[1], mn[2]],
        [mn[0], mn[1], mx[2]], [mx[0], mn[1], mx[2]],
        [mx[0], mx[1], mx[2]], [mn[0], mx[1], mx[2]],
    ])

    # 6 faces
    faces = [
        [corners[0], corners[1], corners[2], corners[3]],
        [corners[4], corners[5], corners[6], corners[7]],
        [corners[0], corners[1], corners[5], corners[4]],
        [corners[2], corners[3], corners[7], corners[6]],
        [corners[0], corners[3], corners[7], corners[4]],
        [corners[1], corners[2], corners[6], corners[5]],
    ]

    poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor=color, linewidths=0.5)
    ax.add_collection3d(poly)


def visualize_pipeline_results(results: dict, title: str = "Pipeline Results"):
    """Create a multi-panel visualization of the pipeline."""
    fig = plt.figure(figsize=(16, 12))
    colors = plt.cm.tab10.colors

    # Panel 1: Raw
    ax1 = fig.add_subplot(221, projection='3d')
    raw = results['raw_points']
    idx = np.random.choice(len(raw), min(3000, len(raw)), replace=False)
    ax1.scatter(raw[idx, 0], raw[idx, 1], raw[idx, 2], s=0.3, c=raw[idx, 2], cmap='viridis')
    ax1.set_title("1. Raw Point Cloud")
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')

    # Panel 2: Ground vs obstacles
    ax2 = fig.add_subplot(222, projection='3d')
    gnd = results['ground']
    obs = results['obstacles']
    idx_g = np.random.choice(len(gnd), min(2000, len(gnd)), replace=False)
    ax2.scatter(gnd[idx_g, 0], gnd[idx_g, 1], gnd[idx_g, 2], s=0.3, c='green', label='Ground')
    idx_o = np.random.choice(len(obs), min(2000, len(obs)), replace=False) if len(obs) > 0 else []
    if len(idx_o) > 0:
        ax2.scatter(obs[idx_o, 0], obs[idx_o, 1], obs[idx_o, 2], s=0.5, c='red', label='Obstacles')
    ax2.set_title("2. Ground Segmentation")
    ax2.legend(markerscale=10)
    ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')

    # Panel 3: Clusters
    ax3 = fig.add_subplot(223, projection='3d')
    for i, cluster in enumerate(results['clusters']):
        c = colors[i % len(colors)]
        ax3.scatter(cluster[:, 0], cluster[:, 1], cluster[:, 2], s=1, c=[c], label=f'Cluster {i}')
    ax3.set_title(f"3. Clustering ({len(results['clusters'])} clusters)")
    ax3.legend(markerscale=10, fontsize=8)
    ax3.set_xlabel('X'); ax3.set_ylabel('Y'); ax3.set_zlabel('Z')

    # Panel 4: With bounding boxes
    ax4 = fig.add_subplot(224, projection='3d')
    for i, (cluster, aabb) in enumerate(zip(results['clusters'], results['bounding_boxes_aabb'])):
        c = colors[i % len(colors)]
        ax4.scatter(cluster[:, 0], cluster[:, 1], cluster[:, 2], s=1, c=[c])
        draw_aabb(ax4, aabb, color=c, alpha=0.1)
    # Show ground faintly
    ax4.scatter(gnd[idx_g, 0], gnd[idx_g, 1], gnd[idx_g, 2], s=0.1, c='lightgreen', alpha=0.2)
    ax4.set_title("4. Detections with Bounding Boxes")
    ax4.set_xlabel('X'); ax4.set_ylabel('Y'); ax4.set_zlabel('Z')

    plt.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print("=" * 60)
    print("Task 6 SOLUTION: Full Pipeline")
    print("=" * 60)

    config = {
        'voxel_size': 0.2,
        'ransac_iterations': 500,
        'ransac_threshold': 0.15,
        'cluster_distance': 0.8,
        'min_cluster_size': 10,
        'max_cluster_size': 5000,
    }

    points = load_point_cloud_npy(os.path.join(DATA_DIR, "sample_pointcloud.npy"))
    print(f"Loaded {len(points)} points")

    np.random.seed(42)
    results = run_pipeline(points, config)

    print(f"\nPipeline Results:")
    print(f"  Raw: {len(results['raw_points'])}")
    print(f"  Downsampled: {len(results['downsampled'])}")
    print(f"  Ground: {len(results['ground'])}")
    print(f"  Obstacles: {len(results['obstacles'])}")
    print(f"  Clusters: {len(results['clusters'])}")
    print(f"  Plane: {results['plane']}")

    for i, (c, aabb) in enumerate(zip(results['clusters'], results['bounding_boxes_aabb'])):
        print(f"  Obstacle {i+1}: {len(c)} pts, "
              f"center=({aabb['center'][0]:.1f}, {aabb['center'][1]:.1f}, {aabb['center'][2]:.1f}), "
              f"size=({aabb['size'][0]:.1f}, {aabb['size'][1]:.1f}, {aabb['size'][2]:.1f})")

    # Evaluate
    gt_path = os.path.join(DATA_DIR, "ground_truth.txt")
    if os.path.exists(gt_path):
        ev = evaluate_pipeline(results['bounding_boxes_aabb'], gt_path)
        print(f"\nEvaluation:")
        print(f"  GT: {ev['num_ground_truth']}, Det: {ev['num_detections']}")
        print(f"  TP: {ev['true_positives']}, FP: {ev['false_positives']}, FN: {ev['false_negatives']}")
        print(f"  Precision: {ev['precision']:.2%}, Recall: {ev['recall']:.2%}")
        print(f"  Mean IoU: {ev['mean_iou']:.4f}")

    visualize_pipeline_results(results, "LiDAR Obstacle Detection Pipeline - SOLUTION")
    print("\nTask 6 SOLUTION complete!")
