#!/usr/bin/env python3
"""
Task 5: Bounding Box Fitting - SOLUTION
=========================================
INSTRUCTOR VERSION - DO NOT DISTRIBUTE TO STUDENTS
"""

import numpy as np
import os

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")


def compute_aabb(points: np.ndarray) -> dict:
    """Compute Axis-Aligned Bounding Box."""
    min_corner = np.min(points, axis=0)
    max_corner = np.max(points, axis=0)
    size = max_corner - min_corner
    center = (min_corner + max_corner) / 2.0
    volume = np.prod(size) if np.all(size > 0) else 0.0

    return {
        'center': center,
        'size': size,
        'min_corner': min_corner,
        'max_corner': max_corner,
        'volume': volume,
    }


def compute_obb_pca(points: np.ndarray) -> dict:
    """Compute Oriented Bounding Box using PCA."""
    centroid = np.mean(points, axis=0)
    centered = points - centroid

    # Covariance matrix
    cov = np.cov(centered.T)

    # Eigendecomposition
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    # Sort by eigenvalue descending
    order = np.argsort(eigenvalues)[::-1]
    eigenvectors = eigenvectors[:, order]

    # Project points onto principal axes
    projected = centered @ eigenvectors

    # Bounding box in principal frame
    min_proj = np.min(projected, axis=0)
    max_proj = np.max(projected, axis=0)
    size = max_proj - min_proj

    # Center in principal frame, then back to original
    center_proj = (min_proj + max_proj) / 2.0
    center = eigenvectors @ center_proj + centroid

    volume = np.prod(size) if np.all(size > 0) else 0.0

    # 8 corners in principal frame
    half = size / 2.0
    signs = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
    ], dtype=float)
    corners_local = signs * half
    corners = (eigenvectors @ corners_local.T).T + center

    return {
        'center': center,
        'size': size,
        'axes': eigenvectors,
        'volume': volume,
        'corners': corners,
    }


def compute_iou_aabb(box1: dict, box2: dict) -> float:
    """Compute 3D IoU between two AABBs."""
    # Intersection
    inter_min = np.maximum(box1['min_corner'], box2['min_corner'])
    inter_max = np.minimum(box1['max_corner'], box2['max_corner'])
    inter_size = np.maximum(inter_max - inter_min, 0.0)
    inter_vol = np.prod(inter_size)

    if inter_vol == 0:
        return 0.0

    union_vol = box1['volume'] + box2['volume'] - inter_vol
    if union_vol == 0:
        return 0.0

    return inter_vol / union_vol


def match_detections_to_ground_truth(detections: list, ground_truth: list,
                                      iou_threshold: float = 0.3) -> list:
    """Match detected bounding boxes to ground truth using greedy IoU matching."""
    matches = []
    matched_gt = set()

    # Compute IoU matrix
    for d_idx, det in enumerate(detections):
        best_iou = 0.0
        best_gt_idx = None

        for g_idx, gt in enumerate(ground_truth):
            if g_idx in matched_gt:
                continue
            # Convert GT to AABB format
            gt_center = np.array(gt['center'])
            gt_size = np.array(gt['size'])
            gt_box = {
                'min_corner': gt_center - gt_size / 2.0,
                'max_corner': gt_center + gt_size / 2.0,
                'volume': np.prod(gt_size),
            }
            iou = compute_iou_aabb(det, gt_box)
            if iou > best_iou:
                best_iou = iou
                best_gt_idx = g_idx

        matched = best_iou >= iou_threshold
        if matched and best_gt_idx is not None:
            matched_gt.add(best_gt_idx)

        matches.append({
            'detection_idx': d_idx,
            'gt_idx': best_gt_idx if matched else None,
            'iou': best_iou,
            'matched': matched,
        })

    return matches


if __name__ == "__main__":
    print("=" * 60)
    print("Task 5 SOLUTION: Bounding Box Fitting")
    print("=" * 60)

    # AABB test
    test_points = np.array([
        [0, 0, 0], [2, 0, 0], [2, 3, 0],
        [0, 3, 0], [0, 0, 1], [2, 3, 1]
    ], dtype=float)
    aabb = compute_aabb(test_points)
    print(f"AABB center: {aabb['center']}, size: {aabb['size']}, vol: {aabb['volume']}")

    # OBB test
    np.random.seed(42)
    angle = np.pi / 4
    R = np.array([[np.cos(angle), -np.sin(angle), 0],
                   [np.sin(angle), np.cos(angle), 0],
                   [0, 0, 1]])
    base_pts = np.random.uniform([-2, -0.5, 0], [2, 0.5, 1], (200, 3))
    rotated_pts = (R @ base_pts.T).T + [5, 5, 0]
    obb = compute_obb_pca(rotated_pts)
    aabb_r = compute_aabb(rotated_pts)
    print(f"OBB vol: {obb['volume']:.2f}, AABB vol: {aabb_r['volume']:.2f}")

    # IoU test
    box1 = {'min_corner': np.array([0,0,0]), 'max_corner': np.array([2,2,2]), 'volume': 8.0}
    box2 = {'min_corner': np.array([1,1,1]), 'max_corner': np.array([3,3,3]), 'volume': 8.0}
    print(f"IoU: {compute_iou_aabb(box1, box2):.4f} (expected {1/15:.4f})")

    print("\nTask 5 SOLUTION complete!")
