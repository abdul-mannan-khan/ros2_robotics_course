#!/usr/bin/env python3
"""
Task 5: Fit Oriented Bounding Boxes to Clusters
=================================================

Objectives:
- Implement Axis-Aligned Bounding Box (AABB)
- Implement Oriented Bounding Box (OBB) using PCA
- Compute Intersection over Union (IoU) for evaluation

Instructions:
- Complete all functions marked with TODO
- Run this file to verify your implementation
- Do NOT use Open3D's bounding box functions
"""

import numpy as np
import os

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def compute_aabb(points: np.ndarray) -> dict:
    """Compute Axis-Aligned Bounding Box for a point cluster.

    Args:
        points: Mx3 array of cluster points.

    Returns:
        dict with:
            'center': np.ndarray (3,) - center of the box
            'size': np.ndarray (3,) - full extents (width, depth, height)
            'min_corner': np.ndarray (3,)
            'max_corner': np.ndarray (3,)
            'volume': float
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def compute_obb_pca(points: np.ndarray) -> dict:
    """Compute Oriented Bounding Box using Principal Component Analysis.

    Args:
        points: Mx3 array of cluster points.

    Returns:
        dict with:
            'center': np.ndarray (3,) - center of the box (in original frame)
            'size': np.ndarray (3,) - full extents along principal axes
            'axes': np.ndarray (3, 3) - rotation matrix (columns = principal axes)
            'volume': float
            'corners': np.ndarray (8, 3) - 8 corner points in original frame

    Algorithm:
        1. Compute centroid of points
        2. Center the points (subtract centroid)
        3. Compute covariance matrix (3x3)
        4. Eigendecomposition -> eigenvalues, eigenvectors
        5. Project centered points onto eigenvectors
        6. Compute min/max along each principal axis -> size
        7. Compute center in principal frame, transform back
        8. Compute 8 corners
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def compute_iou_aabb(box1: dict, box2: dict) -> float:
    """Compute 3D Intersection over Union between two AABBs.

    Args:
        box1, box2: Dicts with 'min_corner' and 'max_corner' keys.

    Returns:
        float: IoU value between 0 and 1.

    Steps:
        1. Compute intersection box: max of mins, min of maxes
        2. If any dimension has no overlap, IoU = 0
        3. Intersection volume = product of overlapping dimensions
        4. Union volume = vol1 + vol2 - intersection
        5. IoU = intersection / union
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def match_detections_to_ground_truth(detections: list, ground_truth: list,
                                      iou_threshold: float = 0.3) -> list:
    """Match detected bounding boxes to ground truth using IoU.

    Args:
        detections: List of AABB dicts from detected clusters.
        ground_truth: List of dicts with 'center' and 'size' keys.
        iou_threshold: Minimum IoU for a valid match.

    Returns:
        list of dicts:
            'detection_idx': int
            'gt_idx': int or None
            'iou': float
            'matched': bool
    """
    # TODO: Implement this function
    # Use greedy matching: for each detection, find best GT match
    # A GT box can only be matched once
    raise NotImplementedError("Complete this function")


# =============================================================================
# Main - Run this to test your implementations
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("Task 5: Bounding Box Fitting")
    print("=" * 60)

    # Test 1: AABB
    print("\n[Test 1] Axis-Aligned Bounding Box")
    test_points = np.array([
        [0, 0, 0], [2, 0, 0], [2, 3, 0],
        [0, 3, 0], [0, 0, 1], [2, 3, 1]
    ], dtype=float)
    aabb = compute_aabb(test_points)
    print(f"  Center: {aabb['center']}")
    print(f"  Size: {aabb['size']}")
    print(f"  Volume: {aabb['volume']:.1f}")
    assert np.allclose(aabb['center'], [1, 1.5, 0.5]), f"Wrong center: {aabb['center']}"
    assert np.allclose(aabb['size'], [2, 3, 1]), f"Wrong size: {aabb['size']}"
    print("  PASSED")

    # Test 2: OBB with PCA
    print("\n[Test 2] Oriented Bounding Box (PCA)")
    np.random.seed(42)
    # Create a rotated elongated cluster
    angle = np.pi / 4  # 45 degrees
    R = np.array([[np.cos(angle), -np.sin(angle), 0],
                   [np.sin(angle), np.cos(angle), 0],
                   [0, 0, 1]])
    base_pts = np.random.uniform([-2, -0.5, 0], [2, 0.5, 1], (200, 3))
    rotated_pts = (R @ base_pts.T).T + np.array([5, 5, 0])

    obb = compute_obb_pca(rotated_pts)
    print(f"  Center: {obb['center'].round(2)}")
    print(f"  Size: {obb['size'].round(2)}")
    print(f"  Volume: {obb['volume']:.2f}")
    print(f"  OBB volume should be < AABB volume")
    aabb_rot = compute_aabb(rotated_pts)
    print(f"  AABB volume: {aabb_rot['volume']:.2f}, OBB volume: {obb['volume']:.2f}")
    assert obb['volume'] < aabb_rot['volume'] * 1.1, "OBB should be tighter than AABB"
    print("  PASSED")

    # Test 3: IoU
    print("\n[Test 3] Intersection over Union")
    box1 = {'min_corner': np.array([0, 0, 0]), 'max_corner': np.array([2, 2, 2]),
            'center': np.array([1, 1, 1]), 'size': np.array([2, 2, 2]), 'volume': 8.0}
    box2 = {'min_corner': np.array([1, 1, 1]), 'max_corner': np.array([3, 3, 3]),
            'center': np.array([2, 2, 2]), 'size': np.array([2, 2, 2]), 'volume': 8.0}
    iou = compute_iou_aabb(box1, box2)
    print(f"  IoU of 50% overlapping boxes: {iou:.4f}")
    expected_iou = 1.0 / 15.0  # intersection=1, union=8+8-1=15
    assert abs(iou - expected_iou) < 0.01, f"Expected IoU={expected_iou:.4f}, got {iou:.4f}"
    print("  PASSED")

    # Test 4: No overlap
    box3 = {'min_corner': np.array([5, 5, 5]), 'max_corner': np.array([6, 6, 6]),
            'center': np.array([5.5, 5.5, 5.5]), 'size': np.array([1, 1, 1]), 'volume': 1.0}
    iou_none = compute_iou_aabb(box1, box3)
    assert iou_none == 0.0, f"Expected IoU=0 for non-overlapping boxes, got {iou_none}"
    print("  No overlap IoU = 0: PASSED")

    print("\nTask 5 complete!")
