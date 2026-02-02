#!/usr/bin/env python3
"""
Task 6: Full LiDAR Obstacle Detection Pipeline
=================================================

Objectives:
- Integrate all previous tasks into a complete pipeline
- Evaluate detection performance against ground truth
- Visualize the full pipeline output

Instructions:
- Import and use your implementations from Tasks 1-5
- Complete the pipeline function and evaluation
- Generate a final visualization showing all stages
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os

# Import your implementations from previous tasks
from task1_load_and_visualize import load_point_cloud_npy
from task2_voxel_downsampling import voxel_grid_downsample
from task3_ransac_ground_removal import ransac_plane_fit, segment_ground
from task4_euclidean_clustering import euclidean_clustering, get_cluster_properties
from task5_bounding_boxes import compute_aabb, compute_obb_pca, compute_iou_aabb, match_detections_to_ground_truth

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def run_pipeline(points: np.ndarray, config: dict) -> dict:
    """Run the complete LiDAR obstacle detection pipeline.

    Args:
        points: Nx3 raw point cloud.
        config: dict with pipeline parameters:
            'voxel_size': float - voxel grid size
            'ransac_iterations': int - RANSAC iterations
            'ransac_threshold': float - RANSAC distance threshold
            'cluster_distance': float - clustering distance threshold
            'min_cluster_size': int - minimum cluster size
            'max_cluster_size': int - maximum cluster size

    Returns:
        dict with:
            'raw_points': np.ndarray - original input
            'downsampled': np.ndarray - after voxel downsampling
            'ground': np.ndarray - ground points
            'obstacles': np.ndarray - obstacle points
            'plane': tuple - ground plane (a, b, c, d)
            'clusters': list of np.ndarray - individual clusters
            'bounding_boxes_aabb': list of dict - AABB for each cluster
            'bounding_boxes_obb': list of dict - OBB for each cluster
    """
    # TODO: Implement the full pipeline
    # Step 1: Voxel grid downsampling
    # Step 2: RANSAC ground removal
    # Step 3: Euclidean clustering on obstacle points
    # Step 4: Bounding box fitting (both AABB and OBB) for each cluster
    # Return all intermediate and final results
    raise NotImplementedError("Complete this function")


def evaluate_pipeline(bounding_boxes: list, ground_truth_path: str,
                      iou_threshold: float = 0.3) -> dict:
    """Evaluate pipeline detections against ground truth.

    Args:
        bounding_boxes: List of AABB dicts from pipeline.
        ground_truth_path: Path to ground_truth.txt file.
        iou_threshold: Minimum IoU for a match.

    Returns:
        dict with:
            'num_detections': int
            'num_ground_truth': int
            'true_positives': int
            'false_positives': int
            'false_negatives': int
            'precision': float
            'recall': float
            'mean_iou': float (of matched detections)
    """
    # TODO: Implement evaluation
    # 1. Load ground truth from file
    # 2. Convert ground truth to AABB format
    # 3. Match detections to GT using IoU
    # 4. Compute precision, recall, mean IoU
    raise NotImplementedError("Complete this function")


def visualize_pipeline_results(results: dict, title: str = "Pipeline Results"):
    """Create a multi-panel visualization of the pipeline.

    Create a figure with 4 subplots:
    1. Raw point cloud
    2. After downsampling (colored by ground/obstacle)
    3. Clustered obstacles (each cluster different color)
    4. Final result with bounding boxes

    Args:
        results: Output dict from run_pipeline().
    """
    # TODO: Implement visualization
    # Create 2x2 subplot figure with 3D axes
    # Panel 1: Raw point cloud (subsample for speed)
    # Panel 2: Ground (green) vs obstacles (red)
    # Panel 3: Each cluster in different color
    # Panel 4: Clusters with bounding boxes drawn
    raise NotImplementedError("Complete this function")


# =============================================================================
# Main - Run this to test your full pipeline
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("Task 6: Full LiDAR Obstacle Detection Pipeline")
    print("=" * 60)

    # Pipeline configuration
    config = {
        'voxel_size': 0.2,
        'ransac_iterations': 500,
        'ransac_threshold': 0.15,
        'cluster_distance': 0.8,
        'min_cluster_size': 10,
        'max_cluster_size': 5000,
    }

    # Load data
    points = load_point_cloud_npy(os.path.join(DATA_DIR, "sample_pointcloud.npy"))
    print(f"Loaded {len(points)} points")

    # Run pipeline
    print("\nRunning pipeline...")
    results = run_pipeline(points, config)

    print(f"\nPipeline Results:")
    print(f"  Raw points:        {len(results['raw_points'])}")
    print(f"  After downsample:  {len(results['downsampled'])}")
    print(f"  Ground points:     {len(results['ground'])}")
    print(f"  Obstacle points:   {len(results['obstacles'])}")
    print(f"  Clusters found:    {len(results['clusters'])}")
    print(f"  Ground plane:      {results['plane']}")

    # Print cluster details
    print("\nDetected Obstacles:")
    for i, (cluster, aabb) in enumerate(zip(results['clusters'], results['bounding_boxes_aabb'])):
        print(f"  Obstacle {i+1}: {len(cluster)} pts, "
              f"center=({aabb['center'][0]:.1f}, {aabb['center'][1]:.1f}, {aabb['center'][2]:.1f}), "
              f"size=({aabb['size'][0]:.1f}, {aabb['size'][1]:.1f}, {aabb['size'][2]:.1f})")

    # Evaluate
    print("\nEvaluation:")
    gt_path = os.path.join(DATA_DIR, "ground_truth.txt")
    eval_results = evaluate_pipeline(results['bounding_boxes_aabb'], gt_path)
    print(f"  Ground Truth:     {eval_results['num_ground_truth']} obstacles")
    print(f"  Detections:       {eval_results['num_detections']}")
    print(f"  True Positives:   {eval_results['true_positives']}")
    print(f"  False Positives:  {eval_results['false_positives']}")
    print(f"  False Negatives:  {eval_results['false_negatives']}")
    print(f"  Precision:        {eval_results['precision']:.2%}")
    print(f"  Recall:           {eval_results['recall']:.2%}")
    print(f"  Mean IoU:         {eval_results['mean_iou']:.4f}")

    # Visualize
    print("\nGenerating visualization...")
    visualize_pipeline_results(results)

    print("\nTask 6 complete! Full pipeline implemented and evaluated.")
