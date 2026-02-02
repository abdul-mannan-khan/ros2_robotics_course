#!/usr/bin/env python3
"""
Task 4: Implement Euclidean Clustering with KD-Tree
=====================================================

Objectives:
- Build a KD-Tree from scratch (or use scipy.spatial.KDTree)
- Implement Euclidean clustering using BFS/DFS with KD-Tree
- Understand clustering parameters and their effects

Instructions:
- Complete all functions marked with TODO
- You may use scipy.spatial.KDTree for the spatial index
- The clustering algorithm itself must be implemented from scratch
- Do NOT use sklearn.cluster.DBSCAN or similar
"""

import numpy as np
from scipy.spatial import KDTree
import os

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def euclidean_clustering(points: np.ndarray, distance_threshold: float,
                         min_cluster_size: int = 10,
                         max_cluster_size: int = 5000) -> list:
    """Perform Euclidean clustering on a point cloud.

    Uses a KD-Tree for efficient neighbor search and BFS to grow clusters.

    Args:
        points: Nx3 array of (x, y, z) points.
        distance_threshold: Maximum distance between points in same cluster.
        min_cluster_size: Minimum points for a valid cluster.
        max_cluster_size: Maximum points for a valid cluster.

    Returns:
        list of np.ndarray: Each element is an Mx3 array of cluster points.
                            Only clusters within [min_size, max_size] are returned.

    Algorithm:
        1. Build KD-Tree from points
        2. Create a boolean array 'visited' initialized to False
        3. For each unvisited point p:
           a. Start a new cluster, add p to queue
           b. Mark p as visited
           c. While queue is not empty:
              - Pop point q from queue
              - Add q to current cluster
              - Find all neighbors of q within distance_threshold using KD-Tree
              - For each unvisited neighbor: mark visited, add to queue
           d. If cluster size is within [min_size, max_size], save it
        4. Return list of valid clusters
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def get_cluster_properties(clusters: list) -> list:
    """Compute properties for each cluster.

    Args:
        clusters: List of Mx3 arrays (output of euclidean_clustering).

    Returns:
        list of dicts, each with:
            'centroid': np.ndarray (3,) - mean position
            'num_points': int
            'extent': np.ndarray (3,) - bounding box size (max - min per axis)
            'min_bound': np.ndarray (3,)
            'max_bound': np.ndarray (3,)
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


def tune_clustering_parameters(points: np.ndarray,
                                distance_thresholds: list,
                                min_sizes: list) -> dict:
    """Experiment with different clustering parameters.

    Args:
        points: Nx3 obstacle points (after ground removal).
        distance_thresholds: List of distance values to try.
        min_sizes: List of min_cluster_size values to try.

    Returns:
        dict mapping (dist_thresh, min_size) -> {
            'num_clusters': int,
            'cluster_sizes': list of int (size of each cluster)
        }
    """
    # TODO: Implement this function
    raise NotImplementedError("Complete this function")


# =============================================================================
# Main - Run this to test your implementations
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("Task 4: Euclidean Clustering with KD-Tree")
    print("=" * 60)

    # Create test data: 3 well-separated clusters
    np.random.seed(42)
    cluster1 = np.random.randn(100, 3) * 0.3 + np.array([0, 0, 0])
    cluster2 = np.random.randn(80, 3) * 0.3 + np.array([5, 0, 0])
    cluster3 = np.random.randn(60, 3) * 0.3 + np.array([0, 5, 0])
    noise = np.random.randn(5, 3) * 5  # scattered noise
    test_points = np.vstack([cluster1, cluster2, cluster3, noise])

    # Test 1: Basic clustering on synthetic data
    print("\n[Test 1] Clustering synthetic data (3 known clusters)")
    clusters = euclidean_clustering(test_points, distance_threshold=1.0,
                                     min_cluster_size=10, max_cluster_size=500)
    print(f"  Found {len(clusters)} clusters")
    for i, c in enumerate(clusters):
        print(f"  Cluster {i}: {len(c)} points, centroid={c.mean(axis=0).round(2)}")
    assert len(clusters) == 3, f"Expected 3 clusters, got {len(clusters)}"
    print("  PASSED")

    # Test 2: Cluster properties
    print("\n[Test 2] Cluster properties")
    props = get_cluster_properties(clusters)
    for i, p in enumerate(props):
        print(f"  Cluster {i}: centroid={p['centroid'].round(2)}, "
              f"points={p['num_points']}, extent={p['extent'].round(2)}")

    # Test 3: On real obstacle data (if available from Task 3)
    print("\n[Test 3] Clustering obstacle points from sample data")
    points = np.load(os.path.join(DATA_DIR, "sample_pointcloud.npy"))

    # Simple ground removal for testing: remove points near z=0
    obstacle_mask = points[:, 2] > 0.2
    obstacle_points = points[obstacle_mask]
    print(f"  Obstacle points (z > 0.2): {len(obstacle_points)}")

    clusters = euclidean_clustering(obstacle_points, distance_threshold=0.8,
                                     min_cluster_size=10, max_cluster_size=5000)
    print(f"  Found {len(clusters)} clusters")
    props = get_cluster_properties(clusters)
    for i, p in enumerate(props):
        print(f"  Cluster {i}: {p['num_points']} pts, "
              f"centroid=({p['centroid'][0]:.1f}, {p['centroid'][1]:.1f}, {p['centroid'][2]:.1f}), "
              f"extent=({p['extent'][0]:.1f}, {p['extent'][1]:.1f}, {p['extent'][2]:.1f})")

    print("\nTask 4 complete!")
