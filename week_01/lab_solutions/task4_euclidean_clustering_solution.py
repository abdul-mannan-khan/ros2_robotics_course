#!/usr/bin/env python3
"""
Task 4: Euclidean Clustering with KD-Tree - SOLUTION
======================================================
INSTRUCTOR VERSION - DO NOT DISTRIBUTE TO STUDENTS
"""

import numpy as np
from scipy.spatial import KDTree
from collections import deque
import os

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")


def euclidean_clustering(points: np.ndarray, distance_threshold: float,
                         min_cluster_size: int = 10,
                         max_cluster_size: int = 5000) -> list:
    """Perform Euclidean clustering on a point cloud."""
    if len(points) == 0:
        return []

    tree = KDTree(points)
    n = len(points)
    visited = np.zeros(n, dtype=bool)
    clusters = []

    for i in range(n):
        if visited[i]:
            continue

        # BFS to grow cluster
        queue = deque([i])
        visited[i] = True
        cluster_indices = []

        while queue:
            idx = queue.popleft()
            cluster_indices.append(idx)

            # Find neighbors within distance threshold
            neighbor_indices = tree.query_ball_point(points[idx], distance_threshold)

            for ni in neighbor_indices:
                if not visited[ni]:
                    visited[ni] = True
                    queue.append(ni)

        # Filter by size
        if min_cluster_size <= len(cluster_indices) <= max_cluster_size:
            clusters.append(points[cluster_indices])

    return clusters


def get_cluster_properties(clusters: list) -> list:
    """Compute properties for each cluster."""
    properties = []
    for cluster in clusters:
        min_bound = np.min(cluster, axis=0)
        max_bound = np.max(cluster, axis=0)
        properties.append({
            'centroid': np.mean(cluster, axis=0),
            'num_points': len(cluster),
            'extent': max_bound - min_bound,
            'min_bound': min_bound,
            'max_bound': max_bound,
        })
    return properties


def tune_clustering_parameters(points: np.ndarray,
                                distance_thresholds: list,
                                min_sizes: list) -> dict:
    """Experiment with different clustering parameters."""
    results = {}
    for dt in distance_thresholds:
        for ms in min_sizes:
            clusters = euclidean_clustering(points, dt, min_cluster_size=ms)
            results[(dt, ms)] = {
                'num_clusters': len(clusters),
                'cluster_sizes': [len(c) for c in clusters],
            }
    return results


if __name__ == "__main__":
    print("=" * 60)
    print("Task 4 SOLUTION: Euclidean Clustering")
    print("=" * 60)

    # Synthetic test
    np.random.seed(42)
    c1 = np.random.randn(100, 3) * 0.3
    c2 = np.random.randn(80, 3) * 0.3 + [5, 0, 0]
    c3 = np.random.randn(60, 3) * 0.3 + [0, 5, 0]
    noise = np.random.randn(5, 3) * 5
    test_points = np.vstack([c1, c2, c3, noise])

    clusters = euclidean_clustering(test_points, 1.0, min_cluster_size=10)
    print(f"Found {len(clusters)} clusters (expected 3)")
    for i, c in enumerate(clusters):
        print(f"  Cluster {i}: {len(c)} pts, centroid={c.mean(axis=0).round(2)}")

    props = get_cluster_properties(clusters)
    for i, p in enumerate(props):
        print(f"  Props {i}: extent={p['extent'].round(2)}")

    # Real data
    points = np.load(os.path.join(DATA_DIR, "sample_pointcloud.npy"))
    obstacles = points[points[:, 2] > 0.2]
    clusters = euclidean_clustering(obstacles, 0.8, min_cluster_size=10)
    print(f"\nReal data: {len(clusters)} clusters from {len(obstacles)} obstacle points")
    for i, c in enumerate(clusters):
        centroid = c.mean(axis=0)
        print(f"  Cluster {i}: {len(c)} pts at ({centroid[0]:.1f}, {centroid[1]:.1f}, {centroid[2]:.1f})")

    print("\nTask 4 SOLUTION complete!")
