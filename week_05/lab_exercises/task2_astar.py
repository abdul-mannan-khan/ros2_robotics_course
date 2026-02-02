#!/usr/bin/env python3
"""
Week 5 - Task 2: A* Path Planning
Implement A* search on an 8-connected grid with diagonal costs.

A* is the global planner used in Nav2's NavFn and SmacPlanner plugins.
It finds the shortest path from start to goal on the costmap.
"""

import os
import sys
import numpy as np
import heapq

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


def load_data():
    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    return grid_map, metadata


def heuristic(a, b):
    """Euclidean distance heuristic between grid cells a and b.

    Args:
        a: tuple (row, col)
        b: tuple (row, col)

    Returns:
        float: Euclidean distance
    """
    # TODO: Implement Euclidean distance
    raise NotImplementedError("Implement heuristic")


def get_neighbors(node, grid):
    """Get valid 8-connected neighbors of a grid cell.

    Args:
        node: tuple (row, col)
        grid: 2D occupancy grid

    Returns:
        list of (neighbor, cost) tuples
        - Cardinal moves cost 1.0
        - Diagonal moves cost sqrt(2) ~ 1.414
        - Only return cells that are free (grid value == 0)
    """
    # TODO: Implement 8-connected neighbor generation
    # Hint: 8 directions = 4 cardinal + 4 diagonal
    raise NotImplementedError("Implement get_neighbors")


def astar(grid, start, goal):
    """A* pathfinding on a 2D grid.

    Args:
        grid: 2D occupancy grid (0=free, 100=occupied)
        start: (row, col) start cell
        goal: (row, col) goal cell

    Returns:
        path: list of (row, col) from start to goal, or empty list if no path
        expanded: set of expanded nodes (for visualization)
    """
    # TODO: Implement A* algorithm
    # Use a priority queue (heapq)
    # Track g_score, f_score, came_from
    # Return path by backtracking from goal through came_from
    raise NotImplementedError("Implement astar")


def smooth_path(path, grid, weight_smooth=0.5, weight_data=0.5, tolerance=1e-4):
    """Smooth a grid path using gradient descent.

    Iteratively adjust path points to minimize:
      - Distance from original path (data term)
      - Curvature between consecutive points (smoothness term)

    Args:
        path: list of (row, col)
        grid: occupancy grid (to check collisions)
        weight_smooth: smoothness weight
        weight_data: data fidelity weight
        tolerance: convergence threshold

    Returns:
        smoothed path as list of (row, col)
    """
    # TODO: Implement path smoothing
    # Hint: Iteratively update each waypoint (except start/goal)
    raise NotImplementedError("Implement smooth_path")


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    grid_map, metadata = load_data()
    resolution = metadata[0]

    # Define start and goal in grid coordinates
    start = (10, 10)
    goal = (85, 85)

    # Run A*
    path, expanded = astar(grid_map, start, goal)

    if not path:
        print("No path found!")
        return

    smoothed = smooth_path(path, grid_map)

    # Visualize
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100)

    # Expanded nodes
    if expanded:
        exp = np.array(list(expanded))
        ax.scatter(exp[:, 1], exp[:, 0], c='lightblue', s=1, alpha=0.3, label='Expanded')

    # Path
    path_arr = np.array(path)
    ax.plot(path_arr[:, 1], path_arr[:, 0], 'b-', linewidth=2, label='A* Path')

    smooth_arr = np.array(smoothed)
    ax.plot(smooth_arr[:, 1], smooth_arr[:, 0], 'g-', linewidth=2, label='Smoothed')

    ax.plot(start[1], start[0], 'go', markersize=10, label='Start')
    ax.plot(goal[1], goal[0], 'ro', markersize=10, label='Goal')

    ax.legend()
    ax.set_title(f'A* Path Planning (path length: {len(path)}, expanded: {len(expanded)})')

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task2_astar.png')
    plt.savefig(out_path, dpi=100)
    print(f"Saved plot to {out_path}")
    print(f"Path length: {len(path)} cells")
    print(f"Nodes expanded: {len(expanded)}")


if __name__ == '__main__':
    main()
