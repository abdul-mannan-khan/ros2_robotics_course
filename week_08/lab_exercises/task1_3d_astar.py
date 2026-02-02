#!/usr/bin/env python3
"""
Week 8 - Task 1: 3D A* Path Planning
Implement A* search on a 3D voxel grid with 26-connected neighbors.

Run generate_3d_env.py first to create the environment data.
"""

import numpy as np
import heapq
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_environment():
    """Load the 3D environment data."""
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
    grid = np.load(os.path.join(data_dir, 'voxel_grid.npy'))
    env_params = np.load(os.path.join(data_dir, 'env_params.npy'), allow_pickle=True).item()
    obstacles = np.load(os.path.join(data_dir, 'obstacles.npy'))
    return grid, env_params, obstacles


def heuristic_3d(a, b):
    """
    Compute 3D Euclidean distance heuristic between two grid cells.

    Args:
        a: tuple (x, y, z) - current cell
        b: tuple (x, y, z) - goal cell

    Returns:
        float: Euclidean distance
    """
    # TODO: Implement 3D Euclidean distance
    pass


def get_neighbors_3d(node, grid):
    """
    Get all valid 26-connected neighbors for a node in 3D grid.

    For each of the 26 directions (dx, dy, dz) where each is in {-1, 0, 1}
    and not all zero, compute the neighbor position and movement cost:
    - Face neighbors (6): cost = 1.0
    - Edge neighbors (12): cost = sqrt(2)
    - Corner neighbors (8): cost = sqrt(3)

    Only return neighbors that are within grid bounds and not occupied.

    Args:
        node: tuple (x, y, z)
        grid: 3D numpy array (0=free, 1=occupied)

    Returns:
        list of (neighbor, cost) tuples
    """
    # TODO: Implement 26-connected neighborhood
    # Hint: iterate over dx, dy, dz in {-1, 0, 1}, skip (0,0,0)
    # Cost = sqrt(dx^2 + dy^2 + dz^2)
    pass


def astar_3d(grid, start, goal):
    """
    3D A* path planning on voxel grid.

    Args:
        grid: 3D numpy array (0=free, 1=occupied)
        start: tuple (x, y, z) - start cell indices
        goal: tuple (x, y, z) - goal cell indices

    Returns:
        list of (x, y, z) tuples representing the path, or None if no path found
    """
    # TODO: Implement A* search
    # Use a priority queue (heapq) with (f_cost, counter, node)
    # Track g_costs and parent pointers
    # Return reconstructed path from start to goal
    pass


def path_length(path):
    """
    Compute total Euclidean length of a path.

    Args:
        path: list of (x, y, z) tuples

    Returns:
        float: total path distance
    """
    # TODO: Sum Euclidean distances between consecutive points
    pass


def main():
    """Run 3D A* and visualize results."""
    grid, env_params, obstacles = load_environment()
    start = tuple(env_params['start'].astype(int))
    goal = tuple(env_params['goal'].astype(int))

    print(f"Grid shape: {grid.shape}")
    print(f"Start: {start}, Goal: {goal}")
    print(f"Start occupied: {grid[start]}, Goal occupied: {grid[goal]}")

    # TODO: Run A* and measure time
    # path = astar_3d(grid, start, goal)

    # TODO: Print results (path length, number of nodes, computation time)

    # TODO: Create 3D visualization
    # - Plot occupied voxels (semi-transparent)
    # - Plot path as a line
    # - Mark start (green) and goal (red)
    # - Set proper labels and title
    # - Save to task1_3d_astar.png

    print("Task 1 stub - implement the functions above")


if __name__ == '__main__':
    main()
