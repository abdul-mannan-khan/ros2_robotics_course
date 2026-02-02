#!/usr/bin/env python3
"""
Week 8 - Task 2: RRT (Rapidly-exploring Random Tree) in 3D
Implement RRT for drone path planning in a 3D voxel environment.

Run generate_3d_env.py first to create the environment data.
"""

import numpy as np
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


def random_sample(bounds, goal, goal_bias=0.1):
    """
    Generate a random 3D sample point.

    With probability goal_bias, return the goal point (goal biasing).
    Otherwise, return a uniformly random point within bounds.

    Args:
        bounds: array [x_min, x_max, y_min, y_max, z_min, z_max]
        goal: array [x, y, z]
        goal_bias: probability of sampling the goal

    Returns:
        numpy array [x, y, z]
    """
    # TODO: Implement random sampling with goal bias
    pass


def nearest_node(tree_positions, point):
    """
    Find the index of the nearest node in the tree to the given point.

    Args:
        tree_positions: Nx3 numpy array of node positions
        point: array [x, y, z]

    Returns:
        int: index of nearest node
    """
    # TODO: Find nearest node by Euclidean distance
    pass


def steer(from_node, to_node, step_size=2.0):
    """
    Steer from from_node toward to_node with maximum step_size.

    If the distance is less than step_size, return to_node directly.

    Args:
        from_node: array [x, y, z]
        to_node: array [x, y, z]
        step_size: maximum step distance

    Returns:
        numpy array [x, y, z] - the new point
    """
    # TODO: Implement steering function
    pass


def collision_free(grid, from_pt, to_pt, resolution=1.0):
    """
    Check if the straight line from from_pt to to_pt is collision-free.

    Sample points along the line at intervals of resolution/2 and check
    each against the voxel grid.

    Args:
        grid: 3D numpy array (0=free, 1=occupied)
        from_pt: array [x, y, z]
        to_pt: array [x, y, z]
        resolution: grid resolution in meters

    Returns:
        bool: True if path is collision-free
    """
    # TODO: Implement ray-voxel collision checking
    # Sample points along the line segment
    # Check each point's voxel for occupancy
    pass


def rrt(grid, start, goal, bounds, max_iter=5000, step_size=2.0, goal_threshold=2.0, goal_bias=0.1):
    """
    RRT path planning in 3D.

    Build a tree from start toward goal. The tree is stored as:
    - positions: list of [x, y, z] arrays
    - parents: list of parent indices (-1 for root)

    Args:
        grid: 3D voxel grid
        start: array [x, y, z]
        goal: array [x, y, z]
        bounds: [x_min, x_max, y_min, y_max, z_min, z_max]
        max_iter: maximum iterations
        step_size: RRT step size
        goal_threshold: distance to consider goal reached
        goal_bias: probability of sampling goal

    Returns:
        tuple: (positions, parents, goal_index) or (positions, parents, None) if failed
    """
    # TODO: Implement RRT
    # 1. Initialize tree with start node
    # 2. For each iteration:
    #    a. Sample random point (with goal bias)
    #    b. Find nearest node in tree
    #    c. Steer toward sample
    #    d. Check collision
    #    e. Add new node if collision-free
    #    f. Check if goal reached
    pass


def extract_path(positions, parents, goal_index):
    """
    Extract path from tree by backtracking from goal to root.

    Args:
        positions: list of [x, y, z] arrays
        parents: list of parent indices
        goal_index: index of the goal node

    Returns:
        list of [x, y, z] arrays from start to goal
    """
    # TODO: Backtrack through parent pointers
    pass


def main():
    """Run RRT and visualize the tree and path."""
    grid, env_params, obstacles = load_environment()
    start = env_params['start']
    goal = env_params['goal']
    bounds = env_params['bounds']

    print(f"Running RRT from {start} to {goal}")

    # TODO: Run RRT
    # TODO: Extract path
    # TODO: Compute path length

    # TODO: Create 3D visualization
    # - Plot obstacles (semi-transparent voxels)
    # - Plot RRT tree edges (light blue, thin)
    # - Plot final path (red, thick)
    # - Mark start and goal
    # - Save to task2_rrt.png

    print("Task 2 stub - implement the functions above")


if __name__ == '__main__':
    main()
