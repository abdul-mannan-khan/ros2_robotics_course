#!/usr/bin/env python3
"""
Week 8 - Task 2 Solution: RRT in 3D
Complete RRT implementation for drone path planning.
"""

import numpy as np
import os
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_environment():
    """Load the 3D environment data."""
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises', 'data')
    grid = np.load(os.path.join(data_dir, 'voxel_grid.npy'))
    env_params = np.load(os.path.join(data_dir, 'env_params.npy'), allow_pickle=True).item()
    obstacles = np.load(os.path.join(data_dir, 'obstacles.npy'))
    return grid, env_params, obstacles


def random_sample(bounds, goal, goal_bias=0.1):
    """Random 3D sample with goal bias."""
    if np.random.random() < goal_bias:
        return goal.copy()
    return np.array([
        np.random.uniform(bounds[0], bounds[1]),
        np.random.uniform(bounds[2], bounds[3]),
        np.random.uniform(bounds[4], bounds[5]),
    ])


def nearest_node(positions, point):
    """Find nearest node index by Euclidean distance."""
    pos_arr = np.array(positions)
    dists = np.linalg.norm(pos_arr - point, axis=1)
    return int(np.argmin(dists))


def steer(from_node, to_node, step_size=2.0):
    """Steer from from_node toward to_node."""
    diff = to_node - from_node
    dist = np.linalg.norm(diff)
    if dist <= step_size:
        return to_node.copy()
    return from_node + diff / dist * step_size


def collision_free(grid, from_pt, to_pt, resolution=1.0):
    """Check line segment for collisions using ray marching."""
    diff = to_pt - from_pt
    dist = np.linalg.norm(diff)
    if dist < 1e-6:
        return True
    n_steps = max(int(dist / (resolution * 0.5)), 2)
    for i in range(n_steps + 1):
        t = i / n_steps
        pt = from_pt + t * diff
        ix, iy, iz = int(round(pt[0])), int(round(pt[1])), int(round(pt[2]))
        if ix < 0 or iy < 0 or iz < 0:
            return False
        if ix >= grid.shape[0] or iy >= grid.shape[1] or iz >= grid.shape[2]:
            return False
        if grid[ix, iy, iz] == 1:
            return False
    return True


def rrt(grid, start, goal, bounds, max_iter=5000, step_size=2.0,
        goal_threshold=2.0, goal_bias=0.1):
    """RRT path planning in 3D."""
    positions = [start.copy()]
    parents = [-1]

    for i in range(max_iter):
        sample = random_sample(bounds, goal, goal_bias)
        near_idx = nearest_node(positions, sample)
        new_pt = steer(np.array(positions[near_idx]), sample, step_size)

        if collision_free(grid, np.array(positions[near_idx]), new_pt):
            positions.append(new_pt)
            parents.append(near_idx)
            new_idx = len(positions) - 1

            if np.linalg.norm(new_pt - goal) < goal_threshold:
                # Connect to goal
                if collision_free(grid, new_pt, goal):
                    positions.append(goal.copy())
                    parents.append(new_idx)
                    return positions, parents, len(positions) - 1

    return positions, parents, None


def extract_path(positions, parents, goal_index):
    """Extract path by backtracking."""
    if goal_index is None:
        return None
    path = []
    idx = goal_index
    while idx != -1:
        path.append(positions[idx])
        idx = parents[idx]
    return path[::-1]


def path_length(path):
    """Total Euclidean path length."""
    if path is None or len(path) < 2:
        return 0.0
    return sum(np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))
               for i in range(len(path) - 1))


def main():
    """Run RRT and visualize."""
    grid, env_params, obstacles = load_environment()
    start = env_params['start']
    goal = env_params['goal']
    bounds = env_params['bounds']

    np.random.seed(42)

    print(f"Running RRT from {start} to {goal}")
    t0 = time.time()
    positions, parents, goal_idx = rrt(grid, start, goal, bounds,
                                        max_iter=8000, step_size=3.0,
                                        goal_threshold=3.0, goal_bias=0.15)
    elapsed = time.time() - t0

    path = extract_path(positions, parents, goal_idx)
    if path is None:
        print("RRT failed to find path!")
        return

    length = path_length(path)
    print(f"Path found: {len(path)} nodes, length: {length:.2f}m, "
          f"tree size: {len(positions)}, time: {elapsed:.3f}s")

    # Visualization
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Obstacles
    occ = np.argwhere(grid == 1)
    if len(occ) > 5000:
        idx = np.random.choice(len(occ), 5000, replace=False)
        occ = occ[idx]
    ax.scatter(occ[:, 0], occ[:, 1], occ[:, 2], c='gray', alpha=0.05, s=2)

    # Tree edges (subsample if large)
    max_edges = 3000
    edge_indices = list(range(1, len(positions)))
    if len(edge_indices) > max_edges:
        edge_indices = list(np.random.choice(edge_indices, max_edges, replace=False))
    for i in edge_indices:
        p = positions[parents[i]]
        c = positions[i]
        ax.plot([p[0], c[0]], [p[1], c[1]], [p[2], c[2]],
                'c-', alpha=0.1, linewidth=0.3)

    # Path
    path_arr = np.array(path)
    ax.plot(path_arr[:, 0], path_arr[:, 1], path_arr[:, 2],
            'r-', linewidth=2.5, label=f'RRT Path ({length:.1f}m)')
    ax.scatter(*start, c='green', s=100, marker='^', label='Start', zorder=5)
    ax.scatter(*goal, c='red', s=100, marker='*', label='Goal', zorder=5)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'RRT 3D Path Planning\nTree: {len(positions)} nodes | Time: {elapsed:.3f}s')
    ax.legend()
    ax.view_init(elev=25, azim=-60)

    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'task2_rrt.png'), dpi=150)
    plt.close()
    print(f"Saved to {out_dir}/task2_rrt.png")


if __name__ == '__main__':
    main()
