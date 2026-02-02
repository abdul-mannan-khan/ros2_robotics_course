#!/usr/bin/env python3
"""
Week 8 - Task 1 Solution: 3D A* Path Planning
Complete implementation of A* on a 3D voxel grid with 26-connected neighbors.
"""

import numpy as np
import heapq
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


def heuristic_3d(a, b):
    """3D Euclidean distance heuristic."""
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)


def get_neighbors_3d(node, grid):
    """Get all valid 26-connected neighbors with diagonal costs."""
    nx, ny, nz = grid.shape
    neighbors = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                x2, y2, z2 = node[0] + dx, node[1] + dy, node[2] + dz
                if 0 <= x2 < nx and 0 <= y2 < ny and 0 <= z2 < nz:
                    if grid[x2, y2, z2] == 0:
                        cost = np.sqrt(dx*dx + dy*dy + dz*dz)
                        neighbors.append(((x2, y2, z2), cost))
    return neighbors


def astar_3d(grid, start, goal):
    """3D A* path planning on voxel grid."""
    open_set = []
    counter = 0
    h = heuristic_3d(start, goal)
    heapq.heappush(open_set, (h, counter, start))
    g_cost = {start: 0.0}
    parent = {start: None}
    closed = set()

    while open_set:
        f, _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = []
            node = goal
            while node is not None:
                path.append(node)
                node = parent[node]
            return path[::-1]

        if current in closed:
            continue
        closed.add(current)

        for neighbor, move_cost in get_neighbors_3d(current, grid):
            if neighbor in closed:
                continue
            tentative_g = g_cost[current] + move_cost
            if tentative_g < g_cost.get(neighbor, float('inf')):
                g_cost[neighbor] = tentative_g
                parent[neighbor] = current
                f_cost = tentative_g + heuristic_3d(neighbor, goal)
                counter += 1
                heapq.heappush(open_set, (f_cost, counter, neighbor))

    return None


def path_length(path):
    """Compute total Euclidean length of a path."""
    if path is None or len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        total += heuristic_3d(path[i], path[i + 1])
    return total


def main():
    """Run 3D A* and visualize results."""
    grid, env_params, obstacles = load_environment()
    start = tuple(env_params['start'].astype(int))
    goal = tuple(env_params['goal'].astype(int))

    print(f"Grid shape: {grid.shape}")
    print(f"Start: {start}, Goal: {goal}")
    print(f"Start occupied: {grid[start]}, Goal occupied: {grid[goal]}")

    t0 = time.time()
    path = astar_3d(grid, start, goal)
    elapsed = time.time() - t0

    if path is None:
        print("No path found!")
        return

    length = path_length(path)
    print(f"Path found: {len(path)} nodes, length: {length:.2f}m, time: {elapsed:.3f}s")

    # Visualization
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot obstacles (subsample for performance)
    occ = np.argwhere(grid == 1)
    if len(occ) > 5000:
        idx = np.random.choice(len(occ), 5000, replace=False)
        occ = occ[idx]
    ax.scatter(occ[:, 0], occ[:, 1], occ[:, 2], c='gray', alpha=0.05, s=2, label='Obstacles')

    # Plot path
    path_arr = np.array(path)
    ax.plot(path_arr[:, 0], path_arr[:, 1], path_arr[:, 2], 'b-', linewidth=2, label='A* Path')
    ax.scatter(*start, c='green', s=100, marker='^', label='Start', zorder=5)
    ax.scatter(*goal, c='red', s=100, marker='*', label='Goal', zorder=5)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'3D A* Path Planning\nLength: {length:.1f}m | Nodes: {len(path)} | Time: {elapsed:.3f}s')
    ax.legend()
    ax.view_init(elev=25, azim=-60)

    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'task1_3d_astar.png'), dpi=150)
    plt.close()
    print(f"Saved to {out_dir}/task1_3d_astar.png")


if __name__ == '__main__':
    main()
