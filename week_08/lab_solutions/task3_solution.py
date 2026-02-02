#!/usr/bin/env python3
"""
Week 8 - Task 3 Solution: RRT* with Rewiring
Complete RRT* implementation with cost comparison to RRT.
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
    return grid, env_params


def random_sample(bounds, goal, goal_bias=0.1):
    if np.random.random() < goal_bias:
        return goal.copy()
    return np.array([
        np.random.uniform(bounds[0], bounds[1]),
        np.random.uniform(bounds[2], bounds[3]),
        np.random.uniform(bounds[4], bounds[5]),
    ])


def nearest_node(positions, point):
    pos_arr = np.array(positions)
    dists = np.linalg.norm(pos_arr - point, axis=1)
    return int(np.argmin(dists))


def steer(from_node, to_node, step_size=2.0):
    diff = to_node - from_node
    dist = np.linalg.norm(diff)
    if dist <= step_size:
        return to_node.copy()
    return from_node + diff / dist * step_size


def collision_free(grid, from_pt, to_pt, resolution=1.0):
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


def near_nodes(positions, point, radius):
    """Find all node indices within radius."""
    indices = []
    for i, p in enumerate(positions):
        if np.linalg.norm(np.array(p) - point) <= radius:
            indices.append(i)
    return indices


def rrt_basic(grid, start, goal, bounds, max_iter=5000, step_size=3.0,
              goal_threshold=3.0, goal_bias=0.15):
    """Standard RRT (no rewiring) for comparison."""
    positions = [start.copy()]
    parents = [-1]
    costs = [0.0]
    best_goal_idx = None
    best_goal_cost = float('inf')
    cost_history = []

    for i in range(max_iter):
        sample = random_sample(bounds, goal, goal_bias)
        near_idx = nearest_node(positions, sample)
        new_pt = steer(np.array(positions[near_idx]), sample, step_size)

        if collision_free(grid, np.array(positions[near_idx]), new_pt):
            new_cost = costs[near_idx] + np.linalg.norm(new_pt - np.array(positions[near_idx]))
            positions.append(new_pt)
            parents.append(near_idx)
            costs.append(new_cost)
            new_idx = len(positions) - 1

            if np.linalg.norm(new_pt - goal) < goal_threshold:
                if collision_free(grid, new_pt, goal):
                    goal_cost = new_cost + np.linalg.norm(new_pt - goal)
                    if goal_cost < best_goal_cost:
                        positions.append(goal.copy())
                        parents.append(new_idx)
                        costs.append(goal_cost)
                        best_goal_idx = len(positions) - 1
                        best_goal_cost = goal_cost

        cost_history.append(best_goal_cost if best_goal_cost < float('inf') else None)

    return positions, parents, best_goal_idx, cost_history


def rrt_star(grid, start, goal, bounds, max_iter=5000, step_size=3.0,
             goal_threshold=3.0, goal_bias=0.15, rewire_radius=6.0):
    """RRT* with rewiring."""
    positions = [start.copy()]
    parents = [-1]
    costs = [0.0]
    best_goal_idx = None
    best_goal_cost = float('inf')
    cost_history = []

    for i in range(max_iter):
        sample = random_sample(bounds, goal, goal_bias)
        near_idx = nearest_node(positions, sample)
        new_pt = steer(np.array(positions[near_idx]), sample, step_size)

        if not collision_free(grid, np.array(positions[near_idx]), new_pt):
            cost_history.append(best_goal_cost if best_goal_cost < float('inf') else None)
            continue

        # Find nearby nodes
        nearby = near_nodes(positions, new_pt, rewire_radius)

        # Choose best parent from nearby nodes
        best_parent = near_idx
        best_cost = costs[near_idx] + np.linalg.norm(new_pt - np.array(positions[near_idx]))

        for ni in nearby:
            c = costs[ni] + np.linalg.norm(new_pt - np.array(positions[ni]))
            if c < best_cost and collision_free(grid, np.array(positions[ni]), new_pt):
                best_parent = ni
                best_cost = c

        positions.append(new_pt)
        parents.append(best_parent)
        costs.append(best_cost)
        new_idx = len(positions) - 1

        # Rewire nearby nodes through new node
        for ni in nearby:
            if ni == best_parent:
                continue
            new_route_cost = best_cost + np.linalg.norm(new_pt - np.array(positions[ni]))
            if new_route_cost < costs[ni]:
                if collision_free(grid, new_pt, np.array(positions[ni])):
                    parents[ni] = new_idx
                    costs[ni] = new_route_cost

        # Check goal
        if np.linalg.norm(new_pt - goal) < goal_threshold:
            if collision_free(grid, new_pt, goal):
                goal_cost = best_cost + np.linalg.norm(new_pt - goal)
                if goal_cost < best_goal_cost:
                    positions.append(goal.copy())
                    parents.append(new_idx)
                    costs.append(goal_cost)
                    best_goal_idx = len(positions) - 1
                    best_goal_cost = goal_cost

        cost_history.append(best_goal_cost if best_goal_cost < float('inf') else None)

    return positions, parents, best_goal_idx, cost_history


def extract_path(positions, parents, goal_index):
    if goal_index is None:
        return None
    path = []
    idx = goal_index
    while idx != -1:
        path.append(positions[idx])
        idx = parents[idx]
    return path[::-1]


def path_length(path):
    if path is None or len(path) < 2:
        return 0.0
    return sum(np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))
               for i in range(len(path) - 1))


def main():
    grid, env_params = load_environment()
    start = env_params['start']
    goal = env_params['goal']
    bounds = env_params['bounds']

    np.random.seed(42)

    print("Running RRT...")
    t0 = time.time()
    rrt_pos, rrt_par, rrt_goal, rrt_hist = rrt_basic(
        grid, start, goal, bounds, max_iter=6000, step_size=3.0,
        goal_threshold=3.0, goal_bias=0.15)
    rrt_time = time.time() - t0
    rrt_path = extract_path(rrt_pos, rrt_par, rrt_goal)
    rrt_len = path_length(rrt_path) if rrt_path else float('inf')
    print(f"  RRT: length={rrt_len:.1f}m, tree={len(rrt_pos)}, time={rrt_time:.3f}s")

    np.random.seed(42)

    print("Running RRT*...")
    t0 = time.time()
    rrt_s_pos, rrt_s_par, rrt_s_goal, rrt_s_hist = rrt_star(
        grid, start, goal, bounds, max_iter=6000, step_size=3.0,
        goal_threshold=3.0, goal_bias=0.15, rewire_radius=6.0)
    rrt_s_time = time.time() - t0
    rrt_s_path = extract_path(rrt_s_pos, rrt_s_par, rrt_s_goal)
    rrt_s_len = path_length(rrt_s_path) if rrt_s_path else float('inf')
    print(f"  RRT*: length={rrt_s_len:.1f}m, tree={len(rrt_s_pos)}, time={rrt_s_time:.3f}s")

    if rrt_len < float('inf') and rrt_s_len < float('inf'):
        improvement = (rrt_len - rrt_s_len) / rrt_len * 100
        print(f"  Improvement: {improvement:.1f}%")

    # Visualization
    fig = plt.figure(figsize=(18, 12))

    # Obstacles for both plots
    occ = np.argwhere(grid == 1)
    if len(occ) > 3000:
        occ_idx = np.random.choice(len(occ), 3000, replace=False)
        occ_sub = occ[occ_idx]
    else:
        occ_sub = occ

    # RRT plot
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.scatter(occ_sub[:, 0], occ_sub[:, 1], occ_sub[:, 2], c='gray', alpha=0.05, s=2)
    if rrt_path:
        p = np.array(rrt_path)
        ax1.plot(p[:, 0], p[:, 1], p[:, 2], 'r-', linewidth=2.5)
    ax1.scatter(*start, c='green', s=80, marker='^', zorder=5)
    ax1.scatter(*goal, c='red', s=80, marker='*', zorder=5)
    ax1.set_title(f'RRT Path ({rrt_len:.1f}m)')
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax1.view_init(elev=25, azim=-60)

    # RRT* plot
    ax2 = fig.add_subplot(222, projection='3d')
    ax2.scatter(occ_sub[:, 0], occ_sub[:, 1], occ_sub[:, 2], c='gray', alpha=0.05, s=2)
    if rrt_s_path:
        p = np.array(rrt_s_path)
        ax2.plot(p[:, 0], p[:, 1], p[:, 2], 'b-', linewidth=2.5)
    ax2.scatter(*start, c='green', s=80, marker='^', zorder=5)
    ax2.scatter(*goal, c='red', s=80, marker='*', zorder=5)
    ax2.set_title(f'RRT* Path ({rrt_s_len:.1f}m)')
    ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')
    ax2.view_init(elev=25, azim=-60)

    # Cost convergence
    ax3 = fig.add_subplot(223)
    valid_rrt = [(i, c) for i, c in enumerate(rrt_hist) if c is not None]
    valid_rrt_s = [(i, c) for i, c in enumerate(rrt_s_hist) if c is not None]
    if valid_rrt:
        ax3.plot([x[0] for x in valid_rrt], [x[1] for x in valid_rrt],
                 'r-', label='RRT', alpha=0.8)
    if valid_rrt_s:
        ax3.plot([x[0] for x in valid_rrt_s], [x[1] for x in valid_rrt_s],
                 'b-', label='RRT*', alpha=0.8)
    ax3.set_xlabel('Iteration')
    ax3.set_ylabel('Best Path Cost (m)')
    ax3.set_title('Cost Convergence')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Comparison bar chart
    ax4 = fig.add_subplot(224)
    methods = ['RRT', 'RRT*']
    lengths = [rrt_len if rrt_len < float('inf') else 0,
               rrt_s_len if rrt_s_len < float('inf') else 0]
    times_arr = [rrt_time, rrt_s_time]
    x = np.arange(2)
    bars1 = ax4.bar(x - 0.2, lengths, 0.35, label='Path Length (m)', color='steelblue')
    ax4_twin = ax4.twinx()
    bars2 = ax4_twin.bar(x + 0.2, times_arr, 0.35, label='Time (s)', color='coral')
    ax4.set_xticks(x)
    ax4.set_xticklabels(methods)
    ax4.set_ylabel('Path Length (m)')
    ax4_twin.set_ylabel('Computation Time (s)')
    ax4.set_title('RRT vs RRT* Comparison')
    ax4.legend(loc='upper left')
    ax4_twin.legend(loc='upper right')

    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'task3_rrt_star.png'), dpi=150)
    plt.close()
    print(f"Saved to {out_dir}/task3_rrt_star.png")


if __name__ == '__main__':
    main()
