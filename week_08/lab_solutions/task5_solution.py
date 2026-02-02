#!/usr/bin/env python3
"""
Week 8 - Task 5 Solution: Dynamic Replanning with Moving Obstacles
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
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises', 'data')
    grid = np.load(os.path.join(data_dir, 'voxel_grid.npy'))
    env_params = np.load(os.path.join(data_dir, 'env_params.npy'), allow_pickle=True).item()
    dynamic_obs = np.load(os.path.join(data_dir, 'dynamic_obstacles.npy'))
    return grid, env_params, dynamic_obs


def heuristic_3d(a, b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def astar_3d(grid, start, goal):
    """A* on 3D grid."""
    open_set = []
    counter = 0
    heapq.heappush(open_set, (heuristic_3d(start, goal), counter, start))
    g_cost = {start: 0.0}
    parent = {start: None}
    closed = set()

    while open_set:
        f, _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            node = goal
            while node is not None:
                path.append(np.array(node, dtype=float))
                node = parent[node]
            return path[::-1]
        if current in closed:
            continue
        closed.add(current)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    nb = (current[0]+dx, current[1]+dy, current[2]+dz)
                    if (0 <= nb[0] < grid.shape[0] and 0 <= nb[1] < grid.shape[1]
                            and 0 <= nb[2] < grid.shape[2] and grid[nb] == 0
                            and nb not in closed):
                        cost = np.sqrt(dx*dx + dy*dy + dz*dz)
                        tg = g_cost[current] + cost
                        if tg < g_cost.get(nb, float('inf')):
                            g_cost[nb] = tg
                            parent[nb] = current
                            counter += 1
                            heapq.heappush(open_set, (tg + heuristic_3d(nb, goal), counter, nb))
    return None


def get_dynamic_obstacle_position(obstacle, t):
    """Get position and radius of dynamic obstacle at time t."""
    sx, sy, sz, vx, vy, vz, radius, t_start, t_end = obstacle
    if t < t_start or t > t_end:
        return None, None
    dt = t - t_start
    pos = np.array([sx + vx * dt, sy + vy * dt, sz + vz * dt])
    return pos, radius


def detect_collision_ahead(path, path_index, dynamic_obstacles, current_time,
                           drone_speed=2.0, horizon=10.0, safety_margin=1.5):
    """Look ahead for collisions with dynamic obstacles."""
    for i in range(path_index, min(len(path) - 1, path_index + 50)):
        dist_along = sum(np.linalg.norm(np.array(path[j+1]) - np.array(path[j]))
                         for j in range(path_index, i))
        future_time = current_time + dist_along / drone_speed
        if future_time - current_time > horizon:
            break

        pt = np.array(path[i])
        for oi, obs in enumerate(dynamic_obstacles):
            pos, radius = get_dynamic_obstacle_position(obs, future_time)
            if pos is None:
                continue
            if np.linalg.norm(pt - pos) < radius + safety_margin:
                return True, i, oi
    return False, None, None


def local_replan_rrt(current_pos, goal_pos, grid, dynamic_obstacles, current_time,
                     bounds, step_size=2.0, max_iter=2000):
    """Quick local RRT replan avoiding static and dynamic obstacles."""
    positions = [current_pos.copy()]
    parents = [-1]
    goal_thresh = 3.0

    for _ in range(max_iter):
        if np.random.random() < 0.2:
            sample = goal_pos.copy()
        else:
            sample = np.array([
                np.random.uniform(bounds[0], bounds[1]),
                np.random.uniform(bounds[2], bounds[3]),
                np.random.uniform(bounds[4], bounds[5]),
            ])

        # Nearest
        pos_arr = np.array(positions)
        near_idx = int(np.argmin(np.linalg.norm(pos_arr - sample, axis=1)))
        from_pt = np.array(positions[near_idx])
        diff = sample - from_pt
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            continue
        new_pt = from_pt + diff / dist * min(step_size, dist)

        # Static collision
        n_check = max(int(np.linalg.norm(new_pt - from_pt) / 0.5), 2)
        collision = False
        for i in range(n_check + 1):
            t_frac = i / n_check
            pt = from_pt + t_frac * (new_pt - from_pt)
            ix, iy, iz = int(round(pt[0])), int(round(pt[1])), int(round(pt[2]))
            if (ix < 0 or iy < 0 or iz < 0 or ix >= grid.shape[0]
                    or iy >= grid.shape[1] or iz >= grid.shape[2] or grid[ix, iy, iz] == 1):
                collision = True
                break
        if collision:
            continue

        # Dynamic collision check at current time
        for obs in dynamic_obstacles:
            pos, radius = get_dynamic_obstacle_position(obs, current_time)
            if pos is not None and np.linalg.norm(new_pt - pos) < radius + 1.5:
                collision = True
                break
        if collision:
            continue

        positions.append(new_pt)
        parents.append(near_idx)

        if np.linalg.norm(new_pt - goal_pos) < goal_thresh:
            path = []
            idx = len(positions) - 1
            while idx != -1:
                path.append(positions[idx])
                idx = parents[idx]
            return path[::-1]

    return None


def run_with_replanning(grid, start, goal, dynamic_obstacles, bounds, drone_speed=2.0, dt=0.5):
    """Execute path with dynamic replanning."""
    start_t = tuple(start.astype(int))
    goal_t = tuple(goal.astype(int))

    print("  Planning initial path with A*...")
    original_path = astar_3d(grid, start_t, goal_t)
    if original_path is None:
        print("  No initial path found!")
        return None

    print(f"  Initial path: {len(original_path)} nodes")

    actual_trajectory = [start.copy()]
    replan_points = []
    replan_times = []
    current_path = [np.array(p) for p in original_path]
    path_index = 0
    current_time = 0.0
    max_time = 200.0

    while path_index < len(current_path) - 1 and current_time < max_time:
        # Check ahead for collisions
        collision, col_idx, col_obs = detect_collision_ahead(
            current_path, path_index, dynamic_obstacles, current_time, drone_speed)

        if collision:
            print(f"  Collision detected at t={current_time:.1f}s, replanning...")
            current_pos = np.array(current_path[path_index])
            replan_points.append(current_pos.copy())
            replan_times.append(current_time)

            new_path = local_replan_rrt(current_pos, goal, grid, dynamic_obstacles,
                                         current_time, bounds)
            if new_path is not None:
                current_path = new_path
                path_index = 0
                print(f"    Replanned: {len(new_path)} nodes")
            else:
                # Wait briefly and retry
                current_time += dt
                continue

        # Move along path
        current_pos = np.array(current_path[path_index])
        next_pos = np.array(current_path[min(path_index + 1, len(current_path) - 1)])
        move_dist = drone_speed * dt
        seg_dist = np.linalg.norm(next_pos - current_pos)

        if seg_dist < move_dist:
            path_index += 1
        actual_trajectory.append(next_pos.copy())
        current_time += dt

    return {
        'original_path': [np.array(p) for p in original_path],
        'actual_trajectory': actual_trajectory,
        'replan_points': replan_points,
        'replan_times': replan_times,
    }


def main():
    grid, env_params, dynamic_obs = load_environment()
    start = env_params['start']
    goal = env_params['goal']
    bounds = env_params['bounds']

    np.random.seed(42)
    print(f"Start: {start}, Goal: {goal}")
    print(f"Dynamic obstacles: {len(dynamic_obs)}")

    result = run_with_replanning(grid, start, goal, dynamic_obs, bounds)
    if result is None:
        print("Planning failed!")
        return

    print(f"Replanning events: {len(result['replan_points'])}")
    print(f"Original path length: {len(result['original_path'])}")
    print(f"Actual trajectory length: {len(result['actual_trajectory'])}")

    # Visualization
    fig = plt.figure(figsize=(18, 10))

    # 3D view
    ax1 = fig.add_subplot(121, projection='3d')

    occ = np.argwhere(grid == 1)
    if len(occ) > 3000:
        occ = occ[np.random.choice(len(occ), 3000, replace=False)]
    ax1.scatter(occ[:, 0], occ[:, 1], occ[:, 2], c='gray', alpha=0.05, s=2)

    # Original path
    orig = np.array(result['original_path'])
    ax1.plot(orig[:, 0], orig[:, 1], orig[:, 2], 'b--', linewidth=1.5,
             alpha=0.5, label='Original Path')

    # Actual trajectory
    actual = np.array(result['actual_trajectory'])
    ax1.plot(actual[:, 0], actual[:, 1], actual[:, 2], 'r-', linewidth=2,
             label='Actual Trajectory')

    # Replan points
    if result['replan_points']:
        rp = np.array(result['replan_points'])
        ax1.scatter(rp[:, 0], rp[:, 1], rp[:, 2], c='orange', s=80,
                    marker='D', label='Replan Points', zorder=5)

    # Dynamic obstacles at t=0
    for obs in dynamic_obs:
        pos, radius = get_dynamic_obstacle_position(obs, 0)
        if pos is not None:
            u = np.linspace(0, 2*np.pi, 12)
            v = np.linspace(0, np.pi, 8)
            xs = pos[0] + radius * np.outer(np.cos(u), np.sin(v))
            ys = pos[1] + radius * np.outer(np.sin(u), np.sin(v))
            zs = pos[2] + radius * np.outer(np.ones_like(u), np.cos(v))
            ax1.plot_surface(xs, ys, zs, alpha=0.2, color='red')

    ax1.scatter(*start, c='green', s=100, marker='^', label='Start', zorder=5)
    ax1.scatter(*goal, c='red', s=100, marker='*', label='Goal', zorder=5)
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Z (m)')
    ax1.set_title('Dynamic Replanning')
    ax1.legend(fontsize=8)
    ax1.view_init(elev=25, azim=-60)

    # Timeline
    ax2 = fig.add_subplot(122)
    if len(actual) > 1:
        t_arr = np.arange(len(actual)) * 0.5
        ax2.plot(t_arr, actual[:, 0], label='X')
        ax2.plot(t_arr, actual[:, 1], label='Y')
        ax2.plot(t_arr, actual[:, 2], label='Z')
        for rt in result['replan_times']:
            ax2.axvline(rt, color='orange', linestyle='--', alpha=0.7)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Position (m)')
        ax2.set_title('Trajectory with Replan Events (orange lines)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'task5_dynamic_replanning.png'), dpi=150)
    plt.close()
    print(f"Saved to {out_dir}/task5_dynamic_replanning.png")


if __name__ == '__main__':
    main()
