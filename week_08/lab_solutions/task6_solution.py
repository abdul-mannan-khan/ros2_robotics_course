#!/usr/bin/env python3
"""
Week 8 - Task 6 Solution: Energy-Aware Path Planning
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
    return grid, env_params


def wind_field(position):
    """Spatially varying wind model."""
    x, y, z = position
    wx = 2.0 * (z / 20.0) + 0.5 * np.sin(x / 5.0)
    wy = 0.5 * np.cos(y / 3.0) * (z / 20.0)
    wz = -0.2 + 0.3 * np.sin(x / 10.0) * np.sin(y / 10.0)
    return np.array([wx, wy, wz])


def energy_cost(from_pt, to_pt, wind_func, params=None):
    """Compute energy cost for a flight segment."""
    if params is None:
        params = {
            'mass': 2.0, 'drag_coeff': 0.1, 'gravity': 9.81,
            'motor_efficiency': 0.7, 'base_power': 50.0,
            'cruise_speed': 2.0, 'regen_factor': 0.3,
        }

    from_arr = np.array(from_pt, dtype=float)
    to_arr = np.array(to_pt, dtype=float)
    diff = to_arr - from_arr
    dist = np.linalg.norm(diff)
    if dist < 1e-6:
        return 0.0

    direction = diff / dist
    dz = diff[2]

    # Base energy (hover + forward flight)
    base_energy = params['base_power'] * dist / params['cruise_speed']

    # Altitude change
    if dz > 0:
        climb_energy = params['mass'] * params['gravity'] * dz / params['motor_efficiency']
    else:
        climb_energy = -params['mass'] * params['gravity'] * abs(dz) * params['regen_factor']

    # Wind effect at midpoint
    midpoint = (from_arr + to_arr) / 2.0
    wind = wind_func(midpoint)
    # Component of wind along travel direction
    wind_along = np.dot(wind, direction)
    # Headwind increases cost, tailwind decreases
    wind_energy = -wind_along * params['drag_coeff'] * dist * 10.0

    # Aerodynamic drag
    drag_energy = params['drag_coeff'] * dist * params['cruise_speed'] ** 2

    total = base_energy + climb_energy + wind_energy + drag_energy
    return max(total, 0.1)  # Minimum positive cost


def get_neighbors_3d(node, grid):
    """26-connected neighbors."""
    nx, ny, nz = grid.shape
    neighbors = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                nb = (node[0]+dx, node[1]+dy, node[2]+dz)
                if (0 <= nb[0] < nx and 0 <= nb[1] < ny and 0 <= nb[2] < nz
                        and grid[nb] == 0):
                    neighbors.append(nb)
    return neighbors


def distance_astar(grid, start, goal):
    """Standard A* with Euclidean distance cost."""
    open_set = []
    counter = 0
    h = np.sqrt(sum((a-b)**2 for a, b in zip(start, goal)))
    heapq.heappush(open_set, (h, counter, start))
    g_cost = {start: 0.0}
    parent = {start: None}
    closed = set()

    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            node = goal
            while node is not None:
                path.append(node)
                node = parent[node]
            return path[::-1]
        if current in closed:
            continue
        closed.add(current)
        for nb in get_neighbors_3d(current, grid):
            if nb in closed:
                continue
            dx = nb[0]-current[0]; dy = nb[1]-current[1]; dz = nb[2]-current[2]
            move = np.sqrt(dx*dx + dy*dy + dz*dz)
            tg = g_cost[current] + move
            if tg < g_cost.get(nb, float('inf')):
                g_cost[nb] = tg
                parent[nb] = current
                h_nb = np.sqrt(sum((a-b)**2 for a, b in zip(nb, goal)))
                counter += 1
                heapq.heappush(open_set, (tg + h_nb, counter, nb))
    return None


def energy_aware_astar(grid, start, goal, battery_capacity, wind_func, params=None):
    """A* with energy cost function."""
    open_set = []
    counter = 0
    # Heuristic: approximate minimum energy (base_power * distance / speed)
    h_dist = np.sqrt(sum((a-b)**2 for a, b in zip(start, goal)))
    h_energy = 25.0 * h_dist  # rough lower bound
    heapq.heappush(open_set, (h_energy, counter, start))
    g_cost = {start: 0.0}
    parent = {start: None}
    closed = set()

    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            energies = []
            node = goal
            while node is not None:
                path.append(node)
                energies.append(g_cost[node])
                node = parent[node]
            return path[::-1], energies[::-1]
        if current in closed:
            continue
        closed.add(current)
        for nb in get_neighbors_3d(current, grid):
            if nb in closed:
                continue
            e = energy_cost(current, nb, wind_func, params)
            tg = g_cost[current] + e
            if tg > battery_capacity:
                continue  # Prune: exceeds battery
            if tg < g_cost.get(nb, float('inf')):
                g_cost[nb] = tg
                parent[nb] = current
                h_dist = np.sqrt(sum((a-b)**2 for a, b in zip(nb, goal)))
                h_e = 25.0 * h_dist
                counter += 1
                heapq.heappush(open_set, (tg + h_e, counter, nb))

    return None, None


def path_distance(path):
    if path is None or len(path) < 2:
        return 0.0
    return sum(np.sqrt(sum((a-b)**2 for a, b in zip(path[i], path[i+1])))
               for i in range(len(path)-1))


def path_energy(path, wind_func, params=None):
    if path is None or len(path) < 2:
        return 0.0
    return sum(energy_cost(path[i], path[i+1], wind_func, params)
               for i in range(len(path)-1))


def main():
    grid, env_params = load_environment()
    start = tuple(env_params['start'].astype(int))
    goal = tuple(env_params['goal'].astype(int))

    print(f"Start: {start}, Goal: {goal}")
    battery = 100000.0  # 100 kJ

    print("Running distance-optimal A*...")
    t0 = time.time()
    dist_path = distance_astar(grid, start, goal)
    t_dist = time.time() - t0

    print("Running energy-optimal A*...")
    t0 = time.time()
    energy_path, energy_profile = energy_aware_astar(grid, start, goal, battery, wind_field)
    t_energy = time.time() - t0

    if dist_path is None or energy_path is None:
        print("Path finding failed!")
        return

    dist_d = path_distance(dist_path)
    dist_e = path_energy(dist_path, wind_field)
    energy_d = path_distance(energy_path)
    energy_e = path_energy(energy_path, wind_field)

    print(f"\nDistance-optimal: dist={dist_d:.1f}m, energy={dist_e:.0f}J, time={t_dist:.3f}s")
    print(f"Energy-optimal:  dist={energy_d:.1f}m, energy={energy_e:.0f}J, time={t_energy:.3f}s")
    print(f"Energy saving: {(dist_e - energy_e)/dist_e*100:.1f}%")

    # Compute cumulative energy for both paths
    cum_energy_dist = [0.0]
    for i in range(len(dist_path)-1):
        cum_energy_dist.append(cum_energy_dist[-1] + energy_cost(dist_path[i], dist_path[i+1], wind_field))
    cum_energy_energy = [0.0]
    for i in range(len(energy_path)-1):
        cum_energy_energy.append(cum_energy_energy[-1] + energy_cost(energy_path[i], energy_path[i+1], wind_field))

    # Visualization
    fig = plt.figure(figsize=(18, 12))

    # 3D paths
    ax1 = fig.add_subplot(221, projection='3d')
    occ = np.argwhere(grid == 1)
    if len(occ) > 3000:
        occ = occ[np.random.choice(len(occ), 3000, replace=False)]
    ax1.scatter(occ[:, 0], occ[:, 1], occ[:, 2], c='gray', alpha=0.05, s=2)
    dp = np.array(dist_path)
    ep = np.array(energy_path)
    ax1.plot(dp[:, 0], dp[:, 1], dp[:, 2], 'b-', linewidth=2, label=f'Shortest ({dist_d:.0f}m)')
    ax1.plot(ep[:, 0], ep[:, 1], ep[:, 2], 'r-', linewidth=2, label=f'Energy-opt ({energy_d:.0f}m)')
    ax1.scatter(*start, c='green', s=80, marker='^', zorder=5)
    ax1.scatter(*goal, c='red', s=80, marker='*', zorder=5)
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax1.set_title('Path Comparison')
    ax1.legend(fontsize=8)
    ax1.view_init(elev=25, azim=-60)

    # Altitude profile
    ax2 = fig.add_subplot(222)
    ax2.plot(range(len(dp)), dp[:, 2], 'b-', label='Shortest')
    ax2.plot(range(len(ep)), ep[:, 2], 'r-', label='Energy-opt')
    ax2.set_xlabel('Path Step')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude Profile')
    ax2.legend(); ax2.grid(True, alpha=0.3)

    # Cumulative energy
    ax3 = fig.add_subplot(223)
    ax3.plot(range(len(cum_energy_dist)), cum_energy_dist, 'b-', label=f'Shortest ({dist_e:.0f}J)')
    ax3.plot(range(len(cum_energy_energy)), cum_energy_energy, 'r-', label=f'Energy-opt ({energy_e:.0f}J)')
    ax3.set_xlabel('Path Step')
    ax3.set_ylabel('Cumulative Energy (J)')
    ax3.set_title('Energy Consumption')
    ax3.legend(); ax3.grid(True, alpha=0.3)

    # Wind along paths
    ax4 = fig.add_subplot(224)
    wind_dist = [np.linalg.norm(wind_field(p)) for p in dist_path]
    wind_energy = [np.linalg.norm(wind_field(p)) for p in energy_path]
    ax4.plot(range(len(wind_dist)), wind_dist, 'b-', alpha=0.7, label='Shortest path')
    ax4.plot(range(len(wind_energy)), wind_energy, 'r-', alpha=0.7, label='Energy-opt path')
    ax4.set_xlabel('Path Step')
    ax4.set_ylabel('Wind Speed (m/s)')
    ax4.set_title('Wind Speed Along Paths')
    ax4.legend(); ax4.grid(True, alpha=0.3)

    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'task6_energy_aware.png'), dpi=150)
    plt.close()
    print(f"Saved to {out_dir}/task6_energy_aware.png")


if __name__ == '__main__':
    main()
