#!/usr/bin/env python3
"""
Week 5 - Task 2 Solution: A* Path Planning
Complete A* implementation with 8-connected grid and path smoothing.
"""

import os
import numpy as np
import heapq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises', 'data')
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


def load_data():
    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    return grid_map, metadata


def heuristic(a, b):
    """Euclidean distance heuristic."""
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def get_neighbors(node, grid):
    """Get valid 8-connected neighbors."""
    r, c = node
    height, width = grid.shape
    neighbors = []
    SQRT2 = 1.414213562
    for dr, dc, cost in [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
                          (-1,-1,SQRT2),(-1,1,SQRT2),(1,-1,SQRT2),(1,1,SQRT2)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < height and 0 <= nc < width and grid[nr, nc] != 100:
            # Also check that diagonal moves don't cut corners
            if dr != 0 and dc != 0:
                if grid[r+dr, c] == 100 or grid[r, c+dc] == 100:
                    continue
            neighbors.append(((nr, nc), cost))
    return neighbors


def astar(grid, start, goal):
    """A* pathfinding on 2D grid."""
    if grid[start[0], start[1]] == 100 or grid[goal[0], goal[1]] == 100:
        print("Start or goal is in an obstacle!")
        return [], set()

    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    expanded = set()
    counter = 0

    while open_set:
        f, _, current = heapq.heappop(open_set)

        if current in expanded:
            continue
        expanded.add(current)

        if current == goal:
            # Reconstruct path
            path = []
            node = goal
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path, expanded

        for neighbor, move_cost in get_neighbors(current, grid):
            tentative_g = g_score[current] + move_cost
            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                came_from[neighbor] = current
                counter += 1
                heapq.heappush(open_set, (f_score, counter, neighbor))

    return [], expanded  # No path found


def smooth_path(path, grid, weight_smooth=0.5, weight_data=0.5, tolerance=1e-4):
    """Gradient descent path smoothing."""
    if len(path) <= 2:
        return path

    smooth = [list(p) for p in path]
    change = tolerance + 1
    iterations = 0

    while change > tolerance and iterations < 1000:
        change = 0
        for i in range(1, len(smooth) - 1):
            for j in range(2):
                orig = smooth[i][j]
                smooth[i][j] += weight_data * (path[i][j] - smooth[i][j])
                smooth[i][j] += weight_smooth * (smooth[i-1][j] + smooth[i+1][j] - 2*smooth[i][j])
                change += abs(smooth[i][j] - orig)

                # Check collision
                gr, gc = int(round(smooth[i][0])), int(round(smooth[i][1]))
                if (0 <= gr < grid.shape[0] and 0 <= gc < grid.shape[1]
                        and grid[gr, gc] == 100):
                    smooth[i][j] = orig  # Revert
        iterations += 1

    return [(s[0], s[1]) for s in smooth]


def main():
    print("=" * 60)
    print("Task 2: A* Path Planning")
    print("=" * 60)
    print()
    print("A* is the most common global planner in Nav2.")
    print("It uses a heuristic to efficiently search for the shortest path.")
    print()

    grid_map, metadata = load_data()
    resolution = metadata[0]

    start = (10, 10)
    goal = (85, 85)
    print(f"Planning from {start} to {goal} on {grid_map.shape} grid")
    print(f"Resolution: {resolution}m/cell")

    # Ensure start/goal are reachable
    # If goal is in unknown region, adjust
    if grid_map[goal[0], goal[1]] != 0:
        goal = (80, 80)
        print(f"Adjusted goal to {goal} (original was in obstacle/unknown)")

    path, expanded = astar(grid_map, start, goal)

    if not path:
        print("No path found!")
        return

    print(f"\nA* Results:")
    print(f"  Path length: {len(path)} cells")
    print(f"  Nodes expanded: {len(expanded)}")

    # Compute actual path distance
    path_dist = sum(heuristic(path[i], path[i+1]) for i in range(len(path)-1))
    print(f"  Path distance: {path_dist:.1f} cells ({path_dist * resolution:.2f}m)")
    print(f"  Straight-line distance: {heuristic(start, goal):.1f} cells")
    print(f"  Path efficiency: {heuristic(start, goal) / path_dist:.2%}")

    smoothed = smooth_path(path, grid_map)
    smooth_dist = sum(heuristic(smoothed[i], smoothed[i+1]) for i in range(len(smoothed)-1))
    print(f"\nSmoothed path distance: {smooth_dist:.1f} cells ({smooth_dist * resolution:.2f}m)")

    # Visualize
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))

    for ax_idx, (ax, title) in enumerate(zip(axes, ['A* Search', 'Smoothed Path'])):
        ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100)

        if ax_idx == 0 and expanded:
            exp = np.array(list(expanded))
            ax.scatter(exp[:, 1], exp[:, 0], c='lightblue', s=1, alpha=0.3, label='Expanded')

        path_arr = np.array(path)
        ax.plot(path_arr[:, 1], path_arr[:, 0], 'b-', linewidth=2,
                alpha=0.5 if ax_idx == 1 else 1.0, label='A* Path')

        if ax_idx == 1:
            smooth_arr = np.array(smoothed)
            ax.plot(smooth_arr[:, 1], smooth_arr[:, 0], 'g-', linewidth=2.5,
                    label='Smoothed')

        ax.plot(start[1], start[0], 'go', markersize=12, label='Start')
        ax.plot(goal[1], goal[0], 'r*', markersize=15, label='Goal')
        ax.legend(fontsize=10)
        ax.set_title(title, fontsize=13)
        ax.set_xlabel('X (cells)')
        ax.set_ylabel('Y (cells)')

    plt.suptitle(f'A* Path Planning (expanded: {len(expanded)}, '
                 f'path: {path_dist * resolution:.2f}m)', fontsize=14)
    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, 'task2_astar.png')
    plt.savefig(out_path, dpi=100)
    print(f"\nSaved plot to {out_path}")

    print("\n--- Key Concepts ---")
    print("- A* guarantees optimal paths with admissible heuristics")
    print("- Euclidean distance is admissible for 8-connected grids")
    print("- Diagonal moves cost sqrt(2) ~ 1.414")
    print("- Path smoothing reduces unnecessary zig-zag patterns")


if __name__ == '__main__':
    main()
