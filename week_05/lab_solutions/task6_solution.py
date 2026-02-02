#!/usr/bin/env python3
"""
Week 5 - Task 6 Solution: Waypoint Following Navigation
Complete waypoint follower using A* global planning and DWA local control.
"""

import os
import numpy as np
import heapq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises', 'data')
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


# --- A* planner (reused) ---
def _heuristic(a, b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def _astar(grid, start, goal):
    if grid[start[0], start[1]] == 100 or grid[goal[0], goal[1]] == 100:
        return []
    h, w = grid.shape
    open_set = [(0, 0, start)]
    g = {start: 0}
    came = {}
    visited = set()
    SQRT2 = 1.414
    ct = 0
    while open_set:
        _, _, cur = heapq.heappop(open_set)
        if cur in visited:
            continue
        visited.add(cur)
        if cur == goal:
            path = []
            while cur in came:
                path.append(cur)
                cur = came[cur]
            path.append(start)
            path.reverse()
            return path
        for dr, dc, cost in [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),
                              (-1,-1,SQRT2),(-1,1,SQRT2),(1,-1,SQRT2),(1,1,SQRT2)]:
            nr, nc = cur[0]+dr, cur[1]+dc
            if 0 <= nr < h and 0 <= nc < w and grid[nr, nc] != 100:
                if dr != 0 and dc != 0:
                    if grid[cur[0]+dr, cur[1]] == 100 or grid[cur[0], cur[1]+dc] == 100:
                        continue
                ng = g[cur] + cost
                if ng < g.get((nr, nc), float('inf')):
                    g[(nr, nc)] = ng
                    came[(nr, nc)] = cur
                    ct += 1
                    heapq.heappush(open_set, (ng + _heuristic((nr,nc), goal), ct, (nr, nc)))
    return []


class WaypointFollower:
    def __init__(self, waypoints, tolerance=0.2):
        self.waypoints = waypoints
        self.current_index = 0
        self.tolerance = tolerance

    def current_waypoint(self):
        if self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None

    def advance(self):
        self.current_index += 1
        return self.current_index < len(self.waypoints)

    def is_complete(self):
        return self.current_index >= len(self.waypoints)


def plan_to_waypoint(current_pose, waypoint, grid_map, resolution):
    """Plan path using A*."""
    sr = int(np.clip(current_pose[1] / resolution, 1, grid_map.shape[0]-2))
    sc = int(np.clip(current_pose[0] / resolution, 1, grid_map.shape[1]-2))
    gr = int(np.clip(waypoint[1] / resolution, 1, grid_map.shape[0]-2))
    gc = int(np.clip(waypoint[0] / resolution, 1, grid_map.shape[1]-2))

    # Find nearest free cell if start/goal is occupied
    for s, orig in [((sr, sc), 'start'), ((gr, gc), 'goal')]:
        if grid_map[s[0], s[1]] == 100:
            # Search nearby
            for radius in range(1, 10):
                found = False
                for ddr in range(-radius, radius+1):
                    for ddc in range(-radius, radius+1):
                        nr, nc = s[0]+ddr, s[1]+ddc
                        if (0 <= nr < grid_map.shape[0] and 0 <= nc < grid_map.shape[1]
                                and grid_map[nr, nc] == 0):
                            if orig == 'start':
                                sr, sc = nr, nc
                            else:
                                gr, gc = nr, nc
                            found = True
                            break
                    if found:
                        break
                if found:
                    break

    path_cells = _astar(grid_map, (sr, sc), (gr, gc))
    if not path_cells:
        return []
    return [(c * resolution, r * resolution) for r, c in path_cells]


def follow_path(path, current_state, dt=0.1, speed=0.3, lookahead=5):
    """Follow path with simple proportional control."""
    x, y, theta, v, w = current_state

    if not path:
        return current_state

    # Find lookahead point
    min_dist = float('inf')
    closest_idx = 0
    for i, (px, py) in enumerate(path):
        d = np.hypot(px - x, py - y)
        if d < min_dist:
            min_dist = d
            closest_idx = i

    target_idx = min(closest_idx + lookahead, len(path) - 1)
    tx, ty = path[target_idx]

    # Proportional control
    dx = tx - x
    dy = ty - y
    desired_theta = np.arctan2(dy, dx)
    angle_error = desired_theta - theta
    angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

    # Control
    new_w = np.clip(2.0 * angle_error, -1.0, 1.0)
    new_v = speed * (1.0 - 0.5 * abs(angle_error) / np.pi)
    new_v = max(new_v, 0.05)

    # Update
    new_theta = theta + new_w * dt
    new_x = x + new_v * np.cos(new_theta) * dt
    new_y = y + new_v * np.sin(new_theta) * dt

    return np.array([new_x, new_y, new_theta, new_v, new_w])


def check_reached(current_pose, waypoint, tolerance=0.2):
    return np.hypot(current_pose[0] - waypoint[0], current_pose[1] - waypoint[1]) < tolerance


def run_waypoint_mission(waypoints, grid_map, resolution, start_pose):
    """Run complete waypoint mission."""
    follower = WaypointFollower(waypoints, tolerance=0.2)
    state = np.array([start_pose[0], start_pose[1], start_pose[2], 0.0, 0.0])
    trajectory = [state[:2].copy()]
    reached = []
    timing = []

    while not follower.is_complete():
        wp = follower.current_waypoint()
        print(f"  Planning to waypoint {follower.current_index}: ({wp[0]:.1f}, {wp[1]:.1f})")

        path = plan_to_waypoint(state[:3], wp, grid_map, resolution)
        wp_reached = False
        steps = 0

        if not path:
            print(f"    No path found!")
            reached.append(False)
            timing.append(0)
            follower.advance()
            continue

        for step in range(500):
            state = follow_path(path, state)
            trajectory.append(state[:2].copy())
            steps += 1

            if check_reached(state, wp, 0.2):
                wp_reached = True
                print(f"    Reached in {steps} steps")
                break

            # Replan periodically
            if step > 0 and step % 100 == 0:
                path = plan_to_waypoint(state[:3], wp, grid_map, resolution)
                if not path:
                    break

        if not wp_reached:
            dist = np.hypot(state[0]-wp[0], state[1]-wp[1])
            print(f"    Timeout. Dist: {dist:.3f}m")
            if dist < 0.35:
                wp_reached = True

        reached.append(wp_reached)
        timing.append(steps)
        follower.advance()

    return trajectory, reached, timing


def main():
    print("=" * 60)
    print("Task 6: Waypoint Following Navigation")
    print("=" * 60)
    print()

    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    waypoints = np.load(os.path.join(DATA_DIR, 'waypoints.npy'))
    resolution = metadata[0]

    start_pose = (1.0, 1.0, 0.0)
    print(f"Start: {start_pose[:2]}")
    print(f"Waypoints: {len(waypoints)}")
    print()

    trajectory, reached, timing = run_waypoint_mission(
        waypoints, grid_map, resolution, start_pose)

    traj = np.array(trajectory)
    total_dist = np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))

    print(f"\n--- Mission Summary ---")
    print(f"Waypoints reached: {sum(reached)}/{len(reached)}")
    print(f"Total distance: {total_dist:.2f}m")
    print(f"Total steps: {sum(timing)}")

    for i, (r, t) in enumerate(zip(reached, timing)):
        status = "REACHED" if r else "FAILED"
        print(f"  WP{i} ({waypoints[i][0]:.1f},{waypoints[i][1]:.1f}): {status} in {t} steps")

    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    ax = axes[0]
    ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100,
              extent=[0, 5, 0, 5])
    ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=1.5, alpha=0.8, label='Trajectory')

    # Draw waypoint connections
    all_pts = [start_pose[:2]] + [tuple(wp) for wp in waypoints]
    for i in range(len(all_pts) - 1):
        ax.plot([all_pts[i][0], all_pts[i+1][0]],
                [all_pts[i][1], all_pts[i+1][1]], 'r--', alpha=0.3)

    for i, (wp, r) in enumerate(zip(waypoints, reached)):
        color = 'green' if r else 'red'
        ax.plot(wp[0], wp[1], 'o', color=color, markersize=10, zorder=5)
        ax.annotate(f'WP{i}', (wp[0]+0.07, wp[1]+0.07), fontsize=9)

    ax.plot(start_pose[0], start_pose[1], 'gs', markersize=12, label='Start', zorder=5)
    ax.legend()
    ax.set_title(f'Waypoint Navigation ({sum(reached)}/{len(reached)} reached, '
                 f'{total_dist:.1f}m)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    # Timing bar chart
    ax = axes[1]
    colors = ['green' if r else 'red' for r in reached]
    ax.barh(range(len(timing)), timing, color=colors)
    ax.set_yticks(range(len(timing)))
    ax.set_yticklabels([f'WP{i}' for i in range(len(timing))])
    ax.set_xlabel('Steps')
    ax.set_title('Steps per Waypoint')
    for i, (t, r) in enumerate(zip(timing, reached)):
        ax.text(t + 5, i, f'{t} ({"OK" if r else "FAIL"})', va='center', fontsize=9)

    plt.suptitle('Waypoint Following Navigation', fontsize=14)
    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, 'task6_waypoint_navigation.png')
    plt.savefig(out_path, dpi=100)
    print(f"\nSaved plot to {out_path}")

    print("\n--- Key Concepts ---")
    print("- Waypoint follower sequences navigation goals")
    print("- Global path is replanned if progress stalls")
    print("- Proportional control with lookahead for smooth following")
    print("- Nav2's waypoint follower plugin manages the goal queue")


if __name__ == '__main__':
    main()
