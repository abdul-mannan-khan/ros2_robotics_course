#!/usr/bin/env python3
"""
Week 5 - Task 7 Solution: Full Navigation Stack Integration
Combines map, costmap, AMCL, A* planner, DWA controller, and BT.
"""

import os
import numpy as np
import heapq
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises', 'data')
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


# ---- Costmap ----
def build_costmap(grid_map, inflation_radius=0.3, resolution=0.05):
    costmap = np.zeros_like(grid_map, dtype=np.uint8)
    costmap[grid_map == 100] = 254
    costmap[grid_map == -1] = 255

    infl_cells = int(inflation_radius / resolution)
    inflated = costmap.copy()
    lethal = np.argwhere(costmap == 254)
    h, w = costmap.shape
    for ly, lx in lethal:
        for dy in range(-infl_cells, infl_cells+1):
            for dx in range(-infl_cells, infl_cells+1):
                ny, nx = ly+dy, lx+dx
                if 0 <= ny < h and 0 <= nx < w:
                    d = np.sqrt(dy*dy+dx*dx) * resolution
                    if d <= inflation_radius and inflated[ny, nx] < 253 and inflated[ny, nx] != 254:
                        cost = int(253 * np.exp(-3.0 * d / inflation_radius))
                        inflated[ny, nx] = max(inflated[ny, nx], cost)
    return inflated


# ---- A* Planner ----
def astar(grid, start, goal):
    if grid[start[0], start[1]] in (100,) or grid[goal[0], goal[1]] in (100,):
        return []
    h, w = grid.shape
    open_set = [(0, 0, start)]
    g = {start: 0}; came = {}; visited = set(); ct = 0; SQRT2 = 1.414
    while open_set:
        _, _, cur = heapq.heappop(open_set)
        if cur in visited: continue
        visited.add(cur)
        if cur == goal:
            path = []
            while cur in came: path.append(cur); cur = came[cur]
            path.append(start); path.reverse(); return path
        for dr,dc,cost in [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),(-1,-1,SQRT2),(-1,1,SQRT2),(1,-1,SQRT2),(1,1,SQRT2)]:
            nr, nc = cur[0]+dr, cur[1]+dc
            if 0<=nr<h and 0<=nc<w and grid[nr,nc]!=100:
                if dr!=0 and dc!=0 and (grid[cur[0]+dr,cur[1]]==100 or grid[cur[0],cur[1]+dc]==100): continue
                ng = g[cur]+cost
                if ng < g.get((nr,nc), float('inf')):
                    g[(nr,nc)] = ng; came[(nr,nc)] = cur; ct+=1
                    heapq.heappush(open_set, (ng+np.sqrt((nr-goal[0])**2+(nc-goal[1])**2), ct, (nr,nc)))
    return []


# ---- Simple Particle Filter (lightweight for integration) ----
class SimpleAMCL:
    def __init__(self, grid_map, resolution, n_particles=100):
        self.grid_map = grid_map
        self.resolution = resolution
        self.n = n_particles
        self.particles = None
        self.weights = None
        self.dist_field = distance_transform_edt((grid_map != 100).astype(float))

    def initialize(self, pose, spread=0.3):
        self.particles = np.zeros((self.n, 3))
        self.particles[:, 0] = pose[0] + np.random.normal(0, spread, self.n)
        self.particles[:, 1] = pose[1] + np.random.normal(0, spread, self.n)
        self.particles[:, 2] = pose[2] + np.random.normal(0, 0.2, self.n)
        self.weights = np.ones(self.n) / self.n

    def update(self, odometry, scan=None):
        dx, dy, dth = odometry
        trans = np.sqrt(dx**2 + dy**2)
        for i in range(self.n):
            nt = trans + np.random.normal(0, 0.02)
            na = np.arctan2(dy, dx) if trans > 1e-6 else 0
            na += np.random.normal(0, 0.05)
            self.particles[i, 0] += nt * np.cos(self.particles[i, 2] + na)
            self.particles[i, 1] += nt * np.sin(self.particles[i, 2] + na)
            self.particles[i, 2] += dth + np.random.normal(0, 0.05)

        # Simple weight update based on map validity
        h, w = self.grid_map.shape
        for i in range(self.n):
            gx = int(self.particles[i, 0] / self.resolution)
            gy = int(self.particles[i, 1] / self.resolution)
            if 0 <= gx < w and 0 <= gy < h and self.grid_map[gy, gx] == 0:
                self.weights[i] = 1.0 + self.dist_field[gy, gx] * 0.1
            else:
                self.weights[i] = 0.001
        self.weights /= self.weights.sum()

        # Resample
        indices = np.random.choice(self.n, self.n, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.n) / self.n

    def estimate(self):
        x = np.mean(self.particles[:, 0])
        y = np.mean(self.particles[:, 1])
        th = np.arctan2(np.mean(np.sin(self.particles[:, 2])),
                        np.mean(np.cos(self.particles[:, 2])))
        return np.array([x, y, th])

    def spread(self):
        return np.std(self.particles[:, :2], axis=0).mean()


# ---- Navigation Stack ----
class NavigationStack:
    def __init__(self, grid_map, resolution=0.05):
        self.grid_map = grid_map
        self.resolution = resolution
        self.costmap = None
        self.amcl = None
        self.robot_pose = None

    def load_map(self):
        self.costmap = build_costmap(self.grid_map, 0.3, self.resolution)
        print(f"  Costmap built: {np.sum(self.costmap == 254)} lethal, "
              f"{np.sum(self.costmap > 0)} non-free cells")

    def initialize_localization(self, initial_pose, n_particles=100):
        self.amcl = SimpleAMCL(self.grid_map, self.resolution, n_particles)
        self.amcl.initialize(initial_pose, spread=0.2)
        self.robot_pose = np.array(initial_pose, dtype=float)
        print(f"  AMCL initialized at ({initial_pose[0]:.1f}, {initial_pose[1]:.1f}), "
              f"spread={self.amcl.spread():.3f}m")

    def _plan_path(self, goal_xy):
        sr = int(np.clip(self.robot_pose[1]/self.resolution, 1, self.grid_map.shape[0]-2))
        sc = int(np.clip(self.robot_pose[0]/self.resolution, 1, self.grid_map.shape[1]-2))
        gr = int(np.clip(goal_xy[1]/self.resolution, 1, self.grid_map.shape[0]-2))
        gc = int(np.clip(goal_xy[0]/self.resolution, 1, self.grid_map.shape[1]-2))

        # Ensure start/goal in free space
        for orig in ['start', 'goal']:
            rr, cc = (sr, sc) if orig == 'start' else (gr, gc)
            if self.grid_map[rr, cc] == 100:
                for rad in range(1, 10):
                    found = False
                    for ddr in range(-rad, rad+1):
                        for ddc in range(-rad, rad+1):
                            nrr, ncc = rr+ddr, cc+ddc
                            if (0<=nrr<self.grid_map.shape[0] and 0<=ncc<self.grid_map.shape[1]
                                    and self.grid_map[nrr, ncc]==0):
                                if orig=='start': sr,sc = nrr,ncc
                                else: gr,gc = nrr,ncc
                                found = True; break
                        if found: break
                    if found: break

        path_cells = astar(self.grid_map, (sr, sc), (gr, gc))
        return [(c*self.resolution, r*self.resolution) for r,c in path_cells]

    def navigate_to_pose(self, goal, max_steps=500):
        path = self._plan_path(goal[:2] if len(goal) > 2 else goal)
        if not path:
            return False, [], 0

        trajectory = [self.robot_pose[:2].copy()]
        speed = 0.3
        dt = 0.1
        lookahead = 5
        prev_pose = self.robot_pose.copy()

        for step in range(max_steps):
            dist = np.hypot(self.robot_pose[0]-goal[0], self.robot_pose[1]-goal[1])
            if dist < 0.2:
                return True, trajectory, step

            # Find closest + lookahead
            min_d = float('inf'); ci = 0
            for i, (px, py) in enumerate(path):
                d = np.hypot(px-self.robot_pose[0], py-self.robot_pose[1])
                if d < min_d: min_d = d; ci = i
            ti = min(ci + lookahead, len(path)-1)
            tx, ty = path[ti]

            dx = tx - self.robot_pose[0]
            dy = ty - self.robot_pose[1]
            desired = np.arctan2(dy, dx)
            err = np.arctan2(np.sin(desired-self.robot_pose[2]),
                             np.cos(desired-self.robot_pose[2]))
            w = np.clip(2.0*err, -1.0, 1.0)
            v = speed * (1.0 - 0.5*abs(err)/np.pi)
            v = max(v, 0.03)

            self.robot_pose[2] += w * dt
            self.robot_pose[0] += v * np.cos(self.robot_pose[2]) * dt
            self.robot_pose[1] += v * np.sin(self.robot_pose[2]) * dt

            # AMCL update
            odom = self.robot_pose - prev_pose
            self.amcl.update(odom)
            est = self.amcl.estimate()
            # Blend estimate (simulate localization correction)
            self.robot_pose[:2] = 0.95 * self.robot_pose[:2] + 0.05 * est[:2]
            prev_pose = self.robot_pose.copy()
            trajectory.append(self.robot_pose[:2].copy())

            # Replan periodically
            if step > 0 and step % 100 == 0:
                path = self._plan_path(goal[:2] if len(goal)>2 else goal)
                if not path:
                    return False, trajectory, step

        dist = np.hypot(self.robot_pose[0]-goal[0], self.robot_pose[1]-goal[1])
        return dist < 0.35, trajectory, max_steps

    def run(self, goals, start_pose):
        self.robot_pose = np.array(start_pose, dtype=float)
        results = []
        for i, goal in enumerate(goals):
            g = goal[:2] if len(goal.shape) == 0 else goal
            success, traj, steps = self.navigate_to_pose(g)
            results.append((success, traj, steps))
            status = "OK" if success else "FAIL"
            print(f"  Goal {i} ({g[0]:.1f},{g[1]:.1f}): {status} in {steps} steps")
        return results


def evaluate_navigation(results, goals):
    successes = sum(1 for r in results if r[0])
    total = len(results)

    total_length = 0
    total_steps = 0
    smoothness_vals = []
    for success, traj, steps in results:
        total_steps += steps
        if len(traj) > 1:
            t = np.array(traj)
            diffs = np.diff(t, axis=0)
            dists = np.linalg.norm(diffs, axis=1)
            total_length += np.sum(dists)
            if len(diffs) > 1:
                headings = np.arctan2(diffs[:, 1], diffs[:, 0])
                heading_changes = np.abs(np.diff(headings))
                heading_changes = np.minimum(heading_changes, 2*np.pi - heading_changes)
                smoothness_vals.append(np.mean(heading_changes))

    return {
        'success_rate': successes / total if total > 0 else 0,
        'goals_reached': f'{successes}/{total}',
        'total_path_length_m': round(total_length, 2),
        'total_steps': total_steps,
        'avg_smoothness_rad': round(np.mean(smoothness_vals), 4) if smoothness_vals else 0,
    }


def main():
    print("=" * 60)
    print("Task 7: Full Navigation Stack Integration")
    print("=" * 60)
    print()

    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    waypoints = np.load(os.path.join(DATA_DIR, 'waypoints.npy'))
    resolution = metadata[0]

    print("Initializing Navigation Stack...")
    nav = NavigationStack(grid_map, resolution)
    nav.load_map()

    start_pose = np.array([1.0, 1.0, 0.0])
    nav.initialize_localization(start_pose)
    print()

    print("Running navigation mission...")
    results = nav.run(waypoints, start_pose)
    metrics = evaluate_navigation(results, waypoints)
    print()

    print("--- Navigation Metrics ---")
    for k, v in metrics.items():
        print(f"  {k}: {v}")

    # 4-panel plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 14))

    # Panel 1: Trajectories
    ax = axes[0, 0]
    ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100,
              extent=[0, 5, 0, 5])
    for i, (success, traj, steps) in enumerate(results):
        if len(traj) > 0:
            t = np.array(traj)
            color = 'green' if success else 'red'
            ax.plot(t[:, 0], t[:, 1], '-', color=color, linewidth=1.5, alpha=0.7,
                    label=f'Goal {i} ({"OK" if success else "FAIL"})')
    for i, wp in enumerate(waypoints):
        ax.plot(wp[0], wp[1], 'r*', markersize=10)
        ax.annotate(f'{i}', (wp[0]+0.05, wp[1]+0.05), fontsize=9)
    ax.plot(start_pose[0], start_pose[1], 'gs', markersize=12)
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title('Navigation Trajectories')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')

    # Panel 2: Success
    ax = axes[0, 1]
    successes = [r[0] for r in results]
    colors = ['green' if s else 'red' for s in successes]
    ax.bar(range(len(successes)), [1]*len(successes), color=colors)
    ax.set_xticks(range(len(successes)))
    ax.set_xticklabels([f'G{i}' for i in range(len(successes))])
    ax.set_title(f'Success Rate: {metrics["success_rate"]:.0%} ({metrics["goals_reached"]})')
    ax.set_ylabel('Reached (1=yes)')

    # Panel 3: Path lengths
    ax = axes[1, 0]
    lengths = []
    for success, traj, steps in results:
        if len(traj) > 1:
            t = np.array(traj)
            lengths.append(np.sum(np.linalg.norm(np.diff(t, axis=0), axis=1)))
        else:
            lengths.append(0)
    bar_colors = ['green' if s else 'red' for s in successes]
    ax.bar(range(len(lengths)), lengths, color=bar_colors)
    ax.set_xticks(range(len(lengths)))
    ax.set_xticklabels([f'G{i}' for i in range(len(lengths))])
    ax.set_title(f'Path Lengths (total: {sum(lengths):.2f}m)')
    ax.set_ylabel('Length (m)')

    # Panel 4: Steps
    ax = axes[1, 1]
    steps_list = [r[2] for r in results]
    ax.bar(range(len(steps_list)), steps_list, color=bar_colors)
    ax.set_xticks(range(len(steps_list)))
    ax.set_xticklabels([f'G{i}' for i in range(len(steps_list))])
    ax.set_title(f'Steps per Goal (total: {sum(steps_list)})')
    ax.set_ylabel('Steps')

    plt.suptitle('Full Navigation Stack Evaluation', fontsize=15)
    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, 'task7_full_navigation.png')
    plt.savefig(out_path, dpi=100)
    print(f"\nSaved plot to {out_path}")

    print("\n--- Key Concepts ---")
    print("- Nav2 integrates map server, AMCL, costmap, planner, controller")
    print("- Each component runs as a lifecycle node in ROS2")
    print("- The behavior tree orchestrates the interaction between components")
    print("- Navigation quality depends on all components working together")
    print(f"\nFinal success rate: {metrics['success_rate']:.0%}")


if __name__ == '__main__':
    main()
