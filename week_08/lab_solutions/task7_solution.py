#!/usr/bin/env python3
"""
Week 8 - Task 7 Solution: Complete 3D Planning Pipeline
Full drone planning system with environment, planning, trajectory optimization,
dynamic replanning, and evaluation.
"""

import numpy as np
import heapq
import os
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage import binary_dilation


def load_all_data():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises', 'data')
    return {
        'grid': np.load(os.path.join(data_dir, 'voxel_grid.npy')),
        'env_params': np.load(os.path.join(data_dir, 'env_params.npy'), allow_pickle=True).item(),
        'obstacles': np.load(os.path.join(data_dir, 'obstacles.npy')),
        'no_fly_zones': np.load(os.path.join(data_dir, 'no_fly_zones.npy')),
        'waypoints': np.load(os.path.join(data_dir, 'waypoints.npy')),
        'dynamic_obstacles': np.load(os.path.join(data_dir, 'dynamic_obstacles.npy')),
    }


class DronePlanner:
    """Complete 3D drone planning system."""

    def __init__(self, data):
        self.raw_grid = data['grid'].copy()
        self.env_params = data['env_params']
        self.obstacles = data['obstacles']
        self.no_fly_zones = data['no_fly_zones']
        self.waypoints = data['waypoints']
        self.dynamic_obstacles = data['dynamic_obstacles']
        self.grid = None
        self.inflated_grid = None
        self.metrics = {}

    def build_environment(self):
        """Process raw grid with no-fly zones."""
        self.grid = self.raw_grid.copy()
        # No-fly zones already baked into grid by generator, but verify
        for nfz in self.no_fly_zones:
            cx, cy, radius, z_min, z_max = nfz
            nx, ny, nz = self.grid.shape
            for x in range(max(0, int(cx - radius)), min(nx, int(cx + radius) + 1)):
                for y in range(max(0, int(cy - radius)), min(ny, int(cy + radius) + 1)):
                    if (x - cx)**2 + (y - cy)**2 <= radius**2:
                        z_lo = max(1, int(z_min))
                        z_hi = min(nz, int(z_max) + 1)
                        self.grid[x, y, z_lo:z_hi] = 1
        return self.grid

    def inflate_obstacles(self, radius=1.0):
        """Inflate obstacles using morphological dilation."""
        r = int(np.ceil(radius))
        struct = np.zeros((2*r+1, 2*r+1, 2*r+1), dtype=bool)
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                for dz in range(-r, r+1):
                    if dx*dx + dy*dy + dz*dz <= radius*radius:
                        struct[dx+r, dy+r, dz+r] = True
        self.inflated_grid = binary_dilation(self.grid, structure=struct).astype(np.uint8)
        return self.inflated_grid

    def plan_global_path(self, start, goal, method='astar'):
        """Plan global path using A* or RRT*."""
        grid = self.inflated_grid if self.inflated_grid is not None else self.grid
        start_t = tuple(np.array(start).astype(int))
        goal_t = tuple(np.array(goal).astype(int))

        t0 = time.time()
        if method == 'astar':
            path = self._astar(grid, start_t, goal_t)
        else:
            path = self._rrt_star(grid, np.array(start), np.array(goal))
        elapsed = time.time() - t0
        self.metrics[f'{method}_time'] = elapsed
        return path

    def _astar(self, grid, start, goal):
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
                                h_nb = np.sqrt(sum((a-b)**2 for a, b in zip(nb, goal)))
                                counter += 1
                                heapq.heappush(open_set, (tg + h_nb, counter, nb))
        return None

    def _rrt_star(self, grid, start, goal, max_iter=6000, step=3.0, rewire_r=6.0):
        bounds = self.env_params['bounds']
        positions = [start.copy()]
        parents = [-1]
        costs = [0.0]
        best_goal_idx = None
        best_cost = float('inf')

        for _ in range(max_iter):
            if np.random.random() < 0.15:
                sample = goal.copy()
            else:
                sample = np.array([np.random.uniform(bounds[0], bounds[1]),
                                   np.random.uniform(bounds[2], bounds[3]),
                                   np.random.uniform(bounds[4], bounds[5])])

            pos_arr = np.array(positions)
            near_idx = int(np.argmin(np.linalg.norm(pos_arr - sample, axis=1)))
            from_pt = positions[near_idx]
            diff = sample - from_pt
            d = np.linalg.norm(diff)
            if d < 1e-6:
                continue
            new_pt = from_pt + diff / d * min(step, d)

            if not self._collision_free(grid, from_pt, new_pt):
                continue

            # Best parent
            nearby = [i for i, p in enumerate(positions) if np.linalg.norm(np.array(p) - new_pt) <= rewire_r]
            bp = near_idx
            bc = costs[near_idx] + np.linalg.norm(new_pt - from_pt)
            for ni in nearby:
                c = costs[ni] + np.linalg.norm(new_pt - np.array(positions[ni]))
                if c < bc and self._collision_free(grid, np.array(positions[ni]), new_pt):
                    bp = ni; bc = c

            positions.append(new_pt)
            parents.append(bp)
            costs.append(bc)
            new_idx = len(positions) - 1

            # Rewire
            for ni in nearby:
                if ni == bp:
                    continue
                nc = bc + np.linalg.norm(new_pt - np.array(positions[ni]))
                if nc < costs[ni] and self._collision_free(grid, new_pt, np.array(positions[ni])):
                    parents[ni] = new_idx
                    costs[ni] = nc

            if np.linalg.norm(new_pt - goal) < 3.0:
                if self._collision_free(grid, new_pt, goal):
                    gc = bc + np.linalg.norm(new_pt - goal)
                    if gc < best_cost:
                        positions.append(goal.copy())
                        parents.append(new_idx)
                        costs.append(gc)
                        best_goal_idx = len(positions) - 1
                        best_cost = gc

        if best_goal_idx is None:
            return None
        path = []
        idx = best_goal_idx
        while idx != -1:
            path.append(positions[idx])
            idx = parents[idx]
        return path[::-1]

    def _collision_free(self, grid, from_pt, to_pt):
        diff = to_pt - from_pt
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            return True
        n = max(int(dist / 0.5), 2)
        for i in range(n + 1):
            pt = from_pt + (i / n) * diff
            ix, iy, iz = int(round(pt[0])), int(round(pt[1])), int(round(pt[2]))
            if (ix < 0 or iy < 0 or iz < 0 or ix >= grid.shape[0]
                    or iy >= grid.shape[1] or iz >= grid.shape[2] or grid[ix, iy, iz] == 1):
                return False
        return True

    def optimize_trajectory(self, path, v_max=5.0, a_max=10.0):
        """Minimum-snap trajectory optimization."""
        if path is None or len(path) < 2:
            return None

        # Subsample path for trajectory optimization (too many waypoints is slow)
        if len(path) > 20:
            indices = np.linspace(0, len(path)-1, 20).astype(int)
            wp = np.array([path[i] for i in indices])
        else:
            wp = np.array(path)

        # Time allocation
        dists = np.linalg.norm(np.diff(wp, axis=0), axis=1)
        seg_times = dists / 2.0  # average speed 2 m/s
        times = np.concatenate([[0.0], np.cumsum(seg_times)])

        # Per-axis minimum snap
        n_seg = len(wp) - 1
        n_coeffs = 8
        trajectories = {}

        for axis in range(3):
            pts = wp[:, axis]
            n_vars = n_seg * n_coeffs
            rows = []
            rhs_vals = []

            def poly_row(seg, t, deriv):
                row = np.zeros(n_vars)
                base = seg * n_coeffs
                for k in range(deriv, n_coeffs):
                    f = 1.0
                    for j in range(k, k - deriv, -1):
                        f *= j
                    row[base + k] = f * (t ** (k - deriv))
                return row

            for s in range(n_seg):
                dt = times[s+1] - times[s]
                rows.append(poly_row(s, 0.0, 0)); rhs_vals.append(pts[s])
                rows.append(poly_row(s, dt, 0)); rhs_vals.append(pts[s+1])

            for d in range(1, 4):
                rows.append(poly_row(0, 0.0, d)); rhs_vals.append(0.0)
            dt_last = times[-1] - times[-2]
            for d in range(1, 4):
                rows.append(poly_row(n_seg-1, dt_last, d)); rhs_vals.append(0.0)

            for w in range(1, n_seg):
                dt_prev = times[w] - times[w-1]
                for d in range(1, 7):
                    row = poly_row(w-1, dt_prev, d) - poly_row(w, 0.0, d)
                    rows.append(row); rhs_vals.append(0.0)

            A = np.array(rows)
            b = np.array(rhs_vals)

            if A.shape[0] < n_vars:
                H = np.zeros((n_vars, n_vars))
                for s in range(n_seg):
                    dt = times[s+1] - times[s]
                    base = s * n_coeffs
                    for i in range(4, n_coeffs):
                        for j in range(4, n_coeffs):
                            fi = 1.0
                            for k in range(i, i-4, -1): fi *= k
                            fj = 1.0
                            for k in range(j, j-4, -1): fj *= k
                            pw = i + j - 8 + 1
                            H[base+i, base+j] = fi * fj * (dt ** pw) / pw

                n_eq = A.shape[0]
                KKT = np.zeros((n_vars + n_eq, n_vars + n_eq))
                KKT[:n_vars, :n_vars] = H + 1e-10 * np.eye(n_vars)
                KKT[:n_vars, n_vars:] = A.T
                KKT[n_vars:, :n_vars] = A
                rhs_kkt = np.zeros(n_vars + n_eq)
                rhs_kkt[n_vars:] = b
                sol = np.linalg.solve(KKT, rhs_kkt)
                coeffs = sol[:n_vars]
            else:
                coeffs, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

            trajectories[axis] = [coeffs[s*n_coeffs:(s+1)*n_coeffs] for s in range(n_seg)]

        # Evaluate
        dt_eval = 0.02
        t_eval = np.arange(0, times[-1], dt_eval)
        pos = np.zeros((len(t_eval), 3))
        vel = np.zeros((len(t_eval), 3))
        acc = np.zeros((len(t_eval), 3))

        for i, t in enumerate(t_eval):
            seg = 0
            for s in range(n_seg):
                if t >= times[s] and (t < times[s+1] or s == n_seg - 1):
                    seg = s; break
            t_loc = t - times[seg]
            for axis in range(3):
                c = trajectories[axis][seg]
                for deriv, arr in [(0, pos), (1, vel), (2, acc)]:
                    val = 0.0
                    for k in range(deriv, len(c)):
                        f = 1.0
                        for j in range(k, k-deriv, -1): f *= j
                        val += f * c[k] * (t_loc ** (k - deriv))
                    arr[i, axis] = val

        return {'time': t_eval, 'position': pos, 'velocity': vel, 'acceleration': acc,
                'waypoints': wp, 'wp_times': times}

    def simulate_execution(self, trajectory, dynamic_obstacles, dt=0.5):
        """Simple trajectory execution simulation."""
        if trajectory is None:
            return None
        pos = trajectory['position']
        t = trajectory['time']
        actual = [pos[0].copy()]
        replan_count = 0

        for i in range(1, len(pos)):
            pt = pos[i]
            # Check dynamic obstacles
            collision = False
            for obs in dynamic_obstacles:
                sx, sy, sz, vx, vy, vz, r, ts, te = obs
                if t[i] < ts or t[i] > te:
                    continue
                dt_obs = t[i] - ts
                op = np.array([sx + vx*dt_obs, sy + vy*dt_obs, sz + vz*dt_obs])
                if np.linalg.norm(pt - op) < r + 1.0:
                    collision = True
                    break
            if collision:
                # Simple avoidance: offset vertically
                pt = pt.copy()
                pt[2] = min(pt[2] + 2.0, 19.0)
                replan_count += 1
            actual.append(pt.copy())

        return {'actual': np.array(actual), 'replan_count': replan_count}

    def evaluate(self, trajectory, path):
        """Evaluate trajectory quality."""
        if trajectory is None:
            return {}
        pos = trajectory['position']
        vel = trajectory['velocity']
        acc = trajectory['acceleration']

        # Path length
        diffs = np.diff(pos, axis=0)
        length = np.sum(np.linalg.norm(diffs, axis=1))

        # Smoothness (integral of squared acceleration)
        dt = trajectory['time'][1] - trajectory['time'][0] if len(trajectory['time']) > 1 else 0.02
        smoothness = np.sum(np.linalg.norm(acc, axis=1)**2) * dt

        # Energy (simplified)
        energy = 0.0
        for i in range(len(pos) - 1):
            d = np.linalg.norm(pos[i+1] - pos[i])
            dz = pos[i+1, 2] - pos[i, 2]
            energy += 50.0 * d / 2.0 + (2.0 * 9.81 * max(dz, 0) / 0.7)

        # Min clearance
        grid = self.grid
        min_clearance = float('inf')
        for p in pos[::10]:
            ix, iy, iz = int(round(p[0])), int(round(p[1])), int(round(p[2]))
            for r in range(1, 10):
                found_obs = False
                for dx in range(-r, r+1):
                    for dy in range(-r, r+1):
                        for dz_val in range(-r, r+1):
                            nx, ny, nz = ix+dx, iy+dy, iz+dz_val
                            if (0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]
                                    and 0 <= nz < grid.shape[2] and grid[nx, ny, nz] == 1):
                                d = np.sqrt(dx*dx + dy*dy + dz_val*dz_val)
                                min_clearance = min(min_clearance, d)
                                found_obs = True
                if found_obs:
                    break

        return {
            'path_length': length,
            'smoothness': smoothness,
            'energy': energy,
            'min_clearance': min_clearance if min_clearance < float('inf') else 0,
            'max_speed': np.max(np.linalg.norm(vel, axis=1)),
            'max_accel': np.max(np.linalg.norm(acc, axis=1)),
        }


def main():
    data = load_all_data()
    start = data['env_params']['start']
    goal = data['env_params']['goal']

    print("=" * 60)
    print("Week 8 - Complete 3D Drone Planning Pipeline")
    print("=" * 60)

    np.random.seed(42)
    planner = DronePlanner(data)

    print("\n1. Building environment...")
    planner.build_environment()
    print(f"   Grid: {planner.grid.shape}, occupied: {np.sum(planner.grid)}")

    print("2. Inflating obstacles (radius=1)...")
    planner.inflate_obstacles(radius=1.0)
    print(f"   Inflated occupied: {np.sum(planner.inflated_grid)}")

    print("3. Planning global paths...")
    t0 = time.time()
    astar_path = planner.plan_global_path(start, goal, 'astar')
    astar_time = time.time() - t0
    print(f"   A*: {'found' if astar_path else 'FAILED'} ({astar_time:.3f}s)")

    np.random.seed(42)
    t0 = time.time()
    rrt_path = planner.plan_global_path(start, goal, 'rrt_star')
    rrt_time = time.time() - t0
    print(f"   RRT*: {'found' if rrt_path else 'FAILED'} ({rrt_time:.3f}s)")

    print("4. Optimizing trajectories...")
    astar_traj = planner.optimize_trajectory(astar_path)
    rrt_traj = planner.optimize_trajectory(rrt_path)

    print("5. Simulating execution...")
    astar_exec = planner.simulate_execution(astar_traj, data['dynamic_obstacles'])
    rrt_exec = planner.simulate_execution(rrt_traj, data['dynamic_obstacles'])

    print("6. Evaluating...")
    astar_eval = planner.evaluate(astar_traj, astar_path)
    rrt_eval = planner.evaluate(rrt_traj, rrt_path)

    for name, ev in [("A*", astar_eval), ("RRT*", rrt_eval)]:
        if ev:
            print(f"\n   {name} metrics:")
            for k, v in ev.items():
                print(f"     {k}: {v:.2f}")

    # 6-panel visualization
    fig = plt.figure(figsize=(20, 14))

    # 1. 3D path comparison
    ax1 = fig.add_subplot(231, projection='3d')
    occ = np.argwhere(planner.grid == 1)
    if len(occ) > 2000:
        occ = occ[np.random.choice(len(occ), 2000, replace=False)]
    ax1.scatter(occ[:, 0], occ[:, 1], occ[:, 2], c='gray', alpha=0.04, s=1)
    if astar_traj:
        p = astar_traj['position']
        ax1.plot(p[:, 0], p[:, 1], p[:, 2], 'b-', linewidth=1.5, label='A*')
    if rrt_traj:
        p = rrt_traj['position']
        ax1.plot(p[:, 0], p[:, 1], p[:, 2], 'r-', linewidth=1.5, label='RRT*')
    ax1.scatter(*start, c='green', s=80, marker='^', zorder=5)
    ax1.scatter(*goal, c='red', s=80, marker='*', zorder=5)
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax1.set_title('3D Path Comparison'); ax1.legend(fontsize=8)
    ax1.view_init(elev=25, azim=-60)

    # 2. XYZ profiles
    ax2 = fig.add_subplot(232)
    if astar_traj:
        t = astar_traj['time']; p = astar_traj['position']
        ax2.plot(t, p[:, 0], 'b-', alpha=0.7, label='A* X')
        ax2.plot(t, p[:, 1], 'b--', alpha=0.7, label='A* Y')
        ax2.plot(t, p[:, 2], 'b:', alpha=0.7, label='A* Z')
    if rrt_traj:
        t = rrt_traj['time']; p = rrt_traj['position']
        ax2.plot(t, p[:, 0], 'r-', alpha=0.7, label='RRT* X')
        ax2.plot(t, p[:, 1], 'r--', alpha=0.7, label='RRT* Y')
        ax2.plot(t, p[:, 2], 'r:', alpha=0.7, label='RRT* Z')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Position (m)')
    ax2.set_title('Position Profiles'); ax2.legend(fontsize=6); ax2.grid(True, alpha=0.3)

    # 3. Velocity profiles
    ax3 = fig.add_subplot(233)
    if astar_traj:
        speeds = np.linalg.norm(astar_traj['velocity'], axis=1)
        ax3.plot(astar_traj['time'], speeds, 'b-', label='A*')
    if rrt_traj:
        speeds = np.linalg.norm(rrt_traj['velocity'], axis=1)
        ax3.plot(rrt_traj['time'], speeds, 'r-', label='RRT*')
    ax3.axhline(5.0, color='k', linestyle='--', alpha=0.5, label='v_max')
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Speed (m/s)')
    ax3.set_title('Velocity Profiles'); ax3.legend(); ax3.grid(True, alpha=0.3)

    # 4. Energy consumption
    ax4 = fig.add_subplot(234)
    for traj, label, color in [(astar_traj, 'A*', 'b'), (rrt_traj, 'RRT*', 'r')]:
        if traj:
            pos = traj['position']
            cum_e = [0.0]
            for i in range(len(pos)-1):
                d = np.linalg.norm(pos[i+1] - pos[i])
                dz = pos[i+1, 2] - pos[i, 2]
                e = 50.0 * d / 2.0 + 2.0 * 9.81 * max(dz, 0) / 0.7
                cum_e.append(cum_e[-1] + e)
            ax4.plot(traj['time'][:len(cum_e)], cum_e, f'{color}-', label=label)
    ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Energy (J)')
    ax4.set_title('Cumulative Energy'); ax4.legend(); ax4.grid(True, alpha=0.3)

    # 5. Clearance
    ax5 = fig.add_subplot(235)
    for traj, label, color in [(astar_traj, 'A*', 'b'), (rrt_traj, 'RRT*', 'r')]:
        if traj:
            pos = traj['position']
            clearances = []
            for p in pos[::5]:
                ix, iy, iz = int(round(p[0])), int(round(p[1])), int(round(p[2]))
                min_d = 10.0
                for dx in range(-5, 6):
                    for dy in range(-5, 6):
                        for dz in range(-5, 6):
                            nx2, ny2, nz2 = ix+dx, iy+dy, iz+dz
                            if (0 <= nx2 < planner.grid.shape[0] and 0 <= ny2 < planner.grid.shape[1]
                                    and 0 <= nz2 < planner.grid.shape[2] and planner.grid[nx2, ny2, nz2] == 1):
                                d = np.sqrt(dx*dx + dy*dy + dz*dz)
                                min_d = min(min_d, d)
                clearances.append(min_d)
            t_sub = traj['time'][::5][:len(clearances)]
            ax5.plot(t_sub, clearances, f'{color}-', label=label, alpha=0.7)
    ax5.axhline(1.0, color='k', linestyle='--', alpha=0.5, label='Safety margin')
    ax5.set_xlabel('Time (s)'); ax5.set_ylabel('Clearance (m)')
    ax5.set_title('Obstacle Clearance'); ax5.legend(); ax5.grid(True, alpha=0.3)

    # 6. Method comparison bar chart
    ax6 = fig.add_subplot(236)
    metrics_names = ['path_length', 'smoothness', 'energy', 'min_clearance']
    labels = ['Path Len (m)', 'Smoothness', 'Energy (J)', 'Min Clear (m)']
    x = np.arange(len(metrics_names))
    astar_vals = [astar_eval.get(m, 0) for m in metrics_names]
    rrt_vals = [rrt_eval.get(m, 0) for m in metrics_names]
    # Normalize for display
    max_vals = [max(abs(a), abs(r), 1e-6) for a, r in zip(astar_vals, rrt_vals)]
    astar_norm = [a / m for a, m in zip(astar_vals, max_vals)]
    rrt_norm = [r / m for r, m in zip(rrt_vals, max_vals)]
    ax6.bar(x - 0.2, astar_norm, 0.35, label='A*', color='steelblue')
    ax6.bar(x + 0.2, rrt_norm, 0.35, label='RRT*', color='coral')
    ax6.set_xticks(x); ax6.set_xticklabels(labels, fontsize=8)
    ax6.set_ylabel('Normalized Value'); ax6.set_title('Method Comparison')
    ax6.legend()

    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'task7_full_planning.png'), dpi=150)
    plt.close()
    print(f"\nSaved to {out_dir}/task7_full_planning.png")
    print("Pipeline complete!")


if __name__ == '__main__':
    main()
