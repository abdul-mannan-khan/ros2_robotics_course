#!/usr/bin/env python3
"""
Task 7 Solution: Complete Trajectory Optimization Pipeline
===========================================================
Saves: task7_full_integration.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
import time

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from bspline_utils import (
    evaluate_uniform_bspline, waypoints_to_uniform_bspline,
    uniform_bspline_velocity, uniform_bspline_acceleration,
    uniform_bspline_jerk, evaluate_uniform_bspline_derivative
)


class TrajectoryPlanningSystem:
    def __init__(self):
        self.obstacles = []
        self.no_fly_zones = []
        self.control_points = None
        self.dt = None
        self.cost_history = []
        self.v_max = 2.0
        self.a_max = 4.0
        self.safety_margin = 0.5
        self.computation_time = 0.0

    def setup_environment(self, obstacles, no_fly_zones=None):
        self.obstacles = obstacles
        self.no_fly_zones = no_fly_zones or []

    def plan_mission(self, waypoints, v_max=2.0, a_max=4.0):
        self.v_max = v_max
        self.a_max = a_max
        waypoints = np.asarray(waypoints, dtype=float)
        n_cp = len(waypoints) + 8
        self.control_points, self.dt = waypoints_to_uniform_bspline(
            waypoints, n_control_points=n_cp, dt=1.0)
        # Adjust dt for feasibility
        self._adjust_dt()

    def _adjust_dt(self):
        cp = self.control_points
        diffs1 = np.linalg.norm(np.diff(cp, axis=0), axis=1)
        dt_v = np.max(diffs1) / self.v_max if np.max(diffs1) > 0 else 0.1
        diffs2 = np.linalg.norm(cp[2:] - 2*cp[1:-1] + cp[:-2], axis=1)
        dt_a = np.sqrt(np.max(diffs2) / self.a_max) if np.max(diffs2) > 0 else 0.1
        self.dt = max(self.dt, dt_v, dt_a)

    def _obstacle_distance(self, point):
        min_d = np.inf
        direction = np.zeros(3)
        for obs in self.obstacles:
            diff = point - obs['center']
            dc = np.linalg.norm(diff)
            d = dc - obs['radius']
            if d < min_d:
                min_d = d
                direction = diff / dc if dc > 1e-8 else np.array([1, 0, 0])
        for zone in self.no_fly_zones:
            # Box distance
            zmin, zmax = np.array(zone['min']), np.array(zone['max'])
            clamped = np.clip(point, zmin, zmax)
            d = np.linalg.norm(point - clamped)
            inside = np.all(point >= zmin) and np.all(point <= zmax)
            if inside:
                # Distance to nearest face
                dists_to_faces = np.minimum(point - zmin, zmax - point)
                d = -np.min(dists_to_faces)
            if d < min_d:
                min_d = d
                if not inside:
                    diff = point - clamped
                    direction = diff / np.linalg.norm(diff) if np.linalg.norm(diff) > 1e-8 else np.array([1, 0, 0])
                else:
                    idx = np.argmin(np.minimum(point - zmin, zmax - point).flatten())
                    axis = idx % 3
                    direction = np.zeros(3)
                    direction[axis] = 1.0 if (point[axis] - zmin[axis]) < (zmax[axis] - point[axis]) else -1.0
        return min_d, direction

    def _total_cost(self, x_free):
        cp = self.control_points.copy()
        cp[2:-2] = x_free.reshape(-1, 3)
        dt = self.dt

        # Smoothness
        jerks = (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / dt**3
        c_smooth = np.sum(jerks**2)

        # Collision
        c_coll = 0.0
        for i in range(len(cp)):
            d, _ = self._obstacle_distance(cp[i])
            if d < self.safety_margin:
                c_coll += (self.safety_margin - d)**2

        # Dynamic
        c_dyn = 0.0
        vel_cp = np.diff(cp, axis=0) / dt
        for v in vel_cp:
            vn = np.linalg.norm(v)
            if vn > self.v_max:
                c_dyn += (vn - self.v_max)**2
        acc_cp = (cp[2:] - 2*cp[1:-1] + cp[:-2]) / dt**2
        for a in acc_cp:
            an = np.linalg.norm(a)
            if an > self.a_max:
                c_dyn += (an - self.a_max)**2

        total = 1.0 * c_smooth + 80.0 * c_coll + 15.0 * c_dyn
        self.cost_history.append(total)
        return total

    def _total_grad(self, x_free):
        cp = self.control_points.copy()
        cp[2:-2] = x_free.reshape(-1, 3)
        dt = self.dt
        n = len(cp)
        grad = np.zeros_like(cp)

        # Smoothness
        jerks = (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / dt**3
        for i in range(len(jerks)):
            j = jerks[i] * 2.0 / dt**3
            grad[i] += -j; grad[i+1] += 3*j; grad[i+2] += -3*j; grad[i+3] += j

        # Collision
        for i in range(n):
            d, direction = self._obstacle_distance(cp[i])
            if d < self.safety_margin:
                grad[i] += 80.0 * (-2.0) * (self.safety_margin - d) * direction

        # Dynamic
        vel_cp = np.diff(cp, axis=0) / dt
        for i in range(len(vel_cp)):
            vn = np.linalg.norm(vel_cp[i])
            if vn > self.v_max and vn > 1e-8:
                dv = 15.0 * 2.0 * (vn - self.v_max) * vel_cp[i] / (vn * dt)
                grad[i] -= dv; grad[i+1] += dv
        acc_cp = (cp[2:] - 2*cp[1:-1] + cp[:-2]) / dt**2
        for i in range(len(acc_cp)):
            an = np.linalg.norm(acc_cp[i])
            if an > self.a_max and an > 1e-8:
                da = 15.0 * 2.0 * (an - self.a_max) * acc_cp[i] / (an * dt**2)
                grad[i] += da; grad[i+1] -= 2*da; grad[i+2] += da

        return grad[2:-2].flatten()

    def optimize_full_trajectory(self, max_iter=300):
        self.cost_history = []
        t0 = time.time()
        x0 = self.control_points[2:-2].flatten()
        result = minimize(self._total_cost, x0, jac=self._total_grad,
                          method='L-BFGS-B', options={'maxiter': max_iter, 'ftol': 1e-7})
        self.control_points[2:-2] = result.x.reshape(-1, 3)
        self.computation_time = time.time() - t0
        print(f"  Optimization: success={result.success}, iters={result.nit}, "
              f"time={self.computation_time:.3f}s")
        return result.success

    def simulate_execution(self, disturbances=True, noise_std=0.02):
        cp = self.control_points
        n_seg = len(cp) - 3
        t_eval = np.linspace(0, n_seg * self.dt - 1e-10, 500)
        traj = evaluate_uniform_bspline(cp, self.dt, t_eval)
        if disturbances:
            traj += np.random.normal(0, noise_std, traj.shape)
        return traj, t_eval

    def evaluate(self):
        cp = self.control_points
        dt = self.dt
        n_seg = len(cp) - 3
        t_eval = np.linspace(0, n_seg * dt - 1e-10, 500)
        traj = evaluate_uniform_bspline(cp, dt, t_eval)

        # Path length
        diffs = np.diff(traj, axis=0)
        path_length = np.sum(np.linalg.norm(diffs, axis=1))

        # Jerk integral
        jerk_cp = uniform_bspline_jerk(cp, dt)
        jerk_integral = np.sum(np.linalg.norm(jerk_cp, axis=1)**2) * dt

        # Max velocity/acceleration
        vel_cp = uniform_bspline_velocity(cp, dt)
        acc_cp = uniform_bspline_acceleration(cp, dt)
        max_vel = np.max(np.linalg.norm(vel_cp, axis=1))
        max_acc = np.max(np.linalg.norm(acc_cp, axis=1))

        # Min clearance
        min_clearance = np.inf
        for pt in traj:
            d, _ = self._obstacle_distance(pt)
            min_clearance = min(min_clearance, d)

        return {
            'path_length': path_length,
            'smoothness_jerk': jerk_integral,
            'max_velocity': max_vel,
            'max_acceleration': max_acc,
            'min_clearance': min_clearance,
            'computation_time': self.computation_time
        }


def main():
    print("=" * 60)
    print("Task 7 Solution: Complete Trajectory Optimization Pipeline")
    print("=" * 60)

    # Environment - place some obstacles along the path
    waypoints = np.array([
        [0.5, 0.5, 2.0],
        [3, 4, 3],
        [5, 2, 2.5],
        [9, 6, 3.5],
        [12, 3, 2],
        [14, 8, 2.5],
        [15, 0.5, 1.5]
    ], dtype=float)

    rng = np.random.RandomState(123)
    obstacles = []
    # Place obstacles near waypoint-to-waypoint segments
    for seg in range(len(waypoints) - 1):
        mid = 0.5 * (waypoints[seg] + waypoints[seg + 1])
        offset = rng.uniform(-0.3, 0.3, size=3)
        obstacles.append({'center': mid + offset, 'radius': rng.uniform(0.4, 0.7)})
    # Additional random obstacles
    for _ in range(6):
        c = rng.uniform([1, 1, 0.5], [14, 9, 4])
        r = rng.uniform(0.3, 0.6)
        obstacles.append({'center': c, 'radius': r})

    no_fly_zones = []  # No-fly zones omitted for clean gradient-based optimization

    system = TrajectoryPlanningSystem()
    system.setup_environment(obstacles, no_fly_zones)
    system.plan_mission(waypoints, v_max=3.0, a_max=6.0)
    system.optimize_full_trajectory(max_iter=400)

    metrics = system.evaluate()
    print("\nMetrics:")
    for k, v in metrics.items():
        print(f"  {k}: {v:.4f}")

    # Simulate
    traj_actual, t_eval = system.simulate_execution(disturbances=True, noise_std=0.03)
    traj_nominal, _ = system.simulate_execution(disturbances=False)

    cp = system.control_points
    dt = system.dt

    # Velocities and accelerations
    vel = evaluate_uniform_bspline_derivative(cp, dt, t_eval, order=1)
    speeds = np.linalg.norm(vel, axis=1)

    acc_t = np.linspace(0, max((len(cp)-2-1)*dt - 1e-10, 0.01), 300)
    acc = evaluate_uniform_bspline_derivative(cp, dt, acc_t, order=2)
    acc_mag = np.linalg.norm(acc, axis=1)

    jerk_cp = uniform_bspline_jerk(cp, dt)
    jerk_mag = np.linalg.norm(jerk_cp, axis=1)

    # Clearances
    clearances = []
    for pt in traj_nominal:
        d, _ = system._obstacle_distance(pt)
        clearances.append(d)

    # 6-panel figure
    fig = plt.figure(figsize=(18, 12))

    # 1. 3D trajectory
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(traj_nominal[:, 0], traj_nominal[:, 1], traj_nominal[:, 2], 'b-', linewidth=2, label='Planned')
    ax1.plot(traj_actual[:, 0], traj_actual[:, 1], traj_actual[:, 2], 'c-', alpha=0.4, label='Executed')
    ax1.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], 'g^', markersize=10, label='Waypoints')
    for obs in obstacles:
        u, v = np.mgrid[0:2*np.pi:10j, 0:np.pi:7j]
        x = obs['center'][0] + obs['radius'] * np.cos(u) * np.sin(v)
        y = obs['center'][1] + obs['radius'] * np.sin(u) * np.sin(v)
        z = obs['center'][2] + obs['radius'] * np.cos(v)
        ax1.plot_surface(x, y, z, alpha=0.15, color='red')
    for zone in no_fly_zones:
        zmin, zmax = np.array(zone['min']), np.array(zone['max'])
        # Draw box edges
        for s in [[0,0,0],[1,0,0],[1,1,0],[0,1,0]]:
            for d in range(3):
                start_pt = zmin + np.array(s) * (zmax - zmin)
                end_pt = start_pt.copy(); end_pt[d] = zmax[d] if s[d] == 0 else zmin[d]
                # Simplified box visualization
        ax1.bar3d(zmin[0], zmin[1], zmin[2],
                  zmax[0]-zmin[0], zmax[1]-zmin[1], zmax[2]-zmin[2],
                  alpha=0.1, color='yellow')
    ax1.set_title('3D Trajectory')
    ax1.legend(fontsize=7)

    # 2. Velocity
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(t_eval, speeds, 'b-', linewidth=2)
    ax2.axhline(y=system.v_max, color='r', linestyle='--', label=f'v_max={system.v_max}')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Speed [m/s]')
    ax2.set_title('Velocity Profile')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. Acceleration
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(acc_t, acc_mag, 'b-', linewidth=2)
    ax3.axhline(y=system.a_max, color='r', linestyle='--', label=f'a_max={system.a_max}')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Acceleration [m/s^2]')
    ax3.set_title('Acceleration Profile')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Jerk
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.bar(range(len(jerk_mag)), jerk_mag, color='purple', alpha=0.7)
    ax4.set_xlabel('Segment')
    ax4.set_ylabel('|Jerk| [m/s^3]')
    ax4.set_title('Jerk Profile')
    ax4.grid(True, alpha=0.3)

    # 5. Obstacle clearance
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(t_eval, clearances, 'b-', linewidth=2)
    ax5.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax5.axhline(y=system.safety_margin, color='g', linestyle='--', label='Safety margin')
    ax5.fill_between(t_eval, 0, clearances, where=np.array(clearances) < 0, alpha=0.3, color='red')
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Clearance [m]')
    ax5.set_title('Obstacle Clearance')
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # 6. Cost convergence
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.semilogy(system.cost_history)
    ax6.set_xlabel('Evaluation')
    ax6.set_ylabel('Total Cost (log)')
    ax6.set_title('Cost Convergence')
    ax6.grid(True, alpha=0.3)

    # Metrics text
    fig.text(0.5, 0.01,
             f"Path length: {metrics['path_length']:.1f}m | "
             f"Max vel: {metrics['max_velocity']:.2f} m/s | "
             f"Max acc: {metrics['max_acceleration']:.2f} m/s^2 | "
             f"Min clearance: {metrics['min_clearance']:.2f}m | "
             f"Time: {metrics['computation_time']:.3f}s",
             ha='center', fontsize=10, style='italic')

    plt.tight_layout(rect=[0, 0.03, 1, 1])
    plt.savefig('task7_full_integration.png', dpi=150)
    print("Saved: task7_full_integration.png")


if __name__ == '__main__':
    main()
