#!/usr/bin/env python3
"""
Task 5 Solution: Complete EGO-Planner Pipeline
================================================
Saves: task5_full_ego_planner.png
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
    evaluate_uniform_bspline, uniform_bspline_velocity,
    uniform_bspline_acceleration, uniform_bspline_jerk,
    evaluate_uniform_bspline_derivative
)


class EGOPlanner:
    def __init__(self, obstacles, v_max=2.0, a_max=4.0,
                 weights=None, safety_margin=0.5):
        self.obstacles = obstacles
        self.v_max = v_max
        self.a_max = a_max
        self.safety_margin = safety_margin
        self.weights = weights or {'smooth': 1.0, 'collision': 50.0, 'dynamic': 10.0}
        self.control_points = None
        self.dt = None
        self.cost_history = []
        self.n_fixed = 2  # fixed endpoints on each side

    def initialize_trajectory(self, start, goal, n_segments=12):
        n_cp = n_segments + 3
        self.dt = 1.0
        self.control_points = np.zeros((n_cp, 3))
        for i in range(n_cp):
            self.control_points[i] = start + (goal - start) * i / (n_cp - 1)
        return self.control_points.copy()

    def _smoothness_cost(self, cp):
        jerks = (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / (self.dt**3)
        return np.sum(jerks**2)

    def _smoothness_grad(self, cp):
        n = len(cp)
        grad = np.zeros_like(cp)
        jerks = (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / (self.dt**3)
        for i in range(len(jerks)):
            j = jerks[i] * 2.0 / (self.dt**3)
            grad[i] += -j
            grad[i+1] += 3*j
            grad[i+2] += -3*j
            grad[i+3] += j
        return grad

    def _collision_cost(self, cp):
        cost = 0.0
        for i in range(len(cp)):
            for obs in self.obstacles:
                d = np.linalg.norm(cp[i] - obs['center']) - obs['radius']
                if d < self.safety_margin:
                    cost += (self.safety_margin - d) ** 2
        return cost

    def _collision_grad(self, cp):
        grad = np.zeros_like(cp)
        for i in range(len(cp)):
            for obs in self.obstacles:
                diff = cp[i] - obs['center']
                dist_c = np.linalg.norm(diff)
                d = dist_c - obs['radius']
                if d < self.safety_margin and dist_c > 1e-8:
                    direction = diff / dist_c
                    grad[i] += -2.0 * (self.safety_margin - d) * direction
        return grad

    def _dynamic_cost(self, cp):
        cost = 0.0
        vel_cp = uniform_bspline_velocity(cp, self.dt)
        acc_cp = uniform_bspline_acceleration(cp, self.dt)
        for v in vel_cp:
            vn = np.linalg.norm(v)
            if vn > self.v_max:
                cost += (vn - self.v_max) ** 2
        for a in acc_cp:
            an = np.linalg.norm(a)
            if an > self.a_max:
                cost += (an - self.a_max) ** 2
        return cost

    def _dynamic_grad(self, cp):
        grad = np.zeros_like(cp)
        dt = self.dt
        # Velocity penalty
        vel_cp = uniform_bspline_velocity(cp, dt)
        for i in range(len(vel_cp)):
            vn = np.linalg.norm(vel_cp[i])
            if vn > self.v_max and vn > 1e-8:
                dv = 2.0 * (vn - self.v_max) * vel_cp[i] / (vn * dt)
                grad[i] -= dv
                grad[i+1] += dv
        # Acceleration penalty
        acc_cp = uniform_bspline_acceleration(cp, dt)
        for i in range(len(acc_cp)):
            an = np.linalg.norm(acc_cp[i])
            if an > self.a_max and an > 1e-8:
                da = 2.0 * (an - self.a_max) * acc_cp[i] / (an * dt**2)
                grad[i] += da
                grad[i+1] -= 2*da
                grad[i+2] += da
        return grad

    def total_cost(self, x_free):
        cp = self._rebuild_cp(x_free)
        w = self.weights
        c_s = w['smooth'] * self._smoothness_cost(cp)
        c_c = w['collision'] * self._collision_cost(cp)
        c_d = w['dynamic'] * self._dynamic_cost(cp)
        total = c_s + c_c + c_d
        self.cost_history.append(total)
        self._last_costs = (c_s, c_c, c_d)
        return total

    def total_gradient(self, x_free):
        cp = self._rebuild_cp(x_free)
        w = self.weights
        g = (w['smooth'] * self._smoothness_grad(cp) +
             w['collision'] * self._collision_grad(cp) +
             w['dynamic'] * self._dynamic_grad(cp))
        # Only return gradient for free CPs
        nf = self.n_fixed
        return g[nf:-nf].flatten()

    def _rebuild_cp(self, x_free):
        cp = self.control_points.copy()
        nf = self.n_fixed
        cp[nf:-nf] = x_free.reshape(-1, 3)
        return cp

    def optimize(self, max_iter=200, tolerance=1e-6):
        self.cost_history = []
        nf = self.n_fixed
        x0 = self.control_points[nf:-nf].flatten()

        result = minimize(
            self.total_cost, x0,
            jac=self.total_gradient,
            method='L-BFGS-B',
            options={'maxiter': max_iter, 'ftol': tolerance}
        )

        self.control_points[nf:-nf] = result.x.reshape(-1, 3)
        print(f"  Optimization: success={result.success}, iters={result.nit}, cost={result.fun:.4f}")
        return result.success

    def check_feasibility(self):
        vel_cp = uniform_bspline_velocity(self.control_points, self.dt)
        acc_cp = uniform_bspline_acceleration(self.control_points, self.dt)
        max_v = np.max(np.linalg.norm(vel_cp, axis=1))
        max_a = np.max(np.linalg.norm(acc_cp, axis=1))
        feasible = (max_v <= self.v_max * 1.05) and (max_a <= self.a_max * 1.05)
        return feasible, max_v, max_a

    def replan_if_needed(self, max_attempts=3):
        for attempt in range(max_attempts):
            feasible, max_v, max_a = self.check_feasibility()
            if feasible:
                return False
            print(f"  Infeasible (v={max_v:.2f}, a={max_a:.2f}), increasing dt...")
            self.dt *= 1.3
            self.optimize(max_iter=100)
        return True


def create_obstacles(start, goal, n_on_path=5, n_random=5, seed=42):
    """Create obstacles, some deliberately placed on the straight-line path."""
    rng = np.random.RandomState(seed)
    obstacles = []
    # Place obstacles along the start-goal line with small offsets
    for i in range(n_on_path):
        alpha = (i + 1) / (n_on_path + 1)
        center = start + alpha * (goal - start)
        # Small perpendicular offset so path goes through
        offset = rng.uniform(-0.3, 0.3, size=3)
        center += offset
        r = rng.uniform(0.4, 0.8)
        obstacles.append({'center': center, 'radius': r})
    # Random obstacles in the environment
    for _ in range(n_random):
        c = rng.uniform([1, 1, 0.5], [11, 9, 4])
        r = rng.uniform(0.3, 0.6)
        obstacles.append({'center': c, 'radius': r})
    return obstacles


def main():
    print("=" * 60)
    print("Task 5 Solution: Full EGO-Planner Pipeline")
    print("=" * 60)

    start = np.array([0.5, 0.5, 2.0])
    goal = np.array([11.5, 9.5, 2.5])
    start = np.array([0.5, 0.5, 2.0])
    goal = np.array([11.5, 9.5, 2.5])
    obstacles = create_obstacles(start, goal)

    planner = EGOPlanner(
        obstacles, v_max=3.0, a_max=6.0,
        weights={'smooth': 1.0, 'collision': 80.0, 'dynamic': 15.0},
        safety_margin=0.5
    )

    t0 = time.time()
    cp_init = planner.initialize_trajectory(start, goal, n_segments=15)
    planner.optimize(max_iter=300)
    replanned = planner.replan_if_needed()
    elapsed = time.time() - t0

    feasible, max_v, max_a = planner.check_feasibility()
    print(f"  Feasible: {feasible}, max_v: {max_v:.2f}, max_a: {max_a:.2f}")
    print(f"  Computation time: {elapsed:.3f}s")

    # Evaluate
    cp = planner.control_points
    dt = planner.dt
    n_seg = len(cp) - 3
    t_eval = np.linspace(0, n_seg * dt - 1e-10, 500)
    traj = evaluate_uniform_bspline(cp, dt, t_eval)
    traj_init = evaluate_uniform_bspline(cp_init, 1.0, np.linspace(0, (len(cp_init)-3)-1e-10, 500))

    vel = evaluate_uniform_bspline_derivative(cp, dt, t_eval, order=1)
    speeds = np.linalg.norm(vel, axis=1)

    # Plot
    fig = plt.figure(figsize=(16, 12))

    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(traj_init[:, 0], traj_init[:, 1], traj_init[:, 2], 'r--', alpha=0.4, label='Initial')
    ax1.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'b-', linewidth=2, label='Optimized')
    for obs in obstacles:
        u, v = np.mgrid[0:2*np.pi:12j, 0:np.pi:8j]
        x = obs['center'][0] + obs['radius'] * np.cos(u) * np.sin(v)
        y = obs['center'][1] + obs['radius'] * np.sin(u) * np.sin(v)
        z = obs['center'][2] + obs['radius'] * np.cos(v)
        ax1.plot_surface(x, y, z, alpha=0.2, color='red')
    ax1.set_title('3D Trajectory')
    ax1.legend()

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.semilogy(planner.cost_history)
    ax2.set_xlabel('Evaluation')
    ax2.set_ylabel('Total Cost (log)')
    ax2.set_title('Cost Convergence')
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(t_eval, speeds, 'b-', linewidth=2)
    ax3.axhline(y=planner.v_max, color='r', linestyle='--', label=f'v_max={planner.v_max}')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Speed [m/s]')
    ax3.set_title('Velocity Profile')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(2, 2, 4)
    last = planner._last_costs
    labels = ['Smooth', 'Collision', 'Dynamic']
    colors = ['blue', 'red', 'green']
    ax4.bar(labels, last, color=colors, alpha=0.7)
    ax4.set_ylabel('Cost')
    ax4.set_title('Final Cost Breakdown')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('task5_full_ego_planner.png', dpi=150)
    print("Saved: task5_full_ego_planner.png")


if __name__ == '__main__':
    main()
