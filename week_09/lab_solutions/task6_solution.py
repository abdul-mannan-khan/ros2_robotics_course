#!/usr/bin/env python3
"""
Task 6 Solution: EGO-Swarm Multi-Drone Planning
=================================================
Saves: task6_multi_drone.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from bspline_utils import (
    evaluate_uniform_bspline, uniform_bspline_velocity,
    uniform_bspline_acceleration, uniform_bspline_jerk
)


class DroneAgent:
    def __init__(self, drone_id, obstacles, v_max=2.0, a_max=4.0, safety_dist=0.8):
        self.drone_id = drone_id
        self.obstacles = obstacles
        self.v_max = v_max
        self.a_max = a_max
        self.safety_dist = safety_dist
        self.control_points = None
        self.dt = 1.0
        self.n_fixed = 2

    def plan(self, start, goal, other_trajectories=None, n_segments=10):
        n_cp = n_segments + 3
        self.dt = 1.0

        if self.control_points is None:
            self.control_points = np.zeros((n_cp, 3))
            for i in range(n_cp):
                self.control_points[i] = start + (goal - start) * i / (n_cp - 1)

        cp_fixed_start = self.control_points[:self.n_fixed].copy()
        cp_fixed_end = self.control_points[-self.n_fixed:].copy()

        def cost_fn(x_free):
            cp = self.control_points.copy()
            cp[self.n_fixed:-self.n_fixed] = x_free.reshape(-1, 3)
            return self._total_cost(cp, other_trajectories)

        def grad_fn(x_free):
            cp = self.control_points.copy()
            cp[self.n_fixed:-self.n_fixed] = x_free.reshape(-1, 3)
            g = self._total_grad(cp, other_trajectories)
            return g[self.n_fixed:-self.n_fixed].flatten()

        x0 = self.control_points[self.n_fixed:-self.n_fixed].flatten()
        result = minimize(cost_fn, x0, jac=grad_fn, method='L-BFGS-B',
                          options={'maxiter': 100, 'ftol': 1e-6})
        self.control_points[self.n_fixed:-self.n_fixed] = result.x.reshape(-1, 3)

    def _total_cost(self, cp, other_trajectories):
        cost = 0.0
        # Smoothness (jerk)
        jerks = (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / self.dt**3
        cost += np.sum(jerks**2)
        # Obstacle collision
        for i in range(len(cp)):
            for obs in self.obstacles:
                d = np.linalg.norm(cp[i] - obs['center']) - obs['radius']
                if d < 0.5:
                    cost += 50.0 * (0.5 - d)**2
        # Inter-drone
        if other_trajectories:
            for other_cp in other_trajectories:
                cost += 30.0 * inter_drone_collision_cost(cp, other_cp, self.dt, self.safety_dist)
        # Dynamic
        vel_cp = np.diff(cp, axis=0) / self.dt
        for v in vel_cp:
            vn = np.linalg.norm(v)
            if vn > self.v_max:
                cost += 10.0 * (vn - self.v_max)**2
        return cost

    def _total_grad(self, cp, other_trajectories):
        n = len(cp)
        grad = np.zeros_like(cp)
        dt = self.dt

        # Smoothness gradient
        jerks = (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / dt**3
        for i in range(len(jerks)):
            j = jerks[i] * 2.0 / dt**3
            grad[i] += -j
            grad[i+1] += 3*j
            grad[i+2] += -3*j
            grad[i+3] += j

        # Obstacle gradient
        for i in range(n):
            for obs in self.obstacles:
                diff = cp[i] - obs['center']
                dc = np.linalg.norm(diff)
                d = dc - obs['radius']
                if d < 0.5 and dc > 1e-8:
                    grad[i] += 50.0 * (-2.0) * (0.5 - d) * diff / dc

        # Inter-drone gradient
        if other_trajectories:
            for other_cp in other_trajectories:
                g = inter_drone_collision_gradient(cp, other_cp, dt, self.safety_dist)
                grad += 30.0 * g

        # Dynamic gradient
        vel_cp = np.diff(cp, axis=0) / dt
        for i in range(len(vel_cp)):
            vn = np.linalg.norm(vel_cp[i])
            if vn > self.v_max and vn > 1e-8:
                dv = 10.0 * 2.0 * (vn - self.v_max) * vel_cp[i] / (vn * dt)
                grad[i] -= dv
                grad[i+1] += dv

        return grad

    def broadcast_trajectory(self):
        if self.control_points is not None:
            return self.control_points.copy()
        return None


def inter_drone_collision_cost(traj_i, traj_j, dt, safety_dist=0.8):
    """Pairwise collision cost using control point proximity."""
    n = min(len(traj_i), len(traj_j))
    cost = 0.0
    # Sample at control point times
    n_seg_i = len(traj_i) - 3
    n_seg_j = len(traj_j) - 3
    n_samples = 50
    t_max = min(n_seg_i, n_seg_j) * dt
    if t_max <= 0:
        return 0.0
    t_eval = np.linspace(0, t_max - 1e-10, n_samples)
    pts_i = evaluate_uniform_bspline(traj_i, dt, t_eval)
    pts_j = evaluate_uniform_bspline(traj_j, dt, t_eval)
    for k in range(n_samples):
        d = np.linalg.norm(pts_i[k] - pts_j[k])
        if d < safety_dist:
            cost += (safety_dist - d)**2
    return cost / n_samples


def inter_drone_collision_gradient(traj_i, traj_j, dt, safety_dist=0.8):
    """Approximate gradient of inter-drone cost w.r.t. traj_i control points."""
    n = len(traj_i)
    grad = np.zeros_like(traj_i)
    eps = 1e-4
    c0 = inter_drone_collision_cost(traj_i, traj_j, dt, safety_dist)
    for i in range(2, n - 2):
        for d in range(3):
            traj_p = traj_i.copy()
            traj_p[i, d] += eps
            cp = inter_drone_collision_cost(traj_p, traj_j, dt, safety_dist)
            grad[i, d] = (cp - c0) / eps
    return grad


def decentralized_planning(agents, starts, goals, iterations=5):
    n_drones = len(agents)
    for it in range(iterations):
        print(f"  Iteration {it+1}/{iterations}")
        for i in range(n_drones):
            other_trajs = []
            for j in range(n_drones):
                if i != j:
                    t = agents[j].broadcast_trajectory()
                    if t is not None:
                        other_trajs.append(t)
            agents[i].plan(starts[i], goals[i], other_trajs if other_trajs else None)

    return [a.broadcast_trajectory() for a in agents]


def main():
    print("=" * 60)
    print("Task 6 Solution: EGO-Swarm Multi-Drone Planning")
    print("=" * 60)

    # Simple obstacles
    obstacles = [
        {'center': np.array([5, 5, 2]), 'radius': 0.8},
        {'center': np.array([3, 7, 2.5]), 'radius': 0.5},
        {'center': np.array([7, 3, 1.5]), 'radius': 0.6},
    ]

    # 4 drones crossing paths
    starts = [
        np.array([0, 0, 2]),
        np.array([10, 0, 2]),
        np.array([10, 10, 2]),
        np.array([0, 10, 2]),
    ]
    goals = [
        np.array([10, 10, 2]),
        np.array([0, 10, 2]),
        np.array([0, 0, 2]),
        np.array([10, 0, 2]),
    ]

    agents = [DroneAgent(i, obstacles, v_max=3.0, a_max=5.0, safety_dist=1.0) for i in range(4)]
    trajectories = decentralized_planning(agents, starts, goals, iterations=4)

    # Evaluate
    colors = ['blue', 'red', 'green', 'orange']
    fig = plt.figure(figsize=(16, 10))

    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    for i, (traj_cp, col) in enumerate(zip(trajectories, colors)):
        n_seg = len(traj_cp) - 3
        t_eval = np.linspace(0, n_seg * 1.0 - 1e-10, 300)
        traj = evaluate_uniform_bspline(traj_cp, 1.0, t_eval)
        ax1.plot(traj[:, 0], traj[:, 1], traj[:, 2], color=col, linewidth=2, label=f'Drone {i}')
        ax1.plot([starts[i][0]], [starts[i][1]], [starts[i][2]], 'o', color=col, markersize=10)
        ax1.plot([goals[i][0]], [goals[i][1]], [goals[i][2]], 's', color=col, markersize=10)
    for obs in obstacles:
        u, v = np.mgrid[0:2*np.pi:12j, 0:np.pi:8j]
        x = obs['center'][0] + obs['radius'] * np.cos(u) * np.sin(v)
        y = obs['center'][1] + obs['radius'] * np.sin(u) * np.sin(v)
        z = obs['center'][2] + obs['radius'] * np.cos(v)
        ax1.plot_surface(x, y, z, alpha=0.3, color='gray')
    ax1.set_title('Multi-Drone Trajectories (3D)')
    ax1.legend()

    # Top-down view
    ax2 = fig.add_subplot(2, 2, 2)
    for i, (traj_cp, col) in enumerate(zip(trajectories, colors)):
        n_seg = len(traj_cp) - 3
        t_eval = np.linspace(0, n_seg * 1.0 - 1e-10, 300)
        traj = evaluate_uniform_bspline(traj_cp, 1.0, t_eval)
        ax2.plot(traj[:, 0], traj[:, 1], color=col, linewidth=2, label=f'Drone {i}')
        ax2.plot(starts[i][0], starts[i][1], 'o', color=col, markersize=10)
        ax2.plot(goals[i][0], goals[i][1], 's', color=col, markersize=10)
    for obs in obstacles:
        circle = plt.Circle(obs['center'][:2], obs['radius'], alpha=0.3, color='gray')
        ax2.add_patch(circle)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title('Top-Down View')
    ax2.set_aspect('equal')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Inter-drone distances
    ax3 = fig.add_subplot(2, 2, 3)
    n_samples = 200
    t_common = np.linspace(0, 7 - 1e-10, n_samples)
    evaluated = []
    for traj_cp in trajectories:
        n_seg = len(traj_cp) - 3
        t_eval = np.linspace(0, n_seg * 1.0 - 1e-10, n_samples)
        evaluated.append(evaluate_uniform_bspline(traj_cp, 1.0, t_eval))

    pair_idx = 0
    for i in range(4):
        for j in range(i+1, 4):
            dists = np.linalg.norm(evaluated[i] - evaluated[j], axis=1)
            ax3.plot(dists, label=f'{i}-{j}')
            pair_idx += 1
    ax3.axhline(y=1.0, color='r', linestyle='--', label='Safety dist')
    ax3.set_xlabel('Time sample')
    ax3.set_ylabel('Distance [m]')
    ax3.set_title('Inter-Drone Distances')
    ax3.legend(fontsize=7, ncol=2)
    ax3.grid(True, alpha=0.3)

    # Speed profiles
    ax4 = fig.add_subplot(2, 2, 4)
    for i, (traj_cp, col) in enumerate(zip(trajectories, colors)):
        vel_cp = uniform_bspline_velocity(traj_cp, 1.0)
        speeds = np.linalg.norm(vel_cp, axis=1)
        ax4.plot(speeds, color=col, label=f'Drone {i}')
    ax4.axhline(y=3.0, color='r', linestyle='--', label='v_max')
    ax4.set_xlabel('Control point index')
    ax4.set_ylabel('Speed [m/s]')
    ax4.set_title('Velocity Control Points')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('task6_multi_drone.png', dpi=150)
    print("Saved: task6_multi_drone.png")


if __name__ == '__main__':
    main()
