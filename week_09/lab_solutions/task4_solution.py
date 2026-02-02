#!/usr/bin/env python3
"""
Task 4 Solution: ESDF-Free Collision Avoidance
================================================
Saves: task4_collision_avoidance.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from bspline_utils import evaluate_uniform_bspline, uniform_bspline_jerk


def create_obstacle_field(bounds, n_obstacles=8, seed=42):
    rng = np.random.RandomState(seed)
    obstacles = []
    for _ in range(n_obstacles):
        center = np.array([
            rng.uniform(bounds[0][0] + 1, bounds[0][1] - 1),
            rng.uniform(bounds[1][0] + 1, bounds[1][1] - 1),
            rng.uniform(bounds[2][0] + 0.5, bounds[2][1] - 0.5),
        ])
        radius = rng.uniform(0.3, 0.8)
        obstacles.append({'center': center, 'radius': radius})
    return obstacles


def check_collision(point, obstacles):
    for obs in obstacles:
        if np.linalg.norm(point - obs['center']) < obs['radius']:
            return True
    return False


def find_closest_obstacle(point, obstacles):
    min_dist = np.inf
    best_grad = np.zeros(3)
    for obs in obstacles:
        diff = point - obs['center']
        dist_center = np.linalg.norm(diff)
        signed_dist = dist_center - obs['radius']
        if signed_dist < min_dist:
            min_dist = signed_dist
            if dist_center > 1e-8:
                best_grad = diff / dist_center
            else:
                best_grad = np.array([1.0, 0.0, 0.0])
    return min_dist, best_grad


def collision_cost(control_points, obstacles, safety_margin=0.3):
    cp = np.asarray(control_points, dtype=float)
    cost = 0.0
    for i in range(len(cp)):
        dist, _ = find_closest_obstacle(cp[i], obstacles)
        if dist < safety_margin:
            cost += (safety_margin - dist) ** 2
    return cost


def collision_gradient(control_points, obstacles, safety_margin=0.3):
    cp = np.asarray(control_points, dtype=float)
    grad = np.zeros_like(cp)
    for i in range(len(cp)):
        dist, direction = find_closest_obstacle(cp[i], obstacles)
        if dist < safety_margin:
            # Push away: gradient points toward decreasing cost
            grad[i] = -2.0 * (safety_margin - dist) * direction
    return grad


def smoothness_cost(control_points, dt):
    cp = np.asarray(control_points, dtype=float)
    jerks = (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / (dt**3)
    return np.sum(jerks**2)


def smoothness_gradient(control_points, dt):
    cp = np.asarray(control_points, dtype=float)
    n = len(cp)
    grad = np.zeros_like(cp)
    jerks = (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / (dt**3)
    for i in range(len(jerks)):
        j = jerks[i] * 2.0 / (dt**3)
        grad[i]     += -j
        grad[i + 1] +=  3*j
        grad[i + 2] += -3*j
        grad[i + 3] +=  j
    return grad


def optimize_collision_free(control_points, obstacles, dt=1.0, max_iter=300,
                            safety_margin=0.5, lambda_smooth=1.0, lambda_coll=50.0,
                            lr=0.01):
    cp = control_points.copy()
    cost_history = []

    for it in range(max_iter):
        c_smooth = smoothness_cost(cp, dt)
        c_coll = collision_cost(cp, obstacles, safety_margin)
        total = lambda_smooth * c_smooth + lambda_coll * c_coll
        cost_history.append(total)

        g_smooth = smoothness_gradient(cp, dt)
        g_coll = collision_gradient(cp, obstacles, safety_margin)
        grad = lambda_smooth * g_smooth + lambda_coll * g_coll

        # Fix endpoints
        grad[:2] = 0.0
        grad[-2:] = 0.0

        cp -= lr * grad

    return cp, cost_history


def main():
    print("=" * 60)
    print("Task 4 Solution: ESDF-Free Collision Avoidance")
    print("=" * 60)

    start = np.array([0.5, 1.0, 2.5])
    goal = np.array([11.5, 9.0, 2.5])

    # Place obstacles along the path to force avoidance
    rng = np.random.RandomState(42)
    obstacles = []
    for i in range(6):
        alpha = (i + 1) / 7.0
        center = start + alpha * (goal - start) + rng.uniform(-0.2, 0.2, size=3)
        obstacles.append({'center': center, 'radius': rng.uniform(0.4, 0.8)})
    # Additional random obstacles
    bounds = ((0, 12), (0, 10), (0, 5))
    for _ in range(5):
        c = np.array([rng.uniform(bounds[0][0]+1, bounds[0][1]-1),
                       rng.uniform(bounds[1][0]+1, bounds[1][1]-1),
                       rng.uniform(bounds[2][0]+0.5, bounds[2][1]-0.5)])
        obstacles.append({'center': c, 'radius': rng.uniform(0.3, 0.6)})

    # Straight-line initial trajectory
    n_cp = 18
    dt = 1.0
    cp_init = np.zeros((n_cp, 3))
    for i in range(n_cp):
        cp_init[i] = start + (goal - start) * i / (n_cp - 1)

    print(f"Initial collision cost: {collision_cost(cp_init, obstacles, 0.5):.4f}")

    cp_opt, cost_history = optimize_collision_free(
        cp_init, obstacles, dt=dt, max_iter=500,
        safety_margin=0.5, lambda_smooth=0.5, lambda_coll=80.0, lr=0.015
    )

    print(f"Final collision cost: {collision_cost(cp_opt, obstacles, 0.5):.4f}")

    # Evaluate trajectories
    n_seg = n_cp - 3
    t_eval = np.linspace(0, n_seg * dt - 1e-10, 500)
    traj_init = evaluate_uniform_bspline(cp_init, dt, t_eval)
    traj_opt = evaluate_uniform_bspline(cp_opt, dt, t_eval)

    # Plot
    fig = plt.figure(figsize=(16, 10))

    # 3D before
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(traj_init[:, 0], traj_init[:, 1], traj_init[:, 2], 'r-', linewidth=2, label='Initial')
    ax1.plot(cp_init[:, 0], cp_init[:, 1], cp_init[:, 2], 'r.', alpha=0.3)
    for obs in obstacles:
        u = np.linspace(0, 2*np.pi, 20)
        v = np.linspace(0, np.pi, 15)
        x = obs['center'][0] + obs['radius'] * np.outer(np.cos(u), np.sin(v))
        y = obs['center'][1] + obs['radius'] * np.outer(np.sin(u), np.sin(v))
        z = obs['center'][2] + obs['radius'] * np.outer(np.ones_like(u), np.cos(v))
        ax1.plot_surface(x, y, z, alpha=0.3, color='gray')
    ax1.set_title('Before Optimization')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')

    # 3D after
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax2.plot(traj_opt[:, 0], traj_opt[:, 1], traj_opt[:, 2], 'b-', linewidth=2, label='Optimized')
    ax2.plot(cp_opt[:, 0], cp_opt[:, 1], cp_opt[:, 2], 'b.', alpha=0.5)
    for obs in obstacles:
        u = np.linspace(0, 2*np.pi, 20)
        v = np.linspace(0, np.pi, 15)
        x = obs['center'][0] + obs['radius'] * np.outer(np.cos(u), np.sin(v))
        y = obs['center'][1] + obs['radius'] * np.outer(np.sin(u), np.sin(v))
        z = obs['center'][2] + obs['radius'] * np.outer(np.ones_like(u), np.cos(v))
        ax2.plot_surface(x, y, z, alpha=0.3, color='gray')
    ax2.set_title('After Optimization')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')

    # Cost convergence
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.semilogy(cost_history)
    ax3.set_xlabel('Iteration')
    ax3.set_ylabel('Total Cost (log)')
    ax3.set_title('Cost Convergence')
    ax3.grid(True, alpha=0.3)

    # Obstacle clearance
    ax4 = fig.add_subplot(2, 2, 4)
    clearances_init = []
    clearances_opt = []
    for i in range(len(traj_init)):
        d_init, _ = find_closest_obstacle(traj_init[i], obstacles)
        d_opt, _ = find_closest_obstacle(traj_opt[i], obstacles)
        clearances_init.append(d_init)
        clearances_opt.append(d_opt)
    ax4.plot(t_eval, clearances_init, 'r-', alpha=0.6, label='Initial')
    ax4.plot(t_eval, clearances_opt, 'b-', label='Optimized')
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax4.axhline(y=0.5, color='g', linestyle='--', alpha=0.5, label='Safety margin')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Clearance [m]')
    ax4.set_title('Obstacle Clearance')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('task4_collision_avoidance.png', dpi=150)
    print("Saved: task4_collision_avoidance.png")


if __name__ == '__main__':
    main()
