#!/usr/bin/env python3
"""
Week 5 - Task 3 Solution: Dynamic Window Approach (DWA)
Complete DWA local planner implementation.

DWA works as a local controller - in Nav2 it follows a global path.
Here we demonstrate DWA navigating through an open area with scattered obstacles.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))


class DWAConfig:
    def __init__(self):
        self.max_speed = 1.0
        self.min_speed = 0.0
        self.max_yaw_rate = 2.0
        self.max_accel = 1.0
        self.max_yaw_accel = 3.0
        self.v_resolution = 0.1
        self.yaw_rate_resolution = 0.2
        self.dt = 0.1
        self.predict_time = 1.5
        self.heading_weight = 3.0
        self.dist_weight = 0.5
        self.velocity_weight = 1.0
        self.robot_radius = 0.3
        self.goal_tolerance = 0.3


def compute_dynamic_window(v, w, config):
    """Compute velocity search space."""
    v_min = max(config.min_speed, v - config.max_accel * config.dt)
    v_max = min(config.max_speed, v + config.max_accel * config.dt)
    w_min = max(-config.max_yaw_rate, w - config.max_yaw_accel * config.dt)
    w_max = min(config.max_yaw_rate, w + config.max_yaw_accel * config.dt)
    return v_min, v_max, w_min, w_max


def predict_trajectory(x, y, theta, v, w, dt, n_steps):
    """Forward simulate trajectory."""
    traj = np.zeros((n_steps + 1, 3))
    traj[0] = [x, y, theta]
    cx, cy, ct = x, y, theta
    for i in range(1, n_steps + 1):
        ct += w * dt
        cx += v * np.cos(ct) * dt
        cy += v * np.sin(ct) * dt
        traj[i] = [cx, cy, ct]
    return traj


def score_trajectory(trajectory, goal, obstacles, config):
    """Score trajectory: heading, clearance, velocity."""
    # Collision check
    if len(obstacles) > 0:
        for point in trajectory[::2]:  # check every other point
            dists = np.hypot(obstacles[:, 0] - point[0], obstacles[:, 1] - point[1])
            if np.min(dists) < config.robot_radius:
                return float('-inf')

    # Out of bounds check (10x10 environment)
    for point in trajectory:
        if point[0] < 0.2 or point[0] > 9.8 or point[1] < 0.2 or point[1] > 9.8:
            return float('-inf')

    # 1. Heading to goal
    final = trajectory[-1]
    angle_to_goal = np.arctan2(goal[1] - final[1], goal[0] - final[0])
    heading_diff = abs(angle_to_goal - final[2])
    heading_diff = min(heading_diff, 2 * np.pi - heading_diff)
    heading_score = np.pi - heading_diff

    # 2. Distance score (closeness to goal from final point)
    dist_to_goal = np.hypot(goal[0] - final[0], goal[1] - final[1])
    goal_progress = -dist_to_goal  # negative = closer is better

    # 3. Clearance
    if len(obstacles) > 0:
        min_dist = float('inf')
        for point in trajectory[::3]:
            dists = np.hypot(obstacles[:, 0] - point[0], obstacles[:, 1] - point[1])
            min_dist = min(min_dist, np.min(dists))
        clearance = min(min_dist, 3.0)
    else:
        clearance = 3.0

    # 4. Velocity
    if len(trajectory) > 1:
        vel = np.hypot(trajectory[-1, 0] - trajectory[-2, 0],
                       trajectory[-1, 1] - trajectory[-2, 1]) / config.dt
    else:
        vel = 0

    score = (config.heading_weight * heading_score +
             config.dist_weight * clearance +
             config.velocity_weight * vel +
             1.0 * goal_progress)  # added goal progress term
    return score


def dwa_control(state, goal, obstacles, config):
    """Select best (v, w) using DWA."""
    x, y, theta, v, w = state
    v_min, v_max, w_min, w_max = compute_dynamic_window(v, w, config)
    n_steps = int(config.predict_time / config.dt)

    best_score = float('-inf')
    best_v, best_w = 0.0, 0.0
    best_traj = None
    all_trajs = []

    # Filter nearby obstacles
    if len(obstacles) > 0:
        dists = np.hypot(obstacles[:, 0] - x, obstacles[:, 1] - y)
        nearby = obstacles[dists < config.predict_time * config.max_speed + 2.0]
    else:
        nearby = obstacles

    vi = v_min
    while vi <= v_max + 1e-6:
        wi = w_min
        while wi <= w_max + 1e-6:
            traj = predict_trajectory(x, y, theta, vi, wi, config.dt, n_steps)
            score = score_trajectory(traj, goal, nearby, config)
            all_trajs.append((traj, score))
            if score > best_score:
                best_score = score
                best_v = vi
                best_w = wi
                best_traj = traj
            wi += config.yaw_rate_resolution
        vi += config.v_resolution

    if best_traj is None:
        best_traj = predict_trajectory(x, y, theta, 0, 0, config.dt, n_steps)

    return best_v, best_w, best_traj, all_trajs


def main():
    print("=" * 60)
    print("Task 3: Dynamic Window Approach (DWA)")
    print("=" * 60)
    print()
    print("DWA samples velocities within a dynamic window and selects")
    print("the trajectory that best balances heading, clearance, and speed.")
    print()
    print("We use a custom open environment with scattered obstacles")
    print("(DWA is a local planner - it works best in open spaces).")
    print()

    config = DWAConfig()

    # Create scattered obstacles in a 10x10 environment
    np.random.seed(42)
    obstacles = np.array([
        [3.0, 3.0], [3.5, 3.2], [3.2, 3.5],  # cluster 1
        [5.0, 5.0], [5.3, 4.8], [4.8, 5.3],  # cluster 2
        [7.0, 2.0], [7.2, 2.3],               # cluster 3
        [2.0, 6.0], [2.3, 6.2],               # cluster 4
        [6.0, 7.0], [6.2, 7.3],               # cluster 5
        [4.0, 1.5], [8.0, 5.0],               # singles
    ])

    state = np.array([1.0, 1.0, 0.5, 0.0, 0.0])  # x, y, theta, v, w
    goal = np.array([8.5, 8.5])

    trajectory_history = [state[:2].copy()]
    sample_trajs = []  # store candidate trajectories at a few steps

    print(f"Start: ({state[0]:.1f}, {state[1]:.1f}), Goal: ({goal[0]:.1f}, {goal[1]:.1f})")
    print(f"Obstacles: {len(obstacles)}")
    print()

    reached = False
    for step in range(500):
        dist_to_goal = np.hypot(state[0] - goal[0], state[1] - goal[1])
        if dist_to_goal < config.goal_tolerance:
            print(f"Goal reached at step {step}! Distance: {dist_to_goal:.3f}m")
            reached = True
            break

        best_v, best_w, best_traj, all_trajs = dwa_control(state, goal, obstacles, config)

        if step % 20 == 0:
            sample_trajs.append((all_trajs, best_traj))
            if step % 60 == 0:
                print(f"  Step {step}: pos=({state[0]:.2f},{state[1]:.2f}), "
                      f"v={best_v:.2f}m/s, w={best_w:.2f}rad/s, dist={dist_to_goal:.2f}m")

        state[0] += best_v * np.cos(state[2]) * config.dt
        state[1] += best_v * np.sin(state[2]) * config.dt
        state[2] += best_w * config.dt
        state[3] = best_v
        state[4] = best_w
        trajectory_history.append(state[:2].copy())

    if not reached:
        dist = np.hypot(state[0]-goal[0], state[1]-goal[1])
        print(f"Did not reach goal. Final distance: {dist:.3f}m")

    traj = np.array(trajectory_history)
    total_dist = np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))
    straight = np.linalg.norm(goal - traj[0])
    print(f"\nPath length: {total_dist:.2f}m, Straight-line: {straight:.2f}m")
    if total_dist > 0:
        print(f"Efficiency: {straight / total_dist:.1%}")

    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))

    # Left: full trajectory
    ax = axes[0]
    ax.set_xlim(-0.5, 10.5)
    ax.set_ylim(-0.5, 10.5)
    ax.set_aspect('equal')

    # Draw obstacles
    for obs in obstacles:
        circle = plt.Circle(obs, config.robot_radius, color='gray', alpha=0.5)
        ax.add_patch(circle)
        ax.plot(obs[0], obs[1], 'ks', markersize=5)

    # Draw some candidate trajectories from first sample
    if sample_trajs:
        for tr, score in sample_trajs[0][0]:
            if score > float('-inf'):
                ax.plot(tr[:, 0], tr[:, 1], 'c-', linewidth=0.3, alpha=0.2)

    ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2, label='DWA Trajectory')
    ax.plot(traj[0, 0], traj[0, 1], 'go', markersize=12, label='Start')
    ax.plot(goal[0], goal[1], 'r*', markersize=15, label='Goal')
    ax.legend(fontsize=11)
    ax.set_title('DWA Navigation with Obstacle Avoidance')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True, alpha=0.3)

    # Right: velocity profile
    ax = axes[1]
    if len(traj) > 1:
        velocities = np.linalg.norm(np.diff(traj, axis=0), axis=1) / config.dt
        ax.plot(velocities, 'b-', linewidth=1.5, label='Linear velocity')
        ax.axhline(y=config.max_speed, color='r', linestyle='--', label='Max speed')
        ax.set_xlabel('Step')
        ax.set_ylabel('Velocity (m/s)')
        ax.legend()
        ax.grid(True, alpha=0.3)
    ax.set_title('Velocity Profile')

    plt.suptitle('Dynamic Window Approach Local Planner', fontsize=14)
    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, 'task3_dwa.png')
    plt.savefig(out_path, dpi=100)
    print(f"\nSaved plot to {out_path}")

    print("\n--- Key Concepts ---")
    print("- DWA searches within dynamically feasible velocities")
    print("- Three objectives: heading to goal, obstacle clearance, forward speed")
    print("- Trajectory simulation checks for collisions before execution")
    print("- In Nav2, DWA follows a global path (not just the final goal)")
    print("- The dynamic window shrinks at high speeds (less maneuverable)")


if __name__ == '__main__':
    main()
