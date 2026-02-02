#!/usr/bin/env python3
"""
Week 5 - Task 3: Dynamic Window Approach (DWA) Local Planner
Implement DWA for local trajectory planning and obstacle avoidance.

DWA is one of Nav2's controller plugins. It samples velocities within
a dynamic window and selects the trajectory that best balances:
  - Heading toward the goal
  - Clearance from obstacles
  - Forward velocity (progress)
"""

import os
import numpy as np

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


class DWAConfig:
    """DWA configuration parameters."""
    def __init__(self):
        # Robot limits
        self.max_speed = 0.5        # m/s
        self.min_speed = 0.0        # m/s
        self.max_yaw_rate = 1.0     # rad/s
        self.max_accel = 0.5        # m/s^2
        self.max_yaw_accel = 1.0    # rad/s^2
        # Sampling
        self.v_resolution = 0.05    # m/s
        self.yaw_rate_resolution = 0.1  # rad/s
        # Trajectory
        self.dt = 0.1               # s
        self.predict_time = 2.0     # s
        # Scoring weights
        self.heading_weight = 1.0
        self.dist_weight = 1.0
        self.velocity_weight = 0.5
        # Safety
        self.robot_radius = 0.15    # m
        self.goal_tolerance = 0.2   # m


def compute_dynamic_window(v, w, config):
    """Compute the dynamic window of reachable velocities.

    The dynamic window is the intersection of:
      1. Velocity limits: [min_speed, max_speed] x [-max_yaw_rate, max_yaw_rate]
      2. Dynamic limits based on current velocity and max acceleration:
         [v - max_accel*dt, v + max_accel*dt] x [w - max_yaw_accel*dt, w + max_yaw_accel*dt]

    Args:
        v: current linear velocity (m/s)
        w: current angular velocity (rad/s)
        config: DWAConfig

    Returns:
        (v_min, v_max, w_min, w_max) tuple
    """
    # TODO: Implement dynamic window computation
    raise NotImplementedError("Implement compute_dynamic_window")


def predict_trajectory(x, y, theta, v, w, dt, n_steps):
    """Forward-simulate a trajectory given constant v, w.

    Args:
        x, y, theta: initial pose
        v: linear velocity
        w: angular velocity
        dt: time step
        n_steps: number of steps

    Returns:
        trajectory as Nx3 array of (x, y, theta)
    """
    # TODO: Implement trajectory prediction
    # Use simple kinematic model: x += v*cos(theta)*dt, etc.
    raise NotImplementedError("Implement predict_trajectory")


def score_trajectory(trajectory, goal, obstacles, config):
    """Score a trajectory based on heading, clearance, and velocity.

    Three objectives:
      1. heading: angle difference between final pose heading and direction to goal
      2. dist: minimum distance from trajectory to any obstacle
      3. velocity: forward speed (higher is better)

    Args:
        trajectory: Nx3 array
        goal: (x, y)
        obstacles: Mx2 array of obstacle positions
        config: DWAConfig

    Returns:
        float score (higher is better), or -inf if collision
    """
    # TODO: Implement trajectory scoring
    # Check for collisions first (return -inf)
    # Then compute weighted sum of normalized objectives
    raise NotImplementedError("Implement score_trajectory")


def dwa_control(state, goal, obstacles, config):
    """Select the best (v, w) using DWA.

    Args:
        state: (x, y, theta, v, w)
        goal: (x, y)
        obstacles: Mx2 array
        config: DWAConfig

    Returns:
        best_v, best_w, best_trajectory
    """
    # TODO: Implement DWA control loop
    # 1. Compute dynamic window
    # 2. Sample (v, w) pairs
    # 3. Predict trajectory for each
    # 4. Score each trajectory
    # 5. Return best
    raise NotImplementedError("Implement dwa_control")


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    config = DWAConfig()
    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    resolution = metadata[0]

    # Extract obstacle positions from grid
    occ = np.argwhere(grid_map == 100)
    obstacles = occ[:, ::-1].astype(float) * resolution  # (x, y) in meters

    # Start state and goal
    state = np.array([1.0, 1.0, 0.0, 0.0, 0.0])  # x, y, theta, v, w
    goal = np.array([3.5, 3.5])

    trajectory_history = [state[:2].copy()]

    # Run DWA for several steps
    for step in range(200):
        x, y = state[0], state[1]
        if np.hypot(x - goal[0], y - goal[1]) < config.goal_tolerance:
            print(f"Goal reached at step {step}!")
            break

        best_v, best_w, best_traj = dwa_control(state, goal, obstacles, config)
        # Update state
        state[0] += best_v * np.cos(state[2]) * config.dt
        state[1] += best_v * np.sin(state[2]) * config.dt
        state[2] += best_w * config.dt
        state[3] = best_v
        state[4] = best_w
        trajectory_history.append(state[:2].copy())

    # Plot
    traj = np.array(trajectory_history)
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100,
              extent=[0, 5, 0, 5])
    ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2, label='DWA Trajectory')
    ax.plot(state[0], state[1], 'bs', markersize=8)
    ax.plot(goal[0], goal[1], 'r*', markersize=15, label='Goal')
    ax.plot(traj[0, 0], traj[0, 1], 'go', markersize=10, label='Start')
    ax.legend()
    ax.set_title('DWA Local Planner')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task3_dwa.png')
    plt.savefig(out_path, dpi=100)
    print(f"Saved plot to {out_path}")


if __name__ == '__main__':
    main()
