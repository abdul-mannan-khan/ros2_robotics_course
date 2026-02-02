#!/usr/bin/env python3
"""
Week 5 - Task 6: Waypoint Following Navigation
Implement a waypoint follower that uses A* for global planning
and DWA for local control.

This mirrors Nav2's waypoint follower functionality which sequences
navigation goals and handles replanning.
"""

import os
import numpy as np

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


class WaypointFollower:
    """Manages a queue of waypoints and navigates through them.

    Attributes:
        waypoints: list of (x, y) goals
        current_index: index of current target waypoint
        tolerance: distance to consider waypoint reached (meters)
    """
    def __init__(self, waypoints, tolerance=0.2):
        # TODO: Initialize waypoint follower
        raise NotImplementedError("Implement WaypointFollower.__init__")

    def current_waypoint(self):
        """Return the current target waypoint or None if done."""
        # TODO
        raise NotImplementedError

    def advance(self):
        """Move to next waypoint. Returns True if more waypoints remain."""
        # TODO
        raise NotImplementedError

    def is_complete(self):
        """Check if all waypoints have been visited."""
        # TODO
        raise NotImplementedError


def plan_to_waypoint(current_pose, waypoint, grid_map, resolution):
    """Plan a global path from current_pose to waypoint using A*.

    Args:
        current_pose: (x, y, theta) in meters
        waypoint: (x, y) in meters
        grid_map: occupancy grid
        resolution: meters per cell

    Returns:
        path: list of (x, y) in meters, or empty list
    """
    # TODO: Convert to grid coords, run A*, convert back
    raise NotImplementedError("Implement plan_to_waypoint")


def follow_path(path, current_state, obstacles, config):
    """Take one step along path using DWA-like control.

    Args:
        path: list of (x, y) waypoints in meters
        current_state: (x, y, theta, v, w)
        obstacles: Mx2 obstacle positions
        config: controller parameters dict

    Returns:
        new_state: updated (x, y, theta, v, w)
    """
    # TODO: Find lookahead point on path, compute control toward it
    raise NotImplementedError("Implement follow_path")


def check_reached(current_pose, waypoint, tolerance=0.2):
    """Check if robot has reached the waypoint.

    Args:
        current_pose: (x, y, ...) robot position
        waypoint: (x, y) target
        tolerance: distance threshold (meters)

    Returns:
        bool
    """
    # TODO: Euclidean distance check
    raise NotImplementedError("Implement check_reached")


def run_waypoint_mission(waypoints, grid_map, resolution, start_pose):
    """Run complete waypoint following mission.

    Args:
        waypoints: Nx2 array of waypoint positions
        grid_map: occupancy grid
        resolution: map resolution
        start_pose: (x, y, theta) initial pose

    Returns:
        trajectory: list of (x, y) positions
        reached: list of booleans for each waypoint
        timing: list of step counts per waypoint
    """
    # TODO: Implement full mission loop
    # For each waypoint: plan path, follow it, check if reached
    raise NotImplementedError("Implement run_waypoint_mission")


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    waypoints = np.load(os.path.join(DATA_DIR, 'waypoints.npy'))
    resolution = metadata[0]

    start_pose = (1.0, 1.0, 0.0)
    trajectory, reached, timing = run_waypoint_mission(
        waypoints, grid_map, resolution, start_pose)

    # Plot
    traj = np.array(trajectory)
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100,
              extent=[0, 5, 0, 5])
    ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=1.5, label='Trajectory')

    for i, (wp, r) in enumerate(zip(waypoints, reached)):
        color = 'green' if r else 'red'
        ax.plot(wp[0], wp[1], 'o', color=color, markersize=10)
        ax.annotate(f'WP{i}', (wp[0]+0.05, wp[1]+0.05))

    ax.plot(start_pose[0], start_pose[1], 'gs', markersize=12, label='Start')
    ax.legend()
    ax.set_title(f'Waypoint Navigation ({sum(reached)}/{len(reached)} reached)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'task6_waypoint_navigation.png')
    plt.savefig(out_path, dpi=100)
    print(f"Saved plot to {out_path}")
    print(f"Waypoints reached: {sum(reached)}/{len(reached)}")
    for i, (r, t) in enumerate(zip(reached, timing)):
        print(f"  WP{i}: {'REACHED' if r else 'FAILED'} in {t} steps")


if __name__ == '__main__':
    main()
