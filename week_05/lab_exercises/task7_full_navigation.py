#!/usr/bin/env python3
"""
Week 5 - Task 7: Full Navigation Stack Integration
Combine all components into a complete navigation system:
  - Map server (load map)
  - AMCL (localization)
  - Costmap (layered costmap)
  - Global planner (A*)
  - Local planner (DWA)
  - Behavior tree (orchestration)
  - Waypoint follower

This mirrors the full Nav2 stack running together.
"""

import os
import numpy as np

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


class NavigationStack:
    """Complete navigation stack combining all components.

    Components:
        - map: occupancy grid
        - costmap: layered costmap
        - localization: particle filter (simplified)
        - global_planner: A*
        - local_planner: DWA
        - behavior_tree: navigation BT
    """

    def __init__(self, grid_map, resolution=0.05):
        # TODO: Initialize all components
        raise NotImplementedError("Implement NavigationStack.__init__")

    def load_map(self):
        """Load and process the map, build costmap."""
        # TODO
        raise NotImplementedError

    def initialize_localization(self, initial_pose, n_particles=200):
        """Set up particle filter with initial pose."""
        # TODO
        raise NotImplementedError

    def navigate_to_pose(self, goal, max_steps=500):
        """Navigate to a single goal pose.

        Returns:
            success: bool
            trajectory: list of poses
            steps: number of steps taken
        """
        # TODO: Use global planner + local planner + localization
        raise NotImplementedError

    def run(self, goals, start_pose):
        """Run navigation to multiple goals.

        Returns:
            results: list of (success, trajectory, steps) per goal
        """
        # TODO
        raise NotImplementedError


def evaluate_navigation(results, goals):
    """Evaluate navigation performance.

    Metrics:
        - Success rate: fraction of goals reached
        - Total path length: sum of trajectory distances
        - Path smoothness: average heading change
        - Total time: sum of steps

    Returns:
        dict of metrics
    """
    # TODO: Compute evaluation metrics
    raise NotImplementedError("Implement evaluate_navigation")


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    waypoints = np.load(os.path.join(DATA_DIR, 'waypoints.npy'))
    resolution = metadata[0]

    nav = NavigationStack(grid_map, resolution)
    nav.load_map()

    start_pose = np.array([1.0, 1.0, 0.0])
    nav.initialize_localization(start_pose)

    results = nav.run(waypoints, start_pose)
    metrics = evaluate_navigation(results, waypoints)

    # 4-panel evaluation plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 14))

    # Panel 1: Trajectories on map
    ax = axes[0, 0]
    ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100,
              extent=[0, 5, 0, 5])
    for i, (success, traj, steps) in enumerate(results):
        if len(traj) > 0:
            t = np.array(traj)
            color = 'green' if success else 'red'
            ax.plot(t[:, 0], t[:, 1], '-', color=color, linewidth=1.5, alpha=0.7)
    for i, wp in enumerate(waypoints):
        ax.plot(wp[0], wp[1], 'r*', markersize=10)
        ax.annotate(f'{i}', (wp[0], wp[1]))
    ax.set_title('Navigation Trajectories')

    # Panel 2: Success per waypoint
    ax = axes[0, 1]
    successes = [r[0] for r in results]
    colors = ['green' if s else 'red' for s in successes]
    ax.bar(range(len(successes)), [1]*len(successes), color=colors)
    ax.set_title(f'Success Rate: {metrics.get("success_rate", 0):.0%}')
    ax.set_xlabel('Waypoint')
    ax.set_ylabel('Reached')

    # Panel 3: Path lengths
    ax = axes[1, 0]
    lengths = []
    for success, traj, steps in results:
        if len(traj) > 1:
            t = np.array(traj)
            length = np.sum(np.linalg.norm(np.diff(t[:, :2], axis=0), axis=1))
        else:
            length = 0
        lengths.append(length)
    ax.bar(range(len(lengths)), lengths)
    ax.set_title(f'Path Lengths (total: {sum(lengths):.2f}m)')
    ax.set_xlabel('Waypoint')
    ax.set_ylabel('Length (m)')

    # Panel 4: Steps per waypoint
    ax = axes[1, 1]
    steps_list = [r[2] for r in results]
    ax.bar(range(len(steps_list)), steps_list)
    ax.set_title(f'Steps per Waypoint (total: {sum(steps_list)})')
    ax.set_xlabel('Waypoint')
    ax.set_ylabel('Steps')

    plt.suptitle('Full Navigation Stack Evaluation', fontsize=14)
    plt.tight_layout()

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'task7_full_navigation.png')
    plt.savefig(out_path, dpi=100)
    print(f"Saved plot to {out_path}")

    print("\nNavigation Metrics:")
    for k, v in metrics.items():
        print(f"  {k}: {v}")


if __name__ == '__main__':
    main()
