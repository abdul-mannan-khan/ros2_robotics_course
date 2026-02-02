#!/usr/bin/env python3
"""
Week 5 - Task 5: Behavior Trees for Navigation
Implement a behavior tree framework similar to Nav2's BT Navigator.

Nav2 uses behavior trees (via BehaviorTree.CPP) to orchestrate navigation:
  - Sequence: execute children in order, fail if any fails
  - Fallback: try children until one succeeds
  - Action: leaf nodes that do work
  - Condition: leaf nodes that check state

The Nav2 default BT: NavigateToPose -> Recovery (Spin, Wait, Backup)
"""

import os
import numpy as np
from enum import Enum

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


class Status(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class BTNode:
    """Base behavior tree node."""
    def __init__(self, name="BTNode"):
        self.name = name
        self.children = []

    def tick(self, blackboard):
        """Execute this node. Override in subclasses.

        Args:
            blackboard: dict shared state

        Returns:
            Status enum
        """
        raise NotImplementedError("Implement tick()")

    def add_child(self, child):
        self.children.append(child)
        return self


class Sequence(BTNode):
    """Execute children in order. Fail if any child fails.

    Returns:
        SUCCESS if all children succeed
        FAILURE if any child fails
        RUNNING if a child is running
    """
    def tick(self, blackboard):
        # TODO: Implement sequence logic
        raise NotImplementedError("Implement Sequence.tick")


class Fallback(BTNode):
    """Try children until one succeeds (selector node).

    Returns:
        SUCCESS if any child succeeds
        FAILURE if all children fail
        RUNNING if a child is running
    """
    def tick(self, blackboard):
        # TODO: Implement fallback logic
        raise NotImplementedError("Implement Fallback.tick")


class Action(BTNode):
    """Leaf action node. Override tick() with specific behavior."""
    pass


class Condition(BTNode):
    """Leaf condition node. Returns SUCCESS if condition is true, FAILURE otherwise."""
    pass


class NavigateToPose(Action):
    """Move the robot one step toward the current goal using simple control.

    Reads from blackboard: 'robot_pose', 'current_goal', 'grid_map', 'resolution'
    Writes to blackboard: 'robot_pose'
    """
    def tick(self, blackboard):
        # TODO: Implement navigation action
        # Move robot toward goal, check for obstacles
        # Return RUNNING if still moving, SUCCESS if reached, FAILURE if stuck
        raise NotImplementedError("Implement NavigateToPose.tick")


class IsPathBlocked(Condition):
    """Check if the direct path to goal is blocked.

    Reads: 'robot_pose', 'current_goal', 'grid_map', 'resolution'
    Returns SUCCESS if path is blocked (condition is true), FAILURE if clear.
    """
    def tick(self, blackboard):
        # TODO: Check if line from robot to goal intersects obstacles
        raise NotImplementedError("Implement IsPathBlocked.tick")


class RecoveryBehavior(Action):
    """Recovery: rotate in place or back up.

    Reads/writes: 'robot_pose', 'recovery_count'
    """
    def tick(self, blackboard):
        # TODO: Implement recovery (spin or backup)
        raise NotImplementedError("Implement RecoveryBehavior.tick")


def build_nav_bt():
    """Build a Nav2-like navigation behavior tree.

    Structure:
      Fallback("MainBT")
        Sequence("Navigate")
          Condition("IsPathClear")  -- inverted IsPathBlocked
          Action("NavigateToPose")
        Sequence("Recovery")
          Condition("IsPathBlocked")
          Action("RecoveryBehavior")

    Returns:
        root BTNode
    """
    # TODO: Build and return the behavior tree
    raise NotImplementedError("Implement build_nav_bt")


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    waypoints = np.load(os.path.join(DATA_DIR, 'waypoints.npy'))
    resolution = metadata[0]

    bt = build_nav_bt()

    blackboard = {
        'robot_pose': np.array([1.0, 1.0, 0.0]),
        'grid_map': grid_map,
        'resolution': resolution,
        'recovery_count': 0,
        'trajectory': [],
        'bt_log': [],
    }

    # Navigate through waypoints
    for wp_idx, wp in enumerate(waypoints[:5]):
        blackboard['current_goal'] = wp
        print(f"\nNavigating to waypoint {wp_idx}: {wp}")

        for step in range(300):
            status = bt.tick(blackboard)
            blackboard['trajectory'].append(blackboard['robot_pose'][:2].copy())
            blackboard['bt_log'].append((wp_idx, step, status.name))

            if status == Status.SUCCESS:
                print(f"  Reached waypoint {wp_idx} in {step} steps")
                break
            elif status == Status.FAILURE:
                print(f"  Failed to reach waypoint {wp_idx}")
                break

    # Plot
    traj = np.array(blackboard['trajectory'])
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100,
              extent=[0, 5, 0, 5])
    ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=1, alpha=0.7, label='Trajectory')
    for i, wp in enumerate(waypoints[:5]):
        ax.plot(wp[0], wp[1], 'r*', markersize=12)
        ax.annotate(f'WP{i}', (wp[0], wp[1]), fontsize=9)
    ax.legend()
    ax.set_title('Behavior Tree Navigation')

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'task5_behavior_tree.png')
    plt.savefig(out_path, dpi=100)
    print(f"\nSaved plot to {out_path}")
    print(f"BT log entries: {len(blackboard['bt_log'])}")


if __name__ == '__main__':
    main()
