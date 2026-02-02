#!/usr/bin/env python3
"""
Task 2: Subscriber with Processing
====================================

Goal: Create a node that subscribes to position data published on
'/robot/position' (Pose messages) and computes running statistics.

ROS2 Concepts Practised:
    - Creating a Subscription (topic, message type, callback, QoS)
    - Processing messages inside a callback
    - Combining a Publisher node with a Subscriber node

Functions to implement:
    1. create_position_subscriber() -> Node
    2. position_callback(msg: Pose)
    3. compute_statistics() -> dict
    4. main()

Run:
    python3 task2_subscriber.py
"""

import sys, os, math
sys.path.insert(0, os.path.dirname(__file__))
import ros2_sim as rclpy
from ros2_sim import Node, Pose, Point


def create_position_subscriber() -> Node:
    """Create a node named 'position_subscriber' that:
    - Subscribes to '/robot/position' (Pose, qos=10) with position_callback
    - Stores received positions in node.positions = []  (list of (x, y, theta) tuples)

    Returns:
        The configured Node.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_position_subscriber()")


def position_callback(node: Node, msg: Pose):
    """Callback invoked for every incoming Pose message.

    Extract x, y from msg.position and theta from msg.orientation.z.
    Append (x, y, theta) to node.positions.
    Log the received count every 10 messages.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement position_callback()")


def compute_statistics(positions: list) -> dict:
    """Compute statistics over the collected positions.

    Args:
        positions: list of (x, y, theta) tuples

    Returns:
        dict with keys:
            'count'  : int
            'mean_x' : float
            'mean_y' : float
            'std_x'  : float
            'std_y'  : float
            'min_x', 'max_x', 'min_y', 'max_y' : float
            'path_length' : float  (sum of Euclidean distances between consecutive points)
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement compute_statistics()")


def main():
    """Create both a publisher (from Task 1 pattern) and this subscriber,
    spin them together for 2 seconds, then print statistics."""
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement main()")


if __name__ == "__main__":
    main()
