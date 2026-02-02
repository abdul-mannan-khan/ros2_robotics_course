#!/usr/bin/env python3
"""
Task 1: Basic Publisher Node
=============================

Goal: Create a ROS2-style node that publishes robot position data at 10 Hz.

The robot moves in a circular path. Every timer tick you compute the new
(x, y, theta) position and publish it as a Pose message on the topic
'/robot/position'.

ROS2 Concepts Practised:
    - Creating a Node
    - Creating a Publisher (topic name, message type, QoS depth)
    - Creating a Timer with a callback
    - Publishing messages
    - Spinning a node

Functions to implement:
    1. create_robot_publisher_node() -> Node
    2. position_callback()           (called by the timer)
    3. main()                        (create node, spin)

Run:
    python3 task1_publisher.py
"""

import sys, os, math
sys.path.insert(0, os.path.dirname(__file__))
import ros2_sim as rclpy
from ros2_sim import Node, Pose, Point, Quaternion


def create_robot_publisher_node() -> Node:
    """Create a node named 'robot_position_publisher' with:
    - A publisher on topic '/robot/position' (Pose, qos=10)
    - A timer at 10 Hz calling position_callback

    The node should store:
        node.step_count = 0        (incremented each callback)
        node.radius = 2.0          (circle radius in metres)
        node.angular_speed = 0.5   (rad/s)

    Returns:
        The configured Node.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_robot_publisher_node()")


def position_callback(node: Node):
    """Timer callback: compute position on a circle and publish.

    Use node.step_count to derive time:
        t = node.step_count * 0.1   (period = 1/10 Hz)
        theta = angular_speed * t
        x = radius * cos(theta)
        y = radius * sin(theta)

    Create a Pose message and publish it via node.publisher.
    Log the position with node.get_logger().info(...).
    Increment node.step_count.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement position_callback()")


def main():
    """Initialise rclpy, create the node, spin for 2 seconds, then shutdown."""
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement main()")


if __name__ == "__main__":
    main()
