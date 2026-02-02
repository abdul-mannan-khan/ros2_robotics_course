#!/usr/bin/env python3
"""
Task 6: Launch File Simulation
================================

Goal: Simulate a ROS2 launch system that reads a YAML config, instantiates
nodes with parameters and topic remappings, then verifies all connections.

ROS2 Concepts Practised:
    - Launch descriptions (nodes, parameters, remappings)
    - YAML parameter files
    - Topic remapping
    - System-level verification

Functions to implement:
    1. load_yaml_config(path) -> dict
    2. create_launch_description(config) -> list[dict]
    3. launch_nodes(description) -> list[Node]
    4. verify_connections(nodes) -> dict
    5. main()

Config YAML (lab_exercises/config/robot_params.yaml) defines:
    - sensor_node with topic remappings and parameters
    - controller_node with PID gains
    - monitor_node

Run:
    python3 task6_launch_system.py
"""

import sys, os, math
sys.path.insert(0, os.path.dirname(__file__))
import ros2_sim as rclpy
from ros2_sim import Node, LaserScan, Twist, Float64, Pose


def load_yaml_config(path: str) -> dict:
    """Load a YAML configuration file and return as a dict.

    Use the PyYAML library (import yaml). If not available, implement a
    minimal parser that handles the robot_params.yaml format.

    Returns:
        Parsed dict.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement load_yaml_config()")


def create_launch_description(config: dict) -> list:
    """Convert the loaded YAML config into a launch description.

    A launch description is a list of dicts, each with:
        {
            'name': str,            # node name
            'parameters': dict,     # param_name -> value
            'remappings': dict,     # original_topic -> remapped_topic
            'publishers': list,     # list of topic names this node publishes
            'subscribers': list,    # list of topic names this node subscribes to
        }

    Read these from config['nodes'].
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_launch_description()")


def launch_nodes(description: list) -> list:
    """Instantiate Node objects from the launch description.

    For each entry:
        1. Create a Node with the given name.
        2. Declare all parameters.
        3. Create publishers and subscriptions listed, applying remappings.
           (Use dummy callbacks for subscriptions.)

    Returns:
        List of instantiated Node objects.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement launch_nodes()")


def verify_connections(nodes: list) -> dict:
    """Verify that every subscription topic has at least one publisher.

    Returns:
        {
            'all_connected': bool,
            'published_topics': list[str],
            'subscribed_topics': list[str],
            'unmatched': list[str],  # subscribed but no publisher
        }
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement verify_connections()")


def main():
    rclpy.init()
    config_path = os.path.join(os.path.dirname(__file__), "config", "robot_params.yaml")
    config = load_yaml_config(config_path)
    description = create_launch_description(config)
    nodes = launch_nodes(description)
    result = verify_connections(nodes)

    print("\n=== Launch Verification ===")
    for k, v in result.items():
        print(f"  {k}: {v}")

    # Spin briefly
    rclpy.spin_nodes(nodes, duration_sec=2.0)
    print("\nLaunch system completed successfully.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
