#!/usr/bin/env python3
"""
Task 7: Full Robot System Integration
=======================================

Goal: Build a complete simulated robot system with five interconnected nodes:

    SensorNode --> LocalizationNode --> PlannerNode --> ControllerNode
                                                           |
                                          MonitorNode <----+

Topic map:
    /imu/data          (Imu)       SensorNode -> LocalizationNode
    /scan              (LaserScan) SensorNode -> PlannerNode
    /odom              (Odometry)  LocalizationNode -> PlannerNode, MonitorNode
    /cmd_vel           (Twist)     PlannerNode -> ControllerNode
    /controller/state  (Float64)   ControllerNode -> MonitorNode

Services:
    /reset_odom        LocalizationNode serves, MonitorNode can call

ROS2 Concepts Practised:
    - Multi-node architecture design
    - Mixed pub/sub and service communication
    - Parameters for tuning
    - System-level evaluation

Functions to implement:
    1. create_sensor_node()        -> Node
    2. create_localization_node()  -> Node
    3. create_planner_node()       -> Node
    4. create_controller_node()    -> Node
    5. create_monitor_node()       -> Node
    6. run_robot_system(duration)  -> list[Node]
    7. evaluate_system(nodes)      -> dict

Run:
    python3 task7_integration.py
"""

import sys, os, math
sys.path.insert(0, os.path.dirname(__file__))
import ros2_sim as rclpy
from ros2_sim import (
    Node, Pose, Twist, LaserScan, Imu, Odometry, Float64,
    Vector3, Point, Quaternion, Header, TransformRequest, TransformResponse,
    QOS_SENSOR_DATA,
)


def create_sensor_node() -> Node:
    """Create 'sensor_node' that publishes:
    - '/imu/data' (Imu) at 50 Hz - simulated IMU with small angular velocity
    - '/scan' (LaserScan) at 10 Hz - simulated 360-degree LiDAR

    Store node.imu_step = 0, node.scan_step = 0.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_sensor_node()")


def create_localization_node() -> Node:
    """Create 'localization_node' that:
    - Subscribes to '/imu/data' for orientation updates
    - Publishes '/odom' (Odometry) at 20 Hz (dead-reckoning)
    - Serves '/reset_odom' to reset position to origin

    Dead-reckoning: integrate angular velocity for heading,
    assume constant forward speed of 0.5 m/s.

    Store node.x, node.y, node.theta = 0.0, 0.0, 0.0
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_localization_node()")


def create_planner_node() -> Node:
    """Create 'planner_node' that:
    - Subscribes to '/odom' for current position
    - Subscribes to '/scan' for obstacle info
    - Publishes '/cmd_vel' (Twist) at 10 Hz

    Simple planning: drive toward goal (10, 10) with obstacle avoidance.
    If nearest scan range < 1.0, turn; otherwise drive toward goal.

    Store node.goal = (10.0, 10.0), node.latest_odom = None, node.latest_scan = None
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_planner_node()")


def create_controller_node() -> Node:
    """Create 'controller_node' that:
    - Subscribes to '/cmd_vel' (Twist)
    - Publishes '/controller/state' (Float64) at 20 Hz (current speed)
    - Parameters: max_linear_speed=1.0, max_angular_speed=2.0

    Apply velocity limits, publish clamped speed magnitude.

    Store node.current_linear = 0.0, node.current_angular = 0.0
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_controller_node()")


def create_monitor_node() -> Node:
    """Create 'monitor_node' that:
    - Subscribes to '/odom' (Odometry)
    - Subscribes to '/controller/state' (Float64)
    - Logs data to node.odom_log and node.speed_log
    - Counts total messages received in node.msg_count
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_monitor_node()")


def run_robot_system(duration_sec: float = 5.0) -> list:
    """Create all five nodes, spin them for duration_sec.
    Returns list of nodes.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement run_robot_system()")


def evaluate_system(nodes: list) -> dict:
    """Evaluate the system after running.

    Returns dict with:
        'total_messages'      : int   (from monitor)
        'final_position'      : (x, y)
        'distance_to_goal'    : float
        'avg_speed'           : float
        'nodes_active'        : int
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement evaluate_system()")


def main():
    rclpy.init()
    nodes = run_robot_system(5.0)
    results = evaluate_system(nodes)
    print("\n=== Robot System Evaluation ===")
    for k, v in results.items():
        print(f"  {k}: {v}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
