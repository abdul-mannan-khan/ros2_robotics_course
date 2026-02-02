#!/usr/bin/env python3
"""
Task 3: Multi-Node Publisher-Subscriber Pipeline
==================================================

Goal: Build a three-node processing pipeline:

    SensorPublisher --[/sensor/ranges]--> TransformNode --[/processed/ranges]--> LoggerNode

1. SensorPublisher  : publishes simulated LaserScan data at 5 Hz
2. TransformNode    : subscribes to raw ranges, scales & filters, republishes
3. LoggerNode       : subscribes to processed ranges, logs statistics to a list

ROS2 Concepts Practised:
    - Multiple nodes communicating via topics
    - Chained publisher-subscriber pattern
    - Topic remapping concept
    - QoS profiles (sensor data vs. reliable)

Functions to implement:
    1. create_sensor_publisher()  -> Node
    2. create_transform_node()   -> Node
    3. create_logger_node()      -> Node
    4. run_pipeline(duration_sec) -> dict   (returns logger stats)

Run:
    python3 task3_pub_sub_system.py
"""

import sys, os, math, random
sys.path.insert(0, os.path.dirname(__file__))
import ros2_sim as rclpy
from ros2_sim import Node, LaserScan, Float64, String, QoSProfile, QOS_SENSOR_DATA


def create_sensor_publisher() -> Node:
    """Create 'sensor_publisher' node.

    - Publisher on '/sensor/ranges' (LaserScan, qos=QOS_SENSOR_DATA)
    - Timer at 5 Hz
    - Each tick: generate 360 range values (simulate a LiDAR):
        base_range = 5.0 + 2.0 * sin(angle + step*0.1)
        Add Gaussian noise (stddev 0.1)
      Publish as LaserScan with angle_min=-pi, angle_max=pi, angle_increment=2*pi/360

    Store node.step = 0 and increment each tick.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_sensor_publisher()")


def create_transform_node() -> Node:
    """Create 'transform_node' node.

    - Subscription on '/sensor/ranges' (LaserScan, qos=QOS_SENSOR_DATA)
    - Publisher on '/processed/ranges' (LaserScan, qos=10)
    - Callback:
        1. Filter out ranges > 25.0 (set to 0.0)
        2. Apply scaling factor 0.9 to all valid ranges
        3. Publish the modified LaserScan
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_transform_node()")


def create_logger_node() -> Node:
    """Create 'logger_node' node.

    - Subscription on '/processed/ranges' (LaserScan, qos=10)
    - node.log_entries = []
    - Callback: compute mean, min, max of ranges and append dict to log_entries
    - Log summary with get_logger().info(...)
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_logger_node()")


def run_pipeline(duration_sec: float = 3.0) -> dict:
    """Create all three nodes, spin them for *duration_sec*, return logger stats.

    Returns:
        dict with 'num_entries', 'avg_mean_range', 'avg_min_range', 'avg_max_range'
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement run_pipeline()")


def main():
    rclpy.init()
    stats = run_pipeline(3.0)
    print("\n=== Pipeline Statistics ===")
    for k, v in stats.items():
        print(f"  {k}: {v}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
