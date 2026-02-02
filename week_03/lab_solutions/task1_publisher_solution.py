#!/usr/bin/env python3
"""
Solution: Task 1 - Basic Publisher Node
=========================================
Demonstrates creating a ROS2 node that publishes robot position at 10 Hz.
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
import ros2_sim as rclpy
from ros2_sim import Node, Pose, Point, Quaternion


def create_robot_publisher_node() -> Node:
    node = Node('robot_position_publisher')
    node.publisher = node.create_publisher(Pose, '/robot/position', qos=10)
    node.step_count = 0
    node.radius = 2.0
    node.angular_speed = 0.5

    node.create_timer(0.1, lambda: position_callback(node))
    node.get_logger().info("Publisher node created - publishing on '/robot/position' at 10 Hz")
    return node


def position_callback(node: Node):
    t = node.step_count * 0.1
    theta = node.angular_speed * t
    x = node.radius * math.cos(theta)
    y = node.radius * math.sin(theta)

    msg = Pose(
        position=Point(x=x, y=y, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=theta, w=1.0),
    )
    node.publisher.publish(msg)

    if node.step_count % 5 == 0:
        node.get_logger().info(
            f"[tick {node.step_count:3d}] Published position: "
            f"x={x:.3f}, y={y:.3f}, theta={theta:.3f} rad"
        )
    node.step_count += 1


def main():
    print("=" * 60)
    print("Task 1 Solution: Basic Publisher Node")
    print("=" * 60)
    print()
    print("ROS2 Concepts demonstrated:")
    print("  - Node creation")
    print("  - Publisher (topic, msg type, QoS)")
    print("  - Timer callback at 10 Hz")
    print("  - Publishing Pose messages")
    print()

    rclpy.init()
    node = create_robot_publisher_node()

    # Spin for 2 seconds (20 timer ticks at 10 Hz)
    rclpy.spin(node, duration_sec=2.0)

    print(f"\nTotal messages published: {node.publisher.publish_count}")
    print(f"Final step count: {node.step_count}")

    node.destroy_node()
    rclpy.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
