#!/usr/bin/env python3
"""
Solution: Task 2 - Subscriber with Processing
===============================================
Demonstrates subscribing to position data and computing statistics.
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
import ros2_sim as rclpy
from ros2_sim import Node, Pose, Point, Quaternion


# ---- Publisher node (reused from Task 1 pattern) ----

def create_publisher_node() -> Node:
    node = Node('robot_position_publisher')
    node.publisher = node.create_publisher(Pose, '/robot/position', qos=10)
    node.step_count = 0
    node.radius = 2.0
    node.angular_speed = 0.5

    def pub_cb():
        t = node.step_count * 0.1
        theta = node.angular_speed * t
        x = node.radius * math.cos(theta)
        y = node.radius * math.sin(theta)
        msg = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=theta, w=1.0),
        )
        node.publisher.publish(msg)
        node.step_count += 1

    node.create_timer(0.1, pub_cb)
    return node


# ---- Subscriber node ----

def create_position_subscriber() -> Node:
    node = Node('position_subscriber')
    node.positions = []

    def callback(msg):
        position_callback(node, msg)

    node.create_subscription(Pose, '/robot/position', callback, qos=10)
    node.get_logger().info("Subscriber node created - listening on '/robot/position'")
    return node


def position_callback(node: Node, msg: Pose):
    x = msg.position.x
    y = msg.position.y
    theta = msg.orientation.z
    node.positions.append((x, y, theta))

    if len(node.positions) % 10 == 0:
        node.get_logger().info(f"Received {len(node.positions)} positions so far")


def compute_statistics(positions: list) -> dict:
    if not positions:
        return {'count': 0}

    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    n = len(positions)

    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    std_x = math.sqrt(sum((x - mean_x) ** 2 for x in xs) / n)
    std_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys) / n)

    path_length = 0.0
    for i in range(1, n):
        dx = positions[i][0] - positions[i - 1][0]
        dy = positions[i][1] - positions[i - 1][1]
        path_length += math.sqrt(dx * dx + dy * dy)

    return {
        'count': n,
        'mean_x': round(mean_x, 4),
        'mean_y': round(mean_y, 4),
        'std_x': round(std_x, 4),
        'std_y': round(std_y, 4),
        'min_x': round(min(xs), 4),
        'max_x': round(max(xs), 4),
        'min_y': round(min(ys), 4),
        'max_y': round(max(ys), 4),
        'path_length': round(path_length, 4),
    }


def main():
    print("=" * 60)
    print("Task 2 Solution: Subscriber with Processing")
    print("=" * 60)
    print()
    print("ROS2 Concepts demonstrated:")
    print("  - Subscription (topic, msg type, callback, QoS)")
    print("  - Processing data inside a callback")
    print("  - Running publisher + subscriber together")
    print()

    rclpy.init()
    pub_node = create_publisher_node()
    sub_node = create_position_subscriber()

    # Spin both nodes for 2 seconds
    rclpy.spin_nodes([pub_node, sub_node], duration_sec=2.0)

    stats = compute_statistics(sub_node.positions)
    print("\n=== Position Statistics ===")
    for k, v in stats.items():
        print(f"  {k:15s}: {v}")

    # Save plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        if sub_node.positions:
            xs = [p[0] for p in sub_node.positions]
            ys = [p[1] for p in sub_node.positions]
            fig, ax = plt.subplots(1, 1, figsize=(6, 6))
            ax.plot(xs, ys, 'b.-', markersize=3)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title('Robot Circular Path')
            ax.set_aspect('equal')
            ax.grid(True)
            plot_path = os.path.join(os.path.dirname(__file__), 'task2_positions.png')
            fig.savefig(plot_path, dpi=100, bbox_inches='tight')
            print(f"\nPlot saved to {plot_path}")
    except ImportError:
        print("\nmatplotlib not available - skipping plot.")

    pub_node.destroy_node()
    sub_node.destroy_node()
    rclpy.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
