#!/usr/bin/env python3
"""
Solution: Task 3 - Multi-Node Publisher-Subscriber Pipeline
=============================================================
Demonstrates chaining three nodes: SensorPublisher -> TransformNode -> LoggerNode.
"""

import sys, os, math, random
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
import ros2_sim as rclpy
from ros2_sim import Node, LaserScan, Header, QOS_SENSOR_DATA


def create_sensor_publisher() -> Node:
    node = Node('sensor_publisher')
    node.publisher = node.create_publisher(LaserScan, '/sensor/ranges', qos=QOS_SENSOR_DATA)
    node.step = 0

    def timer_cb():
        scan = LaserScan(
            header=Header(frame_id='laser_frame'),
            angle_min=-math.pi,
            angle_max=math.pi,
            angle_increment=2.0 * math.pi / 360,
            range_min=0.1,
            range_max=30.0,
        )
        ranges = []
        for i in range(360):
            angle = -math.pi + i * scan.angle_increment
            base = 5.0 + 2.0 * math.sin(angle + node.step * 0.1)
            noisy = base + random.gauss(0.0, 0.1)
            ranges.append(max(scan.range_min, noisy))
        scan.ranges = ranges
        node.publisher.publish(scan)
        if node.step % 5 == 0:
            node.get_logger().info(f"Published scan #{node.step} ({len(ranges)} ranges)")
        node.step += 1

    node.create_timer(0.2, timer_cb)  # 5 Hz
    return node


def create_transform_node() -> Node:
    node = Node('transform_node')
    node.publisher = node.create_publisher(LaserScan, '/processed/ranges', qos=10)
    node.processed_count = 0

    def callback(msg: LaserScan):
        filtered = []
        for r in msg.ranges:
            if r > 25.0:
                filtered.append(0.0)
            else:
                filtered.append(r * 0.9)
        msg.ranges = filtered
        node.publisher.publish(msg)
        node.processed_count += 1

    node.create_subscription(LaserScan, '/sensor/ranges', callback, qos=QOS_SENSOR_DATA)
    node.get_logger().info("TransformNode: subscribing to '/sensor/ranges', publishing to '/processed/ranges'")
    return node


def create_logger_node() -> Node:
    node = Node('logger_node')
    node.log_entries = []

    def callback(msg: LaserScan):
        valid = [r for r in msg.ranges if r > 0.0]
        if valid:
            entry = {
                'mean': sum(valid) / len(valid),
                'min': min(valid),
                'max': max(valid),
                'count': len(valid),
            }
        else:
            entry = {'mean': 0.0, 'min': 0.0, 'max': 0.0, 'count': 0}
        node.log_entries.append(entry)
        if len(node.log_entries) % 5 == 0:
            node.get_logger().info(
                f"Log #{len(node.log_entries)}: mean={entry['mean']:.2f}, "
                f"min={entry['min']:.2f}, max={entry['max']:.2f}"
            )

    node.create_subscription(LaserScan, '/processed/ranges', callback, qos=10)
    return node


def run_pipeline(duration_sec: float = 3.0) -> dict:
    sensor = create_sensor_publisher()
    transform = create_transform_node()
    logger = create_logger_node()

    print("\nPipeline topology:")
    print("  sensor_publisher --[/sensor/ranges]--> transform_node --[/processed/ranges]--> logger_node")
    print(f"\nRunning pipeline for {duration_sec}s ...\n")

    rclpy.spin_nodes([sensor, transform, logger], duration_sec=duration_sec)

    entries = logger.log_entries
    if entries:
        stats = {
            'num_entries': len(entries),
            'avg_mean_range': round(sum(e['mean'] for e in entries) / len(entries), 3),
            'avg_min_range': round(sum(e['min'] for e in entries) / len(entries), 3),
            'avg_max_range': round(sum(e['max'] for e in entries) / len(entries), 3),
        }
    else:
        stats = {'num_entries': 0, 'avg_mean_range': 0, 'avg_min_range': 0, 'avg_max_range': 0}
    return stats


def main():
    print("=" * 60)
    print("Task 3 Solution: Multi-Node Pub/Sub Pipeline")
    print("=" * 60)
    print()
    print("ROS2 Concepts demonstrated:")
    print("  - Multiple nodes communicating via topics")
    print("  - Chained publisher-subscriber pattern")
    print("  - QoS profiles (sensor data vs. reliable)")
    print()

    rclpy.init()
    stats = run_pipeline(3.0)

    print("\n=== Pipeline Statistics ===")
    for k, v in stats.items():
        print(f"  {k}: {v}")

    # Save plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 4))
        ax.bar(range(len(stats)), list(stats.values()), tick_label=list(stats.keys()))
        ax.set_ylabel('Value')
        ax.set_title('Pipeline Statistics')
        plot_path = os.path.join(os.path.dirname(__file__), 'task3_pipeline.png')
        fig.savefig(plot_path, dpi=100, bbox_inches='tight')
        print(f"\nPlot saved to {plot_path}")
    except ImportError:
        pass

    rclpy.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
