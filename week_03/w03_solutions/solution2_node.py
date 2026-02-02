#!/usr/bin/env python3
"""Week 3 - Solution 2: Obstacle Detector Node (Complete Implementation)"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__("obstacle_detector_node")
        self.declare_parameter("num_sectors", 4)
        self.declare_parameter("warning_distance", 0.5)
        self.num_sectors = self.get_parameter("num_sectors").value
        self.warning_distance = self.get_parameter("warning_distance").value
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.sectors_pub = self.create_publisher(Float32MultiArray, "/obstacle_sectors", 10)
        self.sector_labels = ["front", "left", "back", "right"]
        self.get_logger().info(f"Obstacle Detector started ({self.num_sectors} sectors)")

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[ranges < msg.range_min] = float("inf")
        ranges[ranges > msg.range_max] = float("inf")
        ranges[np.isnan(ranges)] = float("inf")
        n = len(ranges)
        sector_size = n // self.num_sectors
        min_ranges = []
        for s in range(self.num_sectors):
            start = s * sector_size
            end = start + sector_size if s < self.num_sectors - 1 else n
            sector = ranges[start:end]
            min_r = float(np.min(sector)) if len(sector) > 0 else float("inf")
            min_ranges.append(min_r)
        out = Float32MultiArray()
        out.data = [float(r) for r in min_ranges]
        self.sectors_pub.publish(out)
        for i, r in enumerate(min_ranges):
            if r < self.warning_distance:
                label = self.sector_labels[i] if i < len(self.sector_labels) else f"sector{i}"
                self.get_logger().warn(f"Obstacle in {label}: {r:.2f}m")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
