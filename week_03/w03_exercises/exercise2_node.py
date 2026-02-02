#!/usr/bin/env python3
"""
Week 3 - Exercise 2: Obstacle Detector Node
Subscribe to /scan (LaserScan), find closest obstacle per sector.
Publish /obstacle_sectors (Float32MultiArray).
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__("obstacle_detector_node")

        # TODO 1: Declare parameters
        # self.declare_parameter("num_sectors", 4)
        # self.declare_parameter("warning_distance", 0.5)
        self.num_sectors = 4  # front, left, back, right
        self.warning_distance = 0.5

        # TODO 2: Create subscriber to /scan
        # self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        # TODO 3: Create publisher for /obstacle_sectors
        # self.sectors_pub = self.create_publisher(Float32MultiArray, "/obstacle_sectors", 10)

        self.get_logger().info(f"Obstacle Detector started ({self.num_sectors} sectors)")

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # TODO 4: Replace invalid readings
        # ranges[ranges < msg.range_min] = float("inf")
        # ranges[ranges > msg.range_max] = float("inf")

        # TODO 5: Divide scan into sectors and find min range per sector
        # n = len(ranges)
        # sector_size = n // self.num_sectors
        # min_ranges = []
        # for s in range(self.num_sectors):
        #     start = s * sector_size
        #     end = start + sector_size if s < self.num_sectors - 1 else n
        #     sector_ranges = ranges[start:end]
        #     min_r = float(np.min(sector_ranges))
        #     min_ranges.append(min_r)

        # TODO 6: Publish sector data
        # out = Float32MultiArray()
        # out.data = [float(r) for r in min_ranges]
        # self.sectors_pub.publish(out)

        # TODO 7: Warn if any sector is too close
        # for i, r in enumerate(min_ranges):
        #     if r < self.warning_distance:
        #         labels = ["front", "left", "back", "right"]
        #         self.get_logger().warn(f"Obstacle in {labels[i]}: {r:.2f}m")

        pass


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
