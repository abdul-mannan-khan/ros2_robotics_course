#!/usr/bin/env python3
"""
Week 1 - Exercise 1: Point Cloud Subscriber Node
Subscribe to /velodyne_points (PointCloud2), count points, compute stats,
publish on /lidar_stats (String).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class PointCloudSubscriberNode(Node):
    def __init__(self):
        super().__init__("point_cloud_subscriber_node")
        self.get_logger().info("Point Cloud Subscriber Node started")

        # TODO 1: Create a subscriber to /velodyne_points (PointCloud2)
        # self.subscription = self.create_subscription(
        #     PointCloud2, "/velodyne_points", self.pointcloud_callback, 10)

        # TODO 2: Create a publisher for /lidar_stats (String)
        # self.stats_pub = self.create_publisher(String, "/lidar_stats", 10)

        self.scan_count = 0

    def pointcloud_callback(self, msg: PointCloud2):
        self.scan_count += 1

        # TODO 3: Get number of points
        # num_points = msg.width * msg.height

        # TODO 4: Parse point cloud data into numpy array
        # points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        # xyz = points[:, :3]

        # TODO 5: Compute range statistics
        # ranges = np.linalg.norm(xyz, axis=1)
        # min_range = float(np.min(ranges))
        # max_range = float(np.max(ranges))
        # mean_range = float(np.mean(ranges))

        # TODO 6: Publish stats
        # stats_msg = String()
        # stats_msg.data = f"Scan {self.scan_count}: pts={num_points}, min={min_range:.2f}, max={max_range:.2f}, mean={mean_range:.2f}"
        # self.stats_pub.publish(stats_msg)

        self.get_logger().info(f"Scan {self.scan_count} received")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
