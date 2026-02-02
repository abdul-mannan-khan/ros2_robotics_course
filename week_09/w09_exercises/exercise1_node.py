#!/usr/bin/env python3
"""
Week 9 Exercise 1: Depth to Pointcloud Node

Convert depth image to 3D pointcloud using camera intrinsics.

Subscribers: /depth_image (Image - 16UC1)
Publishers: /local_pointcloud (PointCloud2)
Parameters: fx, fy, cx, cy, max_depth
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField


class DepthToPointcloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')
        self.declare_parameter('fx', 160.0)
        self.declare_parameter('fy', 120.0)
        self.declare_parameter('cx', 160.0)
        self.declare_parameter('cy', 120.0)
        self.declare_parameter('max_depth', 10.0)
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.max_depth = self.get_parameter('max_depth').value
        self.create_subscription(Image, '/depth_image', self.depth_cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/local_pointcloud', 10)
        self.get_logger().info('Depth to Pointcloud node started')

    def depth_cb(self, msg):
        # TODO: Implement depth to 3D projection
        # 1. Parse depth image: np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        # 2. Convert from mm to meters: depth_m = depth_raw / 1000.0
        # 3. Create pixel coordinate grids: u, v = np.meshgrid(range(W), range(H))
        # 4. Filter by max_depth: mask = (depth_m > 0) & (depth_m < self.max_depth)
        # 5. Back-project: x = (u - cx) * depth / fx, y = (v - cy) * depth / fy, z = depth
        # 6. Stack into Nx3 array and publish as PointCloud2

        pass  # TODO: implement


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DepthToPointcloudNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
