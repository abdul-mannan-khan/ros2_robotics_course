#!/usr/bin/env python3
"""Week 9 Solution 1: Depth to Pointcloud (Complete)"""
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
        H, W = msg.height, msg.width
        depth_raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(H, W)
        depth_m = depth_raw.astype(np.float32) / 1000.0
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        mask = (depth_m > 0.1) & (depth_m < self.max_depth)
        d = depth_m[mask]
        x = (u[mask] - self.cx) * d / self.fx
        y = (v[mask] - self.cy) * d / self.fy
        z = d
        pts = np.stack([x, y, z], axis=1).astype(np.float32)
        pc = PointCloud2()
        pc.header = msg.header
        pc.header.frame_id = 'camera'
        pc.height, pc.width = 1, len(pts)
        pc.fields = [PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate('xyz')]
        pc.is_bigendian = False
        pc.point_step, pc.row_step = 12, 12*len(pts)
        pc.data = pts.tobytes()
        pc.is_dense = True
        self.pub.publish(pc)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DepthToPointcloudNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
