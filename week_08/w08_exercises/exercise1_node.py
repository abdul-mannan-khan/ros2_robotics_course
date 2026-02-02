#!/usr/bin/env python3
"""
Week 8 Exercise 1: 3D Occupancy Map Builder Node

Subscribers: /os1_cloud_node/points (PointCloud2), /odom (Odometry)
Publishers: /occupancy_grid_3d (PointCloud2 - occupied voxel centers)
Parameters: voxel_resolution (0.2), map_bounds_min/max
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry


class OccupancyMapBuilderNode(Node):
    def __init__(self):
        super().__init__('occupancy_map_builder')
        self.declare_parameter('voxel_resolution', 0.2)
        self.declare_parameter('map_bounds_min', [-25.0, -10.0, -1.0])
        self.declare_parameter('map_bounds_max', [25.0, 10.0, 5.0])
        self.res = self.get_parameter('voxel_resolution').value
        self.bmin = np.array(self.get_parameter('map_bounds_min').value)
        self.bmax = np.array(self.get_parameter('map_bounds_max').value)
        self.occupied = set()
        self.pose = None
        self.create_subscription(PointCloud2, '/os1_cloud_node/points', self.cloud_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/occupancy_grid_3d', 10)
        self.create_timer(1.0, self.publish_grid)
        self.get_logger().info('3D Occupancy Map Builder started')

    def odom_cb(self, msg):
        self.pose = msg.pose.pose

    def cloud_cb(self, msg):
        if self.pose is None or msg.width == 0:
            return
        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :3]
        # TODO: Voxel insertion with pose transform
        # 1. Get translation from self.pose.position (tx, ty, tz)
        # 2. For each point, transform to world frame: wx = pt[0]+tx, etc.
        #    (Simplified: ignoring rotation for exercise)
        # 3. Check bounds: all(self.bmin <= [wx,wy,wz]) and all([wx,wy,wz] <= self.bmax)
        # 4. Compute voxel index: vi = int((wx - bmin[0]) / self.res), etc.
        # 5. self.occupied.add((vi, vj, vk))
        pass

    def publish_grid(self):
        if not self.occupied:
            return
        centers = np.array([[self.bmin[0]+(vi+.5)*self.res,
                             self.bmin[1]+(vj+.5)*self.res,
                             self.bmin[2]+(vk+.5)*self.res]
                            for vi,vj,vk in self.occupied], dtype=np.float32)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.height, msg.width = 1, len(centers)
        msg.fields = [PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate('xyz')]
        msg.is_bigendian = False
        msg.point_step, msg.row_step = 12, 12*len(centers)
        msg.data = centers.tobytes()
        msg.is_dense = True
        self.pub.publish(msg)
        self.get_logger().info(f'Published {len(centers)} voxels')

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapBuilderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
