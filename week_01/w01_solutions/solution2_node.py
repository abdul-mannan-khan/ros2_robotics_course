#!/usr/bin/env python3
"""Week 1 - Solution 2: Voxel Grid Filter Node (Complete Implementation)"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class VoxelGridFilterNode(Node):
    def __init__(self):
        super().__init__("voxel_grid_filter_node")
        self.declare_parameter("voxel_size", 0.3)
        self.voxel_size = self.get_parameter("voxel_size").value

        self.subscription = self.create_subscription(
            PointCloud2, "/velodyne_points", self.pointcloud_callback, 10)
        self.filtered_pub = self.create_publisher(PointCloud2, "/filtered_points", 10)
        self.get_logger().info(f"Voxel Grid Filter started (voxel_size={self.voxel_size})")

    def pointcloud_callback(self, msg: PointCloud2):
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)
        xyz = points[:, :3]
        filtered = self.voxel_filter(xyz)
        filtered_msg = self.make_pointcloud2(filtered, msg.header)
        self.filtered_pub.publish(filtered_msg)
        self.get_logger().info(f"Filtered: {len(xyz)} -> {len(filtered)} points")

    def voxel_filter(self, xyz):
        """Downsample using voxel grid. Returns (M,3) centroids."""
        if len(xyz) == 0:
            return xyz
        voxel_idx = np.floor(xyz / self.voxel_size).astype(np.int32)
        # Create unique keys by encoding 3D index as a single value
        _, inverse, counts = np.unique(
            voxel_idx, axis=0, return_inverse=True, return_counts=True)
        # Sum all points per voxel, then divide by count
        centroids = np.zeros((len(counts), 3), dtype=np.float64)
        np.add.at(centroids, inverse, xyz)
        centroids /= counts[:, np.newaxis]
        return centroids.astype(np.float32)

    def make_pointcloud2(self, xyz, header):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(xyz)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(xyz)
        msg.data = xyz.astype(np.float32).tobytes()
        msg.is_dense = True
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = VoxelGridFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
