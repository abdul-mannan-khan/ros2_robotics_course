#!/usr/bin/env python3
"""
Week 1 - Exercise 2: Voxel Grid Filter Node
Subscribe to /velodyne_points, apply voxel grid downsampling,
publish filtered cloud on /filtered_points.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class VoxelGridFilterNode(Node):
    def __init__(self):
        super().__init__("voxel_grid_filter_node")

        # TODO 1: Declare parameter
        # self.declare_parameter("voxel_size", 0.3)
        # self.voxel_size = self.get_parameter("voxel_size").value
        self.voxel_size = 0.3

        # TODO 2: Create subscriber to /velodyne_points
        # self.subscription = self.create_subscription(
        #     PointCloud2, "/velodyne_points", self.pointcloud_callback, 10)

        # TODO 3: Create publisher for /filtered_points
        # self.filtered_pub = self.create_publisher(PointCloud2, "/filtered_points", 10)

        self.get_logger().info(f"Voxel Grid Filter started (voxel_size={self.voxel_size})")

    def pointcloud_callback(self, msg: PointCloud2):
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        xyz = points[:, :3]

        # TODO 4: Implement voxel grid downsampling
        # Steps:
        #   a) voxel_indices = np.floor(xyz / self.voxel_size).astype(int)
        #   b) Find unique voxels using np.unique on structured array
        #   c) For each unique voxel, compute centroid
        # filtered_xyz = self.voxel_filter(xyz)
        filtered_xyz = xyz  # Replace with actual filter

        # TODO 5: Build and publish filtered PointCloud2
        # filtered_msg = self.make_pointcloud2(filtered_xyz, msg.header)
        # self.filtered_pub.publish(filtered_msg)

        self.get_logger().info(f"Filtered: {len(xyz)} -> {len(filtered_xyz)} pts")

    def voxel_filter(self, xyz):
        """Downsample point cloud using voxel grid."""
        # TODO 6: Implement voxel grid filter
        # voxel_idx = np.floor(xyz / self.voxel_size).astype(np.int32)
        # _, inverse, counts = np.unique(
        #     voxel_idx, axis=0, return_inverse=True, return_counts=True)
        # centroids = np.zeros((len(counts), 3), dtype=np.float32)
        # np.add.at(centroids, inverse, xyz)
        # centroids /= counts[:, np.newaxis]
        # return centroids
        return xyz

    def make_pointcloud2(self, xyz, header):
        """Create PointCloud2 from (N,3) xyz array."""
        # TODO 7: Build PointCloud2 message
        # msg = PointCloud2()
        # msg.header = header
        # msg.height = 1
        # msg.width = len(xyz)
        # msg.fields = [
        #     PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        # ]
        # msg.is_bigendian = False
        # msg.point_step = 12
        # msg.row_step = 12 * len(xyz)
        # msg.data = xyz.astype(np.float32).tobytes()
        # msg.is_dense = True
        # return msg
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VoxelGridFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
