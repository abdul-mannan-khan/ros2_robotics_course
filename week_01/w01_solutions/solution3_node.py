#!/usr/bin/env python3
"""Week 1 - Solution 3: Ground Removal + Clustering (Complete Implementation)"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int32


class GroundRemovalClusteringNode(Node):
    def __init__(self):
        super().__init__("ground_removal_clustering_node")
        self.declare_parameter("distance_threshold", 0.2)
        self.declare_parameter("max_iterations", 100)
        self.declare_parameter("cluster_tolerance", 0.5)
        self.declare_parameter("min_cluster_size", 10)
        self.declare_parameter("max_cluster_size", 5000)
        self.distance_threshold = self.get_parameter("distance_threshold").value
        self.max_iterations = self.get_parameter("max_iterations").value
        self.cluster_tolerance = self.get_parameter("cluster_tolerance").value
        self.min_cluster_size = self.get_parameter("min_cluster_size").value
        self.max_cluster_size = self.get_parameter("max_cluster_size").value

        self.subscription = self.create_subscription(
            PointCloud2, "/velodyne_points", self.pointcloud_callback, 10)
        self.ground_pub = self.create_publisher(PointCloud2, "/ground_plane", 10)
        self.obstacles_pub = self.create_publisher(PointCloud2, "/obstacles", 10)
        self.cluster_count_pub = self.create_publisher(Int32, "/cluster_count", 10)
        self.get_logger().info("Ground Removal + Clustering started")

    def pointcloud_callback(self, msg: PointCloud2):
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)
        xyz = points[:, :3]
        ground_mask = self.ransac_ground_removal(xyz)
        ground_pts = xyz[ground_mask]
        obstacle_pts = xyz[~ground_mask]
        labels = self.euclidean_clustering(obstacle_pts)
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        self.ground_pub.publish(self.make_pc2(ground_pts, msg.header))
        self.obstacles_pub.publish(self.make_pc2(obstacle_pts, msg.header))
        cm = Int32(); cm.data = n_clusters
        self.cluster_count_pub.publish(cm)
        self.get_logger().info(f"Ground: {len(ground_pts)}, Obstacles: {len(obstacle_pts)}, Clusters: {n_clusters}")

    def ransac_ground_removal(self, xyz):
        best_mask = np.zeros(len(xyz), dtype=bool)
        best_count = 0
        for _ in range(self.max_iterations):
            idx = np.random.choice(len(xyz), 3, replace=False)
            p1, p2, p3 = xyz[idx]
            normal = np.cross(p2 - p1, p3 - p1)
            norm_len = np.linalg.norm(normal)
            if norm_len < 1e-10:
                continue
            normal /= norm_len
            d = -np.dot(normal, p1)
            distances = np.abs(xyz @ normal + d)
            mask = distances < self.distance_threshold
            count = mask.sum()
            if count > best_count:
                best_count = count
                best_mask = mask
        return best_mask

    def euclidean_clustering(self, xyz):
        if len(xyz) == 0:
            return np.array([], dtype=int)
        try:
            from scipy.spatial import KDTree
        except ImportError:
            self.get_logger().warn("scipy not available, skipping clustering")
            return np.full(len(xyz), -1)
        tree = KDTree(xyz)
        labels = np.full(len(xyz), -1)
        cluster_id = 0
        for i in range(len(xyz)):
            if labels[i] != -1:
                continue
            seed_neighbors = tree.query_ball_point(xyz[i], self.cluster_tolerance)
            if len(seed_neighbors) < self.min_cluster_size:
                continue
            queue = list(seed_neighbors)
            cluster_pts = set(seed_neighbors)
            while queue:
                pt = queue.pop(0)
                if labels[pt] != -1:
                    continue
                nb = tree.query_ball_point(xyz[pt], self.cluster_tolerance)
                if len(nb) >= self.min_cluster_size:
                    for n in nb:
                        if n not in cluster_pts:
                            cluster_pts.add(n)
                            queue.append(n)
            if self.min_cluster_size <= len(cluster_pts) <= self.max_cluster_size:
                for pt in cluster_pts:
                    labels[pt] = cluster_id
                cluster_id += 1
        return labels

    def make_pc2(self, xyz, header):
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
    node = GroundRemovalClusteringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
