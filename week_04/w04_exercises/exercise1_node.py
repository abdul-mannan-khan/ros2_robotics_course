#!/usr/bin/env python3
"""Exercise 1: Scan Matcher Node - ICP-based scan-to-scan matching.
Subscribe to /scan, implement ICP between consecutive scans,
publish /scan_match_pose (PoseStamped)."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

class ScanMatcherNode(Node):
    def __init__(self):
        super().__init__("scan_matcher_node")
        self.declare_parameter("max_iterations", 50)
        self.declare_parameter("convergence_threshold", 0.001)
        self.max_iter = self.get_parameter("max_iterations").value
        self.conv_thresh = self.get_parameter("convergence_threshold").value
        self.sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.pub = self.create_publisher(PoseStamped, "/scan_match_pose", 10)
        self.prev_points = None
        self.cum_x, self.cum_y, self.cum_th = 0.0, 0.0, 0.0
        self.get_logger().info("ScanMatcherNode started")

    def scan_to_points(self, scan):
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        ranges = np.array(scan.ranges)
        valid = (ranges > scan.range_min) & (ranges < scan.range_max)
        r, a = ranges[valid], angles[:len(ranges)][valid]
        return np.column_stack([r * np.cos(a), r * np.sin(a)])

    def find_correspondences(self, src, tgt):
        """TODO: Find nearest neighbor correspondences from src to tgt.
        For each point in src, find closest point in tgt.
        Return array of indices into tgt.
        Hint: Use numpy broadcasting or scipy.spatial.KDTree."""
        pass

    def compute_transform(self, src, tgt, idx):
        """TODO: Compute optimal R, t using SVD.
        1. Compute centroids of matched sets
        2. Center points
        3. H = src_centered.T @ tgt_centered
        4. SVD(H) -> R, t = tgt_centroid - R @ src_centroid
        Return R (2x2), t (2,)."""
        pass

    def icp(self, src, tgt):
        """TODO: ICP core loop.
        1. Find correspondences (nearest neighbors)
        2. Compute optimal transform (SVD)
        3. Apply transform to src points
        4. Check convergence
        5. Repeat until converged or max_iterations
        Return: dx, dy, dtheta"""
        dx, dy, dtheta = 0.0, 0.0, 0.0
        return dx, dy, dtheta

    def scan_cb(self, msg):
        pts = self.scan_to_points(msg)
        if len(pts) < 10: return
        if self.prev_points is not None:
            dx, dy, dth = self.icp(pts, self.prev_points)
            self.cum_x += dx*math.cos(self.cum_th) - dy*math.sin(self.cum_th)
            self.cum_y += dx*math.sin(self.cum_th) + dy*math.cos(self.cum_th)
            self.cum_th += dth
            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = "odom"
            pose.pose.position.x = self.cum_x
            pose.pose.position.y = self.cum_y
            pose.pose.orientation.z = math.sin(self.cum_th/2)
            pose.pose.orientation.w = math.cos(self.cum_th/2)
            self.pub.publish(pose)
        self.prev_points = pts

def main(args=None):
    rclpy.init(args=args)
    node = ScanMatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
