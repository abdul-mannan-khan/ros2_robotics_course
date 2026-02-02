#!/usr/bin/env python3
"""Solution 1: Scan Matcher Node - Full ICP implementation."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from scipy.spatial import KDTree
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

class ScanMatcherSolution(Node):
    def __init__(self):
        super().__init__("scan_matcher_solution")
        self.declare_parameter("max_iterations", 50)
        self.declare_parameter("convergence_threshold", 0.001)
        self.max_iter = self.get_parameter("max_iterations").value
        self.conv_thresh = self.get_parameter("convergence_threshold").value
        self.sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.pub = self.create_publisher(PoseStamped, "/scan_match_pose", 10)
        self.prev_points = None
        self.cum_x, self.cum_y, self.cum_th = 0.0, 0.0, 0.0
        self.get_logger().info("ScanMatcherSolution started")

    def scan_to_points(self, scan):
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        ranges = np.array(scan.ranges)
        valid = (ranges > scan.range_min) & (ranges < scan.range_max)
        r, a = ranges[valid], angles[:len(ranges)][valid]
        return np.column_stack([r * np.cos(a), r * np.sin(a)])

    def icp(self, src, tgt):
        """Full ICP implementation."""
        tree = KDTree(tgt)
        current = src.copy()
        total_R = np.eye(2)
        total_t = np.zeros(2)
        prev_err = float("inf")
        for _ in range(self.max_iter):
            dists, idx = tree.query(current)
            matched_tgt = tgt[idx]
            src_c = current.mean(axis=0)
            tgt_c = matched_tgt.mean(axis=0)
            src_centered = current - src_c
            tgt_centered = matched_tgt - tgt_c
            H = src_centered.T @ tgt_centered
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            t = tgt_c - R @ src_c
            current = (R @ current.T).T + t
            total_R = R @ total_R
            total_t = R @ total_t + t
            err = np.mean(dists)
            if abs(prev_err - err) < self.conv_thresh: break
            prev_err = err
        angle = math.atan2(total_R[1,0], total_R[0,0])
        return total_t[0], total_t[1], angle

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
    node = ScanMatcherSolution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
