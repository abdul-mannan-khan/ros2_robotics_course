#!/usr/bin/env python3
"""Solution 3: Loop Closure Detector - Full implementation."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

class LoopClosureSolution(Node):
    def __init__(self):
        super().__init__("loop_closure_solution")
        self.declare_parameter("correlation_threshold", 0.85)
        self.declare_parameter("min_travel_distance", 5.0)
        self.corr_thresh = self.get_parameter("correlation_threshold").value
        self.min_travel = self.get_parameter("min_travel_distance").value
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_cb, 10)
        self.lc_pub = self.create_publisher(PoseWithCovarianceStamped, "/loop_closure", 10)
        self.map_data = None
        self.map_info = None
        self.scan_history = []
        self.total_dist = 0.0
        self.last_x, self.last_y = 0.0, 0.0
        self.get_logger().info("LoopClosureSolution started")

    def map_cb(self, msg):
        self.map_info = msg.info
        w, h = msg.info.width, msg.info.height
        self.map_data = np.array(msg.data, dtype=np.float64).reshape((h, w))

    def scan_to_local_grid(self, scan, size=60, res=0.05):
        grid = np.zeros((size, size))
        c = size // 2
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max: continue
            if i >= len(angles): break
            ex = r * math.cos(angles[i])
            ey = r * math.sin(angles[i])
            gx = int(ex / res) + c
            gy = int(ey / res) + c
            if 0 <= gx < size and 0 <= gy < size:
                grid[gy, gx] = 100.0
        return grid

    def extract_map_patch(self, cx, cy, size=60):
        if self.map_data is None: return None
        h, w = self.map_data.shape
        hs = size // 2
        x0, y0 = max(0, cx-hs), max(0, cy-hs)
        x1, y1 = min(w, cx+hs), min(h, cy+hs)
        patch = np.zeros((size, size))
        px0, py0 = hs-(cx-x0), hs-(cy-y0)
        patch[py0:py0+(y1-y0), px0:px0+(x1-x0)] = self.map_data[y0:y1, x0:x1]
        return patch

    def ncc(self, a, b):
        a = a - a.mean(); b = b - b.mean()
        na = np.linalg.norm(a); nb = np.linalg.norm(b)
        if na < 1e-10 or nb < 1e-10: return 0.0
        return float(np.sum(a * b) / (na * nb))

    def scan_cb(self, msg):
        if self.map_data is None or self.map_info is None: return
        if self.total_dist < self.min_travel: return
        local = self.scan_to_local_grid(msg)
        best_score, best_x, best_y = 0.0, 0.0, 0.0
        mi = self.map_info
        for sx, sy in self.scan_history[:-20]:
            gx = int((sx - mi.origin.position.x) / mi.resolution)
            gy = int((sy - mi.origin.position.y) / mi.resolution)
            patch = self.extract_map_patch(gx, gy)
            if patch is None: continue
            score = self.ncc(local, patch)
            if score > best_score:
                best_score, best_x, best_y = score, sx, sy
        if best_score > self.corr_thresh:
            lc = PoseWithCovarianceStamped()
            lc.header.stamp = self.get_clock().now().to_msg()
            lc.header.frame_id = "map"
            lc.pose.pose.position.x = best_x
            lc.pose.pose.position.y = best_y
            lc.pose.covariance[0] = 0.1
            lc.pose.covariance[7] = 0.1
            lc.pose.covariance[35] = 0.05
            self.lc_pub.publish(lc)
            self.get_logger().info(f"Loop closure: ({best_x:.1f},{best_y:.1f}) score={best_score:.2f}")
            self.total_dist = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = LoopClosureSolution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
