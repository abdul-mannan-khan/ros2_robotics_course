#!/usr/bin/env python3
"""Exercise 3: Loop Closure Detector Node.
Subscribe to /scan and /map, detect revisited areas via scan-to-map correlation,
publish /loop_closure (PoseWithCovarianceStamped)."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

class LoopClosureDetectorNode(Node):
    def __init__(self):
        super().__init__("loop_closure_detector_node")
        self.declare_parameter("correlation_threshold", 0.85)
        self.declare_parameter("min_travel_distance", 5.0)
        self.corr_thresh = self.get_parameter("correlation_threshold").value
        self.min_travel = self.get_parameter("min_travel_distance").value
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_cb, 10)
        self.lc_pub = self.create_publisher(PoseWithCovarianceStamped, "/loop_closure", 10)
        self.current_map = None
        self.scan_history = []
        self.pose_history = []
        self.total_dist = 0.0
        self.last_x, self.last_y = 0.0, 0.0
        self.get_logger().info("LoopClosureDetectorNode started")

    def map_cb(self, msg):
        self.current_map = msg

    def scan_to_local_grid(self, scan, grid_size=60, res=0.05):
        """TODO: Convert scan to a small local occupancy grid.
        Create a grid_size x grid_size grid centered on robot.
        Mark cells hit by scan rays.
        Return 2D numpy array."""
        grid = np.zeros((grid_size, grid_size))
        return grid

    def correlate_with_map(self, local_grid, map_grid, candidates):
        """TODO: Implement scan-to-map correlation.
        For each candidate pose in candidates:
          1. Extract map patch at that pose
          2. Compute normalized cross-correlation with local_grid
          3. If correlation > threshold, report loop closure
        Return best (x, y, score) or None."""
        return None

    def scan_cb(self, msg):
        if self.current_map is None: return
        if self.total_dist < self.min_travel: return
        local = self.scan_to_local_grid(msg)
        # TODO: Generate candidate poses from pose_history
        # TODO: Call correlate_with_map
        # TODO: If match found, publish loop closure
        pass

def main(args=None):
    rclpy.init(args=args)
    node = LoopClosureDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
