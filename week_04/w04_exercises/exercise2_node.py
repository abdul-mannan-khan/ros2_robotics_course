#!/usr/bin/env python3
"""Exercise 2: Occupancy Grid Builder Node.
Subscribe to /scan and /odom, build 2D occupancy grid, publish /map."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

class OccupancyGridBuilderNode(Node):
    def __init__(self):
        super().__init__("occupancy_grid_builder_node")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("map_size", 100)
        self.declare_parameter("hit_prob", 0.7)
        self.declare_parameter("miss_prob", 0.4)
        self.res = self.get_parameter("resolution").value
        self.map_sz = self.get_parameter("map_size").value
        self.hit_p = self.get_parameter("hit_prob").value
        self.miss_p = self.get_parameter("miss_prob").value
        self.gdim = int(self.map_sz / self.res)
        self.log_odds = np.zeros((self.gdim, self.gdim), dtype=np.float64)
        self.l_hit = math.log(self.hit_p / (1 - self.hit_p))
        self.l_miss = math.log(self.miss_p / (1 - self.miss_p))
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.rx, self.ry, self.rth = 0.0, 0.0, 0.0
        self.get_logger().info("OccupancyGridBuilderNode started")

    def odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.rth = 2.0 * math.atan2(q.z, q.w)

    def w2g(self, wx, wy):
        """World to grid coordinates."""
        o = -self.map_sz / 2.0
        return int((wx - o) / self.res), int((wy - o) / self.res)

    def bresenham(self, x0, y0, x1, y1):
        """TODO: Implement Bresenham line algorithm.
        Return list of (gx, gy) grid cells from (x0,y0) to (x1,y1).
        Used for ray casting through the occupancy grid."""
        cells = []
        return cells

    def update_grid(self, scan):
        """TODO: Ray casting and grid update.
        For each ray in scan:
          1. Compute endpoint in world frame
          2. bresenham() for cells along ray
          3. Free cells: subtract l_miss
          4. Endpoint cell: add l_hit
        Clamp log-odds to [-10, 10]."""
        if scan is None: return
        pass

    def scan_cb(self, msg):
        self.update_grid(msg)

    def publish_map(self):
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "map"
        g.info.resolution = self.res
        g.info.width = self.gdim
        g.info.height = self.gdim
        g.info.origin.position.x = -self.map_sz / 2.0
        g.info.origin.position.y = -self.map_sz / 2.0
        prob = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        g.data = (prob * 100).astype(np.int8).flatten().tolist()
        self.map_pub.publish(g)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridBuilderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
