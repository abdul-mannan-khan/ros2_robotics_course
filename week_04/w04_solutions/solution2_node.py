#!/usr/bin/env python3
"""Solution 2: Occupancy Grid Builder - Full implementation."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

class OccupancyGridSolution(Node):
    def __init__(self):
        super().__init__("occupancy_grid_solution")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("map_size", 100)
        self.declare_parameter("hit_prob", 0.7)
        self.declare_parameter("miss_prob", 0.4)
        self.res = self.get_parameter("resolution").value
        self.map_sz = self.get_parameter("map_size").value
        self.gdim = int(self.map_sz / self.res)
        self.log_odds = np.zeros((self.gdim, self.gdim), dtype=np.float64)
        hp = self.get_parameter("hit_prob").value
        mp = self.get_parameter("miss_prob").value
        self.l_hit = math.log(hp / (1 - hp))
        self.l_miss = math.log(mp / (1 - mp))
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.rx, self.ry, self.rth = 0.0, 0.0, 0.0
        self.get_logger().info("OccupancyGridSolution started")

    def odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.rth = 2.0 * math.atan2(q.z, q.w)

    def w2g(self, wx, wy):
        o = -self.map_sz / 2.0
        return int((wx - o) / self.res), int((wy - o) / self.res)

    def bresenham(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0); dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1: break
            e2 = 2 * err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 < dx: err += dx; y0 += sy
        return cells

    def update_grid(self, scan):
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        ranges = np.array(scan.ranges)
        for i, r in enumerate(ranges):
            if r < scan.range_min or r > scan.range_max: continue
            a = self.rth + angles[i] if i < len(angles) else 0
            ex = self.rx + r * math.cos(a)
            ey = self.ry + r * math.sin(a)
            gx0, gy0 = self.w2g(self.rx, self.ry)
            gx1, gy1 = self.w2g(ex, ey)
            cells = self.bresenham(gx0, gy0, gx1, gy1)
            for cx, cy in cells[:-1]:
                if 0 <= cx < self.gdim and 0 <= cy < self.gdim:
                    self.log_odds[cy, cx] -= self.l_miss
            if cells:
                cx, cy = cells[-1]
                if 0 <= cx < self.gdim and 0 <= cy < self.gdim:
                    self.log_odds[cy, cx] += self.l_hit
        self.log_odds = np.clip(self.log_odds, -10, 10)

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
    node = OccupancyGridSolution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
