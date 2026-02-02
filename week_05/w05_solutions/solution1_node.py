#!/usr/bin/env python3
"""Solution 1: Costmap Generator - Full inflation implementation."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
from scipy.ndimage import distance_transform_edt

class CostmapSolution(Node):
    def __init__(self):
        super().__init__("costmap_solution")
        self.declare_parameter("inflation_radius", 0.3)
        self.declare_parameter("cost_scaling_factor", 5.0)
        self.infl_r = self.get_parameter("inflation_radius").value
        self.cost_scale = self.get_parameter("cost_scaling_factor").value
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_cb, 10)
        self.cost_pub = self.create_publisher(OccupancyGrid, "/costmap", 10)
        self.get_logger().info("CostmapSolution started")

    def inflate(self, grid, w, h, res):
        g = np.array(grid, dtype=np.float64).reshape((h, w))
        occ = (g >= 50).astype(np.float64)
        free = 1.0 - occ
        dist = distance_transform_edt(free) * res
        infl_cells = dist < self.infl_r
        cost = np.where(occ > 0, 100.0, np.where(infl_cells, 100.0 * np.exp(-self.cost_scale * dist), 0.0))
        return np.clip(cost, 0, 100).flatten().astype(np.int8).tolist()

    def map_cb(self, msg):
        w, h = msg.info.width, msg.info.height
        cm = OccupancyGrid()
        cm.header = msg.header; cm.info = msg.info
        cm.data = self.inflate(msg.data, w, h, msg.info.resolution)
        self.cost_pub.publish(cm)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapSolution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
