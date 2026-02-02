#!/usr/bin/env python3
"""Exercise 1: Costmap Generator Node.
Subscribe to /map, inflate obstacles by robot radius, publish /costmap."""
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid

class CostmapGeneratorNode(Node):
    def __init__(self):
        super().__init__("costmap_generator_node")
        self.declare_parameter("inflation_radius", 0.3)
        self.declare_parameter("cost_scaling_factor", 5.0)
        self.infl_r = self.get_parameter("inflation_radius").value
        self.cost_scale = self.get_parameter("cost_scaling_factor").value
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_cb, 10)
        self.cost_pub = self.create_publisher(OccupancyGrid, "/costmap", 10)
        self.get_logger().info("CostmapGeneratorNode started")

    def inflate(self, grid, w, h, res):
        """TODO: Inflate obstacles in the occupancy grid.
        1. Find all occupied cells (value >= 50)
        2. For each occupied cell, mark cells within inflation_radius as costly
        3. Cost decreases exponentially with distance: cost = 100 * exp(-cost_scaling_factor * dist)
        4. Keep maximum cost at each cell
        Return inflated grid as 1D list."""
        inflated = np.array(grid, dtype=np.float64).reshape((h, w))
        # TODO: implement inflation
        return inflated.flatten().astype(np.int8).tolist()

    def map_cb(self, msg):
        w, h = msg.info.width, msg.info.height
        inflated = self.inflate(msg.data, w, h, msg.info.resolution)
        cm = OccupancyGrid()
        cm.header = msg.header
        cm.info = msg.info
        cm.data = inflated
        self.cost_pub.publish(cm)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
