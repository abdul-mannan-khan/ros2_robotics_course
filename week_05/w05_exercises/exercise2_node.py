#!/usr/bin/env python3
"""Exercise 2: A* Path Planner Node.
Subscribe to /costmap, service /plan_path, publish /planned_path."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__("astar_planner_node")
        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("heuristic_weight", 1.0)
        self.diag = self.get_parameter("allow_diagonal").value
        self.h_weight = self.get_parameter("heuristic_weight").value
        self.map_sub = self.create_subscription(OccupancyGrid, "/costmap", self.map_cb, 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.srv = self.create_service(GetPlan, "/plan_path", self.plan_cb)
        self.grid = None
        self.info = None
        self.get_logger().info("AStarPlannerNode started")

    def map_cb(self, msg):
        self.info = msg.info
        self.grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def w2g(self, wx, wy):
        gx = int((wx - self.info.origin.position.x) / self.info.resolution)
        gy = int((wy - self.info.origin.position.y) / self.info.resolution)
        return gx, gy

    def g2w(self, gx, gy):
        wx = gx * self.info.resolution + self.info.origin.position.x
        wy = gy * self.info.resolution + self.info.origin.position.y
        return wx, wy

    def astar(self, sx, sy, gx, gy):
        """TODO: Implement A* search on the costmap grid.
        1. Use priority queue (heapq)
        2. Heuristic: Euclidean distance * heuristic_weight
        3. Neighbors: 4-connected or 8-connected based on allow_diagonal
        4. Skip cells with cost > 90 (obstacles)
        5. Return list of (gx, gy) from start to goal, or empty list"""
        path = []
        return path

    def plan_cb(self, req, resp):
        if self.grid is None:
            return resp
        sx, sy = self.w2g(req.start.pose.position.x, req.start.pose.position.y)
        gx, gy = self.w2g(req.goal.pose.position.x, req.goal.pose.position.y)
        cells = self.astar(sx, sy, gx, gy)
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for cx, cy in cells:
            ps = PoseStamped()
            ps.header = path.header
            wx, wy = self.g2w(cx, cy)
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        resp.plan = path
        self.path_pub.publish(path)
        return resp

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
