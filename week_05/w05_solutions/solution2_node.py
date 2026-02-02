#!/usr/bin/env python3
"""Solution 2: A* Path Planner - Full implementation."""
import rclpy
from rclpy.node import Node
import numpy as np
import math, heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan

class AStarSolution(Node):
    def __init__(self):
        super().__init__("astar_solution")
        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("heuristic_weight", 1.0)
        self.diag = self.get_parameter("allow_diagonal").value
        self.h_w = self.get_parameter("heuristic_weight").value
        self.map_sub = self.create_subscription(OccupancyGrid, "/costmap", self.map_cb, 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.srv = self.create_service(GetPlan, "/plan_path", self.plan_cb)
        self.grid = None; self.info = None
        self.get_logger().info("AStarSolution started")

    def map_cb(self, msg):
        self.info = msg.info
        self.grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def w2g(self, wx, wy):
        return int((wx-self.info.origin.position.x)/self.info.resolution), int((wy-self.info.origin.position.y)/self.info.resolution)

    def g2w(self, gx, gy):
        return gx*self.info.resolution+self.info.origin.position.x, gy*self.info.resolution+self.info.origin.position.y

    def astar(self, sx, sy, gx, gy):
        h, w = self.grid.shape
        if self.diag:
            nbrs = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
        else:
            nbrs = [(-1,0),(1,0),(0,-1),(0,1)]
        open_set = [(0, sx, sy)]
        g_score = {(sx,sy): 0}
        came_from = {}
        closed = set()
        while open_set:
            f, cx, cy = heapq.heappop(open_set)
            if (cx, cy) in closed: continue
            closed.add((cx, cy))
            if cx == gx and cy == gy:
                path = []
                c = (gx, gy)
                while c in came_from:
                    path.append(c)
                    c = came_from[c]
                path.append((sx, sy))
                return list(reversed(path))
            for dx, dy in nbrs:
                nx, ny = cx+dx, cy+dy
                if nx<0 or ny<0 or nx>=w or ny>=h: continue
                if self.grid[ny, nx] > 90: continue
                if (nx, ny) in closed: continue
                cost = math.sqrt(dx*dx+dy*dy) + self.grid[ny,nx]/100.0
                ng = g_score[(cx,cy)] + cost
                if ng < g_score.get((nx,ny), float("inf")):
                    g_score[(nx,ny)] = ng
                    heur = self.h_w * math.sqrt((gx-nx)**2+(gy-ny)**2)
                    heapq.heappush(open_set, (ng+heur, nx, ny))
                    came_from[(nx,ny)] = (cx, cy)
        return []

    def plan_cb(self, req, resp):
        if self.grid is None: return resp
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
            ps.pose.position.x = wx; ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        resp.plan = path
        self.path_pub.publish(path)
        return resp

def main(args=None):
    rclpy.init(args=args)
    node = AStarSolution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
