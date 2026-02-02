#!/usr/bin/env python3
"""
Week 8 Exercise 2: RRT* 3D Planner Node

Subscribers: /occupancy_grid_3d (PointCloud2)
Publishers: /rrt_tree (Marker), /planned_path_3d (Path)
Parameters: start_xyz, goal_xyz, step_size (0.5), max_iterations (5000), goal_threshold (1.0)
"""
import math, random
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class RRTStar3DNode(Node):
    def __init__(self):
        super().__init__('rrt_star_3d_planner')
        self.declare_parameter('start_xyz', [0.0, 0.0, 1.0])
        self.declare_parameter('goal_xyz', [15.0, 0.0, 1.0])
        self.declare_parameter('step_size', 0.5)
        self.declare_parameter('max_iterations', 5000)
        self.declare_parameter('goal_threshold', 1.0)
        self.start = np.array(self.get_parameter('start_xyz').value)
        self.goal = np.array(self.get_parameter('goal_xyz').value)
        self.step = self.get_parameter('step_size').value
        self.max_iter = self.get_parameter('max_iterations').value
        self.goal_thresh = self.get_parameter('goal_threshold').value
        self.obstacles = None
        self.planned = False
        self.create_subscription(PointCloud2, '/occupancy_grid_3d', self.grid_cb, 10)
        self.pub_tree = self.create_publisher(Marker, '/rrt_tree', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path_3d', 10)
        self.get_logger().info('RRT* 3D Planner waiting for occupancy grid...')

    def grid_cb(self, msg):
        if self.planned or msg.width == 0:
            return
        self.obstacles = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :3]
        self.get_logger().info(f'Got {len(self.obstacles)} obstacle points, planning...')
        self.plan()

    def collision_free(self, p1, p2):
        if self.obstacles is None or len(self.obstacles) == 0:
            return True
        # TODO: Check line segment p1->p2 for collisions
        # n = max(2, int(np.linalg.norm(p2-p1)/0.1))
        # for i in range(n):
        #     pt = p1 + (i/(n-1))*(p2-p1)
        #     if np.min(np.linalg.norm(self.obstacles-pt, axis=1)) < 0.3:
        #         return False
        return True

    def plan(self):
        # TODO: Implement RRT* core algorithm
        # tree = [(self.start, -1, 0.0)]  # (position, parent_idx, cost)
        # for it in range(self.max_iter):
        #   1. Sample random point (5% bias toward goal)
        #   2. Find nearest node
        #   3. Steer: new_pt = nearest + step*(sample-nearest)/||sample-nearest||
        #   4. If collision_free(nearest, new_pt):
        #   5.   Find nearby nodes within rewire_radius = min(step*3, ...)
        #   6.   Choose best parent from nearby (min cost + dist, collision free)
        #   7.   Add node to tree
        #   8.   Rewire nearby nodes through new node if cheaper
        #   9.   If ||new_pt - goal|| < goal_thresh: extract path, break

        self.get_logger().warn('RRT* not implemented - TODO')
        self.planned = True
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        self.pub_path.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RRTStar3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
