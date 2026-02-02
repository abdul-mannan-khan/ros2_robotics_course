#!/usr/bin/env python3
"""Week 8 Solution 2: RRT* 3D Planner (Complete)"""
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
        self.get_logger().info('RRT* 3D Planner ready')

    def grid_cb(self, msg):
        if self.planned or msg.width == 0:
            return
        self.obstacles = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :3]
        self.plan()

    def collision_free(self, p1, p2, radius=0.3):
        if self.obstacles is None or len(self.obstacles) == 0:
            return True
        n = max(2, int(np.linalg.norm(p2 - p1) / 0.1))
        for i in range(n):
            pt = p1 + (i / (n - 1)) * (p2 - p1)
            dists = np.linalg.norm(self.obstacles - pt, axis=1)
            if np.min(dists) < radius:
                return False
        return True

    def plan(self):
        positions = [self.start.copy()]
        parents = [-1]
        costs = [0.0]
        rewire_r = self.step * 3.0
        goal_idx = -1
        for it in range(self.max_iter):
            if random.random() < 0.05:
                sample = self.goal + np.random.randn(3) * 0.5
            else:
                sample = self.start + np.random.rand(3) * (self.goal - self.start) * 1.5
                sample += np.random.randn(3) * 2.0
            dists = [np.linalg.norm(p - sample) for p in positions]
            near_idx = int(np.argmin(dists))
            nearest = positions[near_idx]
            direction = sample - nearest
            dist = np.linalg.norm(direction)
            if dist < 1e-6:
                continue
            new_pt = nearest + (direction / dist) * min(self.step, dist)
            if not self.collision_free(nearest, new_pt):
                continue
            # Find nearby nodes for rewiring
            nearby = [j for j, p in enumerate(positions) if np.linalg.norm(p - new_pt) < rewire_r]
            # Choose best parent
            best_parent = near_idx
            best_cost = costs[near_idx] + np.linalg.norm(new_pt - nearest)
            for j in nearby:
                c = costs[j] + np.linalg.norm(new_pt - positions[j])
                if c < best_cost and self.collision_free(positions[j], new_pt):
                    best_cost = c
                    best_parent = j
            new_idx = len(positions)
            positions.append(new_pt)
            parents.append(best_parent)
            costs.append(best_cost)
            # Rewire
            for j in nearby:
                c = best_cost + np.linalg.norm(positions[j] - new_pt)
                if c < costs[j] and self.collision_free(new_pt, positions[j]):
                    parents[j] = new_idx
                    costs[j] = c
            if np.linalg.norm(new_pt - self.goal) < self.goal_thresh:
                goal_idx = new_idx
                self.get_logger().info(f'Goal reached at iteration {it}!')
                break
        # Extract and publish path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        if goal_idx >= 0:
            idx = goal_idx
            waypoints = []
            while idx >= 0:
                waypoints.append(positions[idx])
                idx = parents[idx]
            waypoints.reverse()
            for wp in waypoints:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(wp[0]), float(wp[1]), float(wp[2])
                path_msg.poses.append(ps)
            self.get_logger().info(f'Path: {len(waypoints)} waypoints')
        else:
            self.get_logger().warn(f'No path found after {self.max_iter} iterations')
        self.pub_path.publish(path_msg)
        # Publish tree visualization
        marker = Marker()
        marker.header = path_msg.header
        marker.ns, marker.id = 'rrt_tree', 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color = ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.5)
        for i in range(1, len(positions)):
            p1, p2 = positions[i], positions[parents[i]]
            marker.points.append(Point(x=float(p1[0]),y=float(p1[1]),z=float(p1[2])))
            marker.points.append(Point(x=float(p2[0]),y=float(p2[1]),z=float(p2[2])))
        self.pub_tree.publish(marker)
        self.planned = True

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RRTStar3DNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
