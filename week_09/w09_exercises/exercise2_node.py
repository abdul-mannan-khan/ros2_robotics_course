#!/usr/bin/env python3
"""
Week 9 Exercise 2: Local Planner Node

Build local ESDF-like distance field, plan minimum-jerk trajectory.

Subscribers: /odom (Odometry), /local_pointcloud (PointCloud2)
Publishers: /local_trajectory (Path), /distance_field (PointCloud2)
Parameters: goal_xyz, planning_horizon (5.0), safety_margin (0.5)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')
        self.declare_parameter('goal_xyz', [20.0, 0.0, 1.5])
        self.declare_parameter('planning_horizon', 5.0)
        self.declare_parameter('safety_margin', 0.5)
        self.goal = np.array(self.get_parameter('goal_xyz').value)
        self.horizon = self.get_parameter('planning_horizon').value
        self.margin = self.get_parameter('safety_margin').value
        self.pose = None
        self.obstacles = None
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PointCloud2, '/local_pointcloud', self.cloud_cb, 10)
        self.pub_traj = self.create_publisher(Path, '/local_trajectory', 10)
        self.pub_df = self.create_publisher(PointCloud2, '/distance_field', 10)
        self.create_timer(0.1, self.plan)
        self.get_logger().info('Local Planner started')

    def odom_cb(self, msg):
        self.pose = msg.pose.pose

    def cloud_cb(self, msg):
        if msg.width > 0:
            self.obstacles = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :3]

    def plan(self):
        if self.pose is None:
            return
        # TODO: Distance field computation and trajectory optimization
        # 1. Build local 3D grid around current position
        # 2. For each grid cell, compute distance to nearest obstacle
        # 3. Generate minimum-jerk trajectory from current pos to goal
        #    (or intermediate point at planning_horizon distance)
        # 4. Check trajectory against distance field, push away from obstacles
        # 5. Publish trajectory and distance field visualization

        pass  # TODO: implement


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LocalPlannerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
