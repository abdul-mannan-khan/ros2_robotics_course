#!/usr/bin/env python3
"""
Week 9 Exercise 3: Trajectory Tracker Node

Track B-spline trajectory with feedforward + feedback controller.

Subscribers: /local_trajectory (Path), /odom (Odometry)
Publishers: /cmd_attitude (QuaternionStamped), /cmd_thrust (Float64), /tracking_error (Float64)
Parameters: kp_pos (5.0), kd_pos (2.0), kp_yaw (2.0), max_thrust (20.0), gravity (9.81)
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import Float64


class TrajectoryTrackerNode(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')
        self.declare_parameter('kp_pos', 5.0)
        self.declare_parameter('kd_pos', 2.0)
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('max_thrust', 20.0)
        self.declare_parameter('gravity', 9.81)
        self.kp = self.get_parameter('kp_pos').value
        self.kd = self.get_parameter('kd_pos').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.max_thrust = self.get_parameter('max_thrust').value
        self.g = self.get_parameter('gravity').value
        self.trajectory = None
        self.traj_idx = 0
        self.pose = None
        self.vel = None
        self.create_subscription(Path, '/local_trajectory', self.traj_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub_att = self.create_publisher(QuaternionStamped, '/cmd_attitude', 10)
        self.pub_thrust = self.create_publisher(Float64, '/cmd_thrust', 10)
        self.pub_err = self.create_publisher(Float64, '/tracking_error', 10)
        self.create_timer(0.02, self.control)
        self.get_logger().info('Trajectory Tracker started')

    def traj_cb(self, msg):
        if len(msg.poses) > 0:
            self.trajectory = msg.poses
            self.traj_idx = 0

    def odom_cb(self, msg):
        self.pose = msg.pose.pose
        self.vel = msg.twist.twist.linear

    def control(self):
        if self.trajectory is None or self.pose is None:
            return
        # TODO: Differential flatness-based control
        # 1. Get desired position from trajectory[traj_idx]
        # 2. Compute position error: e_pos = desired - current
        # 3. Compute velocity error: e_vel = 0 - current_vel (or desired_vel)
        # 4. Compute desired acceleration: a_des = kp*e_pos + kd*e_vel
        # 5. Add gravity compensation: a_des[2] += gravity
        # 6. Compute thrust magnitude: thrust = mass * ||a_des||
        # 7. Compute desired attitude from a_des direction
        # 8. Publish cmd_attitude (QuaternionStamped) and cmd_thrust (Float64)
        # 9. Publish tracking_error (Float64)
        # 10. Advance traj_idx if close enough to current waypoint

        pass  # TODO: implement


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrajectoryTrackerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
