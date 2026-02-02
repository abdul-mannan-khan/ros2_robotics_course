#!/usr/bin/env python3
"""Exercise 3: Path Follower Node (Pure Pursuit).
Subscribe to /planned_path and /odom, publish /cmd_vel."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__("path_follower_node")
        self.declare_parameter("lookahead_distance", 0.5)
        self.declare_parameter("max_linear_vel", 0.5)
        self.declare_parameter("max_angular_vel", 1.0)
        self.la = self.get_parameter("lookahead_distance").value
        self.max_v = self.get_parameter("max_linear_vel").value
        self.max_w = self.get_parameter("max_angular_vel").value
        self.path_sub = self.create_subscription(Path, "/planned_path", self.path_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path = None
        self.rx, self.ry, self.rth = 0.0, 0.0, 0.0
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("PathFollowerNode started")

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.rth = 2.0 * math.atan2(q.z, q.w)

    def find_lookahead(self):
        """TODO: Find the lookahead point on the path.
        1. Find the closest point on path to robot
        2. From that point, walk along path until distance >= lookahead_distance
        3. Return (lx, ly) lookahead point or None if path ended"""
        return None

    def pure_pursuit(self, lx, ly):
        """TODO: Compute cmd_vel using pure pursuit.
        1. Transform lookahead to robot frame
        2. Compute curvature: kappa = 2*y_local / L^2
        3. linear_vel = max_linear_vel
        4. angular_vel = kappa * linear_vel (clamp to max_angular_vel)
        Return (v, w)"""
        return 0.0, 0.0

    def control_loop(self):
        if self.path is None or len(self.path) == 0:
            return
        la = self.find_lookahead()
        if la is None:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        v, w = self.pure_pursuit(la[0], la[1])
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
