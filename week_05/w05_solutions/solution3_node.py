#!/usr/bin/env python3
"""Solution 3: Path Follower (Pure Pursuit) - Full implementation."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

class PathFollowerSolution(Node):
    def __init__(self):
        super().__init__("path_follower_solution")
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
        self.get_logger().info("PathFollowerSolution started")

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.rth = 2.0 * math.atan2(q.z, q.w)

    def find_lookahead(self):
        if not self.path: return None
        # Find closest point
        dists = [math.sqrt((px-self.rx)**2+(py-self.ry)**2) for px,py in self.path]
        ci = int(np.argmin(dists))
        # Walk forward to lookahead
        for i in range(ci, len(self.path)):
            px, py = self.path[i]
            d = math.sqrt((px-self.rx)**2+(py-self.ry)**2)
            if d >= self.la:
                return (px, py)
        return self.path[-1] if self.path else None

    def pure_pursuit(self, lx, ly):
        dx = lx - self.rx; dy = ly - self.ry
        local_x = dx*math.cos(self.rth) + dy*math.sin(self.rth)
        local_y = -dx*math.sin(self.rth) + dy*math.cos(self.rth)
        L2 = local_x**2 + local_y**2
        if L2 < 1e-6: return 0.0, 0.0
        kappa = 2.0 * local_y / L2
        v = self.max_v
        w = np.clip(kappa * v, -self.max_w, self.max_w)
        return v, w

    def control_loop(self):
        if not self.path: return
        la = self.find_lookahead()
        cmd = Twist()
        if la:
            d = math.sqrt((la[0]-self.rx)**2+(la[1]-self.ry)**2)
            if d < 0.1:
                self.path = None
            else:
                v, w = self.pure_pursuit(la[0], la[1])
                cmd.linear.x = v; cmd.angular.z = w
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerSolution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
