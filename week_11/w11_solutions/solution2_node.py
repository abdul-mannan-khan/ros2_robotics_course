#!/usr/bin/env python3
"""Week 11 Solution 2: LQR Controller (Complete)"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Wrench
from std_msgs.msg import Float64MultiArray
import numpy as np

try:
    from scipy.linalg import solve_discrete_are
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class LQRControllerNode(Node):
    def __init__(self):
        super().__init__("lqr_controller_node")
        self.declare_parameter("Q_diag", [10.0, 10.0, 20.0, 1.0, 1.0, 1.0])
        self.declare_parameter("R_diag", [1.0, 1.0, 1.0])
        self.declare_parameter("hover_thrust", 9.81)
        self.declare_parameter("dt", 0.02)
        self.declare_parameter("mass", 1.0)

        Q_diag = self.get_parameter("Q_diag").value
        R_diag = self.get_parameter("R_diag").value
        self.hover_thrust = self.get_parameter("hover_thrust").value
        dt = self.get_parameter("dt").value
        m = self.get_parameter("mass").value

        Q = np.diag(Q_diag)
        R = np.diag(R_diag)

        # Linearized double-integrator (continuous)
        A_c = np.zeros((6, 6))
        A_c[:3, 3:] = np.eye(3)
        B_c = np.zeros((6, 3))
        B_c[3:, :] = np.eye(3) / m

        # Discretize (Euler)
        A_d = np.eye(6) + A_c * dt
        B_d = B_c * dt

        # Solve DARE
        if HAS_SCIPY:
            P = solve_discrete_are(A_d, B_d, Q, R)
            self.K = np.linalg.inv(R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)
            self.get_logger().info(f"LQR gain computed, K shape: {self.K.shape}")
        else:
            self.get_logger().warn("scipy not available, using fallback gains")
            self.K = np.zeros((3, 6))
            self.K[0, 0] = 3.0; self.K[0, 3] = 2.0
            self.K[1, 1] = 3.0; self.K[1, 4] = 2.0
            self.K[2, 2] = 5.0; self.K[2, 5] = 3.0

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.ref_sub = self.create_subscription(PoseStamped, "/reference_trajectory", self.ref_callback, 10)
        self.cmd_pub = self.create_publisher(Wrench, "/cmd_thrust_attitude", 10)
        self.gain_pub = self.create_publisher(Float64MultiArray, "/lqr_gain", 10)

        self.state = np.zeros(6)
        self.ref_state = np.zeros(6)
        self.prev_pos = None
        self.prev_time = None

    def ref_callback(self, msg):
        self.ref_state[:3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def odom_callback(self, msg):
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is not None and t > self.prev_time:
            self.state[:3] = pos
            self.state[3:] = (pos - self.prev_pos) / (t - self.prev_time)
        self.prev_pos = pos
        self.prev_time = t

        err = self.state - self.ref_state
        u = -self.K @ err

        cmd = Wrench()
        cmd.force.x, cmd.force.y = float(u[0]), float(u[1])
        cmd.force.z = float(u[2]) + self.hover_thrust
        self.cmd_pub.publish(cmd)

        gain_msg = Float64MultiArray()
        gain_msg.data = self.K.flatten().tolist()
        self.gain_pub.publish(gain_msg)

        self.get_logger().info(f"LQR err:[{err[0]:.2f},{err[1]:.2f},{err[2]:.2f}] u:[{u[0]:.2f},{u[1]:.2f},{u[2]:.2f}]")


def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
