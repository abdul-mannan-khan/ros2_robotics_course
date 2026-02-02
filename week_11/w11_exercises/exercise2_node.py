#!/usr/bin/env python3
"""Week 11 Exercise 2: LQR Controller Node
Subscribe to /odom and /reference_trajectory.
Linearize quadrotor dynamics, compute LQR gain.
Publish /cmd_thrust_attitude (Wrench), /lqr_gain (Float64MultiArray).

TODO: Solve discrete-time Riccati equation, compute gain.
"""
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
        self.dt = self.get_parameter("dt").value
        self.mass = self.get_parameter("mass").value

        # State: [x, y, z, vx, vy, vz], Input: [fx, fy, fz]
        self.n_states = 6
        self.n_inputs = 3
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)

        # ============================================================
        # TODO: Compute LQR gain K
        # 1. Linearize double-integrator dynamics about hover:
        #    A_c = [[0, I], [0, 0]]  (6x6 continuous)
        #    B_c = [[0], [I/m]]      (6x3 continuous)
        # 2. Discretize: A_d = I + A_c*dt, B_d = B_c*dt
        # 3. Solve DARE: P = solve_discrete_are(A_d, B_d, Q, R)
        # 4. Gain: K = inv(R + B.T @ P @ B) @ B.T @ P @ A
        # ============================================================
        self.K = np.zeros((self.n_inputs, self.n_states))

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.ref_sub = self.create_subscription(
            PoseStamped, "/reference_trajectory", self.ref_callback, 10)
        self.cmd_pub = self.create_publisher(Wrench, "/cmd_thrust_attitude", 10)
        self.gain_pub = self.create_publisher(Float64MultiArray, "/lqr_gain", 10)

        self.state = np.zeros(self.n_states)
        self.ref_state = np.zeros(self.n_states)
        self.prev_pos = None
        self.prev_time = None

        self.get_logger().info(f"LQR Controller started (scipy={HAS_SCIPY})")

    def ref_callback(self, msg):
        self.ref_state[:3] = [msg.pose.position.x,
                              msg.pose.position.y,
                              msg.pose.position.z]

    def odom_callback(self, msg):
        pos = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is not None and t > self.prev_time:
            dt = t - self.prev_time
            self.state[:3] = pos
            self.state[3:] = (pos - self.prev_pos) / dt
        self.prev_pos = pos
        self.prev_time = t

        error_state = self.state - self.ref_state
        u = -self.K @ error_state

        cmd = Wrench()
        cmd.force.x = float(u[0])
        cmd.force.y = float(u[1])
        cmd.force.z = float(u[2]) + self.hover_thrust
        self.cmd_pub.publish(cmd)

        gain_msg = Float64MultiArray()
        gain_msg.data = self.K.flatten().tolist()
        self.gain_pub.publish(gain_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
