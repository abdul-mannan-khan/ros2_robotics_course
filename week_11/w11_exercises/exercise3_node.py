#!/usr/bin/env python3
"""Week 11 Exercise 3: MPC Controller Node
Subscribe to /odom and /reference_trajectory.
Simple MPC: predict N steps, optimize over horizon.
Publish /cmd_thrust_attitude (Wrench), /predicted_trajectory (Path), /mpc_cost (Float64).

TODO: QP formulation and solve.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Wrench
from std_msgs.msg import Float64
import numpy as np

try:
    from scipy.optimize import minimize
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class MPCControllerNode(Node):
    def __init__(self):
        super().__init__("mpc_controller_node")
        self.declare_parameter("horizon_N", 10)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("Q_diag", [10.0, 10.0, 20.0, 1.0, 1.0, 1.0])
        self.declare_parameter("R_diag", [0.1, 0.1, 0.1])
        self.declare_parameter("u_min", -5.0)
        self.declare_parameter("u_max", 15.0)
        self.declare_parameter("mass", 1.0)

        self.N = self.get_parameter("horizon_N").value
        self.dt = self.get_parameter("dt").value
        self.Q = np.diag(self.get_parameter("Q_diag").value)
        self.R = np.diag(self.get_parameter("R_diag").value)
        self.u_min = self.get_parameter("u_min").value
        self.u_max = self.get_parameter("u_max").value
        self.mass = self.get_parameter("mass").value

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.ref_sub = self.create_subscription(
            PoseStamped, "/reference_trajectory", self.ref_callback, 10)
        self.cmd_pub = self.create_publisher(Wrench, "/cmd_thrust_attitude", 10)
        self.path_pub = self.create_publisher(Path, "/predicted_trajectory", 10)
        self.cost_pub = self.create_publisher(Float64, "/mpc_cost", 10)

        self.state = np.zeros(6)
        self.ref_pos = np.zeros(3)
        self.prev_pos = None
        self.prev_time = None
        self.get_logger().info(f"MPC Controller: N={self.N}, dt={self.dt}")

    def ref_callback(self, msg):
        self.ref_pos = np.array([msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z])

    def dynamics(self, state, u):
        # ============================================================
        # TODO: Implement discrete dynamics
        # Double integrator: x_{k+1} = A*x_k + B*u_k
        # state = [x, y, z, vx, vy, vz]
        # u = [fx, fy, fz] (force inputs)
        # x_new[:3] = x[:3] + x[3:]*dt
        # x_new[3:] = x[3:] + (u/mass)*dt
        # ============================================================
        x_new = state.copy()
        return x_new

    def mpc_cost_fn(self, u_flat):
        # ============================================================
        # TODO: Implement MPC cost function
        # 1. Reshape u_flat to (N, 3)
        # 2. Simulate forward N steps using dynamics
        # 3. Accumulate cost: sum of x.T @ Q @ x + u.T @ R @ u
        #    where x is error from reference
        # 4. Return total cost
        # ============================================================
        return 0.0

    def odom_callback(self, msg):
        pos = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is not None and t > self.prev_time:
            self.state[:3] = pos
            self.state[3:] = (pos - self.prev_pos) / (t - self.prev_time)
        self.prev_pos = pos
        self.prev_time = t

        # Solve MPC (placeholder: zero input)
        u_opt = np.zeros(3)

        cmd = Wrench()
        cmd.force.x, cmd.force.y, cmd.force.z = float(u_opt[0]), float(u_opt[1]), float(u_opt[2]) + 9.81
        self.cmd_pub.publish(cmd)

        cost_msg = Float64()
        cost_msg.data = 0.0
        self.cost_pub.publish(cost_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
