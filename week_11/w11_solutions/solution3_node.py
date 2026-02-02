#!/usr/bin/env python3
"""Week 11 Solution 3: MPC Controller (Complete)"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Wrench
from std_msgs.msg import Float64, Header
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

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.ref_sub = self.create_subscription(PoseStamped, "/reference_trajectory", self.ref_callback, 10)
        self.cmd_pub = self.create_publisher(Wrench, "/cmd_thrust_attitude", 10)
        self.path_pub = self.create_publisher(Path, "/predicted_trajectory", 10)
        self.cost_pub = self.create_publisher(Float64, "/mpc_cost", 10)

        self.state = np.zeros(6)
        self.ref_pos = np.zeros(3)
        self.prev_pos = None
        self.prev_time = None
        self.warm_start = np.zeros(self.N * 3)
        self.get_logger().info(f"MPC: N={self.N}, scipy={HAS_SCIPY}")

    def ref_callback(self, msg):
        self.ref_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def dynamics(self, x, u):
        x_new = x.copy()
        x_new[:3] = x[:3] + x[3:] * self.dt
        x_new[3:] = x[3:] + (u / self.mass) * self.dt
        return x_new

    def mpc_cost_fn(self, u_flat):
        U = u_flat.reshape(self.N, 3)
        x = self.state.copy()
        ref = np.zeros(6)
        ref[:3] = self.ref_pos
        cost = 0.0
        for k in range(self.N):
            x = self.dynamics(x, U[k])
            err = x - ref
            cost += err @ self.Q @ err + U[k] @ self.R @ U[k]
        return cost

    def odom_callback(self, msg):
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is not None and t > self.prev_time:
            self.state[:3] = pos
            self.state[3:] = (pos - self.prev_pos) / (t - self.prev_time)
        self.prev_pos = pos
        self.prev_time = t

        if HAS_SCIPY:
            bounds = [(self.u_min, self.u_max)] * (self.N * 3)
            result = minimize(self.mpc_cost_fn, self.warm_start,
                            method="SLSQP", bounds=bounds,
                            options={"maxiter": 50, "ftol": 1e-4})
            u_opt = result.x.reshape(self.N, 3)
            self.warm_start = np.roll(result.x.reshape(self.N, 3), -1, axis=0).flatten()
            total_cost = result.fun
        else:
            err = self.ref_pos - self.state[:3]
            u_opt = np.zeros((self.N, 3))
            u_opt[0] = np.clip(3.0 * err - 2.0 * self.state[3:], self.u_min, self.u_max)
            total_cost = float(np.sum(err**2))

        cmd = Wrench()
        cmd.force.x, cmd.force.y = float(u_opt[0, 0]), float(u_opt[0, 1])
        cmd.force.z = float(u_opt[0, 2]) + 9.81
        self.cmd_pub.publish(cmd)

        # Publish predicted trajectory
        path = Path()
        path.header = Header(stamp=msg.header.stamp, frame_id="world")
        x_pred = self.state.copy()
        for k in range(self.N):
            x_pred = self.dynamics(x_pred, u_opt[k])
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x_pred[0])
            ps.pose.position.y = float(x_pred[1])
            ps.pose.position.z = float(x_pred[2])
            path.poses.append(ps)
        self.path_pub.publish(path)

        cost_msg = Float64()
        cost_msg.data = float(total_cost)
        self.cost_pub.publish(cost_msg)
        self.get_logger().info(f"MPC cost: {total_cost:.2f}, u0: [{u_opt[0,0]:.2f},{u_opt[0,1]:.2f},{u_opt[0,2]:.2f}]")


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
