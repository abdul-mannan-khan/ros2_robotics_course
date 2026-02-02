#!/usr/bin/env python3
"""Week 11 Exercise 1: PID Position Controller Node
Subscribe to /odom and /reference_trajectory.
3-axis PID controller for position tracking.
Publish /cmd_thrust_attitude (Wrench) and /tracking_error (Vector3).

TODO: Implement PID with anti-windup.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Wrench, Vector3
import numpy as np


class PIDController:
    def __init__(self, kp, ki, kd, output_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, current_time):
        # ============================================================
        # TODO: Implement PID with anti-windup
        # 1. Compute dt from current_time and self.prev_time
        # 2. Proportional: P = kp * error
        # 3. Integral: self.integral += error * dt
        #    - Anti-windup: clamp integral to [-limit/ki, limit/ki]
        #    I = ki * self.integral
        # 4. Derivative: D = kd * (error - self.prev_error) / dt
        # 5. Output = P + I + D, clamped to [-output_limit, output_limit]
        # 6. Update prev_error, prev_time
        # ============================================================
        output = 0.0  # Replace with PID output
        self.prev_error = error
        self.prev_time = current_time
        return output


class PIDPositionControllerNode(Node):
    def __init__(self):
        super().__init__("pid_position_controller_node")

        self.declare_parameter("kp_x", 2.0)
        self.declare_parameter("kp_y", 2.0)
        self.declare_parameter("kp_z", 4.0)
        self.declare_parameter("ki_x", 0.1)
        self.declare_parameter("ki_y", 0.1)
        self.declare_parameter("ki_z", 0.2)
        self.declare_parameter("kd_x", 0.5)
        self.declare_parameter("kd_y", 0.5)
        self.declare_parameter("kd_z", 1.0)
        self.declare_parameter("max_thrust", 20.0)
        self.declare_parameter("output_limits", 10.0)

        lim = self.get_parameter("output_limits").value
        self.pid_x = PIDController(
            self.get_parameter("kp_x").value,
            self.get_parameter("ki_x").value,
            self.get_parameter("kd_x").value, lim)
        self.pid_y = PIDController(
            self.get_parameter("kp_y").value,
            self.get_parameter("ki_y").value,
            self.get_parameter("kd_y").value, lim)
        self.pid_z = PIDController(
            self.get_parameter("kp_z").value,
            self.get_parameter("ki_z").value,
            self.get_parameter("kd_z").value, lim)

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.ref_sub = self.create_subscription(
            PoseStamped, "/reference_trajectory", self.ref_callback, 10)

        self.cmd_pub = self.create_publisher(Wrench, "/cmd_thrust_attitude", 10)
        self.error_pub = self.create_publisher(Vector3, "/tracking_error", 10)

        self.current_pos = np.zeros(3)
        self.reference_pos = np.zeros(3)
        self.get_logger().info("PID Position Controller started")

    def ref_callback(self, msg):
        self.reference_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z])

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        error = self.reference_pos - self.current_pos

        fx = self.pid_x.compute(error[0], t)
        fy = self.pid_y.compute(error[1], t)
        fz = self.pid_z.compute(error[2], t)

        cmd = Wrench()
        cmd.force.x = fx
        cmd.force.y = fy
        cmd.force.z = fz
        self.cmd_pub.publish(cmd)

        err_msg = Vector3(x=error[0], y=error[1], z=error[2])
        self.error_pub.publish(err_msg)
        self.get_logger().info(
            f"Error: [{error[0]:.2f}, {error[1]:.2f}, {error[2]:.2f}]")


def main(args=None):
    rclpy.init(args=args)
    node = PIDPositionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
