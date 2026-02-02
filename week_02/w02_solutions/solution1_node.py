#!/usr/bin/env python3
"""Week 2 - Solution 1: IMU Processor Node (Complete Implementation)"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped, Quaternion


class IMUProcessorNode(Node):
    def __init__(self):
        super().__init__("imu_processor_node")
        self.subscription = self.create_subscription(Imu, "/imu0", self.imu_callback, 10)
        self.orient_pub = self.create_publisher(QuaternionStamped, "/integrated_orientation", 10)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self.last_time = None
        self.get_logger().info("IMU Processor Node started")

    def imu_callback(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = t
            return
        dt = t - self.last_time
        self.last_time = t
        if dt <= 0 or dt > 1.0:
            return

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # Quaternion kinematics: q_dot = 0.5 * q * [0, wx, wy, wz]
        omega = np.array([0.0, wx, wy, wz])
        q_dot = 0.5 * self.quat_mult(self.q, omega)
        self.q = self.q + q_dot * dt
        self.q = self.q / np.linalg.norm(self.q)

        out = QuaternionStamped()
        out.header = msg.header
        out.quaternion = Quaternion(w=float(self.q[0]), x=float(self.q[1]),
                                    y=float(self.q[2]), z=float(self.q[3]))
        self.orient_pub.publish(out)

    def quat_mult(self, q, r):
        w = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3]
        x = q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2]
        y = q[0]*r[2] - q[1]*r[3] + q[2]*r[0] + q[3]*r[1]
        z = q[0]*r[3] + q[1]*r[2] - q[2]*r[1] + q[3]*r[0]
        return np.array([w, x, y, z])


def main(args=None):
    rclpy.init(args=args)
    node = IMUProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
