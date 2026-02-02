#!/usr/bin/env python3
"""
Week 2 - Exercise 1: IMU Processor Node
Subscribe to /imu0 (Imu), integrate gyro for orientation,
publish /integrated_orientation (QuaternionStamped).
"""
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped, Quaternion


class IMUProcessorNode(Node):
    def __init__(self):
        super().__init__("imu_processor_node")
        self.get_logger().info("IMU Processor Node started")

        # TODO 1: Create subscriber to /imu0 (Imu)
        # self.subscription = self.create_subscription(Imu, "/imu0", self.imu_callback, 10)

        # TODO 2: Create publisher for /integrated_orientation (QuaternionStamped)
        # self.orient_pub = self.create_publisher(QuaternionStamped, "/integrated_orientation", 10)

        # State: orientation as quaternion [w, x, y, z]
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.last_time = None

    def imu_callback(self, msg: Imu):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0 or dt > 1.0:
            return

        # TODO 3: Extract angular velocity from IMU message
        # wx = msg.angular_velocity.x
        # wy = msg.angular_velocity.y
        # wz = msg.angular_velocity.z

        # TODO 4: Integrate gyroscope using quaternion kinematics
        # omega = np.array([0, wx, wy, wz])
        # q_dot = 0.5 * self.quaternion_multiply(self.q, omega)
        # self.q = self.q + q_dot * dt
        # self.q = self.q / np.linalg.norm(self.q)  # Normalize

        # TODO 5: Publish orientation
        # out = QuaternionStamped()
        # out.header = msg.header
        # out.quaternion = Quaternion(w=self.q[0], x=self.q[1], y=self.q[2], z=self.q[3])
        # self.orient_pub.publish(out)

        pass

    def quaternion_multiply(self, q, r):
        """Multiply two quaternions [w, x, y, z]."""
        # TODO 6: Implement quaternion multiplication
        # w = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3]
        # x = q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2]
        # y = q[0]*r[2] - q[1]*r[3] + q[2]*r[0] + q[3]*r[1]
        # z = q[0]*r[3] + q[1]*r[2] - q[2]*r[1] + q[3]*r[0]
        # return np.array([w, x, y, z])
        return np.array([1.0, 0.0, 0.0, 0.0])


def main(args=None):
    rclpy.init(args=args)
    node = IMUProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
