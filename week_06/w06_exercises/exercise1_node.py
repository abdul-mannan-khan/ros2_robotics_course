#!/usr/bin/env python3
"""Exercise 1: Attitude Estimator Node (Complementary Filter).
Subscribe to /imu (1000Hz), publish /estimated_attitude (QuaternionStamped)."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped

class AttitudeEstimatorNode(Node):
    def __init__(self):
        super().__init__("attitude_estimator_node")
        self.declare_parameter("alpha", 0.98)
        self.alpha = self.get_parameter("alpha").value
        self.sub = self.create_subscription(Imu, "/imu", self.imu_cb, 100)
        self.pub = self.create_publisher(QuaternionStamped, "/estimated_attitude", 10)
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.last_time = None
        self.get_logger().info("AttitudeEstimatorNode started")

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = t; return
        dt = t - self.last_time
        self.last_time = t
        if dt <= 0 or dt > 0.1: return
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        # TODO: Implement complementary filter
        # 1. Gyro integration: roll += gx*dt, pitch += gy*dt, yaw += gz*dt
        # 2. Accel angles: roll_acc = atan2(ay, az), pitch_acc = atan2(-ax, sqrt(ay^2+az^2))
        # 3. Fuse: roll = alpha*(roll+gx*dt) + (1-alpha)*roll_acc
        #          pitch = alpha*(pitch+gy*dt) + (1-alpha)*pitch_acc
        #          yaw += gz*dt (no accel correction for yaw)
        # 4. Convert to quaternion and publish
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AttitudeEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
