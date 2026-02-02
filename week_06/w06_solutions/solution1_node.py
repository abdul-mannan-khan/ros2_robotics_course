#!/usr/bin/env python3
"""Solution 1: Attitude Estimator - Complementary filter implementation."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped

class AttitudeEstimatorSolution(Node):
    def __init__(self):
        super().__init__("attitude_estimator_solution")
        self.declare_parameter("alpha", 0.98)
        self.alpha = self.get_parameter("alpha").value
        self.sub = self.create_subscription(Imu, "/imu", self.imu_cb, 100)
        self.pub = self.create_publisher(QuaternionStamped, "/estimated_attitude", 10)
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.last_time = None
        self.get_logger().info("AttitudeEstimatorSolution started")

    def euler_to_quat(self, r, p, y):
        cr,sr=math.cos(r/2),math.sin(r/2)
        cp,sp=math.cos(p/2),math.sin(p/2)
        cy,sy=math.cos(y/2),math.sin(y/2)
        return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)

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
        # Accel-based angles
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        # Complementary filter
        a = self.alpha
        self.roll = a * (self.roll + gx * dt) + (1 - a) * roll_acc
        self.pitch = a * (self.pitch + gy * dt) + (1 - a) * pitch_acc
        self.yaw += gz * dt
        # Publish
        qx, qy, qz, qw = self.euler_to_quat(self.roll, self.pitch, self.yaw)
        out = QuaternionStamped()
        out.header = msg.header
        out.quaternion.x = qx; out.quaternion.y = qy
        out.quaternion.z = qz; out.quaternion.w = qw
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = AttitudeEstimatorSolution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
