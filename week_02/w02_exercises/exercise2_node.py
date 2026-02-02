#!/usr/bin/env python3
"""
Week 2 - Exercise 2: EKF Prediction Node
Subscribe to /imu0, implement EKF predict step.
State: [x, y, z, vx, vy, vz, qw, qx, qy, qz] (10-dim).
Publish /ekf_state (Odometry).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance


class EKFPredictionNode(Node):
    def __init__(self):
        super().__init__("ekf_prediction_node")

        # TODO 1: Declare parameters
        # self.declare_parameter("process_noise_q", 0.01)
        # self.declare_parameter("dt", 0.005)
        self.process_noise_q = 0.01
        self.dt = 0.005

        # State: [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        self.state = np.zeros(10)
        self.state[6] = 1.0  # qw = 1
        self.P = np.eye(10) * 0.1  # Covariance

        # TODO 2: Create subscriber to /imu0
        # self.sub = self.create_subscription(Imu, "/imu0", self.imu_callback, 10)

        # TODO 3: Create publisher for /ekf_state (Odometry)
        # self.state_pub = self.create_publisher(Odometry, "/ekf_state", 10)

        self.last_time = None
        self.get_logger().info("EKF Prediction Node started")

    def imu_callback(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = t
            return
        dt = t - self.last_time
        self.last_time = t
        if dt <= 0 or dt > 1.0:
            return

        # TODO 4: Extract acceleration and angular velocity
        # accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        # gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # TODO 5: EKF Predict step
        # - Rotate acceleration to world frame using current quaternion
        # - Update velocity: v += (R @ accel - gravity) * dt
        # - Update position: p += v * dt
        # - Update quaternion: q += 0.5 * q * [0, gyro] * dt, then normalize
        # - Update covariance: P = F @ P @ F.T + Q

        # TODO 6: Publish state as Odometry
        # odom = Odometry()
        # odom.header = msg.header
        # odom.header.frame_id = "odom"
        # odom.pose.pose = Pose(
        #     position=Point(x=self.state[0], y=self.state[1], z=self.state[2]),
        #     orientation=Quaternion(w=self.state[6], x=self.state[7], y=self.state[8], z=self.state[9]))
        # self.state_pub.publish(odom)

        pass


def main(args=None):
    rclpy.init(args=args)
    node = EKFPredictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
