#!/usr/bin/env python3
"""
Week 2 - Exercise 3: Full EKF Fusion Node
Subscribe to /imu0 and /leica/position.
EKF predict (IMU) + update (position measurements).
Publish /ekf_fused (Odometry), /ekf_covariance (Float64MultiArray).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, PoseWithCovariance, TwistWithCovariance, Twist, Vector3
from std_msgs.msg import Float64MultiArray


class EKFFusionNode(Node):
    def __init__(self):
        super().__init__("ekf_fusion_node")

        # TODO 1: Declare parameters
        # self.declare_parameter("process_noise_q", 0.01)
        # self.declare_parameter("measurement_noise_r", 0.1)
        self.process_noise_q = 0.01
        self.measurement_noise_r = 0.1

        # State: [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        self.state = np.zeros(10)
        self.state[6] = 1.0
        self.P = np.eye(10) * 0.1

        # TODO 2: Subscribe to /imu0 (predict) and /leica/position (update)
        # self.imu_sub = self.create_subscription(Imu, "/imu0", self.imu_callback, 10)
        # self.pos_sub = self.create_subscription(PointStamped, "/leica/position", self.position_callback, 10)

        # TODO 3: Create publishers
        # self.fused_pub = self.create_publisher(Odometry, "/ekf_fused", 10)
        # self.cov_pub = self.create_publisher(Float64MultiArray, "/ekf_covariance", 10)

        self.last_time = None
        self.get_logger().info("EKF Fusion Node started")

    def imu_callback(self, msg: Imu):
        """EKF Predict step using IMU data."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = t
            return
        dt = t - self.last_time
        self.last_time = t
        if dt <= 0 or dt > 1.0:
            return

        # TODO 4: Implement EKF predict (same as Exercise 2)
        # accel = np.array([msg.linear_acceleration.x, ...])
        # gyro = np.array([msg.angular_velocity.x, ...])
        # ... predict state and covariance ...

        self.publish_state(msg.header)

    def position_callback(self, msg: PointStamped):
        """EKF Update step using position measurement."""
        # TODO 5: Implement EKF update
        # Measurement: z = [x, y, z] from position message
        # z = np.array([msg.point.x, msg.point.y, msg.point.z])
        # H = np.zeros((3, 10))  # Observation matrix
        # H[0, 0] = H[1, 1] = H[2, 2] = 1.0  # We observe position directly
        # R = np.eye(3) * self.measurement_noise_r
        # y = z - H @ self.state  # Innovation
        # S = H @ self.P @ H.T + R  # Innovation covariance
        # K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        # self.state = self.state + K @ y
        # self.P = (np.eye(10) - K @ H) @ self.P
        # # Normalize quaternion
        # q_norm = np.linalg.norm(self.state[6:10])
        # if q_norm > 0: self.state[6:10] /= q_norm

        self.publish_state(msg.header)

    def publish_state(self, header):
        """Publish current state estimate."""
        # TODO 6: Publish Odometry and covariance
        # odom = Odometry()
        # odom.header.stamp = header.stamp
        # odom.header.frame_id = "odom"
        # odom.pose.pose = Pose(
        #     position=Point(x=float(self.state[0]), y=float(self.state[1]), z=float(self.state[2])),
        #     orientation=Quaternion(w=float(self.state[6]), x=float(self.state[7]),
        #                           y=float(self.state[8]), z=float(self.state[9])))
        # self.fused_pub.publish(odom)
        # cov_msg = Float64MultiArray()
        # cov_msg.data = self.P.flatten().tolist()
        # self.cov_pub.publish(cov_msg)
        pass


def main(args=None):
    rclpy.init(args=args)
    node = EKFFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
