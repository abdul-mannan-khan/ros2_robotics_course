#!/usr/bin/env python3
"""Week 2 - Solution 2: EKF Prediction Node (Complete Implementation)"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, PoseWithCovariance, TwistWithCovariance, Twist, Vector3


class EKFPredictionNode(Node):
    def __init__(self):
        super().__init__("ekf_prediction_node")
        self.declare_parameter("process_noise_q", 0.01)
        self.q_noise = self.get_parameter("process_noise_q").value
        self.state = np.zeros(10)
        self.state[6] = 1.0
        self.P = np.eye(10) * 0.1
        self.sub = self.create_subscription(Imu, "/imu0", self.imu_callback, 10)
        self.pub = self.create_publisher(Odometry, "/ekf_state", 10)
        self.last_time = None
        self.get_logger().info("EKF Prediction Node started")

    def quat_mult(self, q, r):
        return np.array([
            q[0]*r[0]-q[1]*r[1]-q[2]*r[2]-q[3]*r[3],
            q[0]*r[1]+q[1]*r[0]+q[2]*r[3]-q[3]*r[2],
            q[0]*r[2]-q[1]*r[3]+q[2]*r[0]+q[3]*r[1],
            q[0]*r[3]+q[1]*r[2]-q[2]*r[1]+q[3]*r[0]])

    def quat_to_rot(self, q):
        w,x,y,z = q
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

    def imu_callback(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = t; return
        dt = t - self.last_time
        self.last_time = t
        if dt <= 0 or dt > 1.0: return

        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        gravity = np.array([0.0, 0.0, 9.81])

        # Rotate accel to world frame
        q = self.state[6:10]
        R = self.quat_to_rot(q)
        accel_world = R @ accel - gravity

        # Update state
        self.state[0:3] += self.state[3:6] * dt + 0.5 * accel_world * dt**2
        self.state[3:6] += accel_world * dt

        # Update quaternion
        omega_q = np.array([0.0, gyro[0], gyro[1], gyro[2]])
        q_dot = 0.5 * self.quat_mult(q, omega_q)
        self.state[6:10] += q_dot * dt
        self.state[6:10] /= np.linalg.norm(self.state[6:10])

        # Update covariance (simplified: F ~ I + dt * ...)
        F = np.eye(10)
        F[0:3, 3:6] = np.eye(3) * dt
        Q = np.eye(10) * self.q_noise * dt
        self.P = F @ self.P @ F.T + Q

        # Publish
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        s = self.state
        odom.pose = PoseWithCovariance(
            pose=Pose(position=Point(x=float(s[0]),y=float(s[1]),z=float(s[2])),
                      orientation=Quaternion(w=float(s[6]),x=float(s[7]),y=float(s[8]),z=float(s[9]))),
            covariance=np.zeros(36).tolist())
        odom.twist = TwistWithCovariance(
            twist=Twist(linear=Vector3(x=float(s[3]),y=float(s[4]),z=float(s[5]))),
            covariance=np.zeros(36).tolist())
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = EKFPredictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
