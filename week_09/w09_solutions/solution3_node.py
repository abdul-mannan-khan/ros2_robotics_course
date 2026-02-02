#!/usr/bin/env python3
"""Week 9 Solution 3: Trajectory Tracker (Complete)"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import Float64


class TrajectoryTrackerNode(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')
        self.declare_parameter('kp_pos', 5.0)
        self.declare_parameter('kd_pos', 2.0)
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('max_thrust', 20.0)
        self.declare_parameter('gravity', 9.81)
        self.kp = self.get_parameter('kp_pos').value
        self.kd = self.get_parameter('kd_pos').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.max_thrust = self.get_parameter('max_thrust').value
        self.g = self.get_parameter('gravity').value
        self.trajectory = None
        self.traj_idx = 0
        self.pose = None
        self.vel = None
        self.create_subscription(Path, '/local_trajectory', self.traj_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub_att = self.create_publisher(QuaternionStamped, '/cmd_attitude', 10)
        self.pub_thrust = self.create_publisher(Float64, '/cmd_thrust', 10)
        self.pub_err = self.create_publisher(Float64, '/tracking_error', 10)
        self.create_timer(0.02, self.control)
        self.get_logger().info('Trajectory Tracker started')

    def traj_cb(self, msg):
        if len(msg.poses) > 0:
            self.trajectory = msg.poses
            self.traj_idx = 0

    def odom_cb(self, msg):
        self.pose = msg.pose.pose
        self.vel = msg.twist.twist.linear

    def control(self):
        if self.trajectory is None or self.pose is None or self.vel is None:
            return
        if self.traj_idx >= len(self.trajectory):
            return
        des = self.trajectory[self.traj_idx].pose.position
        cur = self.pose.position
        e_pos = np.array([des.x-cur.x, des.y-cur.y, des.z-cur.z])
        e_vel = np.array([-self.vel.x, -self.vel.y, -self.vel.z])
        error = np.linalg.norm(e_pos)
        # Advance waypoint
        if error < 0.3 and self.traj_idx < len(self.trajectory)-1:
            self.traj_idx += 1
        # PD control + gravity
        a_des = self.kp * e_pos + self.kd * e_vel
        a_des[2] += self.g
        thrust = min(np.linalg.norm(a_des), self.max_thrust)
        # Desired attitude from acceleration direction
        z_body = a_des / max(np.linalg.norm(a_des), 1e-6)
        x_c = np.array([math.cos(0.0), math.sin(0.0), 0.0])
        y_body = np.cross(z_body, x_c)
        y_norm = np.linalg.norm(y_body)
        if y_norm > 1e-6:
            y_body /= y_norm
        else:
            y_body = np.array([0, 1, 0])
        x_body = np.cross(y_body, z_body)
        # Rotation matrix to quaternion
        R = np.column_stack([x_body, y_body, z_body])
        tr = R[0,0]+R[1,1]+R[2,2]
        if tr > 0:
            s = 0.5/math.sqrt(tr+1)
            qw = 0.25/s
            qx = (R[2,1]-R[1,2])*s
            qy = (R[0,2]-R[2,0])*s
            qz = (R[1,0]-R[0,1])*s
        else:
            qw,qx,qy,qz = 1.,0.,0.,0.
        att_msg = QuaternionStamped()
        att_msg.header.stamp = self.get_clock().now().to_msg()
        att_msg.header.frame_id = 'base_link'
        att_msg.quaternion.x, att_msg.quaternion.y = qx, qy
        att_msg.quaternion.z, att_msg.quaternion.w = qz, qw
        self.pub_att.publish(att_msg)
        t_msg = Float64()
        t_msg.data = float(thrust)
        self.pub_thrust.publish(t_msg)
        err_msg = Float64()
        err_msg.data = float(error)
        self.pub_err.publish(err_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrajectoryTrackerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
