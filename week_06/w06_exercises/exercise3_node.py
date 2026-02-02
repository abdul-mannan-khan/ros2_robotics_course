#!/usr/bin/env python3
"""Exercise 3: Attitude Controller Node (PD on SO(3)).
Subscribe to /estimated_attitude and /desired_attitude,
publish /motor_commands (Float32MultiArray)."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import Float32MultiArray

class AttitudeControllerNode(Node):
    def __init__(self):
        super().__init__("attitude_controller_node")
        self.declare_parameter("kp_roll", 5.0)
        self.declare_parameter("kd_roll", 0.5)
        self.declare_parameter("kp_pitch", 5.0)
        self.declare_parameter("kd_pitch", 0.5)
        self.declare_parameter("kp_yaw", 2.0)
        self.declare_parameter("kd_yaw", 0.3)
        self.kp = np.array([self.get_parameter("kp_roll").value, self.get_parameter("kp_pitch").value, self.get_parameter("kp_yaw").value])
        self.kd = np.array([self.get_parameter("kd_roll").value, self.get_parameter("kd_pitch").value, self.get_parameter("kd_yaw").value])
        self.est_sub = self.create_subscription(QuaternionStamped, "/estimated_attitude", self.est_cb, 10)
        self.des_sub = self.create_subscription(QuaternionStamped, "/desired_attitude", self.des_cb, 10)
        self.cmd_pub = self.create_publisher(Float32MultiArray, "/motor_commands", 10)
        self.est_q = None; self.des_q = np.array([0,0,0,1.0])
        self.prev_err = np.zeros(3); self.last_t = None
        self.get_logger().info("AttitudeControllerNode started")

    def quat_to_rot(self, q):
        x,y,z,w = q
        return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

    def est_cb(self, msg):
        q = msg.quaternion
        self.est_q = np.array([q.x, q.y, q.z, q.w])

    def des_cb(self, msg):
        q = msg.quaternion
        self.des_q = np.array([q.x, q.y, q.z, q.w])
        self.compute_control(msg.header.stamp)

    def compute_control(self, stamp):
        """TODO: PD controller on SO(3).
        1. Compute rotation error: R_err = R_des.T @ R_est
        2. Extract error angles (vee map of skew-symmetric part)
        3. PD: torque = -kp * e_r - kd * e_r_dot
        4. Convert torques to motor commands via inverse mixing matrix
        5. Publish motor_commands"""
        if self.est_q is None: return
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AttitudeControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
