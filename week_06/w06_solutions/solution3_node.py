#!/usr/bin/env python3
"""Solution 3: Attitude Controller - PD on SO(3)."""
import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import Float32MultiArray

class AttitudeControllerSolution(Node):
    def __init__(self):
        super().__init__("attitude_controller_solution")
        self.declare_parameter("kp_roll", 5.0)
        self.declare_parameter("kd_roll", 0.5)
        self.declare_parameter("kp_pitch", 5.0)
        self.declare_parameter("kd_pitch", 0.5)
        self.declare_parameter("kp_yaw", 2.0)
        self.declare_parameter("kd_yaw", 0.3)
        self.declare_parameter("arm_length", 0.17)
        self.declare_parameter("k_thrust", 1.0e-5)
        self.declare_parameter("k_torque", 1.0e-7)
        self.declare_parameter("hover_speed", 500.0)
        self.kp = np.array([self.get_parameter(k).value for k in ["kp_roll","kp_pitch","kp_yaw"]])
        self.kd = np.array([self.get_parameter(k).value for k in ["kd_roll","kd_pitch","kd_yaw"]])
        L = self.get_parameter("arm_length").value
        kt = self.get_parameter("k_thrust").value
        kq = self.get_parameter("k_torque").value
        self.hover = self.get_parameter("hover_speed").value
        s2 = np.sqrt(2)/2
        mix = np.array([[kt,kt,kt,kt],[L*kt*s2,L*kt*s2,-L*kt*s2,-L*kt*s2],[L*kt*s2,-L*kt*s2,-L*kt*s2,L*kt*s2],[-kq,kq,-kq,kq]])
        self.mix_inv = np.linalg.pinv(mix)
        self.est_sub = self.create_subscription(QuaternionStamped, "/estimated_attitude", self.est_cb, 10)
        self.des_sub = self.create_subscription(QuaternionStamped, "/desired_attitude", self.des_cb, 10)
        self.cmd_pub = self.create_publisher(Float32MultiArray, "/motor_commands", 10)
        self.est_q = None; self.des_q = np.array([0,0,0,1.0])
        self.prev_err = np.zeros(3); self.last_t = None
        self.get_logger().info("AttitudeControllerSolution started")

    def q2R(self, q):
        x,y,z,w = q
        return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

    def vee(self, S):
        return np.array([S[2,1], S[0,2], S[1,0]])

    def est_cb(self, msg):
        q = msg.quaternion
        self.est_q = np.array([q.x, q.y, q.z, q.w])
        self.compute_control(msg.header.stamp)

    def des_cb(self, msg):
        q = msg.quaternion
        self.des_q = np.array([q.x, q.y, q.z, q.w])

    def compute_control(self, stamp):
        if self.est_q is None: return
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t; return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0 or dt > 0.1: return
        R_est = self.q2R(self.est_q)
        R_des = self.q2R(self.des_q)
        R_err = R_des.T @ R_est
        e_R = 0.5 * self.vee(R_err.T - R_err)
        e_R_dot = (e_R - self.prev_err) / dt
        self.prev_err = e_R
        torques = -self.kp * e_R - self.kd * e_R_dot
        # Hover thrust + attitude torques
        thrust = 9.81 * 0.5  # approx hover
        wrench = np.array([thrust, torques[0], torques[1], torques[2]])
        w2 = self.mix_inv @ wrench
        w2 = np.clip(w2, 0, None)
        speeds = np.sqrt(w2)
        mc = Float32MultiArray()
        mc.data = [float(s) for s in speeds]
        self.cmd_pub.publish(mc)

def main(args=None):
    rclpy.init(args=args)
    node = AttitudeControllerSolution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
