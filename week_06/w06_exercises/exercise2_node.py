#!/usr/bin/env python3
"""Exercise 2: Thrust Mixer Node.
Subscribe to /motor_commands (Float32MultiArray, 4 motors),
compute net thrust and torques, publish /body_wrench (Wrench)."""
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Wrench

class ThrustMixerNode(Node):
    def __init__(self):
        super().__init__("thrust_mixer_node")
        self.declare_parameter("arm_length", 0.17)
        self.declare_parameter("k_thrust", 1.0e-5)
        self.declare_parameter("k_torque", 1.0e-7)
        self.L = self.get_parameter("arm_length").value
        self.kt = self.get_parameter("k_thrust").value
        self.kq = self.get_parameter("k_torque").value
        self.sub = self.create_subscription(Float32MultiArray, "/motor_commands", self.motor_cb, 10)
        self.pub = self.create_publisher(Wrench, "/body_wrench", 10)
        self.get_logger().info("ThrustMixerNode started")

    def motor_cb(self, msg):
        if len(msg.data) != 4: return
        w = np.array(msg.data)
        # TODO: Compute mixing matrix and body wrench
        # Motor layout (X config): front-right(0), front-left(1), back-left(2), back-right(3)
        # Thrust per motor: F_i = k_thrust * w_i^2
        # Torque per motor: tau_i = k_torque * w_i^2
        # Total thrust (z): sum of all F_i
        # Roll torque (x): L * (F1 + F2 - F0 - F3) / sqrt(2)
        # Pitch torque (y): L * (F0 + F1 - F2 - F3) / sqrt(2)
        # Yaw torque (z): -tau0 + tau1 - tau2 + tau3
        # Publish as Wrench message
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ThrustMixerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
