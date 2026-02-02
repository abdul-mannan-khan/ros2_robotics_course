#!/usr/bin/env python3
"""Solution 2: Thrust Mixer - Full mixing matrix implementation."""
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Wrench

class ThrustMixerSolution(Node):
    def __init__(self):
        super().__init__("thrust_mixer_solution")
        self.declare_parameter("arm_length", 0.17)
        self.declare_parameter("k_thrust", 1.0e-5)
        self.declare_parameter("k_torque", 1.0e-7)
        self.L = self.get_parameter("arm_length").value
        self.kt = self.get_parameter("k_thrust").value
        self.kq = self.get_parameter("k_torque").value
        self.sub = self.create_subscription(Float32MultiArray, "/motor_commands", self.motor_cb, 10)
        self.pub = self.create_publisher(Wrench, "/body_wrench", 10)
        s2 = np.sqrt(2) / 2
        self.mix = np.array([[self.kt, self.kt, self.kt, self.kt],
                             [self.L*self.kt*s2, self.L*self.kt*s2, -self.L*self.kt*s2, -self.L*self.kt*s2],
                             [self.L*self.kt*s2, -self.L*self.kt*s2, -self.L*self.kt*s2, self.L*self.kt*s2],
                             [-self.kq, self.kq, -self.kq, self.kq]])
        self.get_logger().info("ThrustMixerSolution started")

    def motor_cb(self, msg):
        if len(msg.data) != 4: return
        w2 = np.array(msg.data) ** 2
        r = self.mix @ w2
        wr = Wrench()
        wr.force.z = float(r[0])
        wr.torque.x = float(r[1])
        wr.torque.y = float(r[2])
        wr.torque.z = float(r[3])
        self.pub.publish(wr)

def main(args=None):
    rclpy.init(args=args)
    node = ThrustMixerSolution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
