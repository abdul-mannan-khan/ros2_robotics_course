#!/usr/bin/env python3
"""Week 3 - Solution 3: Reactive Controller Node (Complete Implementation)"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class ReactiveControllerNode(Node):
    def __init__(self):
        super().__init__("reactive_controller_node")
        self.declare_parameter("min_distance", 0.3)
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.5)
        self.min_distance = self.get_parameter("min_distance").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.sectors_sub = self.create_subscription(
            Float32MultiArray, "/obstacle_sectors", self.sectors_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("Reactive Controller started")

    def sectors_cb(self, msg: Float32MultiArray):
        sectors = msg.data
        if len(sectors) < 4:
            return
        front, left, back, right = sectors[0], sectors[1], sectors[2], sectors[3]
        cmd = Twist()
        if front < self.min_distance:
            cmd.linear.x = 0.0
            if left > right:
                cmd.angular.z = self.angular_speed
            else:
                cmd.angular.z = -self.angular_speed
        elif front < self.min_distance * 3:
            cmd.linear.x = self.linear_speed * 0.5
            if left > right:
                cmd.angular.z = self.angular_speed * 0.5
            else:
                cmd.angular.z = -self.angular_speed * 0.5
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"cmd: lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
