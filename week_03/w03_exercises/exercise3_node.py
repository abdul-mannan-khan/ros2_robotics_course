#!/usr/bin/env python3
"""
Week 3 - Exercise 3: Reactive Controller Node
Subscribe to /obstacle_sectors, implement simple reactive control.
Publish /cmd_vel (Twist).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class ReactiveControllerNode(Node):
    def __init__(self):
        super().__init__("reactive_controller_node")

        # TODO 1: Declare parameters
        # self.declare_parameter("min_distance", 0.3)
        # self.declare_parameter("linear_speed", 0.2)
        # self.declare_parameter("angular_speed", 0.5)
        self.min_distance = 0.3
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        # TODO 2: Subscribe to /obstacle_sectors
        # self.sectors_sub = self.create_subscription(
        #     Float32MultiArray, "/obstacle_sectors", self.sectors_cb, 10)

        # TODO 3: Create publisher for /cmd_vel
        # self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Reactive Controller started")

    def sectors_cb(self, msg: Float32MultiArray):
        sectors = msg.data  # [front, left, back, right]

        # TODO 4: Implement reactive control logic
        # cmd = Twist()
        #
        # if len(sectors) < 4:
        #     return
        #
        # front = sectors[0]
        # left = sectors[1]
        # right = sectors[3]
        #
        # if front < self.min_distance:
        #     # Too close in front - stop and turn
        #     cmd.linear.x = 0.0
        #     if left > right:
        #         cmd.angular.z = self.angular_speed  # Turn left
        #     else:
        #         cmd.angular.z = -self.angular_speed  # Turn right
        # elif front < self.min_distance * 3:
        #     # Getting close - slow down and steer
        #     cmd.linear.x = self.linear_speed * 0.5
        #     if left > right:
        #         cmd.angular.z = self.angular_speed * 0.5
        #     else:
        #         cmd.angular.z = -self.angular_speed * 0.5
        # else:
        #     # Clear path - go forward
        #     cmd.linear.x = self.linear_speed
        #     cmd.angular.z = 0.0
        #
        # self.cmd_pub.publish(cmd)
        # self.get_logger().info(f"cmd: lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f}")

        pass


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
