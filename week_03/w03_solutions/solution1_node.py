#!/usr/bin/env python3
"""Week 3 - Solution 1: Multi-Topic Listener (Complete Implementation)"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class MultiTopicListenerNode(Node):
    def __init__(self):
        super().__init__("multi_topic_listener_node")
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_cb, 10)
        self.rates_pub = self.create_publisher(String, "/topic_rates", 10)
        self.counts = {"scan": 0, "odom": 0, "imu": 0}
        self.last_report = time.time()
        self.timer = self.create_timer(1.0, self.report_rates)
        self.get_logger().info("Multi-Topic Listener started")

    def scan_cb(self, msg): self.counts["scan"] += 1
    def odom_cb(self, msg): self.counts["odom"] += 1
    def imu_cb(self, msg): self.counts["imu"] += 1

    def report_rates(self):
        now = time.time()
        dt = now - self.last_report
        if dt > 0:
            rates = {k: v/dt for k,v in self.counts.items()}
            msg = String()
            msg.data = " ".join(f"{k}={rates[k]:.1f}Hz" for k in self.counts)
            self.rates_pub.publish(msg)
            self.get_logger().info(msg.data)
        self.counts = {k: 0 for k in self.counts}
        self.last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
