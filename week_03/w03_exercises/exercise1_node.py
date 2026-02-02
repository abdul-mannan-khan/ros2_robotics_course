#!/usr/bin/env python3
"""
Week 3 - Exercise 1: Multi-Topic Listener Node
Subscribe to /scan, /odom, /imu simultaneously.
Log rates of each topic, publish /topic_rates (String).
"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class MultiTopicListenerNode(Node):
    def __init__(self):
        super().__init__("multi_topic_listener_node")
        self.get_logger().info("Multi-Topic Listener started")

        # TODO 1: Create subscribers for /scan, /odom, /imu
        # self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        # self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        # self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_cb, 10)

        # TODO 2: Create publisher for /topic_rates (String)
        # self.rates_pub = self.create_publisher(String, "/topic_rates", 10)

        # Counters for rate calculation
        self.counts = {"scan": 0, "odom": 0, "imu": 0}
        self.last_report = time.time()

        # TODO 3: Create a timer to publish rates every 1 second
        # self.timer = self.create_timer(1.0, self.report_rates)

    def scan_cb(self, msg):
        # TODO 4: Increment scan counter
        # self.counts["scan"] += 1
        pass

    def odom_cb(self, msg):
        # TODO 5: Increment odom counter
        # self.counts["odom"] += 1
        pass

    def imu_cb(self, msg):
        # TODO 6: Increment imu counter
        # self.counts["imu"] += 1
        pass

    def report_rates(self):
        # TODO 7: Calculate and publish rates
        # now = time.time()
        # dt = now - self.last_report
        # if dt > 0:
        #     rates = {k: v/dt for k,v in self.counts.items()}
        #     msg = String()
        #     msg.data = f"scan={rates['scan']:.1f}Hz odom={rates['odom']:.1f}Hz imu={rates['imu']:.1f}Hz"
        #     self.rates_pub.publish(msg)
        #     self.get_logger().info(msg.data)
        # self.counts = {k: 0 for k in self.counts}
        # self.last_report = now
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
