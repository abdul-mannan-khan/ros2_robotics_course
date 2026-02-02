#!/usr/bin/env python3
"""
Week 7 Exercise 1: PX4 Telemetry Monitor Node

Subscribe to PX4 vehicle odometry and status topics,
parse and log telemetry, publish a summary string.

Subscribers:
  /fmu/out/vehicle_odometry (Odometry)
  /fmu/out/vehicle_status   (String - JSON encoded)
Publishers:
  /telemetry_summary (String)
Parameters:
  update_rate_hz (double, default 2.0)
"""
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class PX4TelemetryMonitorNode(Node):
    def __init__(self):
        super().__init__('px4_telemetry_monitor')
        self.declare_parameter('update_rate_hz', 2.0)
        rate = self.get_parameter('update_rate_hz').value

        self.latest_odom = None
        self.latest_status = None

        self.sub_odom = self.create_subscription(
            Odometry, '/fmu/out/vehicle_odometry', self.odom_cb, 10)
        self.sub_status = self.create_subscription(
            String, '/fmu/out/vehicle_status', self.status_cb, 10)
        self.pub_summary = self.create_publisher(String, '/telemetry_summary', 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_cb)
        self.get_logger().info('PX4 Telemetry Monitor started')

    def odom_cb(self, msg):
        self.latest_odom = msg

    def status_cb(self, msg):
        self.latest_status = msg

    def timer_cb(self):
        summary = {}

        if self.latest_odom is not None:
            pos = self.latest_odom.pose.pose.position
            vel = self.latest_odom.twist.twist.linear
            summary['position'] = {'x': pos.x, 'y': pos.y, 'z': pos.z}
            summary['velocity'] = {'vx': vel.x, 'vy': vel.y, 'vz': vel.z}

        if self.latest_status is not None:
            # TODO: Parse telemetry fields from JSON status string
            # 1. Parse JSON: status_data = json.loads(self.latest_status.data)
            # 2. Extract fields: armed, flight_mode, battery_pct
            # 3. Add to summary dict
            # 4. Log: self.get_logger().info(f"Mode: {flight_mode}, Armed: {armed}")
            pass

        msg = String()
        msg.data = json.dumps(summary)
        self.pub_summary.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4TelemetryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
