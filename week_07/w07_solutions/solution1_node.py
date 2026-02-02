#!/usr/bin/env python3
"""Week 7 Solution 1: PX4 Telemetry Monitor Node (Complete)"""
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
            summary['position'] = {'x': round(pos.x,3), 'y': round(pos.y,3), 'z': round(pos.z,3)}
            summary['velocity'] = {'vx': round(vel.x,3), 'vy': round(vel.y,3), 'vz': round(vel.z,3)}
            summary['speed'] = round((vel.x**2+vel.y**2+vel.z**2)**0.5, 3)
            summary['altitude'] = round(-pos.z, 3)
        if self.latest_status is not None:
            try:
                sd = json.loads(self.latest_status.data)
                summary['armed'] = sd.get('armed', False)
                summary['flight_mode'] = sd.get('flight_mode', 'UNKNOWN')
                summary['battery_pct'] = sd.get('battery_pct', -1)
                summary['gps_fix'] = sd.get('gps_fix', 0)
                self.get_logger().info(
                    f"Mode: {summary['flight_mode']}, Armed: {summary['armed']}, "
                    f"Alt: {summary.get('altitude','N/A')}m, Batt: {summary['battery_pct']:.1f}%")
            except (json.JSONDecodeError, KeyError) as e:
                self.get_logger().warn(f'Failed to parse status: {e}')
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
