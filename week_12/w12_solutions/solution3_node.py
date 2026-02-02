#!/usr/bin/env python3
"""Week 12 Solution 3: System Monitor and Logger (Complete)"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import json
import time


class TopicMonitor:
    def __init__(self, name, expected_rate):
        self.name = name
        self.expected_rate = expected_rate
        self.timestamps = []
        self.last_msg_time = None
        self.msg_count = 0

    def record(self):
        now = time.time()
        self.timestamps.append(now)
        self.last_msg_time = now
        self.msg_count += 1
        if len(self.timestamps) > 200:
            self.timestamps = self.timestamps[-200:]

    def get_rate(self):
        if len(self.timestamps) < 2:
            return 0.0
        dt = self.timestamps[-1] - self.timestamps[0]
        if dt <= 0:
            return 0.0
        return (len(self.timestamps) - 1) / dt

    def get_latency(self):
        if self.last_msg_time is None:
            return float("inf")
        return time.time() - self.last_msg_time

    def is_healthy(self, max_latency=2.0):
        rate = self.get_rate()
        latency = self.get_latency()
        return rate > 0.5 * self.expected_rate and latency < max_latency


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__("system_monitor_node")
        self.declare_parameter("expected_rates",
            ["lidar:10.0", "camera:10.0", "imu:50.0", "odom:50.0"])
        self.declare_parameter("max_latency", 2.0)
        self.declare_parameter("log_file_path", "/tmp/system_health.log")

        rates_strs = self.get_parameter("expected_rates").value
        self.max_latency = self.get_parameter("max_latency").value
        self.log_path = self.get_parameter("log_file_path").value

        self.monitors = {}
        for rs in rates_strs:
            name, rate = rs.split(":")
            self.monitors[name] = TopicMonitor(name, float(rate))

        self.create_subscription(PointCloud2, "/lidar_points", lambda m: self.rec("lidar"), 10)
        self.create_subscription(Image, "/camera/image", lambda m: self.rec("camera"), 10)
        self.create_subscription(Imu, "/imu", lambda m: self.rec("imu"), 10)
        self.create_subscription(Odometry, "/odom", lambda m: self.rec("odom"), 10)

        self.health_pub = self.create_publisher(String, "/system_health", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.timer = self.create_timer(1.0, self.publish_health)
        self.log_file = None
        try:
            self.log_file = open(self.log_path, "a")
        except Exception as e:
            self.get_logger().warn(f"Cannot open log: {e}")
        self.get_logger().info("System Monitor started")

    def rec(self, name):
        if name in self.monitors:
            self.monitors[name].record()

    def publish_health(self):
        health = {}
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        for name, mon in self.monitors.items():
            rate = mon.get_rate()
            latency = mon.get_latency()
            healthy = mon.is_healthy(self.max_latency)
            health[name] = {
                "rate": round(rate, 2),
                "expected": mon.expected_rate,
                "latency": round(latency, 4),
                "count": mon.msg_count,
                "healthy": healthy
            }
            st = DiagnosticStatus()
            st.name = f"sensor/{name}"
            st.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
            st.message = f"{rate:.1f} Hz" + ("" if healthy else " [SLOW]")
            st.values = [
                KeyValue(key="rate", value=f"{rate:.2f}"),
                KeyValue(key="expected", value=f"{mon.expected_rate:.1f}"),
                KeyValue(key="latency", value=f"{latency:.4f}"),
                KeyValue(key="count", value=str(mon.msg_count)),
            ]
            diag.status.append(st)

        self.health_pub.publish(String(data=json.dumps(health, indent=2)))
        self.diag_pub.publish(diag)

        if self.log_file:
            self.log_file.write(json.dumps(health) + chr(10))
            self.log_file.flush()

        ok = all(h["healthy"] for h in health.values())
        self.get_logger().info(f"Health: {'ALL OK' if ok else 'WARNING'}")

    def destroy_node(self):
        if self.log_file:
            self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
