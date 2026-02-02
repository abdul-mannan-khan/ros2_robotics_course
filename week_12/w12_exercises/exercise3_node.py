#!/usr/bin/env python3
"""Week 12 Exercise 3: System Monitor and Logger Node
Subscribe to ALL major topics. Monitor rates, latencies, data quality.
Publish /system_health (String JSON) and /diagnostics (DiagnosticArray).

TODO: Rate monitoring, latency computation, health report generation.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import json
import time


class TopicMonitor:
    def __init__(self, topic_name, expected_rate):
        self.topic_name = topic_name
        self.expected_rate = expected_rate
        self.timestamps = []
        self.last_msg_time = None
        self.msg_count = 0

    def record(self):
        now = time.time()
        self.timestamps.append(now)
        self.last_msg_time = now
        self.msg_count += 1
        # Keep only last 100 timestamps
        if len(self.timestamps) > 100:
            self.timestamps = self.timestamps[-100:]

    def get_rate(self):
        # ============================================================
        # TODO: Compute actual message rate
        # 1. Use timestamps list to compute average rate
        # 2. Rate = (n-1) / (t_last - t_first) if n >= 2
        # 3. Return 0.0 if insufficient data
        # ============================================================
        return 0.0

    def get_latency(self):
        # ============================================================
        # TODO: Compute latency since last message
        # latency = current_time - last_msg_time
        # ============================================================
        return 0.0

    def is_healthy(self, max_latency=2.0):
        # ============================================================
        # TODO: Determine if topic is healthy
        # Healthy if: rate > 0.5 * expected_rate AND latency < max_latency
        # ============================================================
        return True


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

        # Subscribers
        self.create_subscription(PointCloud2, "/lidar_points", lambda m: self.record("lidar"), 10)
        self.create_subscription(Image, "/camera/image", lambda m: self.record("camera"), 10)
        self.create_subscription(Imu, "/imu", lambda m: self.record("imu"), 10)
        self.create_subscription(Odometry, "/odom", lambda m: self.record("odom"), 10)

        self.health_pub = self.create_publisher(String, "/system_health", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.timer = self.create_timer(1.0, self.publish_health)
        self.get_logger().info("System Monitor started")

    def record(self, name):
        if name in self.monitors:
            self.monitors[name].record()

    def publish_health(self):
        # ============================================================
        # TODO: Generate health report
        # 1. For each monitor, get rate, latency, healthy status
        # 2. Build JSON health report
        # 3. Build DiagnosticArray with status per topic
        # 4. Write to log file
        # ============================================================

        health = {}
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        for name, mon in self.monitors.items():
            rate = mon.get_rate()
            latency = mon.get_latency()
            healthy = mon.is_healthy(self.max_latency)
            health[name] = {
                "rate": round(rate, 2),
                "expected_rate": mon.expected_rate,
                "latency": round(latency, 4),
                "msg_count": mon.msg_count,
                "healthy": healthy
            }

            status = DiagnosticStatus()
            status.name = f"sensor/{name}"
            status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
            status.message = f"Rate: {rate:.1f} Hz"
            status.values = [
                KeyValue(key="rate", value=str(round(rate, 2))),
                KeyValue(key="latency", value=str(round(latency, 4))),
                KeyValue(key="count", value=str(mon.msg_count)),
            ]
            diag_array.status.append(status)

        health_msg = String()
        health_msg.data = json.dumps(health, indent=2)
        self.health_pub.publish(health_msg)
        self.diag_pub.publish(diag_array)

        all_healthy = all(h["healthy"] for h in health.values())
        self.get_logger().info(f"System: {'OK' if all_healthy else 'WARN'}")


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
