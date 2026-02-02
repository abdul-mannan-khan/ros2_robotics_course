#!/usr/bin/env python3
"""Week 12 Exercise 1: Multi-Sensor Fusion Pipeline Node
Subscribe to /lidar_points, /camera/image, /imu, /gps.
Fuse sensor data: transform LiDAR to camera frame, overlay detections.
Publish /fused_perception (PointCloud2) and /sensor_status (String).

TODO: Coordinate transforms and sensor fusion.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from std_msgs.msg import String
import numpy as np
import json

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False


class MultiSensorFusionNode(Node):
    def __init__(self):
        super().__init__("multi_sensor_fusion_node")
        self.declare_parameter("camera_lidar_extrinsics",
            [1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0])
        self.declare_parameter("fusion_mode", "overlay")

        ext = self.get_parameter("camera_lidar_extrinsics").value
        self.T_cam_lidar = np.array(ext).reshape(4, 4)
        self.fusion_mode = self.get_parameter("fusion_mode").value

        self.lidar_sub = self.create_subscription(
            PointCloud2, "/lidar_points", self.lidar_callback, 10)
        self.image_sub = self.create_subscription(
            Image, "/camera/image", self.image_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, "/imu", self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps", self.gps_callback, 10)

        self.fused_pub = self.create_publisher(PointCloud2, "/fused_perception", 10)
        self.status_pub = self.create_publisher(String, "/sensor_status", 10)

        self.latest_image = None
        self.latest_lidar = None
        self.latest_imu = None
        self.latest_gps = None

        if HAS_CV:
            self.bridge = CvBridge()
        self.get_logger().info(f"Multi-Sensor Fusion: mode={self.fusion_mode}")

    def lidar_callback(self, msg):
        self.latest_lidar = msg
        self.try_fuse()

    def image_callback(self, msg):
        self.latest_image = msg

    def imu_callback(self, msg):
        self.latest_imu = msg

    def gps_callback(self, msg):
        self.latest_gps = msg

    def try_fuse(self):
        # ============================================================
        # TODO: Implement sensor fusion
        # 1. Transform LiDAR points to camera frame using T_cam_lidar
        #    pts_cam = T_cam_lidar @ pts_lidar_homogeneous
        # 2. Project LiDAR points onto image plane
        #    u = fx * X/Z + cx, v = fy * Y/Z + cy
        # 3. Color the point cloud using corresponding image pixels
        # 4. Publish the colored point cloud as /fused_perception
        # 5. Report sensor status (which sensors are active)
        # ============================================================

        # Publish sensor status
        status = {
            "lidar": self.latest_lidar is not None,
            "camera": self.latest_image is not None,
            "imu": self.latest_imu is not None,
            "gps": self.latest_gps is not None
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

        # Pass through lidar as placeholder
        if self.latest_lidar is not None:
            self.fused_pub.publish(self.latest_lidar)

        self.get_logger().info(f"Sensors: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
