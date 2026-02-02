#!/usr/bin/env python3
"""Week 12 Solution 1: Multi-Sensor Fusion Pipeline (Complete)"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, Imu, NavSatFix
from std_msgs.msg import String, Header
import numpy as np
import json
import struct

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
        self.declare_parameter("fx", 500.0)
        self.declare_parameter("fy", 500.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)

        ext = self.get_parameter("camera_lidar_extrinsics").value
        self.T_cam_lidar = np.array(ext).reshape(4, 4)
        self.fx = self.get_parameter("fx").value
        self.fy = self.get_parameter("fy").value
        self.cx = self.get_parameter("cx").value
        self.cy = self.get_parameter("cy").value

        self.lidar_sub = self.create_subscription(PointCloud2, "/lidar_points", self.lidar_cb, 10)
        self.image_sub = self.create_subscription(Image, "/camera/image", self.image_cb, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_cb, 10)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps", self.gps_cb, 10)

        self.fused_pub = self.create_publisher(PointCloud2, "/fused_perception", 10)
        self.status_pub = self.create_publisher(String, "/sensor_status", 10)

        self.latest_image = None
        self.latest_lidar = None
        self.latest_imu = None
        self.latest_gps = None
        if HAS_CV:
            self.bridge = CvBridge()
        self.get_logger().info("Multi-Sensor Fusion started")

    def lidar_cb(self, msg):
        self.latest_lidar = msg
        self.fuse()

    def image_cb(self, msg):
        self.latest_image = msg

    def imu_cb(self, msg):
        self.latest_imu = msg

    def gps_cb(self, msg):
        self.latest_gps = msg

    def pointcloud2_to_xyz(self, msg):
        n = msg.width * msg.height
        data = np.frombuffer(bytes(msg.data), dtype=np.float32)
        step = msg.point_step // 4
        if step >= 3:
            data = data.reshape(-1, step)
            return data[:, :3].copy()
        return np.zeros((0, 3), dtype=np.float32)

    def fuse(self):
        if self.latest_lidar is None:
            return

        pts = self.pointcloud2_to_xyz(self.latest_lidar)
        n = len(pts)

        # Transform to camera frame
        pts_h = np.hstack([pts, np.ones((n, 1))])
        pts_cam = (self.T_cam_lidar @ pts_h.T).T[:, :3]

        # Project onto image and color
        colors = np.full((n, 3), 128, dtype=np.uint8)  # default gray

        if self.latest_image is not None and HAS_CV:
            img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            h, w = img.shape[:2]
            valid = pts_cam[:, 2] > 0.1
            u = (self.fx * pts_cam[:, 0] / pts_cam[:, 2] + self.cx).astype(int)
            v = (self.fy * pts_cam[:, 1] / pts_cam[:, 2] + self.cy).astype(int)
            in_bounds = valid & (u >= 0) & (u < w) & (v >= 0) & (v < h)
            colors[in_bounds] = img[v[in_bounds], u[in_bounds]]

        # Build colored point cloud
        fused = PointCloud2()
        fused.header = self.latest_lidar.header
        fused.height = 1
        fused.width = n
        fused.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        fused.point_step = 16
        fused.row_step = 16 * n
        fused.is_dense = True

        buf = bytearray(16 * n)
        for i in range(n):
            struct.pack_into("fff", buf, i*16, pts[i,0], pts[i,1], pts[i,2])
            rgb_int = (int(colors[i,2]) << 16) | (int(colors[i,1]) << 8) | int(colors[i,0])
            struct.pack_into("f", buf, i*16+12, struct.unpack("f", struct.pack("I", rgb_int))[0])
        fused.data = bytes(buf)
        self.fused_pub.publish(fused)

        status = json.dumps({
            "lidar": True, "camera": self.latest_image is not None,
            "imu": self.latest_imu is not None, "gps": self.latest_gps is not None,
            "fused_points": n})
        self.status_pub.publish(String(data=status))
        self.get_logger().info(f"Fused {n} points")


def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
