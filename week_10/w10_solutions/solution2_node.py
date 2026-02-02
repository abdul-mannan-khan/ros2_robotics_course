#!/usr/bin/env python3
"""Week 10 Solution 2: Stereo Depth Estimator Node (Complete Implementation)"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import message_filters

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False


class StereoDepthEstimatorNode(Node):
    def __init__(self):
        super().__init__("stereo_depth_estimator_node")
        self.declare_parameter("baseline", 0.54)
        self.declare_parameter("focal_length", 721.5)
        self.declare_parameter("num_disparities", 128)
        self.declare_parameter("block_size", 11)

        self.baseline = self.get_parameter("baseline").value
        self.focal_length = self.get_parameter("focal_length").value
        self.num_disparities = self.get_parameter("num_disparities").value
        self.block_size = self.get_parameter("block_size").value

        self.left_sub = message_filters.Subscriber(self, Image, "/camera/left/image_raw")
        self.right_sub = message_filters.Subscriber(self, Image, "/camera/right/image_raw")
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.stereo_callback)

        self.disparity_pub = self.create_publisher(Image, "/disparity", 10)
        self.depth_pub = self.create_publisher(Image, "/depth_image", 10)

        if HAS_CV:
            self.bridge = CvBridge()
            # Create StereoSGBM matcher
            self.stereo = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=self.num_disparities,
                blockSize=self.block_size,
                P1=8 * 3 * self.block_size ** 2,
                P2=32 * 3 * self.block_size ** 2,
                disp12MaxDiff=1,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
        else:
            self.get_logger().error("OpenCV not available!")

        self.get_logger().info(
            f"Stereo Depth: baseline={self.baseline}m, focal={self.focal_length}px, "
            f"disparities={self.num_disparities}, block={self.block_size}")

    def stereo_callback(self, left_msg, right_msg):
        if not HAS_CV:
            return

        left_img = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
        right_img = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # Compute disparity map
        raw_disparity = self.stereo.compute(left_gray, right_gray)
        disparity = raw_disparity.astype(np.float32) / 16.0

        # Compute depth: Z = baseline * focal_length / disparity
        depth = np.zeros_like(disparity, dtype=np.float32)
        valid = disparity > 0
        depth[valid] = (self.baseline * self.focal_length) / disparity[valid]
        # Clamp depth to reasonable range
        depth = np.clip(depth, 0.0, 100.0)

        # Normalize disparity for visualization
        disp_vis = disparity.copy()
        disp_vis[disp_vis < 0] = 0
        if disp_vis.max() > 0:
            disp_vis = (disp_vis / disp_vis.max() * 255.0).astype(np.uint8)
        else:
            disp_vis = np.zeros_like(disp_vis, dtype=np.uint8)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

        # Publish disparity
        disp_msg = self.bridge.cv2_to_imgmsg(disp_color, "bgr8")
        disp_msg.header = left_msg.header
        self.disparity_pub.publish(disp_msg)

        # Publish depth
        depth_msg = self.bridge.cv2_to_imgmsg(depth, "32FC1")
        depth_msg.header = left_msg.header
        self.depth_pub.publish(depth_msg)

        self.get_logger().info(
            f"Disparity: [{disparity[valid].min():.1f}, {disparity[valid].max():.1f}], "
            f"Depth: [{depth[valid].min():.1f}m, {depth[valid].max():.1f}m]")


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
