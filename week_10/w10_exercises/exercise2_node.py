#!/usr/bin/env python3
"""Week 10 Exercise 2: Stereo Depth Estimator Node
Subscribe to /camera/left/image_raw and /camera/right/image_raw.
Compute disparity map and depth image.
Publish /disparity (Image) and /depth_image (Image).

TODO: Implement stereo matching algorithm.
"""
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

        # Parameters
        self.declare_parameter("baseline", 0.54)  # meters (KITTI baseline)
        self.declare_parameter("focal_length", 721.5)  # pixels
        self.declare_parameter("num_disparities", 128)
        self.declare_parameter("block_size", 11)

        self.baseline = self.get_parameter("baseline").value
        self.focal_length = self.get_parameter("focal_length").value
        self.num_disparities = self.get_parameter("num_disparities").value
        self.block_size = self.get_parameter("block_size").value

        # Synchronized subscribers
        self.left_sub = message_filters.Subscriber(
            self, Image, "/camera/left/image_raw")
        self.right_sub = message_filters.Subscriber(
            self, Image, "/camera/right/image_raw")
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.stereo_callback)

        # Publishers
        self.disparity_pub = self.create_publisher(Image, "/disparity", 10)
        self.depth_pub = self.create_publisher(Image, "/depth_image", 10)

        if HAS_CV:
            self.bridge = CvBridge()
        else:
            self.get_logger().error("OpenCV/cv_bridge not available!")

        self.get_logger().info(
            f"Stereo Depth Estimator: baseline={self.baseline}m, "
            f"focal={self.focal_length}px")

    def stereo_callback(self, left_msg, right_msg):
        if not HAS_CV:
            return

        left_img = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
        right_img = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # ============================================================
        # TODO: Implement stereo matching
        # 1. Create StereoBM or StereoSGBM matcher:
        #    stereo = cv2.StereoSGBM_create(
        #        minDisparity=0,
        #        numDisparities=self.num_disparities,
        #        blockSize=self.block_size,
        #        P1=8 * 3 * self.block_size ** 2,
        #        P2=32 * 3 * self.block_size ** 2,
        #        disp12MaxDiff=1,
        #        uniquenessRatio=10,
        #        speckleWindowSize=100,
        #        speckleRange=32)
        # 2. Compute disparity:
        #    disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0
        # 3. Compute depth from disparity:
        #    depth = (self.baseline * self.focal_length) / (disparity + 1e-6)
        #    depth[disparity <= 0] = 0.0
        # 4. Normalize disparity for visualization (0-255)
        # 5. Publish disparity and depth images
        # ============================================================

        h, w = left_gray.shape
        disparity = np.zeros((h, w), dtype=np.float32)  # Replace
        depth = np.zeros((h, w), dtype=np.float32)  # Replace

        # Publish disparity as normalized image
        disp_vis = np.zeros_like(disparity, dtype=np.uint8)  # Replace with normalized
        disp_msg = self.bridge.cv2_to_imgmsg(disp_vis, encoding="mono8")
        disp_msg.header = left_msg.header
        self.disparity_pub.publish(disp_msg)

        # Publish depth image (32FC1)
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header = left_msg.header
        self.depth_pub.publish(depth_msg)

        self.get_logger().info(f"Disparity range: [{disparity.min():.1f}, {disparity.max():.1f}]")


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
