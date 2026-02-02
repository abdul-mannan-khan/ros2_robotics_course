#!/usr/bin/env python3
"""Week 10 Exercise 1: Image Feature Detector Node
Subscribe to /camera/left/image_raw, detect ORB/SIFT features.
Publish /features (annotated Image) and /feature_count (Int32).

TODO: Implement feature detection with OpenCV.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False


class ImageFeatureDetectorNode(Node):
    def __init__(self):
        super().__init__("image_feature_detector_node")

        # Parameters
        self.declare_parameter("feature_type", "ORB")
        self.declare_parameter("max_features", 500)
        self.declare_parameter("quality_threshold", 0.01)

        self.feature_type = self.get_parameter("feature_type").value
        self.max_features = self.get_parameter("max_features").value
        self.quality_threshold = self.get_parameter("quality_threshold").value

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, "/camera/left/image_raw", self.image_callback, 10)

        # Publishers
        self.features_pub = self.create_publisher(Image, "/features", 10)
        self.count_pub = self.create_publisher(Int32, "/feature_count", 10)

        if HAS_CV:
            self.bridge = CvBridge()
        else:
            self.get_logger().error("OpenCV/cv_bridge not available!")

        self.get_logger().info(
            f"Feature Detector started: type={self.feature_type}, "
            f"max={self.max_features}")

    def image_callback(self, msg):
        if not HAS_CV:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # ============================================================
        # TODO: Implement feature detection
        # 1. Create a feature detector based on self.feature_type
        #    - If "ORB": use cv2.ORB_create(nfeatures=self.max_features)
        #    - If "SIFT": use cv2.SIFT_create(nfeatures=self.max_features)
        # 2. Detect keypoints and compute descriptors:
        #    keypoints, descriptors = detector.detectAndCompute(gray, None)
        # 3. Draw keypoints on cv_image:
        #    annotated = cv2.drawKeypoints(cv_image, keypoints, None,
        #        color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # 4. Publish the annotated image and feature count
        # ============================================================

        keypoints = []  # Replace with detected keypoints
        descriptors = None  # Replace with computed descriptors
        annotated = cv_image.copy()  # Replace with annotated image

        # Publish feature count
        count_msg = Int32()
        count_msg.data = len(keypoints)
        self.count_pub.publish(count_msg)

        # Publish annotated image
        features_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        features_msg.header = msg.header
        self.features_pub.publish(features_msg)

        self.get_logger().info(f"Detected {len(keypoints)} features")


def main(args=None):
    rclpy.init(args=args)
    node = ImageFeatureDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
