#!/usr/bin/env python3
"""Week 10 Solution 1: Image Feature Detector Node (Complete Implementation)"""
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
        self.declare_parameter("feature_type", "ORB")
        self.declare_parameter("max_features", 500)
        self.declare_parameter("quality_threshold", 0.01)

        self.feature_type = self.get_parameter("feature_type").value
        self.max_features = self.get_parameter("max_features").value
        self.quality_threshold = self.get_parameter("quality_threshold").value

        self.image_sub = self.create_subscription(
            Image, "/camera/left/image_raw", self.image_callback, 10)
        self.features_pub = self.create_publisher(Image, "/features", 10)
        self.count_pub = self.create_publisher(Int32, "/feature_count", 10)

        if HAS_CV:
            self.bridge = CvBridge()
            if self.feature_type.upper() == "SIFT":
                self.detector = cv2.SIFT_create(nfeatures=self.max_features)
            else:
                self.detector = cv2.ORB_create(nfeatures=self.max_features)
        else:
            self.get_logger().error("OpenCV/cv_bridge not available!")
            self.detector = None

        self.get_logger().info(
            f"Feature Detector: type={self.feature_type}, max={self.max_features}")

    def image_callback(self, msg):
        if not HAS_CV or self.detector is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect keypoints and compute descriptors
        keypoints, descriptors = self.detector.detectAndCompute(gray, None)

        # Filter by quality/response if needed
        if keypoints and self.quality_threshold > 0:
            keypoints = [kp for kp in keypoints
                         if kp.response >= self.quality_threshold]
            keypoints = sorted(keypoints, key=lambda k: k.response, reverse=True)
            keypoints = keypoints[:self.max_features]

        # Draw keypoints
        annotated = cv2.drawKeypoints(
            cv_image, keypoints, None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Add text overlay
        cv2.putText(annotated, f"{self.feature_type}: {len(keypoints)} features",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # Publish feature count
        count_msg = Int32()
        count_msg.data = len(keypoints)
        self.count_pub.publish(count_msg)

        # Publish annotated image
        features_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        features_msg.header = msg.header
        self.features_pub.publish(features_msg)

        self.get_logger().info(f"Detected {len(keypoints)} {self.feature_type} features")


def main(args=None):
    rclpy.init(args=args)
    node = ImageFeatureDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
