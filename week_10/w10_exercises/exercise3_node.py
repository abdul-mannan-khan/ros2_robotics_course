#!/usr/bin/env python3
"""Week 10 Exercise 3: 2D Object Tracker Node
Subscribe to /camera/left/image_raw.
Simple multi-object tracker: detect via color/contour, track with Kalman filter.
Publish /tracked_objects (MarkerArray) and /tracking_image (Image).

TODO: Implement Kalman filter prediction/update and Hungarian assignment.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
import numpy as np

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False

try:
    from scipy.optimize import linear_sum_assignment
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class TrackedObject:
    """Represents a tracked object with Kalman filter state."""
    _next_id = 0

    def __init__(self, bbox):
        self.id = TrackedObject._next_id
        TrackedObject._next_id += 1
        self.bbox = bbox  # (x, y, w, h)
        self.age = 0
        self.hits = 1
        self.time_since_update = 0
        # State: [x, y, w, h, dx, dy, dw, dh]
        self.state = np.array([bbox[0], bbox[1], bbox[2], bbox[3],
                               0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        # Covariance
        self.P = np.eye(8) * 100.0
        self.P[4:, 4:] *= 10.0

    def predict(self):
        # ============================================================
        # TODO: Implement Kalman filter prediction step
        # 1. Define state transition matrix F (constant velocity model):
        #    F = np.eye(8)
        #    F[0,4] = F[1,5] = F[2,6] = F[3,7] = 1.0  # dt=1
        # 2. Process noise Q
        # 3. Predict: state = F @ state, P = F @ P @ F.T + Q
        # 4. Update self.bbox from predicted state
        # ============================================================
        self.age += 1
        self.time_since_update += 1

    def update(self, bbox):
        # ============================================================
        # TODO: Implement Kalman filter update step
        # 1. Measurement z = [bbox_x, bbox_y, bbox_w, bbox_h]
        # 2. Observation matrix H (maps state to measurement)
        #    H = np.eye(4, 8)  # observe [x,y,w,h] from state
        # 3. Innovation: y = z - H @ state
        # 4. Innovation covariance: S = H @ P @ H.T + R
        # 5. Kalman gain: K = P @ H.T @ inv(S)
        # 6. Update: state = state + K @ y
        # 7. Update: P = (I - K @ H) @ P
        # ============================================================
        self.bbox = bbox
        self.hits += 1
        self.time_since_update = 0


class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__("object_tracker_node")

        # Parameters
        self.declare_parameter("detection_threshold", 100.0)
        self.declare_parameter("max_age", 10)
        self.declare_parameter("min_hits", 3)

        self.detection_threshold = self.get_parameter("detection_threshold").value
        self.max_age = self.get_parameter("max_age").value
        self.min_hits = self.get_parameter("min_hits").value

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, "/camera/left/image_raw", self.image_callback, 10)

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, "/tracked_objects", 10)
        self.tracking_pub = self.create_publisher(Image, "/tracking_image", 10)

        self.trackers = []

        if HAS_CV:
            self.bridge = CvBridge()
        else:
            self.get_logger().error("OpenCV/cv_bridge not available!")

        self.get_logger().info("Object Tracker started")

    def detect_objects(self, cv_image):
        """Simple color/contour-based detection proxy."""
        # ============================================================
        # TODO: Implement object detection (simple proxy)
        # 1. Convert to HSV or use edge detection
        # 2. Threshold to find blobs
        # 3. Find contours with cv2.findContours
        # 4. Filter by area > detection_threshold
        # 5. Return list of bounding boxes [(x,y,w,h), ...]
        # ============================================================
        detections = []
        return detections

    def associate_detections(self, detections):
        """Hungarian algorithm to match detections to existing trackers."""
        # ============================================================
        # TODO: Implement Hungarian assignment
        # 1. Build cost matrix: IoU or Euclidean distance between
        #    each tracker's predicted bbox and each detection
        # 2. Use linear_sum_assignment(cost_matrix) from scipy
        # 3. Return matched pairs, unmatched detections, unmatched trackers
        # ============================================================
        matched = []
        unmatched_dets = list(range(len(detections)))
        unmatched_trks = []
        return matched, unmatched_dets, unmatched_trks

    def image_callback(self, msg):
        if not HAS_CV:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Predict existing trackers
        for trk in self.trackers:
            trk.predict()

        # Detect objects
        detections = self.detect_objects(cv_image)

        # Associate
        matched, unmatched_dets, unmatched_trks = self.associate_detections(detections)

        # Update matched trackers
        for trk_idx, det_idx in matched:
            self.trackers[trk_idx].update(detections[det_idx])

        # Create new trackers for unmatched detections
        for det_idx in unmatched_dets:
            self.trackers.append(TrackedObject(detections[det_idx]))

        # Remove dead trackers
        self.trackers = [t for t in self.trackers
                         if t.time_since_update < self.max_age]

        # Draw and publish
        annotated = cv_image.copy()
        marker_array = MarkerArray()
        active_trackers = [t for t in self.trackers if t.hits >= self.min_hits]

        for i, trk in enumerate(active_trackers):
            x, y, w, h = [int(v) for v in trk.bbox]
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(annotated, f"ID:{trk.id}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            marker = Marker()
            marker.header = msg.header
            marker.ns = "tracked_objects"
            marker.id = trk.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=float(x + w / 2), y=float(y + h / 2), z=0.0)
            marker.scale = Vector3(x=float(w), y=float(h), z=1.0)
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        tracking_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        tracking_msg.header = msg.header
        self.tracking_pub.publish(tracking_msg)

        self.get_logger().info(f"Tracking {len(active_trackers)} objects")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
