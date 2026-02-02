#!/usr/bin/env python3
"""Week 10 Solution 3: 2D Object Tracker (Complete)"""
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

def iou(b1, b2):
    x1, y1 = max(b1[0],b2[0]), max(b1[1],b2[1])
    x2, y2 = min(b1[0]+b1[2],b2[0]+b2[2]), min(b1[1]+b1[3],b2[1]+b2[3])
    inter = max(0,x2-x1)*max(0,y2-y1)
    union = b1[2]*b1[3]+b2[2]*b2[3]-inter
    return inter/union if union>0 else 0.0

class KalmanTracker:
    _next_id = 0
    def __init__(self, bbox):
        self.id = KalmanTracker._next_id; KalmanTracker._next_id += 1
        self.bbox = np.array(bbox, dtype=np.float64)
        self.age = 0; self.hits = 1; self.time_since_update = 0
        self.x = np.zeros(8); self.x[:4] = bbox
        self.F = np.eye(8); self.F[0,4]=self.F[1,5]=self.F[2,6]=self.F[3,7]=1.0
        self.H = np.eye(4, 8)
        self.P = np.eye(8)*10.0; self.P[4:,4:]*=100.0
        self.Q = np.eye(8)*1.0; self.Q[4:,4:]*=0.01
        self.R = np.eye(4)*1.0
    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1; self.time_since_update += 1
        self.bbox = self.x[:4].copy()
        return self.bbox
    def update(self, z):
        z = np.array(z, dtype=np.float64)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(8) - K @ self.H) @ self.P
        self.bbox = self.x[:4].copy()
        self.hits += 1; self.time_since_update = 0

class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__("object_tracker_node")
        self.declare_parameter("detection_threshold", 500.0)
        self.declare_parameter("max_age", 10)
        self.declare_parameter("min_hits", 3)
        self.declare_parameter("iou_threshold", 0.3)
        self.detection_threshold = self.get_parameter("detection_threshold").value
        self.max_age = self.get_parameter("max_age").value
        self.min_hits = self.get_parameter("min_hits").value
        self.iou_threshold = self.get_parameter("iou_threshold").value
        self.image_sub = self.create_subscription(Image, "/camera/left/image_raw", self.image_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/tracked_objects", 10)
        self.tracking_pub = self.create_publisher(Image, "/tracking_image", 10)
        self.trackers = []
        if HAS_CV: self.bridge = CvBridge()
        self.get_logger().info("Object Tracker started")

    def detect_objects(self, img):
        if not HAS_CV: return []
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0,50,50), (180,255,255))
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k), cv2.MORPH_OPEN, k)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return [cv2.boundingRect(c) for c in contours if cv2.contourArea(c) >= self.detection_threshold]

    def associate(self, dets):
        if not self.trackers or not dets:
            return [], list(range(len(dets))), list(range(len(self.trackers)))
        cost = np.array([[1.0-iou(t.bbox,d) for d in dets] for t in self.trackers])
        if HAS_SCIPY:
            ri, ci = linear_sum_assignment(cost)
        else:
            ri, ci, used = [], [], set()
            for t in range(len(self.trackers)):
                bd, bc = -1, 1.0
                for d in range(len(dets)):
                    if d not in used and cost[t,d]<bc: bd,bc=d,cost[t,d]
                if bd>=0 and bc<1.0-self.iou_threshold: ri.append(t);ci.append(bd);used.add(bd)
        matched, mt, md = [], set(), set()
        for t,d in zip(ri,ci):
            if cost[t,d]<1.0-self.iou_threshold: matched.append((t,d));mt.add(t);md.add(d)
        return matched, [d for d in range(len(dets)) if d not in md], [t for t in range(len(self.trackers)) if t not in mt]

    def image_callback(self, msg):
        if not HAS_CV: return
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        for t in self.trackers: t.predict()
        dets = self.detect_objects(img)
        matched, ud, ut = self.associate(dets)
        for ti,di in matched: self.trackers[ti].update(dets[di])
        for di in ud: self.trackers.append(KalmanTracker(dets[di]))
        self.trackers = [t for t in self.trackers if t.time_since_update < self.max_age]
        ann = img.copy()
        ma = MarkerArray()
        colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(0,255,255)]
        active = [t for t in self.trackers if t.hits >= self.min_hits]
        for trk in active:
            x,y,w,h = [int(v) for v in trk.bbox]
            c = colors[trk.id % len(colors)]
            cv2.rectangle(ann,(x,y),(x+w,y+h),c,2)
            cv2.putText(ann,f"ID:{trk.id}",(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.6,c,2)
            m = Marker(); m.header=msg.header; m.ns="tracked"; m.id=trk.id
            m.type=Marker.CUBE; m.action=Marker.ADD
            m.pose.position=Point(x=float(x+w/2),y=float(y+h/2),z=0.0)
            m.scale=Vector3(x=float(max(w,1)),y=float(max(h,1)),z=1.0)
            m.color=ColorRGBA(r=c[2]/255.0,g=c[1]/255.0,b=c[0]/255.0,a=0.8)
            m.lifetime.nanosec=500000000
            ma.markers.append(m)
        cv2.putText(ann,f"Tracking: {len(active)}",(10,30),cv2.FONT_HERSHEY_SIMPLEX,1.0,(0,255,0),2)
        self.marker_pub.publish(ma)
        out=self.bridge.cv2_to_imgmsg(ann,"bgr8"); out.header=msg.header
        self.tracking_pub.publish(out)
        self.get_logger().info(f"Tracking {len(active)}, detections {len(dets)}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
