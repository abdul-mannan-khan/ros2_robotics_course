#!/usr/bin/env python3
"""Generate synthetic Capstone comprehensive bag data for Week 12."""
import os, math, shutil
import numpy as np
import rclpy
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2, PointField

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'bag_data', 'synthetic_capstone')

def make_header(stamp_ns, frame_id='map'):
    h = Header()
    h.stamp = Time(sec=int(stamp_ns // 10**9), nanosec=int(stamp_ns % 10**9))
    h.frame_id = frame_id
    return h

def write_bag(output_dir, topics_msgs):
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(os.path.dirname(output_dir), exist_ok=True)
    storage = rosbag2_py.StorageOptions(uri=output_dir, storage_id='sqlite3')
    converter = rosbag2_py.ConverterOptions('cdr', 'cdr')
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage, converter)
    for name, type_str, _ in topics_msgs:
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=name, type=type_str, serialization_format='cdr'))
    for name, _, msgs in topics_msgs:
        for ts, msg in sorted(msgs, key=lambda x: x[0]):
            writer.write(name, serialize_message(msg), ts)
    del writer

def yaw_to_quat(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q

def generate():
    print('[Week 12] Generating synthetic Capstone bag data...')
    t0 = 0
    rng = np.random.RandomState(42)
    duration = 30.0
    radius = 50.0
    height = 20.0
    omega = 2.0 * math.pi / duration  # one full circle in 30s

    # Circular survey position helpers
    def survey_x(t):
        return radius * math.cos(omega * t)
    def survey_y(t):
        return radius * math.sin(omega * t)
    def survey_yaw(t):
        return omega * t + math.pi / 2.0  # tangent direction

    print('  Generating /odom (1500 msgs)...')
    odom_msgs = []
    for i in range(1500):
        t = i / 50.0
        ts = t0 + int(t * 1e9)
        msg = Odometry()
        msg.header = make_header(ts, 'odom')
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = survey_x(t)
        msg.pose.pose.position.y = survey_y(t)
        msg.pose.pose.position.z = height
        msg.pose.pose.orientation = yaw_to_quat(survey_yaw(t))
        msg.twist.twist.linear.x = -radius * omega * math.sin(omega * t)
        msg.twist.twist.linear.y = radius * omega * math.cos(omega * t)
        odom_msgs.append((ts, msg))

    print('  Generating /lidar_points (300 msgs)...')
    lidar_msgs = []
    for i in range(300):
        t = i / 10.0
        ts = t0 + int(t * 1e9)
        msg = PointCloud2()
        msg.header = make_header(ts, 'lidar_link')
        n_points = 10000
        msg.height = 1
        msg.width = n_points
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n_points
        pts = np.zeros((n_points, 4), dtype=np.float32)
        # Terrain around current position
        cx, cy = survey_x(t), survey_y(t)
        n_ground = 7000
        n_building = 3000
        pts[:n_ground, 0] = rng.uniform(cx - 30, cx + 30, n_ground).astype(np.float32)
        pts[:n_ground, 1] = rng.uniform(cy - 30, cy + 30, n_ground).astype(np.float32)
        pts[:n_ground, 2] = rng.uniform(-0.5, 0.5, n_ground).astype(np.float32)
        pts[:n_ground, 3] = rng.uniform(0.1, 0.5, n_ground).astype(np.float32)
        # Buildings
        for b in range(3):
            start = n_ground + b * 1000
            end = start + 1000
            bx = cx + rng.uniform(-20, 20)
            by = cy + rng.uniform(-20, 20)
            pts[start:end, 0] = rng.uniform(bx - 3, bx + 3, 1000).astype(np.float32)
            pts[start:end, 1] = rng.uniform(by - 3, by + 3, 1000).astype(np.float32)
            pts[start:end, 2] = rng.uniform(0, 15, 1000).astype(np.float32)
            pts[start:end, 3] = rng.uniform(0.5, 1.0, 1000).astype(np.float32)
        msg.data = pts.tobytes()
        msg.is_dense = True
        lidar_msgs.append((ts, msg))

    print('  Generating /camera/image (300 msgs)...')
    cam_msgs = []
    for i in range(300):
        t = i / 10.0
        ts = t0 + int(t * 1e9)
        msg = Image()
        msg.header = make_header(ts, 'camera_link')
        msg.height = 480
        msg.width = 640
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = 640 * 3
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:, :] = [40, 120, 50]  # green ground BGR
        # Gray buildings as rectangles
        for _ in range(4):
            bx = rng.randint(50, 550)
            by = rng.randint(100, 350)
            bw = rng.randint(30, 80)
            bh = rng.randint(40, 100)
            img[by:by+bh, bx:bx+bw] = [130, 130, 130]
        msg.data = img.tobytes()
        cam_msgs.append((ts, msg))

    print('  Generating /imu (6000 msgs)...')
    imu_msgs = []
    for i in range(6000):
        t = i / 200.0
        ts = t0 + int(t * 1e9)
        msg = Imu()
        msg.header = make_header(ts, 'imu_link')
        # Centripetal acceleration
        ax = -radius * omega**2 * math.cos(omega * t) + rng.normal(0, 0.02)
        ay = -radius * omega**2 * math.sin(omega * t) + rng.normal(0, 0.02)
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = 9.81 + rng.normal(0, 0.01)
        msg.angular_velocity.x = rng.normal(0, 0.005)
        msg.angular_velocity.y = rng.normal(0, 0.005)
        msg.angular_velocity.z = omega + rng.normal(0, 0.002)
        msg.orientation = yaw_to_quat(survey_yaw(t))
        imu_msgs.append((ts, msg))

    print('  Generating /gps (30 msgs)...')
    gps_msgs = []
    lat0, lon0 = 47.3977, 8.5456  # Zurich-ish
    for i in range(30):
        t = i / 1.0
        ts = t0 + int(t * 1e9)
        msg = NavSatFix()
        msg.header = make_header(ts, 'gps_link')
        # Convert local x,y to lat/lon offset (approx)
        dx = survey_x(t)
        dy = survey_y(t)
        msg.latitude = lat0 + dy / 111320.0
        msg.longitude = lon0 + dx / (111320.0 * math.cos(math.radians(lat0)))
        msg.altitude = height
        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1
        msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
        gps_msgs.append((ts, msg))

    topics = [
        ('/lidar_points', 'sensor_msgs/msg/PointCloud2', lidar_msgs),
        ('/camera/image', 'sensor_msgs/msg/Image', cam_msgs),
        ('/imu', 'sensor_msgs/msg/Imu', imu_msgs),
        ('/gps', 'sensor_msgs/msg/NavSatFix', gps_msgs),
        ('/odom', 'nav_msgs/msg/Odometry', odom_msgs),
    ]
    print(f'  Writing bag to {OUTPUT_DIR}...')
    write_bag(OUTPUT_DIR, topics)
    print('[Week 12] Done!')

if __name__ == '__main__':
    generate()
