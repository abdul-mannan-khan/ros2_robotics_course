#!/usr/bin/env python3
"""Generate synthetic EGO-Planner bag data for Week 9."""
import os, math, shutil
import numpy as np
import rclpy
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'bag_data', 'synthetic_ego_planner')

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
    print('[Week 9] Generating synthetic EGO-Planner bag data...')
    t0 = 0
    print('  Generating /odom (750 msgs)...')
    odom_msgs = []
    for i in range(750):
        t = i / 50.0
        ts = t0 + int(t * 1e9)
        msg = Odometry()
        msg.header = make_header(ts, 'odom')
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = t * 1.0
        msg.pose.pose.position.y = 0.3 * math.sin(0.5 * t)
        msg.pose.pose.position.z = 2.0 + 0.5 * math.sin(0.8 * t)
        msg.pose.pose.orientation = yaw_to_quat(0.1 * math.sin(0.3 * t))
        msg.twist.twist.linear.x = 1.0
        msg.twist.twist.linear.z = 0.5 * 0.8 * math.cos(0.8 * t)
        odom_msgs.append((ts, msg))
    print('  Generating /depth_image (225 msgs)...')
    depth_msgs = []
    rng = np.random.RandomState(42)
    for i in range(225):
        t = i / 15.0
        ts = t0 + int(t * 1e9)
        msg = Image()
        msg.header = make_header(ts, 'camera_link')
        msg.height = 480
        msg.width = 640
        msg.encoding = '16UC1'
        msg.is_bigendian = False
        msg.step = 640 * 2
        depth = np.full((480, 640), 5000, dtype=np.uint16)
        depth[:, :80] = 2000
        depth[:, 560:] = 2000
        ox = rng.randint(100, 540)
        oy = rng.randint(100, 380)
        depth[oy:oy+80, ox:ox+80] = rng.randint(1000, 3000)
        depth[350:, :] = np.linspace(3000, 500, 130).astype(np.uint16)[:, None]
        msg.data = depth.tobytes()
        depth_msgs.append((ts, msg))
    print('  Generating /bspline (30 msgs)...')
    path_msgs = []
    for i in range(30):
        t = i / 2.0
        ts = t0 + int(t * 1e9)
        msg = Path()
        msg.header = make_header(ts, 'map')
        for j in range(10):
            tj = t + j * 0.2
            ps = PoseStamped()
            ps.header = make_header(ts, 'map')
            ps.pose.position.x = tj * 1.0
            ps.pose.position.y = 0.3 * math.sin(0.5 * tj)
            ps.pose.position.z = 2.0 + 0.5 * math.sin(0.8 * tj)
            ps.pose.orientation = yaw_to_quat(0.0)
            msg.poses.append(ps)
        path_msgs.append((ts, msg))
    topics = [
        ('/odom', 'nav_msgs/msg/Odometry', odom_msgs),
        ('/depth_image', 'sensor_msgs/msg/Image', depth_msgs),
        ('/bspline', 'nav_msgs/msg/Path', path_msgs),
    ]
    print(f'  Writing bag to {OUTPUT_DIR}...')
    write_bag(OUTPUT_DIR, topics)
    print('[Week 9] Done!')

if __name__ == '__main__':
    generate()
