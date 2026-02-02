#!/usr/bin/env python3
"""Generate synthetic Computer Vision bag data for Week 10."""
import os, math, shutil, struct
import numpy as np
import rclpy
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'bag_data', 'synthetic_vision')

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

def make_stereo_image(rng, height=375, width=1242, shift=0):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    # Sky gradient (top half)
    for r in range(height // 2):
        blue = 200 + int(55 * r / (height // 2))
        img[r, :] = [blue, 150, 100]  # BGR sky
    # Ground (bottom half)
    img[height // 2:, :] = [60, 120, 60]  # BGR green-ish ground
    # Random rectangles as cars
    for _ in range(5):
        x1 = rng.randint(0, width - 100) + shift
        y1 = rng.randint(height // 3, height - 60)
        w = rng.randint(40, 100)
        h = rng.randint(30, 60)
        x1 = max(0, min(x1, width - w))
        color = rng.randint(0, 255, 3).tolist()
        img[y1:y1+h, x1:x1+w] = color
    return img

def make_pointcloud2(stamp_ns, rng, n_points=20000):
    msg = PointCloud2()
    msg.header = make_header(stamp_ns, 'velodyne')
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
    pts[:, 0] = rng.uniform(-40, 40, n_points)  # x
    pts[:, 1] = rng.uniform(-40, 40, n_points)  # y
    pts[:, 2] = rng.uniform(-2, 2, n_points)     # z
    pts[:, 3] = rng.uniform(0, 1, n_points)       # intensity
    msg.data = pts.tobytes()
    msg.is_dense = True
    return msg

def generate():
    print('[Week 10] Generating synthetic Computer Vision bag data...')
    t0 = 0
    rng = np.random.RandomState(42)

    print('  Generating /camera/left/image_raw (100 msgs)...')
    left_msgs = []
    print('  Generating /camera/right/image_raw (100 msgs)...')
    right_msgs = []
    for i in range(100):
        t = i / 10.0
        ts = t0 + int(t * 1e9)
        frame_rng = np.random.RandomState(42 + i)
        left_img = make_stereo_image(frame_rng, shift=0)
        msg_l = Image()
        msg_l.header = make_header(ts, 'camera_left')
        msg_l.height = 375
        msg_l.width = 1242
        msg_l.encoding = 'bgr8'
        msg_l.is_bigendian = False
        msg_l.step = 1242 * 3
        msg_l.data = left_img.tobytes()
        left_msgs.append((ts, msg_l))

        frame_rng2 = np.random.RandomState(42 + i)
        right_img = make_stereo_image(frame_rng2, shift=5)
        msg_r = Image()
        msg_r.header = make_header(ts, 'camera_right')
        msg_r.height = 375
        msg_r.width = 1242
        msg_r.encoding = 'bgr8'
        msg_r.is_bigendian = False
        msg_r.step = 1242 * 3
        msg_r.data = right_img.tobytes()
        right_msgs.append((ts, msg_r))

    print('  Generating /velodyne_points (100 msgs)...')
    pc_msgs = []
    for i in range(100):
        t = i / 10.0
        ts = t0 + int(t * 1e9)
        msg = make_pointcloud2(ts, rng, 20000)
        pc_msgs.append((ts, msg))

    topics = [
        ('/camera/left/image_raw', 'sensor_msgs/msg/Image', left_msgs),
        ('/camera/right/image_raw', 'sensor_msgs/msg/Image', right_msgs),
        ('/velodyne_points', 'sensor_msgs/msg/PointCloud2', pc_msgs),
    ]
    print(f'  Writing bag to {OUTPUT_DIR}...')
    write_bag(OUTPUT_DIR, topics)
    print('[Week 10] Done!')

if __name__ == '__main__':
    generate()
