#!/usr/bin/env python3
"""Generate synthetic Control reference trajectory bag data for Week 11."""
import os, math, shutil
import numpy as np
import rclpy
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Imu

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'bag_data', 'synthetic_control')

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
    print('[Week 11] Generating synthetic Control bag data...')
    t0 = 0
    rng = np.random.RandomState(42)

    print('  Generating /odom (1500 msgs)...')
    odom_msgs = []
    for i in range(1500):
        t = i / 50.0
        ts = t0 + int(t * 1e9)
        # Figure-8 with noise, starting from hover at (0,0,2)
        blend = min(t / 3.0, 1.0)  # ramp up over 3 seconds
        ref_x = 2.0 * math.sin(0.2 * t)
        ref_y = 2.0 * math.sin(0.4 * t)
        ref_z = 2.0
        noise_x = rng.normal(0, 0.05)
        noise_y = rng.normal(0, 0.05)
        noise_z = rng.normal(0, 0.02)
        msg = Odometry()
        msg.header = make_header(ts, 'odom')
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = blend * ref_x + noise_x
        msg.pose.pose.position.y = blend * ref_y + noise_y
        msg.pose.pose.position.z = ref_z + noise_z
        yaw = math.atan2(0.2 * 2.0 * math.cos(0.2 * t), 0.4 * 2.0 * math.cos(0.4 * t))
        msg.pose.pose.orientation = yaw_to_quat(yaw)
        msg.twist.twist.linear.x = blend * 2.0 * 0.2 * math.cos(0.2 * t)
        msg.twist.twist.linear.y = blend * 2.0 * 0.4 * math.cos(0.4 * t)
        odom_msgs.append((ts, msg))

    print('  Generating /reference_trajectory (1500 msgs)...')
    ref_msgs = []
    for i in range(1500):
        t = i / 50.0
        ts = t0 + int(t * 1e9)
        msg = PoseStamped()
        msg.header = make_header(ts, 'map')
        msg.pose.position.x = 2.0 * math.sin(0.2 * t)
        msg.pose.position.y = 2.0 * math.sin(0.4 * t)
        msg.pose.position.z = 2.0
        yaw = math.atan2(0.2 * 2.0 * math.cos(0.2 * t), 0.4 * 2.0 * math.cos(0.4 * t))
        msg.pose.orientation = yaw_to_quat(yaw)
        ref_msgs.append((ts, msg))

    print('  Generating /imu (6000 msgs)...')
    imu_msgs = []
    for i in range(6000):
        t = i / 200.0
        ts = t0 + int(t * 1e9)
        msg = Imu()
        msg.header = make_header(ts, 'imu_link')
        # Accelerations from figure-8 second derivatives + gravity compensation
        ax = -2.0 * (0.2**2) * math.sin(0.2 * t)
        ay = -2.0 * (0.4**2) * math.sin(0.4 * t)
        msg.linear_acceleration.x = ax + rng.normal(0, 0.01)
        msg.linear_acceleration.y = ay + rng.normal(0, 0.01)
        msg.linear_acceleration.z = 9.81 + rng.normal(0, 0.01)
        msg.angular_velocity.x = rng.normal(0, 0.005)
        msg.angular_velocity.y = rng.normal(0, 0.005)
        msg.angular_velocity.z = rng.normal(0, 0.01)
        msg.orientation = yaw_to_quat(math.atan2(0.2 * 2.0 * math.cos(0.2 * t), 0.4 * 2.0 * math.cos(0.4 * t)))
        imu_msgs.append((ts, msg))

    topics = [
        ('/odom', 'nav_msgs/msg/Odometry', odom_msgs),
        ('/reference_trajectory', 'geometry_msgs/msg/PoseStamped', ref_msgs),
        ('/imu', 'sensor_msgs/msg/Imu', imu_msgs),
    ]
    print(f'  Writing bag to {OUTPUT_DIR}...')
    write_bag(OUTPUT_DIR, topics)
    print('[Week 11] Done!')

if __name__ == '__main__':
    generate()
