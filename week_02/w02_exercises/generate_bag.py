#!/usr/bin/env python3
"""Generate synthetic IMU + Position data for EKF (EuRoC-like)."""
import os
import shutil
import math
import numpy as np

from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from std_msgs.msg import Header


def make_header(stamp_ns, frame_id='map'):
    h = Header()
    h.stamp = Time()
    h.stamp.sec = int(stamp_ns // 1_000_000_000)
    h.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    h.frame_id = frame_id
    return h


def create_bag(output_dir, topic_data):
    os.makedirs(os.path.dirname(output_dir), exist_ok=True)
    storage_options = rosbag2_py.StorageOptions(uri=output_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
    for topic_name, msg_type_str, _ in topic_data:
        topic_info = rosbag2_py.TopicMetadata(id=0, name=topic_name, type=msg_type_str, serialization_format='cdr')
        writer.create_topic(topic_info)
    for topic_name, _, messages in topic_data:
        for timestamp_ns, serialized_msg in messages:
            writer.write(topic_name, serialized_msg, timestamp_ns)
    del writer


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return w, x, y, z


def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])
    return R


def figure8_trajectory(t, duration=20.0):
    freq = 2 * math.pi / duration
    # Position
    x = 4.0 * math.sin(freq * t)
    y = 2.0 * math.sin(2 * freq * t)
    z = 1.0 + 0.3 * math.sin(freq * t)
    # Velocity
    vx = 4.0 * freq * math.cos(freq * t)
    vy = 2.0 * 2 * freq * math.cos(2 * freq * t)
    vz = 0.3 * freq * math.cos(freq * t)
    # Acceleration
    ax = -4.0 * freq**2 * math.sin(freq * t)
    ay = -2.0 * 4 * freq**2 * math.sin(2 * freq * t)
    az = -0.3 * freq**2 * math.sin(freq * t)
    # Yaw from velocity direction
    yaw = math.atan2(vy, vx)
    roll = 0.1 * math.sin(freq * t)
    pitch = 0.05 * math.cos(2 * freq * t)
    return (x, y, z), (vx, vy, vz), (ax, ay, az), (roll, pitch, yaw)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'bag_data', 'synthetic_euroc')
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
        print(f'Removed existing {output_dir}')

    rng = np.random.default_rng(42)
    duration = 20.0
    imu_rate = 200
    pos_rate = 10
    gravity = np.array([0, 0, -9.81])

    # Generate IMU messages
    imu_msgs = []
    num_imu = int(duration * imu_rate)
    dt_imu = 1.0 / imu_rate
    print(f'Generating {num_imu} IMU messages...')
    for i in range(num_imu):
        t = i * dt_imu
        stamp_ns = int(t * 1e9)
        pos, vel, acc, (roll, pitch, yaw) = figure8_trajectory(t, duration)
        R = rotation_matrix(roll, pitch, yaw)
        # Body-frame acceleration: R^T * (world_acc - gravity)
        world_acc = np.array(acc)
        body_acc = R.T @ (world_acc - gravity)
        body_acc += rng.normal(0, 0.1, 3)  # noise
        # Angular velocity (numerical derivative approximation)
        dt_small = 0.001
        _, _, _, (r2, p2, y2) = figure8_trajectory(t + dt_small, duration)
        omega_x = (r2 - roll) / dt_small
        omega_y = (p2 - pitch) / dt_small
        omega_z = (y2 - yaw) / dt_small
        omega = np.array([omega_x, omega_y, omega_z]) + rng.normal(0, 0.01, 3)
        w, qx, qy, qz = euler_to_quat(roll, pitch, yaw)
        # Add small orientation noise
        qx += rng.normal(0, 0.002)
        qy += rng.normal(0, 0.002)
        qz += rng.normal(0, 0.002)
        norm = math.sqrt(w**2 + qx**2 + qy**2 + qz**2)
        w, qx, qy, qz = w/norm, qx/norm, qy/norm, qz/norm

        msg = Imu()
        msg.header = make_header(stamp_ns, 'imu')
        msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=w)
        msg.angular_velocity = Vector3(x=float(omega[0]), y=float(omega[1]), z=float(omega[2]))
        msg.linear_acceleration = Vector3(x=float(body_acc[0]), y=float(body_acc[1]), z=float(body_acc[2]))
        msg.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        imu_msgs.append((stamp_ns, serialize_message(msg)))

    # Generate position messages
    pos_msgs = []
    num_pos = int(duration * pos_rate)
    dt_pos = 1.0 / pos_rate
    print(f'Generating {num_pos} position messages...')
    for i in range(num_pos):
        t = i * dt_pos
        stamp_ns = int(t * 1e9)
        pos, _, _, _ = figure8_trajectory(t, duration)
        msg = PoseStamped()
        msg.header = make_header(stamp_ns, 'map')
        msg.pose.position.x = pos[0] + rng.normal(0, 0.01)
        msg.pose.position.y = pos[1] + rng.normal(0, 0.01)
        msg.pose.position.z = pos[2] + rng.normal(0, 0.01)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pos_msgs.append((stamp_ns, serialize_message(msg)))

    topic_data = [
        ('/imu0', 'sensor_msgs/msg/Imu', imu_msgs),
        ('/leica/position', 'geometry_msgs/msg/PoseStamped', pos_msgs),
    ]
    print(f'Writing bag to {output_dir} ...')
    create_bag(output_dir, topic_data)
    print(f'Done! Bag saved to: {output_dir}')


if __name__ == '__main__':
    main()
