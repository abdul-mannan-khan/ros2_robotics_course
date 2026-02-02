#!/usr/bin/env python3
"""Generate synthetic Velodyne 64-beam LiDAR PointCloud2 bag data."""
import os
import shutil
import numpy as np

from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py

from sensor_msgs.msg import PointCloud2, PointField
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


def generate_pointcloud(frame_idx, obstacles, rng):
    num_beams = 64
    num_horizontal = 1800
    sensor_height = 1.8
    car_x = frame_idx * 0.5
    vert_angles = np.linspace(np.radians(-24.8), np.radians(2.0), num_beams)
    horiz_angles = np.linspace(0, 2 * np.pi, num_horizontal, endpoint=False)
    va_grid, ha_grid = np.meshgrid(vert_angles, horiz_angles, indexing='ij')
    va_flat = va_grid.ravel()
    ha_flat = ha_grid.ravel()
    cos_va = np.cos(va_flat)
    sin_va = np.sin(va_flat)
    cos_ha = np.cos(ha_flat)
    sin_ha = np.sin(ha_flat)
    down_mask = sin_va < -0.001
    dist = np.full_like(va_flat, 200.0)
    dist[down_mask] = sensor_height / (-sin_va[down_mask])
    valid = dist < 80.0
    for (ox, oy, rad, height) in obstacles:
        rel_x = ox - car_x
        rel_y = oy
        dx = cos_va * cos_ha
        dy = cos_va * sin_ha
        dz = sin_va
        denom = dx**2 + dy**2
        denom = np.where(denom < 1e-12, 1e-12, denom)
        t_close = (rel_x * dx + rel_y * dy) / denom
        hx = t_close * dx - rel_x
        hy = t_close * dy - rel_y
        d_sq = hx**2 + hy**2
        hit = (d_sq < rad**2) & (t_close > 0.5) & (t_close < dist)
        hz = sensor_height + t_close * dz
        hit = hit & (hz > 0) & (hz < height)
        dist = np.where(hit, t_close, dist)
        valid = valid | hit
    dist += rng.normal(0, 0.02, size=dist.shape)
    valid = valid & (dist > 0.3) & (rng.random(size=dist.shape) < 0.95)
    x = dist[valid] * cos_va[valid] * cos_ha[valid]
    y = dist[valid] * cos_va[valid] * sin_ha[valid]
    z = sensor_height + dist[valid] * sin_va[valid]
    intensity = np.clip(100.0 + rng.normal(0, 20, size=x.shape), 0, 255).astype(np.float32)
    return np.column_stack([x, y, z, intensity]).astype(np.float32)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'bag_data', 'synthetic_lidar')
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
        print(f'Removed existing {output_dir}')
    rng = np.random.default_rng(42)
    obstacles = [(rng.uniform(5, 50), rng.uniform(-20, 20), rng.uniform(0.5, 2.5), rng.uniform(0.5, 3.0)) for _ in range(25)]
    num_frames = 100
    dt_ns = 100_000_000
    fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='intensity', offset=12, datatype=7, count=1),
    ]
    messages = []
    for i in range(num_frames):
        stamp_ns = i * dt_ns
        if (i + 1) % 10 == 0 or i == 0:
            print(f'  Generating frame {i+1}/{num_frames}...')
        pts = generate_pointcloud(i, obstacles, rng)
        msg = PointCloud2()
        msg.header = make_header(stamp_ns, 'velodyne')
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = pts.shape[0] * 16
        msg.is_dense = True
        msg.data = pts.tobytes()
        messages.append((stamp_ns, serialize_message(msg)))
    print(f'Generated {num_frames} frames')
    topic_data = [('/velodyne_points', 'sensor_msgs/msg/PointCloud2', messages)]
    print(f'Writing bag to {output_dir} ...')
    create_bag(output_dir, topic_data)
    print(f'Done! Bag saved to: {output_dir}')


if __name__ == '__main__':
    main()
