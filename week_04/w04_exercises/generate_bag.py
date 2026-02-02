#!/usr/bin/env python3
"""Generate synthetic 2D SLAM bag data with loop closure."""
import os
import shutil
import math
import numpy as np

from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
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


def simulate_laserscan(robot_x, robot_y, robot_yaw, walls, rng):
    num_rays = 360
    angle_min = 0.0
    angle_max = 2 * math.pi
    range_min = 0.12
    range_max = 12.0
    angles = np.linspace(angle_min, angle_max, num_rays, endpoint=False)
    ranges = np.full(num_rays, range_max, dtype=np.float32)
    for i, a in enumerate(angles):
        ray_angle = robot_yaw + a
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        min_dist = range_max
        for (wx1, wy1, wx2, wy2) in walls:
            ex, ey = wx2 - wx1, wy2 - wy1
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-10:
                continue
            t = ((wx1 - robot_x) * ey - (wy1 - robot_y) * ex) / denom
            u = ((wx1 - robot_x) * dy - (wy1 - robot_y) * dx) / denom
            if t > 0 and 0 <= u <= 1 and t < min_dist:
                min_dist = t
        if min_dist < range_min:
            min_dist = range_min
        ranges[i] = float(min_dist) + rng.normal(0, 0.01)
    ranges = np.clip(ranges, range_min, range_max).astype(np.float32)
    return ranges, angle_min, angle_max, range_min, range_max, num_rays


def build_corridor_walls():
    """Build a rectangular loop corridor (building-like)."""
    walls = []
    corridor_width = 2.0
    # Outer rectangle 20x15
    outer = [(0, 0), (20, 0), (20, 15), (0, 15)]
    # Inner rectangle
    iw = corridor_width
    inner = [(iw, iw), (20 - iw, iw), (20 - iw, 15 - iw), (iw, 15 - iw)]
    for rect in [outer, inner]:
        for j in range(len(rect)):
            x1, y1 = rect[j]
            x2, y2 = rect[(j + 1) % len(rect)]
            walls.append((x1, y1, x2, y2))
    # Add some internal features for scan matching
    walls.append((5, 0, 5, 1.0))
    walls.append((10, 0, 10, 0.8))
    walls.append((15, 0, 15, 1.0))
    walls.append((5, 15, 5, 14.0))
    walls.append((15, 15, 15, 14.0))
    walls.append((0, 5, 1.0, 5))
    walls.append((0, 10, 0.8, 10))
    walls.append((20, 5, 19.0, 5))
    walls.append((20, 10, 19.0, 10))
    return walls


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'bag_data', 'synthetic_2d_slam')
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
        print(f'Removed existing {output_dir}')

    rng = np.random.default_rng(42)
    duration = 60.0
    walls = build_corridor_walls()

    # Robot drives a rectangular loop in the corridor
    # Waypoints along corridor center
    cw = 2.0
    cx = cw / 2  # center of corridor = 1.0
    waypoints = [
        (1.0, 1.0, 0.0),       # start bottom-left, facing right
        (19.0, 1.0, 0.0),      # bottom-right
        (19.0, 14.0, math.pi/2),  # top-right
        (1.0, 14.0, math.pi),  # top-left
        (1.0, 1.0, -math.pi/2),  # back to start (loop closure)
    ]

    # Interpolate trajectory
    total_path_len = 0
    segments = []
    for i in range(len(waypoints) - 1):
        x1, y1, _ = waypoints[i]
        x2, y2, _ = waypoints[i + 1]
        seg_len = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        total_path_len += seg_len
        segments.append((x1, y1, x2, y2, seg_len))

    speed = total_path_len / duration  # constant speed

    dt_fine = 0.001
    num_steps = int(duration / dt_fine)
    traj = []
    cum_dist = 0.0
    seg_idx = 0
    seg_dist = 0.0
    for i in range(num_steps):
        t = i * dt_fine
        if seg_idx < len(segments):
            x1, y1, x2, y2, seg_len = segments[seg_idx]
            frac = seg_dist / seg_len if seg_len > 0 else 0
            rx = x1 + frac * (x2 - x1)
            ry = y1 + frac * (y2 - y1)
            ryaw = math.atan2(y2 - y1, x2 - x1)
            seg_dist += speed * dt_fine
            if seg_dist >= seg_len and seg_idx < len(segments) - 1:
                seg_dist -= seg_len
                seg_idx += 1
        else:
            rx, ry = waypoints[-1][0], waypoints[-1][1]
            ryaw = waypoints[-1][2]
        traj.append((t, rx, ry, ryaw, speed))

    def get_state(t):
        idx = min(int(t / dt_fine), len(traj) - 1)
        return traj[idx]

    # Generate scan messages (10Hz)
    scan_msgs = []
    scan_dt = 1.0 / 10
    num_scans = int(duration * 10)
    print(f'Generating {num_scans} scan messages...')
    for i in range(num_scans):
        t = i * scan_dt
        stamp_ns = int(t * 1e9)
        _, rx, ry, ryaw, _ = get_state(t)
        ranges, amin, amax, rmin, rmax, nrays = simulate_laserscan(rx, ry, ryaw, walls, rng)
        msg = LaserScan()
        msg.header = make_header(stamp_ns, 'base_laser')
        msg.angle_min = float(amin)
        msg.angle_max = float(amax)
        msg.angle_increment = float((amax - amin) / nrays)
        msg.time_increment = 0.0
        msg.scan_time = float(scan_dt)
        msg.range_min = float(rmin)
        msg.range_max = float(rmax)
        msg.ranges = ranges.tolist()
        msg.intensities = []
        scan_msgs.append((stamp_ns, serialize_message(msg)))
        if (i + 1) % 100 == 0:
            print(f'  Scan {i+1}/{num_scans}')

    # Generate odom messages (20Hz) with drift
    odom_msgs = []
    odom_dt = 1.0 / 20
    num_odom = int(duration * 20)
    print(f'Generating {num_odom} odom messages...')
    drift_x, drift_y, drift_yaw = 0.0, 0.0, 0.0
    prev_rx, prev_ry, prev_yaw = None, None, None
    for i in range(num_odom):
        t = i * odom_dt
        stamp_ns = int(t * 1e9)
        _, rx, ry, ryaw, v = get_state(t)
        if prev_rx is not None:
            dx = rx - prev_rx
            dy = ry - prev_ry
            dyaw = ryaw - prev_yaw
            # Accumulate drift
            drift_x += rng.normal(0, 0.001)
            drift_y += rng.normal(0, 0.001)
            drift_yaw += rng.normal(0, 0.0005)
        prev_rx, prev_ry, prev_yaw = rx, ry, ryaw

        odom_x = rx + drift_x
        odom_y = ry + drift_y
        odom_yaw = ryaw + drift_yaw
        qw = math.cos(odom_yaw / 2)
        qz = math.sin(odom_yaw / 2)

        msg = Odometry()
        msg.header = make_header(stamp_ns, 'odom')
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = odom_x
        msg.pose.pose.position.y = odom_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        msg.twist.twist.linear = Vector3(x=v, y=0.0, z=0.0)
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        odom_msgs.append((stamp_ns, serialize_message(msg)))

    topic_data = [
        ('/scan', 'sensor_msgs/msg/LaserScan', scan_msgs),
        ('/odom', 'nav_msgs/msg/Odometry', odom_msgs),
    ]
    print(f'Writing bag to {output_dir} ...')
    create_bag(output_dir, topic_data)
    print(f'Done! Bag saved to: {output_dir}')


if __name__ == '__main__':
    main()
