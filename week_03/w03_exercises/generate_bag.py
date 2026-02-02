#!/usr/bin/env python3
"""Generate synthetic TurtleBot3 simulation bag data."""
import os
import shutil
import math
import numpy as np

from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py

from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, Quaternion, Vector3
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
    """Simulate a 360-ray LaserScan in a rectangular room."""
    num_rays = 360
    angle_min = 0.0
    angle_max = 2 * math.pi
    range_min = 0.12
    range_max = 3.5
    angles = np.linspace(angle_min, angle_max, num_rays, endpoint=False)
    ranges = np.full(num_rays, range_max, dtype=np.float32)
    for i, a in enumerate(angles):
        ray_angle = robot_yaw + a
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        min_dist = range_max
        for (wx1, wy1, wx2, wy2) in walls:
            # Ray-segment intersection
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
        ranges[i] = float(min_dist) + rng.normal(0, 0.005)
    ranges = np.clip(ranges, range_min, range_max).astype(np.float32)
    return ranges, angle_min, angle_max, range_min, range_max, num_rays


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'bag_data', 'synthetic_turtlebot')
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
        print(f'Removed existing {output_dir}')

    rng = np.random.default_rng(42)
    duration = 30.0

    # Room walls: 5m x 5m centered at (2.5, 2.5)
    walls = [
        (0, 0, 5, 0),  # bottom
        (5, 0, 5, 5),  # right
        (5, 5, 0, 5),  # top
        (0, 5, 0, 0),  # left
        # Internal obstacle
        (2, 2, 3, 2),
        (3, 2, 3, 3),
    ]

    # Robot trajectory: forward then turn
    dt_fine = 0.001
    num_steps = int(duration / dt_fine)
    times = np.arange(num_steps) * dt_fine
    robot_x, robot_y, robot_yaw = 1.0, 1.0, 0.0
    traj = []  # (t, x, y, yaw, vx, wz)
    for i in range(num_steps):
        t = times[i]
        # Vary cmd_vel over time
        if t < 10:
            v = 0.2
            w = 0.1
        elif t < 20:
            v = 0.15
            w = -0.2
        else:
            v = 0.2
            w = 0.15
        robot_yaw += w * dt_fine
        robot_x += v * math.cos(robot_yaw) * dt_fine
        robot_y += v * math.sin(robot_yaw) * dt_fine
        # Clamp to room
        robot_x = np.clip(robot_x, 0.3, 4.7)
        robot_y = np.clip(robot_y, 0.3, 4.7)
        traj.append((t, robot_x, robot_y, robot_yaw, v, w))

    def get_state(t):
        idx = min(int(t / dt_fine), len(traj) - 1)
        return traj[idx]

    # Generate scan messages (5Hz)
    scan_msgs = []
    scan_dt = 1.0 / 5
    num_scans = int(duration * 5)
    print(f'Generating {num_scans} scan messages...')
    for i in range(num_scans):
        t = i * scan_dt
        stamp_ns = int(t * 1e9)
        _, rx, ry, ryaw, _, _ = get_state(t)
        ranges, amin, amax, rmin, rmax, nrays = simulate_laserscan(rx, ry, ryaw, walls, rng)
        msg = LaserScan()
        msg.header = make_header(stamp_ns, 'base_scan')
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

    # Generate odom messages (20Hz)
    odom_msgs = []
    odom_dt = 1.0 / 20
    num_odom = int(duration * 20)
    print(f'Generating {num_odom} odom messages...')
    for i in range(num_odom):
        t = i * odom_dt
        stamp_ns = int(t * 1e9)
        _, rx, ry, ryaw, v, w = get_state(t)
        qw = math.cos(ryaw / 2)
        qz = math.sin(ryaw / 2)
        msg = Odometry()
        msg.header = make_header(stamp_ns, 'odom')
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = rx + rng.normal(0, 0.002)
        msg.pose.pose.position.y = ry + rng.normal(0, 0.002)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        msg.twist.twist.linear = Vector3(x=v, y=0.0, z=0.0)
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=w)
        odom_msgs.append((stamp_ns, serialize_message(msg)))

    # Generate IMU messages (50Hz)
    imu_msgs = []
    imu_dt = 1.0 / 50
    num_imu = int(duration * 50)
    print(f'Generating {num_imu} IMU messages...')
    for i in range(num_imu):
        t = i * imu_dt
        stamp_ns = int(t * 1e9)
        _, rx, ry, ryaw, v, w = get_state(t)
        qw = math.cos(ryaw / 2)
        qz = math.sin(ryaw / 2)
        msg = Imu()
        msg.header = make_header(stamp_ns, 'imu_link')
        msg.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        msg.angular_velocity = Vector3(x=rng.normal(0, 0.01), y=rng.normal(0, 0.01), z=w + rng.normal(0, 0.01))
        # Centripetal + gravity
        msg.linear_acceleration = Vector3(x=rng.normal(0, 0.05), y=rng.normal(0, 0.05), z=9.81 + rng.normal(0, 0.05))
        msg.orientation_covariance = [0.001]*9
        msg.angular_velocity_covariance = [0.01]*9
        msg.linear_acceleration_covariance = [0.1]*9
        imu_msgs.append((stamp_ns, serialize_message(msg)))

    # Generate cmd_vel messages (10Hz)
    cmd_msgs = []
    cmd_dt = 1.0 / 10
    num_cmd = int(duration * 10)
    print(f'Generating {num_cmd} cmd_vel messages...')
    for i in range(num_cmd):
        t = i * cmd_dt
        stamp_ns = int(t * 1e9)
        _, _, _, _, v, w = get_state(t)
        msg = Twist()
        msg.linear = Vector3(x=v, y=0.0, z=0.0)
        msg.angular = Vector3(x=0.0, y=0.0, z=w)
        cmd_msgs.append((stamp_ns, serialize_message(msg)))

    topic_data = [
        ('/scan', 'sensor_msgs/msg/LaserScan', scan_msgs),
        ('/odom', 'nav_msgs/msg/Odometry', odom_msgs),
        ('/imu', 'sensor_msgs/msg/Imu', imu_msgs),
        ('/cmd_vel', 'geometry_msgs/msg/Twist', cmd_msgs),
    ]
    print(f'Writing bag to {output_dir} ...')
    create_bag(output_dir, topic_data)
    print(f'Done! Bag saved to: {output_dir}')


if __name__ == '__main__':
    main()
