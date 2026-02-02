#!/usr/bin/env python3
"""Generate synthetic 3D environment pointcloud bag data for Week 8 exercises."""
import os, math, shutil, struct
import numpy as np
import rclpy
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "bag_data", "synthetic_3d_env")

def make_header(stamp_ns, frame_id="map"):
    h = Header()
    h.stamp = Time(sec=int(stamp_ns // 10**9), nanosec=int(stamp_ns % 10**9))
    h.frame_id = frame_id
    return h

def write_bag(output_dir, topics_msgs):
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(os.path.dirname(output_dir), exist_ok=True)
    storage = rosbag2_py.StorageOptions(uri=output_dir, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("cdr", "cdr")
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage, converter)
    for name, type_str, _ in topics_msgs:
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=name, type=type_str, serialization_format="cdr"))
    for name, _, msgs in topics_msgs:
        for ts, msg in sorted(msgs, key=lambda x: x[0]):
            writer.write(name, serialize_message(msg), ts)
    del writer

def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return Quaternion(
        x=sr*cp*cy - cr*sp*sy,
        y=cr*sp*cy + sr*cp*sy,
        z=cr*cp*sy - sr*sp*cy,
        w=cr*cp*cy + sr*sp*sy,
    )

def generate_corridor_pointcloud(robot_x, robot_y, robot_z=1.0):
    """Simulate Ouster OS-1 64-beam LiDAR in a corridor."""
    # Corridor: walls at y=+/-2, floor z=0, ceiling z=3
    # Pillars at fixed x positions
    pillars = [(5.0, 1.0, 0.3), (10.0, -0.5, 0.4), (15.0, 0.8, 0.3), (20.0, -1.0, 0.35)]

    points = []
    n_beams = 64
    n_azimuth = 128
    lidar_z = robot_z + 0.5  # lidar mounted above robot

    for az_i in range(n_azimuth):
        azimuth = (az_i / n_azimuth) * 2 * math.pi
        for beam_i in range(n_beams):
            elevation = math.radians(-16.6 + (beam_i / 63.0) * 33.2)  # OS-1 FOV
            dx = math.cos(elevation) * math.cos(azimuth)
            dy = math.cos(elevation) * math.sin(azimuth)
            dz = math.sin(elevation)

            min_t = 50.0  # max range
            # Floor (z=0)
            if dz < -0.001:
                t = (0 - lidar_z) / dz
                if 0.3 < t < min_t:
                    min_t = t
            # Ceiling (z=3)
            if dz > 0.001:
                t = (3.0 - lidar_z) / dz
                if 0.3 < t < min_t:
                    min_t = t
            # Left wall (y=2 relative to robot_y -> world y = robot_y + local_y)
            if dy > 0.001:
                t = (2.0 - robot_y) / dy  # approximate in local frame
                if 0.3 < t < min_t:
                    min_t = t
            # Right wall (y=-2)
            if dy < -0.001:
                t = (-2.0 - robot_y) / dy
                if abs(t) > 0 and 0.3 < t < min_t:
                    min_t = t

            # Pillars (cylinders)
            for px, py, pr in pillars:
                # Ray-cylinder intersection in 2D (x-y plane from lidar pos)
                ox = robot_x - px
                oy = robot_y - py
                a = dx*dx + dy*dy
                b = 2*(ox*dx + oy*dy)
                c = ox*ox + oy*oy - pr*pr
                disc = b*b - 4*a*c
                if disc > 0:
                    t1 = (-b - math.sqrt(disc)) / (2*a)
                    if 0.3 < t1 < min_t:
                        hit_z = lidar_z + dz * t1
                        if 0 <= hit_z <= 3.0:
                            min_t = t1

            if min_t < 49.0:
                hx = dx * min_t
                hy = dy * min_t
                hz = dz * min_t
                intensity = max(0.1, 1.0 - min_t / 50.0) * 255
                points.append((hx, hy, hz + lidar_z - robot_z, intensity))

    return points

def make_pointcloud2(header, points):
    """Create PointCloud2 from list of (x,y,z,intensity) tuples."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(points)
    msg.is_dense = True
    data = bytearray()
    for x, y, z, i in points:
        data.extend(struct.pack("<ffff", x, y, z, i))
    msg.data = bytes(data)
    return msg

def main():
    print("=== Week 8: Generating synthetic 3D environment bag ===")
    print(f"Output: {OUTPUT_DIR}")

    # /os1_cloud_node/points at 10Hz, 20s = 200 msgs
    pc_msgs = []
    dt_ns = int(1e9 / 10)
    for i in range(200):
        ts = i * dt_ns
        t = i / 10.0
        # Robot walks forward through corridor
        robot_x = 0.5 * t  # 0.5 m/s forward
        robot_y = 0.1 * math.sin(0.5 * t)  # slight sway
        points = generate_corridor_pointcloud(robot_x, robot_y)
        header = make_header(ts, "os1_lidar")
        msg = make_pointcloud2(header, points)
        pc_msgs.append((ts, msg))
        if (i + 1) % 50 == 0:
            print(f"    pointcloud {i+1}/200 ({len(points)} points)")
    print(f"  [/os1_cloud_node/points] {len(pc_msgs)} messages")

    # /odom at 20Hz, 20s = 400 msgs
    odom_msgs = []
    dt_ns = int(1e9 / 20)
    for i in range(400):
        ts = i * dt_ns
        t = i / 20.0
        robot_x = 0.5 * t
        robot_y = 0.1 * math.sin(0.5 * t)
        vx = 0.5
        vy = 0.1 * 0.5 * math.cos(0.5 * t)
        yaw = math.atan2(vy, vx)
        o = Odometry(); o.header = make_header(ts, "odom"); o.child_frame_id = "base_link"
        o.pose.pose.position = Point(x=float(robot_x), y=float(robot_y), z=0.0)
        o.pose.pose.orientation = euler_to_quat(0, 0, yaw)
        o.twist.twist.linear = Vector3(x=float(vx), y=float(vy), z=0.0)
        odom_msgs.append((ts, o))
    print(f"  [/odom] {len(odom_msgs)} messages")

    topics = [
        ("/os1_cloud_node/points", "sensor_msgs/msg/PointCloud2", pc_msgs),
        ("/odom", "nav_msgs/msg/Odometry", odom_msgs),
    ]
    print("  Writing bag...")
    write_bag(OUTPUT_DIR, topics)
    print("  Done!")

if __name__ == "__main__":
    main()
