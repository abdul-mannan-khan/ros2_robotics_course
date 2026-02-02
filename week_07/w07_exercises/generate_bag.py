#!/usr/bin/env python3
"""Generate synthetic PX4 telemetry bag data for Week 7 exercises."""
import os, math, shutil, json
import numpy as np
import rclpy
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py
from std_msgs.msg import Header, String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "bag_data", "synthetic_px4")

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

def get_trajectory(t):
    """Takeoff 0-5s, circle 5-25s, land 25-30s."""
    if t < 5.0:
        # Takeoff: climb to 10m
        frac = t / 5.0
        x, y, z = 0.0, 0.0, 10.0 * frac
        yaw = 0.0
    elif t < 25.0:
        # Circle: radius 20m, period 20s
        phase = (t - 5.0) / 20.0 * 2 * math.pi
        x = 20.0 * math.cos(phase)
        y = 20.0 * math.sin(phase)
        z = 10.0
        yaw = phase + math.pi / 2  # face tangent
    else:
        # Land: descend from 10m
        frac = (t - 25.0) / 5.0
        last_phase = 1.0 * 2 * math.pi  # where circle ended
        x = 20.0 * math.cos(last_phase)
        y = 20.0 * math.sin(last_phase)
        z = 10.0 * (1.0 - frac)
        yaw = last_phase + math.pi / 2
    return x, y, z, yaw

def main():
    print("=== Week 7: Generating synthetic PX4 bag ===")
    print(f"Output: {OUTPUT_DIR}")

    # /fmu/out/vehicle_odometry at 50Hz = 1500 msgs
    odom_msgs = []
    dt_ns = int(1e9 / 50)
    for i in range(1500):
        ts = i * dt_ns
        t = i / 50.0
        x, y, z, yaw = get_trajectory(t)
        # Compute velocity via finite diff
        dt = 0.02
        if t > dt:
            x0, y0, z0, _ = get_trajectory(t - dt)
            vx, vy, vz = (x-x0)/dt, (y-y0)/dt, (z-z0)/dt
        else:
            vx, vy, vz = 0.0, 0.0, 2.0
        o = Odometry(); o.header = make_header(ts, "odom"); o.child_frame_id = "base_link"
        o.pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        o.pose.pose.orientation = euler_to_quat(0.0, 0.0, yaw)
        o.twist.twist.linear = Vector3(x=float(vx), y=float(vy), z=float(vz))
        odom_msgs.append((ts, o))
    print(f"  [/fmu/out/vehicle_odometry] {len(odom_msgs)} messages")

    # /fmu/out/vehicle_status at 1Hz = 30 msgs
    status_msgs = []
    for i in range(30):
        ts = i * int(1e9)
        t = float(i)
        if t < 1.0:
            mode, armed = "TAKEOFF", False
        elif t < 5.0:
            mode, armed = "TAKEOFF", True
        elif t < 25.0:
            mode, armed = "OFFBOARD", True
        elif t < 29.0:
            mode, armed = "LAND", True
        else:
            mode, armed = "LAND", False
        battery = max(0.80, 0.95 - 0.005 * t)
        msg = String()
        msg.data = json.dumps({"armed": armed, "mode": mode, "battery": round(battery, 3)})
        status_msgs.append((ts, msg))
    print(f"  [/fmu/out/vehicle_status] {len(status_msgs)} messages")

    # /fmu/out/sensor_combined at 200Hz = 6000 msgs
    imu_msgs = []
    dt_ns = int(1e9 / 200)
    for i in range(6000):
        ts = i * dt_ns
        t = i / 200.0
        _, _, _, yaw = get_trajectory(t)
        # Yaw rate
        dt2 = 0.005
        if t > dt2:
            _, _, _, yaw0 = get_trajectory(t - dt2)
            yaw_rate = (yaw - yaw0) / dt2
        else:
            yaw_rate = 0.0

        msg = Imu(); msg.header = make_header(ts, "base_link")
        msg.linear_acceleration = Vector3(x=float(np.random.normal(0, 0.1)),
                                          y=float(np.random.normal(0, 0.1)),
                                          z=float(-9.81 + np.random.normal(0, 0.1)))
        msg.angular_velocity = Vector3(x=float(np.random.normal(0, 0.01)),
                                       y=float(np.random.normal(0, 0.01)),
                                       z=float(yaw_rate + np.random.normal(0, 0.01)))
        msg.orientation = euler_to_quat(0, 0, yaw)
        imu_msgs.append((ts, msg))
    print(f"  [/fmu/out/sensor_combined] {len(imu_msgs)} messages")

    topics = [
        ("/fmu/out/vehicle_odometry", "nav_msgs/msg/Odometry", odom_msgs),
        ("/fmu/out/vehicle_status", "std_msgs/msg/String", status_msgs),
        ("/fmu/out/sensor_combined", "sensor_msgs/msg/Imu", imu_msgs),
    ]
    print("  Writing bag...")
    write_bag(OUTPUT_DIR, topics)
    print("  Done!")

if __name__ == "__main__":
    main()
