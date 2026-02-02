#!/usr/bin/env python3
"""Generate synthetic quadrotor bag data for Week 6 exercises."""
import os, math, shutil
import numpy as np
import rclpy
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py
from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Point, Quaternion, QuaternionStamped, Vector3

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "bag_data", "synthetic_quadrotor")

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

def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ])

def main():
    print("=== Week 6: Generating synthetic quadrotor bag ===")
    print(f"Output: {OUTPUT_DIR}")
    duration = 10.0
    g = np.array([0, 0, -9.81])

    # Attitude profile: hover then gentle oscillation
    def get_attitude(t):
        roll = 0.05 * math.sin(2 * math.pi * 0.5 * t)
        pitch = 0.03 * math.sin(2 * math.pi * 0.3 * t + 0.5)
        yaw = 0.01 * t
        return roll, pitch, yaw

    def get_attitude_dot(t, dt=1e-5):
        r1, p1, y1 = get_attitude(t - dt)
        r2, p2, y2 = get_attitude(t + dt)
        return (r2-r1)/(2*dt), (p2-p1)/(2*dt), (y2-y1)/(2*dt)

    # /imu at 1000Hz = 10000 msgs
    imu_msgs = []
    dt_ns = int(1e9 / 1000)
    for i in range(10000):
        ts = i * dt_ns
        t = i / 1000.0
        roll, pitch, yaw = get_attitude(t)
        R = rotation_matrix(roll, pitch, yaw)
        # Gravity in body frame (accelerometer measures specific force = -g in body)
        acc_body = -R.T @ g + np.random.normal(0, 0.05, 3)
        rd, pd, yd = get_attitude_dot(t)
        gyro = np.array([rd, pd, yd]) + np.random.normal(0, 0.01, 3)
        q = euler_to_quat(roll, pitch, yaw)

        msg = Imu(); msg.header = make_header(ts, "imu_link")
        msg.orientation = q
        msg.angular_velocity = Vector3(x=float(gyro[0]), y=float(gyro[1]), z=float(gyro[2]))
        msg.linear_acceleration = Vector3(x=float(acc_body[0]), y=float(acc_body[1]), z=float(acc_body[2]))
        imu_msgs.append((ts, msg))
    print(f"  [/imu] {len(imu_msgs)} messages")

    # /ground_truth_pose at 100Hz = 1000 msgs
    pose_msgs = []
    dt_ns = int(1e9 / 100)
    for i in range(1000):
        ts = i * dt_ns
        t = i / 100.0
        roll, pitch, yaw = get_attitude(t)
        msg = PoseStamped(); msg.header = make_header(ts, "map")
        x_osc = 0.1 * math.sin(2 * math.pi * 0.2 * t)
        y_osc = 0.1 * math.cos(2 * math.pi * 0.15 * t)
        z_osc = 2.0 + 0.05 * math.sin(2 * math.pi * 0.1 * t)
        msg.pose.position = Point(x=float(x_osc), y=float(y_osc), z=float(z_osc))
        msg.pose.orientation = euler_to_quat(roll, pitch, yaw)
        pose_msgs.append((ts, msg))
    print(f"  [/ground_truth_pose] {len(pose_msgs)} messages")

    # /motor_commands at 100Hz = 1000 msgs
    motor_msgs = []
    hover_speed = 550.0
    for i in range(1000):
        ts = i * int(1e9 / 100)
        t = i / 100.0
        roll, pitch, yaw = get_attitude(t)
        # Small perturbations around hover
        speeds = [
            hover_speed + 10.0 * roll + 5.0 * pitch + np.random.normal(0, 2),
            hover_speed - 10.0 * roll + 5.0 * pitch + np.random.normal(0, 2),
            hover_speed - 10.0 * roll - 5.0 * pitch + np.random.normal(0, 2),
            hover_speed + 10.0 * roll - 5.0 * pitch + np.random.normal(0, 2),
        ]
        msg = Float32MultiArray()
        msg.data = [float(s) for s in speeds]
        motor_msgs.append((ts, msg))
    print(f"  [/motor_commands] {len(motor_msgs)} messages")

    # /desired_attitude at 100Hz = 1000 msgs
    att_msgs = []
    for i in range(1000):
        ts = i * int(1e9 / 100)
        t = i / 100.0
        # Desired is slightly ahead of actual
        des_roll = 0.05 * math.sin(2 * math.pi * 0.5 * (t + 0.05))
        des_pitch = 0.03 * math.sin(2 * math.pi * 0.3 * (t + 0.05) + 0.5)
        des_yaw = 0.01 * (t + 0.05)
        msg = QuaternionStamped()
        msg.header = make_header(ts, "map")
        msg.quaternion = euler_to_quat(des_roll, des_pitch, des_yaw)
        att_msgs.append((ts, msg))
    print(f"  [/desired_attitude] {len(att_msgs)} messages")

    topics = [
        ("/imu", "sensor_msgs/msg/Imu", imu_msgs),
        ("/ground_truth_pose", "geometry_msgs/msg/PoseStamped", pose_msgs),
        ("/motor_commands", "std_msgs/msg/Float32MultiArray", motor_msgs),
        ("/desired_attitude", "geometry_msgs/msg/QuaternionStamped", att_msgs),
    ]
    print("  Writing bag...")
    write_bag(OUTPUT_DIR, topics)
    print("  Done!")

if __name__ == "__main__":
    main()
