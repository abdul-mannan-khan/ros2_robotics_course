#!/usr/bin/env python3
"""Generate synthetic Nav2 bag data for Week 5 exercises."""
import os, math, shutil
import numpy as np
import rclpy
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
import rosbag2_py
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "bag_data", "synthetic_nav2")

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

def ray_cast_room(angle, robot_x, robot_y):
    walls = [
        ((-4.5,-4.5),(-4.5,4.5)), ((4.5,-4.5),(4.5,4.5)),
        ((-4.5,-4.5),(4.5,-4.5)), ((-4.5,4.5),(4.5,4.5)),
    ]
    obstacles = [
        [(-1,-1),(1,-1),(1,1),(-1,1)],
        [(2,2),(3,2),(3,3),(2,3)],
        [(-3,1),(-2,1),(-2,3),(-3,3)],
    ]
    for obs in obstacles:
        for i in range(len(obs)):
            walls.append((obs[i], obs[(i+1)%len(obs)]))
    dx, dy = math.cos(angle), math.sin(angle)
    min_dist = 30.0
    for (x1,y1),(x2,y2) in walls:
        denom = dx*(y1-y2) - dy*(x1-x2)
        if abs(denom) < 1e-10: continue
        t = ((x1-robot_x)*(y1-y2)-(y1-robot_y)*(x1-x2))/denom
        u = -(dx*(y1-robot_y)-dy*(x1-robot_x))/denom
        if t > 0.01 and 0.0 <= u <= 1.0:
            min_dist = min(min_dist, t)
    return min_dist

def main():
    print("=== Week 5: Generating synthetic Nav2 bag ===")
    print(f"Output: {OUTPUT_DIR}")

    scan_msgs = []
    for i in range(150):
        ts = i * 200000000
        s = LaserScan(); s.header = make_header(ts, "base_scan")
        s.angle_min = 0.0; s.angle_max = 2*math.pi; s.angle_increment = 2*math.pi/360
        s.scan_time = 0.2; s.range_min = 0.12; s.range_max = 30.0
        ranges = []
        for r in range(360):
            d = ray_cast_room(r * s.angle_increment, -3.0, -3.0) + np.random.normal(0, 0.02)
            ranges.append(max(s.range_min, min(s.range_max, d)))
        s.ranges = [float(v) for v in ranges]; s.intensities = [100.0] * 360
        scan_msgs.append((ts, s))
    print(f"  [/scan] {len(scan_msgs)} messages")

    odom_msgs = []
    for i in range(600):
        ts = i * 50000000
        o = Odometry(); o.header = make_header(ts, "odom"); o.child_frame_id = "base_footprint"
        o.pose.pose.position = Point(x=-3.0, y=-3.0, z=0.0)
        o.pose.pose.orientation = Quaternion(w=1.0)
        odom_msgs.append((ts, o))
    print(f"  [/odom] {len(odom_msgs)} messages")

    g = OccupancyGrid(); g.header = make_header(0, "map")
    g.info = MapMetaData(); g.info.resolution = 0.05; g.info.width = 200; g.info.height = 200
    g.info.origin = Pose()
    g.info.origin.position = Point(x=-5.0, y=-5.0, z=0.0)
    g.info.origin.orientation = Quaternion(w=1.0)
    data = np.zeros((200, 200), dtype=np.int8)
    data[0, :] = 100; data[-1, :] = 100; data[:, 0] = 100; data[:, -1] = 100
    def w2g(wx, wy):
        return max(0, min(199, int((wx+5)/0.05))), max(0, min(199, int((wy+5)/0.05)))
    for (ax, ay), (bx, by) in [((-1,-1),(1,1)), ((2,2),(3,3)), ((-3,1),(-2,3))]:
        gx1, gy1 = w2g(ax, ay); gx2, gy2 = w2g(bx, by)
        data[gy1:gy2, gx1:gx2] = 100
    gx1, gy1 = w2g(3.5, -4.5); gx2, gy2 = w2g(4.5, -3.5)
    data[gy1:gy2, gx1:gx2] = -1
    g.data = data.flatten().tolist()
    print("  [/map] 1 message (200x200)")

    topics = [
        ("/scan", "sensor_msgs/msg/LaserScan", scan_msgs),
        ("/odom", "nav_msgs/msg/Odometry", odom_msgs),
        ("/map", "nav_msgs/msg/OccupancyGrid", [(0, g)]),
    ]
    print("  Writing bag...")
    write_bag(OUTPUT_DIR, topics)
    print("  Done!")

if __name__ == "__main__":
    main()
