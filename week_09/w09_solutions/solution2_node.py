#!/usr/bin/env python3
"""Week 9 Solution 2: Local Planner (Complete)"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')
        self.declare_parameter('goal_xyz', [20.0, 0.0, 1.5])
        self.declare_parameter('planning_horizon', 5.0)
        self.declare_parameter('safety_margin', 0.5)
        self.goal = np.array(self.get_parameter('goal_xyz').value)
        self.horizon = self.get_parameter('planning_horizon').value
        self.margin = self.get_parameter('safety_margin').value
        self.pose = None
        self.obstacles = None
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PointCloud2, '/local_pointcloud', self.cloud_cb, 10)
        self.pub_traj = self.create_publisher(Path, '/local_trajectory', 10)
        self.pub_df = self.create_publisher(PointCloud2, '/distance_field', 10)
        self.create_timer(0.1, self.plan)
        self.get_logger().info('Local Planner started')

    def odom_cb(self, msg):
        self.pose = msg.pose.pose

    def cloud_cb(self, msg):
        if msg.width > 0:
            self.obstacles = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :3]

    def compute_distance_field(self, center, size=5.0, res=0.5):
        n = int(size / res)
        grid = np.zeros((n, n, n), dtype=np.float32)
        pts = []
        for i in range(n):
            for j in range(n):
                for k in range(n):
                    x = center[0] - size/2 + i*res
                    y = center[1] - size/2 + j*res
                    z = center[2] - size/2 + k*res
                    pt = np.array([x, y, z])
                    if self.obstacles is not None and len(self.obstacles) > 0:
                        d = np.min(np.linalg.norm(self.obstacles - pt, axis=1))
                    else:
                        d = size
                    grid[i, j, k] = d
                    pts.append([x, y, z, d])
        return grid, np.array(pts, dtype=np.float32)

    def min_jerk_trajectory(self, start, end, n_pts=20):
        t = np.linspace(0, 1, n_pts)
        s = 10*t**3 - 15*t**4 + 6*t**5
        traj = start[None,:] + s[:,None] * (end - start)[None,:]
        return traj

    def plan(self):
        if self.pose is None:
            return
        pos = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
        direction = self.goal - pos
        dist = np.linalg.norm(direction)
        if dist < 0.5:
            return
        if dist > self.horizon:
            local_goal = pos + direction / dist * self.horizon
        else:
            local_goal = self.goal
        traj = self.min_jerk_trajectory(pos, local_goal, 20)
        # Push trajectory away from obstacles
        if self.obstacles is not None and len(self.obstacles) > 0:
            for i in range(len(traj)):
                dists = np.linalg.norm(self.obstacles - traj[i], axis=1)
                min_idx = np.argmin(dists)
                min_d = dists[min_idx]
                if min_d < self.margin:
                    push = traj[i] - self.obstacles[min_idx]
                    push_norm = np.linalg.norm(push)
                    if push_norm > 1e-6:
                        traj[i] += push / push_norm * (self.margin - min_d)
        # Publish trajectory
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        for pt in traj:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(pt[0]), float(pt[1]), float(pt[2])
            path.poses.append(ps)
        self.pub_traj.publish(path)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LocalPlannerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
