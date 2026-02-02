#!/usr/bin/env python3
"""Week 8 Solution 3: Trajectory Smoother (Complete)"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA


class TrajectorySmootherNode(Node):
    def __init__(self):
        super().__init__('trajectory_smoother')
        self.declare_parameter('smoothing_factor', 0.5)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('max_acceleration', 1.0)
        self.smoothing = self.get_parameter('smoothing_factor').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.max_acc = self.get_parameter('max_acceleration').value
        self.create_subscription(Path, '/planned_path_3d', self.path_cb, 10)
        self.pub_smooth = self.create_publisher(Path, '/smooth_trajectory', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/trajectory_markers', 10)
        self.get_logger().info('Trajectory Smoother ready')

    def path_cb(self, msg):
        if len(msg.poses) < 2:
            return
        wps = np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])
        try:
            from scipy.interpolate import splprep, splev
            k = min(3, len(wps) - 1)
            tck, u = splprep([wps[:,0], wps[:,1], wps[:,2]], s=self.smoothing, k=k)
            u_fine = np.linspace(0, 1, max(len(wps) * 10, 50))
            smooth = np.array(splev(u_fine, tck)).T
        except ImportError:
            self.get_logger().warn('scipy not available, using linear interpolation')
            smooth = self._linear_interp(wps, len(wps) * 10)
        # Velocity profiling
        diffs = np.diff(smooth, axis=0)
        seg_lens = np.linalg.norm(diffs, axis=1)
        cum_len = np.concatenate([[0], np.cumsum(seg_lens)])
        total_len = cum_len[-1]
        # Trapezoidal profile
        accel_dist = self.max_vel**2 / (2 * self.max_acc)
        if 2 * accel_dist > total_len:
            accel_dist = total_len / 2
        velocities = np.zeros(len(smooth))
        for i, s in enumerate(cum_len):
            if s < accel_dist:
                velocities[i] = min(self.max_vel, np.sqrt(2 * self.max_acc * max(s, 0.001)))
            elif s > total_len - accel_dist:
                velocities[i] = min(self.max_vel, np.sqrt(2 * self.max_acc * max(total_len - s, 0.001)))
            else:
                velocities[i] = self.max_vel
        # Publish smoothed path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        for pt in smooth:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(pt[0]), float(pt[1]), float(pt[2])
            path_msg.poses.append(ps)
        self.pub_smooth.publish(path_msg)
        self.get_logger().info(f'Smoothed path: {len(smooth)} points, length={total_len:.2f}m')
        # Publish markers
        ma = MarkerArray()
        line = Marker()
        line.header = path_msg.header
        line.ns, line.id = 'smooth', 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        for pt in smooth:
            line.points.append(Point(x=float(pt[0]),y=float(pt[1]),z=float(pt[2])))
        ma.markers.append(line)
        self.pub_markers.publish(ma)

    def _linear_interp(self, wps, n):
        diffs = np.diff(wps, axis=0)
        seg_lens = np.linalg.norm(diffs, axis=1)
        cum = np.concatenate([[0], np.cumsum(seg_lens)])
        total = cum[-1]
        result = []
        for t in np.linspace(0, total, n):
            idx = np.searchsorted(cum, t, side='right') - 1
            idx = np.clip(idx, 0, len(wps) - 2)
            alpha = (t - cum[idx]) / max(seg_lens[idx], 1e-6)
            result.append(wps[idx] + alpha * diffs[idx])
        return np.array(result)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrajectorySmootherNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
