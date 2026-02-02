#!/usr/bin/env python3
"""
Week 8 Exercise 3: Trajectory Smoother Node

Subscribers: /planned_path_3d (Path)
Publishers: /smooth_trajectory (Path), /trajectory_markers (MarkerArray)
Parameters: smoothing_factor (0.5), max_velocity (2.0), max_acceleration (1.0)
"""
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
        self.get_logger().info('Trajectory Smoother waiting for path...')

    def path_cb(self, msg):
        if len(msg.poses) < 2:
            return
        wps = np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])

        # TODO: B-spline fitting and velocity profiling
        # 1. from scipy.interpolate import splprep, splev
        # 2. tck, u = splprep([wps[:,0],wps[:,1],wps[:,2]], s=self.smoothing)
        # 3. u_fine = np.linspace(0, 1, len(wps)*10)
        # 4. smooth = np.array(splev(u_fine, tck)).T
        # 5. Compute arc length, apply trapezoidal velocity profile
        # 6. Publish smoothed Path and MarkerArray visualization

        self.get_logger().warn('B-spline smoothing not implemented - TODO')
        self.pub_smooth.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySmootherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
