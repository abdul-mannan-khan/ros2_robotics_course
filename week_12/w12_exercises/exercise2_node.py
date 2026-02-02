#!/usr/bin/env python3
"""Week 12 Exercise 2: Mission Planner Node
Subscribe to /odom, /fused_perception, /map.
High-level mission: visit N waypoints, avoid obstacles, return home.
Publish /current_goal, /mission_status, /mission_path.

TODO: Finite state machine for mission execution.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Header
import numpy as np
import json
from enum import Enum


class MissionState(Enum):
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    NAVIGATE = "NAVIGATE"
    AT_WAYPOINT = "AT_WAYPOINT"
    RETURN_HOME = "RETURN_HOME"
    LANDED = "LANDED"
    EMERGENCY = "EMERGENCY"


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("mission_planner_node")
        self.declare_parameter("mission_waypoints",
            [5.0, 0.0, 2.0, 0.0, 5.0, 2.0, -5.0, 0.0, 2.0, 0.0, -5.0, 2.0])
        self.declare_parameter("mission_type", "survey")
        self.declare_parameter("waypoint_threshold", 0.5)
        self.declare_parameter("hover_altitude", 2.0)

        wp_flat = self.get_parameter("mission_waypoints").value
        self.waypoints = [wp_flat[i:i+3] for i in range(0, len(wp_flat), 3)]
        self.mission_type = self.get_parameter("mission_type").value
        self.wp_thresh = self.get_parameter("waypoint_threshold").value

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.perception_sub = self.create_subscription(PointCloud2, "/fused_perception", self.perception_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)

        self.goal_pub = self.create_publisher(PoseStamped, "/current_goal", 10)
        self.status_pub = self.create_publisher(String, "/mission_status", 10)
        self.path_pub = self.create_publisher(Path, "/mission_path", 10)

        self.state = MissionState.IDLE
        self.current_wp_idx = 0
        self.current_pos = np.zeros(3)
        self.home_pos = None

        self.timer = self.create_timer(0.1, self.mission_tick)
        self.get_logger().info(f"Mission Planner: type={self.mission_type}, {len(self.waypoints)} waypoints")

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z])
        if self.home_pos is None:
            self.home_pos = self.current_pos.copy()

    def perception_callback(self, msg):
        pass  # Could check for obstacles

    def map_callback(self, msg):
        pass  # Could update obstacle map

    def mission_tick(self):
        # ============================================================
        # TODO: Implement finite state machine
        # States: IDLE -> TAKEOFF -> NAVIGATE -> AT_WAYPOINT -> ... -> RETURN_HOME -> LANDED
        # IDLE: Wait for start command or auto-start
        # TAKEOFF: Go to hover altitude
        # NAVIGATE: Move to current waypoint
        # AT_WAYPOINT: Perform action (e.g., hover for survey), then next wp
        # RETURN_HOME: Navigate back to home position
        # LANDED: Mission complete
        #
        # Transitions based on distance to goal < waypoint_threshold
        # ============================================================

        # Publish status
        status = {
            "state": self.state.value,
            "waypoint_idx": self.current_wp_idx,
            "total_waypoints": len(self.waypoints),
            "position": self.current_pos.tolist()
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

        # Publish current goal
        if self.current_wp_idx < len(self.waypoints):
            wp = self.waypoints[self.current_wp_idx]
            goal = PoseStamped()
            goal.header = Header(frame_id="world")
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position = Point(x=wp[0], y=wp[1], z=wp[2])
            goal.pose.orientation = Quaternion(w=1.0)
            self.goal_pub.publish(goal)

        # Publish mission path
        path = Path()
        path.header = Header(frame_id="world")
        path.header.stamp = self.get_clock().now().to_msg()
        for wp in self.waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position = Point(x=wp[0], y=wp[1], z=wp[2])
            path.poses.append(ps)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
