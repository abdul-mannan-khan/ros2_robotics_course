#!/usr/bin/env python3
"""Week 12 Solution 2: Mission Planner (Complete FSM)"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Header
import numpy as np
import json
from enum import Enum


class State(Enum):
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    NAVIGATE = "NAVIGATE"
    AT_WAYPOINT = "AT_WAYPOINT"
    RETURN_HOME = "RETURN_HOME"
    LANDED = "LANDED"


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("mission_planner_node")
        self.declare_parameter("mission_waypoints",
            [5.0,0.0,2.0, 0.0,5.0,2.0, -5.0,0.0,2.0, 0.0,-5.0,2.0])
        self.declare_parameter("mission_type", "survey")
        self.declare_parameter("waypoint_threshold", 0.5)
        self.declare_parameter("hover_altitude", 2.0)
        self.declare_parameter("hover_time", 3.0)

        wp_flat = self.get_parameter("mission_waypoints").value
        self.waypoints = [wp_flat[i:i+3] for i in range(0, len(wp_flat), 3)]
        self.wp_thresh = self.get_parameter("waypoint_threshold").value
        self.hover_alt = self.get_parameter("hover_altitude").value
        self.hover_time = self.get_parameter("hover_time").value

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/current_goal", 10)
        self.status_pub = self.create_publisher(String, "/mission_status", 10)
        self.path_pub = self.create_publisher(Path, "/mission_path", 10)

        self.state = State.IDLE
        self.wp_idx = 0
        self.pos = np.zeros(3)
        self.home = None
        self.state_enter_time = None
        self.started = False

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(f"Mission: {len(self.waypoints)} waypoints")

    def odom_cb(self, msg):
        self.pos = np.array([msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z])
        if self.home is None:
            self.home = self.pos.copy()
        if not self.started:
            self.started = True
            self.transition(State.TAKEOFF)

    def transition(self, new_state):
        self.get_logger().info(f"State: {self.state.value} -> {new_state.value}")
        self.state = new_state
        self.state_enter_time = self.get_clock().now()

    def elapsed(self):
        if self.state_enter_time is None:
            return 0.0
        return (self.get_clock().now() - self.state_enter_time).nanoseconds * 1e-9

    def dist_to(self, target):
        return np.linalg.norm(self.pos - np.array(target))

    def tick(self):
        now = self.get_clock().now().to_msg()
        goal_pos = None

        if self.state == State.IDLE:
            pass

        elif self.state == State.TAKEOFF:
            goal_pos = [self.home[0], self.home[1], self.hover_alt]
            if self.dist_to(goal_pos) < self.wp_thresh:
                self.wp_idx = 0
                self.transition(State.NAVIGATE)

        elif self.state == State.NAVIGATE:
            if self.wp_idx < len(self.waypoints):
                goal_pos = self.waypoints[self.wp_idx]
                if self.dist_to(goal_pos) < self.wp_thresh:
                    self.transition(State.AT_WAYPOINT)
            else:
                self.transition(State.RETURN_HOME)

        elif self.state == State.AT_WAYPOINT:
            goal_pos = self.waypoints[self.wp_idx] if self.wp_idx < len(self.waypoints) else self.home.tolist()
            if self.elapsed() > self.hover_time:
                self.wp_idx += 1
                if self.wp_idx < len(self.waypoints):
                    self.transition(State.NAVIGATE)
                else:
                    self.transition(State.RETURN_HOME)

        elif self.state == State.RETURN_HOME:
            goal_pos = [self.home[0], self.home[1], self.hover_alt]
            if self.dist_to(goal_pos) < self.wp_thresh:
                self.transition(State.LANDED)

        elif self.state == State.LANDED:
            goal_pos = self.home.tolist() if self.home is not None else [0,0,0]

        # Publish goal
        if goal_pos is not None:
            g = PoseStamped()
            g.header = Header(stamp=now, frame_id="world")
            g.pose.position = Point(x=goal_pos[0], y=goal_pos[1], z=goal_pos[2])
            g.pose.orientation = Quaternion(w=1.0)
            self.goal_pub.publish(g)

        # Publish status
        s = json.dumps({"state": self.state.value, "wp": self.wp_idx,
                        "total": len(self.waypoints), "pos": self.pos.tolist()})
        self.status_pub.publish(String(data=s))

        # Publish path
        path = Path()
        path.header = Header(stamp=now, frame_id="world")
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
