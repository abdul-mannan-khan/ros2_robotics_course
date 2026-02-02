#!/usr/bin/env python3
"""Week 7 Solution 2: Offboard Commander Node (Complete)"""
import json, math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class OffboardCommanderNode(Node):
    def __init__(self):
        super().__init__('offboard_commander')
        self.declare_parameter('waypoints', [
            '[0.0,0.0,-5.0]','[10.0,0.0,-5.0]',
            '[10.0,10.0,-5.0]','[0.0,10.0,-5.0]'])
        self.declare_parameter('waypoint_radius', 1.0)
        self.declare_parameter('cruise_speed', 2.0)
        wp_strs = self.get_parameter('waypoints').value
        self.waypoints = [json.loads(w) for w in wp_strs]
        self.wp_radius = self.get_parameter('waypoint_radius').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.current_wp_idx = 0
        self.current_pose = None
        self.mission_complete = False
        self.sub_odom = self.create_subscription(
            Odometry, '/fmu/out/vehicle_odometry', self.odom_cb, 10)
        self.pub_setpoint = self.create_publisher(
            PoseStamped, '/fmu/in/trajectory_setpoint', 10)
        self.pub_mode = self.create_publisher(
            String, '/fmu/in/offboard_control_mode', 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f'Offboard Commander: {len(self.waypoints)} waypoints')

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        mode_msg = String()
        mode_msg.data = json.dumps({'mode':'OFFBOARD','position':True,'velocity':False})
        self.pub_mode.publish(mode_msg)
        if self.current_pose is None or self.mission_complete:
            return
        if self.current_wp_idx >= len(self.waypoints):
            self.mission_complete = True
            self.get_logger().info('Mission complete!')
            return
        target = self.waypoints[self.current_wp_idx]
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        cz = self.current_pose.position.z
        dx, dy, dz = target[0]-cx, target[1]-cy, target[2]-cz
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < self.wp_radius:
            self.get_logger().info(f'Reached waypoint {self.current_wp_idx}: {target}')
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints):
                self.mission_complete = True
                self.get_logger().info('All waypoints reached!')
                return
            target = self.waypoints[self.current_wp_idx]
        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = 'odom_ned'
        setpoint.pose.position.x = target[0]
        setpoint.pose.position.y = target[1]
        setpoint.pose.position.z = target[2]
        yaw = math.atan2(target[1]-cy, target[0]-cx)
        setpoint.pose.orientation.w = math.cos(yaw/2)
        setpoint.pose.orientation.z = math.sin(yaw/2)
        self.pub_setpoint.publish(setpoint)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardCommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
