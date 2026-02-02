#!/usr/bin/env python3
"""Week 7 Solution 3: Geofence Monitor Node (Complete)"""
import json, math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String


class GeofenceMonitorNode(Node):
    def __init__(self):
        super().__init__('geofence_monitor')
        self.declare_parameter('fence_center_x', 0.0)
        self.declare_parameter('fence_center_y', 0.0)
        self.declare_parameter('fence_radius', 50.0)
        self.declare_parameter('fence_max_height', 30.0)
        self.cx = self.get_parameter('fence_center_x').value
        self.cy = self.get_parameter('fence_center_y').value
        self.radius = self.get_parameter('fence_radius').value
        self.max_h = self.get_parameter('fence_max_height').value
        self.sub_odom = self.create_subscription(
            Odometry, '/fmu/out/vehicle_odometry', self.odom_cb, 10)
        self.pub_status = self.create_publisher(Bool, '/geofence_status', 10)
        self.pub_dist = self.create_publisher(Float64, '/distance_to_boundary', 10)
        self.pub_cmd = self.create_publisher(String, '/fmu/in/vehicle_command', 10)
        self.breach_triggered = False
        self.get_logger().info(
            f'Geofence: center=({self.cx},{self.cy}), r={self.radius}, h={self.max_h}')

    def odom_cb(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        horiz_dist = math.sqrt((px-self.cx)**2 + (py-self.cy)**2)
        altitude = -pz
        horiz_margin = self.radius - horiz_dist
        vert_margin = self.max_h - altitude
        ground_margin = altitude
        dist_to_boundary = min(horiz_margin, vert_margin, ground_margin)
        inside = horiz_margin > 0 and vert_margin > 0 and altitude >= 0
        status_msg = Bool()
        status_msg.data = inside
        self.pub_status.publish(status_msg)
        dist_msg = Float64()
        dist_msg.data = dist_to_boundary
        self.pub_dist.publish(dist_msg)
        if not inside and not self.breach_triggered:
            self.breach_triggered = True
            cmd = String()
            cmd.data = json.dumps({
                'command': 'VEHICLE_CMD_NAV_RETURN_TO_LAUNCH',
                'reason': 'geofence_breach',
                'position': {'x': px, 'y': py, 'z': pz},
                'horiz_dist': round(horiz_dist, 2),
                'altitude': round(altitude, 2)})
            self.pub_cmd.publish(cmd)
            self.get_logger().warn(
                f'GEOFENCE BREACH at ({px:.1f},{py:.1f},{pz:.1f}) - RTL commanded!')
        if inside and self.breach_triggered:
            self.breach_triggered = False
            self.get_logger().info('Back inside geofence')


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
