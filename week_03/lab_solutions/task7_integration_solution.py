#!/usr/bin/env python3
"""
Solution: Task 7 - Full Robot System Integration
==================================================
Five interconnected nodes: Sensor, Localization, Planner, Controller, Monitor.
"""

import sys, os, math, random
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
import ros2_sim as rclpy
from ros2_sim import (
    Node, Pose, Twist, LaserScan, Imu, Odometry, Float64,
    Vector3, Point, Quaternion, Header, TransformRequest, TransformResponse,
    QOS_SENSOR_DATA,
)


# ======================================================================
# SensorNode
# ======================================================================

def create_sensor_node() -> Node:
    node = Node('sensor_node')
    node.imu_pub = node.create_publisher(Imu, '/imu/data', qos=QOS_SENSOR_DATA)
    node.scan_pub = node.create_publisher(LaserScan, '/scan', qos=QOS_SENSOR_DATA)
    node.imu_step = 0
    node.scan_step = 0

    def imu_cb():
        msg = Imu(
            header=Header(frame_id='imu_link'),
            angular_velocity=Vector3(x=0.0, y=0.0, z=0.1 + random.gauss(0, 0.01)),
            linear_acceleration=Vector3(x=0.5 + random.gauss(0, 0.05), y=0.0, z=9.81),
        )
        node.imu_pub.publish(msg)
        node.imu_step += 1

    def scan_cb():
        scan = LaserScan(
            header=Header(frame_id='laser_frame'),
            angle_min=-math.pi,
            angle_max=math.pi,
            angle_increment=2.0 * math.pi / 360,
            range_min=0.1,
            range_max=30.0,
        )
        scan.ranges = [
            5.0 + 3.0 * math.sin(i * scan.angle_increment + node.scan_step * 0.05)
            + random.gauss(0, 0.1)
            for i in range(360)
        ]
        node.scan_pub.publish(scan)
        node.scan_step += 1

    node.create_timer(0.02, imu_cb)   # 50 Hz
    node.create_timer(0.1, scan_cb)   # 10 Hz
    node.get_logger().info("SensorNode created: /imu/data @50Hz, /scan @10Hz")
    return node


# ======================================================================
# LocalizationNode
# ======================================================================

def create_localization_node() -> Node:
    node = Node('localization_node')
    node.x = 0.0
    node.y = 0.0
    node.theta = 0.0
    node.latest_omega = 0.0
    node.odom_pub = node.create_publisher(Odometry, '/odom', qos=10)

    def imu_cb(msg: Imu):
        node.latest_omega = msg.angular_velocity.z

    node.create_subscription(Imu, '/imu/data', imu_cb, qos=QOS_SENSOR_DATA)

    def odom_cb():
        dt = 0.05  # 20 Hz
        node.theta += node.latest_omega * dt
        speed = 0.5  # assumed constant forward speed
        node.x += speed * math.cos(node.theta) * dt
        node.y += speed * math.sin(node.theta) * dt

        odom = Odometry(
            header=Header(frame_id='odom'),
            child_frame_id='base_link',
            pose=Pose(
                position=Point(x=node.x, y=node.y, z=0.0),
                orientation=Quaternion(z=math.sin(node.theta / 2), w=math.cos(node.theta / 2)),
            ),
            twist=Twist(
                linear=Vector3(x=speed, y=0.0, z=0.0),
                angular=Vector3(z=node.latest_omega),
            ),
        )
        node.odom_pub.publish(odom)

    node.create_timer(0.05, odom_cb)  # 20 Hz

    # Reset service
    def reset_cb(request):
        node.x = 0.0
        node.y = 0.0
        node.theta = 0.0
        node.get_logger().info("Odometry reset to origin")
        return TransformResponse(success=True)

    node.create_service(None, '/reset_odom', reset_cb)
    node.get_logger().info("LocalizationNode created: /odom @20Hz, service /reset_odom")
    return node


# ======================================================================
# PlannerNode
# ======================================================================

def create_planner_node() -> Node:
    node = Node('planner_node')
    node.goal = (10.0, 10.0)
    node.latest_odom = None
    node.latest_scan = None
    node.cmd_pub = node.create_publisher(Twist, '/cmd_vel', qos=10)

    def odom_cb(msg: Odometry):
        node.latest_odom = msg

    def scan_cb(msg: LaserScan):
        node.latest_scan = msg

    node.create_subscription(Odometry, '/odom', odom_cb, qos=10)
    node.create_subscription(LaserScan, '/scan', scan_cb, qos=QOS_SENSOR_DATA)

    def plan_cb():
        cmd = Twist()
        if node.latest_odom is None:
            node.cmd_pub.publish(cmd)
            return

        x = node.latest_odom.pose.position.x
        y = node.latest_odom.pose.position.y
        # Extract theta from quaternion z component (simplified)
        theta = 2.0 * math.asin(max(-1.0, min(1.0, node.latest_odom.pose.orientation.z)))

        # Check obstacle
        obstacle = False
        if node.latest_scan and node.latest_scan.ranges:
            front_ranges = node.latest_scan.ranges[170:190]  # roughly forward
            if front_ranges and min(front_ranges) < 1.5:
                obstacle = True

        if obstacle:
            cmd.linear = Vector3(x=0.1)
            cmd.angular = Vector3(z=0.5)
        else:
            dx = node.goal[0] - x
            dy = node.goal[1] - y
            dist = math.sqrt(dx * dx + dy * dy)
            angle_to_goal = math.atan2(dy, dx)
            angle_err = angle_to_goal - theta
            # Wrap to [-pi, pi]
            angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))
            cmd.linear = Vector3(x=min(0.5, 0.3 * dist))
            cmd.angular = Vector3(z=max(-1.0, min(1.0, 1.5 * angle_err)))

        node.cmd_pub.publish(cmd)

    node.create_timer(0.1, plan_cb)  # 10 Hz
    node.get_logger().info(f"PlannerNode created: goal={node.goal}, /cmd_vel @10Hz")
    return node


# ======================================================================
# ControllerNode
# ======================================================================

def create_controller_node() -> Node:
    node = Node('controller_node')
    node.declare_parameter('max_linear_speed', 1.0)
    node.declare_parameter('max_angular_speed', 2.0)
    node.current_linear = 0.0
    node.current_angular = 0.0
    node.state_pub = node.create_publisher(Float64, '/controller/state', qos=10)

    def cmd_cb(msg: Twist):
        max_lin = node.get_parameter('max_linear_speed').value
        max_ang = node.get_parameter('max_angular_speed').value
        node.current_linear = max(-max_lin, min(max_lin, msg.linear.x))
        node.current_angular = max(-max_ang, min(max_ang, msg.angular.z))

    node.create_subscription(Twist, '/cmd_vel', cmd_cb, qos=10)

    def state_cb():
        speed = math.sqrt(node.current_linear ** 2 + node.current_angular ** 2)
        node.state_pub.publish(Float64(data=speed))

    node.create_timer(0.05, state_cb)  # 20 Hz
    node.get_logger().info("ControllerNode created: /controller/state @20Hz")
    return node


# ======================================================================
# MonitorNode
# ======================================================================

def create_monitor_node() -> Node:
    node = Node('monitor_node')
    node.odom_log = []
    node.speed_log = []
    node.msg_count = 0

    def odom_cb(msg: Odometry):
        node.odom_log.append((msg.pose.position.x, msg.pose.position.y))
        node.msg_count += 1

    def speed_cb(msg: Float64):
        node.speed_log.append(msg.data)
        node.msg_count += 1

    node.create_subscription(Odometry, '/odom', odom_cb, qos=10)
    node.create_subscription(Float64, '/controller/state', speed_cb, qos=10)
    node.get_logger().info("MonitorNode created: logging /odom and /controller/state")
    return node


# ======================================================================
# System runner and evaluator
# ======================================================================

def run_robot_system(duration_sec: float = 5.0) -> list:
    sensor = create_sensor_node()
    localization = create_localization_node()
    planner = create_planner_node()
    controller = create_controller_node()
    monitor = create_monitor_node()
    nodes = [sensor, localization, planner, controller, monitor]

    print(f"\nSpinning {len(nodes)} nodes for {duration_sec}s ...\n")
    rclpy.spin_nodes(nodes, duration_sec=duration_sec)
    return nodes


def evaluate_system(nodes: list) -> dict:
    # Find monitor node
    monitor = next((n for n in nodes if n.get_name() == 'monitor_node'), None)
    localization = next((n for n in nodes if n.get_name() == 'localization_node'), None)

    total_msgs = monitor.msg_count if monitor else 0
    final_x = localization.x if localization else 0.0
    final_y = localization.y if localization else 0.0
    goal = (10.0, 10.0)
    dist = math.sqrt((goal[0] - final_x) ** 2 + (goal[1] - final_y) ** 2)
    avg_speed = 0.0
    if monitor and monitor.speed_log:
        avg_speed = sum(monitor.speed_log) / len(monitor.speed_log)

    return {
        'total_messages': total_msgs,
        'final_position': (round(final_x, 3), round(final_y, 3)),
        'distance_to_goal': round(dist, 3),
        'avg_speed': round(avg_speed, 4),
        'nodes_active': len(nodes),
    }


def main():
    print("=" * 60)
    print("Task 7 Solution: Full Robot System Integration")
    print("=" * 60)
    print()
    print("ROS2 Concepts demonstrated:")
    print("  - Multi-node architecture (5 nodes)")
    print("  - Mixed pub/sub and service communication")
    print("  - Parameters for controller tuning")
    print("  - System-level evaluation")
    print()
    print("Topic map:")
    print("  /imu/data   (Imu)       SensorNode -> LocalizationNode")
    print("  /scan        (LaserScan) SensorNode -> PlannerNode")
    print("  /odom        (Odometry)  LocalizationNode -> PlannerNode, MonitorNode")
    print("  /cmd_vel     (Twist)     PlannerNode -> ControllerNode")
    print("  /controller/state (Float64) ControllerNode -> MonitorNode")
    print()

    rclpy.init()
    nodes = run_robot_system(5.0)
    results = evaluate_system(nodes)

    print("\n=== Robot System Evaluation ===")
    for k, v in results.items():
        print(f"  {k}: {v}")

    # Save trajectory plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        monitor = next((n for n in nodes if n.get_name() == 'monitor_node'), None)
        if monitor and monitor.odom_log:
            xs = [p[0] for p in monitor.odom_log]
            ys = [p[1] for p in monitor.odom_log]
            fig, ax = plt.subplots(figsize=(8, 8))
            ax.plot(xs, ys, 'b-', linewidth=1, label='Robot path')
            ax.plot(xs[0], ys[0], 'go', markersize=10, label='Start')
            ax.plot(10, 10, 'r*', markersize=15, label='Goal')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title('Robot Trajectory')
            ax.legend()
            ax.grid(True)
            ax.set_aspect('equal')
            plot_path = os.path.join(os.path.dirname(__file__), 'task7_trajectory.png')
            fig.savefig(plot_path, dpi=100, bbox_inches='tight')
            print(f"\nPlot saved to {plot_path}")
    except ImportError:
        print("\nmatplotlib not available - skipping plot.")

    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
