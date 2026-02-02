#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    bag_path = os.path.join(pkg_dir, "bag_data", "px4_sitl_bag")
    config_path = os.path.join(pkg_dir, "config", "params.yaml")
    return LaunchDescription([
        ExecuteProcess(cmd=["ros2","bag","play",bag_path,"--loop","--rate","1.0"],output="screen"),
        Node(package="week_07_px4_integration",executable="exercise1_node",
             name="px4_telemetry_monitor",parameters=[config_path],output="screen"),
    ])
