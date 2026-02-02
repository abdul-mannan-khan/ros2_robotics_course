#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "params.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="week_11_control_systems",
            executable="exercise1_node",
            name="pid_position_controller_node",
            output="screen",
            parameters=[config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
