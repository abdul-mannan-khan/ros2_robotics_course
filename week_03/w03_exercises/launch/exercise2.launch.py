#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    bag=os.path.join(pkg,"bag_data","synthetic_turtlebot3")
    cfg=os.path.join(pkg,"config","params.yaml")
    return LaunchDescription([
        ExecuteProcess(cmd=["ros2","bag","play",bag,"--loop"],output="screen"),
        Node(package="week_03_ros2_fundamentals",executable="exercise2_node",name="exercise2_node",parameters=[cfg],output="screen"),
    ])
