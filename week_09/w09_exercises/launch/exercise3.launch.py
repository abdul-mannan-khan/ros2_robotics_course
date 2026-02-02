#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    d=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    b=os.path.join(d,"bag_data","ego_planner_sim")
    cfg=os.path.join(d,"config","params.yaml")
    return LaunchDescription([ExecuteProcess(cmd=["ros2","bag","play",b,"--loop"],output="screen"),Node(package="week_09_ego_planner",executable="exercise3_node",name="trajectory_tracker",parameters=[cfg],output="screen")])
