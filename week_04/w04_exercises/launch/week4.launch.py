from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    d=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg=os.path.join(d,"config","params.yaml")
    bag=os.path.join(d,"bag_data","synthetic_2d_slam")
    return LaunchDescription([
        ExecuteProcess(cmd=["ros2","bag","play",bag,"--rate","1.0"],output="screen"),
        Node(package="week_04_slam_2d",executable="exercise1_node",name="scan_matcher",parameters=[cfg],output="screen"),
        Node(package="week_04_slam_2d",executable="exercise2_node",name="occ_grid_builder",parameters=[cfg],output="screen"),
        Node(package="week_04_slam_2d",executable="exercise3_node",name="loop_closure",parameters=[cfg],output="screen"),
    ])
