from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    d=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg=os.path.join(d,"config","params.yaml")
    bag=os.path.join(d,"bag_data","synthetic_nav2")
    return LaunchDescription([
        ExecuteProcess(cmd=["ros2","bag","play",bag,"--rate","1.0"],output="screen"),
        Node(package="week_05_nav2",executable="exercise1_node",name="costmap_gen",parameters=[cfg],output="screen"),
        Node(package="week_05_nav2",executable="exercise2_node",name="astar_planner",parameters=[cfg],output="screen"),
        Node(package="week_05_nav2",executable="exercise3_node",name="path_follower",parameters=[cfg],output="screen"),
    ])
