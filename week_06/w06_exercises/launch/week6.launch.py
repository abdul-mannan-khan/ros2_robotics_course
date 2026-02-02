from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    d=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg=os.path.join(d,"config","params.yaml")
    bag=os.path.join(d,"bag_data","synthetic_quadrotor")
    return LaunchDescription([
        ExecuteProcess(cmd=["ros2","bag","play",bag,"--rate","1.0"],output="screen"),
        Node(package="week_06_quadrotor_dynamics",executable="exercise1_node",name="attitude_est",parameters=[cfg],output="screen"),
        Node(package="week_06_quadrotor_dynamics",executable="exercise2_node",name="thrust_mixer",parameters=[cfg],output="screen"),
        Node(package="week_06_quadrotor_dynamics",executable="exercise3_node",name="attitude_ctrl",parameters=[cfg],output="screen"),
    ])
