#!/usr/bin/env python3
"""Demo launch: bag playback + 3 solution nodes + RViz2 for week 02."""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Use source directory (not installed) so bag data is found
    src_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))),
    )
    bag_dir = os.path.join(src_dir, 'w02_exercises', 'bag_data', 'synthetic_euroc')
    config = os.path.join(src_dir, 'w02_exercises', 'config', 'params.yaml')
    rviz_config = os.path.join(src_dir, 'w02_exercises', 'config', 'rviz_config.rviz')

    # Filter snap paths from LD_LIBRARY_PATH to avoid rviz2 crash
    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    clean_ld = ':'.join(p for p in ld_path.split(':') if 'snap' not in p)

    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', clean_ld),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_dir, '--rate', '2.0', '--loop'],
            output='screen',
        ),
        TimerAction(period=2.0, actions=[
            Node(
                package='week_02_sensor_fusion',
                executable='solution1_node',
                name='solution1',
                parameters=[config],
                output='screen',
            ),
            Node(
                package='week_02_sensor_fusion',
                executable='solution2_node',
                name='solution2',
                parameters=[config],
                output='screen',
            ),
            Node(
                package='week_02_sensor_fusion',
                executable='solution3_node',
                name='solution3',
                parameters=[config],
                output='screen',
            ),
        ]),
        TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config],
                output='screen',
            ),
        ]),
    ])
