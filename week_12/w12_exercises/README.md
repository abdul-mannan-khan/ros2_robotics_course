# Week 12: Capstone Full Integration

## Topic Overview
Integrate perception, control, planning, and monitoring into a complete system.

## Learning Objectives
- Fuse LiDAR, camera, IMU, and GPS data in a unified pipeline
- Implement a finite state machine mission planner
- Build system monitoring for production-quality robotics

## Dataset
**nuScenes Mini** or synthetic combined scenario with all sensor modalities.

## Industry Context
Integration skills are used at every robotics company. Waymo, Tesla,
Boston Dynamics all require engineers who can combine subsystems.

## Exercises

### Ex1: Multi-Sensor Fusion
- In: /lidar_points, /camera/image, /imu, /gps
- Out: /fused_perception, /sensor_status

### Ex2: Mission Planner
- In: /odom, /fused_perception, /map
- Out: /current_goal, /mission_status, /mission_path

### Ex3: System Monitor
- In: all major topics
- Out: /system_health, /diagnostics

## Run
ros2 bag play bag_data/capstone_scenario/
ros2 launch ros2_robotics_course exercise1.launch.py

See config/params.yaml for parameters.
