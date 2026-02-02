# Week 7: PX4/ArduPilot and ROS2 Integration

## Overview
Integration between ROS2 and PX4/ArduPilot for autonomous drones.
Companies like Skydio, DJI, Auterion build on PX4-based stacks.
The PX4-ROS2 bridge via micro-XRCE-DDS enables direct DDS communication.

## Dataset
Synthetic PX4 SITL bag from generate_bag.py. Simulates takeoff, circle orbit, landing.

## Exercises
1. PX4 Telemetry Monitor - parse odometry and status, publish summary
2. Offboard Commander - waypoint-following state machine
3. Geofence Monitor - cylindrical geofence with breach detection and RTL

## How to Run
bash download_data.sh
ros2 launch ros2_exercises/launch/exercise1.launch.py
