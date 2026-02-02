# Week 7: PX4/ArduPilot & ROS2 Integration - Lab Exercises

## Overview
These exercises teach PX4 flight controller integration with ROS2 using a pure-Python simulation layer. No actual PX4 or ROS2 installation is required.

## Prerequisites
- Python 3.8+
- numpy
- matplotlib

## Simulation Framework
`px4_sim.py` provides simulated PX4/MAVLink/MAVROS interfaces:
- **MAVLink messages**: Heartbeat, CommandLong, CommandAck, position/attitude/status
- **PX4SITL**: Simulated flight controller with realistic dynamics, flight modes, failsafes
- **MAVROSInterface**: Simulated MAVROS topics and services
- **OffboardController**: Helper for offboard mode (handles setpoint streaming requirement)

## Exercises

| Task | Topic | Key Concepts |
|------|-------|-------------|
| 1 | MAVLink Protocol | Message structure, serialization, GCS-vehicle communication |
| 2 | Flight Modes | Mode transitions, arming, MANUAL/STABILIZED/POSITION/OFFBOARD |
| 3 | Offboard Control | Setpoint streaming, position/velocity control, waypoint following |
| 4 | Mission Planning | Mission items, lawnmower survey, autonomous execution |
| 5 | Sensor Integration | Depth camera, LiDAR, obstacle detection, reactive avoidance |
| 6 | Failsafe Handling | Battery, GPS loss, comm loss, failsafe verification |
| 7 | Full Integration | Complete mission lifecycle, contingency handling, evaluation |

## Running
```bash
cd lab_exercises
python3 task1_mavlink_protocol.py
```

## Solutions
Complete solutions are in `../lab_solutions/`. Run them to generate output plots.
