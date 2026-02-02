# Week 3 Lab Exercises: ROS2 Fundamentals & Node Architecture

## Overview

These 7 exercises teach core ROS2 concepts using a pure-Python simulation layer (`ros2_sim.py`). No ROS2 installation is required -- just run with `python3`.

## Setup

```bash
cd week_03/lab_exercises
python3 task1_publisher.py   # run any exercise directly
```

## Exercises

| Task | Topic | Key Concepts |
|------|-------|--------------|
| 1 | Basic Publisher Node | Node, Publisher, Timer, spin |
| 2 | Subscriber with Processing | Subscription, callback, statistics |
| 3 | Multi-Node Pipeline | Chained pub/sub, QoS profiles |
| 4 | Service Server & Client | Service, Client, request/response |
| 5 | Parameter Configuration | declare_parameter, dynamic updates, validation |
| 6 | Launch System Simulation | YAML config, remappings, connection verification |
| 7 | Full Robot System | 5-node architecture, mixed communication patterns |

## How to Complete

Each `taskN_*.py` file contains function stubs with `raise NotImplementedError`. Read the docstrings, implement each function, and run the file to test. Solutions are in `../lab_solutions/`.

## Simulation Framework

`ros2_sim.py` provides:
- `Node`, `Publisher`, `Subscription`, `Timer`, `Service`, `Client`
- Message types: `String`, `Float64`, `Twist`, `Pose`, `LaserScan`, `Imu`, `Odometry`
- `spin()`, `spin_once()`, `spin_nodes()`, `spin_fast()`
- `QoSProfile`, parameters, logging
- `init()`, `shutdown()`, `ok()`

Import it the same way you would `rclpy` in real ROS2 code.
