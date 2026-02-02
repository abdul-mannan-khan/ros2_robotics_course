# Week 6: Quadrotor Dynamics & Attitude Control

## Overview

This week covers the fundamentals of quadrotor UAV dynamics, attitude representation,
and control. All exercises run as standalone Python scripts (no ROS2 required).

**Convention:** ENU/FLU (x-forward, y-left, z-up) following the ROS2 standard.

## Topics

1. **Rotation Math** - Euler angles, quaternions, gimbal lock
2. **Dynamics Model** - Newton-Euler equations of motion for a rigid-body quadrotor
3. **PID Attitude Control** - Cascaded PID for roll/pitch/yaw stabilization
4. **Motor Mixing** - Mapping control commands to individual motor speeds
5. **Cascaded Control** - Position + attitude control for waypoint tracking
6. **Complementary Filter** - Fusing gyroscope and accelerometer for attitude estimation
7. **Full Simulation** - Integrated system flying a square trajectory

## Lab Exercises

Exercise stubs are in `lab_exercises/`. Each file contains function signatures and
docstrings describing what to implement. The shared simulation module is
`lab_exercises/quadrotor_sim.py`.

## Solutions

Complete solutions are in `lab_solutions/`. Run any solution with:

```bash
python3 lab_solutions/task1_solution.py
```

Each solution prints educational output and saves a plot image.

## Prerequisites

- Python 3
- NumPy
- Matplotlib
