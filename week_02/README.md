# Week 2: Sensor Fusion with Extended Kalman Filter

**Category:** STATE ESTIMATION | **Priority:** CRITICAL
**Target Companies:** Amazon Robotics, Tesla, Skydio, DJI, Zipline
**Time:** 4 hours

---

## Overview

Implement an Extended Kalman Filter (EKF) to fuse IMU, wheel odometry, and GPS data for accurate robot pose estimation. Handle asynchronous sensor updates and varying noise characteristics. This is asked in almost every robotics interview.

## Structure

```
week_02/
├── README.md                  # This file
├── lecture/
│   └── Week2_Sensor_Fusion_EKF.html   # Full lecture (open in browser)
├── lab_exercises/             # Student version (NO solutions)
│   ├── README.md              # Lab instructions
│   ├── generate_sensor_data.py
│   ├── task1_prediction.py
│   ├── task2_encoder_update.py
│   ├── task3_gps_update.py
│   ├── task4_jacobians.py
│   ├── task5_covariance_tuning.py
│   ├── task6_full_ekf.py
│   └── task7_evaluation.py
├── lab_solutions/             # Instructor version (WITH solutions)
│   ├── task1_prediction_solution.py
│   ├── task2_encoder_update_solution.py
│   ├── task3_gps_update_solution.py
│   ├── task4_jacobians_solution.py
│   ├── task5_covariance_tuning_solution.py
│   ├── task6_full_ekf_solution.py
│   └── task7_evaluation_solution.py
└── resources/
    └── references.md
```

## Prerequisites

- Python 3.10+
- NumPy, SciPy, Matplotlib

```bash
pip install numpy scipy matplotlib
```

## State Vector

```
x = [px, py, theta, vx, vy, omega]^T
```

- `px, py` — Position in world frame (metres)
- `theta` — Heading angle (radians)
- `vx, vy` — Linear velocity (m/s)
- `omega` — Angular velocity (rad/s)

## Sensors

| Sensor | Rate | Measures | Noise |
|--------|------|----------|-------|
| IMU | 100 Hz | ax, ay, alpha | accel: 0.1 m/s², gyro: 0.01 rad/s |
| Wheel Encoders | 50 Hz | v, omega | v: 0.05 m/s, omega: 0.02 rad/s |
| GPS | 1 Hz | px, py | position: 2.0 m |

## Lab Tasks

| Task | Topic | Key Concept |
|------|-------|-------------|
| 1 | Prediction Step | Nonlinear motion model with IMU |
| 2 | Encoder Update | Measurement model and Kalman gain |
| 3 | GPS Update | Absolute position correction |
| 4 | Jacobian Verification | Analytic vs numerical Jacobians |
| 5 | Covariance Tuning | Q and R parameter search |
| 6 | Full EKF | Asynchronous multi-sensor fusion |
| 7 | Evaluation | RMSE, sensor dropout testing |

## GitHub References

- [AtsushiSakai/PythonRobotics - EKF](https://github.com/AtsushiSakai/PythonRobotics/tree/master/Localization/extended_kalman_filter)
- [rlabbe/Kalman-and-Bayesian-Filters-in-Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)
- [andrestoga/take_home_robotics_coding_test](https://github.com/andrestoga/take_home_robotics_coding_test/tree/master/odometry)

## Lab Equipment

- Astra Pro RGB-D Camera
- RPLiDAR S2 (2D LiDAR, 30m range)
- ROS2 Jazzy driver: `ros2 launch rplidar_ros rplidar_s2_launch.py`
