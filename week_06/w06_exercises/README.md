# Week 6: Quadrotor Dynamics & Attitude Control — ROS2 Applied Exercises

**Dataset:** UZH-FPV indoor_forward_3 (IMU 1000Hz + camera)
**Platform:** FPV racing quadrotor
**Sensors:** IMU 1000Hz, camera, ground truth laser tracker
**Companies:** Skydio, DJI, Autel Robotics, Zipline

## Overview
Implement the core components of a quadrotor flight controller: attitude estimation from IMU data, thrust/torque mixing from motor commands, and SO(3) attitude control. These run at 1000Hz on real flight controllers.

## Exercises

### Exercise 1: Attitude Estimator (`exercise1_node.py`)
- Subscribe to `/imu` (Imu at 1000Hz)
- Complementary filter combining gyro integration + accelerometer correction
- Publish `/estimated_attitude` (QuaternionStamped)
- **Key concepts:** Complementary filter, gyro integration, gravity vector

### Exercise 2: Thrust Mixer (`exercise2_node.py`)
- Subscribe to `/motor_commands` (Float32MultiArray, 4 motors)
- Compute net thrust and torques using mixing matrix
- Publish `/body_wrench` (Wrench)
- **Key concepts:** Motor mixing matrix, X-configuration, thrust/torque coefficients

### Exercise 3: Attitude Controller (`exercise3_node.py`)
- Subscribe to `/estimated_attitude` and `/desired_attitude`
- PD controller on SO(3) orientation error
- Publish `/motor_commands` (Float32MultiArray)
- **Key concepts:** SO(3) error, PD control, inverse mixing

## How to Run

```bash
# Generate synthetic data
python3 ros2_exercises/generate_bag.py

# Run exercises
ros2 bag play ros2_exercises/bag_data/ &
ros2 run week_06 exercise1_node --ros-args --params-file ros2_exercises/config/params.yaml
```

## Parameters (config/params.yaml)
- `alpha`: Complementary filter coefficient (default: 0.98)
- `arm_length`: Quadrotor arm length (default: 0.17m)
- `k_thrust` / `k_torque`: Motor coefficients
- `kp_roll/pitch/yaw`, `kd_roll/pitch/yaw`: PD gains
