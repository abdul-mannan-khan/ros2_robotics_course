# Week 11: Control Systems PID to MPC

## Topic Overview
Progress from PID through LQR to MPC for quadrotor position control.

## Learning Objectives
- Implement PID controller with anti-windup for 3-axis position tracking
- Derive and implement LQR by solving the discrete Riccati equation
- Formulate and solve a receding-horizon MPC optimization problem

## Dataset
**EuRoC MAV MH_01_easy** - IMU, stereo camera, and ground truth poses
from a micro aerial vehicle in a machine hall environment.

## Industry Context
Skydio uses MPC for agile autonomous flight. DJI and PX4 autopilots
use cascaded PID. LQR is used in SpaceX landing controllers.

## Exercises

### Ex1: PID Controller
- In: /odom, /reference_trajectory
- Out: /cmd_thrust_attitude, /tracking_error

### Ex2: LQR Controller
- In: /odom, /reference_trajectory
- Out: /cmd_thrust_attitude, /lqr_gain

### Ex3: MPC Controller
- In: /odom, /reference_trajectory
- Out: /cmd_thrust_attitude, /predicted_trajectory, /mpc_cost

## Run
ros2 bag play bag_data/euroc_mh01/
ros2 launch ros2_robotics_course exercise1.launch.py

See config/params.yaml