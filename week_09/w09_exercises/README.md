# Week 9: EGO-Planner and Trajectory Optimization

## Overview
Local planning and trajectory optimization for agile drone flight. EGO-Planner by Zhou et al. (2020) is used at companies like DJI and in DARPA SubT challenge teams. This week covers depth-based obstacle perception, ESDF-based planning, and differential-flatness tracking.

## Dataset
Synthetic depth images and odometry from generate_bag.py simulating forward flight through obstacles.

## Exercises
1. Depth to Pointcloud - camera intrinsics back-projection
2. Local Planner - ESDF distance field and minimum-jerk trajectory
3. Trajectory Tracker - differential flatness-based PD control

## Run
bash download_data.sh
ros2 launch ros2_exercises/launch/exercise1.launch.py
