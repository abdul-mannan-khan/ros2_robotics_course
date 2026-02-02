# Week 8: 3D Path Planning

## Overview
3D path planning for aerial and ground robots. Companies like Boston Dynamics, Skydio, and Wing use 3D planning for buildings, forests, urban canyons. Uses Newer College Dataset (Ouster OS-1 64-beam LiDAR).

## Dataset
Synthetic building pointclouds from generate_bag.py. Real: Newer College Dataset.

## Exercises
1. 3D Occupancy Map Builder - voxelize pointclouds
2. RRT* 3D Planner - sample-based 3D path planning
3. Trajectory Smoother - B-spline fitting with velocity profiling

## Run
bash download_data.sh
ros2 launch ros2_exercises/launch/exercise1.launch.py
