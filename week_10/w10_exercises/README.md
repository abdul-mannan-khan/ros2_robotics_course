# Week 10: Computer Vision for Robotics

## Topic Overview
Core computer vision techniques for autonomous vehicles and mobile robots:
feature detection, stereo depth estimation, and multi-object tracking.

## Learning Objectives
- Detect and describe image features using ORB and SIFT algorithms
- Compute depth maps from stereo camera pairs using disparity estimation
- Implement a multi-object tracker using Kalman filters and Hungarian assignment

## Dataset
**KITTI Tracking Dataset (Sequence 0000)**
- Stereo camera images (left/right) at 1242x375 resolution
- Velodyne LiDAR point clouds (64-beam)
- Ground truth object labels with tracking IDs
- Source: http://www.cvlibs.net/datasets/kitti/eval_tracking.php

## Platform
Autonomous driving platform with stereo camera and 3D LiDAR sensor suite.

## Industry Context
Tesla, Waymo, and Cruise all use computer vision pipelines for perception.
Feature detection underpins visual odometry (used in Tesla Autopilot).
Stereo depth is critical for platforms without LiDAR.
Multi-object tracking (MOT) is essential for predicting other agents.

## Exercises

### Ex1: Image Feature Detector
- In: /camera/left/image_raw
- Out: /features, /feature_count

### Ex2: Stereo Depth
- In: left+right image_raw
- Out: /disparity, /depth_image

### Ex3: Object Tracker
- In: /camera/left/image_raw
- Out: /tracked_objects, /tracking_image

## Run
ros2 bag play bag_data/kitti_tracking_0000/
ros2 launch ros2_robotics_course exercise1.launch.py

See config/params.yaml