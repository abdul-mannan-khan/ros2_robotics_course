# Week 4: 2D SLAM with LiDAR — ROS2 Applied Exercises

**Dataset:** Cartographer Deutsches Museum 2D bag (2D LiDAR + IMU)
**Platform:** Backpack-mounted robot
**Sensors:** 2D LiDAR, IMU
**Companies:** Google (Cartographer), Waymo, iRobot, Boston Dynamics

## Overview
Build a 2D SLAM pipeline from scratch using ROS2 nodes. You'll implement scan matching, occupancy grid mapping, and loop closure detection — the core components of any SLAM system.

## Exercises

### Exercise 1: ICP Scan Matcher (`exercise1_node.py`)
- Subscribe to `/scan` (LaserScan)
- Implement ICP scan-to-scan matching between consecutive scans
- Publish incremental pose on `/scan_match_pose` (PoseStamped)
- **Key concepts:** Iterative Closest Point, SVD for rigid transforms

### Exercise 2: Occupancy Grid Builder (`exercise2_node.py`)
- Subscribe to `/scan` and `/odom` (or `/scan_match_pose`)
- Build 2D occupancy grid using ray casting and log-odds updates
- Publish `/map` (OccupancyGrid)
- **Key concepts:** Bresenham ray casting, log-odds probability

### Exercise 3: Loop Closure Detector (`exercise3_node.py`)
- Subscribe to `/scan` and `/map`
- Detect revisited areas using scan-to-map correlation
- Publish `/loop_closure` (PoseWithCovarianceStamped)
- **Key concepts:** Normalized cross-correlation, spatial search

## How to Run

```bash
# Generate synthetic data (if no real dataset available)
python3 ros2_exercises/generate_bag.py

# Run with bag playback
ros2 launch week_04 week4.launch.py

# Or manually:
ros2 bag play ros2_exercises/bag_data/ &
ros2 run week_04 exercise1_node --ros-args --params-file ros2_exercises/config/params.yaml
```

## Parameters (config/params.yaml)
- `resolution`: Map cell size (default: 0.05m)
- `max_iterations`: ICP iterations (default: 50)
- `convergence_threshold`: ICP convergence (default: 0.001)
- `hit_prob` / `miss_prob`: Log-odds update values
