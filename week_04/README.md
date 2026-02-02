# Week 4: 2D SLAM with LiDAR

## Overview

This week covers the fundamentals of Simultaneous Localization and Mapping (SLAM)
in 2D using simulated LiDAR data. You will implement each component of a SLAM
system from scratch: occupancy grid mapping, ICP scan matching, motion models,
loop closure detection, and pose graph optimization.

## Prerequisites

- Python 3.8+
- NumPy
- Matplotlib

No ROS2 installation is required. All exercises run as standalone Python scripts.

## Getting Started

1. **Generate the dataset first:**
   ```bash
   cd lab_exercises
   python3 generate_slam_data.py
   ```
   This creates simulated LiDAR, odometry, and environment data in `lab_exercises/data/`.

2. **Work through exercises** in `lab_exercises/task1_*.py` through `task7_*.py`.

3. **Check solutions** in `lab_solutions/` after attempting each task.

## Lab Exercises

| Task | Topic | Key Concepts |
|------|-------|-------------|
| 1 | Occupancy Grid Mapping | Log-odds, Bresenham ray casting, inverse sensor model |
| 2 | ICP Scan Matching | SVD-based transform estimation, nearest-neighbor correspondences |
| 3 | Motion Model | Probabilistic odometry, particle propagation, uncertainty growth |
| 4 | Scan Matching Localization | Odometry + ICP correction, ATE evaluation |
| 5 | Mapping with SLAM | Map quality vs pose accuracy comparison |
| 6 | Loop Closure | Scan similarity, pose graph, least-squares optimization |
| 7 | Full SLAM Pipeline | Complete system integration and evaluation |

## Running Solutions

```bash
cd lab_solutions
python3 task1_occupancy_grid_solution.py
python3 task2_icp_solution.py
python3 task3_motion_model_solution.py
python3 task4_scan_matching_localization_solution.py
python3 task5_occupancy_mapping_with_slam_solution.py
python3 task6_loop_closure_solution.py
python3 task7_full_slam_solution.py
```

All solutions save plots to `lab_solutions/` as PNG files.

## Directory Structure

```
week_04/
  README.md
  lab_exercises/
    generate_slam_data.py
    data/                    (generated)
    task1_occupancy_grid.py
    task2_icp.py
    task3_motion_model.py
    task4_scan_matching_localization.py
    task5_occupancy_mapping_with_slam.py
    task6_loop_closure.py
    task7_full_slam.py
  lab_solutions/
    task1_occupancy_grid_solution.py
    task2_icp_solution.py
    task3_motion_model_solution.py
    task4_scan_matching_localization_solution.py
    task5_occupancy_mapping_with_slam_solution.py
    task6_loop_closure_solution.py
    task7_full_slam_solution.py
  resources/
    references.md
```
