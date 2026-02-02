# Week 1 Lab: LiDAR Point Cloud Processing Pipeline

**Course:** TC70045E - Robotics & Drone Engineering
**Challenge Style:** Waymo / Cruise / Aurora Perception Interview
**Time Limit:** 6 hours
**Language:** Python 3
**ROS2 Version:** Jazzy

---

## Overview

You will implement a complete LiDAR obstacle detection pipeline from scratch.
The pipeline processes raw 3D point cloud data to detect and classify obstacles,
implementing: filtering, ground removal, clustering, and bounding box fitting.

## Setup

```bash
cd ~/auto_ws/ros2_robotics_course/week_01/lab_exercises
pip install open3d numpy matplotlib scipy
python3 generate_sample_data.py   # generates sample point clouds
```

## Evaluation Criteria

| Criteria | Weight |
|----------|--------|
| Algorithm correctness | 30% |
| Code quality and efficiency | 25% |
| Handling of edge cases | 20% |
| Documentation | 15% |
| Visualization quality | 10% |

---

## Tasks

Complete each task in the corresponding Python file. Each file has function
stubs with docstrings describing the expected inputs, outputs, and behavior.

### Task 1: Load and Visualize Point Cloud Data
**File:** `task1_load_and_visualize.py`

### Task 2: Implement Voxel Grid Downsampling
**File:** `task2_voxel_downsampling.py`

### Task 3: Implement RANSAC Ground Plane Removal
**File:** `task3_ransac_ground_removal.py`

### Task 4: Implement Euclidean Clustering with KD-Tree
**File:** `task4_euclidean_clustering.py`

### Task 5: Fit Oriented Bounding Boxes to Clusters
**File:** `task5_bounding_boxes.py`

### Task 6: Full Pipeline and Evaluation
**File:** `task6_full_pipeline.py`

---

## Submission

- All 6 task files completed
- A brief report (in comments or separate file) answering:
  1. What voxel size did you choose and why?
  2. How many RANSAC iterations are needed for 99% confidence?
  3. What clustering parameters worked best?
- Visualization screenshots showing your pipeline output

## Interview Questions (Answer in your report)

1. Explain RANSAC and its time complexity. How do you choose the number of iterations?
2. Why use KD-Tree for clustering? What's the time complexity vs brute force?
3. How do you handle dynamic objects vs static obstacles in LiDAR data?
4. Describe the difference between Euclidean clustering and DBSCAN.
5. How would you optimize this pipeline for real-time performance (10Hz)?
