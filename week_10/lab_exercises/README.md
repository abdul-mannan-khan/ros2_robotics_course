# Week 10: Computer Vision for Drones - Lab Exercises

## Overview
Implement core computer vision algorithms from scratch using only NumPy, SciPy, and Matplotlib.

## Setup
```bash
cd lab_exercises
python3 generate_vision_data.py   # Generate synthetic data
```

## Tasks

| Task | Topic | Key Concepts |
|------|-------|--------------|
| 1 | Camera Model | Projection, undistortion, DLT calibration |
| 2 | Feature Detection | Harris corners, patch descriptors, ratio test matching |
| 3 | Homography | DLT, RANSAC, decomposition into R,t |
| 4 | Object Detection | Sliding window, HOG features, NMS, precision/recall |
| 5 | Stereo Depth | Block matching (SAD), disparity-to-depth, point clouds |
| 6 | Optical Flow | Lucas-Kanade (2x2 system), motion estimation |
| 7 | Full Pipeline | Integrate all components for drone visual odometry |

## Running
```bash
python3 taskN_camera_model.py   # Exercise stubs
# Solutions in ../lab_solutions/
python3 ../lab_solutions/taskN_solution.py
```

## Dependencies
- numpy
- scipy
- matplotlib

No OpenCV required - all algorithms implemented from scratch.
