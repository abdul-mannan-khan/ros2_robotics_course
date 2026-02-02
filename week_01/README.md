# Week 1: LiDAR Point Cloud Processing Fundamentals

**Category:** PERCEPTION | **Priority:** CRITICAL
**Target Companies:** Waymo, Cruise, Aurora, Skydio, DJI
**Time:** 6 hours

---

## Overview

Build a complete 3D LiDAR obstacle detection pipeline: filtering, ground removal, clustering, and bounding box fitting. This is the most common perception interview challenge at autonomous vehicle companies.

## Structure

```
week_01/
├── README.md                  # This file
├── lecture/
│   └── 01_lidar_point_cloud_processing.md   # Full lecture notes
├── lab_exercises/             # Student version (NO solutions)
│   ├── README.md              # Lab instructions
│   ├── generate_sample_data.py
│   ├── task1_load_and_visualize.py
│   ├── task2_voxel_downsampling.py
│   ├── task3_ransac_ground_removal.py
│   ├── task4_euclidean_clustering.py
│   ├── task5_bounding_boxes.py
│   └── task6_full_pipeline.py
├── lab_solutions/             # Instructor version (WITH solutions)
│   ├── task1_load_and_visualize_solution.py
│   ├── task2_voxel_downsampling_solution.py
│   ├── task3_ransac_ground_removal_solution.py
│   ├── task4_euclidean_clustering_solution.py
│   ├── task5_bounding_boxes_solution.py
│   └── task6_full_pipeline_solution.py
└── resources/
    └── references.md
```

## Prerequisites

- Python 3.10+
- NumPy, SciPy, Matplotlib
- Open3D (optional, for comparison)

```bash
pip install numpy scipy matplotlib open3d
```

## Lab Tasks

| Task | Topic | Key Algorithm |
|------|-------|---------------|
| 1 | Load & Visualize | PCD parsing, 3D plotting |
| 2 | Voxel Downsampling | Voxel grid filtering |
| 3 | Ground Removal | RANSAC plane fitting |
| 4 | Clustering | Euclidean clustering + KD-Tree |
| 5 | Bounding Boxes | AABB, OBB via PCA, IoU |
| 6 | Full Pipeline | End-to-end integration + evaluation |

## GitHub References

- [knaaga/lidar-obstacle-detection](https://github.com/knaaga/lidar-obstacle-detection) - C++ with PCL
- [arief25ramadhan/point-cloud-processing](https://github.com/arief25ramadhan/point-cloud-processing) - Python with Open3D

## Lab Equipment

- RPLiDAR S2 (2D LiDAR, 30m range)
- ROS2 Jazzy driver: `ros2 launch rplidar_ros rplidar_s2_launch.py`
