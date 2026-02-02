# Week 1: LiDAR Processing

Dataset: KITTI raw seq 0001 (Velodyne 64-beam)

## Exercises

1. **Point Cloud Subscriber** - Subscribe to PointCloud2, compute stats
2. **Voxel Grid Filter** - Downsample point clouds
3. **Ground Removal + Clustering** - RANSAC ground plane, Euclidean clustering

## Setup

bash download_data.sh  # or python3 generate_bag.py

## Run

ros2 launch exercise1.launch.py
