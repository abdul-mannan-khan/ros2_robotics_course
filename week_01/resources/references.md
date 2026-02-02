# Week 1 References

## GitHub Repositories
- [knaaga/lidar-obstacle-detection](https://github.com/knaaga/lidar-obstacle-detection) - C++ LiDAR pipeline with PCL
- [arief25ramadhan/point-cloud-processing](https://github.com/arief25ramadhan/point-cloud-processing) - Python pipeline with Open3D
- [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) - General robotics algorithms

## Libraries
- **Open3D**: http://www.open3d.org/docs/release/
- **PCL** (Point Cloud Library): https://pointclouds.org/documentation/
- **NumPy**: https://numpy.org/doc/
- **SciPy KDTree**: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.html

## Key Papers
- Fischler & Bolles (1981) - "Random Sample Consensus: A Paradigm for Model Fitting"
- Ester et al. (1996) - "A Density-Based Algorithm for Discovering Clusters in Large Spatial Databases"
- Bentley (1975) - "Multidimensional Binary Search Trees Used for Associative Searching"

## ROS2 Jazzy + RPLiDAR
- RPLiDAR ROS2: https://github.com/Slamtec/rplidar_ros (ros2 branch)
- Sensor messages: `sensor_msgs/msg/PointCloud2`, `sensor_msgs/msg/LaserScan`

## Interview Preparation
- Waymo perception roles: LiDAR processing, 3D detection, tracking
- Cruise: similar pipeline + deep learning integration
- Aurora: real-time constraints emphasis
