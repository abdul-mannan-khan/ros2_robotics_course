# Week 4 References: 2D SLAM with LiDAR

## Textbooks

- **Probabilistic Robotics** by S. Thrun, W. Burgard, D. Fox (2005)
  - Chapter 9: Occupancy Grid Mapping
  - Chapter 10: SLAM with Extended Kalman Filters
  - Chapter 11: Graph-Based SLAM

- **Introduction to Autonomous Mobile Robots** by R. Siegwart, I. Nourbakhsh, D. Scaramuzza (2011)
  - Chapter 5: Perception (LiDAR sensing models)
  - Chapter 6: SLAM

## Key Papers

- P. Besl and N. McKay, "A Method for Registration of 3-D Shapes," IEEE TPAMI, 1992.
  (Original ICP paper)

- S. Rusinkiewicz and M. Levoy, "Efficient Variants of the ICP Algorithm," 3DIM, 2001.
  (ICP variants and improvements)

- E. Olson, J. Leonard, S. Teller, "Fast Iterative Alignment of Pose Graphs with Poor Initial Estimates," ICRA, 2006.
  (Pose graph optimization)

- G. Grisetti, C. Stachniss, W. Burgard, "Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters," IEEE TRO, 2007.
  (GMapping - particle filter SLAM)

- R. Kuemmerle et al., "g2o: A General Framework for Graph Optimization," ICRA, 2011.
  (Graph optimization framework)

- W. Hess et al., "Real-Time Loop Closure in 2D LIDAR SLAM," ICRA, 2016.
  (Google Cartographer)

## Open-Source Implementations

- **GMapping** - ROS package for 2D grid-based SLAM
  https://github.com/ros-perception/slam_gmapping

- **Cartographer** - Google's real-time 2D/3D SLAM
  https://github.com/cartographer-project/cartographer

- **SLAM Toolbox** - ROS2 SLAM package
  https://github.com/SteveMacenski/slam_toolbox

- **g2o** - General graph optimization
  https://github.com/RainerKuemmerle/g2o

## Concepts Reference

### Log-Odds Occupancy Mapping
- Prior: l_0 = log(0.5/0.5) = 0
- Update: l_new = l_old + l_measurement - l_0
- Since l_0 = 0: l_new = l_old + l_measurement
- Convert back: P = 1 - 1/(1 + exp(l))

### ICP Algorithm
1. Find nearest-neighbor correspondences
2. Compute optimal R, t via SVD of cross-covariance H = src^T @ tgt
3. Apply transform to source
4. Repeat until convergence

### Pose Graph SLAM
- Nodes: robot poses
- Edges: relative pose constraints (from odometry or loop closures)
- Optimization: minimize sum of squared residuals weighted by information matrices
