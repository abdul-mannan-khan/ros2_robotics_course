# Week 4 Lab Exercises: 2D SLAM with LiDAR

## Setup

Generate the dataset before starting:
```bash
python3 generate_slam_data.py
```

This creates `data/` with:
- `environment.npy` - 500x500 occupancy grid (0.1m resolution, 50x50m)
- `ground_truth_poses.npy` - Nx3 [x, y, theta] ground truth trajectory
- `odometry.npy` - Nx3 [dx, dy, dtheta] noisy relative odometry
- `lidar_scans.npy` - Nx360 range values (360 rays, 12m max range)
- `timestamps.npy` - N timestamps

## Task 1: Occupancy Grid Mapping

**File:** `task1_occupancy_grid.py`

Build an occupancy grid map from known (ground truth) poses and LiDAR scans.

**Key concepts:**
- Log-odds representation: `l = log(P/(1-P))`, update via addition
- Bresenham ray casting: trace rays through grid cells efficiently
- Inverse sensor model: classify cells as free/occupied/unknown based on range
- Bayesian update: fuse multiple observations per cell

**Functions to implement:**
1. `initialize_grid()` - Create log-odds grid (all zeros = P=0.5)
2. `log_odds_update()` - `l_new = l_prior + l_measurement`
3. `bresenham_ray()` - Trace ray through grid cells
4. `inverse_sensor_model()` - Return log-odds for free/occupied/unknown
5. `update_grid_with_scan()` - Integrate one full LiDAR scan
6. `log_odds_to_probability()` - Convert back: `P = 1 - 1/(1+exp(l))`

## Task 2: ICP Scan Matching

**File:** `task2_icp.py`

Align consecutive LiDAR scans using Iterative Closest Point.

**Key concepts:**
- Polar to Cartesian conversion
- Nearest-neighbor correspondences
- SVD-based rigid transform estimation: `H = src.T @ tgt`, then `R = V @ U.T`
- Iterative refinement until convergence

**Functions to implement:**
1. `scan_to_points()` - Polar to Cartesian, filter max-range
2. `find_correspondences()` - Nearest neighbor matching
3. `estimate_transform()` - SVD solution for R, t
4. `apply_transform()` - Apply rigid transform
5. `icp()` - Full iterative loop

## Task 3: Probabilistic Motion Model

**File:** `task3_motion_model.py`

Model odometry uncertainty and visualize drift growth.

**Key concepts:**
- Noise proportional to motion magnitude
- Particle-based uncertainty representation
- Dead reckoning accumulates unbounded error

**Functions to implement:**
1. `odometry_motion_model()` - Sample noisy pose from odometry
2. `propagate_particles()` - Propagate particle set
3. `compute_pose_from_odometry()` - Dead reckoning integration
4. `visualize_uncertainty()` - Plot particle spread

## Task 4: Scan Matching Localization

**File:** `task4_scan_matching_localization.py`

Combine odometry prediction with ICP correction.

**Key concepts:**
- Predict-correct cycle (like a filter)
- ICP provides local corrections to odometry
- ATE (Absolute Trajectory Error) metric

**Functions to implement:**
1. `predict_pose()` - Apply odometry
2. `correct_pose_icp()` - ICP correction
3. `scan_matching_pipeline()` - Full pipeline
4. `evaluate_trajectory()` - Compute ATE

## Task 5: Mapping with SLAM Poses

**File:** `task5_occupancy_mapping_with_slam.py`

Compare maps built with ground truth vs estimated poses.

**Key concepts:**
- Pose error propagates to map quality
- Map comparison metrics (accuracy, precision, recall)

## Task 6: Loop Closure

**File:** `task6_loop_closure.py`

Detect revisited locations and optimize the pose graph.

**Key concepts:**
- Scan similarity for loop detection (normalized cross-correlation)
- ICP for relative pose constraints
- Pose graph: nodes = poses, edges = constraints
- Gradient descent optimization

**Functions to implement:**
1. `detect_loop_closure()` - Find matching past scans
2. `compute_loop_constraint()` - ICP between matched scans
3. `build_pose_graph()` - Odometry + loop edges
4. `optimize_pose_graph()` - Iterative least-squares

## Task 7: Full SLAM Pipeline

**File:** `task7_full_slam.py`

Integrate everything into a complete SLAM system.

**Key concepts:**
- System architecture: scan matching + loop closure + optimization + mapping
- Comprehensive evaluation: ATE, RPE, map accuracy
- The full SLAM pipeline should outperform odometry-only approaches

## Tips

- Start with Task 1 and work sequentially - later tasks build on earlier ones.
- Each task has a `main()` function that loads data and orchestrates the pipeline.
- Print intermediate results to verify correctness.
- Compare your output plots with the solutions.
