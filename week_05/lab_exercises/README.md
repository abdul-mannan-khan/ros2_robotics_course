# Week 5 Lab Exercises: ROS2 Navigation Stack (Nav2)

## Setup

First, generate the navigation data:

```bash
python3 generate_nav_data.py
```

This creates the `data/` directory with grid maps, laser scans, waypoints, and robot footprint data.

## Exercises

### Task 1: Costmap Layers (`task1_costmap.py`)
Implement Nav2-style layered costmaps:
- **Static layer**: Convert occupancy grid to cost values (0=free, 254=lethal, 255=unknown)
- **Obstacle layer**: Mark obstacles detected by laser scan
- **Inflation layer**: Exponential decay buffer around obstacles for robot safety
- **Layer combination**: Element-wise maximum of all layers

### Task 2: A* Path Planning (`task2_astar.py`)
Implement the A* global planner used by Nav2:
- Euclidean distance heuristic
- 8-connected grid neighbors (diagonal cost = sqrt(2))
- Full A* with open/closed sets and priority queue
- Gradient descent path smoothing

### Task 3: Dynamic Window Approach (`task3_dwa.py`)
Implement DWA local planner for trajectory control:
- Dynamic window computation from velocity limits and acceleration
- Forward trajectory simulation with constant (v, w)
- Multi-objective scoring: heading, clearance, velocity
- Best trajectory selection

### Task 4: Particle Filter / AMCL (`task4_particle_filter.py`)
Implement AMCL-style Monte Carlo localization:
- Uniform particle initialization in free space
- Odometry motion model with noise
- Likelihood field sensor model for laser scans
- Low-variance resampling
- Weighted mean pose estimation

### Task 5: Behavior Trees (`task5_behavior_tree.py`)
Implement Nav2's behavior tree navigation framework:
- Base node classes: Sequence, Fallback, Action, Condition
- NavigateToPose action using A* path following
- IsPathBlocked condition check
- RecoveryBehavior (spin/backup)
- Build and run a Nav2-like navigation BT

### Task 6: Waypoint Navigation (`task6_waypoint_navigation.py`)
Implement waypoint following:
- WaypointFollower class with goal queue management
- Global planning (A*) to each waypoint
- Path following with lookahead control
- Goal reached detection

### Task 7: Full Navigation Stack (`task7_full_navigation.py`)
Integrate all components:
- NavigationStack class combining map, costmap, AMCL, planner, controller
- Navigate through multiple goals
- Evaluate performance: success rate, path length, smoothness, timing

## Running

Each task is standalone:

```bash
python3 task1_costmap.py
python3 task2_astar.py
# ... etc
```

All tasks save output plots as PNG files and print educational information.
