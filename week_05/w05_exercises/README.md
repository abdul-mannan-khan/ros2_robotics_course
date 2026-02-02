# Week 5: ROS2 Navigation Stack (Nav2) — ROS2 Applied Exercises

**Dataset:** TurtleBot3 navigation bag (generated)
**Platform:** TurtleBot3 Waffle
**Sensors:** LaserScan, Odometry, cmd_vel, map
**Companies:** iRobot, Amazon Robotics, Locus Robotics, Open Navigation

## Overview
Implement the core components of a navigation stack: costmap generation, path planning with A*, and path following with pure pursuit. These are the fundamental building blocks used by Nav2 and every mobile robot navigation system.

## Exercises

### Exercise 1: Costmap Generator (`exercise1_node.py`)
- Subscribe to `/map` (OccupancyGrid)
- Inflate obstacles by robot radius using distance transform
- Publish `/costmap` (OccupancyGrid)
- **Key concepts:** Obstacle inflation, cost scaling

### Exercise 2: A* Path Planner (`exercise2_node.py`)
- Subscribe to `/costmap`
- Implement A* search with diagonal movement support
- Service `/plan_path` or param-based start/goal
- Publish `/planned_path` (Path)
- **Key concepts:** A* search, heuristics, grid-based planning

### Exercise 3: Path Follower (`exercise3_node.py`)
- Subscribe to `/planned_path` (Path) and `/odom`
- Pure pursuit controller to follow the planned path
- Publish `/cmd_vel` (Twist)
- **Key concepts:** Pure pursuit, lookahead distance, velocity control

## How to Run

```bash
# Generate synthetic data
python3 ros2_exercises/generate_bag.py

# Run exercises
ros2 bag play ros2_exercises/bag_data/ &
ros2 run week_05 exercise1_node --ros-args --params-file ros2_exercises/config/params.yaml
```

## Parameters (config/params.yaml)
- `inflation_radius`: Robot inflation radius (default: 0.3m)
- `allow_diagonal`: Enable diagonal A* movement (default: true)
- `lookahead_distance`: Pure pursuit lookahead (default: 0.5m)
- `max_linear_vel` / `max_angular_vel`: Velocity limits
