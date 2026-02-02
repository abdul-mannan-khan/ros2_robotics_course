# Week 8 Lab Exercises: 3D Path Planning for Drones

## Setup
Run the data generator first to create the 3D environment:
```bash
python3 generate_3d_env.py
```
This creates the `data/` directory with voxel grids, obstacles, waypoints, and dynamic obstacles.

## Exercises
Each task file contains function stubs with docstrings and TODO markers. Implement the missing functions.

1. **task1_3d_astar.py** - 3D A* with 26-connected neighbors
2. **task2_rrt.py** - RRT in 3D space
3. **task3_rrt_star.py** - RRT* with rewiring
4. **task4_trajectory_optimization.py** - Minimum snap trajectory
5. **task5_dynamic_replanning.py** - Replanning with moving obstacles
6. **task6_energy_aware.py** - Energy-constrained planning
7. **task7_full_planning.py** - Complete planning pipeline

## Solutions
Complete solutions are in `../lab_solutions/`.
