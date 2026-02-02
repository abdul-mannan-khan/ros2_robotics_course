# Week 8: 3D Path Planning for Drones

## Overview
This week covers 3D path planning algorithms for autonomous drone navigation, including graph-based search, sampling-based planning, trajectory optimization, dynamic replanning, and energy-aware planning.

## Lab Exercises

| Task | Topic | Key Concepts |
|------|-------|-------------|
| 1 | 3D A* | 26-connected grid search, Euclidean heuristic, diagonal costs |
| 2 | RRT | Rapidly-exploring Random Trees, 3D collision checking |
| 3 | RRT* | Rewiring, asymptotic optimality, cost convergence |
| 4 | Trajectory Optimization | Minimum snap, 7th-order polynomials, QP formulation |
| 5 | Dynamic Replanning | Moving obstacles, look-ahead collision detection, local replanning |
| 6 | Energy-Aware Planning | Wind models, altitude costs, battery constraints |
| 7 | Full Pipeline | Integration of all components, multi-metric evaluation |

## Getting Started

```bash
# Generate environment data first
cd lab_exercises
python3 generate_3d_env.py

# Run exercise stubs (incomplete - fill in the TODOs)
python3 task1_3d_astar.py

# Run complete solutions
cd ../lab_solutions
python3 task1_solution.py
```

## Prerequisites
- Python 3.8+
- numpy
- matplotlib
- scipy (for task 7)

All exercises run as standalone Python scripts without ROS2.
