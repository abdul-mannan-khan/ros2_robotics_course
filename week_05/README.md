# Week 5: ROS2 Navigation Stack (Nav2)

## Overview

This week covers the ROS2 Navigation Stack (Nav2), the primary framework for autonomous robot navigation in ROS2. Students implement each core component from scratch to understand how they work together.

## Topics

1. **Costmap Layers** - Static, obstacle, and inflation layers for spatial reasoning
2. **A* Path Planning** - Global path planning on occupancy grids
3. **Dynamic Window Approach** - Local trajectory planning and obstacle avoidance
4. **Particle Filter (AMCL)** - Monte Carlo localization from laser scans
5. **Behavior Trees** - Task orchestration for navigation decisions
6. **Waypoint Navigation** - Sequential goal following
7. **Full Stack Integration** - All components working together

## Directory Structure

```
week_05/
  lab_exercises/          # Exercise stubs with TODOs
    generate_nav_data.py  # Data generator (run first)
    data/                 # Generated data files
    task1_costmap.py
    task2_astar.py
    task3_dwa.py
    task4_particle_filter.py
    task5_behavior_tree.py
    task6_waypoint_navigation.py
    task7_full_navigation.py
  lab_solutions/          # Complete working solutions
    task1_solution.py
    task2_solution.py
    task3_solution.py
    task4_solution.py
    task5_solution.py
    task6_solution.py
    task7_solution.py
  resources/
    references.md
```

## Prerequisites

- Python 3, NumPy, Matplotlib
- SciPy (for Task 4 distance transform)
- No ROS2 installation required

## Getting Started

```bash
cd lab_exercises
python3 generate_nav_data.py    # Generate data
python3 task1_costmap.py        # Start with Task 1
```

For solutions:
```bash
cd lab_solutions
python3 task1_solution.py
```
