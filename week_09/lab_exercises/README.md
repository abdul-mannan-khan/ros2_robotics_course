# Week 9 Lab Exercises: EGO-Planner & Trajectory Optimization

## Overview
This week covers B-spline trajectory optimization for drone path planning,
following the EGO-Planner approach (ESDF-free gradient-based optimization).

## Prerequisites
- Python 3, NumPy, SciPy, Matplotlib
- Understanding of B-spline curves and optimization

## Exercises

| Task | Topic | Key Concepts |
|------|-------|-------------|
| Task 1 | B-Spline Basics | Basis functions, Cox-de Boor, local support, convex hull |
| Task 2 | Trajectory Representation | Waypoint fitting, velocity/acceleration profiles, feasibility |
| Task 3 | Smoothness Cost | Jerk/snap minimization, gradient descent |
| Task 4 | Collision Avoidance | ESDF-free collision cost, gradient-based avoidance |
| Task 5 | Full EGO-Planner | Combined cost, L-BFGS optimization, replanning |
| Task 6 | Multi-Drone | EGO-Swarm decentralized planning |
| Task 7 | Full Integration | Complete pipeline with evaluation |

## Running

Each task is a standalone Python script:
```bash
python3 task1_bspline_basics.py
python3 task2_trajectory_representation.py
# etc.
```

Solutions are in `../lab_solutions/`.

## Shared Library

`bspline_utils.py` provides reusable B-spline functions used by all tasks.

## Key Equations

**Uniform cubic B-spline velocity:**
```
v_i = (Q_{i+1} - Q_i) / dt
```

**Uniform cubic B-spline acceleration:**
```
a_i = (Q_{i+2} - 2*Q_{i+1} + Q_i) / dt^2
```

**EGO-Planner cost:**
```
J = w_s * J_smooth + w_c * J_collision + w_d * J_dynamic
```
