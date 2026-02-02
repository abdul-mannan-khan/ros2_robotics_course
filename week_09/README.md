# Week 9: EGO-Planner & Trajectory Optimization

## Description
This module covers B-spline based trajectory optimization for autonomous drones,
implementing the EGO-Planner approach: ESDF-free collision avoidance with
gradient-based optimization of uniform cubic B-spline trajectories.

## Topics
- B-spline fundamentals (Cox-de Boor, uniform B-splines, matrix form)
- Trajectory representation using uniform cubic B-splines
- Smoothness optimization (jerk/snap minimization)
- ESDF-free collision avoidance (EGO-Planner style)
- Dynamic feasibility enforcement (velocity and acceleration limits)
- Multi-drone planning (EGO-Swarm decentralized approach)
- L-BFGS-B optimization with combined cost functions

## Structure
- `lab_exercises/` - Exercise stubs with TODOs
- `lab_solutions/` - Complete working solutions
- `lab_exercises/bspline_utils.py` - Shared B-spline utility library

## Dependencies
- python3-numpy
- python3-scipy
- python3-matplotlib
