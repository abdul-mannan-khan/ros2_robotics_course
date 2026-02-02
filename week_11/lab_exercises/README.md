# Week 11 Lab Exercises: Control Systems - PID to MPC

**Course:** TC70045E Robotics & Drone Engineering
**Instructor:** Dr. Abdul Manan Khan, University of West London

## Overview

This lab covers control system design from classical PID to modern Model Predictive Control (MPC), applied to quadrotor drones.

## Prerequisites

```bash
pip install numpy scipy matplotlib
```

## Tasks

| Task | Topic | Key Concepts |
|------|-------|--------------|
| 1 | PID Analysis | Step response, Ziegler-Nichols, frequency analysis, anti-windup |
| 2 | State-Space | Modeling, controllability, observability, discretization |
| 3 | Pole Placement | Eigenvalue assignment, Ackermann's formula, stability regions |
| 4 | LQR | Riccati equation, Q/R tuning, quadrotor hover |
| 5 | MPC Basics | Receding horizon, QP formulation, constraints |
| 6 | MPC Drone | Trajectory tracking, disturbance rejection |
| 7 | Full Comparison | PID vs LQR vs MPC on identical tasks |

## Running

Each task is standalone:

```bash
cd lab_exercises
python3 task1_pid_analysis.py
```

Solutions are in `../lab_solutions/`.

## Key Files

- `control_sim.py` - Simulation module (LinearSystem, PID, LQR, MPC classes)
- `task1_pid_analysis.py` through `task7_full_comparison.py` - Exercise stubs
