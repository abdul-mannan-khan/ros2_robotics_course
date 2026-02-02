# Week 12: Full System Integration & Capstone Project

**Robotics & Drone Engineering Course** | Dr. Abdul Manan Khan

## Overview

This is the final week of the course. All subsystems from Weeks 1-11 are combined into
a complete autonomous drone system capable of perception, state estimation, path planning,
trajectory generation, and control.

## Contents

- `lecture/` - HTML lecture on system integration, capstone projects, and deployment
- `lab_exercises/` - Exercise stubs and capstone simulation module
- `lab_solutions/` - Complete solutions for all 7 tasks
- `resources/` - References and additional materials

## Lab Exercises

| Task | Topic | Description |
|------|-------|-------------|
| 1 | System Architecture | Design component graph and health monitoring |
| 2 | Sensor Fusion | Integrate IMU + GPS with EKF, handle failures |
| 3 | Planning-Control | A* -> B-spline -> LQR/PID pipeline |
| 4 | Perception Pipeline | Detect -> map update -> replan cycle |
| 5 | Full Mission | Complete autonomous mission |
| 6 | Stress Testing | Adverse conditions testing |
| 7 | Evaluation | Multi-mission evaluation with 8-panel report |

## Running

```bash
cd lab_exercises
python3 capstone_sim.py          # Run demo mission
python3 task5_full_mission.py     # Run full mission (stub)
```

For solutions:
```bash
cd lab_solutions
python3 task5_solution.py         # Complete mission solution
python3 task7_solution.py         # Full evaluation report
```

## Dependencies

- Python 3.8+
- NumPy, SciPy, Matplotlib
