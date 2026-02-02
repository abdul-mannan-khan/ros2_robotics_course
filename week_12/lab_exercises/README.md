# Week 12 Lab Exercises

## Capstone Integration Module

`capstone_sim.py` contains all subsystem implementations:

- **QuadrotorDynamics** - 6-DOF quadrotor with RK4 integration (Week 6)
- **EKF** - Extended Kalman Filter for IMU+GPS fusion (Week 2)
- **AStarPlanner** - 3D A* path planning on occupancy grid (Week 8)
- **BSplineTrajectory** - Smooth trajectory generation (Week 9)
- **PIDController / LQRController** - Position controllers (Week 11)
- **ObstacleDetector** - Simulated depth sensor (Week 10)
- **AutonomousDroneSystem** - Complete integrated system

## Tasks

Complete the TODO sections in each task file. Run the corresponding
solution in `lab_solutions/` to verify expected behaviour.

### Quick Start

```bash
python3 capstone_sim.py   # Verify the module works
```

Then work through tasks 1-7 in order.
