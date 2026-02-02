#!/usr/bin/env python3
"""
Task 3: Planning-Control Integration
======================================
TC70045E Week 12 - Dr. Abdul Manan Khan, UWL

Connect A* planner to B-spline trajectory to LQR/PID tracking.

TODO: Complete the functions marked with TODO
"""
import numpy as np
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capstone_sim import (
    QuadrotorDynamics, EKF, OccupancyGrid3D, AStarPlanner,
    BSplineTrajectory, PIDController, LQRController
)


def plan_and_track(controller_type='lqr', duration=40.0, dt=0.01):
    """Full pipeline: A* -> B-spline -> controller tracking.
    TODO:
    1. Create environment with obstacles
    2. Plan path with A* from [0,0,2] to [10,10,2]
    3. Generate B-spline trajectory
    4. Track with selected controller
    5. Measure RMSE, max error, planning time
    6. Save plot as task3_track_{controller_type}.png
    """
    pass  # TODO


def test_replanning(duration=50.0, dt=0.01):
    """Test replanning when new obstacles appear.
    TODO:
    1. Plan initial path
    2. At t=15s, add new obstacle on path
    3. Detect and replan
    4. Verify no collision
    5. Save plot as task3_replan.png
    """
    pass  # TODO


def measure_latency():
    """Measure end-to-end latency.
    TODO: Time A* planning, B-spline generation, and control computation.
    Print results.
    """
    pass  # TODO


def main():
    print("Task 3: Planning-Control Integration")
    print("=" * 50)
    print("\nPart A: LQR tracking")
    plan_and_track('lqr')
    print("\nPart B: PID tracking")
    plan_and_track('pid')
    print("\nPart C: Replanning")
    test_replanning()
    print("\nPart D: Latency")
    measure_latency()
    print("\nTask 3 complete.")


if __name__ == '__main__':
    main()
