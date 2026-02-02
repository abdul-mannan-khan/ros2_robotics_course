#!/usr/bin/env python3
"""
Task 4: Perception Pipeline
=============================
TC70045E Week 12 - Dr. Abdul Manan Khan, UWL

Obstacle detection -> map update -> replan cycle.

TODO: Complete the functions marked with TODO
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capstone_sim import (
    QuadrotorDynamics, EKF, OccupancyGrid3D, AStarPlanner,
    BSplineTrajectory, LQRController, ObstacleDetector
)


def run_perception_pipeline(duration=50.0, dt=0.01):
    """Full perception: detect -> update map -> replan.
    TODO:
    1. Set up environment with known and unknown obstacles
    2. Detect obstacles as drone flies
    3. Update occupancy grid with detections
    4. Trigger replanning when new obstacle on path
    5. Save plot as task4_perception.png
    """
    pass  # TODO


def test_false_positive_handling():
    """Test false positive handling.
    TODO:
    1. Set false_positive_rate=0.1
    2. Run mission, track false vs true detections
    3. Verify no excessive replanning
    """
    pass  # TODO


def visualize_occupancy_grid(grid, obstacles, drone_path, filename='task4_grid.png'):
    """Visualize the occupancy grid.
    TODO: Create 2D slice visualization.
    """
    pass  # TODO


def main():
    print("Task 4: Perception Pipeline")
    print("=" * 50)
    print("\nPart A: Full pipeline")
    run_perception_pipeline()
    print("\nPart B: False positive handling")
    test_false_positive_handling()
    print("\nTask 4 complete.")


if __name__ == '__main__':
    main()
