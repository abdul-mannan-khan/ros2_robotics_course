#!/usr/bin/env python3
"""
Task 5: Full Autonomous Mission
=================================
TC70045E Week 12 - Dr. Abdul Manan Khan, UWL

Complete mission: takeoff -> navigate waypoints -> avoid obstacles -> land.

TODO: Complete the functions marked with TODO
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capstone_sim import AutonomousDroneSystem, MissionState


def run_full_mission():
    """Execute a complete autonomous mission.
    TODO:
    1. Create AutonomousDroneSystem with LQR controller
    2. Set up environment with bounds [-2,-2,-0.5] to [15,15,8]
    3. Add 3+ obstacles and 4+ waypoints
    4. Run mission
    5. Verify: mission completed, no collisions, all waypoints reached
    6. Save 4-panel plot as task5_mission.png
    7. Print metrics
    """
    pass  # TODO


def run_challenging_mission():
    """Run a more challenging mission.
    TODO:
    1. 6+ obstacles, 6+ waypoints, narrow passages
    2. Evaluate performance
    3. Save plot as task5_challenging.png
    """
    pass  # TODO


def plot_mission_results(system, metrics, filename='task5_mission.png'):
    """Create comprehensive mission visualization.
    TODO: 4-panel plot: trajectory, error, position, mission states.
    """
    pass  # TODO


def main():
    print("Task 5: Full Autonomous Mission")
    print("=" * 50)
    print("\nPart A: Standard mission")
    run_full_mission()
    print("\nPart B: Challenging mission")
    run_challenging_mission()
    print("\nTask 5 complete.")


if __name__ == '__main__':
    main()
