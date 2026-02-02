#!/usr/bin/env python3
"""
Task 2: Sensor Fusion Integration
===================================
TC70045E Week 12 - Dr. Abdul Manan Khan, UWL

Integrate IMU + GPS + simulated camera with EKF.
Handle sensor failures gracefully.

TODO: Complete the functions marked with TODO
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capstone_sim import QuadrotorDynamics, IMUSensor, GPSSensor, EKF


def run_sensor_fusion_comparison(duration=30.0, dt=0.01):
    """Run sensor fusion and compare fused vs individual sensors.
    TODO:
    1. Simulate a quadrotor following a circular trajectory
    2. Generate IMU and GPS measurements
    3. Run the EKF to fuse both sensors
    4. Compare: GPS-only, IMU-only (dead reckoning), and fused estimate
    5. Plot all three estimates vs ground truth
    6. Save plot as task2_fusion_comparison.png
    """
    pass  # TODO


def test_sensor_failure_handling(duration=30.0, dt=0.01):
    """Test system behaviour under sensor failures.
    TODO:
    1. Simulate normal flight for 10s
    2. At t=10s, simulate GPS dropout (return None for all GPS)
    3. At t=20s, restore GPS
    4. Measure drift during dropout and recovery time
    5. Plot results and save as task2_failure.png
    """
    pass  # TODO


def compute_sensor_fusion_metrics(true_positions, gps_positions, fused_positions):
    """Compute comparison metrics.
    TODO: Compute RMSE for GPS-only and fused estimates.
    Return dict with 'gps_rmse' and 'fused_rmse'.
    """
    pass  # TODO


def main():
    print("Task 2: Sensor Fusion Integration")
    print("=" * 50)
    print("\nPart A: Sensor fusion comparison")
    run_sensor_fusion_comparison()
    print("\nPart B: Sensor failure handling")
    test_sensor_failure_handling()
    print("\nTask 2 complete.")


if __name__ == '__main__':
    main()
