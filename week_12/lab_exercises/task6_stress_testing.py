#!/usr/bin/env python3
"""
Task 6: Stress Testing
========================
TC70045E Week 12 - Dr. Abdul Manan Khan, UWL

Test system under adverse conditions.

TODO: Complete the functions marked with TODO
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capstone_sim import AutonomousDroneSystem, QuadrotorParams


def test_high_sensor_noise(noise_scales=[1.0, 2.0, 5.0, 10.0]):
    """Test with increasing sensor noise.
    TODO:
    1. Run same mission at each noise_scale
    2. Collect RMSE and success at each level
    3. Plot degradation curve, save as task6_noise.png
    """
    pass  # TODO


def test_gps_dropout():
    """Test with GPS dropouts.
    TODO:
    1. Create GPS with dropout_prob=0.3
    2. Run mission, measure degradation
    3. Compare with normal GPS
    """
    pass  # TODO


def test_wind_disturbance():
    """Test with simulated wind.
    TODO:
    1. Add wind force to dynamics
    2. Test increasing wind speeds
    3. Plot tracking degradation
    """
    pass  # TODO


def test_multiple_failures():
    """Test combined failures.
    TODO:
    1. High noise + GPS dropout + wind
    2. Measure if mission still completes
    """
    pass  # TODO


def main():
    print("Task 6: Stress Testing")
    print("=" * 50)
    print("\nTest A: Sensor noise")
    test_high_sensor_noise()
    print("\nTest B: GPS dropout")
    test_gps_dropout()
    print("\nTest C: Wind")
    test_wind_disturbance()
    print("\nTest D: Combined failures")
    test_multiple_failures()
    print("\nTask 6 complete.")


if __name__ == '__main__':
    main()
