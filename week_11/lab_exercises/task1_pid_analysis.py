#!/usr/bin/env python3
"""
Task 1: PID Control Analysis
=============================
Week 11 - Control Systems: PID to MPC
Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London

Objectives:
- Implement PID step response analysis
- Compare different tuning methods (Ziegler-Nichols, manual)
- Analyze frequency response characteristics
- Evaluate anti-windup and derivative filtering effects

TODO: Complete all functions marked with 'TODO'
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from control_sim import LinearSystem, PIDController, simulate_system, plot_response


def create_test_plant():
    """Create a second-order test plant: G(s) = 1/(s^2 + 2*zeta*wn*s + wn^2).
    State-space form with position and velocity states.
    """
    wn = 2.0   # natural frequency
    zeta = 0.3  # damping ratio
    A = np.array([[0, 1],
                  [-wn**2, -2*zeta*wn]])
    B = np.array([[0], [wn**2]])
    return LinearSystem(A, B)


def task1a_step_response():
    """TODO: Implement PID step response analysis.

    Steps:
    1. Create the test plant using create_test_plant()
    2. Design three PID controllers with different gains:
       - P-only: kp=1.0
       - PI: kp=1.0, ki=0.5
       - PID: kp=1.5, ki=1.0, kd=0.3
    3. Simulate each for 10 seconds with dt=0.01, step reference [1, 0]
    4. Plot all three responses on one figure
    5. Compute and print rise time, overshoot, settling time for each

    Returns: None (saves plot as 'task1a_step_response.png')
    """
    # TODO: Your implementation here
    pass


def task1b_ziegler_nichols():
    """TODO: Implement Ziegler-Nichols tuning.

    Steps:
    1. Use the test plant
    2. Find ultimate gain Ku by increasing kp until sustained oscillation
       (Hint: closed-loop poles on imaginary axis)
    3. Measure ultimate period Tu from the oscillation
    4. Apply Z-N table: kp=0.6*Ku, ki=kp/(0.5*Tu), kd=kp*0.125*Tu
    5. Simulate and compare with manual tuning
    6. Plot comparison

    Returns: None (saves plot as 'task1b_ziegler_nichols.png')
    """
    # TODO: Your implementation here
    pass


def task1c_frequency_analysis():
    """TODO: Analyze PID in the frequency domain.

    Steps:
    1. Create open-loop transfer function G(s)*C(s) for different PID gains
    2. Compute Bode plot data (magnitude and phase vs frequency)
    3. Find gain margin and phase margin
    4. Plot Bode diagrams
    5. Print stability margins

    Returns: None (saves plot as 'task1c_frequency.png')
    """
    # TODO: Your implementation here
    pass


def task1d_antiwindup_comparison():
    """TODO: Compare PID with and without anti-windup.

    Steps:
    1. Create plant with saturation (output_limits=(-5, 5))
    2. Design PID with high integral gain (ki=5.0)
    3. Simulate with anti-windup ON (integrator_limits=(-10, 10))
    4. Simulate with anti-windup OFF (integrator_limits=(-1e6, 1e6))
    5. Plot comparison showing windup effect
    6. Print analysis of the difference

    Returns: None (saves plot as 'task1d_antiwindup.png')
    """
    # TODO: Your implementation here
    pass


def main():
    """Run all Task 1 exercises."""
    print("=" * 70)
    print("Task 1: PID Control Analysis")
    print("=" * 70)

    print("\n--- Task 1a: Step Response ---")
    task1a_step_response()

    print("\n--- Task 1b: Ziegler-Nichols Tuning ---")
    task1b_ziegler_nichols()

    print("\n--- Task 1c: Frequency Analysis ---")
    task1c_frequency_analysis()

    print("\n--- Task 1d: Anti-Windup Comparison ---")
    task1d_antiwindup_comparison()

    print("\n[DONE] Task 1 complete. Check generated plots.")


if __name__ == '__main__':
    main()
