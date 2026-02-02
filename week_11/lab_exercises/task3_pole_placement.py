#!/usr/bin/env python3
"""
Task 3: Pole Placement & Eigenvalue Assignment
================================================
Week 11 - Control Systems: PID to MPC
Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London

Objectives:
- Apply pole placement for state feedback design
- Use Ackermann's formula
- Analyze stability regions and transient specifications
- Design controllers meeting time-domain specs

TODO: Complete all functions marked with 'TODO'
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal
from control_sim import LinearSystem, pole_placement, simulate_system, plot_response


def task3a_basic_pole_placement():
    """TODO: Basic pole placement for a double integrator.

    Plant: x'' = u  -> A=[[0,1],[0,0]], B=[[0],[1]]

    Steps:
    1. Place poles at s = -2 +/- 2j (damping ratio ~0.707)
    2. Compute gain K using pole_placement()
    3. Verify closed-loop eigenvalues = desired poles
    4. Simulate step response and plot
    5. Print K and closed-loop eigenvalues

    Returns: None (saves plot as 'task3a_pole_placement.png')
    """
    # TODO: Your implementation here
    pass


def task3b_specification_design():
    """TODO: Design controller from time-domain specifications.

    Given specs: settling time < 2s, overshoot < 10%

    Steps:
    1. Convert specs to desired pole locations
       (Hint: ts ~ 4/(zeta*wn), Mp = exp(-pi*zeta/sqrt(1-zeta^2)))
    2. Use the mass-spring-damper plant from Task 2
    3. Place poles to meet specs
    4. Simulate and verify specs are met
    5. Plot with spec lines (overshoot, settling time markers)

    Returns: None (saves plot as 'task3b_specs.png')
    """
    # TODO: Your implementation here
    pass


def task3c_stability_regions():
    """TODO: Visualize stability regions in the s-plane and z-plane.

    Steps:
    1. Create s-plane plot showing stable region (Re(s) < 0)
    2. Overlay lines of constant damping ratio and natural frequency
    3. Place poles at 5 different locations and show resulting responses
    4. Create z-plane plot showing unit circle
    5. Map continuous poles to discrete (z = e^(s*dt))

    Returns: None (saves plot as 'task3c_stability_regions.png')
    """
    # TODO: Your implementation here
    pass


def task3d_ackermann_formula():
    """TODO: Implement Ackermann's formula from scratch.

    Steps:
    1. Implement Ackermann's formula manually:
       K = [0 0 ... 1] * C^(-1) * phi_d(A)
       where C = controllability matrix, phi_d = desired char. polynomial
    2. Compare result with scipy.signal.place_poles
    3. Test on 3rd order system
    4. Print comparison and verify equality

    Returns: None
    """
    # TODO: Your implementation here
    pass


def main():
    """Run all Task 3 exercises."""
    print("=" * 70)
    print("Task 3: Pole Placement & Eigenvalue Assignment")
    print("=" * 70)

    print("\n--- Task 3a: Basic Pole Placement ---")
    task3a_basic_pole_placement()

    print("\n--- Task 3b: Specification-Based Design ---")
    task3b_specification_design()

    print("\n--- Task 3c: Stability Regions ---")
    task3c_stability_regions()

    print("\n--- Task 3d: Ackermann's Formula ---")
    task3d_ackermann_formula()

    print("\n[DONE] Task 3 complete.")


if __name__ == '__main__':
    main()
