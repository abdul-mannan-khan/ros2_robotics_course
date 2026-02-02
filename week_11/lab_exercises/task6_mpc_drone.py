#!/usr/bin/env python3
"""
Task 6: MPC for Drone Trajectory Tracking
===========================================
Week 11 - Control Systems: PID to MPC
Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London

Objectives:
- Apply MPC to quadrotor trajectory tracking
- Handle thrust and angle constraints
- Track time-varying references (circle, figure-8)
- Evaluate real-time feasibility

TODO: Complete all functions marked with 'TODO'
"""

import numpy as np
from scipy import linalg
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from control_sim import (QuadrotorLinearized, MPCController, LQRController,
                         simulate_system, plot_response)


def generate_circle_trajectory(t, radius=2.0, height=5.0, omega=0.5):
    """Generate circular reference trajectory.
    Returns 12-state reference: [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r]
    """
    x_ref = np.zeros(12)
    x_ref[0] = radius * np.cos(omega * t)
    x_ref[1] = radius * np.sin(omega * t)
    x_ref[2] = height
    x_ref[3] = -radius * omega * np.sin(omega * t)
    x_ref[4] = radius * omega * np.cos(omega * t)
    return x_ref


def generate_figure8_trajectory(t, scale=2.0, height=5.0, omega=0.3):
    """Generate figure-8 reference trajectory."""
    x_ref = np.zeros(12)
    x_ref[0] = scale * np.sin(omega * t)
    x_ref[1] = scale * np.sin(2 * omega * t) / 2.0
    x_ref[2] = height
    x_ref[3] = scale * omega * np.cos(omega * t)
    x_ref[4] = scale * omega * np.cos(2 * omega * t)
    return x_ref


def task6a_hover_mpc():
    """TODO: MPC for quadrotor hover stabilization.

    Steps:
    1. Create QuadrotorLinearized, discretize with dt=0.05
    2. Use reduced model (z, vz, phi, theta, p, q) - 6 states, 3 inputs
       OR use full 12-state model
    3. Set up MPC with:
       - N=10, Q penalizing position/angles, R penalizing inputs
       - Thrust constraint: |delta_T| <= 5.0 N
       - Angle constraint: |phi|, |theta| <= 30 deg
    4. Simulate from perturbed hover (z offset, angle offset)
    5. Plot position and angle convergence

    Returns: None (saves plot as 'task6a_hover_mpc.png')
    """
    # TODO: Your implementation here
    pass


def task6b_circle_tracking():
    """TODO: MPC for circular trajectory tracking.

    Steps:
    1. Use reduced altitude + lateral model (6 states)
    2. Set up MPC with constraints
    3. Track circular trajectory using generate_circle_trajectory
    4. Plot 3D trajectory (actual vs reference)
    5. Plot tracking error over time

    Returns: None (saves plot as 'task6b_circle_tracking.png')
    """
    # TODO: Your implementation here
    pass


def task6c_figure8_tracking():
    """TODO: MPC for figure-8 trajectory tracking.

    Steps:
    1. Same setup as 6b but with figure-8 reference
    2. This is more challenging due to curvature changes
    3. Plot 2D XY trajectory (actual vs reference)
    4. Plot tracking error
    5. Compare performance with circle tracking

    Returns: None (saves plot as 'task6c_figure8.png')
    """
    # TODO: Your implementation here
    pass


def task6d_disturbance_rejection():
    """TODO: MPC disturbance rejection for quadrotor.

    Steps:
    1. Hover MPC from 6a
    2. Add wind disturbance: step force at t=3s in x-direction
    3. Add periodic gust at t=7s
    4. Show how MPC rejects disturbances
    5. Compare with LQR disturbance rejection
    6. Plot disturbance, state response, and control effort

    Returns: None (saves plot as 'task6d_disturbance.png')
    """
    # TODO: Your implementation here
    pass


def main():
    """Run all Task 6 exercises."""
    print("=" * 70)
    print("Task 6: MPC for Drone Trajectory Tracking")
    print("=" * 70)

    print("\n--- Task 6a: Hover MPC ---")
    task6a_hover_mpc()

    print("\n--- Task 6b: Circle Tracking ---")
    task6b_circle_tracking()

    print("\n--- Task 6c: Figure-8 Tracking ---")
    task6c_figure8_tracking()

    print("\n--- Task 6d: Disturbance Rejection ---")
    task6d_disturbance_rejection()

    print("\n[DONE] Task 6 complete.")


if __name__ == '__main__':
    main()
