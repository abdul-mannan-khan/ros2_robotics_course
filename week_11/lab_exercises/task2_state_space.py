#!/usr/bin/env python3
"""
Task 2: State-Space Modeling
==============================
Week 11 - Control Systems: PID to MPC
Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London

Objectives:
- Build state-space models from physical systems
- Check controllability and observability
- Simulate open-loop and closed-loop responses
- Convert between representations

TODO: Complete all functions marked with 'TODO'
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from control_sim import LinearSystem, QuadrotorLinearized, simulate_system, plot_response


def task2a_mass_spring_damper():
    """TODO: Create and analyze a mass-spring-damper system.

    System: m*x'' + c*x' + k*x = F
    Parameters: m=1.0, c=0.5, k=2.0
    State: [position, velocity]
    Input: [force]

    Steps:
    1. Derive A, B, C, D matrices
    2. Create LinearSystem
    3. Check eigenvalues, stability, controllability, observability
    4. Simulate open-loop step response (F=1.0 for t>0)
    5. Plot and save

    Returns: LinearSystem instance
    """
    # TODO: Your implementation here
    pass


def task2b_dc_motor():
    """TODO: Model a DC motor.

    States: [angular_position, angular_velocity, current]
    Input: [voltage]
    Parameters: R=1.0, L=0.01, J=0.01, b=0.1, Kt=0.01, Ke=0.01

    Steps:
    1. Derive state-space matrices
    2. Check properties
    3. Simulate step response
    4. Plot angular position and velocity

    Returns: LinearSystem instance
    """
    # TODO: Your implementation here
    pass


def task2c_quadrotor_analysis():
    """TODO: Analyze the linearized quadrotor model.

    Steps:
    1. Create QuadrotorLinearized instance
    2. Print all eigenvalues and classify stability
    3. Check controllability and observability
    4. Identify which modes are stable/unstable/marginally stable
    5. Discretize with dt=0.01 and compare eigenvalue locations
    6. Plot eigenvalue map (s-plane and z-plane side by side)

    Returns: None (saves plot as 'task2c_quadrotor_eigenvalues.png')
    """
    # TODO: Your implementation here
    pass


def task2d_controllability_observability():
    """TODO: Study controllability and observability in depth.

    Steps:
    1. Create a system that is controllable but not observable
    2. Create a system that is observable but not controllable
    3. Demonstrate with rank tests
    4. Show how uncontrollable/unobservable modes affect simulation
    5. Print educational summary

    Returns: None
    """
    # TODO: Your implementation here
    pass


def main():
    """Run all Task 2 exercises."""
    print("=" * 70)
    print("Task 2: State-Space Modeling")
    print("=" * 70)

    print("\n--- Task 2a: Mass-Spring-Damper ---")
    task2a_mass_spring_damper()

    print("\n--- Task 2b: DC Motor ---")
    task2b_dc_motor()

    print("\n--- Task 2c: Quadrotor Analysis ---")
    task2c_quadrotor_analysis()

    print("\n--- Task 2d: Controllability & Observability ---")
    task2d_controllability_observability()

    print("\n[DONE] Task 2 complete.")


if __name__ == '__main__':
    main()
