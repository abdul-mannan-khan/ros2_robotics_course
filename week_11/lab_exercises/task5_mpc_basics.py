#!/usr/bin/env python3
"""
Task 5: MPC Basics
===================
Week 11 - Control Systems: PID to MPC
Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London

Objectives:
- Understand receding horizon control concept
- Formulate MPC optimization problem
- Solve QP with scipy.optimize.minimize
- Compare MPC with LQR for unconstrained/constrained cases

TODO: Complete all functions marked with 'TODO'
"""

import numpy as np
from scipy import linalg, optimize
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from control_sim import (LinearSystem, MPCController, LQRController,
                         simulate_system, plot_response)


def task5a_mpc_formulation():
    """TODO: Formulate and solve basic MPC.

    Steps:
    1. Double integrator: Ad, Bd from discretization (dt=0.1)
    2. Prediction horizon N=10
    3. Cost: Q=diag([10,1]), R=[[0.1]]
    4. No constraints
    5. Solve MPC for one step from x0=[2, 0], ref=[0, 0]
    6. Show predicted trajectory
    7. Compare first input with LQR

    Returns: None (saves plot as 'task5a_mpc_formulation.png')
    """
    # TODO: Your implementation here
    pass


def task5b_receding_horizon():
    """TODO: Implement receding horizon simulation.

    Steps:
    1. Same system as 5a
    2. Simulate 5 seconds with MPC (re-solve at each step)
    3. At each step, also plot the predicted trajectory
    4. Show how predictions update over time
    5. Compare closed-loop with LQR

    Returns: None (saves plot as 'task5b_receding_horizon.png')
    """
    # TODO: Your implementation here
    pass


def task5c_constrained_mpc():
    """TODO: MPC with input and state constraints.

    Steps:
    1. Double integrator with |u| <= 1.0 and |x[0]| <= 3.0
    2. Start from x0 = [2.5, 0] -> reference [0, 0]
    3. Solve constrained MPC (N=15)
    4. Compare with unconstrained MPC and LQR
    5. Show constraint satisfaction in plots
    6. Highlight where constraints are active

    Returns: None (saves plot as 'task5c_constrained.png')
    """
    # TODO: Your implementation here
    pass


def task5d_horizon_effect():
    """TODO: Study the effect of prediction horizon length.

    Steps:
    1. Same constrained system as 5c
    2. Try N = 3, 5, 10, 20, 30
    3. For each, simulate and record:
       - Settling time
       - Total cost
       - Computation time
    4. Plot results vs N
    5. Discuss optimal horizon selection

    Returns: None (saves plot as 'task5d_horizon.png')
    """
    # TODO: Your implementation here
    pass


def main():
    """Run all Task 5 exercises."""
    print("=" * 70)
    print("Task 5: MPC Basics")
    print("=" * 70)

    print("\n--- Task 5a: MPC Formulation ---")
    task5a_mpc_formulation()

    print("\n--- Task 5b: Receding Horizon ---")
    task5b_receding_horizon()

    print("\n--- Task 5c: Constrained MPC ---")
    task5c_constrained_mpc()

    print("\n--- Task 5d: Horizon Effect ---")
    task5d_horizon_effect()

    print("\n[DONE] Task 5 complete.")


if __name__ == '__main__':
    main()
