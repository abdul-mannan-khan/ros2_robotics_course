#!/usr/bin/env python3
"""
Task 2: Quadrotor Dynamics Model
==================================
Implement the equations of motion for a quadrotor UAV.

State: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
Convention: ENU/FLU (z-up), ZYX Euler angles.
"""
import numpy as np
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quadrotor_sim import QuadrotorParams, euler_to_rotation, motor_mixing_matrix


def compute_thrust_torque(motor_speeds, params):
    """
    Compute total thrust and torques from motor speeds using mixing matrix.

    Args:
        motor_speeds: [w1, w2, w3, w4] rad/s
        params: QuadrotorParams
    Returns:
        thrust: total thrust (scalar)
        torques: [tau_x, tau_y, tau_z]
    """
    # TODO: Apply mixing matrix to motor_speeds^2
    pass


def translational_dynamics(state, R, thrust, params):
    """
    Compute translational acceleration in world frame.
    F = m*a = R*[0,0,T]^T + [0,0,-mg]^T

    Args:
        state: full 12-state vector
        R: 3x3 rotation matrix (body to world)
        thrust: total thrust
        params: QuadrotorParams
    Returns:
        acceleration: [ax, ay, az] in world frame
    """
    # TODO: Compute translational acceleration
    pass


def rotational_dynamics(state, torques, params):
    """
    Compute angular acceleration using Euler's equation.
    tau = I * omega_dot + omega x (I * omega)

    Args:
        state: full 12-state vector
        torques: [tau_x, tau_y, tau_z] in body frame
        params: QuadrotorParams
    Returns:
        omega_dot: [p_dot, q_dot, r_dot]
    """
    # TODO: Implement Euler's rotational equation of motion
    pass


def full_dynamics(state, motor_speeds, params):
    """
    Compute complete state derivative.

    Args:
        state: 12-state vector
        motor_speeds: [w1, w2, w3, w4]
        params: QuadrotorParams
    Returns:
        state_dot: 12-element derivative vector
    """
    # TODO: Combine translational and rotational dynamics
    # Don't forget the kinematic relation between Euler rates and body rates
    pass


def main():
    """Test dynamics with free-fall, hover, and single-motor scenarios."""
    print("=" * 60)
    print("Task 2: Quadrotor Dynamics Model")
    print("=" * 60)

    # TODO: Test 1 - Free fall (all motors off), verify az = -g
    # TODO: Test 2 - Hover (equal motor speeds), verify az ~ 0
    # TODO: Test 3 - Single motor on, check torque generation
    print("\nImplement the functions and run the test scenarios!")


if __name__ == "__main__":
    main()
