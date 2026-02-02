#!/usr/bin/env python3
"""
Task 1: Rotation Math - Euler Angles and Quaternions
=====================================================
Implement rotation representations used in quadrotor attitude control.

Convention: ZYX Euler angles (yaw-pitch-roll), quaternion as [w, x, y, z].
"""
import numpy as np


def euler_to_rotation_matrix(phi, theta, psi):
    """
    Build rotation matrix from ZYX Euler angles.
    R = Rz(psi) * Ry(theta) * Rx(phi)

    Args:
        phi: roll angle (rad)
        theta: pitch angle (rad)
        psi: yaw angle (rad)
    Returns:
        R: 3x3 rotation matrix (body-to-world)
    """
    # TODO: Implement ZYX rotation matrix
    pass


def rotation_to_euler(R):
    """
    Extract ZYX Euler angles from rotation matrix.

    Args:
        R: 3x3 rotation matrix
    Returns:
        (phi, theta, psi): roll, pitch, yaw in radians
    """
    # TODO: Extract angles, handle gimbal lock when cos(theta) ~ 0
    pass


def quaternion_from_euler(phi, theta, psi):
    """
    Convert ZYX Euler angles to quaternion [w, x, y, z].

    Args:
        phi, theta, psi: roll, pitch, yaw (rad)
    Returns:
        q: quaternion as numpy array [w, x, y, z]
    """
    # TODO: Implement conversion
    pass


def quaternion_to_euler(q):
    """
    Convert quaternion [w, x, y, z] to ZYX Euler angles.

    Args:
        q: quaternion [w, x, y, z]
    Returns:
        (phi, theta, psi): roll, pitch, yaw (rad)
    """
    # TODO: Implement conversion
    pass


def quaternion_rotate_vector(q, v):
    """
    Rotate vector v by quaternion q using q * v * q_conj.

    Args:
        q: unit quaternion [w, x, y, z]
        v: 3D vector
    Returns:
        rotated vector (3,)
    """
    # TODO: Implement quaternion-vector rotation
    pass


def test_gimbal_lock():
    """
    Demonstrate gimbal lock at theta = +/-90 degrees.
    Show that Euler angles lose a degree of freedom.

    Returns:
        dict with test results
    """
    # TODO: Test rotation at theta=pi/2, show that different phi/psi
    # combinations give the same rotation matrix
    pass


def main():
    """Run all rotation math tests."""
    print("=" * 60)
    print("Task 1: Rotation Math - Euler Angles & Quaternions")
    print("=" * 60)

    # TODO: Test Euler -> R -> Euler roundtrip
    # TODO: Test Euler -> Quaternion -> Euler roundtrip
    # TODO: Compare R from Euler vs R from quaternion
    # TODO: Test vector rotation consistency
    # TODO: Run gimbal lock test
    print("\nImplement the functions above and test them!")


if __name__ == "__main__":
    main()
