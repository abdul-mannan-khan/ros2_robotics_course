#!/usr/bin/env python3
"""
Task 1 Solution: Rotation Math - Euler Angles and Quaternions
==============================================================
Complete implementation of rotation representations for quadrotor control.
"""
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def euler_to_rotation_matrix(phi, theta, psi):
    """
    Build rotation matrix from ZYX Euler angles.
    R = Rz(psi) * Ry(theta) * Rx(phi)
    """
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cs, ss = np.cos(psi), np.sin(psi)

    R = np.array([
        [ct*cs, cs*st*sp - ss*cp, cs*st*cp + ss*sp],
        [ct*ss, ss*st*sp + cs*cp, ss*st*cp - cs*sp],
        [-st,   ct*sp,            ct*cp            ]
    ])
    return R


def rotation_to_euler(R):
    """Extract ZYX Euler angles from rotation matrix."""
    theta = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    ct = np.cos(theta)
    if abs(ct) > 1e-6:
        phi = np.arctan2(R[2, 1], R[2, 2])
        psi = np.arctan2(R[1, 0], R[0, 0])
    else:
        phi = 0.0
        psi = np.arctan2(-R[0, 1], R[1, 1])
    return phi, theta, psi


def quaternion_from_euler(phi, theta, psi):
    """Convert ZYX Euler angles to quaternion [w, x, y, z]."""
    cp, sp = np.cos(phi/2), np.sin(phi/2)
    ct, st = np.cos(theta/2), np.sin(theta/2)
    cs, ss = np.cos(psi/2), np.sin(psi/2)
    return np.array([
        cp*ct*cs + sp*st*ss,
        sp*ct*cs - cp*st*ss,
        cp*st*cs + sp*ct*ss,
        cp*ct*ss - sp*st*cs,
    ])


def quaternion_to_euler(q):
    """Convert quaternion [w, x, y, z] to ZYX Euler angles."""
    w, x, y, z = q / np.linalg.norm(q)
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    phi = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2*(w*y - z*x)
    theta = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    psi = np.arctan2(siny_cosp, cosy_cosp)
    return phi, theta, psi


def quaternion_multiply(q1, q2):
    """Hamilton product."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quaternion_rotate_vector(q, v):
    """Rotate vector v by quaternion q."""
    qv = np.array([0.0, v[0], v[1], v[2]])
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    rotated = quaternion_multiply(quaternion_multiply(q, qv), q_conj)
    return rotated[1:]


def quaternion_to_rotation(q):
    """Quaternion to rotation matrix."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)    ],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)    ],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def test_gimbal_lock():
    """Demonstrate gimbal lock at theta = +/-90 degrees."""
    print("\n--- Gimbal Lock Test ---")
    theta = np.pi / 2  # 90 degrees pitch

    results = []
    for phi, psi in [(0.3, 0.0), (0.0, 0.3), (0.15, 0.15)]:
        R = euler_to_rotation_matrix(phi, theta, psi)
        phi_r, theta_r, psi_r = rotation_to_euler(R)
        results.append({
            'input': (phi, theta, psi),
            'recovered': (phi_r, theta_r, psi_r),
            'R': R.copy()
        })
        print(f"  Input:     phi={np.degrees(phi):7.2f}, theta={np.degrees(theta):7.2f}, psi={np.degrees(psi):7.2f}")
        print(f"  Recovered: phi={np.degrees(phi_r):7.2f}, theta={np.degrees(theta_r):7.2f}, psi={np.degrees(psi_r):7.2f}")

    # Show R matrices are different even though recovered angles may differ
    R_diff_01 = np.linalg.norm(results[0]['R'] - results[1]['R'])
    print(f"\n  ||R(0.3,90,0) - R(0,90,0.3)|| = {R_diff_01:.6f}")
    print(f"  At gimbal lock, only (phi - psi) or (phi + psi) matters,")
    print(f"  so one degree of freedom is lost.")
    return results


def main():
    print("=" * 60)
    print("Task 1 Solution: Rotation Math - Euler Angles & Quaternions")
    print("=" * 60)

    # Test 1: Euler -> R -> Euler roundtrip
    print("\n--- Test 1: Euler -> R -> Euler Roundtrip ---")
    test_angles = [
        (0, 0, 0),
        (0.3, 0.2, 0.5),
        (-0.4, 0.6, -1.0),
        (np.pi/4, np.pi/6, np.pi/3),
    ]
    max_err = 0
    for phi, theta, psi in test_angles:
        R = euler_to_rotation_matrix(phi, theta, psi)
        phi_r, theta_r, psi_r = rotation_to_euler(R)
        err = max(abs(phi - phi_r), abs(theta - theta_r), abs(psi - psi_r))
        max_err = max(max_err, err)
        print(f"  ({np.degrees(phi):7.2f}, {np.degrees(theta):7.2f}, {np.degrees(psi):7.2f}) deg "
              f"-> R -> ({np.degrees(phi_r):7.2f}, {np.degrees(theta_r):7.2f}, {np.degrees(psi_r):7.2f}) deg  "
              f"err={err:.2e}")
    print(f"  Max roundtrip error: {max_err:.2e} {'PASS' if max_err < 1e-10 else 'FAIL'}")

    # Test 2: Euler -> Quaternion -> Euler roundtrip
    print("\n--- Test 2: Euler -> Quaternion -> Euler Roundtrip ---")
    max_err = 0
    for phi, theta, psi in test_angles:
        q = quaternion_from_euler(phi, theta, psi)
        phi_r, theta_r, psi_r = quaternion_to_euler(q)
        err = max(abs(phi - phi_r), abs(theta - theta_r), abs(psi - psi_r))
        max_err = max(max_err, err)
        print(f"  ({np.degrees(phi):7.2f}, {np.degrees(theta):7.2f}, {np.degrees(psi):7.2f}) deg "
              f"-> q={q} -> ({np.degrees(phi_r):7.2f}, {np.degrees(theta_r):7.2f}, {np.degrees(psi_r):7.2f}) deg  "
              f"err={err:.2e}")
    print(f"  Max roundtrip error: {max_err:.2e} {'PASS' if max_err < 1e-10 else 'FAIL'}")

    # Test 3: R from Euler vs R from quaternion
    print("\n--- Test 3: R(Euler) vs R(Quaternion) Consistency ---")
    max_err = 0
    for phi, theta, psi in test_angles:
        R_euler = euler_to_rotation_matrix(phi, theta, psi)
        q = quaternion_from_euler(phi, theta, psi)
        R_quat = quaternion_to_rotation(q)
        err = np.max(np.abs(R_euler - R_quat))
        max_err = max(max_err, err)
    print(f"  Max difference between R(Euler) and R(Quaternion): {max_err:.2e} {'PASS' if max_err < 1e-10 else 'FAIL'}")

    # Test 4: Vector rotation consistency
    print("\n--- Test 4: Vector Rotation Consistency ---")
    v = np.array([1.0, 0.0, 0.0])
    max_err = 0
    for phi, theta, psi in test_angles:
        R = euler_to_rotation_matrix(phi, theta, psi)
        v_R = R @ v
        q = quaternion_from_euler(phi, theta, psi)
        v_q = quaternion_rotate_vector(q, v)
        err = np.linalg.norm(v_R - v_q)
        max_err = max(max_err, err)
    print(f"  Max difference R*v vs q*v*q_conj: {max_err:.2e} {'PASS' if max_err < 1e-10 else 'FAIL'}")

    # Test 5: Rotation matrix properties
    print("\n--- Test 5: Rotation Matrix Properties ---")
    for phi, theta, psi in test_angles:
        R = euler_to_rotation_matrix(phi, theta, psi)
        det = np.linalg.det(R)
        orth_err = np.max(np.abs(R @ R.T - np.eye(3)))
        print(f"  det(R)={det:.10f}, ||R*R^T - I||_max={orth_err:.2e}")

    # Test 6: Gimbal lock
    test_gimbal_lock()

    # Plot: visualize rotation of unit vectors
    print("\n--- Generating rotation visualization ---")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), subplot_kw={'projection': '3d'})

    for ax_idx, (phi, theta, psi) in enumerate([(0.5, 0, 0), (0, 0.5, 0), (0, 0, 0.5)]):
        ax = axes[ax_idx]
        R = euler_to_rotation_matrix(phi, theta, psi)
        origin = np.zeros(3)
        colors = ['r', 'g', 'b']
        labels = ['x', 'y', 'z']
        for i in range(3):
            # Original axis
            e = np.zeros(3)
            e[i] = 1.0
            ax.quiver(*origin, *e, color=colors[i], alpha=0.3, linewidth=1, label=f'{labels[i]} original')
            # Rotated axis
            e_rot = R @ e
            ax.quiver(*origin, *e_rot, color=colors[i], linewidth=2, label=f'{labels[i]} rotated')
        name = ['Roll', 'Pitch', 'Yaw'][ax_idx]
        angle_deg = np.degrees([phi, theta, psi][ax_idx])
        ax.set_title(f'{name} = {angle_deg:.0f} deg')
        ax.set_xlim([-1.2, 1.2])
        ax.set_ylim([-1.2, 1.2])
        ax.set_zlim([-1.2, 1.2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

    plt.suptitle('Task 1: Euler Angle Rotations (ZYX Convention)', fontsize=14)
    plt.tight_layout()
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task1_rotation_math.png')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"  Plot saved to {save_path}")
    plt.close()

    print("\n" + "=" * 60)
    print("Task 1 Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
