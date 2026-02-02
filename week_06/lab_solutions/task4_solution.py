#!/usr/bin/env python3
"""
Task 4 Solution: Motor Mixing and Allocation
==============================================
"""
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from quadrotor_sim import QuadrotorParams, hover_motor_speed


def build_mixing_matrix(params):
    """Construct the 4x4 mixing matrix."""
    kt = params.k_thrust
    kq = params.k_torque
    L = params.arm_length

    M = np.array([
        [kt,     kt,     kt,     kt    ],
        [0,      L*kt,   0,     -L*kt  ],
        [L*kt,   0,     -L*kt,  0      ],
        [-kq,    kq,    -kq,    kq     ],
    ])
    return M


def allocate_motors(desired_thrust, desired_torques, params):
    """Inverse mixing: desired wrench to motor speeds."""
    M = build_mixing_matrix(params)
    M_inv = np.linalg.inv(M)
    wrench = np.array([desired_thrust, desired_torques[0], desired_torques[1], desired_torques[2]])
    w_sq = M_inv @ wrench
    # Handle negative w_sq (can happen with large torque demands)
    w_sq = np.maximum(w_sq, 0.0)
    motor_speeds = np.sqrt(w_sq)
    motor_speeds = np.clip(motor_speeds, params.motor_min, params.motor_max)
    return motor_speeds


def saturate_motors(motor_speeds_sq, limits, desired_wrench):
    """
    Handle motor saturation with thrust priority.
    Strategy: if any motor exceeds limits, scale torque commands down
    while preserving average (thrust).
    """
    min_sq = limits[0] ** 2
    max_sq = limits[1] ** 2

    # Compute average (proportional to thrust)
    avg = np.mean(motor_speeds_sq)

    # Clip average to valid range
    avg = np.clip(avg, min_sq, max_sq)

    # Compute deviations (proportional to torques)
    deviations = motor_speeds_sq - np.mean(motor_speeds_sq)

    # Find max scaling factor that keeps all motors in range
    scale = 1.0
    for i in range(4):
        if deviations[i] > 0:
            max_dev = max_sq - avg
            if deviations[i] > 1e-10:
                scale = min(scale, max_dev / deviations[i])
        elif deviations[i] < 0:
            min_dev = min_sq - avg
            if abs(deviations[i]) > 1e-10:
                scale = min(scale, min_dev / deviations[i])

    scale = max(scale, 0.0)
    result_sq = avg + scale * deviations
    result_sq = np.clip(result_sq, min_sq, max_sq)
    return np.sqrt(result_sq)


def test_mixing_roundtrip():
    """Verify forward/inverse consistency."""
    params = QuadrotorParams()
    M = build_mixing_matrix(params)
    M_inv = np.linalg.inv(M)

    identity_err = np.max(np.abs(M @ M_inv - np.eye(4)))
    print(f"  ||M * M_inv - I||_max = {identity_err:.2e}")

    # Roundtrip test: wrench -> motors -> wrench
    test_wrenches = [
        np.array([params.mass * params.g, 0, 0, 0]),           # hover
        np.array([params.mass * params.g, 0.1, 0, 0]),         # roll
        np.array([params.mass * params.g, 0, 0.1, 0]),         # pitch
        np.array([params.mass * params.g, 0, 0, 0.01]),        # yaw
        np.array([params.mass * params.g, 0.05, 0.05, 0.005]), # combined
    ]
    max_err = 0
    for wrench in test_wrenches:
        w_sq = M_inv @ wrench
        wrench_recovered = M @ w_sq
        err = np.max(np.abs(wrench - wrench_recovered))
        max_err = max(max_err, err)

    print(f"  Max roundtrip error: {max_err:.2e} {'PASS' if max_err < 1e-10 else 'FAIL'}")
    return max_err


def main():
    print("=" * 60)
    print("Task 4 Solution: Motor Mixing and Allocation")
    print("=" * 60)

    params = QuadrotorParams()
    w_hover = hover_motor_speed(params)

    # Test 1: Mixing roundtrip
    print("\n--- Test 1: Mixing Matrix Roundtrip ---")
    test_mixing_roundtrip()

    # Test 2: Hover allocation
    print("\n--- Test 2: Hover Allocation ---")
    hover_thrust = params.mass * params.g
    motors = allocate_motors(hover_thrust, [0, 0, 0], params)
    print(f"  Hover thrust: {hover_thrust:.4f} N")
    print(f"  Motor speeds: [{motors[0]:.2f}, {motors[1]:.2f}, {motors[2]:.2f}, {motors[3]:.2f}] rad/s")
    print(f"  Expected:     [{w_hover:.2f}, {w_hover:.2f}, {w_hover:.2f}, {w_hover:.2f}] rad/s")
    err = np.max(np.abs(motors - w_hover))
    print(f"  Max error: {err:.2e} {'PASS' if err < 0.01 else 'FAIL'}")

    # Test 3: Roll, pitch, yaw commands
    print("\n--- Test 3: Individual Axis Commands ---")
    M = build_mixing_matrix(params)
    test_cases = [
        ("Roll (+)",  hover_thrust, [0.1, 0, 0]),
        ("Roll (-)",  hover_thrust, [-0.1, 0, 0]),
        ("Pitch (+)", hover_thrust, [0, 0.1, 0]),
        ("Pitch (-)", hover_thrust, [0, -0.1, 0]),
        ("Yaw (+)",   hover_thrust, [0, 0, 0.01]),
        ("Yaw (-)",   hover_thrust, [0, 0, -0.01]),
    ]
    for name, thrust, torques in test_cases:
        motors = allocate_motors(thrust, torques, params)
        # Verify by forward mixing
        wrench_check = M @ (motors ** 2)
        print(f"  {name:12s}: motors=[{motors[0]:.1f}, {motors[1]:.1f}, {motors[2]:.1f}, {motors[3]:.1f}], "
              f"T_check={wrench_check[0]:.3f}")

    # Test 4: Saturation behavior
    print("\n--- Test 4: Saturation Behavior ---")
    M_inv = np.linalg.inv(M)

    # Large torque demand that would cause negative w^2
    large_wrench = np.array([hover_thrust, 2.0, 0, 0])
    w_sq_raw = M_inv @ large_wrench
    print(f"  Large roll command w^2: [{w_sq_raw[0]:.0f}, {w_sq_raw[1]:.0f}, {w_sq_raw[2]:.0f}, {w_sq_raw[3]:.0f}]")
    print(f"  Has negative values: {np.any(w_sq_raw < 0)}")

    motors_sat = saturate_motors(w_sq_raw, (params.motor_min, params.motor_max), large_wrench)
    print(f"  After saturation: [{motors_sat[0]:.1f}, {motors_sat[1]:.1f}, {motors_sat[2]:.1f}, {motors_sat[3]:.1f}]")
    wrench_sat = M @ (motors_sat ** 2)
    print(f"  Achieved wrench: T={wrench_sat[0]:.3f}, tau=[{wrench_sat[1]:.4f}, {wrench_sat[2]:.4f}, {wrench_sat[3]:.6f}]")
    print(f"  Thrust preserved within: {abs(wrench_sat[0] - hover_thrust)/hover_thrust*100:.1f}%")

    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Motor speeds for different commands
    ax = axes[0, 0]
    commands = ['Hover', '+Roll', '-Roll', '+Pitch', '-Pitch', '+Yaw', '-Yaw']
    all_motors = []
    all_motors.append(allocate_motors(hover_thrust, [0, 0, 0], params))
    for _, thrust, torques in test_cases:
        all_motors.append(allocate_motors(thrust, torques, params))
    all_motors = np.array(all_motors)
    x = np.arange(len(commands))
    width = 0.2
    for m_idx in range(4):
        ax.bar(x + m_idx*width, all_motors[:, m_idx], width, label=f'Motor {m_idx+1}')
    ax.set_xticks(x + 1.5*width)
    ax.set_xticklabels(commands, rotation=15)
    ax.set_ylabel('Motor Speed (rad/s)')
    ax.set_title('Motor Allocation for Various Commands')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    # Mixing matrix visualization
    ax = axes[0, 1]
    im = ax.imshow(M, cmap='RdBu', aspect='auto')
    ax.set_xticks(range(4))
    ax.set_xticklabels(['w1^2', 'w2^2', 'w3^2', 'w4^2'])
    ax.set_yticks(range(4))
    ax.set_yticklabels(['T', 'tau_x', 'tau_y', 'tau_z'])
    ax.set_title('Mixing Matrix M')
    for i in range(4):
        for j in range(4):
            ax.text(j, i, f'{M[i,j]:.2e}', ha='center', va='center', fontsize=7)
    plt.colorbar(im, ax=ax)

    # Saturation test: sweep roll torque
    ax = axes[1, 0]
    roll_torques = np.linspace(-1.0, 1.0, 100)
    motor_results = np.zeros((100, 4))
    thrust_achieved = np.zeros(100)
    for i, tau_x in enumerate(roll_torques):
        wrench = np.array([hover_thrust, tau_x, 0, 0])
        w_sq = M_inv @ wrench
        motors_s = saturate_motors(w_sq, (params.motor_min, params.motor_max), wrench)
        motor_results[i] = motors_s
        thrust_achieved[i] = (M @ (motors_s**2))[0]

    for m_idx in range(4):
        ax.plot(roll_torques, motor_results[:, m_idx], label=f'Motor {m_idx+1}')
    ax.axhline(y=params.motor_max, color='r', linestyle='--', alpha=0.5, label='Max speed')
    ax.set_xlabel('Roll Torque Command (N*m)')
    ax.set_ylabel('Motor Speed (rad/s)')
    ax.set_title('Motor Speeds vs Roll Torque (with saturation)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Thrust preservation under saturation
    ax = axes[1, 1]
    ax.plot(roll_torques, thrust_achieved, 'b-', label='Achieved thrust')
    ax.axhline(y=hover_thrust, color='r', linestyle='--', label='Desired thrust')
    ax.set_xlabel('Roll Torque Command (N*m)')
    ax.set_ylabel('Thrust (N)')
    ax.set_title('Thrust Preservation Under Saturation')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.suptitle('Task 4: Motor Mixing and Allocation', fontsize=14)
    plt.tight_layout()
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task4_motor_mixing.png')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n  Plot saved to {save_path}")
    plt.close()

    print("\n" + "=" * 60)
    print("Task 4 Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
