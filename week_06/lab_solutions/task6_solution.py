#!/usr/bin/env python3
"""
Task 6 Solution: Complementary Filter for Attitude Estimation
==============================================================
"""
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from quadrotor_sim import QuadrotorParams


def complementary_filter(gyro, accel, alpha, dt, prev_estimate):
    """
    One step of complementary filter for roll and pitch.

    Gyro integration (high-pass): angle += gyro * dt
    Accelerometer (low-pass): angle from atan2 of accel components
    Fusion: estimate = alpha * gyro_est + (1-alpha) * accel_est
    """
    # Gyro integration
    gyro_estimate = np.array([
        prev_estimate[0] + gyro[0] * dt,
        prev_estimate[1] + gyro[1] * dt,
    ])

    # Accelerometer-based angles
    # phi (roll) from atan2(ay, az), theta (pitch) from atan2(-ax, sqrt(ay^2 + az^2))
    ax, ay, az = accel
    accel_phi = np.arctan2(ay, az)
    accel_theta = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
    accel_estimate = np.array([accel_phi, accel_theta])

    # Complementary fusion
    estimate = alpha * gyro_estimate + (1 - alpha) * accel_estimate
    return estimate


def generate_imu_data(true_angles, true_rates, dt, noise_params):
    """
    Generate simulated noisy IMU data from ground truth.

    Args:
        true_angles: (N, 3) true [phi, theta, psi]
        true_rates: (N, 3) true [p, q, r]
        dt: time step
        noise_params: dict with 'gyro_noise', 'accel_noise', 'gyro_bias'
    Returns:
        imu_data: dict with 'gyro' (N,3), 'accel' (N,3)
    """
    N = len(true_angles)
    g = 9.81

    # Gyro: true rates + noise + bias
    gyro_noise = noise_params.get('gyro_noise', 0.01)
    gyro_bias = noise_params.get('gyro_bias', np.array([0.01, -0.005, 0.002]))
    gyro = true_rates + gyro_noise * np.random.randn(N, 3) + gyro_bias

    # Accel: gravity rotated into body frame + noise
    accel_noise = noise_params.get('accel_noise', 0.5)
    accel = np.zeros((N, 3))
    for i in range(N):
        phi, theta = true_angles[i, 0], true_angles[i, 1]
        # Gravity in body frame (simplified for small angles, but exact here)
        accel[i, 0] = -g * np.sin(theta)
        accel[i, 1] = g * np.sin(phi) * np.cos(theta)
        accel[i, 2] = g * np.cos(phi) * np.cos(theta)
    accel += accel_noise * np.random.randn(N, 3)

    return {'gyro': gyro, 'accel': accel}


def run_filter(imu_data, alpha, dt):
    """Process full IMU sequence with complementary filter."""
    N = len(imu_data['gyro'])
    estimates = np.zeros((N, 2))
    estimates[0] = [0.0, 0.0]

    for i in range(1, N):
        estimates[i] = complementary_filter(
            imu_data['gyro'][i], imu_data['accel'][i],
            alpha, dt, estimates[i-1]
        )
    return estimates


def compare_alpha_values(imu_data, true_angles, dt):
    """Test different alpha values."""
    alphas = [0.0, 0.5, 0.9, 0.95, 0.98, 0.99, 1.0]
    results = {}
    for alpha in alphas:
        est = run_filter(imu_data, alpha, dt)
        rmse_phi = np.sqrt(np.mean((est[:, 0] - true_angles[:, 0])**2))
        rmse_theta = np.sqrt(np.mean((est[:, 1] - true_angles[:, 1])**2))
        rmse_total = np.sqrt(rmse_phi**2 + rmse_theta**2)
        results[alpha] = {
            'rmse_phi': rmse_phi,
            'rmse_theta': rmse_theta,
            'rmse_total': rmse_total,
            'estimates': est,
        }
    return results


def main():
    print("=" * 60)
    print("Task 6 Solution: Complementary Filter")
    print("=" * 60)

    np.random.seed(42)
    dt = 0.005
    t_final = 20.0
    N = int(t_final / dt)
    times = np.linspace(0, t_final, N)

    # Generate a realistic flight trajectory
    print("\n--- Generating simulated flight trajectory ---")
    true_angles = np.zeros((N, 3))
    true_rates = np.zeros((N, 3))

    for i in range(N):
        t = times[i]
        # Sinusoidal attitude variations (like gentle maneuvering)
        true_angles[i, 0] = 0.15 * np.sin(0.8 * t) + 0.05 * np.sin(2.0 * t)   # roll
        true_angles[i, 1] = 0.10 * np.cos(0.6 * t) + 0.08 * np.sin(1.5 * t)   # pitch
        true_angles[i, 2] = 0.3 * np.sin(0.2 * t)                               # yaw
        true_rates[i, 0] = 0.15*0.8*np.cos(0.8*t) + 0.05*2.0*np.cos(2.0*t)
        true_rates[i, 1] = -0.10*0.6*np.sin(0.6*t) + 0.08*1.5*np.cos(1.5*t)
        true_rates[i, 2] = 0.3*0.2*np.cos(0.2*t)

    print(f"  Duration: {t_final}s, dt={dt}s, samples={N}")
    print(f"  Roll range:  [{np.degrees(true_angles[:,0].min()):.1f}, {np.degrees(true_angles[:,0].max()):.1f}] deg")
    print(f"  Pitch range: [{np.degrees(true_angles[:,1].min()):.1f}, {np.degrees(true_angles[:,1].max()):.1f}] deg")

    # Generate noisy IMU data
    print("\n--- Generating noisy IMU data ---")
    noise_params = {
        'gyro_noise': 0.02,    # rad/s
        'accel_noise': 0.8,    # m/s^2
        'gyro_bias': np.array([0.015, -0.01, 0.005]),  # rad/s
    }
    imu_data = generate_imu_data(true_angles, true_rates, dt, noise_params)
    print(f"  Gyro noise: {noise_params['gyro_noise']} rad/s")
    print(f"  Accel noise: {noise_params['accel_noise']} m/s^2")
    print(f"  Gyro bias: {noise_params['gyro_bias']} rad/s")

    # Run complementary filter with default alpha
    print("\n--- Running complementary filter (alpha=0.98) ---")
    estimates = run_filter(imu_data, 0.98, dt)
    rmse_phi = np.sqrt(np.mean((estimates[:, 0] - true_angles[:, 0])**2))
    rmse_theta = np.sqrt(np.mean((estimates[:, 1] - true_angles[:, 1])**2))
    print(f"  RMSE roll:  {np.degrees(rmse_phi):.3f} deg")
    print(f"  RMSE pitch: {np.degrees(rmse_theta):.3f} deg")

    # Compare alpha values
    print("\n--- Alpha Sweep ---")
    results = compare_alpha_values(imu_data, true_angles, dt)
    best_alpha = min(results.keys(), key=lambda a: results[a]['rmse_total'])
    print(f"  {'Alpha':>6s}  {'RMSE Roll (deg)':>15s}  {'RMSE Pitch (deg)':>16s}  {'Total':>10s}")
    for alpha in sorted(results.keys()):
        r = results[alpha]
        marker = ' <-- best' if alpha == best_alpha else ''
        print(f"  {alpha:6.2f}  {np.degrees(r['rmse_phi']):15.4f}  {np.degrees(r['rmse_theta']):16.4f}  "
              f"{np.degrees(r['rmse_total']):10.4f}{marker}")

    # Also compute gyro-only and accel-only for reference
    gyro_only = run_filter(imu_data, 1.0, dt)
    accel_only = run_filter(imu_data, 0.0, dt)

    # Plot
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    # Roll estimation
    ax = axes[0, 0]
    ax.plot(times, np.degrees(true_angles[:, 0]), 'k-', linewidth=2, label='True')
    ax.plot(times, np.degrees(estimates[:, 0]), 'b-', linewidth=1, label='CF (0.98)')
    ax.plot(times, np.degrees(gyro_only[:, 0]), 'r-', alpha=0.4, linewidth=0.7, label='Gyro only')
    ax.plot(times, np.degrees(accel_only[:, 0]), 'g-', alpha=0.4, linewidth=0.7, label='Accel only')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Roll (deg)')
    ax.set_title('Roll Estimation')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Pitch estimation
    ax = axes[0, 1]
    ax.plot(times, np.degrees(true_angles[:, 1]), 'k-', linewidth=2, label='True')
    ax.plot(times, np.degrees(estimates[:, 1]), 'b-', linewidth=1, label='CF (0.98)')
    ax.plot(times, np.degrees(gyro_only[:, 1]), 'r-', alpha=0.4, linewidth=0.7, label='Gyro only')
    ax.plot(times, np.degrees(accel_only[:, 1]), 'g-', alpha=0.4, linewidth=0.7, label='Accel only')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch (deg)')
    ax.set_title('Pitch Estimation')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Estimation error
    ax = axes[0, 2]
    err_phi = np.degrees(estimates[:, 0] - true_angles[:, 0])
    err_theta = np.degrees(estimates[:, 1] - true_angles[:, 1])
    ax.plot(times, err_phi, 'r-', linewidth=0.8, label='Roll error')
    ax.plot(times, err_theta, 'b-', linewidth=0.8, label='Pitch error')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (deg)')
    ax.set_title('Estimation Error (alpha=0.98)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Alpha sweep - RMSE
    ax = axes[1, 0]
    alphas_sorted = sorted(results.keys())
    rmse_rolls = [np.degrees(results[a]['rmse_phi']) for a in alphas_sorted]
    rmse_pitchs = [np.degrees(results[a]['rmse_theta']) for a in alphas_sorted]
    ax.plot(alphas_sorted, rmse_rolls, 'ro-', label='Roll RMSE')
    ax.plot(alphas_sorted, rmse_pitchs, 'bs-', label='Pitch RMSE')
    ax.axvline(x=best_alpha, color='g', linestyle='--', alpha=0.5, label=f'Best: {best_alpha}')
    ax.set_xlabel('Alpha')
    ax.set_ylabel('RMSE (deg)')
    ax.set_title('RMSE vs Alpha')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Different alpha roll comparisons
    ax = axes[1, 1]
    for alpha in [0.0, 0.5, 0.9, 0.98, 1.0]:
        est = results[alpha]['estimates']
        ax.plot(times, np.degrees(est[:, 0]), linewidth=0.8, label=f'a={alpha}')
    ax.plot(times, np.degrees(true_angles[:, 0]), 'k-', linewidth=2, label='True')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Roll (deg)')
    ax.set_title('Roll Estimate for Various Alpha')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Gyro drift visualization
    ax = axes[1, 2]
    ax.plot(times, np.degrees(gyro_only[:, 0] - true_angles[:, 0]), 'r-', label='Gyro drift (roll)')
    ax.plot(times, np.degrees(gyro_only[:, 1] - true_angles[:, 1]), 'b-', label='Gyro drift (pitch)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Drift (deg)')
    ax.set_title('Gyro-Only Drift (bias accumulation)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.suptitle('Task 6: Complementary Filter for Attitude Estimation', fontsize=14)
    plt.tight_layout()
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task6_complementary_filter.png')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n  Plot saved to {save_path}")
    plt.close()

    print("\n" + "=" * 60)
    print("Task 6 Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
