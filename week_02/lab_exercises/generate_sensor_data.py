#!/usr/bin/env python3
"""
Generate synthetic sensor data for a differential drive robot
following a figure-8 (lemniscate) trajectory.

State vector: x = [px, py, theta, vx, vy, omega]^T

Sensors:
  - IMU at 100 Hz: (ax, ay, alpha)
  - Wheel encoders at 50 Hz: (v, omega)
  - GPS at 1 Hz: (px, py)
"""

import os
import numpy as np


def generate_figure8_trajectory(duration=60.0, dt=0.001, scale=5.0):
    """
    Generate ground truth trajectory along a lemniscate of Bernoulli.

    Parametric form:
        px(t) = scale * sin(w*t)
        py(t) = scale * sin(w*t) * cos(w*t)   [= scale/2 * sin(2*w*t)]

    where w = 2*pi / duration so the robot completes one full figure-8.
    """
    w = 2.0 * np.pi / duration
    t = np.arange(0.0, duration, dt)

    # Position
    px = scale * np.sin(w * t)
    py = scale * np.sin(w * t) * np.cos(w * t)  # = (scale/2)*sin(2wt)

    # Velocity (analytic derivatives)
    vx = scale * w * np.cos(w * t)
    vy = scale * w * (np.cos(w * t) ** 2 - np.sin(w * t) ** 2)  # cos(2wt)*scale*w

    # Heading from velocity direction
    theta = np.arctan2(vy, vx)

    # Angular velocity (finite difference of theta, unwrapped)
    theta_unwrapped = np.unwrap(theta)
    omega = np.gradient(theta_unwrapped, dt)

    # Acceleration (analytic second derivatives)
    ax = -scale * w ** 2 * np.sin(w * t)
    ay = -4.0 * scale * w ** 2 * np.sin(w * t) * np.cos(w * t)

    # Angular acceleration (finite difference of omega)
    alpha = np.gradient(omega, dt)

    return t, px, py, theta, vx, vy, omega, ax, ay, alpha


def sample_imu(t, ax, ay, alpha, rate=100.0,
               accel_std=0.1, gyro_std=0.01):
    """Sample IMU measurements at the given rate with additive Gaussian noise."""
    dt_sensor = 1.0 / rate
    indices = np.round(np.arange(0.0, t[-1], dt_sensor) / (t[1] - t[0])).astype(int)
    indices = indices[indices < len(t)]

    t_imu = t[indices]
    ax_meas = ax[indices] + np.random.normal(0, accel_std, len(indices))
    ay_meas = ay[indices] + np.random.normal(0, accel_std, len(indices))
    alpha_meas = alpha[indices] + np.random.normal(0, gyro_std, len(indices))

    return np.column_stack([t_imu, ax_meas, ay_meas, alpha_meas])


def sample_encoders(t, vx, vy, omega, rate=50.0,
                    v_std=0.05, omega_std=0.02):
    """Sample wheel encoder measurements at the given rate with noise."""
    dt_sensor = 1.0 / rate
    indices = np.round(np.arange(0.0, t[-1], dt_sensor) / (t[1] - t[0])).astype(int)
    indices = indices[indices < len(t)]

    t_enc = t[indices]
    # Speed magnitude
    v_true = np.sqrt(vx[indices] ** 2 + vy[indices] ** 2)
    v_meas = v_true + np.random.normal(0, v_std, len(indices))
    omega_meas = omega[indices] + np.random.normal(0, omega_std, len(indices))

    return np.column_stack([t_enc, v_meas, omega_meas])


def sample_gps(t, px, py, rate=1.0, pos_std=2.0):
    """Sample GPS measurements at the given rate with noise."""
    dt_sensor = 1.0 / rate
    indices = np.round(np.arange(0.0, t[-1], dt_sensor) / (t[1] - t[0])).astype(int)
    indices = indices[indices < len(t)]

    t_gps = t[indices]
    px_meas = px[indices] + np.random.normal(0, pos_std, len(indices))
    py_meas = py[indices] + np.random.normal(0, pos_std, len(indices))

    return np.column_stack([t_gps, px_meas, py_meas])


def main():
    np.random.seed(42)

    # Generate ground truth
    t, px, py, theta, vx, vy, omega, ax, ay, alpha = \
        generate_figure8_trajectory(duration=60.0, dt=0.001, scale=5.0)

    # Build ground truth array (Nx7): time, px, py, theta, vx, vy, omega
    ground_truth = np.column_stack([t, px, py, theta, vx, vy, omega])

    # Generate sensor data
    imu_data = sample_imu(t, ax, ay, alpha,
                          rate=100.0, accel_std=0.1, gyro_std=0.01)
    encoder_data = sample_encoders(t, vx, vy, omega,
                                   rate=50.0, v_std=0.05, omega_std=0.02)
    gps_data = sample_gps(t, px, py,
                          rate=1.0, pos_std=2.0)

    # Create output directory next to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, "data")
    os.makedirs(data_dir, exist_ok=True)

    # Save
    np.save(os.path.join(data_dir, "ground_truth.npy"), ground_truth)
    np.save(os.path.join(data_dir, "imu_data.npy"), imu_data)
    np.save(os.path.join(data_dir, "encoder_data.npy"), encoder_data)
    np.save(os.path.join(data_dir, "gps_data.npy"), gps_data)

    # Summary
    print("=== Sensor Data Generation Summary ===")
    print(f"Duration:        60.0 s")
    print(f"Ground truth:    {ground_truth.shape[0]} samples (Nx7)")
    print(f"IMU data:        {imu_data.shape[0]} samples at 100 Hz (Mx4)")
    print(f"Encoder data:    {encoder_data.shape[0]} samples at  50 Hz (Kx3)")
    print(f"GPS data:        {gps_data.shape[0]} samples at   1 Hz (Jx3)")
    print(f"Files saved to:  {data_dir}/")


if __name__ == "__main__":
    main()
