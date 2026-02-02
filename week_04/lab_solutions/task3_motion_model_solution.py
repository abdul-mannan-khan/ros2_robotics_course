#!/usr/bin/env python3
"""
Week 4 - Task 3 Solution: Probabilistic Odometry Motion Model

Demonstrates dead reckoning drift and particle-based uncertainty propagation.
"""

import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab_exercises", "data")
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


def odometry_motion_model(x, u, noise_params):
    """
    Sample-based odometry motion model.
    x: [x, y, theta], u: [dx, dy, dtheta]
    noise_params: dict with alpha1..alpha4
    """
    dx, dy, dtheta = u
    dist = np.sqrt(dx**2 + dy**2)
    a1, a2, a3, a4 = noise_params['alpha1'], noise_params['alpha2'], noise_params['alpha3'], noise_params['alpha4']

    # Add noise proportional to motion
    noisy_dx = dx + np.random.normal(0, a1 * dist + a2 * abs(dtheta))
    noisy_dy = dy + np.random.normal(0, a1 * dist + a2 * abs(dtheta))
    noisy_dtheta = dtheta + np.random.normal(0, a3 * dist + a4 * abs(dtheta))

    new_x = x[0] + noisy_dx
    new_y = x[1] + noisy_dy
    new_theta = x[2] + noisy_dtheta
    new_theta = np.arctan2(np.sin(new_theta), np.cos(new_theta))

    return np.array([new_x, new_y, new_theta])


def propagate_particles(particles, u, noise_params):
    """Propagate all particles through the motion model."""
    new_particles = np.zeros_like(particles)
    for i in range(len(particles)):
        new_particles[i] = odometry_motion_model(particles[i], u, noise_params)
    return new_particles


def compute_pose_from_odometry(initial_pose, odometry_sequence):
    """Dead reckoning: integrate odometry sequentially."""
    N = len(odometry_sequence)
    poses = np.zeros((N, 3))
    poses[0] = initial_pose
    for i in range(1, N):
        poses[i, 0] = poses[i-1, 0] + odometry_sequence[i, 0]
        poses[i, 1] = poses[i-1, 1] + odometry_sequence[i, 1]
        poses[i, 2] = poses[i-1, 2] + odometry_sequence[i, 2]
        poses[i, 2] = np.arctan2(np.sin(poses[i, 2]), np.cos(poses[i, 2]))
    return poses


def main():
    print("=" * 60)
    print("Task 3: Probabilistic Odometry Motion Model")
    print("=" * 60)

    poses = np.load(os.path.join(DATA_DIR, "ground_truth_poses.npy"))
    odometry = np.load(os.path.join(DATA_DIR, "odometry.npy"))

    # Step 1: Dead reckoning
    print("\n[Step 1] Computing dead reckoning trajectory...")
    odom_poses = compute_pose_from_odometry(poses[0], odometry)
    pos_errors = np.sqrt((odom_poses[:, 0] - poses[:, 0])**2 + (odom_poses[:, 1] - poses[:, 1])**2)
    print(f"  Final position error: {pos_errors[-1]:.3f}m")
    print(f"  Max position error: {pos_errors.max():.3f}m")

    # Step 2: Particle propagation
    print("\n[Step 2] Propagating 200 particles through motion model...")
    num_particles = 200
    noise_params = {'alpha1': 0.05, 'alpha2': 0.01, 'alpha3': 0.01, 'alpha4': 0.05}
    particles = np.tile(poses[0], (num_particles, 1))

    # Record snapshots
    snapshot_indices = [0, 50, 100, 150, 200, 250, len(poses)-1]
    snapshot_indices = [s for s in snapshot_indices if s < len(poses)]
    particle_snapshots = {}

    for i in range(1, len(poses)):
        particles = propagate_particles(particles, odometry[i], noise_params)
        if i in snapshot_indices:
            particle_snapshots[i] = particles.copy()
            spread = np.std(particles[:, :2], axis=0)
            print(f"  Step {i}: position spread (std) = ({spread[0]:.3f}, {spread[1]:.3f})m")

    # Step 3: Visualization
    print("\n[Step 3] Creating visualization...")
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    # Trajectory comparison
    ax = axes[0, 0]
    ax.plot(poses[:, 0], poses[:, 1], 'b-', linewidth=2, label='Ground Truth')
    ax.plot(odom_poses[:, 0], odom_poses[:, 1], 'r--', linewidth=1.5, label='Dead Reckoning')
    ax.legend()
    ax.set_title("Trajectory: Ground Truth vs Dead Reckoning")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Position error over time
    ax = axes[0, 1]
    ax.plot(pos_errors, 'r-')
    ax.set_title("Dead Reckoning Position Error")
    ax.set_xlabel("Timestep")
    ax.set_ylabel("Error (m)")
    ax.grid(True, alpha=0.3)

    # Particle cloud snapshots
    ax = axes[1, 0]
    colors = plt.cm.viridis(np.linspace(0, 1, len(snapshot_indices)))
    for ci, idx in enumerate(snapshot_indices):
        if idx in particle_snapshots:
            p = particle_snapshots[idx]
            ax.scatter(p[:, 0], p[:, 1], s=2, c=[colors[ci]], alpha=0.5, label=f"Step {idx}")
    ax.plot(poses[:, 0], poses[:, 1], 'k-', linewidth=1, alpha=0.3)
    ax.legend(fontsize=7)
    ax.set_title("Particle Cloud Evolution")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect('equal')

    # Uncertainty growth
    ax = axes[1, 1]
    steps = sorted(particle_snapshots.keys())
    spreads_x = [np.std(particle_snapshots[s][:, 0]) for s in steps]
    spreads_y = [np.std(particle_snapshots[s][:, 1]) for s in steps]
    ax.plot(steps, spreads_x, 'b-o', label='x std')
    ax.plot(steps, spreads_y, 'r-o', label='y std')
    ax.set_title("Position Uncertainty Growth")
    ax.set_xlabel("Timestep")
    ax.set_ylabel("Standard Deviation (m)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, "task3_motion_model.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"  Saved: {out_path}")

    print("\n[Key Insight] Odometry drift grows unboundedly over time.")
    print("  Without corrections (e.g., scan matching), dead reckoning diverges.")
    print("  The particle cloud shows how uncertainty spreads with each step.")
    print("\nTask 3 complete.")


if __name__ == "__main__":
    main()
