#!/usr/bin/env python3
"""
Week 5 - Task 4 Solution: Particle Filter Localization (AMCL)
Complete AMCL-style particle filter with likelihood field model.
"""

import os
import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises', 'data')
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


def load_data():
    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    scans = np.load(os.path.join(DATA_DIR, 'laser_scans.npy'))
    scan_poses = np.load(os.path.join(DATA_DIR, 'scan_poses.npy'))
    return grid_map, metadata, scans, scan_poses


def compute_distance_field(grid_map):
    """Precompute distance transform for likelihood field model."""
    occupied = (grid_map == 100).astype(float)
    # distance_transform_edt gives distance to nearest True cell
    # We want distance from each free cell to nearest obstacle
    dist = distance_transform_edt(1 - occupied)
    return dist


def initialize_particles(n_particles, map_bounds, grid_map=None, resolution=0.05):
    """Initialize particles uniformly in free space."""
    x_min, x_max, y_min, y_max = map_bounds
    particles = np.zeros((n_particles, 3))
    weights = np.ones(n_particles) / n_particles

    if grid_map is not None:
        free_cells = np.argwhere(grid_map == 0)
        if len(free_cells) > 0:
            indices = np.random.choice(len(free_cells), size=n_particles, replace=True)
            chosen = free_cells[indices]
            particles[:, 0] = chosen[:, 1] * resolution + np.random.uniform(-resolution/2, resolution/2, n_particles)
            particles[:, 1] = chosen[:, 0] * resolution + np.random.uniform(-resolution/2, resolution/2, n_particles)
            particles[:, 2] = np.random.uniform(-np.pi, np.pi, n_particles)
            return particles, weights

    particles[:, 0] = np.random.uniform(x_min, x_max, n_particles)
    particles[:, 1] = np.random.uniform(y_min, y_max, n_particles)
    particles[:, 2] = np.random.uniform(-np.pi, np.pi, n_particles)
    return particles, weights


def motion_update(particles, odometry, noise_params):
    """Propagate particles with odometry and noise."""
    dx, dy, dtheta = odometry
    a1, a2, a3, a4 = noise_params
    n = len(particles)

    # Add noise proportional to motion magnitude
    trans = np.sqrt(dx**2 + dy**2)
    rot1 = np.arctan2(dy, dx) if trans > 1e-6 else 0
    rot2 = dtheta - rot1

    for i in range(n):
        noisy_rot1 = rot1 + np.random.normal(0, a1 * abs(rot1) + a2 * trans + 0.01)
        noisy_trans = trans + np.random.normal(0, a3 * trans + a4 * (abs(rot1) + abs(rot2)) + 0.01)
        noisy_rot2 = rot2 + np.random.normal(0, a1 * abs(rot2) + a2 * trans + 0.01)

        particles[i, 0] += noisy_trans * np.cos(particles[i, 2] + noisy_rot1)
        particles[i, 1] += noisy_trans * np.sin(particles[i, 2] + noisy_rot1)
        particles[i, 2] += noisy_rot1 + noisy_rot2

    return particles


def sensor_update(particles, scan, grid_map, dist_field, resolution=0.05,
                  max_range=3.5, sigma_hit=0.5, z_hit=0.7, z_random=0.3,
                  num_beams_used=12):
    """Likelihood field sensor model."""
    n = len(particles)
    weights = np.ones(n)
    height, width = grid_map.shape
    num_beams = len(scan)
    # Subsample beams for efficiency
    beam_indices = np.linspace(0, num_beams - 1, num_beams_used, dtype=int)
    angles = np.linspace(-np.pi, np.pi, num_beams, endpoint=False)

    for i in range(n):
        px, py, ptheta = particles[i]
        log_prob = 0

        for bi in beam_indices:
            r = scan[bi]
            if r >= max_range:
                continue
            beam_angle = ptheta + angles[bi]
            # Endpoint in world
            ex = px + r * np.cos(beam_angle)
            ey = py + r * np.sin(beam_angle)
            # Convert to grid
            gx = int(ex / resolution)
            gy = int(ey / resolution)

            if 0 <= gx < width and 0 <= gy < height:
                dist_to_obs = dist_field[gy, gx] * resolution
                p_hit = z_hit * np.exp(-0.5 * (dist_to_obs / sigma_hit)**2) / (sigma_hit * np.sqrt(2 * np.pi))
                p_rand = z_random / max_range
                p = p_hit + p_rand
            else:
                p = z_random / max_range

            log_prob += np.log(max(p, 1e-300))

        weights[i] = np.exp(log_prob)

    # Normalize
    total = weights.sum()
    if total > 0:
        weights /= total
    else:
        weights[:] = 1.0 / n
    return weights


def resample(particles, weights):
    """Low-variance resampling."""
    n = len(particles)
    new_particles = np.zeros_like(particles)
    r = np.random.uniform(0, 1.0 / n)
    c = weights[0]
    j = 0
    for i in range(n):
        u = r + i / n
        while u > c:
            j += 1
            if j >= n:
                j = n - 1
                break
            c += weights[j]
        new_particles[i] = particles[j]
    new_weights = np.ones(n) / n
    return new_particles, new_weights


def estimate_pose(particles, weights):
    """Weighted mean pose estimation."""
    x = np.average(particles[:, 0], weights=weights)
    y = np.average(particles[:, 1], weights=weights)
    # Circular mean for angle
    sin_mean = np.average(np.sin(particles[:, 2]), weights=weights)
    cos_mean = np.average(np.cos(particles[:, 2]), weights=weights)
    theta = np.arctan2(sin_mean, cos_mean)
    return np.array([x, y, theta])


def effective_particle_count(weights):
    """Compute effective sample size."""
    return 1.0 / np.sum(weights**2)


def main():
    print("=" * 60)
    print("Task 4: Particle Filter Localization (AMCL)")
    print("=" * 60)
    print()
    print("AMCL uses a particle filter to estimate the robot's pose.")
    print("Particles converge as sensor observations disambiguate location.")
    print()

    grid_map, metadata, scans, scan_poses = load_data()
    resolution = metadata[0]
    map_bounds = (0, 5, 0, 5)

    dist_field = compute_distance_field(grid_map)

    n_particles = 500
    particles, weights = initialize_particles(n_particles, map_bounds, grid_map, resolution)

    # Create a smooth trajectory with many small steps
    # Robot moves from scan_poses[0] in small increments, getting scan at each step
    true_pose = scan_poses[0].copy()
    step_size = 0.1  # small odometry steps
    n_steps = 20

    # Generate a path: move +x, then +y
    true_poses = [true_pose.copy()]
    for _ in range(10):
        true_pose = true_pose.copy()
        true_pose[0] += step_size
        true_poses.append(true_pose.copy())
    for _ in range(10):
        true_pose = true_pose.copy()
        true_pose[1] += step_size
        true_pose[2] = np.pi / 2
        true_poses.append(true_pose.copy())
    true_poses = np.array(true_poses)

    # Simulate scans at each pose
    def sim_scan(pose, gm, res):
        px, py, theta = pose
        num_beams = 360; max_range = 3.5
        ranges = np.full(num_beams, max_range)
        angles = np.linspace(-np.pi, np.pi, num_beams, endpoint=False)
        h, w = gm.shape
        for ii, angle in enumerate(angles):
            ba = theta + angle
            for step in range(1, int(max_range / res) + 1):
                d = step * res
                wx = px + d * np.cos(ba)
                wy = py + d * np.sin(ba)
                gx = int(wx / res); gy = int(wy / res)
                if gx < 0 or gx >= w or gy < 0 or gy >= h:
                    ranges[ii] = d; break
                if gm[gy, gx] == 100:
                    ranges[ii] = d; break
        return ranges

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    spreads = []
    errors = []

    # Pick 5 evenly spaced steps to visualize
    vis_steps = [0, 5, 10, 15, 20]

    print(f"Initialized {n_particles} particles in free space")
    print(f"Running {len(true_poses)} localization steps")
    print()

    vis_idx = 0
    for i in range(len(true_poses)):
        # Motion update
        if i > 0:
            dx = true_poses[i, 0] - true_poses[i-1, 0]
            dy = true_poses[i, 1] - true_poses[i-1, 1]
            dth = true_poses[i, 2] - true_poses[i-1, 2]
            particles = motion_update(particles, (dx, dy, dth),
                                      noise_params=(0.05, 0.05, 0.02, 0.02))

        # Sensor update
        scan = sim_scan(true_poses[i], grid_map, resolution)
        weights = sensor_update(particles, scan, grid_map, dist_field, resolution)

        est = estimate_pose(particles, weights)
        spread = np.std(particles[:, :2], axis=0).mean()
        spreads.append(spread)
        neff = effective_particle_count(weights)
        error = np.hypot(est[0]-true_poses[i,0], est[1]-true_poses[i,1])
        errors.append(error)

        if i in vis_steps and vis_idx < 5:
            ax = axes.flat[vis_idx]
            ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100,
                      extent=[0, 5, 0, 5])
            w_norm = weights / weights.max() if weights.max() > 0 else weights
            ax.scatter(particles[:, 0], particles[:, 1], c=w_norm,
                       cmap='Blues', s=3, alpha=0.5)
            ax.plot(true_poses[i, 0], true_poses[i, 1], 'r*', markersize=15, label='True')
            ax.plot(est[0], est[1], 'g^', markersize=12, label='Estimate')
            ax.set_title(f'Step {i}: spread={spread:.3f}m, err={error:.3f}m', fontsize=11)
            ax.legend(fontsize=8)
            ax.set_xlim(0, 5); ax.set_ylim(0, 5)
            vis_idx += 1

            print(f"Step {i}: spread={spread:.3f}m, Neff={neff:.0f}, error={error:.3f}m")

        # Resample
        particles, weights = resample(particles, weights)

    # Convergence plot
    ax = axes.flat[5]
    ax.plot(spreads, 'bo-', markersize=4, linewidth=2, label='Spread')
    ax.plot(errors, 'rs-', markersize=4, linewidth=2, label='Error')
    ax.set_xlabel('Step')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Convergence')
    ax.legend()
    ax.grid(True)

    plt.suptitle('AMCL Particle Filter Localization', fontsize=15)
    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, 'task4_particle_filter.png')
    plt.savefig(out_path, dpi=100)
    print(f"\nSaved plot to {out_path}")

    print(f"\nInitial spread: {spreads[0]:.3f}m, Final spread: {spreads[-1]:.3f}m")
    print(f"Initial error: {errors[0]:.3f}m, Final error: {errors[-1]:.3f}m")
    if spreads[-1] < spreads[0]:
        print("Particles converged successfully!")
    else:
        print("Warning: Particles may not have fully converged.")

    print("\n--- Key Concepts ---")
    print("- Likelihood field model precomputes distances for fast lookup")
    print("- Low-variance resampling avoids particle depletion")
    print("- Effective particle count (Neff) indicates filter health")
    print("- Spread decreasing indicates convergence to true pose")


if __name__ == '__main__':
    main()
