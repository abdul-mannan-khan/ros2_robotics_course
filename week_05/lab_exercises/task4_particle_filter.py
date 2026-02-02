#!/usr/bin/env python3
"""
Week 5 - Task 4: Particle Filter Localization (AMCL)
Implement an AMCL-style particle filter for robot localization.

AMCL (Adaptive Monte Carlo Localization) is Nav2's default localization method.
It uses a particle filter with:
  - Motion model: propagate particles with odometry + noise
  - Sensor model: likelihood field model for laser scans
  - Resampling: low-variance resampling when effective particle count drops
"""

import os
import numpy as np

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


def load_data():
    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    scans = np.load(os.path.join(DATA_DIR, 'laser_scans.npy'))
    scan_poses = np.load(os.path.join(DATA_DIR, 'scan_poses.npy'))
    return grid_map, metadata, scans, scan_poses


def initialize_particles(n_particles, map_bounds, grid_map=None, resolution=0.05):
    """Initialize particles uniformly in free space.

    Args:
        n_particles: number of particles
        map_bounds: (x_min, x_max, y_min, y_max) in meters
        grid_map: occupancy grid (to avoid placing particles in walls)
        resolution: map resolution

    Returns:
        particles: Nx3 array (x, y, theta)
        weights: N array, uniform (1/N)
    """
    # TODO: Implement particle initialization
    # Place particles in free cells only, random orientation
    raise NotImplementedError("Implement initialize_particles")


def motion_update(particles, odometry, noise_params):
    """Propagate particles using odometry motion model with noise.

    Odometry: (delta_x, delta_y, delta_theta) in robot frame
    Noise: (alpha1, alpha2, alpha3, alpha4) controlling rotation/translation noise

    Args:
        particles: Nx3 array
        odometry: (dx, dy, dtheta)
        noise_params: (a1, a2, a3, a4)

    Returns:
        updated particles Nx3
    """
    # TODO: Implement motion update with noise
    # For each particle, apply odometry + sampled noise
    raise NotImplementedError("Implement motion_update")


def sensor_update(particles, scan, grid_map, resolution=0.05, max_range=3.5,
                  sigma_hit=0.2, z_hit=0.9, z_random=0.1):
    """Update particle weights using likelihood field sensor model.

    For each particle:
      - For a subset of scan beams, compute where the beam endpoint falls
      - Look up the distance to the nearest obstacle in the map
      - Compute likelihood using Gaussian (sigma_hit)

    Args:
        particles: Nx3 array
        scan: 1D array of ranges
        grid_map: occupancy grid
        resolution: map resolution
        max_range: sensor max range
        sigma_hit: std dev for hit model
        z_hit: weight for hit component
        z_random: weight for random component

    Returns:
        weights: N array (unnormalized)
    """
    # TODO: Implement likelihood field sensor model
    # Hint: Precompute a distance transform of the map for efficiency
    raise NotImplementedError("Implement sensor_update")


def resample(particles, weights):
    """Low-variance resampling.

    Args:
        particles: Nx3 array
        weights: N array (normalized)

    Returns:
        new_particles: Nx3 resampled
        new_weights: N array (uniform 1/N)
    """
    # TODO: Implement low-variance resampling
    # Use a single random number and systematic step through CDF
    raise NotImplementedError("Implement resample")


def estimate_pose(particles, weights):
    """Estimate robot pose as weighted mean of particles.

    Args:
        particles: Nx3
        weights: N

    Returns:
        (x, y, theta) estimated pose
    """
    # TODO: Implement weighted mean estimation
    # Note: averaging angles requires circular mean
    raise NotImplementedError("Implement estimate_pose")


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    grid_map, metadata, scans, scan_poses = load_data()
    resolution = metadata[0]
    map_bounds = (0, 5, 0, 5)

    n_particles = 500
    particles, weights = initialize_particles(n_particles, map_bounds, grid_map, resolution)

    # Simulate robot moving along scan_poses
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))

    for i in range(min(len(scan_poses), 5)):
        ax = axes.flat[i]
        ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100,
                  extent=[0, 5, 0, 5])

        # Motion update (use difference between consecutive poses as odometry)
        if i > 0:
            dx = scan_poses[i, 0] - scan_poses[i-1, 0]
            dy = scan_poses[i, 1] - scan_poses[i-1, 1]
            dth = scan_poses[i, 2] - scan_poses[i-1, 2]
            particles = motion_update(particles, (dx, dy, dth),
                                      noise_params=(0.1, 0.1, 0.05, 0.05))

        # Sensor update
        weights = sensor_update(particles, scans[i], grid_map, resolution)
        weights /= weights.sum()

        # Estimate
        est = estimate_pose(particles, weights)

        # Plot
        ax.scatter(particles[:, 0], particles[:, 1], c='blue', s=1, alpha=0.3)
        ax.plot(scan_poses[i, 0], scan_poses[i, 1], 'r*', markersize=15, label='True')
        ax.plot(est[0], est[1], 'g^', markersize=10, label='Estimate')
        spread = np.std(particles[:, :2], axis=0).mean()
        ax.set_title(f'Step {i}, spread={spread:.3f}m')
        ax.legend(fontsize=8)

        # Resample
        particles, weights = resample(particles, weights)

    axes.flat[5].set_visible(False)
    plt.suptitle('AMCL Particle Filter Localization')
    plt.tight_layout()

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'task4_particle_filter.png')
    plt.savefig(out_path, dpi=100)
    print(f"Saved plot to {out_path}")


if __name__ == '__main__':
    main()
