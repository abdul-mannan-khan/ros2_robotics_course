#!/usr/bin/env python3
"""
Week 8 - Task 4 Solution: Minimum Snap Trajectory Optimization
Complete minimum-snap trajectory generation through 3D waypoints.
"""

import numpy as np
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_waypoints():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises', 'data')
    return np.load(os.path.join(data_dir, 'waypoints.npy'))


def allocate_times(waypoints, average_speed=2.0):
    """Allocate time based on segment distances."""
    dists = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    segment_times = dists / average_speed
    times = np.concatenate([[0.0], np.cumsum(segment_times)])
    return times


def evaluate_polynomial(coeffs, t, derivative=0):
    """Evaluate polynomial or its derivative at time t."""
    n = len(coeffs) - 1
    result = 0.0
    for k in range(derivative, n + 1):
        # coefficient of t^(k-derivative) after differentiation
        factor = 1.0
        for j in range(k, k - derivative, -1):
            factor *= j
        result += factor * coeffs[k] * (t ** (k - derivative))
    return result


def minimum_snap_1d(waypoints_1d, times):
    """
    Minimum snap trajectory for one dimension using 7th order polynomials.

    Each segment has 8 coefficients. Constraints:
    - Position at each waypoint boundary
    - Zero vel, accel, jerk at start and end
    - Continuity of pos, vel, accel, jerk, snap, crackle at interior waypoints
    """
    n_seg = len(waypoints_1d) - 1
    order = 7  # 7th order polynomial -> 8 coefficients per segment
    n_coeffs = 8
    n_vars = n_seg * n_coeffs

    # Build constraint matrix
    rows = []
    rhs = []

    def poly_row(seg_idx, t, deriv):
        """Create a row for the constraint matrix for a given segment, time, and derivative."""
        row = np.zeros(n_vars)
        base = seg_idx * n_coeffs
        for k in range(deriv, n_coeffs):
            factor = 1.0
            for j in range(k, k - deriv, -1):
                factor *= j
            row[base + k] = factor * (t ** (k - deriv))
        return row

    # 1. Position constraints at waypoints
    for seg in range(n_seg):
        dt = times[seg + 1] - times[seg]
        # Start of segment
        row = poly_row(seg, 0.0, 0)
        rows.append(row)
        rhs.append(waypoints_1d[seg])
        # End of segment
        row = poly_row(seg, dt, 0)
        rows.append(row)
        rhs.append(waypoints_1d[seg + 1])

    # 2. Zero velocity, acceleration, jerk at start
    for deriv in range(1, 4):
        row = poly_row(0, 0.0, deriv)
        rows.append(row)
        rhs.append(0.0)

    # 3. Zero velocity, acceleration, jerk at end
    dt_last = times[-1] - times[-2]
    for deriv in range(1, 4):
        row = poly_row(n_seg - 1, dt_last, deriv)
        rows.append(row)
        rhs.append(0.0)

    # 4. Continuity at interior waypoints (derivatives 1 through 6)
    for wp in range(1, n_seg):
        dt_prev = times[wp] - times[wp - 1]
        for deriv in range(1, 7):
            row_end = poly_row(wp - 1, dt_prev, deriv)
            row_start = poly_row(wp, 0.0, deriv)
            row = row_end - row_start
            rows.append(row)
            rhs.append(0.0)

    A = np.array(rows)
    b = np.array(rhs)

    # If underdetermined, use least squares with minimum snap cost
    if A.shape[0] < n_vars:
        # Build snap cost matrix (Hessian of integral of snap^2)
        H = np.zeros((n_vars, n_vars))
        for seg in range(n_seg):
            dt = times[seg + 1] - times[seg]
            base = seg * n_coeffs
            for i in range(4, n_coeffs):
                for j in range(4, n_coeffs):
                    fi = 1.0
                    for k in range(i, i - 4, -1):
                        fi *= k
                    fj = 1.0
                    for k in range(j, j - 4, -1):
                        fj *= k
                    power = i + j - 8 + 1
                    H[base + i, base + j] = fi * fj * (dt ** power) / power

        # Solve via KKT system
        n_eq = A.shape[0]
        KKT = np.zeros((n_vars + n_eq, n_vars + n_eq))
        KKT[:n_vars, :n_vars] = H + 1e-10 * np.eye(n_vars)
        KKT[:n_vars, n_vars:] = A.T
        KKT[n_vars:, :n_vars] = A
        rhs_kkt = np.zeros(n_vars + n_eq)
        rhs_kkt[n_vars:] = b

        solution = np.linalg.solve(KKT, rhs_kkt)
        coeffs_flat = solution[:n_vars]
    else:
        # Exact or overdetermined
        coeffs_flat, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

    # Split into per-segment coefficients
    segments = []
    for seg in range(n_seg):
        segments.append(coeffs_flat[seg * n_coeffs:(seg + 1) * n_coeffs])
    return segments


def generate_3d_trajectory(waypoints_3d, times, dt=0.02):
    """Generate smooth 3D min-snap trajectory."""
    n_seg = len(waypoints_3d) - 1

    # Solve per axis
    seg_x = minimum_snap_1d(waypoints_3d[:, 0], times)
    seg_y = minimum_snap_1d(waypoints_3d[:, 1], times)
    seg_z = minimum_snap_1d(waypoints_3d[:, 2], times)

    # Evaluate trajectory
    t_total = times[-1]
    t_eval = np.arange(0, t_total, dt)
    pos = np.zeros((len(t_eval), 3))
    vel = np.zeros((len(t_eval), 3))
    acc = np.zeros((len(t_eval), 3))
    snap = np.zeros((len(t_eval), 3))

    for i, t in enumerate(t_eval):
        # Find segment
        seg = 0
        for s in range(n_seg):
            if t >= times[s] and (t < times[s + 1] or s == n_seg - 1):
                seg = s
                break
        t_local = t - times[seg]

        for axis, segs in enumerate([seg_x, seg_y, seg_z]):
            pos[i, axis] = evaluate_polynomial(segs[seg], t_local, 0)
            vel[i, axis] = evaluate_polynomial(segs[seg], t_local, 1)
            acc[i, axis] = evaluate_polynomial(segs[seg], t_local, 2)
            snap[i, axis] = evaluate_polynomial(segs[seg], t_local, 4)

    return {
        'time': t_eval,
        'position': pos,
        'velocity': vel,
        'acceleration': acc,
        'snap': snap,
    }


def check_feasibility(trajectory, v_max=5.0, a_max=10.0):
    """Check dynamic feasibility."""
    speeds = np.linalg.norm(trajectory['velocity'], axis=1)
    accels = np.linalg.norm(trajectory['acceleration'], axis=1)
    max_speed = np.max(speeds)
    max_accel = np.max(accels)
    return {
        'feasible': max_speed <= v_max and max_accel <= a_max,
        'max_speed': max_speed,
        'max_accel': max_accel,
    }


def main():
    waypoints = load_waypoints()
    print(f"Waypoints ({len(waypoints)}):\n{waypoints}")

    times = allocate_times(waypoints, average_speed=2.0)
    print(f"Time allocation: {times}")

    traj = generate_3d_trajectory(waypoints, times)
    feas = check_feasibility(traj)
    print(f"Feasibility: {feas}")

    # Create 6-panel figure
    fig = plt.figure(figsize=(18, 12))

    # 3D trajectory
    ax1 = fig.add_subplot(231, projection='3d')
    pos = traj['position']
    ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'b-', linewidth=1.5)
    ax1.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                c='red', s=60, zorder=5, label='Waypoints')
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Min-Snap Trajectory')
    ax1.legend()
    ax1.view_init(elev=25, azim=-60)

    t = traj['time']

    # XYZ position
    ax2 = fig.add_subplot(232)
    for i, label in enumerate(['X', 'Y', 'Z']):
        ax2.plot(t, pos[:, i], label=label)
    for wp_t in times:
        ax2.axvline(wp_t, color='gray', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Position (m)')
    ax2.set_title('Position Profiles'); ax2.legend(); ax2.grid(True, alpha=0.3)

    # Speed profile
    ax3 = fig.add_subplot(233)
    speeds = np.linalg.norm(traj['velocity'], axis=1)
    ax3.plot(t, speeds, 'b-')
    ax3.axhline(5.0, color='r', linestyle='--', label='v_max')
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Speed (m/s)')
    ax3.set_title(f'Speed (max={feas["max_speed"]:.2f} m/s)')
    ax3.legend(); ax3.grid(True, alpha=0.3)

    # Acceleration
    ax4 = fig.add_subplot(234)
    accels = np.linalg.norm(traj['acceleration'], axis=1)
    ax4.plot(t, accels, 'r-')
    ax4.axhline(10.0, color='k', linestyle='--', label='a_max')
    ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Accel (m/s^2)')
    ax4.set_title(f'Acceleration (max={feas["max_accel"]:.2f} m/s^2)')
    ax4.legend(); ax4.grid(True, alpha=0.3)

    # Snap profile
    ax5 = fig.add_subplot(235)
    snap_mag = np.linalg.norm(traj['snap'], axis=1)
    ax5.plot(t, snap_mag, 'g-')
    ax5.set_xlabel('Time (s)'); ax5.set_ylabel('Snap (m/s^4)')
    ax5.set_title('Snap Profile'); ax5.grid(True, alpha=0.3)

    # Velocity components
    ax6 = fig.add_subplot(236)
    for i, label in enumerate(['vx', 'vy', 'vz']):
        ax6.plot(t, traj['velocity'][:, i], label=label)
    ax6.set_xlabel('Time (s)'); ax6.set_ylabel('Velocity (m/s)')
    ax6.set_title('Velocity Components'); ax6.legend(); ax6.grid(True, alpha=0.3)

    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'task4_trajectory_optimization.png'), dpi=150)
    plt.close()
    print(f"Saved to {out_dir}/task4_trajectory_optimization.png")


if __name__ == '__main__':
    main()
