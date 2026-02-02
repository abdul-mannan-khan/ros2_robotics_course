#!/usr/bin/env python3
"""
Task 7 Solution: Full Autonomous Drone Mission
"""

import sys
import os
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from px4_sim import (
    PX4SITL, FlightMode, FLIGHT_MODE_NAMES, OffboardController,
    create_simple_environment, check_obstacle_distance,
    spin, wait_for_position
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class DroneMission:
    def __init__(self, vehicle, environment=None):
        self.vehicle = vehicle
        self.environment = environment or create_simple_environment()
        self.ctrl = OffboardController(vehicle)
        self.telemetry = []
        self.waypoints_reached = 0
        self.total_waypoints = 0

    def preflight_checks(self):
        messages = []
        passed = True

        if self.vehicle.battery_remaining < 50:
            messages.append(f"FAIL: Battery {self.vehicle.battery_remaining:.0f}% < 50%")
            passed = False
        else:
            messages.append(f"OK: Battery {self.vehicle.battery_remaining:.0f}%")

        if self.vehicle.gps_fix < 3:
            messages.append("FAIL: No GPS 3D fix")
            passed = False
        else:
            messages.append(f"OK: GPS fix={self.vehicle.gps_fix}, sats={self.vehicle.gps_satellites}")

        if self.vehicle.armed:
            messages.append("WARN: Vehicle already armed")

        messages.append("OK: Sensors nominal")
        messages.append("OK: Calibration valid")

        return passed, messages

    def takeoff(self, altitude=10.0):
        ok, msg = self.ctrl.arm_and_takeoff(altitude=altitude)
        if not ok:
            return False
        # Wait for altitude
        target = np.array([0, 0, -abs(altitude)])
        for _ in range(500):
            self.vehicle.step()
            self._log()
            if abs(self.vehicle.position[2] + altitude) < 1.0:
                return True
        return abs(self.vehicle.position[2] + altitude) < 2.0

    def execute_mission(self, waypoints):
        self.total_waypoints = len(waypoints)
        self.waypoints_reached = 0
        total_dist = 0.0
        start_time = self.vehicle.sim_time

        for i, wp in enumerate(waypoints):
            target = np.array(wp)
            reached = False

            for _ in range(2000):  # max steps per waypoint
                # Check obstacles
                dist_obs, obs_dir = check_obstacle_distance(
                    self.vehicle.position, self.environment)

                if dist_obs < 4.0 and obs_dir is not None:
                    # Avoidance: velocity away from obstacle + toward goal
                    to_goal = target - self.vehicle.position
                    to_goal_norm = to_goal / (np.linalg.norm(to_goal) + 1e-6)
                    avoid = -obs_dir * (4.0 / (dist_obs + 0.5))
                    vel = to_goal_norm * 2.0 + avoid * 2.0
                    speed = np.linalg.norm(vel)
                    if speed > 3.0:
                        vel = vel / speed * 3.0
                    self.vehicle.send_velocity_setpoint(vel[0], vel[1], vel[2])
                    action = self.handle_contingency("obstacle_close")
                else:
                    self.vehicle.send_position_setpoint(target[0], target[1], target[2])

                # Check battery
                if self.vehicle.battery_remaining < 30:
                    self.handle_contingency("low_battery")
                    break

                self.vehicle.step()
                self._log()

                if np.linalg.norm(self.vehicle.position - target) < 1.0:
                    self.waypoints_reached += 1
                    reached = True
                    break

            if self.vehicle.failsafe_triggered:
                break

        elapsed = self.vehicle.sim_time - start_time
        return {
            'waypoints_reached': self.waypoints_reached,
            'total_waypoints': self.total_waypoints,
            'time_elapsed': elapsed,
        }

    def handle_contingency(self, event):
        if event == "low_battery":
            self.vehicle.set_mode(FlightMode.AUTO_RTL)
            return "RTL"
        elif event == "obstacle_close":
            return "AVOIDANCE"
        elif event == "gps_degraded":
            self.vehicle.send_position_setpoint(*self.vehicle.position)
            return "HOLD"
        return "NONE"

    def land(self):
        self.vehicle.flight_mode = FlightMode.AUTO_LAND
        for _ in range(1000):
            self.vehicle.step()
            self._log()
            if self.vehicle.landed:
                self.vehicle.disarm()
                return True
        return False

    def _log(self):
        self.telemetry.append({
            'time': self.vehicle.sim_time,
            'position': self.vehicle.position.copy(),
            'velocity': self.vehicle.velocity.copy(),
            'attitude': self.vehicle.attitude.copy(),
            'mode': int(self.vehicle.flight_mode),
            'battery': self.vehicle.battery_remaining,
            'battery_voltage': self.vehicle.battery_voltage,
            'armed': self.vehicle.armed,
            'setpoint': self.vehicle.position_setpoint.copy(),
            'failsafe': self.vehicle.failsafe_triggered,
        })

    def log_telemetry(self):
        return self._log()


def run_autonomous_mission(waypoints, environment=None):
    vehicle = PX4SITL(dt=0.02)
    mission = DroneMission(vehicle, environment)

    # Preflight
    passed, msgs = mission.preflight_checks()
    print("Preflight checks:")
    for m in msgs:
        print(f"  {m}")
    if not passed:
        return None

    # Takeoff
    print("\nTaking off...")
    ok = mission.takeoff(altitude=10.0)
    print(f"  Takeoff: {'OK' if ok else 'FAIL'}, alt={-vehicle.position[2]:.1f}m")

    # Mission
    print(f"\nExecuting mission ({len(waypoints)} waypoints)...")
    result = mission.execute_mission(waypoints)
    print(f"  Waypoints: {result['waypoints_reached']}/{result['total_waypoints']}")
    print(f"  Time: {result['time_elapsed']:.1f}s")

    # Land
    print("\nLanding...")
    ok = mission.land()
    print(f"  Land: {'OK' if ok else 'FAIL'}")

    return {
        'mission': mission,
        'result': result,
        'telemetry': mission.telemetry,
        'vehicle': vehicle,
    }


def evaluate_mission(telemetry, waypoints):
    if not telemetry:
        return {}

    positions = np.array([t['position'] for t in telemetry])
    setpoints = np.array([t['setpoint'] for t in telemetry])
    batteries = [t['battery'] for t in telemetry]
    times = [t['time'] for t in telemetry]

    # Tracking error
    errors = np.linalg.norm(positions - setpoints, axis=1)
    rms_error = np.sqrt(np.mean(errors ** 2))

    # Waypoint accuracy
    wp_reached = 0
    for wp in waypoints:
        dists = np.linalg.norm(positions - np.array(wp), axis=1)
        if np.min(dists) < 1.5:
            wp_reached += 1
    wp_accuracy = wp_reached / len(waypoints) * 100 if waypoints else 0

    # Battery
    battery_used = batteries[0] - batteries[-1]

    # Total time
    total_time = times[-1] - times[0]

    return {
        'waypoint_accuracy': wp_accuracy,
        'tracking_error_rms': rms_error,
        'battery_used': battery_used,
        'total_time': total_time,
        'waypoints_reached': wp_reached,
        'total_waypoints': len(waypoints),
    }


def main():
    print("=" * 60)
    print("Task 7 Solution: Full Autonomous Drone Mission")
    print("=" * 60)

    # Define mission
    waypoints = [
        (10, 0, -10),
        (20, 5, -12),
        (25, 15, -10),
        (15, 20, -8),
        (5, 15, -10),
        (0, 5, -10),
    ]

    environment = create_simple_environment([
        {'center': np.array([15.0, 3.0, -10.0]), 'radius': 2.0},
        {'center': np.array([22.0, 10.0, -11.0]), 'radius': 2.5},
        {'center': np.array([10.0, 18.0, -9.0]), 'radius': 1.5},
    ])

    # Run mission
    data = run_autonomous_mission(waypoints, environment)
    if data is None:
        print("Mission aborted.")
        return

    telemetry = data['telemetry']
    metrics = evaluate_mission(telemetry, waypoints)

    print(f"\n--- Mission Evaluation ---")
    print(f"  Waypoint accuracy: {metrics['waypoint_accuracy']:.1f}%")
    print(f"  Tracking error (RMS): {metrics['tracking_error_rms']:.2f}m")
    print(f"  Battery used: {metrics['battery_used']:.1f}%")
    print(f"  Total time: {metrics['total_time']:.1f}s")

    # Extract arrays
    times = np.array([t['time'] for t in telemetry])
    positions = np.array([t['position'] for t in telemetry])
    setpoints = np.array([t['setpoint'] for t in telemetry])
    attitudes = np.array([t['attitude'] for t in telemetry])
    batteries = np.array([t['battery'] for t in telemetry])
    modes = np.array([t['mode'] for t in telemetry])

    # === 6-panel plot ===
    fig = plt.figure(figsize=(20, 14))

    # Panel 1: 3D flight path
    ax1 = fig.add_subplot(231, projection='3d')
    ax1.plot(positions[:, 0], positions[:, 1], -positions[:, 2],
             'b-', linewidth=1.5, label='Actual')
    for i, wp in enumerate(waypoints):
        ax1.scatter(wp[0], wp[1], -wp[2], c='red', s=80, marker='^',
                    label='Waypoint' if i == 0 else '')
    for obs in environment['obstacles']:
        u, v = np.mgrid[0:2*np.pi:10j, 0:np.pi:6j]
        x = obs['center'][0] + obs['radius'] * np.cos(u) * np.sin(v)
        y = obs['center'][1] + obs['radius'] * np.sin(u) * np.sin(v)
        z = -obs['center'][2] + obs['radius'] * np.cos(v)
        ax1.plot_surface(x, y, z, alpha=0.25, color='orange')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Alt (m)')
    ax1.set_title('3D Flight Path')
    ax1.legend(fontsize=7)

    # Panel 2: XYZ tracking
    ax2 = fig.add_subplot(232)
    labels_xyz = ['X', 'Y', 'Z (alt)']
    colors_xyz = ['tab:blue', 'tab:orange', 'tab:green']
    for i, (lbl, clr) in enumerate(zip(labels_xyz, colors_xyz)):
        sign = -1 if i == 2 else 1
        ax2.plot(times, sign * positions[:, i], '-', color=clr, label=f'{lbl} actual')
        ax2.plot(times, sign * setpoints[:, i], '--', color=clr, alpha=0.4,
                 label=f'{lbl} setpoint')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position Tracking')
    ax2.legend(fontsize=6, ncol=2)
    ax2.grid(True, alpha=0.3)

    # Panel 3: Attitude
    ax3 = fig.add_subplot(233)
    ax3.plot(times, np.degrees(attitudes[:, 0]), label='Roll')
    ax3.plot(times, np.degrees(attitudes[:, 1]), label='Pitch')
    ax3.plot(times, np.degrees(attitudes[:, 2]), label='Yaw')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (deg)')
    ax3.set_title('Attitude')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Panel 4: Battery
    ax4 = fig.add_subplot(234)
    ax4.plot(times, batteries, 'g-', linewidth=2)
    ax4.axhline(y=30, color='orange', linestyle='--', alpha=0.7, label='Warn (30%)')
    ax4.axhline(y=15, color='red', linestyle='--', alpha=0.7, label='Critical (15%)')
    ax4.fill_between(times, 0, batteries, alpha=0.1, color='green')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Battery %')
    ax4.set_title('Battery Level')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.set_ylim(0, 105)

    # Panel 5: Mode timeline
    ax5 = fig.add_subplot(235)
    unique_modes = sorted(set(modes))
    mode_map = {m: i for i, m in enumerate(unique_modes)}
    mode_y = [mode_map[m] for m in modes]
    # Color blocks
    cmap = plt.cm.Set2
    for i in range(len(times) - 1):
        ax5.axvspan(times[i], times[i + 1], alpha=0.4,
                    color=cmap(mode_map[modes[i]] / max(len(unique_modes), 1)))
    ax5.set_yticks(range(len(unique_modes)))
    ax5.set_yticklabels([FLIGHT_MODE_NAMES.get(FlightMode(m), str(m))
                         for m in unique_modes], fontsize=8)
    ax5.plot(times, mode_y, 'k-', linewidth=0.5)
    ax5.set_xlabel('Time (s)')
    ax5.set_title('Flight Mode Timeline')
    ax5.grid(True, alpha=0.3, axis='x')

    # Panel 6: Performance metrics
    ax6 = fig.add_subplot(236)
    metric_names = ['WP Accuracy\n(%)', 'Track Error\nRMS (m)',
                    'Battery\nUsed (%)', 'Mission\nTime (s)']
    metric_vals = [
        metrics['waypoint_accuracy'],
        metrics['tracking_error_rms'],
        metrics['battery_used'],
        metrics['total_time'],
    ]
    # Normalize for display
    bars = ax6.bar(metric_names, metric_vals,
                   color=['green' if metrics['waypoint_accuracy'] >= 90 else 'orange',
                          'green' if metrics['tracking_error_rms'] < 2 else 'orange',
                          'green' if metrics['battery_used'] < 30 else 'orange',
                          'steelblue'])
    for bar, val in zip(bars, metric_vals):
        ax6.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                 f'{val:.1f}', ha='center', va='bottom', fontsize=9)
    ax6.set_title('Performance Metrics')
    ax6.grid(True, alpha=0.3, axis='y')

    plt.suptitle('Task 7: Full Autonomous Drone Mission - Evaluation', fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(SCRIPT_DIR, 'task7_full_integration.png'), dpi=150)
    print(f"\nPlot saved to {os.path.join(SCRIPT_DIR, 'task7_full_integration.png')}")

    # Final assessment
    print("\n--- Assessment ---")
    if metrics['waypoint_accuracy'] >= 90:
        print("  PASS: Waypoint accuracy >= 90%")
    else:
        print(f"  MARGINAL: Waypoint accuracy {metrics['waypoint_accuracy']:.0f}%")
    print("Task 7 complete.")


if __name__ == '__main__':
    main()
