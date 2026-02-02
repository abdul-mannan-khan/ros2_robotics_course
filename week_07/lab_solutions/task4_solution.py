#!/usr/bin/env python3
"""
Task 4 Solution: Mission Planning
"""

import sys
import os
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from px4_sim import (
    PX4SITL, MissionItem, MAVCmd, FlightMode, FLIGHT_MODE_NAMES,
    OffboardController, spin
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def create_mission_item(x, y, z, command=MAVCmd.NAV_WAYPOINT, params=None):
    if params is None:
        params = {}
    return MissionItem(
        command=command,
        x=x, y=y, z=z,
        param1=params.get('hold_time', 0.0),
        param2=params.get('acceptance_radius', 1.5),
        param3=params.get('pass_radius', 0.0),
        param4=params.get('yaw', 0.0),
    )


def upload_mission(vehicle, items):
    for i, item in enumerate(items):
        item.seq = i
    return vehicle.upload_mission(items)


def start_mission(vehicle):
    ok, msg = vehicle.start_mission()
    print(f"  Mission start: {'OK' if ok else 'FAIL'} - {msg}")
    return ok


def monitor_mission(vehicle, timeout=120.0):
    start = vehicle.sim_time
    total_wps = len(vehicle.mission_items)
    last_wp = -1

    while vehicle.sim_time - start < timeout:
        vehicle.step()
        if vehicle.mission_current != last_wp:
            last_wp = vehicle.mission_current
            pct = (last_wp / total_wps * 100) if total_wps > 0 else 0
            print(f"  WP {last_wp}/{total_wps} ({pct:.0f}%) "
                  f"t={vehicle.sim_time - start:.1f}s")

        if not vehicle.mission_running and vehicle.mission_current >= total_wps:
            return {
                'completed': True,
                'waypoints_reached': last_wp,
                'time': vehicle.sim_time - start,
            }

        # Also stop if landed after mission
        if not vehicle.armed and vehicle.landed and last_wp > 0:
            return {
                'completed': True,
                'waypoints_reached': last_wp,
                'time': vehicle.sim_time - start,
            }

    return {
        'completed': False,
        'waypoints_reached': last_wp,
        'time': timeout,
    }


def plan_survey_mission(center_x, center_y, width, height, altitude, spacing):
    """Generate lawnmower survey pattern."""
    items = []
    alt_ned = -abs(altitude)

    # Takeoff item
    items.append(MissionItem(
        command=MAVCmd.NAV_TAKEOFF,
        x=center_x - width / 2,
        y=center_y - height / 2,
        z=alt_ned,
        param2=2.0,
    ))

    # Lawnmower lines
    x_start = center_x - width / 2
    x_end = center_x + width / 2
    y_start = center_y - height / 2
    y_end = center_y + height / 2

    num_lines = int(np.ceil(height / spacing)) + 1
    forward = True
    for i in range(num_lines):
        y = y_start + i * spacing
        if y > y_end:
            y = y_end
        if forward:
            items.append(create_mission_item(x_start, y, alt_ned,
                                             params={'acceptance_radius': 1.5}))
            items.append(create_mission_item(x_end, y, alt_ned,
                                             params={'acceptance_radius': 1.5}))
        else:
            items.append(create_mission_item(x_end, y, alt_ned,
                                             params={'acceptance_radius': 1.5}))
            items.append(create_mission_item(x_start, y, alt_ned,
                                             params={'acceptance_radius': 1.5}))
        forward = not forward

    # Land at end
    items.append(MissionItem(command=MAVCmd.NAV_LAND, x=center_x, y=center_y, z=0.0))

    return items


def main():
    print("=" * 60)
    print("Task 4 Solution: Mission Planning")
    print("=" * 60)

    # Create vehicle
    vehicle = PX4SITL(dt=0.02)

    # Arm via offboard
    ctrl = OffboardController(vehicle)
    ok, msg = ctrl.arm_and_takeoff(altitude=15.0)
    print(f"Takeoff: {msg}")

    # Wait for altitude
    for _ in range(500):
        vehicle.step()
    print(f"Altitude: {-vehicle.position[2]:.1f}m")

    # Plan survey
    print("\nPlanning survey mission...")
    items = plan_survey_mission(
        center_x=20.0, center_y=20.0,
        width=40.0, height=40.0,
        altitude=15.0, spacing=8.0,
    )
    print(f"Mission items: {len(items)}")

    # Upload and execute
    upload_mission(vehicle, items)
    start_mission(vehicle)

    result = monitor_mission(vehicle, timeout=300.0)
    print(f"\nMission {'completed' if result['completed'] else 'timed out'}")
    print(f"Waypoints: {result['waypoints_reached']}/{len(items)}")
    print(f"Time: {result['time']:.1f}s")

    # Extract data
    positions = np.array([e['position'] for e in vehicle.log])
    times = [e['time'] for e in vehicle.log]

    # Planned waypoints
    wp_x = [item.x for item in items if item.command == MAVCmd.NAV_WAYPOINT]
    wp_y = [item.y for item in items if item.command == MAVCmd.NAV_WAYPOINT]

    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    # Top-down survey pattern
    ax = axes[0, 0]
    ax.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=0.8, alpha=0.7, label='Actual')
    ax.plot(wp_x, wp_y, 'r--o', markersize=4, linewidth=0.8, label='Planned WPs')
    # Survey area
    cx, cy, w, h = 20, 20, 40, 40
    rect = plt.Rectangle((cx - w/2, cy - h/2), w, h,
                          fill=False, edgecolor='green', linestyle='--', linewidth=2)
    ax.add_patch(rect)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Survey Coverage (Top-Down)')
    ax.legend(fontsize=8)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Altitude profile
    ax = axes[0, 1]
    ax.plot(times, -positions[:, 2], 'b-')
    ax.axhline(y=15, color='r', linestyle='--', alpha=0.5, label='Target alt')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude Profile')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # XY over time
    ax = axes[1, 0]
    ax.plot(times, positions[:, 0], label='X')
    ax.plot(times, positions[:, 1], label='Y')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('X/Y Position vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Mission progress
    ax = axes[1, 1]
    modes = [e['mode'] for e in vehicle.log]
    mode_names_list = [FLIGHT_MODE_NAMES.get(FlightMode(m), str(m)) for m in modes]
    # Color-code modes
    unique_modes = list(set(modes))
    colors = plt.cm.Set1(np.linspace(0, 1, max(len(unique_modes), 1)))
    mode_color_map = {m: colors[i] for i, m in enumerate(unique_modes)}
    for i in range(len(times) - 1):
        ax.axvspan(times[i], times[i+1], alpha=0.3,
                   color=mode_color_map[modes[i]])
    # Legend
    for m in unique_modes:
        ax.barh(0, 0, color=mode_color_map[m], alpha=0.3,
                label=FLIGHT_MODE_NAMES.get(FlightMode(m), str(m)))
    ax.plot(times, [e['battery'] for e in vehicle.log], 'k-', linewidth=2, label='Battery %')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Battery %')
    ax.set_title('Mode Timeline & Battery')
    ax.legend(fontsize=7, loc='lower left')
    ax.grid(True, alpha=0.3)

    plt.suptitle('Task 4: Autonomous Survey Mission', fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(SCRIPT_DIR, 'task4_mission_planning.png'), dpi=150)
    print(f"\nPlot saved to {os.path.join(SCRIPT_DIR, 'task4_mission_planning.png')}")
    print("Task 4 complete.")


if __name__ == '__main__':
    main()
