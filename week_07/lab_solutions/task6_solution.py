#!/usr/bin/env python3
"""
Task 6 Solution: Failsafe Handling
"""

import sys
import os
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from px4_sim import (
    PX4SITL, FlightMode, FLIGHT_MODE_NAMES, OffboardController, spin
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def configure_failsafes(vehicle, config):
    vehicle.failsafe_battery_warn = config.get('battery_warn', 30.0)
    vehicle.failsafe_battery_critical = config.get('battery_critical', 15.0)
    vehicle.failsafe_gps_timeout = config.get('gps_timeout', 3.0)
    vehicle.failsafe_comms_timeout = config.get('comms_timeout', 5.0)


def create_flying_vehicle(altitude=15.0, battery=100.0, position=None):
    """Helper: create a vehicle that is already flying at given position."""
    vehicle = PX4SITL(dt=0.05)
    vehicle.battery_remaining = max(battery + 5, battery)  # extra for climb
    ctrl = OffboardController(vehicle)
    ctrl.arm_and_takeoff(altitude=altitude)
    target_z = -abs(altitude)

    # If position given, fly there; otherwise hover at origin
    tx = position[0] if position else 0.0
    ty = position[1] if position else 0.0

    # Fly up first
    for _ in range(600):
        vehicle.send_position_setpoint(tx, ty, target_z)
        vehicle.step()
        if abs(vehicle.position[2] - target_z) < 1.0 and \
           np.linalg.norm(vehicle.position[:2] - np.array([tx, ty])) < 1.0:
            break

    # Set battery to desired level after climb
    vehicle.battery_remaining = battery
    vehicle.battery_voltage = 12.0 + 4.8 * (battery / 100.0)
    # Clear log so failsafe test starts fresh
    vehicle.log.clear()
    vehicle.failsafe_triggered = False
    vehicle.failsafe_reason = ""
    return vehicle


def simulate_battery_drain(vehicle, rate, duration):
    """Simulate battery drain."""
    vehicle.set_battery_drain_rate(rate)
    records = []
    start = vehicle.sim_time

    while vehicle.sim_time - start < duration:
        # Maintain setpoint stream to keep offboard alive
        vehicle.send_position_setpoint(vehicle.position[0], vehicle.position[1],
                                       vehicle.position[2])
        vehicle.step()
        records.append({
            'time': vehicle.sim_time - start,
            'battery': vehicle.battery_remaining,
            'mode': int(vehicle.flight_mode),
            'failsafe': vehicle.failsafe_triggered,
            'reason': vehicle.failsafe_reason,
            'altitude': -vehicle.position[2],
            'armed': vehicle.armed,
        })
        if not vehicle.armed and vehicle.landed:
            break

    return records


def simulate_gps_loss(vehicle, start_time, duration, sim_duration):
    records = []
    t0 = vehicle.sim_time

    while vehicle.sim_time - t0 < sim_duration:
        t_rel = vehicle.sim_time - t0
        # Toggle GPS
        if start_time <= t_rel < start_time + duration:
            vehicle.set_gps_available(False)
        else:
            vehicle.set_gps_available(True)

        # Keep sending setpoint to stay in offboard
        vehicle.send_position_setpoint(vehicle.position[0], vehicle.position[1],
                                       vehicle.position[2])
        vehicle.step()
        records.append({
            'time': t_rel,
            'gps': vehicle._gps_available,
            'mode': int(vehicle.flight_mode),
            'failsafe': vehicle.failsafe_triggered,
            'reason': vehicle.failsafe_reason,
            'altitude': -vehicle.position[2],
            'armed': vehicle.armed,
        })
        if not vehicle.armed and vehicle.landed and t_rel > start_time:
            break

    return records


def simulate_comm_loss(vehicle, start_time, duration, sim_duration):
    records = []
    t0 = vehicle.sim_time

    while vehicle.sim_time - t0 < sim_duration:
        t_rel = vehicle.sim_time - t0
        if start_time <= t_rel < start_time + duration:
            vehicle.set_comms_available(False)
        else:
            vehicle.set_comms_available(True)

        # Only send setpoints when comms available
        if vehicle._comms_available:
            vehicle.send_position_setpoint(vehicle.position[0], vehicle.position[1],
                                           vehicle.position[2])
        vehicle.step()
        records.append({
            'time': t_rel,
            'comms': vehicle._comms_available,
            'mode': int(vehicle.flight_mode),
            'failsafe': vehicle.failsafe_triggered,
            'reason': vehicle.failsafe_reason,
            'altitude': -vehicle.position[2],
            'armed': vehicle.armed,
        })
        if not vehicle.armed and vehicle.landed and t_rel > start_time:
            break

    return records


def test_failsafe_response(scenario):
    """Test and verify failsafe response."""
    if scenario == "low_battery":
        vehicle = create_flying_vehicle(battery=60.0, position=(30.0, 30.0))
        configure_failsafes(vehicle, {'battery_warn': 30.0, 'battery_critical': 10.0})
        records = simulate_battery_drain(vehicle, rate=0.5, duration=80.0)
        # Check that RTL was triggered
        rtl_triggered = any(r['mode'] == int(FlightMode.AUTO_RTL) for r in records)
        return {
            'scenario': scenario,
            'expected': 'AUTO_RTL',
            'triggered': rtl_triggered,
            'passed': rtl_triggered,
            'records': records,
        }

    elif scenario == "critical_battery":
        vehicle = create_flying_vehicle(battery=35.0, position=(30.0, 30.0))
        configure_failsafes(vehicle, {'battery_warn': 30.0, 'battery_critical': 15.0})
        records = simulate_battery_drain(vehicle, rate=0.8, duration=60.0)
        land_triggered = any(r['mode'] == int(FlightMode.AUTO_LAND) for r in records)
        return {
            'scenario': scenario,
            'expected': 'AUTO_LAND',
            'triggered': land_triggered,
            'passed': land_triggered,
            'records': records,
        }

    elif scenario == "gps_loss":
        vehicle = create_flying_vehicle()
        configure_failsafes(vehicle, {'gps_timeout': 3.0})
        records = simulate_gps_loss(vehicle, start_time=2.0, duration=10.0,
                                    sim_duration=30.0)
        land_triggered = any(r['mode'] == int(FlightMode.AUTO_LAND) for r in records)
        return {
            'scenario': scenario,
            'expected': 'AUTO_LAND',
            'triggered': land_triggered,
            'passed': land_triggered,
            'records': records,
        }

    elif scenario == "comm_loss":
        vehicle = create_flying_vehicle(position=(30.0, 30.0))
        configure_failsafes(vehicle, {'comms_timeout': 5.0})
        records = simulate_comm_loss(vehicle, start_time=2.0, duration=15.0,
                                     sim_duration=40.0)
        rtl_triggered = any(r['mode'] == int(FlightMode.AUTO_RTL) for r in records)
        return {
            'scenario': scenario,
            'expected': 'AUTO_RTL',
            'triggered': rtl_triggered,
            'passed': rtl_triggered,
            'records': records,
        }

    return {'scenario': scenario, 'passed': False, 'records': []}


def main():
    print("=" * 60)
    print("Task 6 Solution: Failsafe Handling")
    print("=" * 60)

    scenarios = ["low_battery", "critical_battery", "gps_loss", "comm_loss"]
    results = {}

    for scenario in scenarios:
        print(f"\n--- Testing: {scenario} ---")
        result = test_failsafe_response(scenario)
        results[scenario] = result
        status = "PASS" if result['passed'] else "FAIL"
        print(f"  Expected: {result.get('expected', 'N/A')}")
        print(f"  Triggered: {result.get('triggered', False)}")
        print(f"  Result: [{status}]")

    # Summary
    print("\n" + "=" * 40)
    total = len(scenarios)
    passed = sum(1 for r in results.values() if r['passed'])
    print(f"Failsafe Tests: {passed}/{total} passed")

    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    titles = {
        'low_battery': 'Low Battery -> RTL',
        'critical_battery': 'Critical Battery -> LAND',
        'gps_loss': 'GPS Loss -> LAND',
        'comm_loss': 'Comm Loss -> RTL',
    }

    for idx, scenario in enumerate(scenarios):
        ax = axes[idx // 2, idx % 2]
        records = results[scenario]['records']
        if not records:
            continue

        times = [r['time'] for r in records]
        alts = [r['altitude'] for r in records]
        modes = [r['mode'] for r in records]

        # Altitude
        ax.plot(times, alts, 'b-', linewidth=2, label='Altitude')

        # Mode coloring
        mode_colors = {
            int(FlightMode.OFFBOARD): 'lightblue',
            int(FlightMode.POSITION): 'lightgreen',
            int(FlightMode.AUTO_RTL): 'yellow',
            int(FlightMode.AUTO_LAND): 'lightyellow',
            int(FlightMode.AUTO_MISSION): 'lightcyan',
        }
        for i in range(len(times) - 1):
            c = mode_colors.get(modes[i], 'white')
            ax.axvspan(times[i], times[i+1], alpha=0.3, color=c)

        # Failsafe trigger point
        for r in records:
            if r['failsafe'] and r['reason']:
                ax.axvline(x=r['time'], color='red', linestyle='--', alpha=0.5)
                break

        # Scenario-specific overlay
        if scenario in ('low_battery', 'critical_battery'):
            ax2 = ax.twinx()
            batt = [r['battery'] for r in records]
            ax2.plot(times, batt, 'r--', linewidth=1.5, label='Battery %')
            ax2.set_ylabel('Battery %', color='red')
            ax2.tick_params(axis='y', labelcolor='red')
            ax2.legend(loc='upper right', fontsize=8)
        elif scenario == 'gps_loss':
            gps = [1 if r.get('gps', True) else 0 for r in records]
            ax.fill_between(times, 0, [a * g for a, g in zip(alts, gps)],
                            alpha=0.1, color='green', label='GPS OK')
        elif scenario == 'comm_loss':
            comms = [1 if r.get('comms', True) else 0 for r in records]
            ax.fill_between(times, 0, [a * c for a, c in zip(alts, comms)],
                            alpha=0.1, color='green', label='Comms OK')

        status = "PASS" if results[scenario]['passed'] else "FAIL"
        ax.set_title(f'{titles[scenario]} [{status}]')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Altitude (m)')
        ax.legend(loc='upper left', fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.suptitle('Task 6: Failsafe Handling & Testing', fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(SCRIPT_DIR, 'task6_failsafe.png'), dpi=150)
    print(f"\nPlot saved to {os.path.join(SCRIPT_DIR, 'task6_failsafe.png')}")
    print("Task 6 complete.")


if __name__ == '__main__':
    main()
