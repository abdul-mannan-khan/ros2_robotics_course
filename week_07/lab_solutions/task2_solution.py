#!/usr/bin/env python3
"""
Task 2 Solution: Flight Mode Management
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from px4_sim import (
    PX4SITL, FlightMode, FLIGHT_MODE_NAMES, VALID_MODE_TRANSITIONS,
    MAVROSInterface, OffboardController, spin
)


def setup_vehicle():
    vehicle = PX4SITL(dt=0.02)
    return vehicle


def arm_vehicle(vehicle):
    ok, msg = vehicle.arm()
    print(f"  Arm: {'OK' if ok else 'FAIL'} - {msg}")
    return ok


def change_mode(vehicle, mode):
    mode_name = FLIGHT_MODE_NAMES.get(mode, str(mode))
    ok, msg = vehicle.set_mode(mode)
    print(f"  Mode -> {mode_name}: {'OK' if ok else 'FAIL'} - {msg}")
    return ok


def test_mode_transitions(vehicle):
    results = []
    tests = [
        (FlightMode.MANUAL, FlightMode.STABILIZED, True),
        (FlightMode.STABILIZED, FlightMode.POSITION, True),
        (FlightMode.POSITION, FlightMode.OFFBOARD, False),  # no setpoints
        (FlightMode.POSITION, FlightMode.MANUAL, True),
        (FlightMode.MANUAL, FlightMode.AUTO_MISSION, False),  # invalid from MANUAL
    ]

    for from_mode, to_mode, expected in tests:
        # Reset to from_mode if needed
        vehicle.flight_mode = from_mode
        ok, msg = vehicle.set_mode(to_mode)
        passed = ok == expected
        from_name = FLIGHT_MODE_NAMES[from_mode]
        to_name = FLIGHT_MODE_NAMES[to_mode]
        status = "PASS" if passed else "FAIL"
        print(f"  {from_name:15s} -> {to_name:15s}: "
              f"expected={'OK' if expected else 'DENY':4s}, "
              f"got={'OK' if ok else 'DENY':4s} [{status}]")
        results.append((from_mode, to_mode, ok))

    return results


def get_vehicle_state(vehicle):
    return {
        'armed': vehicle.armed,
        'mode': FLIGHT_MODE_NAMES[vehicle.flight_mode],
        'position': vehicle.position.tolist(),
        'battery': vehicle.battery_remaining,
        'gps_fix': vehicle.gps_fix,
        'in_air': vehicle.in_air,
    }


def print_state(label, vehicle):
    state = get_vehicle_state(vehicle)
    print(f"\n  [{label}]")
    for k, v in state.items():
        print(f"    {k}: {v}")


def main():
    print("=" * 60)
    print("Task 2 Solution: Flight Mode Management")
    print("=" * 60)

    # Setup
    vehicle = setup_vehicle()
    print_state("Initial State", vehicle)

    # Test mode transitions
    print("\n--- Mode Transition Tests ---")
    test_mode_transitions(vehicle)

    # Reset and do a proper sequence
    vehicle = setup_vehicle()
    print("\n--- Proper Flight Sequence ---")

    print("\nStep 1: MANUAL -> STABILIZED")
    change_mode(vehicle, FlightMode.STABILIZED)

    print("\nStep 2: STABILIZED -> POSITION")
    change_mode(vehicle, FlightMode.POSITION)

    print("\nStep 3: Arm in POSITION mode")
    arm_vehicle(vehicle)
    print_state("After Arm", vehicle)

    print("\nStep 4: Try OFFBOARD without setpoints")
    change_mode(vehicle, FlightMode.OFFBOARD)

    print("\nStep 5: Stream setpoints, then OFFBOARD")
    ctrl = OffboardController(vehicle)
    ctrl.start_setpoint_stream()
    ok = change_mode(vehicle, FlightMode.OFFBOARD)
    print_state("After Offboard", vehicle)

    print("\nStep 6: Back to POSITION")
    change_mode(vehicle, FlightMode.POSITION)

    # Disarm
    vehicle.in_air = False
    vehicle.landed = True
    ok, msg = vehicle.disarm()
    print(f"\nDisarm: {'OK' if ok else 'FAIL'} - {msg}")
    print_state("Final State", vehicle)

    # Show all valid transitions
    print("\n--- Valid Mode Transitions ---")
    for from_mode, to_modes in VALID_MODE_TRANSITIONS.items():
        targets = ', '.join(FLIGHT_MODE_NAMES[m] for m in to_modes)
        print(f"  {FLIGHT_MODE_NAMES[from_mode]:15s} -> {targets}")

    print("\nTask 2 complete.")


if __name__ == '__main__':
    main()
