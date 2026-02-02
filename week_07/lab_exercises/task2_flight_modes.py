#!/usr/bin/env python3
"""
Task 2: Flight Mode Management
================================
Understand PX4 flight modes and transitions.

Objectives:
- Initialize and configure a simulated PX4 vehicle
- Arm/disarm with proper safety checks
- Navigate flight mode transitions (MANUAL -> STABILIZED -> POSITION -> OFFBOARD)
- Understand valid/invalid mode transition rules
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_sim import (
    PX4SITL, FlightMode, FLIGHT_MODE_NAMES, VALID_MODE_TRANSITIONS,
    MAVROSInterface, spin
)


def setup_vehicle() -> PX4SITL:
    """
    Initialize a simulated PX4 vehicle.

    Returns:
        Configured PX4SITL instance

    TODO: Create a PX4SITL, verify initial state
    (disarmed, MANUAL mode, on ground).
    """
    # YOUR CODE HERE
    pass


def arm_vehicle(vehicle: PX4SITL) -> bool:
    """
    Arm the vehicle with proper checks.

    Args:
        vehicle: PX4SITL instance

    Returns:
        True if armed successfully

    TODO: Call vehicle.arm(), handle result, print status.
    """
    # YOUR CODE HERE
    pass


def change_mode(vehicle: PX4SITL, mode: FlightMode) -> bool:
    """
    Request a flight mode change.

    Args:
        vehicle: PX4SITL instance
        mode: Target FlightMode

    Returns:
        True if mode change succeeded

    TODO: Call vehicle.set_mode(), handle result, print status.
    """
    # YOUR CODE HERE
    pass


def test_mode_transitions(vehicle: PX4SITL) -> list:
    """
    Test valid and invalid mode transitions.

    Args:
        vehicle: PX4SITL instance

    Returns:
        List of (from_mode, to_mode, success) tuples

    TODO: Test transitions:
    - MANUAL -> STABILIZED (valid)
    - STABILIZED -> POSITION (valid)
    - POSITION -> OFFBOARD (needs setpoints)
    - MANUAL -> AUTO_MISSION (invalid from MANUAL)
    Print results for each attempt.
    """
    # YOUR CODE HERE
    pass


def get_vehicle_state(vehicle: PX4SITL) -> dict:
    """
    Read current vehicle state.

    Returns:
        Dict with armed, mode, position, battery, gps info

    TODO: Gather state from vehicle and return as dict.
    """
    # YOUR CODE HERE
    pass


def main():
    """Walk through mode transitions and print state at each step."""
    print("=" * 60)
    print("Task 2: Flight Mode Management")
    print("=" * 60)

    # TODO: 1. Setup vehicle
    # TODO: 2. Print initial state
    # TODO: 3. Test mode transitions (MANUAL -> STABILIZED -> POSITION)
    # TODO: 4. Arm vehicle
    # TODO: 5. Try OFFBOARD (should fail without setpoints)
    # TODO: 6. Print final state

    print("\nTask 2: Not yet implemented")


if __name__ == '__main__':
    main()
