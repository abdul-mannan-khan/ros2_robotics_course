#!/usr/bin/env python3
"""
Task 3: Offboard Position Control
====================================
Implement basic offboard control to fly waypoint missions.

Objectives:
- Understand PX4 offboard mode requirements (setpoint streaming)
- Send position and velocity setpoints
- Fly a square pattern and record trajectory
- Visualize 3D flight path
"""

import sys
import os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_sim import (
    PX4SITL, FlightMode, OffboardController, spin,
    wait_for_position, MAVROSInterface
)


def setup_offboard(vehicle: PX4SITL) -> OffboardController:
    """
    Prepare vehicle for offboard mode.

    Must stream setpoints before engaging offboard (PX4 safety requirement).

    Args:
        vehicle: PX4SITL instance

    Returns:
        OffboardController ready for use

    TODO: Create OffboardController, call start_setpoint_stream().
    """
    # YOUR CODE HERE
    pass


def send_position_setpoint(vehicle: PX4SITL, x: float, y: float, z: float,
                           yaw: float = 0.0):
    """
    Send a position setpoint in local NED frame.

    Args:
        vehicle: PX4SITL instance
        x, y, z: Position in NED (z negative = up)
        yaw: Heading in radians

    TODO: Call vehicle.send_position_setpoint().
    """
    # YOUR CODE HERE
    pass


def send_velocity_setpoint(vehicle: PX4SITL, vx: float, vy: float, vz: float,
                           yaw_rate: float = 0.0):
    """
    Send a velocity setpoint in local NED frame.

    TODO: Call vehicle.send_velocity_setpoint().
    """
    # YOUR CODE HERE
    pass


def fly_to_position(vehicle: PX4SITL, x: float, y: float, z: float,
                    tolerance: float = 0.5, timeout: float = 30.0) -> bool:
    """
    Fly to a position and wait until reached.

    TODO: Send setpoint and use wait_for_position().
    """
    # YOUR CODE HERE
    pass


def offboard_mission(vehicle: PX4SITL, waypoints: list) -> list:
    """
    Fly through a list of waypoints in offboard mode.

    Args:
        vehicle: PX4SITL instance (must be in offboard, armed, airborne)
        waypoints: List of (x, y, z) tuples

    Returns:
        List of recorded positions

    TODO: For each waypoint, send setpoint and wait for arrival.
    Record position at each sim step.
    """
    # YOUR CODE HERE
    pass


def main():
    """Takeoff, fly square, land, plot 3D trajectory."""
    print("=" * 60)
    print("Task 3: Offboard Position Control")
    print("=" * 60)

    # TODO: 1. Create vehicle and offboard controller
    # TODO: 2. Arm and takeoff to 10m
    # TODO: 3. Fly square pattern: (10,0,-10) -> (10,10,-10) -> (0,10,-10) -> (0,0,-10)
    # TODO: 4. Land
    # TODO: 5. Plot 3D trajectory and save figure

    print("\nTask 3: Not yet implemented")


if __name__ == '__main__':
    main()
