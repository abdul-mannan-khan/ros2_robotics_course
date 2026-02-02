#!/usr/bin/env python3
"""
Task 1: MAVLink Protocol - Understanding MAVLink Messaging
===========================================================
Learn the MAVLink communication protocol used between PX4 and companion computers.

Objectives:
- Understand MAVLink message structure (header, payload)
- Create and parse heartbeat and command messages
- Simulate GCS <-> vehicle communication
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_sim import (
    MAVLinkMessage, Heartbeat, CommandLong, CommandAck,
    MAVLinkConnection, MAVType, MAVAutopilot, MAVState,
    MAVCmd, MAVResult, MAVModeFlag
)


def create_heartbeat(system_id: int, autopilot_type: int, mode: int) -> Heartbeat:
    """
    Create a MAVLink heartbeat message.

    Args:
        system_id: System ID (1 for vehicle, 255 for GCS)
        autopilot_type: MAVAutopilot enum value
        mode: Base mode flags

    Returns:
        Heartbeat message

    TODO: Create and return a Heartbeat with the given parameters.
    Set mav_type to QUADROTOR for vehicle (sys_id=1) or GCS (sys_id=255).
    Set system_status to ACTIVE.
    """
    # YOUR CODE HERE
    pass


def create_command_long(target: int, command_id: int, params: list) -> CommandLong:
    """
    Create a MAVLink COMMAND_LONG message.

    Args:
        target: Target system ID
        command_id: MAVCmd enum value
        params: List of up to 7 float parameters

    Returns:
        CommandLong message

    TODO: Create a CommandLong with the given command and parameters.
    Pad params with 0.0 if fewer than 7 are provided.
    """
    # YOUR CODE HERE
    pass


def parse_message(raw_bytes: bytes) -> dict:
    """
    Parse a serialized MAVLink message into a dictionary.

    Args:
        raw_bytes: Serialized message bytes

    Returns:
        Dictionary with message fields

    TODO: Use MAVLinkMessage.from_bytes() to extract header fields.
    Return dict with keys: system_id, component_id, msg_id, timestamp.
    """
    # YOUR CODE HERE
    pass


def simulate_communication(drone_conn: MAVLinkConnection,
                           gcs_conn: MAVLinkConnection) -> list:
    """
    Simulate a MAVLink communication sequence between drone and GCS.

    Sequence:
    1. Drone sends heartbeat
    2. GCS sends heartbeat
    3. GCS sends arm command
    4. Drone sends command ack
    5. GCS sends takeoff command
    6. Drone sends command ack

    Args:
        drone_conn: Drone's MAVLink connection
        gcs_conn: GCS's MAVLink connection

    Returns:
        List of (sender, message) tuples representing the exchange

    TODO: Implement the message exchange sequence.
    """
    # YOUR CODE HERE
    pass


def main():
    """Demonstrate MAVLink message exchange."""
    print("=" * 60)
    print("Task 1: MAVLink Protocol")
    print("=" * 60)

    # TODO: 1. Create heartbeats for drone and GCS
    # TODO: 2. Create arm and takeoff commands
    # TODO: 3. Serialize and parse messages
    # TODO: 4. Set up connections and simulate communication
    # TODO: 5. Print the message flow

    print("\nTask 1: Not yet implemented")


if __name__ == '__main__':
    main()
