#!/usr/bin/env python3
"""
Task 1 Solution: MAVLink Protocol
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from px4_sim import (
    MAVLinkMessage, Heartbeat, CommandLong, CommandAck,
    MAVLinkConnection, MAVType, MAVAutopilot, MAVState,
    MAVCmd, MAVResult, MAVModeFlag
)


def create_heartbeat(system_id, autopilot_type, mode):
    """Create a MAVLink heartbeat message."""
    if system_id == 255:
        mav_type = MAVType.GCS
    else:
        mav_type = MAVType.QUADROTOR

    return Heartbeat(
        system_id=system_id,
        component_id=1,
        mav_type=mav_type,
        autopilot=autopilot_type,
        base_mode=mode,
        custom_mode=0,
        system_status=MAVState.ACTIVE,
    )


def create_command_long(target, command_id, params):
    """Create a MAVLink COMMAND_LONG message."""
    padded = list(params) + [0.0] * (7 - len(params))
    return CommandLong(
        target_system=target,
        target_component=1,
        command=command_id,
        param1=padded[0],
        param2=padded[1],
        param3=padded[2],
        param4=padded[3],
        param5=padded[4],
        param6=padded[5],
        param7=padded[6],
    )


def parse_message(raw_bytes):
    """Parse a serialized MAVLink message."""
    msg, remaining = MAVLinkMessage.from_bytes(raw_bytes)
    return {
        'system_id': msg.system_id,
        'component_id': msg.component_id,
        'msg_id': msg.msg_id,
        'timestamp': msg.timestamp,
        'remaining_bytes': len(remaining),
    }


def simulate_communication(drone_conn, gcs_conn):
    """Simulate MAVLink communication sequence."""
    exchange = []

    # 1. Drone heartbeat
    hb_drone = create_heartbeat(1, MAVAutopilot.PX4, MAVModeFlag.CUSTOM_MODE_ENABLED)
    drone_conn.send(hb_drone)
    exchange.append(("DRONE", "HEARTBEAT", hb_drone))

    # 2. GCS heartbeat
    hb_gcs = create_heartbeat(255, MAVAutopilot.GENERIC, 0)
    gcs_conn.send(hb_gcs)
    exchange.append(("GCS", "HEARTBEAT", hb_gcs))

    # 3. GCS sends arm command
    arm_cmd = create_command_long(1, MAVCmd.COMPONENT_ARM_DISARM, [1.0])
    gcs_conn.send(arm_cmd)
    exchange.append(("GCS", "CMD_ARM", arm_cmd))

    # 4. Drone acks arm
    arm_ack = CommandAck(command=MAVCmd.COMPONENT_ARM_DISARM, result=MAVResult.ACCEPTED)
    drone_conn.send(arm_ack)
    exchange.append(("DRONE", "CMD_ACK (ARM)", arm_ack))

    # 5. GCS sends takeoff
    takeoff_cmd = create_command_long(1, MAVCmd.NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, 10.0])
    gcs_conn.send(takeoff_cmd)
    exchange.append(("GCS", "CMD_TAKEOFF", takeoff_cmd))

    # 6. Drone acks takeoff
    takeoff_ack = CommandAck(command=MAVCmd.NAV_TAKEOFF, result=MAVResult.ACCEPTED)
    drone_conn.send(takeoff_ack)
    exchange.append(("DRONE", "CMD_ACK (TAKEOFF)", takeoff_ack))

    return exchange


def main():
    print("=" * 60)
    print("Task 1 Solution: MAVLink Protocol")
    print("=" * 60)

    # Create heartbeats
    print("\n--- Heartbeat Messages ---")
    drone_hb = create_heartbeat(1, MAVAutopilot.PX4, MAVModeFlag.CUSTOM_MODE_ENABLED)
    gcs_hb = create_heartbeat(255, MAVAutopilot.GENERIC, 0)
    print(f"Drone HB: sys_id={drone_hb.system_id}, type={drone_hb.mav_type}, "
          f"autopilot={drone_hb.autopilot}, status={drone_hb.system_status}")
    print(f"GCS HB:   sys_id={gcs_hb.system_id}, type={gcs_hb.mav_type}, "
          f"autopilot={gcs_hb.autopilot}")

    # Create commands
    print("\n--- Command Messages ---")
    arm_cmd = create_command_long(1, MAVCmd.COMPONENT_ARM_DISARM, [1.0])
    print(f"ARM cmd: target={arm_cmd.target_system}, cmd={arm_cmd.command}, "
          f"param1={arm_cmd.param1}")

    takeoff_cmd = create_command_long(1, MAVCmd.NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, 10.0])
    print(f"TAKEOFF cmd: target={takeoff_cmd.target_system}, cmd={takeoff_cmd.command}, "
          f"alt={takeoff_cmd.param7}m")

    # Serialize / parse
    print("\n--- Serialization ---")
    raw = drone_hb.to_bytes()
    print(f"Serialized heartbeat: {len(raw)} bytes")
    parsed = parse_message(raw)
    print(f"Parsed: {parsed}")

    # Communication simulation
    print("\n--- Communication Simulation ---")
    drone_conn = MAVLinkConnection(system_id=1, component_id=1)
    gcs_conn = MAVLinkConnection(system_id=255, component_id=1)
    drone_conn.connect(gcs_conn)

    exchange = simulate_communication(drone_conn, gcs_conn)
    for sender, desc, msg in exchange:
        print(f"  [{sender:5s}] -> {desc}")

    # Verify messages received
    print("\n--- Message Routing ---")
    drone_msgs = drone_conn.receive_all()
    gcs_msgs = gcs_conn.receive_all()
    print(f"Messages in drone inbox: {len(drone_msgs)}")
    print(f"Messages in GCS inbox:   {len(gcs_msgs)}")

    print("\nTask 1 complete.")


if __name__ == '__main__':
    main()
