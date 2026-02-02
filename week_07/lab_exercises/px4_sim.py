"""
PX4 Simulation Framework
=========================
Pure-Python simulation of PX4/MAVLink/ROS2 interfaces for standalone lab exercises.
No ROS2 or PX4 installation required.

This module provides:
- MAVLink message types and connection simulation
- PX4 SITL flight controller simulation with realistic dynamics
- MAVROS-like interface bridge
- Offboard control helper

Author: ROS2 Robotics Course - Week 7
"""

import time
import struct
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Callable, Tuple, Any
from enum import IntEnum
import threading
import copy


# =============================================================================
# MAVLink Enums
# =============================================================================

class MAVType(IntEnum):
    GENERIC = 0
    FIXED_WING = 1
    QUADROTOR = 2
    HEXAROTOR = 13
    OCTOROTOR = 14
    GCS = 6


class MAVAutopilot(IntEnum):
    GENERIC = 0
    PX4 = 12
    ARDUPILOT = 3


class MAVState(IntEnum):
    UNINIT = 0
    BOOT = 1
    CALIBRATING = 2
    STANDBY = 3
    ACTIVE = 4
    CRITICAL = 5
    EMERGENCY = 6


class MAVModeFlag(IntEnum):
    SAFETY_ARMED = 128
    MANUAL_INPUT_ENABLED = 64
    HIL_ENABLED = 32
    STABILIZE_ENABLED = 16
    GUIDED_ENABLED = 8
    AUTO_ENABLED = 4
    CUSTOM_MODE_ENABLED = 1


class MAVCmd(IntEnum):
    COMPONENT_ARM_DISARM = 400
    NAV_TAKEOFF = 22
    NAV_LAND = 21
    NAV_WAYPOINT = 16
    NAV_RETURN_TO_LAUNCH = 20
    DO_SET_MODE = 176
    NAV_LOITER_UNLIM = 17
    DO_REPOSITION = 192
    MISSION_START = 300


class MAVResult(IntEnum):
    ACCEPTED = 0
    TEMPORARILY_REJECTED = 1
    DENIED = 2
    UNSUPPORTED = 3
    FAILED = 4
    IN_PROGRESS = 5


class FlightMode(IntEnum):
    MANUAL = 0
    STABILIZED = 1
    ALTITUDE = 2
    POSITION = 3
    OFFBOARD = 4
    AUTO_MISSION = 5
    AUTO_LAND = 6
    AUTO_RTL = 7


FLIGHT_MODE_NAMES = {
    FlightMode.MANUAL: "MANUAL",
    FlightMode.STABILIZED: "STABILIZED",
    FlightMode.ALTITUDE: "ALTITUDE",
    FlightMode.POSITION: "POSITION",
    FlightMode.OFFBOARD: "OFFBOARD",
    FlightMode.AUTO_MISSION: "AUTO_MISSION",
    FlightMode.AUTO_LAND: "AUTO_LAND",
    FlightMode.AUTO_RTL: "AUTO_RTL",
}

# Valid mode transitions (from -> set of allowed to)
VALID_MODE_TRANSITIONS = {
    FlightMode.MANUAL: {FlightMode.STABILIZED, FlightMode.ALTITUDE, FlightMode.POSITION},
    FlightMode.STABILIZED: {FlightMode.MANUAL, FlightMode.ALTITUDE, FlightMode.POSITION},
    FlightMode.ALTITUDE: {FlightMode.MANUAL, FlightMode.STABILIZED, FlightMode.POSITION},
    FlightMode.POSITION: {FlightMode.MANUAL, FlightMode.STABILIZED, FlightMode.ALTITUDE,
                          FlightMode.OFFBOARD, FlightMode.AUTO_MISSION, FlightMode.AUTO_LAND,
                          FlightMode.AUTO_RTL},
    FlightMode.OFFBOARD: {FlightMode.POSITION, FlightMode.AUTO_LAND, FlightMode.AUTO_RTL,
                          FlightMode.MANUAL},
    FlightMode.AUTO_MISSION: {FlightMode.POSITION, FlightMode.AUTO_LAND, FlightMode.AUTO_RTL,
                              FlightMode.MANUAL, FlightMode.OFFBOARD},
    FlightMode.AUTO_LAND: {FlightMode.POSITION, FlightMode.MANUAL},
    FlightMode.AUTO_RTL: {FlightMode.POSITION, FlightMode.MANUAL, FlightMode.AUTO_LAND},
}


# =============================================================================
# MAVLink Messages
# =============================================================================

@dataclass
class MAVLinkMessage:
    """Base MAVLink message."""
    system_id: int = 1
    component_id: int = 1
    msg_id: int = 0
    timestamp: float = field(default_factory=time.time)

    def to_bytes(self) -> bytes:
        """Serialize to bytes (simplified)."""
        header = struct.pack('<BBHd', self.system_id, self.component_id,
                             self.msg_id, self.timestamp)
        return header + self._payload_bytes()

    def _payload_bytes(self) -> bytes:
        return b''

    @classmethod
    def from_bytes(cls, data: bytes):
        """Deserialize from bytes (simplified)."""
        sys_id, comp_id, msg_id, ts = struct.unpack('<BBHd', data[:12])
        msg = cls(system_id=sys_id, component_id=comp_id, msg_id=msg_id, timestamp=ts)
        return msg, data[12:]


@dataclass
class Heartbeat(MAVLinkMessage):
    """MAVLink Heartbeat message."""
    msg_id: int = 0
    mav_type: int = MAVType.QUADROTOR
    autopilot: int = MAVAutopilot.PX4
    base_mode: int = 0
    custom_mode: int = 0
    system_status: int = MAVState.STANDBY

    def _payload_bytes(self) -> bytes:
        return struct.pack('<BBBIB', self.mav_type, self.autopilot,
                           self.base_mode, self.custom_mode, self.system_status)


@dataclass
class CommandLong(MAVLinkMessage):
    """MAVLink Command Long message."""
    msg_id: int = 76
    target_system: int = 1
    target_component: int = 1
    command: int = 0
    confirmation: int = 0
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    param5: float = 0.0
    param6: float = 0.0
    param7: float = 0.0

    def _payload_bytes(self) -> bytes:
        return struct.pack('<BBHBfffffff', self.target_system, self.target_component,
                           self.command, self.confirmation,
                           self.param1, self.param2, self.param3, self.param4,
                           self.param5, self.param6, self.param7)


@dataclass
class CommandAck(MAVLinkMessage):
    """MAVLink Command Acknowledgment."""
    msg_id: int = 77
    command: int = 0
    result: int = MAVResult.ACCEPTED

    def _payload_bytes(self) -> bytes:
        return struct.pack('<HB', self.command, self.result)


@dataclass
class SetPositionTargetLocalNED(MAVLinkMessage):
    """Position/velocity setpoint in local NED frame."""
    msg_id: int = 84
    coordinate_frame: int = 1  # MAV_FRAME_LOCAL_NED
    type_mask: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0

    def _payload_bytes(self) -> bytes:
        return struct.pack('<BHfffffffffff', self.coordinate_frame, self.type_mask,
                           self.x, self.y, self.z, self.vx, self.vy, self.vz,
                           0, 0, 0, self.yaw, self.yaw_rate)


@dataclass
class VehicleAttitude(MAVLinkMessage):
    """Vehicle attitude (quaternion)."""
    msg_id: int = 30
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    rollspeed: float = 0.0
    pitchspeed: float = 0.0
    yawspeed: float = 0.0
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))


@dataclass
class VehicleLocalPosition(MAVLinkMessage):
    """Vehicle local position NED."""
    msg_id: int = 32
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


@dataclass
class VehicleStatus(MAVLinkMessage):
    """Vehicle status."""
    msg_id: int = 1
    armed: bool = False
    flight_mode: int = FlightMode.MANUAL
    system_status: int = MAVState.STANDBY
    landed: bool = True
    in_air: bool = False


@dataclass
class BatteryStatus(MAVLinkMessage):
    """Battery status."""
    msg_id: int = 147
    voltage: float = 16.8  # 4S LiPo full
    current: float = 0.0
    remaining: float = 100.0  # percentage
    temperature: float = 25.0


@dataclass
class GPSRawInt(MAVLinkMessage):
    """GPS raw data."""
    msg_id: int = 24
    fix_type: int = 3  # 3D fix
    lat: int = 473977418  # 47.3977418 degrees * 1e7
    lon: int = 85455939   # 8.5455939 degrees * 1e7
    alt: int = 488000     # 488m * 1000
    satellites_visible: int = 12
    hdop: int = 80  # 0.8 * 100
    vdop: int = 120  # 1.2 * 100


@dataclass
class MissionItem:
    """Mission item / waypoint."""
    seq: int = 0
    command: int = MAVCmd.NAV_WAYPOINT
    x: float = 0.0  # lat or local x
    y: float = 0.0  # lon or local y
    z: float = 0.0  # alt
    param1: float = 0.0  # hold time
    param2: float = 0.0  # acceptance radius
    param3: float = 0.0  # pass-through radius
    param4: float = 0.0  # yaw
    autocontinue: bool = True
    frame: int = 1  # LOCAL_NED


# =============================================================================
# MAVLink Connection
# =============================================================================

class MAVLinkConnection:
    """Simulated MAVLink connection for message passing."""

    def __init__(self, system_id: int = 1, component_id: int = 1):
        self.system_id = system_id
        self.component_id = component_id
        self._inbox: List[MAVLinkMessage] = []
        self._outbox: List[MAVLinkMessage] = []
        self._handlers: Dict[int, List[Callable]] = {}
        self._connected_to: Optional['MAVLinkConnection'] = None

    def connect(self, other: 'MAVLinkConnection'):
        """Connect two MAVLink endpoints."""
        self._connected_to = other
        other._connected_to = self

    def send(self, msg: MAVLinkMessage):
        """Send a message."""
        msg.system_id = self.system_id
        msg.component_id = self.component_id
        msg.timestamp = time.time()
        self._outbox.append(msg)
        if self._connected_to is not None:
            self._connected_to._inbox.append(copy.deepcopy(msg))

    def receive(self) -> Optional[MAVLinkMessage]:
        """Receive next message from inbox."""
        if self._inbox:
            return self._inbox.pop(0)
        return None

    def receive_all(self) -> List[MAVLinkMessage]:
        """Receive all pending messages."""
        msgs = list(self._inbox)
        self._inbox.clear()
        return msgs

    def register_handler(self, msg_id: int, handler: Callable):
        """Register a handler for a message type."""
        if msg_id not in self._handlers:
            self._handlers[msg_id] = []
        self._handlers[msg_id].append(handler)

    def process_messages(self):
        """Process inbox with registered handlers."""
        for msg in self.receive_all():
            if msg.msg_id in self._handlers:
                for handler in self._handlers[msg.msg_id]:
                    handler(msg)


# =============================================================================
# PX4 SITL Flight Controller
# =============================================================================

class PX4SITL:
    """
    Simulated PX4 flight controller with realistic behavior.

    Features:
    - Position/velocity control with first-order dynamics
    - Flight mode management with transition validation
    - Arming/disarming logic
    - Battery simulation
    - GPS simulation
    - Failsafe logic (battery, GPS loss, comms loss)
    - Mission execution
    """

    def __init__(self, dt: float = 0.02):
        self.dt = dt
        self.sim_time = 0.0

        # State
        self.position = np.array([0.0, 0.0, 0.0])  # NED (z negative = up)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        self.attitude = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw (rad)
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        # Control
        self.position_setpoint = np.array([0.0, 0.0, 0.0])
        self.velocity_setpoint = np.array([0.0, 0.0, 0.0])
        self.yaw_setpoint = 0.0
        self.yaw_rate_setpoint = 0.0
        self.setpoint_type = 'position'  # 'position' or 'velocity'

        # Flight state
        self.armed = False
        self.flight_mode = FlightMode.MANUAL
        self.system_status = MAVState.STANDBY
        self.landed = True
        self.in_air = False

        # Battery
        self.battery_voltage = 16.8
        self.battery_remaining = 100.0
        self.battery_current = 0.0
        self._battery_drain_rate = 0.0  # %/s, set externally

        # GPS
        self.gps_fix = 3
        self.gps_satellites = 12
        self.gps_hdop = 0.8
        self._gps_available = True

        # Communication
        self.connection = MAVLinkConnection(system_id=1, component_id=1)
        self._comms_available = True

        # Offboard
        self._offboard_setpoint_count = 0
        self._offboard_setpoint_timeout = 0.0
        self._offboard_active = False
        self._OFFBOARD_SETPOINT_MIN = 10  # Min setpoints before arming in offboard
        self._OFFBOARD_TIMEOUT = 0.5  # seconds without setpoint

        # Mission
        self.mission_items: List[MissionItem] = []
        self.mission_current = 0
        self.mission_running = False

        # Failsafe config
        self.failsafe_battery_warn = 30.0  # % -> RTL
        self.failsafe_battery_critical = 15.0  # % -> LAND
        self.failsafe_gps_timeout = 3.0  # seconds -> POSITION HOLD / LAND
        self.failsafe_comms_timeout = 5.0  # seconds -> RTL
        self._gps_loss_start = None
        self._comms_loss_start = None

        # Dynamics parameters
        self._pos_tau = 0.5  # position time constant (s)
        self._vel_tau = 0.3  # velocity time constant (s)
        self._yaw_tau = 0.4  # yaw time constant (s)
        self._max_velocity = 5.0  # m/s
        self._max_acceleration = 3.0  # m/s^2
        self._takeoff_speed = 1.5  # m/s
        self._land_speed = 0.7  # m/s

        # Home position
        self.home_position = np.array([0.0, 0.0, 0.0])

        # Logging
        self.log: List[Dict] = []
        self._log_enabled = True

        # Callbacks
        self._state_callbacks: List[Callable] = []
        self._position_callbacks: List[Callable] = []

        # Telemetry publish rate
        self._publish_rate = 50  # Hz
        self._publish_counter = 0

        # Failsafe triggered flags
        self.failsafe_triggered = False
        self.failsafe_reason = ""

    def register_state_callback(self, cb: Callable):
        self._state_callbacks.append(cb)

    def register_position_callback(self, cb: Callable):
        self._position_callbacks.append(cb)

    def arm(self) -> Tuple[bool, str]:
        """Attempt to arm the vehicle."""
        if self.armed:
            return True, "Already armed"

        # Pre-arm checks
        if self.battery_remaining < 20:
            return False, "Battery too low for arming"
        if not self._gps_available and self.flight_mode != FlightMode.MANUAL:
            return False, "No GPS fix"

        # Offboard requires streaming setpoints
        if self.flight_mode == FlightMode.OFFBOARD:
            if self._offboard_setpoint_count < self._OFFBOARD_SETPOINT_MIN:
                return False, (f"Offboard: need {self._OFFBOARD_SETPOINT_MIN} setpoints "
                               f"before arming (got {self._offboard_setpoint_count})")

        self.armed = True
        self.system_status = MAVState.ACTIVE
        self.home_position = self.position.copy()
        return True, "Armed successfully"

    def disarm(self) -> Tuple[bool, str]:
        """Attempt to disarm."""
        if not self.armed:
            return True, "Already disarmed"
        if self.in_air:
            return False, "Cannot disarm while in air"
        self.armed = False
        self.system_status = MAVState.STANDBY
        return True, "Disarmed"

    def set_mode(self, mode: int) -> Tuple[bool, str]:
        """Change flight mode."""
        mode = FlightMode(mode)
        if mode == self.flight_mode:
            return True, f"Already in {FLIGHT_MODE_NAMES[mode]}"

        # Check valid transition
        if mode not in VALID_MODE_TRANSITIONS.get(self.flight_mode, set()):
            return False, (f"Invalid transition: {FLIGHT_MODE_NAMES[self.flight_mode]} -> "
                           f"{FLIGHT_MODE_NAMES[mode]}")

        # Offboard requires setpoints
        if mode == FlightMode.OFFBOARD:
            if self._offboard_setpoint_count < self._OFFBOARD_SETPOINT_MIN:
                return False, (f"Need {self._OFFBOARD_SETPOINT_MIN} setpoints for offboard "
                               f"(got {self._offboard_setpoint_count})")

        old_mode = self.flight_mode
        self.flight_mode = mode

        # Mode-specific initialization
        if mode == FlightMode.OFFBOARD:
            self._offboard_active = True
            self._offboard_setpoint_timeout = self.sim_time
        elif mode == FlightMode.AUTO_LAND:
            self.velocity_setpoint = np.array([0.0, 0.0, self._land_speed])
            self.setpoint_type = 'velocity'
        elif mode == FlightMode.AUTO_RTL:
            target = self.home_position.copy()
            target[2] = min(self.position[2], -10.0)  # RTL altitude
            self.position_setpoint = target
            self.setpoint_type = 'position'
        elif mode == FlightMode.POSITION:
            self.position_setpoint = self.position.copy()
            self.setpoint_type = 'position'

        return True, f"Mode changed: {FLIGHT_MODE_NAMES[old_mode]} -> {FLIGHT_MODE_NAMES[mode]}"

    def send_position_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Send position setpoint (NED frame, z negative = up)."""
        self.position_setpoint = np.array([x, y, z])
        self.yaw_setpoint = yaw
        self.setpoint_type = 'position'
        self._offboard_setpoint_count += 1
        self._offboard_setpoint_timeout = self.sim_time

    def send_velocity_setpoint(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Send velocity setpoint (NED frame)."""
        self.velocity_setpoint = np.array([vx, vy, vz])
        self.yaw_rate_setpoint = yaw_rate
        self.setpoint_type = 'velocity'
        self._offboard_setpoint_count += 1
        self._offboard_setpoint_timeout = self.sim_time

    def upload_mission(self, items: List[MissionItem]) -> bool:
        """Upload mission items."""
        self.mission_items = list(items)
        self.mission_current = 0
        return True

    def start_mission(self) -> Tuple[bool, str]:
        """Start mission execution."""
        if not self.mission_items:
            return False, "No mission uploaded"
        if not self.armed:
            return False, "Vehicle not armed"
        self.mission_running = True
        self.mission_current = 0
        ok, msg = self.set_mode(FlightMode.AUTO_MISSION)
        if not ok:
            # Force it for mission start
            self.flight_mode = FlightMode.AUTO_MISSION
        return True, "Mission started"

    def process_command(self, cmd: CommandLong) -> CommandAck:
        """Process an incoming MAVLink command."""
        ack = CommandAck(command=cmd.command)

        if cmd.command == MAVCmd.COMPONENT_ARM_DISARM:
            if cmd.param1 == 1.0:
                ok, msg = self.arm()
            else:
                ok, msg = self.disarm()
            ack.result = MAVResult.ACCEPTED if ok else MAVResult.DENIED

        elif cmd.command == MAVCmd.DO_SET_MODE:
            mode = int(cmd.param2)
            ok, msg = self.set_mode(mode)
            ack.result = MAVResult.ACCEPTED if ok else MAVResult.DENIED

        elif cmd.command == MAVCmd.NAV_TAKEOFF:
            alt = cmd.param7 if cmd.param7 != 0 else 10.0
            if not self.armed:
                ack.result = MAVResult.DENIED
            else:
                self.position_setpoint = np.array([self.position[0], self.position[1], -abs(alt)])
                self.setpoint_type = 'position'
                self.landed = False
                self.in_air = True
                ack.result = MAVResult.ACCEPTED

        elif cmd.command == MAVCmd.NAV_LAND:
            self.set_mode(FlightMode.AUTO_LAND)
            ack.result = MAVResult.ACCEPTED

        elif cmd.command == MAVCmd.NAV_RETURN_TO_LAUNCH:
            self.set_mode(FlightMode.AUTO_RTL)
            ack.result = MAVResult.ACCEPTED

        else:
            ack.result = MAVResult.UNSUPPORTED

        return ack

    def step(self, dt: float = None):
        """Advance simulation by one time step."""
        if dt is None:
            dt = self.dt
        self.sim_time += dt

        # Battery drain
        if self.armed:
            base_drain = 0.02  # %/s idle
            if self.in_air:
                speed = np.linalg.norm(self.velocity)
                base_drain = 0.05 + 0.02 * speed  # more drain at speed
            total_drain = base_drain + self._battery_drain_rate
            self.battery_remaining = max(0.0, self.battery_remaining - total_drain * dt)
            self.battery_voltage = 12.0 + 4.8 * (self.battery_remaining / 100.0)
            self.battery_current = base_drain * 10.0

        # Check failsafes
        self._check_failsafes(dt)

        if not self.armed:
            self._log_state()
            return

        # Mode-specific control
        if self.flight_mode == FlightMode.OFFBOARD:
            self._update_offboard(dt)
        elif self.flight_mode == FlightMode.AUTO_MISSION:
            self._update_mission(dt)
        elif self.flight_mode == FlightMode.AUTO_LAND:
            self._update_landing(dt)
        elif self.flight_mode == FlightMode.AUTO_RTL:
            self._update_rtl(dt)
        elif self.flight_mode == FlightMode.POSITION:
            pass  # hold position setpoint

        # Update dynamics
        self._update_dynamics(dt)

        # Airborne detection: if altitude > 0.5m, mark as in_air
        if self.armed and self.position[2] < -0.5:
            if not self.in_air:
                self.in_air = True
                self.landed = False

        # Landing detection
        if self.in_air and self.position[2] >= -0.15:
            self.position[2] = 0.0
            self.velocity = np.zeros(3)
            self.landed = True
            self.in_air = False
            if self.flight_mode in (FlightMode.AUTO_LAND, FlightMode.AUTO_RTL):
                self.flight_mode = FlightMode.POSITION
                self.disarm()

        self._log_state()

    def _update_dynamics(self, dt: float):
        """First-order dynamics towards setpoints."""
        if self.setpoint_type == 'position':
            # Position control: compute desired velocity
            pos_error = self.position_setpoint - self.position
            desired_vel = pos_error / self._pos_tau
            # Clamp velocity
            speed = np.linalg.norm(desired_vel)
            if speed > self._max_velocity:
                desired_vel = desired_vel / speed * self._max_velocity
            # First-order velocity response
            vel_error = desired_vel - self.velocity
            self.acceleration = vel_error / self._vel_tau
            # Clamp acceleration
            acc_mag = np.linalg.norm(self.acceleration)
            if acc_mag > self._max_acceleration:
                self.acceleration = self.acceleration / acc_mag * self._max_acceleration
        else:
            # Velocity control
            vel_error = self.velocity_setpoint - self.velocity
            self.acceleration = vel_error / self._vel_tau
            acc_mag = np.linalg.norm(self.acceleration)
            if acc_mag > self._max_acceleration:
                self.acceleration = self.acceleration / acc_mag * self._max_acceleration

        # Integrate
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt

        # Yaw control
        if self.setpoint_type == 'position':
            yaw_error = self._wrap_angle(self.yaw_setpoint - self.attitude[2])
            yaw_rate = yaw_error / self._yaw_tau
        else:
            yaw_rate = self.yaw_rate_setpoint
        self.attitude[2] += yaw_rate * dt
        self.attitude[2] = self._wrap_angle(self.attitude[2])

        # Tilt from acceleration (simplified)
        if self.in_air:
            self.attitude[1] = np.clip(self.velocity[0] * 0.05, -0.3, 0.3)  # pitch from vx
            self.attitude[0] = np.clip(-self.velocity[1] * 0.05, -0.3, 0.3)  # roll from vy
        else:
            self.attitude[0] = 0.0
            self.attitude[1] = 0.0

        # Ground constraint
        if self.position[2] > 0:
            self.position[2] = 0.0
            if self.velocity[2] > 0:
                self.velocity[2] = 0.0

    def _update_offboard(self, dt: float):
        """Offboard mode: check setpoint stream."""
        time_since_sp = self.sim_time - self._offboard_setpoint_timeout
        if time_since_sp > self._OFFBOARD_TIMEOUT:
            # Lost offboard setpoint stream -> hold position
            self.position_setpoint = self.position.copy()
            self.setpoint_type = 'position'

    def _update_mission(self, dt: float):
        """Mission mode: follow waypoints."""
        if not self.mission_running or not self.mission_items:
            return

        if self.mission_current >= len(self.mission_items):
            self.mission_running = False
            self.set_mode(FlightMode.AUTO_LAND)
            return

        item = self.mission_items[self.mission_current]

        if item.command == MAVCmd.NAV_WAYPOINT:
            target = np.array([item.x, item.y, item.z])
            self.position_setpoint = target
            self.yaw_setpoint = item.param4
            self.setpoint_type = 'position'

            dist = np.linalg.norm(self.position - target)
            acceptance = item.param2 if item.param2 > 0 else 1.0
            if dist < acceptance:
                self.mission_current += 1

        elif item.command == MAVCmd.NAV_TAKEOFF:
            alt = item.z
            target = np.array([self.position[0], self.position[1], alt])
            self.position_setpoint = target
            self.setpoint_type = 'position'
            if abs(self.position[2] - alt) < 0.5:
                self.in_air = True
                self.landed = False
                self.mission_current += 1

        elif item.command == MAVCmd.NAV_LAND:
            self.set_mode(FlightMode.AUTO_LAND)
            self.mission_running = False

        elif item.command == MAVCmd.NAV_RETURN_TO_LAUNCH:
            self.set_mode(FlightMode.AUTO_RTL)
            self.mission_running = False

    def _update_landing(self, dt: float):
        """Auto-land mode."""
        self.velocity_setpoint = np.array([0.0, 0.0, self._land_speed])
        self.setpoint_type = 'velocity'

    def _update_rtl(self, dt: float):
        """Return to launch."""
        dist_to_home_horiz = np.linalg.norm(self.position[:2] - self.home_position[:2])
        if dist_to_home_horiz > 1.0:
            # Fly to home at RTL altitude
            target = self.home_position.copy()
            target[2] = min(self.position[2], -10.0)
            self.position_setpoint = target
            self.setpoint_type = 'position'
        else:
            # Above home, descend
            self.set_mode(FlightMode.AUTO_LAND)

    def _check_failsafes(self, dt: float):
        """Check and trigger failsafes."""
        # Battery failsafe
        if self.armed and self.battery_remaining <= self.failsafe_battery_critical:
            if self.in_air and self.flight_mode != FlightMode.AUTO_LAND:
                self.flight_mode = FlightMode.AUTO_LAND
                self.failsafe_triggered = True
                self.failsafe_reason = "CRITICAL_BATTERY"
        elif self.armed and self.battery_remaining <= self.failsafe_battery_warn:
            if self.in_air and self.flight_mode not in (FlightMode.AUTO_RTL, FlightMode.AUTO_LAND):
                self.flight_mode = FlightMode.AUTO_RTL
                self.failsafe_triggered = True
                self.failsafe_reason = "LOW_BATTERY"

        # GPS failsafe
        if not self._gps_available:
            if self._gps_loss_start is None:
                self._gps_loss_start = self.sim_time
            elif (self.sim_time - self._gps_loss_start) > self.failsafe_gps_timeout:
                if self.armed and self.in_air:
                    if self.flight_mode not in (FlightMode.AUTO_LAND, FlightMode.MANUAL):
                        # Hold position (or land if prolonged)
                        self.position_setpoint = self.position.copy()
                        self.setpoint_type = 'position'
                        self.flight_mode = FlightMode.AUTO_LAND
                        self.failsafe_triggered = True
                        self.failsafe_reason = "GPS_LOSS"
        else:
            self._gps_loss_start = None

        # Comms failsafe
        if not self._comms_available:
            if self._comms_loss_start is None:
                self._comms_loss_start = self.sim_time
            elif (self.sim_time - self._comms_loss_start) > self.failsafe_comms_timeout:
                if self.armed and self.in_air:
                    if self.flight_mode not in (FlightMode.AUTO_RTL, FlightMode.AUTO_LAND):
                        self.flight_mode = FlightMode.AUTO_RTL
                        self.failsafe_triggered = True
                        self.failsafe_reason = "COMMS_LOSS"
        else:
            self._comms_loss_start = None

    def set_gps_available(self, available: bool):
        self._gps_available = available
        if available:
            self.gps_fix = 3
            self.gps_satellites = 12
        else:
            self.gps_fix = 0
            self.gps_satellites = 0

    def set_comms_available(self, available: bool):
        self._comms_available = available

    def set_battery_drain_rate(self, rate: float):
        """Set additional battery drain rate (%/s)."""
        self._battery_drain_rate = rate

    def get_vehicle_status(self) -> VehicleStatus:
        return VehicleStatus(
            armed=self.armed,
            flight_mode=self.flight_mode,
            system_status=self.system_status,
            landed=self.landed,
            in_air=self.in_air,
        )

    def get_local_position(self) -> VehicleLocalPosition:
        return VehicleLocalPosition(
            x=self.position[0], y=self.position[1], z=self.position[2],
            vx=self.velocity[0], vy=self.velocity[1], vz=self.velocity[2],
        )

    def get_attitude(self) -> VehicleAttitude:
        r, p, y = self.attitude
        # Euler to quaternion (simplified)
        cr, sr = np.cos(r/2), np.sin(r/2)
        cp, sp = np.cos(p/2), np.sin(p/2)
        cy, sy = np.cos(y/2), np.sin(y/2)
        q = np.array([
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
        ])
        return VehicleAttitude(
            roll=r, pitch=p, yaw=y,
            q=q,
        )

    def get_battery_status(self) -> BatteryStatus:
        return BatteryStatus(
            voltage=self.battery_voltage,
            current=self.battery_current,
            remaining=self.battery_remaining,
        )

    def get_gps(self) -> GPSRawInt:
        return GPSRawInt(
            fix_type=self.gps_fix,
            satellites_visible=self.gps_satellites,
        )

    def _log_state(self):
        if self._log_enabled:
            self.log.append({
                'time': self.sim_time,
                'position': self.position.copy(),
                'velocity': self.velocity.copy(),
                'attitude': self.attitude.copy(),
                'mode': int(self.flight_mode),
                'armed': self.armed,
                'in_air': self.in_air,
                'battery': self.battery_remaining,
                'battery_voltage': self.battery_voltage,
                'gps_fix': self.gps_fix,
                'failsafe': self.failsafe_triggered,
                'failsafe_reason': self.failsafe_reason,
                'setpoint': self.position_setpoint.copy(),
            })

    @staticmethod
    def _wrap_angle(a: float) -> float:
        return (a + np.pi) % (2 * np.pi) - np.pi


# =============================================================================
# MAVROS Interface (Simulated)
# =============================================================================

class MAVROSInterface:
    """
    Simulated MAVROS-like ROS2 bridge.
    Provides topic-like and service-like interfaces to PX4SITL.
    """

    def __init__(self, vehicle: PX4SITL):
        self.vehicle = vehicle
        self._subscribers: Dict[str, List[Callable]] = {}

        # "Topic" latest values
        self.topics = {
            '/mavros/state': None,
            '/mavros/local_position/pose': None,
            '/mavros/setpoint_position/local': None,
            '/mavros/imu/data': None,
            '/mavros/battery': None,
            '/mavros/global_position/raw/fix': None,
        }

    def subscribe(self, topic: str, callback: Callable):
        if topic not in self._subscribers:
            self._subscribers[topic] = []
        self._subscribers[topic].append(callback)

    def publish(self, topic: str, data: Any):
        self.topics[topic] = data
        for cb in self._subscribers.get(topic, []):
            cb(data)

    def call_service(self, service: str, **kwargs) -> Dict:
        """Call a simulated service."""
        if service == '/mavros/cmd/arming':
            value = kwargs.get('value', True)
            if value:
                ok, msg = self.vehicle.arm()
            else:
                ok, msg = self.vehicle.disarm()
            return {'success': ok, 'message': msg}

        elif service == '/mavros/set_mode':
            mode = kwargs.get('custom_mode', 'POSITION')
            mode_map = {v: k for k, v in FLIGHT_MODE_NAMES.items()}
            if mode in mode_map:
                ok, msg = self.vehicle.set_mode(mode_map[mode])
                return {'success': ok, 'message': msg}
            return {'success': False, 'message': f'Unknown mode: {mode}'}

        elif service == '/mavros/cmd/takeoff':
            alt = kwargs.get('altitude', 10.0)
            cmd = CommandLong(command=MAVCmd.NAV_TAKEOFF, param7=alt)
            ack = self.vehicle.process_command(cmd)
            return {'success': ack.result == MAVResult.ACCEPTED}

        elif service == '/mavros/cmd/land':
            cmd = CommandLong(command=MAVCmd.NAV_LAND)
            ack = self.vehicle.process_command(cmd)
            return {'success': ack.result == MAVResult.ACCEPTED}

        return {'success': False, 'message': 'Unknown service'}

    def update(self):
        """Update topic values from vehicle state."""
        self.topics['/mavros/state'] = self.vehicle.get_vehicle_status()
        self.topics['/mavros/local_position/pose'] = self.vehicle.get_local_position()
        self.topics['/mavros/imu/data'] = self.vehicle.get_attitude()
        self.topics['/mavros/battery'] = self.vehicle.get_battery_status()
        self.topics['/mavros/global_position/raw/fix'] = self.vehicle.get_gps()

        # Notify subscribers
        for topic, data in self.topics.items():
            if data is not None:
                for cb in self._subscribers.get(topic, []):
                    cb(data)


# =============================================================================
# Offboard Controller Helper
# =============================================================================

class OffboardController:
    """Helper for offboard mode management following PX4 safety requirements."""

    def __init__(self, vehicle: PX4SITL):
        self.vehicle = vehicle
        self._streaming = False
        self._stream_count = 0

    def start_setpoint_stream(self, position: np.ndarray = None, rate: int = 20,
                              duration: float = 1.0):
        """
        Stream setpoints to satisfy PX4 offboard requirements.
        Must stream setpoints before switching to offboard mode.
        """
        if position is None:
            position = self.vehicle.position.copy()

        steps = int(duration * rate)
        dt = 1.0 / rate
        for _ in range(steps):
            self.vehicle.send_position_setpoint(position[0], position[1], position[2])
            self.vehicle.step(dt)
            self._stream_count += 1
        self._streaming = True

    def is_ready(self) -> bool:
        return self._stream_count >= self.vehicle._OFFBOARD_SETPOINT_MIN

    def engage_offboard(self) -> Tuple[bool, str]:
        """Switch to offboard mode (must have streamed setpoints first).
        Automatically transitions through intermediate modes if needed."""
        if not self.is_ready():
            return False, "Not enough setpoints streamed"
        # Ensure we can reach OFFBOARD (need to be in POSITION or similar)
        if self.vehicle.flight_mode == FlightMode.MANUAL:
            self.vehicle.set_mode(FlightMode.STABILIZED)
        if self.vehicle.flight_mode in (FlightMode.STABILIZED, FlightMode.ALTITUDE):
            self.vehicle.set_mode(FlightMode.POSITION)
        return self.vehicle.set_mode(FlightMode.OFFBOARD)

    def arm_and_takeoff(self, altitude: float = 10.0) -> Tuple[bool, str]:
        """
        Full offboard takeoff sequence:
        1. Stream setpoints
        2. Switch to offboard
        3. Arm
        4. Command takeoff
        """
        # Stream setpoints at current position
        takeoff_pos = self.vehicle.position.copy()
        takeoff_pos[2] = -abs(altitude)

        self.start_setpoint_stream(position=takeoff_pos)

        ok, msg = self.engage_offboard()
        if not ok:
            return False, f"Offboard engage failed: {msg}"

        ok, msg = self.vehicle.arm()
        if not ok:
            return False, f"Arm failed: {msg}"

        # Command takeoff
        self.vehicle.send_position_setpoint(takeoff_pos[0], takeoff_pos[1], takeoff_pos[2])
        self.vehicle.landed = False
        self.vehicle.in_air = True

        return True, "Takeoff initiated"


# =============================================================================
# Utility Functions
# =============================================================================

def spin(vehicle: PX4SITL, duration: float, mavros: MAVROSInterface = None):
    """Run simulation for a duration."""
    steps = int(duration / vehicle.dt)
    for _ in range(steps):
        vehicle.step()
        if mavros:
            mavros.update()


def spin_once(vehicle: PX4SITL, mavros: MAVROSInterface = None):
    """Single simulation step."""
    vehicle.step()
    if mavros:
        mavros.update()


def wait_for_position(vehicle: PX4SITL, target: np.ndarray, tolerance: float = 0.5,
                      timeout: float = 30.0, mavros: MAVROSInterface = None) -> bool:
    """Block until vehicle reaches target position or timeout.
    Continuously re-sends the position setpoint to maintain offboard stream."""
    start = vehicle.sim_time
    while vehicle.sim_time - start < timeout:
        dist = np.linalg.norm(vehicle.position - target)
        if dist < tolerance:
            return True
        # Keep sending setpoint to maintain offboard stream
        vehicle.send_position_setpoint(target[0], target[1], target[2])
        vehicle.step()
        if mavros:
            mavros.update()
    return False


def create_simple_environment(obstacles: List[Dict] = None) -> Dict:
    """Create a simple 3D environment with obstacles."""
    if obstacles is None:
        obstacles = [
            {'center': np.array([10.0, 5.0, -5.0]), 'radius': 2.0},
            {'center': np.array([15.0, -3.0, -8.0]), 'radius': 1.5},
            {'center': np.array([5.0, 10.0, -6.0]), 'radius': 3.0},
            {'center': np.array([20.0, 0.0, -5.0]), 'radius': 2.5},
        ]
    return {
        'obstacles': obstacles,
        'bounds': {'x': (-50, 50), 'y': (-50, 50), 'z': (-50, 0)},
    }


def check_obstacle_distance(position: np.ndarray, environment: Dict) -> Tuple[float, Optional[np.ndarray]]:
    """Check distance to nearest obstacle. Returns (distance, direction_to_obstacle)."""
    min_dist = float('inf')
    closest_dir = None
    for obs in environment.get('obstacles', []):
        diff = obs['center'] - position
        dist = np.linalg.norm(diff) - obs['radius']
        if dist < min_dist:
            min_dist = dist
            closest_dir = diff / np.linalg.norm(diff) if np.linalg.norm(diff) > 0 else None
    return min_dist, closest_dir
