#!/usr/bin/env python3
"""
ros2_sim.py - Lightweight ROS2 Simulation Framework
=====================================================

A pure-Python simulation of core ROS2 concepts, allowing students to learn
ROS2 node architecture, pub/sub, services, parameters, and launch patterns
without requiring a full ROS2 installation.

Simulated ROS2 features:
    - Node lifecycle (create, spin, destroy)
    - Publishers and Subscriptions with topic-based routing
    - Timers with configurable periods
    - Services and Clients (request/response)
    - Parameters with validation callbacks
    - QoS profiles (reliability, durability, history depth)
    - Time, Duration, Rate
    - Common message types (String, Float64, Int32, Twist, Pose, LaserScan, Odometry)
    - Executor with spin / spin_once
    - Logging (get_logger)

Usage:
    import ros2_sim as rclpy
    rclpy.init()
    node = ros2_sim.Node('my_node')
    ...
    rclpy.spin(node)
    rclpy.shutdown()
"""

import time
import threading
import copy
import math
import logging
import sys
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Global message bus (topic -> list of subscription callbacks)
# ---------------------------------------------------------------------------
_topic_callbacks: Dict[str, List[Callable]] = {}
_service_handlers: Dict[str, Callable] = {}
_nodes: List["Node"] = []
_initialized = False
_shutdown_flag = False

# ---------------------------------------------------------------------------
# Initialization / Shutdown
# ---------------------------------------------------------------------------

def init(args=None, context=None):
    """Initialize the ROS2 simulation runtime."""
    global _initialized, _shutdown_flag
    _initialized = True
    _shutdown_flag = False

def shutdown(context=None):
    """Shutdown the ROS2 simulation runtime."""
    global _shutdown_flag
    _shutdown_flag = True

def ok():
    """Return True while the simulation is running."""
    return _initialized and not _shutdown_flag

# ---------------------------------------------------------------------------
# QoS Profile
# ---------------------------------------------------------------------------

class QoSReliabilityPolicy:
    RELIABLE = "RELIABLE"
    BEST_EFFORT = "BEST_EFFORT"

class QoSDurabilityPolicy:
    TRANSIENT_LOCAL = "TRANSIENT_LOCAL"
    VOLATILE = "VOLATILE"

class QoSHistoryPolicy:
    KEEP_LAST = "KEEP_LAST"
    KEEP_ALL = "KEEP_ALL"

@dataclass
class QoSProfile:
    reliability: str = QoSReliabilityPolicy.RELIABLE
    durability: str = QoSDurabilityPolicy.VOLATILE
    history: str = QoSHistoryPolicy.KEEP_LAST
    depth: int = 10

    def __repr__(self):
        return (f"QoSProfile(reliability={self.reliability}, "
                f"durability={self.durability}, history={self.history}, "
                f"depth={self.depth})")

# Predefined profiles
QOS_RELIABLE = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE)
QOS_BEST_EFFORT = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT)
QOS_SENSOR_DATA = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=5,
)
QOS_PARAMETERS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

# ---------------------------------------------------------------------------
# Message Types
# ---------------------------------------------------------------------------

@dataclass
class String:
    data: str = ""

@dataclass
class Float64:
    data: float = 0.0

@dataclass
class Int32:
    data: int = 0

@dataclass
class Bool:
    data: bool = False

@dataclass
class Header:
    stamp_sec: int = 0
    stamp_nanosec: int = 0
    frame_id: str = ""

@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Twist:
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)

@dataclass
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)

@dataclass
class PoseStamped:
    header: Header = field(default_factory=Header)
    pose: Pose = field(default_factory=Pose)

@dataclass
class LaserScan:
    header: Header = field(default_factory=Header)
    angle_min: float = -math.pi
    angle_max: float = math.pi
    angle_increment: float = 0.01
    time_increment: float = 0.0
    scan_time: float = 0.1
    range_min: float = 0.1
    range_max: float = 30.0
    ranges: List[float] = field(default_factory=list)
    intensities: List[float] = field(default_factory=list)

@dataclass
class Imu:
    header: Header = field(default_factory=Header)
    orientation: Quaternion = field(default_factory=Quaternion)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    linear_acceleration: Vector3 = field(default_factory=Vector3)

@dataclass
class Odometry:
    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    pose: Pose = field(default_factory=Pose)
    twist: Twist = field(default_factory=Twist)

@dataclass
class PointStamped:
    header: Header = field(default_factory=Header)
    point: Point = field(default_factory=Point)

@dataclass
class TransformRequest:
    """Request message for coordinate transform service."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rotation_angle: float = 0.0  # radians
    tx: float = 0.0  # translation x
    ty: float = 0.0  # translation y
    tz: float = 0.0  # translation z

@dataclass
class TransformResponse:
    """Response message for coordinate transform service."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    success: bool = False

# ---------------------------------------------------------------------------
# Time / Duration / Rate
# ---------------------------------------------------------------------------

class Time:
    def __init__(self, seconds=0, nanoseconds=0):
        self._sec = seconds
        self._nanosec = nanoseconds

    @staticmethod
    def now():
        t = time.time()
        return Time(seconds=int(t), nanoseconds=int((t % 1) * 1e9))

    def to_sec(self):
        return self._sec + self._nanosec * 1e-9

    @property
    def sec(self):
        return self._sec

    @property
    def nanosec(self):
        return self._nanosec

    def __repr__(self):
        return f"Time(sec={self._sec}, nanosec={self._nanosec})"

class Duration:
    def __init__(self, seconds=0, nanoseconds=0):
        self._sec = seconds
        self._nanosec = nanoseconds

    def to_sec(self):
        return self._sec + self._nanosec * 1e-9

    def __repr__(self):
        return f"Duration(sec={self._sec}, nanosec={self._nanosec})"

class Rate:
    def __init__(self, hz: float):
        self._period = 1.0 / hz if hz > 0 else 1.0
        self._last = time.time()

    def sleep(self):
        elapsed = time.time() - self._last
        remaining = self._period - elapsed
        if remaining > 0:
            time.sleep(remaining)
        self._last = time.time()

# ---------------------------------------------------------------------------
# Logger
# ---------------------------------------------------------------------------

class Logger:
    def __init__(self, name: str):
        self._name = name

    def _fmt(self, msg):
        return f"[{self._name}] {msg}"

    def info(self, msg):
        print(self._fmt(msg))

    def warn(self, msg):
        print(f"WARN: {self._fmt(msg)}")

    def error(self, msg):
        print(f"ERROR: {self._fmt(msg)}", file=sys.stderr)

    def debug(self, msg):
        print(f"DEBUG: {self._fmt(msg)}")

# ---------------------------------------------------------------------------
# Publisher / Subscription / Timer / Service / Client
# ---------------------------------------------------------------------------

class Publisher:
    def __init__(self, msg_type, topic: str, qos: QoSProfile, node_name: str = ""):
        self._msg_type = msg_type
        self._topic = topic
        self._qos = qos
        self._node_name = node_name
        self._count = 0

    @property
    def topic(self):
        return self._topic

    def publish(self, msg):
        self._count += 1
        # Deliver to all subscribers on this topic
        msg_copy = copy.deepcopy(msg)
        for cb in _topic_callbacks.get(self._topic, []):
            try:
                cb(msg_copy)
            except Exception as e:
                print(f"ERROR in subscriber callback for '{self._topic}': {e}")

    def get_subscription_count(self):
        return len(_topic_callbacks.get(self._topic, []))

    @property
    def publish_count(self):
        return self._count


class Subscription:
    def __init__(self, msg_type, topic: str, callback: Callable, qos: QoSProfile):
        self._msg_type = msg_type
        self._topic = topic
        self._callback = callback
        self._qos = qos
        # Register callback globally
        _topic_callbacks.setdefault(topic, []).append(callback)

    @property
    def topic(self):
        return self._topic

    def destroy(self):
        cbs = _topic_callbacks.get(self._topic, [])
        if self._callback in cbs:
            cbs.remove(self._callback)


class TimerHandle:
    def __init__(self, period_sec: float, callback: Callable, node: "Node"):
        self._period = period_sec
        self._callback = callback
        self._node = node
        self._canceled = False
        self._tick_count = 0

    @property
    def period(self):
        return self._period

    @property
    def tick_count(self):
        return self._tick_count

    def cancel(self):
        self._canceled = True

    def is_canceled(self):
        return self._canceled

    def fire(self):
        if not self._canceled:
            self._tick_count += 1
            self._callback()


class ServiceHandle:
    def __init__(self, srv_type, name: str, callback: Callable):
        self._srv_type = srv_type
        self._name = name
        self._callback = callback
        _service_handlers[name] = callback

    @property
    def service_name(self):
        return self._name

    def destroy(self):
        _service_handlers.pop(self._name, None)


class ClientHandle:
    def __init__(self, srv_type, name: str):
        self._srv_type = srv_type
        self._name = name

    @property
    def service_name(self):
        return self._name

    def service_is_ready(self):
        return self._name in _service_handlers

    def wait_for_service(self, timeout_sec=5.0):
        return self.service_is_ready()

    def call(self, request):
        """Synchronous service call (simulated)."""
        handler = _service_handlers.get(self._name)
        if handler is None:
            raise RuntimeError(f"Service '{self._name}' not available")
        return handler(request)

    def call_async(self, request):
        """Return a simple future-like object with the result."""
        result = self.call(request)
        return _Future(result)


class _Future:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def done(self):
        return True

# ---------------------------------------------------------------------------
# Parameter
# ---------------------------------------------------------------------------

@dataclass
class Parameter:
    name: str
    value: Any
    type_name: str = "string"  # string, int, double, bool

class ParameterEvent:
    def __init__(self, parameters: List[Parameter]):
        self.parameters = parameters

# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class Node:
    def __init__(self, name: str, namespace: str = ""):
        self._name = name
        self._namespace = namespace
        self._logger = Logger(name)
        self._publishers: List[Publisher] = []
        self._subscriptions: List[Subscription] = []
        self._timers: List[TimerHandle] = []
        self._services: List[ServiceHandle] = []
        self._clients: List[ClientHandle] = []
        self._parameters: Dict[str, Parameter] = {}
        self._param_callbacks: List[Callable] = []
        _nodes.append(self)

    # -- Introspection --
    def get_name(self) -> str:
        return self._name

    def get_namespace(self) -> str:
        return self._namespace

    def get_fully_qualified_name(self) -> str:
        ns = self._namespace if self._namespace else ""
        return f"{ns}/{self._name}"

    def get_logger(self) -> Logger:
        return self._logger

    # -- Publisher --
    def create_publisher(self, msg_type, topic: str, qos=10):
        if isinstance(qos, int):
            qos = QoSProfile(depth=qos)
        pub = Publisher(msg_type, topic, qos, self._name)
        self._publishers.append(pub)
        return pub

    # -- Subscription --
    def create_subscription(self, msg_type, topic: str, callback: Callable, qos=10):
        if isinstance(qos, int):
            qos = QoSProfile(depth=qos)
        sub = Subscription(msg_type, topic, callback, qos)
        self._subscriptions.append(sub)
        return sub

    # -- Timer --
    def create_timer(self, period_sec: float, callback: Callable):
        timer = TimerHandle(period_sec, callback, self)
        self._timers.append(timer)
        return timer

    # -- Service --
    def create_service(self, srv_type, name: str, callback: Callable):
        srv = ServiceHandle(srv_type, name, callback)
        self._services.append(srv)
        return srv

    # -- Client --
    def create_client(self, srv_type, name: str):
        client = ClientHandle(srv_type, name)
        self._clients.append(client)
        return client

    # -- Parameters --
    def declare_parameter(self, name: str, value: Any = None):
        p = Parameter(name=name, value=value)
        self._parameters[name] = p
        return p

    def get_parameter(self, name: str) -> Parameter:
        if name not in self._parameters:
            raise RuntimeError(f"Parameter '{name}' not declared")
        return self._parameters[name]

    def set_parameter(self, name: str, value: Any):
        if name not in self._parameters:
            self.declare_parameter(name, value)
        old = self._parameters[name]
        new_param = Parameter(name=name, value=value, type_name=old.type_name)
        # Run validation callbacks
        for cb in self._param_callbacks:
            result = cb(ParameterEvent([new_param]))
            if result is False:
                self._logger.warn(f"Parameter update rejected for '{name}'")
                return False
        self._parameters[name] = new_param
        return True

    def set_parameters(self, params: List[Parameter]):
        for p in params:
            self.set_parameter(p.name, p.value)

    def add_on_set_parameters_callback(self, callback: Callable):
        self._param_callbacks.append(callback)

    def get_parameters(self, names: List[str]) -> List[Parameter]:
        return [self._parameters[n] for n in names if n in self._parameters]

    def has_parameter(self, name: str) -> bool:
        return name in self._parameters

    # -- Destroy --
    def destroy_node(self):
        for sub in self._subscriptions:
            sub.destroy()
        for srv in self._services:
            srv.destroy()
        if self in _nodes:
            _nodes.remove(self)

    def __repr__(self):
        return f"Node('{self._name}')"

# ---------------------------------------------------------------------------
# Spin helpers
# ---------------------------------------------------------------------------

def spin_once(node: Node, timeout_sec: float = 0.1):
    """Fire all timers once (simulates one executor cycle)."""
    for timer in node._timers:
        if not timer.is_canceled():
            timer.fire()
    time.sleep(timeout_sec)


def spin(node: Node, duration_sec: float = None):
    """Spin a single node, firing timers at their configured rates.

    If *duration_sec* is given, spin for that duration then return.
    Otherwise spin until shutdown() is called.
    """
    start = time.time()
    last_fire: Dict[int, float] = {}
    while ok():
        if duration_sec is not None and (time.time() - start) >= duration_sec:
            break
        for timer in node._timers:
            tid = id(timer)
            now = time.time()
            if tid not in last_fire:
                last_fire[tid] = now
            if now - last_fire[tid] >= timer.period:
                timer.fire()
                last_fire[tid] = now
        time.sleep(0.005)  # small sleep to avoid busy-wait


def spin_nodes(nodes: List[Node], duration_sec: float = 5.0):
    """Spin multiple nodes concurrently for *duration_sec* seconds."""
    start = time.time()
    last_fire: Dict[int, float] = {}
    while ok() and (time.time() - start) < duration_sec:
        for node in nodes:
            for timer in node._timers:
                tid = id(timer)
                now = time.time()
                if tid not in last_fire:
                    last_fire[tid] = now
                if now - last_fire[tid] >= timer.period:
                    timer.fire()
                    last_fire[tid] = now
        time.sleep(0.005)


# ---------------------------------------------------------------------------
# Convenience: Simulated spin that fires timer N times (no wall-clock wait)
# ---------------------------------------------------------------------------

def spin_fast(node_or_nodes, n_ticks: int = 50):
    """Fire every timer on every node *n_ticks* times with no real-time delay.
    Useful for quick testing and solution validation.
    """
    if isinstance(node_or_nodes, Node):
        nodes = [node_or_nodes]
    else:
        nodes = list(node_or_nodes)
    for _ in range(n_ticks):
        if not ok():
            break
        for node in nodes:
            for timer in node._timers:
                if not timer.is_canceled():
                    timer.fire()


# ---------------------------------------------------------------------------
# Reset (useful between tests)
# ---------------------------------------------------------------------------

def reset():
    """Clear all global state. Useful between exercises."""
    global _topic_callbacks, _service_handlers, _nodes, _initialized, _shutdown_flag
    _topic_callbacks = {}
    _service_handlers = {}
    _nodes = []
    _initialized = False
    _shutdown_flag = False
