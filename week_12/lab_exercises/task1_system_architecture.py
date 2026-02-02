#!/usr/bin/env python3
"""
Task 1: System Architecture Design
===================================
TC70045E Week 12 - Dr. Abdul Manan Khan, UWL

Design and implement the full system node graph with component registry
and health monitoring.

TODO: Complete the functions marked with TODO
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import Dict, List
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capstone_sim import HealthMonitor


class ComponentInterface:
    """Defines an interface between two components."""
    def __init__(self, name: str, msg_type: str, rate_hz: float):
        self.name = name
        self.msg_type = msg_type
        self.rate_hz = rate_hz


class ComponentRegistry:
    """Registry of all system components and their interfaces."""
    def __init__(self):
        self.components: Dict[str, Dict] = {}
        self.connections: List[Dict] = []

    def register_component(self, name: str, comp_type: str, rate_hz: float):
        """Register a system component.
        TODO: Store the component with its type and rate.
        """
        pass  # TODO

    def add_connection(self, source: str, target: str, interface: ComponentInterface):
        """Add a connection between two components.
        TODO: Store the connection and validate both components exist.
        """
        pass  # TODO

    def verify_all_connected(self) -> bool:
        """Verify all components have at least one connection.
        TODO: Check every registered component appears in at least one connection.
        """
        pass  # TODO

    def get_component_graph(self) -> Dict:
        """Return the full component graph.
        TODO: Return dict with 'components' and 'connections' keys.
        """
        pass  # TODO


def build_drone_architecture() -> ComponentRegistry:
    """Build the complete drone system architecture.
    TODO: Register all components (perception, estimation, planning, control, actuation)
    and define all connections between them with appropriate interfaces.
    Expected components: lidar_node, camera_node, imu_node, obstacle_detector,
    ekf_fusion, map_server, path_planner, trajectory_gen, controller, px4_bridge
    """
    registry = ComponentRegistry()
    # TODO: Register components and add connections
    return registry


def setup_health_monitoring(registry: ComponentRegistry) -> HealthMonitor:
    """Set up health monitoring for all registered components.
    TODO: Create HealthMonitor and register all components with appropriate timeouts.
    """
    monitor = HealthMonitor()
    # TODO
    return monitor


def visualize_architecture(registry: ComponentRegistry, filename='task1_architecture.png'):
    """Visualize the system architecture.
    TODO: Create a visualization showing components and connections.
    """
    pass  # TODO


def main():
    print("Task 1: System Architecture Design")
    print("=" * 50)
    registry = build_drone_architecture()
    print(f"Components registered: {len(registry.components)}")
    connected = registry.verify_all_connected()
    print(f"All connected: {connected}")
    monitor = setup_health_monitoring(registry)
    print(f"Health monitor components: {len(monitor.components)}")
    visualize_architecture(registry)
    print("\nTask 1 complete.")


if __name__ == '__main__':
    main()
