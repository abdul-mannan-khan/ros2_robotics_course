#!/usr/bin/env python3
"""Task 1 Solution: System Architecture Design"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import Dict, List
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from capstone_sim import HealthMonitor


class ComponentInterface:
    def __init__(self, name, msg_type, rate_hz):
        self.name = name
        self.msg_type = msg_type
        self.rate_hz = rate_hz


class ComponentRegistry:
    def __init__(self):
        self.components = {}
        self.connections = []

    def register_component(self, name, comp_type, rate_hz):
        self.components[name] = {'type': comp_type, 'rate_hz': rate_hz}

    def add_connection(self, source, target, interface):
        assert source in self.components, f"Unknown source: {source}"
        assert target in self.components, f"Unknown target: {target}"
        self.connections.append({'source': source, 'target': target, 'interface': interface})

    def verify_all_connected(self):
        connected = set()
        for c in self.connections:
            connected.add(c['source'])
            connected.add(c['target'])
        return all(name in connected for name in self.components)

    def get_component_graph(self):
        return {'components': dict(self.components), 'connections': list(self.connections)}


def build_drone_architecture():
    r = ComponentRegistry()
    r.register_component('lidar_node', 'perception', 10)
    r.register_component('camera_node', 'perception', 30)
    r.register_component('imu_node', 'sensor', 200)
    r.register_component('obstacle_detector', 'perception', 10)
    r.register_component('ekf_fusion', 'estimation', 100)
    r.register_component('map_server', 'mapping', 5)
    r.register_component('path_planner', 'planning', 2)
    r.register_component('trajectory_gen', 'planning', 50)
    r.register_component('controller', 'control', 100)
    r.register_component('px4_bridge', 'actuation', 250)
    conns = [
        ('lidar_node', 'obstacle_detector', ComponentInterface('/pointcloud', 'PointCloud2', 10)),
        ('camera_node', 'obstacle_detector', ComponentInterface('/image', 'Image', 30)),
        ('imu_node', 'ekf_fusion', ComponentInterface('/imu', 'Imu', 200)),
        ('obstacle_detector', 'map_server', ComponentInterface('/obstacles', 'ObstacleArray', 10)),
        ('ekf_fusion', 'path_planner', ComponentInterface('/pose', 'PoseStamped', 100)),
        ('ekf_fusion', 'controller', ComponentInterface('/state', 'State', 100)),
        ('map_server', 'path_planner', ComponentInterface('/map', 'OccupancyGrid', 5)),
        ('path_planner', 'trajectory_gen', ComponentInterface('/path', 'Path', 2)),
        ('trajectory_gen', 'controller', ComponentInterface('/trajectory', 'Trajectory', 50)),
        ('controller', 'px4_bridge', ComponentInterface('/cmd', 'AttitudeTarget', 100)),
        ('camera_node', 'ekf_fusion', ComponentInterface('/visual_odom', 'Odometry', 30)),
    ]
    for s, t, iface in conns:
        r.add_connection(s, t, iface)
    return r


def setup_health_monitoring(registry):
    m = HealthMonitor()
    timeouts = {'lidar_node': 0.2, 'camera_node': 0.1, 'imu_node': 0.01,
                'obstacle_detector': 0.2, 'ekf_fusion': 0.05, 'map_server': 0.5,
                'path_planner': 2.0, 'trajectory_gen': 0.1, 'controller': 0.02, 'px4_bridge': 0.01}
    for name in registry.components:
        m.register(name, timeouts.get(name, 1.0))
    return m


def visualize_architecture(registry, filename='task1_architecture.png'):
    fig, ax = plt.subplots(1, 1, figsize=(14, 8))
    positions = {
        'lidar_node': (0.5, 4), 'camera_node': (0.5, 3), 'imu_node': (0.5, 2),
        'obstacle_detector': (3, 3.5), 'ekf_fusion': (3, 2),
        'map_server': (5.5, 4), 'path_planner': (5.5, 3),
        'trajectory_gen': (5.5, 2), 'controller': (8, 2.5), 'px4_bridge': (10, 2.5),
    }
    colors = {'perception': '#e91e63', 'sensor': '#ff9800', 'estimation': '#9c27b0',
              'mapping': '#2196f3', 'planning': '#03a9f4', 'control': '#4CAF50', 'actuation': '#ff5722'}
    for name, info in registry.components.items():
        x, y = positions.get(name, (5, 2.5))
        c = colors.get(info['type'], '#999')
        ax.add_patch(plt.Rectangle((x-0.7, y-0.3), 1.4, 0.6, facecolor=c, alpha=0.8, edgecolor='black', linewidth=1.5, zorder=2))
        ax.text(x, y, name.replace('_', '\n'), ha='center', va='center', fontsize=7, fontweight='bold', color='white', zorder=3)
    for conn in registry.connections:
        s = positions.get(conn['source'], (5, 2.5))
        t = positions.get(conn['target'], (5, 2.5))
        ax.annotate('', xy=(t[0]-0.7, t[1]), xytext=(s[0]+0.7, s[1]),
                    arrowprops=dict(arrowstyle='->', color='#555', lw=1.5), zorder=1)
    ax.set_xlim(-1, 12)
    ax.set_ylim(1, 5)
    ax.set_title('Autonomous Drone System Architecture', fontsize=14, fontweight='bold')
    ax.set_aspect('equal')
    ax.axis('off')
    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Saved: {filename}")
    plt.close()


def main():
    print("Task 1 Solution: System Architecture Design")
    print("=" * 50)
    registry = build_drone_architecture()
    print(f"Components registered: {len(registry.components)}")
    print(f"Connections: {len(registry.connections)}")
    print(f"All connected: {registry.verify_all_connected()}")
    monitor = setup_health_monitoring(registry)
    print(f"Health monitor: {len(monitor.components)} components")
    # Simulate health checks
    for t in [0.0, 0.05, 0.1, 0.5]:
        monitor.heartbeat('ekf_fusion', t)
        monitor.heartbeat('controller', t)
    status = monitor.check(0.5)
    print(f"Health at t=0.5s: {status}")
    visualize_architecture(registry)
    graph = registry.get_component_graph()
    print(f"\nGraph summary: {len(graph['components'])} nodes, {len(graph['connections'])} edges")
    print("\nTask 1 Solution complete.")


if __name__ == '__main__':
    main()
