#!/usr/bin/env python3
"""
Solution: Task 6 - Launch File Simulation
==========================================
Demonstrates YAML config loading, node instantiation, topic remapping, and
connection verification - all patterns used in ROS2 launch files.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
import ros2_sim as rclpy
from ros2_sim import Node, LaserScan, Twist, Float64, Pose


def load_yaml_config(path: str) -> dict:
    try:
        import yaml
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    except ImportError:
        # Minimal YAML-like parser as fallback
        import json, re
        with open(path, 'r') as f:
            text = f.read()
        # Very simple parser: convert YAML to something we can eval
        # For the specific robot_params.yaml format only
        result = {'nodes': []}
        current_node = None
        current_section = None
        current_dict = None
        for line in text.split('\n'):
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            indent = len(line) - len(line.lstrip())
            if stripped.startswith('- name:'):
                current_node = {
                    'name': stripped.split(':', 1)[1].strip(),
                    'parameters': {},
                    'publishers': [],
                    'subscribers': [],
                    'remappings': {},
                }
                result['nodes'].append(current_node)
                current_section = None
                current_dict = None
            elif current_node is not None:
                if stripped == 'parameters:':
                    current_section = 'parameters'
                    current_dict = current_node['parameters']
                elif stripped == 'publishers:':
                    current_section = 'publishers'
                elif stripped == 'subscribers:':
                    current_section = 'subscribers'
                elif stripped == 'remappings:':
                    current_section = 'remappings'
                    current_dict = current_node['remappings']
                elif stripped.startswith('- ') and current_section in ('publishers', 'subscribers'):
                    val = stripped[2:].strip()
                    current_node[current_section].append(val)
                elif ':' in stripped and current_section in ('parameters', 'remappings'):
                    key, val = stripped.split(':', 1)
                    val = val.strip()
                    # Type coercion
                    if val == 'true':
                        val = True
                    elif val == 'false':
                        val = False
                    elif val == '[]':
                        val = []
                    else:
                        try:
                            val = float(val)
                            if val == int(val) and '.' not in stripped.split(':')[1]:
                                val = int(val)
                        except (ValueError, TypeError):
                            pass
                    if current_section == 'parameters':
                        current_node['parameters'][key.strip()] = val
                    else:
                        current_node['remappings'][key.strip()] = val
        return result


def create_launch_description(config: dict) -> list:
    description = []
    for node_cfg in config.get('nodes', []):
        entry = {
            'name': node_cfg['name'],
            'parameters': node_cfg.get('parameters', {}),
            'remappings': node_cfg.get('remappings', {}),
            'publishers': node_cfg.get('publishers', []),
            'subscribers': node_cfg.get('subscribers', []),
        }
        description.append(entry)
    return description


def launch_nodes(description: list) -> list:
    nodes = []
    for entry in description:
        node = Node(entry['name'])
        # Declare parameters
        for pname, pval in entry['parameters'].items():
            node.declare_parameter(pname, pval)
        # Apply remappings to topics
        remappings = entry['remappings']
        # Create publishers
        for topic in entry['publishers']:
            remapped = remappings.get(topic, topic)
            node.create_publisher(None, remapped, qos=10)
            node.get_logger().info(f"Publisher: {topic} -> {remapped}")
        # Create subscriptions
        for topic in entry['subscribers']:
            remapped = remappings.get(topic, topic)
            node.create_subscription(None, remapped, lambda msg: None, qos=10)
            node.get_logger().info(f"Subscription: {topic} -> {remapped}")
        nodes.append(node)
    return nodes


def verify_connections(nodes: list) -> dict:
    published = set()
    subscribed = set()
    for node in nodes:
        for pub in node._publishers:
            published.add(pub.topic)
        for sub in node._subscriptions:
            subscribed.add(sub.topic)
    unmatched = sorted(subscribed - published)
    return {
        'all_connected': len(unmatched) == 0,
        'published_topics': sorted(published),
        'subscribed_topics': sorted(subscribed),
        'unmatched': unmatched,
    }


def main():
    print("=" * 60)
    print("Task 6 Solution: Launch File Simulation")
    print("=" * 60)
    print()
    print("ROS2 Concepts demonstrated:")
    print("  - YAML parameter files")
    print("  - Launch descriptions (nodes, params, remappings)")
    print("  - Topic remapping")
    print("  - System connection verification")
    print()

    rclpy.init()
    config_path = os.path.join(os.path.dirname(__file__), '..', 'lab_exercises', 'config', 'robot_params.yaml')
    print(f"Loading config: {config_path}\n")

    config = load_yaml_config(config_path)
    print(f"Found {len(config['nodes'])} node definitions\n")

    description = create_launch_description(config)
    print("Launch description:")
    for entry in description:
        print(f"  Node '{entry['name']}':")
        print(f"    Parameters: {list(entry['parameters'].keys())}")
        print(f"    Publishers: {entry['publishers']}")
        print(f"    Subscribers: {entry['subscribers']}")
        print(f"    Remappings: {entry['remappings']}")
    print()

    print("Launching nodes...\n")
    nodes = launch_nodes(description)

    result = verify_connections(nodes)
    print("\n=== Launch Verification ===")
    for k, v in result.items():
        print(f"  {k}: {v}")

    rclpy.spin_nodes(nodes, duration_sec=1.0)

    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
