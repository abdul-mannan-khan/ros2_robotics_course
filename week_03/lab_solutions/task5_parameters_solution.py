#!/usr/bin/env python3
"""
Solution: Task 5 - Parameter Configuration
============================================
Demonstrates ROS2 parameters with a PID controller.
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
import ros2_sim as rclpy
from ros2_sim import Node, Float64, Twist, Parameter, ParameterEvent


def parameter_validation_callback(event: ParameterEvent) -> bool:
    for p in event.parameters:
        if p.name == 'max_speed' and p.value <= 0:
            print(f"  REJECTED: max_speed must be > 0 (got {p.value})")
            return False
        if p.name in ('kp', 'ki', 'kd') and p.value < 0:
            print(f"  REJECTED: {p.name} must be >= 0 (got {p.value})")
            return False
        if p.name == 'control_rate' and (p.value <= 0 or p.value > 100):
            print(f"  REJECTED: control_rate must be in (0, 100] (got {p.value})")
            return False
    return True


def control_loop(node: Node):
    sp = node.get_parameter('setpoint').value
    kp = node.get_parameter('kp').value
    ki = node.get_parameter('ki').value
    kd = node.get_parameter('kd').value
    max_speed = node.get_parameter('max_speed').value
    dt = 1.0 / node.get_parameter('control_rate').value

    error = sp - node.process_value
    node.integral += error * dt
    derivative = (error - node.prev_error) / dt
    output = kp * error + ki * node.integral + kd * derivative
    output = max(-max_speed, min(max_speed, output))

    node.process_value += output * dt
    node.prev_error = error
    node.control_history.append((node.process_value, error, output))

    step = len(node.control_history)
    if step % 10 == 0:
        node.get_logger().info(
            f"Step {step:3d}: PV={node.process_value:.4f}, "
            f"err={error:.4f}, out={output:.4f}"
        )


def create_configurable_controller() -> Node:
    node = Node('pid_controller')
    node.declare_parameter('max_speed', 1.0)
    node.declare_parameter('kp', 1.0)
    node.declare_parameter('ki', 0.0)
    node.declare_parameter('kd', 0.1)
    node.declare_parameter('control_rate', 10.0)
    node.declare_parameter('setpoint', 0.0)

    node.integral = 0.0
    node.prev_error = 0.0
    node.process_value = 5.0
    node.control_history = []

    node.add_on_set_parameters_callback(parameter_validation_callback)
    node.create_timer(1.0 / node.get_parameter('control_rate').value,
                      lambda: control_loop(node))

    node.get_logger().info("PID controller created with parameters:")
    for name in ['max_speed', 'kp', 'ki', 'kd', 'control_rate', 'setpoint']:
        node.get_logger().info(f"  {name} = {node.get_parameter(name).value}")
    return node


def test_parameter_updates(node: Node):
    print("\n--- Phase 1: Default parameters (setpoint=0), 50 ticks ---")
    rclpy.spin_fast(node, 50)
    pv1 = node.process_value
    print(f"  Process value after 50 ticks: {pv1:.4f}")

    print("\n--- Phase 2: Change setpoint to 10.0, 50 ticks ---")
    node.set_parameter('setpoint', 10.0)
    node.get_logger().info("Setpoint changed to 10.0")
    rclpy.spin_fast(node, 50)
    pv2 = node.process_value
    print(f"  Process value after 50 more ticks: {pv2:.4f}")

    print("\n--- Phase 3: Try invalid max_speed = -1.0 ---")
    result = node.set_parameter('max_speed', -1.0)
    print(f"  Parameter update accepted: {result}")

    print("\n--- Phase 4: Change kp to 2.0, 50 more ticks ---")
    node.set_parameter('kp', 2.0)
    node.get_logger().info("kp changed to 2.0")
    rclpy.spin_fast(node, 50)
    pv3 = node.process_value
    print(f"  Process value after 50 more ticks: {pv3:.4f}")

    print(f"\n=== Summary ===")
    print(f"  Total control steps: {len(node.control_history)}")
    print(f"  Final process value: {node.process_value:.4f}")
    print(f"  Final setpoint:      {node.get_parameter('setpoint').value}")
    print(f"  Final error:         {node.prev_error:.4f}")

    # Save plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        hist = node.control_history
        steps = range(len(hist))
        pvs = [h[0] for h in hist]
        errs = [h[1] for h in hist]
        outs = [h[2] for h in hist]

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axes[0].plot(steps, pvs, 'b-')
        axes[0].set_ylabel('Process Value')
        axes[0].axhline(y=0, color='r', linestyle='--', alpha=0.5, label='SP=0')
        axes[0].axhline(y=10, color='g', linestyle='--', alpha=0.5, label='SP=10')
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(steps, errs, 'r-')
        axes[1].set_ylabel('Error')
        axes[1].grid(True)

        axes[2].plot(steps, outs, 'g-')
        axes[2].set_ylabel('Control Output')
        axes[2].set_xlabel('Step')
        axes[2].grid(True)

        fig.suptitle('PID Controller with Dynamic Parameter Updates')
        plot_path = os.path.join(os.path.dirname(__file__), 'task5_pid.png')
        fig.savefig(plot_path, dpi=100, bbox_inches='tight')
        print(f"\nPlot saved to {plot_path}")
    except ImportError:
        print("\nmatplotlib not available - skipping plot.")


def main():
    print("=" * 60)
    print("Task 5 Solution: Parameter Configuration")
    print("=" * 60)
    print()
    print("ROS2 Concepts demonstrated:")
    print("  - Declaring parameters with defaults")
    print("  - Reading parameters in control loop")
    print("  - Dynamic parameter updates with validation")
    print("  - Parameter rejection on invalid values")
    print()

    rclpy.init()
    node = create_configurable_controller()
    test_parameter_updates(node)
    node.destroy_node()
    rclpy.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
