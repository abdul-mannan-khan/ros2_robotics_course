#!/usr/bin/env python3
"""
Task 5: Parameter Configuration
=================================

Goal: Create a configurable PID robot controller that reads its gains
and limits from ROS2-style parameters and supports runtime updates.

ROS2 Concepts Practised:
    - Declaring parameters with default values
    - Reading parameters in callbacks
    - Dynamic parameter update with validation callback
    - Using parameters in a control loop

Functions to implement:
    1. create_configurable_controller() -> Node
    2. parameter_validation_callback(event) -> bool
    3. control_loop(node)  (timer callback that runs PID)
    4. test_parameter_updates(node)

Run:
    python3 task5_parameters.py
"""

import sys, os, math
sys.path.insert(0, os.path.dirname(__file__))
import ros2_sim as rclpy
from ros2_sim import Node, Float64, Twist


def create_configurable_controller() -> Node:
    """Create 'pid_controller' node with parameters:
        - max_speed    (float, default 1.0)
        - kp           (float, default 1.0)
        - ki           (float, default 0.0)
        - kd           (float, default 0.1)
        - control_rate (float, default 10.0)  in Hz
        - setpoint     (float, default 0.0)

    Also initialise:
        node.integral = 0.0
        node.prev_error = 0.0
        node.process_value = 5.0   (simulated plant state, starts far from setpoint)
        node.control_history = []  (list of (pv, error, output) tuples)

    Register parameter_validation_callback via add_on_set_parameters_callback.
    Create a timer at control_rate Hz calling control_loop.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_configurable_controller()")


def parameter_validation_callback(event) -> bool:
    """Validate parameter updates.

    Rules:
        - max_speed must be > 0
        - kp must be >= 0
        - ki must be >= 0
        - kd must be >= 0
        - control_rate must be > 0 and <= 100

    Return True if all valid, False otherwise.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement parameter_validation_callback()")


def control_loop(node: Node):
    """PID control step.

    1. Read setpoint, kp, ki, kd, max_speed from parameters.
    2. error = setpoint - node.process_value
    3. node.integral += error * dt   (dt = 1/control_rate)
    4. derivative = (error - node.prev_error) / dt
    5. output = kp*error + ki*integral + kd*derivative
    6. Clamp output to [-max_speed, max_speed]
    7. Simulate plant: node.process_value += output * dt
    8. Store node.prev_error = error
    9. Append (process_value, error, output) to node.control_history
    10. Log every 10 steps.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement control_loop()")


def test_parameter_updates(node: Node):
    """Test dynamic parameter changes.

    1. Run 50 ticks with default parameters.
    2. Change setpoint to 10.0, run 50 more ticks.
    3. Try setting max_speed to -1.0 (should be rejected).
    4. Change kp to 2.0, run 50 more ticks.
    5. Print summary of control_history.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement test_parameter_updates()")


def main():
    rclpy.init()
    node = create_configurable_controller()
    test_parameter_updates(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
