#!/usr/bin/env python3
"""
Task 4: Service Server and Client
===================================

Goal: Create a service that performs 2-D coordinate transforms
(rotation + translation) and a client that calls it.

ROS2 Concepts Practised:
    - Creating a Service (name, type, callback)
    - Creating a Client (name, type)
    - Synchronous and asynchronous service calls
    - Request / Response pattern

Functions to implement:
    1. transform_service_callback(request) -> response
    2. create_transform_server() -> Node
    3. create_transform_client() -> Node
    4. main()

Run:
    python3 task4_service.py
"""

import sys, os, math
sys.path.insert(0, os.path.dirname(__file__))
import ros2_sim as rclpy
from ros2_sim import Node, TransformRequest, TransformResponse


def transform_service_callback(request: TransformRequest) -> TransformResponse:
    """Apply a 2-D rotation then translation to the input point.

    Rotation (about z-axis by request.rotation_angle):
        x' = x * cos(a) - y * sin(a)
        y' = x * sin(a) + y * cos(a)
    Translation:
        x'' = x' + tx
        y'' = y' + ty

    Set response.success = True and return.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement transform_service_callback()")


def create_transform_server() -> Node:
    """Create 'transform_server' node with service '/transform_point'
    using transform_service_callback.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_transform_server()")


def create_transform_client() -> Node:
    """Create 'transform_client' node with a client for '/transform_point'.
    Store client handle as node.client.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement create_transform_client()")


def main():
    """Start server, create client, send several test requests, verify results.

    Test cases:
        1. Rotate (1, 0) by pi/2 -> expect (0, 1)
        2. Translate (0, 0) by (3, 4) -> expect (3, 4)
        3. Rotate (1, 0) by pi/4 then translate (1, 1) -> expect (1+cos45, 1+sin45)
    Print PASS/FAIL for each.
    """
    # >>> YOUR CODE HERE <<<
    raise NotImplementedError("Implement main()")


if __name__ == "__main__":
    main()
