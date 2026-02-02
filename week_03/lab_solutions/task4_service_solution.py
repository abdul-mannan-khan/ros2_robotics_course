#!/usr/bin/env python3
"""
Solution: Task 4 - Service Server and Client
==============================================
Demonstrates ROS2 service request/response for coordinate transforms.
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab_exercises'))
import ros2_sim as rclpy
from ros2_sim import Node, TransformRequest, TransformResponse


def transform_service_callback(request: TransformRequest) -> TransformResponse:
    a = request.rotation_angle
    # Rotation
    xr = request.x * math.cos(a) - request.y * math.sin(a)
    yr = request.x * math.sin(a) + request.y * math.cos(a)
    # Translation
    response = TransformResponse(
        x=xr + request.tx,
        y=yr + request.ty,
        z=request.z + request.tz,
        success=True,
    )
    return response


def create_transform_server() -> Node:
    node = Node('transform_server')
    node.create_service(None, '/transform_point', transform_service_callback)
    node.get_logger().info("Service '/transform_point' is ready")
    return node


def create_transform_client() -> Node:
    node = Node('transform_client')
    node.client = node.create_client(None, '/transform_point')
    node.get_logger().info("Client for '/transform_point' created")
    return node


def main():
    print("=" * 60)
    print("Task 4 Solution: Service Server and Client")
    print("=" * 60)
    print()
    print("ROS2 Concepts demonstrated:")
    print("  - Service server (name, callback)")
    print("  - Service client (synchronous call)")
    print("  - Request / Response pattern")
    print()

    rclpy.init()
    server_node = create_transform_server()
    client_node = create_transform_client()

    # -- Test cases --
    tests = [
        {
            'desc': "Rotate (1,0) by pi/2",
            'request': TransformRequest(x=1.0, y=0.0, rotation_angle=math.pi / 2),
            'expected_x': 0.0,
            'expected_y': 1.0,
        },
        {
            'desc': "Translate (0,0) by (3,4)",
            'request': TransformRequest(x=0.0, y=0.0, tx=3.0, ty=4.0),
            'expected_x': 3.0,
            'expected_y': 4.0,
        },
        {
            'desc': "Rotate (1,0) by pi/4 then translate (1,1)",
            'request': TransformRequest(x=1.0, y=0.0, rotation_angle=math.pi / 4, tx=1.0, ty=1.0),
            'expected_x': 1.0 + math.cos(math.pi / 4),
            'expected_y': 1.0 + math.sin(math.pi / 4),
        },
    ]

    print("Running test cases:\n")
    all_pass = True
    for i, test in enumerate(tests, 1):
        resp = client_node.client.call(test['request'])
        tol = 1e-6
        pass_x = abs(resp.x - test['expected_x']) < tol
        pass_y = abs(resp.y - test['expected_y']) < tol
        passed = pass_x and pass_y and resp.success

        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False
        print(f"  Test {i}: {test['desc']}")
        print(f"    Expected: ({test['expected_x']:.4f}, {test['expected_y']:.4f})")
        print(f"    Got:      ({resp.x:.4f}, {resp.y:.4f})")
        print(f"    Result:   {status}")
        print()

    print(f"Overall: {'ALL TESTS PASSED' if all_pass else 'SOME TESTS FAILED'}")

    server_node.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
