# Week 3: ROS2 Fundamentals & Node Architecture

## Learning Objectives

- Understand ROS2 node lifecycle and architecture
- Create publishers and subscribers with proper QoS configuration
- Implement services for request/response communication
- Configure nodes with parameters and dynamic reconfiguration
- Design multi-node systems with proper topic routing
- Simulate a ROS2 launch system with YAML configuration

## Structure

```
week_03/
  lab_exercises/          # Exercise stubs (implement these)
    ros2_sim.py           # Simulation framework (provided)
    task1_publisher.py    # Basic publisher node
    task2_subscriber.py   # Subscriber with statistics
    task3_pub_sub_system.py  # Multi-node pipeline
    task4_service.py      # Service server & client
    task5_parameters.py   # Parameter configuration
    task6_launch_system.py   # Launch file simulation
    task7_integration.py  # Full robot system
    config/
      robot_params.yaml   # Configuration for task 6
  lab_solutions/          # Complete working solutions
  resources/
    references.md         # ROS2 documentation links
```

## Running

All exercises run standalone with Python 3 -- no ROS2 installation needed:

```bash
cd lab_exercises
python3 task1_publisher.py
```

Solutions:

```bash
cd lab_solutions
python3 task1_publisher_solution.py
```

## Topics Covered

1. **Nodes** -- the fundamental computation unit in ROS2
2. **Topics** -- anonymous publish/subscribe messaging
3. **Services** -- synchronous request/response calls
4. **Parameters** -- runtime configuration with validation
5. **QoS** -- Quality of Service profiles for reliability/performance
6. **Launch** -- system-level orchestration of multiple nodes
7. **Integration** -- designing a complete robot software stack
