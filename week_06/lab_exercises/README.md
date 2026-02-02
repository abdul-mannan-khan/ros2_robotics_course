# Week 6 Lab Exercises: Quadrotor Dynamics & Attitude Control

## Shared Module

`quadrotor_sim.py` provides the core simulation infrastructure: quadrotor parameters,
rotation utilities, dynamics, RK4 integration, and a simulation loop. All exercises
import from this module.

## Exercises

### Task 1: Rotation Math (`task1_rotation_math.py`)
Implement Euler angle and quaternion operations:
- ZYX rotation matrix construction and extraction
- Euler-to-quaternion and back conversion
- Quaternion vector rotation
- Gimbal lock demonstration at 90 degrees pitch

### Task 2: Dynamics Model (`task2_dynamics_model.py`)
Implement the equations of motion:
- Motor mixing (speeds to thrust/torques)
- Translational dynamics (F=ma with gravity)
- Rotational dynamics (Euler's equation)
- Verify with free-fall, hover, and single-motor tests

### Task 3: PID Attitude Controller (`task3_pid_attitude.py`)
Build a cascaded PID controller:
- PID class with anti-windup and output limiting
- Outer angle loop + inner rate loop
- Altitude controller with gravity feedforward
- Test hover stabilization and step responses

### Task 4: Motor Mixing (`task4_motor_mixing.py`)
Motor allocation and saturation:
- Build the 4x4 mixing matrix for + configuration
- Inverse mixing for motor allocation
- Saturation handling with thrust priority
- Verify forward/inverse roundtrip consistency

### Task 5: Cascaded Control (`task5_cascaded_control.py`)
Full position + attitude control:
- PD position controller
- Acceleration-to-attitude conversion (small angle approximation)
- Complete cascaded pipeline
- Fly waypoint mission: takeoff, move, return, land

### Task 6: Complementary Filter (`task6_complementary_filter.py`)
IMU-based attitude estimation:
- Complementary filter fusing gyro + accelerometer
- Simulated noisy IMU data generation
- Alpha parameter sweep analysis
- Comparison with ground truth

### Task 7: Full Simulation (`task7_full_simulation.py`)
Integrated quadrotor system:
- Dynamics + control + estimation + mixing
- Square trajectory mission
- Performance evaluation (tracking error, settling time, energy)
- Comprehensive 6-panel visualization

## Running

```bash
python3 task1_rotation_math.py
```

Each exercise prints guidance on what to implement.
