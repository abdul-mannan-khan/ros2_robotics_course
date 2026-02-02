# Quadrotor Dynamics & Control References

## Textbooks and Papers

1. **Quan, Q. (2017).** *Introduction to Multicopter Design and Control.* Springer.
   - Comprehensive treatment of quadrotor modeling and control.

2. **Mahony, R., Kumar, V., & Corke, P. (2012).** Multirotor Aerial Vehicles: Modeling,
   Estimation, and Control of Quadrotor. *IEEE Robotics & Automation Magazine, 19*(3), 20-32.
   - Excellent overview of quadrotor dynamics and control architectures.

3. **Beard, R. W., & McLain, T. W. (2012).** *Small Unmanned Aircraft: Theory and Practice.*
   Princeton University Press.
   - Covers fixed-wing and rotary-wing UAV dynamics.

## Rotation Representations

4. **Diebel, J. (2006).** Representing Attitude: Euler Angles, Unit Quaternions, and
   Rotation Vectors. Stanford University Technical Report.
   - Clear derivation of all rotation representations and their relationships.

5. **Sola, J. (2017).** Quaternion kinematics for the error-state Kalman filter.
   arXiv:1711.02508.
   - Rigorous quaternion math for state estimation.

## Control

6. **Mellinger, D., & Kumar, V. (2011).** Minimum Snap Trajectory Generation and Control
   for Quadrotors. *IEEE ICRA.*
   - Differential flatness and trajectory generation.

7. **Lee, T., Leok, M., & McClamroch, N. H. (2010).** Geometric Tracking Control of a
   Quadrotor UAV on SE(3). *IEEE CDC.*
   - Geometric control on the special Euclidean group.

## Attitude Estimation

8. **Mahony, R., Hamel, T., & Pflimlin, J. M. (2008).** Nonlinear Complementary Filters
   on the Special Orthogonal Group. *IEEE TAC, 53*(5), 1203-1218.
   - Theoretical foundation for complementary filters.

9. **Madgwick, S. O. H. (2010).** An efficient orientation filter for inertial and
   inertial/magnetic sensor arrays. University of Bristol Technical Report.
   - Widely used gradient descent orientation filter.

## Conventions

- **ENU (East-North-Up):** Standard ROS2 world frame convention.
- **FLU (Forward-Left-Up):** Standard ROS2 body frame convention.
- **ZYX Euler Angles:** Yaw (psi) -> Pitch (theta) -> Roll (phi) intrinsic rotation order.
- **Quaternion format:** [w, x, y, z] (scalar-first, Hamilton convention).
