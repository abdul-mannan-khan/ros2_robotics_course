#!/usr/bin/env python3
"""
Quadrotor Simulation Module
============================
Reusable quadrotor dynamics simulator for Week 6 lab exercises.

Convention: ENU/FLU (x-forward, y-left, z-up) -- ROS2 standard.
State vector: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]  (12 states)
  - (x,y,z): position in world ENU frame
  - (vx,vy,vz): velocity in world frame
  - (phi,theta,psi): roll, pitch, yaw (ZYX Euler angles)
  - (p,q,r): body angular rates

Motor layout (top view, + config):
    Motor 1 (front, +x)
    Motor 2 (left,  +y)
    Motor 3 (rear,  -x)
    Motor 4 (right, -y)
Motors 1,3 spin CW (positive torque about z-down = negative about z-up)
Motors 2,4 spin CCW (positive torque about z-up)
"""

import numpy as np
from dataclasses import dataclass, field


@dataclass
class QuadrotorParams:
    """Physical parameters for a ~1 kg quadrotor."""
    mass: float = 1.0            # kg
    arm_length: float = 0.25     # m (center to motor)
    I_xx: float = 0.0082         # kg*m^2
    I_yy: float = 0.0082         # kg*m^2
    I_zz: float = 0.0150         # kg*m^2
    k_thrust: float = 6.8e-6     # thrust coeff: T_i = k_thrust * w_i^2
    k_torque: float = 2.6e-7     # torque coeff: Q_i = k_torque * w_i^2
    g: float = 9.81              # m/s^2
    motor_min: float = 0.0       # rad/s
    motor_max: float = 838.0     # rad/s  (~max thrust per motor ~ 2.1 N)


def euler_to_rotation(phi, theta, psi):
    """
    ZYX Euler angles to rotation matrix (body-to-world).
    R = Rz(psi) * Ry(theta) * Rx(phi)
    """
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cs, ss = np.cos(psi), np.sin(psi)

    R = np.array([
        [ct*cs, cs*st*sp - ss*cp, cs*st*cp + ss*sp],
        [ct*ss, ss*st*sp + cs*cp, ss*st*cp - cs*sp],
        [-st,   ct*sp,            ct*cp            ]
    ])
    return R


def rotation_to_euler(R):
    """Extract ZYX Euler angles from rotation matrix."""
    theta = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    ct = np.cos(theta)
    if abs(ct) > 1e-6:
        phi = np.arctan2(R[2, 1], R[2, 2])
        psi = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock
        phi = 0.0
        psi = np.arctan2(-R[0, 1], R[1, 1])
    return phi, theta, psi


def quaternion_multiply(q1, q2):
    """Hamilton product of two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quaternion_conjugate(q):
    """Conjugate of quaternion [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quaternion_to_rotation(q):
    """Quaternion [w, x, y, z] to rotation matrix."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)    ],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)    ],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def euler_to_quaternion(phi, theta, psi):
    """ZYX Euler angles to quaternion [w, x, y, z]."""
    cp, sp = np.cos(phi/2), np.sin(phi/2)
    ct, st = np.cos(theta/2), np.sin(theta/2)
    cs, ss = np.cos(psi/2), np.sin(psi/2)
    return np.array([
        cp*ct*cs + sp*st*ss,
        sp*ct*cs - cp*st*ss,
        cp*st*cs + sp*ct*ss,
        cp*ct*ss - sp*st*cs,
    ])


def quaternion_to_euler(q):
    """Quaternion [w, x, y, z] to ZYX Euler angles (phi, theta, psi)."""
    w, x, y, z = q / np.linalg.norm(q)
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    phi = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2*(w*y - z*x)
    theta = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    psi = np.arctan2(siny_cosp, cosy_cosp)
    return phi, theta, psi


def quaternion_rotate_vector(q, v):
    """Rotate vector v by quaternion q."""
    qv = np.array([0.0, v[0], v[1], v[2]])
    rotated = quaternion_multiply(quaternion_multiply(q, qv), quaternion_conjugate(q))
    return rotated[1:]


def motor_mixing_matrix(params):
    """
    Build 4x4 mixing matrix M such that:
      [T, tau_x, tau_y, tau_z]^T = M * [w1^2, w2^2, w3^2, w4^2]^T

    Motor layout (+ config):
      1: front (+x), CW   -> positive pitch torque, negative yaw torque
      2: left  (+y), CCW  -> positive roll torque,  positive yaw torque
      3: rear  (-x), CW   -> negative pitch torque, negative yaw torque
      4: right (-y), CCW  -> negative roll torque,  positive yaw torque
    """
    kt = params.k_thrust
    kq = params.k_torque
    L = params.arm_length

    M = np.array([
        [kt,     kt,     kt,     kt    ],   # Total thrust (z-up)
        [0,      L*kt,   0,     -L*kt  ],   # tau_x (roll): motors 2,4
        [L*kt,   0,     -L*kt,  0      ],   # tau_y (pitch): motors 1,3
        [-kq,    kq,    -kq,    kq     ],   # tau_z (yaw): CW neg, CCW pos
    ])
    return M


def inverse_mixing(params):
    """
    Inverse mixing: [T, tau_x, tau_y, tau_z] -> [w1^2, w2^2, w3^2, w4^2].
    Returns the 4x4 inverse mixing matrix.
    """
    M = motor_mixing_matrix(params)
    return np.linalg.inv(M)


def quadrotor_dynamics(state, motor_speeds, params):
    """
    Compute state derivative for the 12-state quadrotor model.

    state: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    motor_speeds: [w1, w2, w3, w4] in rad/s
    params: QuadrotorParams

    Returns: state_dot (12,)
    """
    x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
    m = params.mass
    g = params.g
    Ixx, Iyy, Izz = params.I_xx, params.I_yy, params.I_zz

    # Motor forces and torques via mixing matrix
    w_sq = np.clip(motor_speeds, params.motor_min, params.motor_max) ** 2
    M = motor_mixing_matrix(params)
    wrench = M @ w_sq  # [T, tau_x, tau_y, tau_z]
    T = wrench[0]
    tau = wrench[1:]

    # Rotation matrix (body to world)
    R = euler_to_rotation(phi, theta, psi)

    # Translational dynamics in world frame
    # Thrust is along body z-axis
    thrust_world = R @ np.array([0, 0, T])
    gravity = np.array([0, 0, -m * g])
    acc = (thrust_world + gravity) / m

    # Euler angle rates from body rates
    # Using the standard transformation (valid away from gimbal lock)
    cp, sp = np.cos(phi), np.sin(phi)
    ct, tt = np.cos(theta), np.tan(theta)
    if abs(ct) < 1e-10:
        ct = 1e-10
        tt = np.tan(np.sign(theta) * (np.pi/2 - 1e-10))

    euler_dot = np.array([
        p + sp*tt*q + cp*tt*r,
        cp*q - sp*r,
        (sp/ct)*q + (cp/ct)*r,
    ])

    # Rotational dynamics (Euler's equation in body frame)
    # tau = I * omega_dot + omega x (I * omega)
    omega = np.array([p, q, r])
    I_diag = np.array([Ixx, Iyy, Izz])
    I_omega = I_diag * omega
    omega_dot = (tau - np.cross(omega, I_omega)) / I_diag

    state_dot = np.zeros(12)
    state_dot[0:3] = [vx, vy, vz]       # position dot = velocity
    state_dot[3:6] = acc                  # velocity dot = acceleration
    state_dot[6:9] = euler_dot            # euler angle rates
    state_dot[9:12] = omega_dot           # angular acceleration
    return state_dot


def rk4_step(state, motor_speeds, params, dt):
    """4th-order Runge-Kutta integration step."""
    k1 = quadrotor_dynamics(state, motor_speeds, params)
    k2 = quadrotor_dynamics(state + 0.5*dt*k1, motor_speeds, params)
    k3 = quadrotor_dynamics(state + 0.5*dt*k2, motor_speeds, params)
    k4 = quadrotor_dynamics(state + dt*k3, motor_speeds, params)
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


def simulate(state0, controller, params, dt=0.001, t_final=10.0):
    """
    Run full simulation loop.

    Args:
        state0: initial state (12,)
        controller: callable(t, state) -> motor_speeds (4,)
        params: QuadrotorParams
        dt: time step
        t_final: simulation duration

    Returns:
        times: array of time values
        states: array of states (N, 12)
        motor_history: array of motor speeds (N, 4)
    """
    N = int(t_final / dt) + 1
    times = np.linspace(0, t_final, N)
    states = np.zeros((N, 12))
    motor_history = np.zeros((N, 4))
    states[0] = state0

    for i in range(N - 1):
        motors = controller(times[i], states[i])
        motors = np.clip(motors, params.motor_min, params.motor_max)
        motor_history[i] = motors
        states[i + 1] = rk4_step(states[i], motors, params, dt)

    # Last motor command
    motor_history[-1] = motor_history[-2] if N > 1 else np.zeros(4)
    return times, states, motor_history


def hover_motor_speed(params):
    """Compute motor speed for hover (each motor provides mg/4 thrust)."""
    w_hover = np.sqrt(params.mass * params.g / (4 * params.k_thrust))
    return w_hover


def default_params():
    """Return default QuadrotorParams."""
    return QuadrotorParams()
