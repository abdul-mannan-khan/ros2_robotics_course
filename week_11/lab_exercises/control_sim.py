#!/usr/bin/env python3
"""
Week 11 - Control Systems Simulation Module
============================================
Provides simulation classes for PID, LQR, and MPC control design.

Course: TC70045E Robotics & Drone Engineering
Instructor: Dr. Abdul Manan Khan, University of West London
"""

import numpy as np
from scipy import linalg, signal, optimize
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# =============================================================================
# Linear System
# =============================================================================
class LinearSystem:
    """Continuous-time linear state-space model: dx/dt = Ax + Bu, y = Cx + Du."""

    def __init__(self, A, B, C=None, D=None):
        self.A = np.atleast_2d(np.array(A, dtype=float))
        self.B = np.atleast_2d(np.array(B, dtype=float))
        self.n = self.A.shape[0]  # number of states
        self.m = self.B.shape[1]  # number of inputs
        if C is None:
            self.C = np.eye(self.n)
        else:
            self.C = np.atleast_2d(np.array(C, dtype=float))
        if D is None:
            self.D = np.zeros((self.C.shape[0], self.m))
        else:
            self.D = np.atleast_2d(np.array(D, dtype=float))
        self.p = self.C.shape[0]  # number of outputs

    def simulate_step(self, x, u, dt):
        """One Euler step."""
        x = np.array(x, dtype=float).flatten()
        u = np.array(u, dtype=float).flatten()
        dx = self.A @ x + self.B @ u
        x_next = x + dx * dt
        y = self.C @ x_next + self.D @ u
        return x_next, y

    def discretize(self, dt):
        """Convert to discrete-time using matrix exponential (ZOH)."""
        n = self.n
        # Build augmented matrix
        M = np.zeros((n + self.m, n + self.m))
        M[:n, :n] = self.A * dt
        M[:n, n:] = self.B * dt
        eM = linalg.expm(M)
        Ad = eM[:n, :n]
        Bd = eM[:n, n:]
        return Ad, Bd, self.C.copy(), self.D.copy()

    def controllability_matrix(self):
        """Return controllability matrix [B, AB, A^2B, ..., A^(n-1)B]."""
        C_mat = self.B.copy()
        Ak = np.eye(self.n)
        for _ in range(1, self.n):
            Ak = Ak @ self.A
            C_mat = np.hstack([C_mat, Ak @ self.B])
        return C_mat

    def is_controllable(self):
        return np.linalg.matrix_rank(self.controllability_matrix()) == self.n

    def observability_matrix(self):
        """Return observability matrix."""
        O_mat = self.C.copy()
        Ak = np.eye(self.n)
        for _ in range(1, self.n):
            Ak = Ak @ self.A
            O_mat = np.vstack([O_mat, self.C @ Ak])
        return O_mat

    def is_observable(self):
        return np.linalg.matrix_rank(self.observability_matrix()) == self.n

    def eigenvalues(self):
        return np.linalg.eigvals(self.A)

    def is_stable(self):
        return np.all(np.real(self.eigenvalues()) < 0)


# =============================================================================
# Quadrotor Linearized Model
# =============================================================================
class QuadrotorLinearized(LinearSystem):
    """
    Linearized quadrotor model about hover.
    State: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r] (12 states)
    Input: [delta_T, delta_phi, delta_theta, delta_psi] (4 inputs)
    Simplified: decoupled altitude + attitude model.
    """

    def __init__(self, mass=1.5, g=9.81, Ixx=0.03, Iyy=0.03, Izz=0.06, arm_length=0.25):
        self.mass = mass
        self.g = g
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.L = arm_length

        # 12-state model
        n = 12
        m = 4
        A = np.zeros((n, n))
        B = np.zeros((n, m))

        # Position derivatives = velocity
        A[0, 3] = 1.0  # dx/dt = vx
        A[1, 4] = 1.0  # dy/dt = vy
        A[2, 5] = 1.0  # dz/dt = vz

        # Velocity derivatives (linearized gravity coupling)
        A[3, 7] = -g   # dvx/dt ~ -g * theta
        A[4, 6] = g    # dvy/dt ~  g * phi
        # dvz/dt depends on thrust input

        # Angle derivatives = angular rates
        A[6, 9] = 1.0   # dphi/dt = p
        A[7, 10] = 1.0  # dtheta/dt = q
        A[8, 11] = 1.0  # dpsi/dt = r

        # Input matrix
        B[5, 0] = 1.0 / mass          # thrust -> vz_dot
        B[9, 1] = self.L / Ixx        # roll torque -> p_dot
        B[10, 2] = self.L / Iyy       # pitch torque -> q_dot
        B[11, 3] = 1.0 / Izz          # yaw torque -> r_dot

        C = np.eye(n)
        super().__init__(A, B, C)


# =============================================================================
# PID Controller
# =============================================================================
class PIDController:
    """Standard PID controller with anti-windup and derivative filter."""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.01,
                 output_limits=(-np.inf, np.inf),
                 integrator_limits=(-100.0, 100.0),
                 derivative_filter=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limits = output_limits
        self.integrator_limits = integrator_limits
        self.alpha = derivative_filter  # low-pass filter coefficient

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

    def compute(self, error):
        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup clamping
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, *self.integrator_limits)
        i_term = self.ki * self.integral

        # Derivative with low-pass filter
        raw_derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        filtered_derivative = self.alpha * raw_derivative + (1 - self.alpha) * self.prev_derivative
        d_term = self.kd * filtered_derivative

        self.prev_error = error
        self.prev_derivative = filtered_derivative

        output = p_term + i_term + d_term
        output = np.clip(output, *self.output_limits)
        return output


# =============================================================================
# LQR Controller
# =============================================================================
class LQRController:
    """Linear Quadratic Regulator using continuous algebraic Riccati equation."""

    def __init__(self, A, B, Q=None, R=None):
        self.A = np.atleast_2d(np.array(A, dtype=float))
        self.B = np.atleast_2d(np.array(B, dtype=float))
        n = self.A.shape[0]
        m = self.B.shape[1]

        if Q is None:
            self.Q = np.eye(n)
        else:
            self.Q = np.atleast_2d(np.array(Q, dtype=float))

        if R is None:
            self.R = np.eye(m)
        else:
            self.R = np.atleast_2d(np.array(R, dtype=float))

        # Solve continuous algebraic Riccati equation
        self.P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.solve(self.R, self.B.T @ self.P)

    def compute(self, x, x_ref=None):
        """Compute control input u = -K(x - x_ref)."""
        x = np.array(x, dtype=float).flatten()
        if x_ref is not None:
            x_ref = np.array(x_ref, dtype=float).flatten()
            error = x - x_ref
        else:
            error = x
        u = -self.K @ error
        return u

    def closed_loop_eigenvalues(self):
        A_cl = self.A - self.B @ self.K
        return np.linalg.eigvals(A_cl)


# =============================================================================
# MPC Controller
# =============================================================================
class MPCController:
    """
    Model Predictive Controller using scipy.optimize.minimize.
    Uses a discrete-time linear model for prediction.
    """

    def __init__(self, Ad, Bd, Q, R, N=10,
                 u_min=None, u_max=None, x_min=None, x_max=None):
        self.Ad = np.atleast_2d(np.array(Ad, dtype=float))
        self.Bd = np.atleast_2d(np.array(Bd, dtype=float))
        self.Q = np.atleast_2d(np.array(Q, dtype=float))
        self.R = np.atleast_2d(np.array(R, dtype=float))
        self.N = N  # prediction horizon
        self.n = self.Ad.shape[0]
        self.m = self.Bd.shape[1]

        # Constraints
        self.u_min = u_min if u_min is not None else -np.inf * np.ones(self.m)
        self.u_max = u_max if u_max is not None else np.inf * np.ones(self.m)
        self.x_min = x_min if x_min is not None else -np.inf * np.ones(self.n)
        self.x_max = x_max if x_max is not None else np.inf * np.ones(self.n)

        self.u_prev = np.zeros(self.N * self.m)

    def _predict(self, x0, u_seq):
        """Predict state trajectory given initial state and input sequence."""
        states = [x0.copy()]
        x = x0.copy()
        for k in range(self.N):
            uk = u_seq[k * self.m:(k + 1) * self.m]
            x = self.Ad @ x + self.Bd @ uk
            states.append(x.copy())
        return states

    def _cost(self, u_seq, x0, x_ref):
        """Compute MPC cost: sum of stage costs + terminal cost."""
        states = self._predict(x0, u_seq)
        cost = 0.0
        for k in range(self.N):
            dx = states[k + 1] - x_ref
            uk = u_seq[k * self.m:(k + 1) * self.m]
            cost += dx @ self.Q @ dx + uk @ self.R @ uk
        # Terminal cost (weighted more)
        dx_terminal = states[-1] - x_ref
        cost += dx_terminal @ (self.Q * 10) @ dx_terminal
        return cost

    def _constraints(self, u_seq, x0, x_ref):
        """Return constraint violations as inequality constraints (>= 0)."""
        states = self._predict(x0, u_seq)
        cons = []
        for k in range(self.N):
            uk = u_seq[k * self.m:(k + 1) * self.m]
            # Input constraints: u_min <= u <= u_max
            cons.extend(uk - self.u_min)
            cons.extend(self.u_max - uk)
            # State constraints: x_min <= x <= x_max
            xk = states[k + 1]
            cons.extend(xk - self.x_min)
            cons.extend(self.x_max - xk)
        return np.array(cons)

    def compute(self, x, x_ref=None):
        """Solve MPC optimization and return first control input."""
        x = np.array(x, dtype=float).flatten()
        if x_ref is None:
            x_ref = np.zeros(self.n)
        else:
            x_ref = np.array(x_ref, dtype=float).flatten()

        # Bounds on decision variables (input sequence)
        bounds = []
        for k in range(self.N):
            for j in range(self.m):
                bounds.append((self.u_min[j], self.u_max[j]))

        # Solve using SLSQP
        constraints = {
            'type': 'ineq',
            'fun': lambda u: self._constraints(u, x, x_ref)
        }

        result = optimize.minimize(
            self._cost,
            self.u_prev,
            args=(x, x_ref),
            method='SLSQP',
            bounds=bounds if not np.any(np.isinf(self.u_min)) else None,
            constraints=constraints,
            options={'maxiter': 100, 'ftol': 1e-6}
        )

        if result.success:
            self.u_prev = result.x.copy()
        else:
            # Warm start shift
            self.u_prev[:-self.m] = self.u_prev[self.m:]

        u_optimal = result.x[:self.m]
        return u_optimal


# =============================================================================
# Utility Functions
# =============================================================================
def rk4_step(f, x, u, dt):
    """Fourth-order Runge-Kutta integration step.
    f(x, u) returns dx/dt.
    """
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate_system(system, controller, x0, x_ref, dt, T, disturbance_fn=None):
    """
    Simulate closed-loop system.

    Parameters:
        system: LinearSystem instance
        controller: object with compute(x, x_ref) -> u
        x0: initial state
        x_ref: reference state (or callable(t) -> x_ref)
        dt: time step
        T: total simulation time
        disturbance_fn: optional callable(t) -> disturbance vector

    Returns:
        t_hist, x_hist, u_hist, y_hist
    """
    steps = int(T / dt)
    x = np.array(x0, dtype=float).flatten()
    n = system.n
    m = system.m

    t_hist = np.zeros(steps)
    x_hist = np.zeros((steps, n))
    u_hist = np.zeros((steps, m))
    y_hist = np.zeros((steps, system.p))

    for k in range(steps):
        t = k * dt
        t_hist[k] = t
        x_hist[k] = x

        # Get reference
        if callable(x_ref):
            ref = x_ref(t)
        else:
            ref = np.array(x_ref, dtype=float).flatten()

        # Compute control
        u = controller.compute(x, ref)
        u = np.array(u, dtype=float).flatten()
        if u.shape[0] < m:
            u = np.pad(u, (0, m - u.shape[0]))
        u_hist[k] = u[:m]

        # Apply disturbance
        if disturbance_fn is not None:
            d = disturbance_fn(t)
        else:
            d = np.zeros(n)

        # Simulate one step (RK4)
        def dynamics(xx, uu):
            return system.A @ xx + system.B @ uu + d

        x = rk4_step(dynamics, x, u[:m], dt)
        y = system.C @ x + system.D @ u[:m]
        y_hist[k] = y

    return t_hist, x_hist, u_hist, y_hist


def plot_response(t, x, u=None, x_ref=None, state_labels=None, input_labels=None,
                  title="System Response", filename=None):
    """Plot state and input trajectories."""
    n_states = x.shape[1]
    n_plots = n_states
    if u is not None:
        n_inputs = u.shape[1]
        n_plots += n_inputs

    cols = 2
    rows = (n_plots + 1) // 2

    fig, axes = plt.subplots(rows, cols, figsize=(14, 3 * rows))
    axes = axes.flatten()

    for i in range(n_states):
        ax = axes[i]
        label = state_labels[i] if state_labels and i < len(state_labels) else f"x[{i}]"
        ax.plot(t, x[:, i], 'b-', linewidth=1.5, label=label)
        if x_ref is not None:
            if callable(x_ref):
                ref_vals = np.array([x_ref(ti) for ti in t])
                if ref_vals.ndim == 2 and i < ref_vals.shape[1]:
                    ax.plot(t, ref_vals[:, i], 'r--', linewidth=1.0, label='Reference')
            else:
                ref_arr = np.array(x_ref).flatten()
                if i < len(ref_arr):
                    ax.axhline(y=ref_arr[i], color='r', linestyle='--', linewidth=1.0, label='Reference')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(label)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    if u is not None:
        for j in range(n_inputs):
            idx = n_states + j
            if idx < len(axes):
                ax = axes[idx]
                label = input_labels[j] if input_labels and j < len(input_labels) else f"u[{j}]"
                ax.plot(t, u[:, j], 'g-', linewidth=1.5, label=label)
                ax.set_xlabel('Time [s]')
                ax.set_ylabel(label)
                ax.legend(fontsize=8)
                ax.grid(True, alpha=0.3)

    # Hide unused axes
    for idx in range(n_plots, len(axes)):
        axes[idx].set_visible(False)

    fig.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()

    if filename:
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"[INFO] Plot saved to {filename}")
    plt.close(fig)
    return fig


def pole_placement(A, B, desired_poles):
    """Compute state feedback gain K via pole placement (Ackermann's formula).
    Uses scipy.signal.place_poles.
    """
    A = np.atleast_2d(np.array(A, dtype=float))
    B = np.atleast_2d(np.array(B, dtype=float))
    desired_poles = np.array(desired_poles, dtype=complex)

    result = signal.place_poles(A, B, desired_poles)
    return result.gain_matrix


def main():
    """Demonstrate the control_sim module."""
    print("=" * 70)
    print("Week 11 - Control Systems Simulation Module")
    print("=" * 70)

    # Demo: Simple second-order system
    print("\n--- Demo: Second-order system with PID control ---")
    A = np.array([[0, 1], [0, -2]])
    B = np.array([[0], [1]])
    sys = LinearSystem(A, B)
    print(f"System: n={sys.n} states, m={sys.m} inputs")
    print(f"Eigenvalues: {sys.eigenvalues()}")
    print(f"Controllable: {sys.is_controllable()}")
    print(f"Observable: {sys.is_observable()}")

    pid = PIDController(kp=10, ki=5, kd=2, dt=0.01)

    class PIDWrapper:
        def __init__(self, pid_ctrl, state_idx=0):
            self.pid = pid_ctrl
            self.idx = state_idx
        def compute(self, x, x_ref):
            error = x_ref[self.idx] - x[self.idx]
            return np.array([self.pid.compute(error)])

    ctrl = PIDWrapper(pid)
    t, x, u, y = simulate_system(sys, ctrl, [0, 0], [1, 0], 0.01, 5.0)
    plot_response(t, x, u, [1, 0],
                  state_labels=['Position', 'Velocity'],
                  input_labels=['Force'],
                  title='PID Control Demo',
                  filename='control_sim_demo.png')

    # Demo: LQR
    print("\n--- Demo: LQR control ---")
    lqr = LQRController(A, B, Q=np.diag([10, 1]), R=np.array([[1]]))
    print(f"LQR gain K: {lqr.K}")
    print(f"Closed-loop eigenvalues: {lqr.closed_loop_eigenvalues()}")

    t2, x2, u2, y2 = simulate_system(sys, lqr, [1, 0], [0, 0], 0.01, 5.0)
    plot_response(t2, x2, u2, [0, 0],
                  state_labels=['Position', 'Velocity'],
                  input_labels=['Force'],
                  title='LQR Control Demo',
                  filename='control_sim_lqr_demo.png')

    # Demo: Quadrotor model
    print("\n--- Demo: Quadrotor linearized model ---")
    quad = QuadrotorLinearized()
    print(f"Quadrotor: {quad.n} states, {quad.m} inputs")
    print(f"Controllable: {quad.is_controllable()}")
    eigs = quad.eigenvalues()
    print(f"Open-loop eigenvalues (real parts): {np.real(eigs)}")

    print("\n[OK] Control simulation module loaded successfully.")


if __name__ == '__main__':
    main()
