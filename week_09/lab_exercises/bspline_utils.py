"""
B-Spline Trajectory Utility Library for EGO-Planner
====================================================
Provides B-spline basis evaluation, uniform B-spline matrix form,
derivative computation, and trajectory fitting utilities.

Mathematical Reference:
- Cox-de Boor recursion for general B-splines
- Matrix form for uniform B-splines (degree 3)
- Uniform B-spline: velocity v_i = (Q_{i+1} - Q_i) / dt
- Uniform B-spline: acceleration a_i = (Q_{i+2} - 2*Q_{i+1} + Q_i) / dt^2
"""

import numpy as np
from scipy import linalg


# ---------------------------------------------------------------------------
# General B-spline evaluation (Cox-de Boor)
# ---------------------------------------------------------------------------

def bspline_basis(i, p, t, knots):
    """
    Evaluate the B-spline basis function N_{i,p}(t) using Cox-de Boor recursion.

    Parameters
    ----------
    i : int
        Basis function index.
    p : int
        Degree.
    t : float
        Parameter value.
    knots : array-like
        Knot vector.

    Returns
    -------
    float
        Value of N_{i,p}(t).
    """
    if p == 0:
        return 1.0 if knots[i] <= t < knots[i + 1] else 0.0

    denom1 = knots[i + p] - knots[i]
    denom2 = knots[i + p + 1] - knots[i + 1]

    c1 = 0.0 if denom1 == 0.0 else (t - knots[i]) / denom1 * bspline_basis(i, p - 1, t, knots)
    c2 = 0.0 if denom2 == 0.0 else (knots[i + p + 1] - t) / denom2 * bspline_basis(i + 1, p - 1, t, knots)

    return c1 + c2


def evaluate_bspline(control_points, knots, degree, t_eval):
    """
    Evaluate a B-spline curve at parameter values t_eval.

    Parameters
    ----------
    control_points : ndarray, shape (n, d)
        Control points.
    knots : ndarray
        Knot vector of length n + degree + 1.
    degree : int
        B-spline degree.
    t_eval : ndarray
        Parameter values to evaluate.

    Returns
    -------
    ndarray, shape (len(t_eval), d)
        Evaluated curve points.
    """
    control_points = np.asarray(control_points, dtype=float)
    knots = np.asarray(knots, dtype=float)
    n = len(control_points)
    d = control_points.shape[1] if control_points.ndim > 1 else 1
    if control_points.ndim == 1:
        control_points = control_points[:, None]

    result = np.zeros((len(t_eval), d))
    for idx, t in enumerate(t_eval):
        # Handle endpoint
        if t >= knots[-1]:
            t = knots[-1] - 1e-10
        for i in range(n):
            result[idx] += bspline_basis(i, degree, t, knots) * control_points[i]
    return result


def bspline_derivative(control_points, knots, degree, order=1):
    """
    Compute control points of the derivative B-spline.

    For a B-spline of degree p with control points Q_i and knot vector,
    the derivative has degree p-1 and control points:
        Q'_i = p * (Q_{i+1} - Q_i) / (t_{i+p+1} - t_{i+1})

    Parameters
    ----------
    control_points : ndarray
    knots : ndarray
    degree : int
    order : int
        Derivative order.

    Returns
    -------
    new_cp : ndarray
        Derivative control points.
    new_knots : ndarray
        Derivative knot vector.
    new_degree : int
    """
    cp = np.asarray(control_points, dtype=float)
    kn = np.asarray(knots, dtype=float)
    p = degree

    for _ in range(order):
        n = len(cp)
        new_cp = np.zeros((n - 1, cp.shape[1] if cp.ndim > 1 else 1))
        if cp.ndim == 1:
            cp = cp[:, None]
        for i in range(n - 1):
            denom = kn[i + p + 1] - kn[i + 1]
            if denom == 0:
                new_cp[i] = 0.0
            else:
                new_cp[i] = p * (cp[i + 1] - cp[i]) / denom
        cp = new_cp
        kn = kn[1:-1]
        p = p - 1

    return cp, kn, p


# ---------------------------------------------------------------------------
# Uniform B-spline utilities (used by EGO-Planner)
# ---------------------------------------------------------------------------

def uniform_bspline_matrix(p=3):
    """
    Return the matrix form M for a uniform B-spline of given degree.

    For degree 3 (cubic):
        M = (1/6) * [[-1, 3, -3, 1],
                     [ 3,-6,  3, 0],
                     [-3, 0,  3, 0],
                     [ 1, 4,  1, 0]]

    Parameters
    ----------
    p : int
        Degree (currently supports 3).

    Returns
    -------
    ndarray, shape (p+1, p+1)
    """
    if p == 3:
        return (1.0 / 6.0) * np.array([
            [-1,  3, -3, 1],
            [ 3, -6,  3, 0],
            [-3,  0,  3, 0],
            [ 1,  4,  1, 0]
        ], dtype=float)
    elif p == 2:
        return 0.5 * np.array([
            [ 1, -2, 1],
            [-2,  2, 0],
            [ 1,  1, 0]
        ], dtype=float)
    else:
        raise NotImplementedError(f"Matrix form not implemented for degree {p}")


def evaluate_uniform_bspline(control_points, dt, t_eval):
    """
    Evaluate a uniform cubic B-spline at given parameter values.

    For a uniform B-spline with knot spacing dt, the curve in segment j
    (between control points Q_j ... Q_{j+3}) at local parameter s in [0,1) is:
        C(s) = [s^3, s^2, s, 1] * M * [Q_j, Q_{j+1}, Q_{j+2}, Q_{j+3}]^T

    Parameters
    ----------
    control_points : ndarray, shape (n, d)
        Control points.
    dt : float
        Uniform knot spacing (time interval between control points).
    t_eval : ndarray
        Time values to evaluate (from 0 to (n-3)*dt).

    Returns
    -------
    ndarray, shape (len(t_eval), d)
    """
    cp = np.asarray(control_points, dtype=float)
    n = len(cp)
    d = cp.shape[1] if cp.ndim > 1 else 1
    if cp.ndim == 1:
        cp = cp[:, None]

    M = uniform_bspline_matrix(3)
    n_segments = n - 3  # number of valid segments
    t_max = n_segments * dt

    result = np.zeros((len(t_eval), d))
    for idx, t in enumerate(t_eval):
        t_clamped = np.clip(t, 0, t_max - 1e-10)
        j = int(t_clamped / dt)
        j = min(j, n_segments - 1)
        s = (t_clamped - j * dt) / dt
        s_vec = np.array([s**3, s**2, s, 1.0])
        coeffs = s_vec @ M  # shape (4,)
        result[idx] = coeffs @ cp[j:j+4]

    return result if d > 1 else result.squeeze()


def uniform_bspline_velocity(control_points, dt):
    """
    Compute velocity control points for a uniform cubic B-spline.

    v_i = (Q_{i+1} - Q_i) / dt

    Parameters
    ----------
    control_points : ndarray, shape (n, d)
    dt : float

    Returns
    -------
    ndarray, shape (n-1, d)
        Velocity control points.
    """
    cp = np.asarray(control_points, dtype=float)
    return np.diff(cp, axis=0) / dt


def uniform_bspline_acceleration(control_points, dt):
    """
    Compute acceleration control points for a uniform cubic B-spline.

    a_i = (Q_{i+2} - 2*Q_{i+1} + Q_i) / dt^2

    Parameters
    ----------
    control_points : ndarray, shape (n, d)
    dt : float

    Returns
    -------
    ndarray, shape (n-2, d)
        Acceleration control points.
    """
    cp = np.asarray(control_points, dtype=float)
    return (cp[2:] - 2.0 * cp[1:-1] + cp[:-2]) / (dt ** 2)


def uniform_bspline_jerk(control_points, dt):
    """
    Compute jerk control points for a uniform cubic B-spline.

    j_i = (Q_{i+3} - 3*Q_{i+2} + 3*Q_{i+1} - Q_i) / dt^3

    Parameters
    ----------
    control_points : ndarray, shape (n, d)
    dt : float

    Returns
    -------
    ndarray, shape (n-3, d)
    """
    cp = np.asarray(control_points, dtype=float)
    return (cp[3:] - 3*cp[2:-1] + 3*cp[1:-2] - cp[:-3]) / (dt ** 3)


def evaluate_uniform_bspline_derivative(control_points, dt, t_eval, order=1):
    """
    Evaluate the order-th derivative of a uniform cubic B-spline.

    Parameters
    ----------
    control_points : ndarray, shape (n, d)
    dt : float
    t_eval : ndarray
    order : int (1=velocity, 2=acceleration)

    Returns
    -------
    ndarray
    """
    cp = np.asarray(control_points, dtype=float)
    if order == 1:
        vel_cp = uniform_bspline_velocity(cp, dt)
        # Velocity spline is degree 2
        return _evaluate_uniform_bspline_degree2(vel_cp, dt, t_eval)
    elif order == 2:
        acc_cp = uniform_bspline_acceleration(cp, dt)
        # Acceleration spline is degree 1
        return _evaluate_uniform_bspline_degree1(acc_cp, dt, t_eval)
    else:
        raise NotImplementedError("Only order 1 and 2 supported")


def _evaluate_uniform_bspline_degree2(control_points, dt, t_eval):
    """Evaluate uniform quadratic B-spline."""
    cp = np.asarray(control_points, dtype=float)
    n = len(cp)
    d = cp.shape[1] if cp.ndim > 1 else 1
    if cp.ndim == 1:
        cp = cp[:, None]

    M = uniform_bspline_matrix(2)
    n_seg = n - 2
    t_max = n_seg * dt

    result = np.zeros((len(t_eval), d))
    for idx, t in enumerate(t_eval):
        tc = np.clip(t, 0, t_max - 1e-10)
        j = min(int(tc / dt), n_seg - 1)
        s = (tc - j * dt) / dt
        s_vec = np.array([s**2, s, 1.0])
        coeffs = s_vec @ M
        result[idx] = coeffs @ cp[j:j+3]
    return result


def _evaluate_uniform_bspline_degree1(control_points, dt, t_eval):
    """Evaluate uniform linear B-spline."""
    cp = np.asarray(control_points, dtype=float)
    n = len(cp)
    d = cp.shape[1] if cp.ndim > 1 else 1
    if cp.ndim == 1:
        cp = cp[:, None]

    n_seg = n - 1
    t_max = n_seg * dt

    result = np.zeros((len(t_eval), d))
    for idx, t in enumerate(t_eval):
        tc = np.clip(t, 0, t_max - 1e-10)
        j = min(int(tc / dt), n_seg - 1)
        s = (tc - j * dt) / dt
        result[idx] = (1 - s) * cp[j] + s * cp[j + 1]
    return result


# ---------------------------------------------------------------------------
# Fitting
# ---------------------------------------------------------------------------

def create_uniform_knots(n_control_points, degree):
    """
    Create a clamped uniform knot vector.

    Parameters
    ----------
    n_control_points : int
    degree : int

    Returns
    -------
    ndarray
        Knot vector of length n_control_points + degree + 1.
    """
    n = n_control_points
    p = degree
    m = n + p + 1
    knots = np.zeros(m)
    for i in range(m):
        if i <= p:
            knots[i] = 0.0
        elif i >= m - p - 1:
            knots[i] = 1.0
        else:
            knots[i] = (i - p) / (n - p)
    return knots


def fit_bspline_to_waypoints(waypoints, n_control_points, degree=3):
    """
    Fit a B-spline to waypoints using least-squares.

    Parameters
    ----------
    waypoints : ndarray, shape (m, d)
        Points to fit.
    n_control_points : int
        Number of control points.
    degree : int

    Returns
    -------
    control_points : ndarray, shape (n_control_points, d)
    knots : ndarray
    """
    waypoints = np.asarray(waypoints, dtype=float)
    m, d = waypoints.shape
    knots = create_uniform_knots(n_control_points, degree)

    # Parameter values for each waypoint (chord-length parameterization)
    dists = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    total = np.sum(dists)
    if total < 1e-12:
        t_params = np.linspace(0, 1, m)
    else:
        t_params = np.zeros(m)
        t_params[1:] = np.cumsum(dists) / total
    t_params[-1] = 1.0 - 1e-10  # avoid endpoint issue

    # Build basis matrix
    N = np.zeros((m, n_control_points))
    for row, t in enumerate(t_params):
        for col in range(n_control_points):
            N[row, col] = bspline_basis(col, degree, t, knots)

    # Least-squares solve: N @ P = W
    control_points, _, _, _ = np.linalg.lstsq(N, waypoints, rcond=None)

    return control_points, knots


def waypoints_to_uniform_bspline(waypoints, n_control_points=None, dt=1.0):
    """
    Convert waypoints to uniform cubic B-spline control points.

    Simple approach: use waypoints as initial guess then fit via least squares
    in the uniform B-spline formulation.

    Parameters
    ----------
    waypoints : ndarray, shape (m, d)
    n_control_points : int or None
        If None, uses m + 4.
    dt : float

    Returns
    -------
    control_points : ndarray, shape (n_control_points, d)
    dt : float
    """
    waypoints = np.asarray(waypoints, dtype=float)
    m, d = waypoints.shape

    if n_control_points is None:
        n_control_points = m + 4

    n_seg = n_control_points - 3
    M = uniform_bspline_matrix(3)

    # Distribute waypoints across segments
    t_params = np.linspace(0, n_seg * dt - 1e-10, m)

    # Build basis matrix
    N = np.zeros((m, n_control_points))
    for row, t in enumerate(t_params):
        j = min(int(t / dt), n_seg - 1)
        s = (t - j * dt) / dt
        s_vec = np.array([s**3, s**2, s, 1.0])
        coeffs = s_vec @ M
        N[row, j:j+4] = coeffs

    control_points, _, _, _ = np.linalg.lstsq(N, waypoints, rcond=None)
    return control_points, dt
