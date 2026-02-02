#!/usr/bin/env python3
"""
Task 1: B-Spline Basics
========================
Learn the fundamentals of B-spline curves: basis functions, evaluation,
local support property, and convex hull property.

Exercises:
1. Implement basis function evaluation
2. Create uniform knot vectors
3. Evaluate B-spline curves
4. Demonstrate local support (moving one CP only affects nearby curve)
5. Demonstrate convex hull property

Saves: task1_bspline_basics.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bspline_utils import bspline_basis, create_uniform_knots, evaluate_bspline


def compute_basis_functions(degree, knots, t_values):
    """
    Evaluate all B-spline basis functions N_{i,p}(t) for the given knot vector.

    Parameters
    ----------
    degree : int
    knots : ndarray
    t_values : ndarray

    Returns
    -------
    ndarray, shape (n_basis, len(t_values))
        Each row is one basis function evaluated at all t_values.
    """
    # TODO: Compute the number of basis functions (n = len(knots) - degree - 1)
    # TODO: For each basis function i, evaluate at all t_values using bspline_basis()
    # TODO: Return as 2D array
    raise NotImplementedError("Implement compute_basis_functions")


def create_uniform_knots_clamped(n_control_points, degree):
    """
    Generate a clamped uniform knot vector.

    A clamped knot vector has degree+1 repeated knots at each end,
    ensuring the curve passes through the first and last control points.

    Parameters
    ----------
    n_control_points : int
    degree : int

    Returns
    -------
    ndarray
    """
    # TODO: Use create_uniform_knots from bspline_utils
    # HINT: knot vector length = n_control_points + degree + 1
    raise NotImplementedError("Implement create_uniform_knots_clamped")


def evaluate_curve(control_points, degree, knots, t_values):
    """
    Evaluate the B-spline curve C(t) = sum_i N_{i,p}(t) * P_i.

    Parameters
    ----------
    control_points : ndarray, shape (n, d)
    degree : int
    knots : ndarray
    t_values : ndarray

    Returns
    -------
    ndarray, shape (len(t_values), d)
    """
    # TODO: Use evaluate_bspline from bspline_utils
    raise NotImplementedError("Implement evaluate_curve")


def demonstrate_local_support(control_points):
    """
    Demonstrate local support: perturb one control point and show that
    only a local portion of the curve changes.

    Parameters
    ----------
    control_points : ndarray, shape (n, 2)

    Returns
    -------
    original_curve : ndarray
    modified_curve : ndarray
    modified_cp : ndarray
    """
    # TODO: Create knot vector and evaluate original curve
    # TODO: Copy control_points, perturb the middle control point
    # TODO: Evaluate modified curve
    # TODO: Return both curves and modified control points
    raise NotImplementedError("Implement demonstrate_local_support")


def demonstrate_convex_hull(control_points):
    """
    Show that the B-spline curve lies within the convex hull of its
    control points.

    Parameters
    ----------
    control_points : ndarray, shape (n, 2)

    Returns
    -------
    curve : ndarray
    hull_vertices : ndarray
    """
    # TODO: Evaluate the curve
    # TODO: Compute the convex hull of control_points (use scipy.spatial.ConvexHull)
    # TODO: Return curve and hull vertices
    raise NotImplementedError("Implement demonstrate_convex_hull")


def main():
    """Main function: create all visualizations."""
    print("=" * 60)
    print("Task 1: B-Spline Basics")
    print("=" * 60)

    # TODO: Create a figure with 4 subplots:
    # 1. Basis functions for degree 3 with 7 control points
    # 2. 2D B-spline curve with control polygon
    # 3. Local support demonstration
    # 4. 3D B-spline curve

    # HINT: Use the helper functions above
    # HINT: Save figure as 'task1_bspline_basics.png'

    print("Task 1 not yet implemented. Complete the TODOs!")


if __name__ == '__main__':
    main()
