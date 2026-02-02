# Week 9 References

## Papers
1. **EGO-Planner**: Zhou et al., "EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors," IEEE RA-L, 2021.
2. **EGO-Swarm**: Zhou et al., "EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments," ICRA 2021.
3. **Minimum Snap**: Mellinger and Kumar, "Minimum snap trajectory generation and control for quadrotors," ICRA 2011.

## B-Spline Theory
- de Boor, C. "A Practical Guide to Splines," Springer, 2001.
- Piegl, L. and Tiller, W. "The NURBS Book," Springer, 1997.

## Key Concepts
- **Uniform B-spline**: Equal knot spacing enables efficient matrix-form evaluation
- **ESDF-free**: Avoids expensive Euclidean Signed Distance Field computation
- **Convex hull property**: Bounding control points bounds the curve
- **Local support**: Moving one control point only affects nearby curve segments
