#!/usr/bin/env python3
"""
Week 10 - Task 6: Optical Flow Estimation
Implement Lucas-Kanade sparse optical flow and motion estimation.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


def compute_image_gradients(image):
    """
    Compute spatial and temporal image gradients.

    Args:
        image: 2D grayscale image

    Returns:
        Ix, Iy: spatial gradients (same shape as image)
    """
    # TODO: Use finite differences or Sobel-like kernels
    raise NotImplementedError("Implement compute_image_gradients")


def lucas_kanade(frame1, frame2, points, window_size=15):
    """
    Lucas-Kanade sparse optical flow.

    Args:
        frame1, frame2: 2D grayscale images
        points: Nx2 array of points to track (x, y)
        window_size: size of the local window

    Returns:
        flow: Nx2 flow vectors (dx, dy)
        status: N boolean array (True if flow was computed successfully)
    """
    # TODO: For each point:
    # 1. Extract local window from frame1 and frame2
    # 2. Compute Ix, Iy, It in the window
    # 3. Build A = [[Ix1, Iy1], [Ix2, Iy2], ...] and b = -[It1, It2, ...]
    # 4. Solve A^T A v = A^T b (2x2 system)
    # 5. Check eigenvalues of A^T A for reliability
    raise NotImplementedError("Implement lucas_kanade")


def flow_to_motion(flow_vectors, K, depth=1.0):
    """
    Estimate camera motion from optical flow.

    Args:
        flow_vectors: Nx2 flow vectors
        K: 3x3 intrinsic matrix
        depth: assumed constant depth

    Returns:
        translation: estimated (tx, ty, tz) camera translation
    """
    # TODO: Use median flow to estimate dominant motion
    raise NotImplementedError("Implement flow_to_motion")


def visualize_flow(image, points, flow, output_path=None):
    """
    Visualize optical flow as arrows on image.

    Args:
        image: 2D grayscale image
        points: Nx2 starting points
        flow: Nx2 flow vectors
        output_path: optional path to save figure
    """
    # TODO: Plot image with quiver arrows showing flow
    raise NotImplementedError("Implement visualize_flow")


def main():
    """Main function: compute optical flow, estimate motion."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, 'data')

    print("Week 10 - Task 6: Optical Flow Estimation")
    print("=" * 44)

    frame1 = np.load(os.path.join(data_dir, 'optical_flow_frame1.npy'))
    frame2 = np.load(os.path.join(data_dir, 'optical_flow_frame2.npy'))
    gt_flow = np.load(os.path.join(data_dir, 'optical_flow_gt.npy'))

    print(f"Frame shape: {frame1.shape}")
    print(f"Ground truth flow shape: {gt_flow.shape}")

    # TODO: Select tracking points (e.g., on a grid)
    # TODO: Compute Lucas-Kanade flow
    # TODO: Compare with ground truth
    # TODO: Estimate camera motion

    print("\nImplement the functions above and complete main().")


if __name__ == '__main__':
    main()
