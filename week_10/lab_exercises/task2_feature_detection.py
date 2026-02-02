#!/usr/bin/env python3
"""
Week 10 - Task 2: Feature Detection and Matching
Implement Harris corner detector, descriptor computation, and feature matching.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


def harris_corner_response(image, k=0.04, window_size=5):
    """
    Compute Harris corner response using structure tensor.

    Args:
        image: 2D grayscale image (HxW)
        k: Harris detector parameter (typically 0.04-0.06)
        window_size: Gaussian window size for structure tensor smoothing

    Returns:
        Response map (HxW) - positive values indicate corners
    """
    # TODO: Implement Harris corner detection
    # 1. Compute image gradients Ix, Iy (finite differences)
    # 2. Compute products Ix^2, Iy^2, Ix*Iy
    # 3. Apply Gaussian smoothing to get structure tensor elements
    # 4. Compute response R = det(M) - k * trace(M)^2
    raise NotImplementedError("Implement harris_corner_response")


def non_maximum_suppression(response, threshold, radius=5):
    """
    Non-maximum suppression on Harris response map.

    Args:
        response: HxW response map
        threshold: minimum response value
        radius: suppression radius

    Returns:
        Nx2 array of corner positions (x, y)
    """
    # TODO: Find local maxima above threshold
    raise NotImplementedError("Implement non_maximum_suppression")


def compute_simple_descriptor(image, keypoint, patch_size=16):
    """
    Compute a simple normalized patch descriptor for a keypoint.

    Args:
        image: 2D grayscale image
        keypoint: (x, y) position
        patch_size: size of patch to extract

    Returns:
        Descriptor vector (patch_size*patch_size,)
    """
    # TODO: Extract patch, normalize to zero mean unit variance
    raise NotImplementedError("Implement compute_simple_descriptor")


def match_features(desc1, desc2, ratio_threshold=0.75):
    """
    Brute-force feature matching with Lowe's ratio test.

    Args:
        desc1: NxD descriptors from image 1
        desc2: MxD descriptors from image 2
        ratio_threshold: max ratio of best/second-best match

    Returns:
        matches: list of (idx1, idx2) tuples
    """
    # TODO: For each descriptor in desc1, find two nearest in desc2
    # Apply ratio test
    raise NotImplementedError("Implement match_features")


def main():
    """Main function: detect features, match, visualize."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, 'data')

    print("Week 10 - Task 2: Feature Detection and Matching")
    print("=" * 50)

    # Load synthetic image data
    frame1 = np.load(os.path.join(data_dir, 'optical_flow_frame1.npy'))
    frame2 = np.load(os.path.join(data_dir, 'optical_flow_frame2.npy'))

    print(f"Image 1 shape: {frame1.shape}")
    print(f"Image 2 shape: {frame2.shape}")

    # TODO: Detect Harris corners in both images
    # TODO: Compute descriptors at detected corners
    # TODO: Match features
    # TODO: Visualize matches

    print("\nImplement the functions above and complete main().")


if __name__ == '__main__':
    main()
