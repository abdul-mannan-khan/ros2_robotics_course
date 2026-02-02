#!/usr/bin/env python3
"""
Week 10 - Task 4: Simple Object Detection
Implement sliding window detection with HOG features (no deep learning).
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def sliding_window(image, window_size, stride):
    """
    Generate sliding window positions over an image.

    Args:
        image: 2D array (HxW)
        window_size: (width, height) tuple
        stride: step size in pixels

    Returns:
        List of (x, y, window_patch) tuples
    """
    # TODO: Iterate over image with given stride and window size
    raise NotImplementedError("Implement sliding_window")


def compute_hog_features(patch, cell_size=8, n_bins=9):
    """
    Compute simplified HOG (Histogram of Oriented Gradients) descriptor.

    Args:
        patch: 2D image patch
        cell_size: size of each cell in pixels
        n_bins: number of orientation bins

    Returns:
        HOG feature vector
    """
    # TODO: Implement HOG
    # 1. Compute gradient magnitudes and orientations
    # 2. Divide patch into cells
    # 3. Build orientation histogram for each cell
    # 4. Concatenate and normalize
    raise NotImplementedError("Implement compute_hog_features")


def classify_window(features, templates):
    """
    Score a window by comparing HOG features against templates.

    Args:
        features: HOG descriptor of the window
        templates: dict mapping label -> list of template HOG descriptors

    Returns:
        best_label, best_score
    """
    # TODO: Compute similarity (e.g., normalized correlation) to each template
    raise NotImplementedError("Implement classify_window")


def compute_iou(box1, box2):
    """
    Compute Intersection over Union of two bounding boxes.

    Args:
        box1, box2: (x1, y1, x2, y2) format

    Returns:
        IoU value
    """
    # TODO: Compute intersection area / union area
    raise NotImplementedError("Implement compute_iou")


def non_max_suppression_boxes(boxes, scores, iou_threshold=0.5):
    """
    Non-maximum suppression for bounding boxes.

    Args:
        boxes: Nx4 array of (x1, y1, x2, y2)
        scores: N array of confidence scores
        iou_threshold: overlap threshold for suppression

    Returns:
        indices: list of kept box indices
    """
    # TODO: Greedy NMS - sort by score, suppress overlapping boxes
    raise NotImplementedError("Implement non_max_suppression_boxes")


def detect_objects(image, templates):
    """
    Full detection pipeline: sliding window + HOG + NMS.

    Args:
        image: 2D grayscale image
        templates: template HOG descriptors per class

    Returns:
        detections: list of (x1, y1, x2, y2, label, score)
    """
    # TODO: Run sliding window, classify each, apply NMS
    raise NotImplementedError("Implement detect_objects")


def evaluate_detections(detections, ground_truth, iou_threshold=0.5):
    """
    Compute precision, recall, and average precision.

    Args:
        detections: list of (x1, y1, x2, y2, label, score)
        ground_truth: list of (x1, y1, x2, y2, label)
        iou_threshold: matching threshold

    Returns:
        precision, recall, ap (average precision)
    """
    # TODO: Match detections to ground truth, compute metrics
    raise NotImplementedError("Implement evaluate_detections")


def main():
    """Main function: detect objects in synthetic scene, compute metrics."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, 'data')

    print("Week 10 - Task 4: Simple Object Detection")
    print("=" * 44)

    scene = np.load(os.path.join(data_dir, 'scene_objects.npy'))
    frame = np.load(os.path.join(data_dir, 'optical_flow_frame1.npy'))

    print(f"Scene objects: {scene.shape[0]}")
    print(f"Image shape: {frame.shape}")

    # TODO: Build templates from scene objects
    # TODO: Run detection pipeline
    # TODO: Evaluate detections
    # TODO: Visualize

    print("\nImplement the functions above and complete main().")


if __name__ == '__main__':
    main()
