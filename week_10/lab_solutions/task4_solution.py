#!/usr/bin/env python3
"""
Week 10 - Task 4 Solution: Simple Object Detection
Sliding window, HOG features, template matching, NMS.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def sliding_window(image, window_size, stride):
    """Generate sliding window positions over an image."""
    h, w = image.shape
    win_w, win_h = window_size
    windows = []
    for y in range(0, h - win_h + 1, stride):
        for x in range(0, w - win_w + 1, stride):
            patch = image[y:y + win_h, x:x + win_w]
            windows.append((x, y, patch))
    return windows


def compute_hog_features(patch, cell_size=8, n_bins=9):
    """Compute simplified HOG descriptor."""
    h, w = patch.shape
    # Gradient computation
    gy, gx = np.gradient(patch.astype(np.float64))
    magnitude = np.sqrt(gx ** 2 + gy ** 2)
    orientation = np.arctan2(gy, gx)
    # Map orientation to [0, pi) for unsigned gradients
    orientation = orientation % np.pi

    n_cells_y = h // cell_size
    n_cells_x = w // cell_size
    hog = []

    bin_edges = np.linspace(0, np.pi, n_bins + 1)
    for cy in range(n_cells_y):
        for cx in range(n_cells_x):
            y0 = cy * cell_size
            x0 = cx * cell_size
            cell_mag = magnitude[y0:y0 + cell_size, x0:x0 + cell_size].flatten()
            cell_ori = orientation[y0:y0 + cell_size, x0:x0 + cell_size].flatten()
            hist, _ = np.histogram(cell_ori, bins=bin_edges, weights=cell_mag)
            hog.extend(hist)

    hog = np.array(hog, dtype=np.float64)
    norm = np.linalg.norm(hog) + 1e-8
    return hog / norm


def classify_window(features, templates):
    """Score window by normalized correlation with templates."""
    best_label = -1
    best_score = -1.0
    for label, tmpl_list in templates.items():
        for tmpl in tmpl_list:
            if len(tmpl) != len(features):
                continue
            score = np.dot(features, tmpl) / (np.linalg.norm(features) * np.linalg.norm(tmpl) + 1e-8)
            if score > best_score:
                best_score = score
                best_label = label
    return best_label, best_score


def compute_iou(box1, box2):
    """Compute IoU of two boxes (x1, y1, x2, y2)."""
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union = area1 + area2 - inter
    return inter / union if union > 0 else 0.0


def non_max_suppression_boxes(boxes, scores, iou_threshold=0.5):
    """Greedy NMS for bounding boxes."""
    if len(boxes) == 0:
        return []
    order = np.argsort(-np.array(scores))
    keep = []
    suppressed = set()
    for i in order:
        if i in suppressed:
            continue
        keep.append(i)
        for j in order:
            if j in suppressed or j == i:
                continue
            if compute_iou(boxes[i], boxes[j]) > iou_threshold:
                suppressed.add(j)
    return keep


def detect_objects(image, templates, window_size=(64, 48), stride=16, score_threshold=0.3):
    """Full detection pipeline."""
    windows = sliding_window(image, window_size, stride)
    boxes, scores, labels = [], [], []

    for x, y, patch in windows:
        features = compute_hog_features(patch)
        label, score = classify_window(features, templates)
        if score > score_threshold:
            boxes.append([x, y, x + window_size[0], y + window_size[1]])
            scores.append(score)
            labels.append(label)

    if not boxes:
        return []

    keep = non_max_suppression_boxes(boxes, scores, iou_threshold=0.3)
    detections = [(boxes[k][0], boxes[k][1], boxes[k][2], boxes[k][3], labels[k], scores[k])
                  for k in keep]
    return detections


def evaluate_detections(detections, ground_truth, iou_threshold=0.5):
    """Compute precision, recall, and AP."""
    if not detections or not ground_truth:
        return 0.0, 0.0, 0.0

    # Sort detections by score descending
    detections = sorted(detections, key=lambda d: d[5], reverse=True)
    gt_matched = [False] * len(ground_truth)
    tp, fp = 0, 0
    precisions, recalls = [], []

    for det in detections:
        best_iou = 0
        best_gt = -1
        for gi, gt in enumerate(ground_truth):
            if gt_matched[gi]:
                continue
            iou = compute_iou(det[:4], gt[:4])
            if iou > best_iou:
                best_iou = iou
                best_gt = gi

        if best_iou >= iou_threshold and best_gt >= 0:
            tp += 1
            gt_matched[best_gt] = True
        else:
            fp += 1

        precision = tp / (tp + fp)
        recall = tp / len(ground_truth)
        precisions.append(precision)
        recalls.append(recall)

    # AP via trapezoidal
    ap = 0.0
    for i in range(1, len(recalls)):
        ap += (recalls[i] - recalls[i - 1]) * precisions[i]

    final_precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    final_recall = tp / len(ground_truth) if ground_truth else 0
    return final_precision, final_recall, ap


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, '..', 'lab_exercises', 'data')
    output_dir = script_dir

    print("Week 10 - Task 4 Solution: Simple Object Detection")
    print("=" * 52)

    scene = np.load(os.path.join(data_dir, 'scene_objects.npy'))
    frame = np.load(os.path.join(data_dir, 'optical_flow_frame1.npy'))
    print(f"Scene has {len(scene)} objects, image shape: {frame.shape}")

    # Build ground truth boxes from scene objects
    gt_boxes = []
    label_names = {0: 'building', 1: 'tree', 2: 'car', 3: 'road'}
    for obj in scene:
        type_id, cx, cy, w, h, label = obj
        if type_id == 2:  # skip lines
            continue
        x1 = int(cx - w / 2)
        y1 = int(cy - h / 2)
        x2 = int(cx + w / 2)
        y2 = int(cy + h / 2)
        gt_boxes.append((x1, y1, x2, y2, int(label)))

    # Render synthetic image with objects
    synth_img = np.ones_like(frame) * 0.5
    for obj in scene:
        type_id, cx, cy, w, h, label = obj
        cx, cy, w, h = int(cx), int(cy), int(w), int(h)
        intensity = 0.3 + 0.15 * label
        if type_id == 0:  # rectangle
            x1, y1 = max(0, cx - w // 2), max(0, cy - h // 2)
            x2, y2 = min(frame.shape[1], cx + w // 2), min(frame.shape[0], cy + h // 2)
            synth_img[y1:y2, x1:x2] = intensity
        elif type_id == 1:  # circle
            yy, xx = np.ogrid[0:frame.shape[0], 0:frame.shape[1]]
            mask = ((xx - cx) ** 2 + (yy - cy) ** 2) <= (w // 2) ** 2
            synth_img[mask] = intensity

    # Add noise
    synth_img += np.random.randn(*synth_img.shape) * 0.02
    synth_img = np.clip(synth_img, 0, 1)

    # Build templates from GT regions
    templates = {}
    for gt in gt_boxes:
        x1, y1, x2, y2, label = gt
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(synth_img.shape[1], x2), min(synth_img.shape[0], y2)
        patch = synth_img[y1:y2, x1:x2]
        if patch.size == 0:
            continue
        # Resize patch to standard window by simple sampling
        from scipy.ndimage import zoom
        target_h, target_w = 48, 64
        zh = target_h / patch.shape[0]
        zw = target_w / patch.shape[1]
        resized = zoom(patch, (zh, zw))
        hog = compute_hog_features(resized)
        if label not in templates:
            templates[label] = []
        templates[label].append(hog)

    print(f"Templates per class: { {label_names.get(k, k): len(v) for k, v in templates.items()} }")

    # Run detection
    detections = detect_objects(synth_img, templates, window_size=(64, 48),
                                stride=16, score_threshold=0.5)
    print(f"Detections after NMS: {len(detections)}")

    # Evaluate
    precision, recall, ap = evaluate_detections(detections, gt_boxes, iou_threshold=0.3)
    print(f"Precision: {precision:.3f}, Recall: {recall:.3f}, AP: {ap:.3f}")

    # Visualization
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Synthetic scene
    axes[0, 0].imshow(synth_img, cmap='gray')
    for gt in gt_boxes:
        x1, y1, x2, y2, label = gt
        rect = plt.Rectangle((x1, y1), x2 - x1, y2 - y1, linewidth=2,
                              edgecolor='green', facecolor='none')
        axes[0, 0].add_patch(rect)
        axes[0, 0].text(x1, y1 - 3, label_names.get(label, '?'), color='green', fontsize=8)
    axes[0, 0].set_title('Ground Truth')

    # Detections
    axes[0, 1].imshow(synth_img, cmap='gray')
    for det in detections:
        x1, y1, x2, y2, label, score = det
        rect = plt.Rectangle((x1, y1), x2 - x1, y2 - y1, linewidth=2,
                              edgecolor='red', facecolor='none')
        axes[0, 1].add_patch(rect)
        axes[0, 1].text(x1, y1 - 3, f'{label_names.get(label, "?")} {score:.2f}',
                         color='red', fontsize=7)
    axes[0, 1].set_title(f'Detections ({len(detections)})')

    # HOG visualization for one template
    if templates:
        first_label = list(templates.keys())[0]
        hog_vec = templates[first_label][0]
        axes[1, 0].bar(range(len(hog_vec)), hog_vec, color='steelblue')
        axes[1, 0].set_title(f'HOG Descriptor ({label_names.get(first_label, "?")})')
        axes[1, 0].set_xlabel('Bin index')
        axes[1, 0].set_ylabel('Value')

    # Metrics
    metrics_text = (f"Precision: {precision:.3f}\n"
                    f"Recall: {recall:.3f}\n"
                    f"Average Precision: {ap:.3f}\n"
                    f"Detections: {len(detections)}\n"
                    f"Ground Truth: {len(gt_boxes)}")
    axes[1, 1].text(0.3, 0.5, metrics_text, fontsize=14, family='monospace',
                     verticalalignment='center', transform=axes[1, 1].transAxes)
    axes[1, 1].set_title('Detection Metrics')
    axes[1, 1].axis('off')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'task4_object_detection.png'), dpi=150)
    plt.close()
    print(f"\nPlot saved to {os.path.join(output_dir, 'task4_object_detection.png')}")


if __name__ == '__main__':
    main()
