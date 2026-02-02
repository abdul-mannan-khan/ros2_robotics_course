#!/usr/bin/env python3
"""
Week 10 - Task 7 Solution: Complete Drone Vision Pipeline
Integrates camera model, features, homography, detection, stereo, and flow.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter, zoom


# ---- Reuse implementations from previous tasks ----

def project_point(K, R, t, point_3d):
    p_cam = R @ point_3d + t.flatten()
    p_img = K @ p_cam
    return p_img[:2] / p_img[2]


def harris_corner_response(image, k=0.04, window_size=5):
    Iy, Ix = np.gradient(image)
    Ixx = gaussian_filter(Ix * Ix, sigma=window_size / 3.0)
    Iyy = gaussian_filter(Iy * Iy, sigma=window_size / 3.0)
    Ixy = gaussian_filter(Ix * Iy, sigma=window_size / 3.0)
    det_M = Ixx * Iyy - Ixy ** 2
    trace_M = Ixx + Iyy
    return det_M - k * trace_M ** 2


def non_maximum_suppression(response, threshold, radius=5):
    h, w = response.shape
    corners = []
    ys, xs = np.where(response > threshold)
    values = response[response > threshold]
    order = np.argsort(-values)
    suppressed = np.zeros_like(response, dtype=bool)
    for idx in order:
        y, x = ys[idx], xs[idx]
        if suppressed[y, x]:
            continue
        corners.append([x, y])
        y0 = max(0, y - radius)
        y1 = min(h, y + radius + 1)
        x0 = max(0, x - radius)
        x1 = min(w, x + radius + 1)
        suppressed[y0:y1, x0:x1] = True
    return np.array(corners) if corners else np.empty((0, 2))


def compute_simple_descriptor(image, keypoint, patch_size=16):
    x, y = int(round(keypoint[0])), int(round(keypoint[1]))
    half = patch_size // 2
    h, w = image.shape
    if y - half < 0 or y + half >= h or x - half < 0 or x + half >= w:
        return None
    patch = image[y - half:y + half, x - half:x + half].flatten().astype(np.float64)
    std = np.std(patch)
    if std < 1e-8:
        return None
    return (patch - np.mean(patch)) / std


def match_features_bf(desc1, desc2, ratio_threshold=0.8):
    matches = []
    for i, d1 in enumerate(desc1):
        dists = np.linalg.norm(desc2 - d1, axis=1)
        if len(dists) < 2:
            continue
        idx = np.argsort(dists)
        if dists[idx[1]] > 1e-8 and dists[idx[0]] / dists[idx[1]] < ratio_threshold:
            matches.append((i, idx[0]))
    return matches


def compute_homography_dlt(src_pts, dst_pts):
    n = len(src_pts)
    A = np.zeros((2 * n, 9))
    for i in range(n):
        x, y = src_pts[i]
        u, v = dst_pts[i]
        A[2 * i] = [-x, -y, -1, 0, 0, 0, u * x, u * y, u]
        A[2 * i + 1] = [0, 0, 0, -x, -y, -1, v * x, v * y, v]
    _, _, Vt = np.linalg.svd(A)
    H = Vt[-1].reshape(3, 3)
    return H / H[2, 2]


def apply_homography(H, points):
    ones = np.ones((len(points), 1))
    pts_h = np.hstack([points, ones])
    transformed = (H @ pts_h.T).T
    return transformed[:, :2] / transformed[:, 2:3]


def ransac_homography(src_pts, dst_pts, threshold=5.0, max_iter=1000):
    n = len(src_pts)
    best_inliers = np.zeros(n, dtype=bool)
    best_H = np.eye(3)
    best_count = 0
    for _ in range(max_iter):
        idx = np.random.choice(n, 4, replace=False)
        try:
            H = compute_homography_dlt(src_pts[idx], dst_pts[idx])
        except Exception:
            continue
        proj = apply_homography(H, src_pts)
        errs = np.linalg.norm(proj - dst_pts, axis=1)
        inliers = errs < threshold
        count = np.sum(inliers)
        if count > best_count:
            best_count = count
            best_inliers = inliers.copy()
            best_H = H.copy()
    if best_count >= 4:
        best_H = compute_homography_dlt(src_pts[best_inliers], dst_pts[best_inliers])
    return best_H, best_inliers


def lucas_kanade(frame1, frame2, points, window_size=15):
    half = window_size // 2
    h, w = frame1.shape
    f1 = gaussian_filter(frame1.astype(np.float64), sigma=1.0)
    f2 = gaussian_filter(frame2.astype(np.float64), sigma=1.0)
    Iy, Ix = np.gradient(f1)
    It = f2 - f1
    flow = np.zeros((len(points), 2))
    status = np.zeros(len(points), dtype=bool)
    for i, (px, py) in enumerate(points):
        x, y = int(round(px)), int(round(py))
        if y - half < 0 or y + half >= h or x - half < 0 or x + half >= w:
            continue
        ix = Ix[y - half:y + half + 1, x - half:x + half + 1].flatten()
        iy = Iy[y - half:y + half + 1, x - half:x + half + 1].flatten()
        it = It[y - half:y + half + 1, x - half:x + half + 1].flatten()
        ATA = np.array([[np.sum(ix * ix), np.sum(ix * iy)],
                         [np.sum(ix * iy), np.sum(iy * iy)]])
        ATb = -np.array([np.sum(ix * it), np.sum(iy * it)])
        eigs = np.linalg.eigvalsh(ATA)
        if np.min(eigs) < 1e-6:
            continue
        flow[i] = np.linalg.solve(ATA, ATb)
        status[i] = True
    return flow, status


def compute_disparity_fast(left, right, block_size=7, max_disparity=32):
    h, w = left.shape
    half = block_size // 2
    disparity = np.zeros((h, w))
    for y in range(half, h - half):
        for x in range(half + max_disparity, w - half):
            lb = left[y - half:y + half + 1, x - half:x + half + 1]
            best_d, best_sad = 0, float('inf')
            for d in range(max_disparity):
                rx = x - d
                if rx - half < 0:
                    break
                rb = right[y - half:y + half + 1, rx - half:rx + half + 1]
                sad = np.sum(np.abs(lb - rb))
                if sad < best_sad:
                    best_sad = sad
                    best_d = d
            disparity[y, x] = best_d
    return disparity


class DroneVisionPipeline:
    def __init__(self, K, dist_coeffs=None):
        self.K = K
        self.dist_coeffs = dist_coeffs
        self.prev_features = None
        self.trajectory = []

    def calibrate(self, object_points, image_points):
        n = len(object_points)
        A = np.zeros((2 * n, 12))
        for i in range(n):
            X, Y, Z = object_points[i]
            u, v = image_points[i]
            A[2 * i] = [X, Y, Z, 1, 0, 0, 0, 0, -u * X, -u * Y, -u * Z, -u]
            A[2 * i + 1] = [0, 0, 0, 0, X, Y, Z, 1, -v * X, -v * Y, -v * Z, -v]
        _, _, Vt = np.linalg.svd(A)
        P = Vt[-1].reshape(3, 4)
        M = P[:, :3]
        Q, R_mat = np.linalg.qr(np.flipud(M).T)
        K_est = np.flipud(np.fliplr(R_mat.T))
        K_est = K_est / K_est[2, 2]
        return K_est

    def detect_features(self, image):
        resp = harris_corner_response(image, k=0.04, window_size=5)
        thresh = np.percentile(resp[resp > 0], 95) if np.any(resp > 0) else 0.001
        kps = non_maximum_suppression(resp, thresh, radius=10)
        descs, valid = [], []
        for kp in kps:
            d = compute_simple_descriptor(image, kp)
            if d is not None:
                descs.append(d)
                valid.append(kp)
        return np.array(valid), np.array(descs) if descs else np.empty((0, 0))

    def estimate_motion(self, features_prev, features_curr):
        kp1, d1 = features_prev
        kp2, d2 = features_curr
        if len(d1) == 0 or len(d2) == 0:
            return np.eye(3), np.zeros((3, 1))
        matches = match_features_bf(d1, d2)
        if len(matches) < 4:
            return np.eye(3), np.zeros((3, 1))
        src = np.array([kp1[m[0]] for m in matches])
        dst = np.array([kp2[m[1]] for m in matches])
        H, inliers = ransac_homography(src, dst)
        # Simple decomposition
        K_inv = np.linalg.inv(self.K)
        Hn = K_inv @ H @ self.K
        U, S, Vt = np.linalg.svd(Hn)
        Hn = Hn / S[1]
        R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
        t = (Hn[:, 2] - R[:, 2]).reshape(3, 1)
        return R, t

    def detect_markers(self, image):
        """Simple threshold-based marker detection."""
        markers = []
        h, w = image.shape
        # Find dark rectangular regions
        binary = (image < 0.2).astype(np.float64)
        # Simple connected component-like search (scan for marker-sized regions)
        visited = np.zeros_like(binary, dtype=bool)
        marker_id = 0
        for y in range(0, h - 50, 10):
            for x in range(0, w - 50, 10):
                if binary[y, x] > 0 and not visited[y, x]:
                    # Check if this looks like a marker border
                    for sz in [84, 70, 56]:
                        if y + sz < h and x + sz < w:
                            border_top = np.mean(binary[y, x:x + sz])
                            border_bot = np.mean(binary[y + sz - 1, x:x + sz])
                            border_left = np.mean(binary[y:y + sz, x])
                            border_right = np.mean(binary[y:y + sz, x + sz - 1])
                            if min(border_top, border_bot, border_left, border_right) > 0.5:
                                corners = np.array([
                                    [x, y], [x + sz, y],
                                    [x + sz, y + sz], [x, y + sz]
                                ], dtype=np.float64)
                                markers.append((marker_id, corners))
                                marker_id += 1
                                visited[y:y + sz, x:x + sz] = True
                                break
        return markers

    def estimate_depth(self, left, right, baseline, focal_length):
        step = 4
        ls, rs = left[::step, ::step], right[::step, ::step]
        disp_sub = compute_disparity_fast(ls, rs, block_size=7, max_disparity=32)
        disp = zoom(disp_sub, step, order=1) * step
        disp = disp[:left.shape[0], :left.shape[1]]
        depth = np.zeros_like(disp)
        valid = disp > 0
        depth[valid] = (focal_length * baseline) / disp[valid]
        depth[(depth < 0.1) | (depth > 80)] = 0
        return depth

    def full_pipeline(self, frames, drone_poses):
        results = {
            'trajectory_est': [],
            'features_per_frame': [],
            'n_matches': [],
        }
        pose = np.eye(4)
        results['trajectory_est'].append(pose[:3, 3].copy())

        prev_feat = None
        for i, frame in enumerate(frames):
            kps, descs = self.detect_features(frame)
            results['features_per_frame'].append(len(kps))

            if prev_feat is not None and len(descs) > 0:
                R, t = self.estimate_motion(prev_feat, (kps, descs))
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = t.flatten()
                pose = pose @ T
                results['trajectory_est'].append(pose[:3, 3].copy())
                results['n_matches'].append(len(descs))
            else:
                results['n_matches'].append(0)

            prev_feat = (kps, descs)

        results['trajectory_est'] = np.array(results['trajectory_est'])
        return results


def evaluate_visual_odometry(estimated_poses, ground_truth):
    """Compute ATE and RPE."""
    n = min(len(estimated_poses), len(ground_truth))
    est = estimated_poses[:n]
    gt = ground_truth[:n, :3]

    # Align by subtracting first pose
    est_aligned = est - est[0]
    gt_aligned = gt - gt[0]

    # Scale alignment
    est_scale = np.max(np.linalg.norm(est_aligned, axis=1))
    gt_scale = np.max(np.linalg.norm(gt_aligned, axis=1))
    if est_scale > 1e-6:
        est_aligned = est_aligned * (gt_scale / est_scale)

    # ATE
    ate = np.sqrt(np.mean(np.sum((est_aligned - gt_aligned) ** 2, axis=1)))

    # RPE
    rpe_errors = []
    for i in range(1, n):
        d_est = est_aligned[i] - est_aligned[i - 1]
        d_gt = gt_aligned[i] - gt_aligned[i - 1]
        rpe_errors.append(np.linalg.norm(d_est - d_gt))
    rpe = np.sqrt(np.mean(np.array(rpe_errors) ** 2)) if rpe_errors else 0.0

    return ate, rpe


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, '..', 'lab_exercises', 'data')
    output_dir = script_dir

    print("Week 10 - Task 7 Solution: Complete Drone Vision Pipeline")
    print("=" * 58)

    K = np.load(os.path.join(data_dir, 'camera_intrinsics.npy'))
    dist_coeffs = np.load(os.path.join(data_dir, 'distortion_coeffs.npy'))
    drone_poses = np.load(os.path.join(data_dir, 'drone_poses.npy'))
    frame1 = np.load(os.path.join(data_dir, 'optical_flow_frame1.npy'))
    frame2 = np.load(os.path.join(data_dir, 'optical_flow_frame2.npy'))
    gt_flow = np.load(os.path.join(data_dir, 'optical_flow_gt.npy'))
    left = np.load(os.path.join(data_dir, 'stereo_left.npy'))
    right = np.load(os.path.join(data_dir, 'stereo_right.npy'))
    stereo_params = np.load(os.path.join(data_dir, 'stereo_params.npy'))
    marker_img = np.load(os.path.join(data_dir, 'marker_image.npy'))
    marker_gt = np.load(os.path.join(data_dir, 'marker_corners_gt.npy'))

    baseline, focal_length = stereo_params

    # Initialize pipeline
    pipeline = DroneVisionPipeline(K, dist_coeffs)

    # 1. Feature detection on frame1
    kps1, descs1 = pipeline.detect_features(frame1)
    kps2, descs2 = pipeline.detect_features(frame2)
    print(f"Features: frame1={len(kps1)}, frame2={len(kps2)}")

    # 2. Motion estimation
    R_est, t_est = pipeline.estimate_motion((kps1, descs1), (kps2, descs2))
    print(f"Estimated rotation trace: {np.trace(R_est):.4f}")

    # 3. Marker detection
    markers = pipeline.detect_markers(marker_img)
    print(f"Markers detected: {len(markers)}")

    # 4. Stereo depth
    depth = pipeline.estimate_depth(left, right, baseline, focal_length)
    valid_depth = depth[depth > 0]
    print(f"Depth points: {len(valid_depth)}, range: [{valid_depth.min():.2f}, {valid_depth.max():.2f}] m" if len(valid_depth) > 0 else "No valid depth")

    # 5. Optical flow
    step = 30
    xs = np.arange(30, frame1.shape[1] - 30, step)
    ys = np.arange(30, frame1.shape[0] - 30, step)
    xx, yy = np.meshgrid(xs, ys)
    track_pts = np.column_stack([xx.ravel(), yy.ravel()]).astype(np.float64)
    flow, flow_status = lucas_kanade(frame1, frame2, track_pts, window_size=21)
    print(f"Optical flow: tracked {np.sum(flow_status)}/{len(track_pts)}")

    # 6. Visual odometry on synthetic trajectory
    # Generate simple shifted frames for VO
    np.random.seed(77)
    n_frames = min(20, len(drone_poses))
    frames = []
    for i in range(n_frames):
        shift_x = int(drone_poses[i, 0] * 2)
        shift_y = int(drone_poses[i, 1] * 2)
        shifted = np.roll(np.roll(frame1, shift_x, axis=1), shift_y, axis=0)
        shifted += np.random.randn(*shifted.shape) * 0.01
        frames.append(np.clip(shifted, 0, 1))

    results = pipeline.full_pipeline(frames, drone_poses[:n_frames])
    ate, rpe = evaluate_visual_odometry(results['trajectory_est'], drone_poses[:n_frames])
    print(f"Visual Odometry - ATE: {ate:.4f}, RPE: {rpe:.4f}")

    # ---- 6-Panel Visualization ----
    fig, axes = plt.subplots(2, 3, figsize=(18, 11))

    # Panel 1: Trajectory
    gt = drone_poses[:n_frames, :3]
    est = results['trajectory_est']
    est_aligned = est - est[0]
    gt_aligned = gt - gt[0]
    s = np.max(np.linalg.norm(est_aligned, axis=1))
    if s > 1e-6:
        est_aligned = est_aligned * (np.max(np.linalg.norm(gt_aligned, axis=1)) / s)
    axes[0, 0].plot(gt_aligned[:, 0], gt_aligned[:, 1], 'b-o', markersize=3, label='Ground Truth')
    axes[0, 0].plot(est_aligned[:, 0], est_aligned[:, 1], 'r--x', markersize=3, label='Estimated')
    axes[0, 0].set_title(f'Trajectory (ATE={ate:.3f})')
    axes[0, 0].legend()
    axes[0, 0].set_xlabel('X')
    axes[0, 0].set_ylabel('Y')
    axes[0, 0].set_aspect('equal')

    # Panel 2: Feature tracks
    axes[0, 1].imshow(frame1, cmap='gray')
    if len(kps1) > 0:
        axes[0, 1].scatter(kps1[:, 0], kps1[:, 1], c='lime', s=8, marker='+')
    matches = match_features_bf(descs1, descs2) if len(descs1) > 0 and len(descs2) > 0 else []
    for m1, m2 in matches[:30]:
        axes[0, 1].plot([kps1[m1, 0], kps2[m2, 0]], [kps1[m1, 1], kps2[m2, 1]], 'c-', alpha=0.5, linewidth=0.5)
    axes[0, 1].set_title(f'Feature Tracks ({len(matches)} matches)')

    # Panel 3: Depth map
    im = axes[0, 2].imshow(depth, cmap='plasma')
    axes[0, 2].set_title('Stereo Depth')
    plt.colorbar(im, ax=axes[0, 2], fraction=0.046)

    # Panel 4: Marker detection
    axes[1, 0].imshow(marker_img, cmap='gray')
    for mid, corners in markers:
        c = np.vstack([corners, corners[0]])
        axes[1, 0].plot(c[:, 0], c[:, 1], 'r-', linewidth=2)
        axes[1, 0].text(corners[0, 0], corners[0, 1] - 5, f'ID:{mid}', color='red', fontsize=9)
    # Also show GT
    for mi in range(len(marker_gt)):
        gc = np.vstack([marker_gt[mi], marker_gt[mi, 0]])
        axes[1, 0].plot(gc[:, 0], gc[:, 1], 'g--', linewidth=1)
    axes[1, 0].set_title(f'Marker Detection ({len(markers)} found)')

    # Panel 5: Optical flow
    fv = track_pts[flow_status]
    flv = flow[flow_status]
    mag = np.sqrt(flv[:, 0] ** 2 + flv[:, 1] ** 2)
    axes[1, 1].imshow(frame1, cmap='gray')
    axes[1, 1].quiver(fv[:, 0], fv[:, 1], flv[:, 0], flv[:, 1],
                       mag, cmap='jet', scale=80, width=0.003)
    axes[1, 1].set_title(f'Optical Flow ({np.sum(flow_status)} pts)')

    # Panel 6: Error metrics
    metrics = (
        f"ATE: {ate:.4f}\n"
        f"RPE: {rpe:.4f}\n"
        f"Features (f1): {len(kps1)}\n"
        f"Features (f2): {len(kps2)}\n"
        f"Matches: {len(matches)}\n"
        f"Markers: {len(markers)}\n"
        f"Flow tracked: {np.sum(flow_status)}\n"
        f"Depth pts: {len(valid_depth)}"
    )
    axes[1, 2].text(0.1, 0.5, metrics, fontsize=13, family='monospace',
                     verticalalignment='center', transform=axes[1, 2].transAxes)
    axes[1, 2].set_title('Pipeline Metrics')
    axes[1, 2].axis('off')

    plt.suptitle('Week 10: Complete Drone Vision Pipeline', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'task7_full_vision.png'), dpi=150)
    plt.close()
    print(f"\nPlot saved to {os.path.join(output_dir, 'task7_full_vision.png')}")


if __name__ == '__main__':
    main()
