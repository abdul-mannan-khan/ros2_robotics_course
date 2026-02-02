# Week 1: LiDAR Point Cloud Processing Fundamentals

**Course:** M.Sc. Control & Automation
**Module Leader:** Dr. Abdul Manan Khan
**Category:** PERCEPTION | Priority: CRITICAL
**Target Companies:** Waymo, Cruise, Aurora, Skydio, DJI

---

## 1. Introduction to LiDAR

### 1.1 What is LiDAR?

**LiDAR** (Light Detection and Ranging) is an active sensing technology that measures distances by illuminating targets with laser light and measuring the reflected pulses with a sensor.

**Key Properties:**
- Direct 3D measurements (unlike cameras which produce 2D projections)
- Works in darkness and varying lighting conditions
- Typical range: 10m to 200m+
- Angular resolution: 0.1° to 0.4°
- Point density: thousands to millions of points per scan

### 1.2 Types of LiDAR Sensors

| Type | Example | Points/sec | Range | Use Case |
|------|---------|-----------|-------|----------|
| 2D Scanning | RPLiDAR S2 | 32,000 | 30m | Mobile robots, SLAM |
| 3D Spinning | Velodyne VLP-16 | 300,000 | 100m | Autonomous vehicles |
| Solid-State | Livox Mid-360 | 200,000 | 40m | Drones, compact robots |
| Flash | Continental HFL110 | N/A | 30m | Automotive ADAS |

### 1.3 Point Cloud Data

A point cloud is a set of 3D points:

```
P = {p_i | p_i = (x_i, y_i, z_i, I_i)}, i = 1, ..., N
```

Where:
- `(x, y, z)` = 3D coordinates in the sensor frame
- `I` = intensity (reflectivity of the surface)
- `N` = number of points (typically 10K-1M per scan)

**Common file formats:**
- **PCD** (Point Cloud Data) - PCL native format
- **PLY** (Polygon File Format) - widely supported
- **LAS/LAZ** - standard for aerial LiDAR
- **NumPy .npy** - convenient for Python workflows

---

## 2. Point Cloud Preprocessing

### 2.1 Voxel Grid Downsampling

**Problem:** Raw point clouds are too dense for real-time processing.

**Solution:** Divide 3D space into cubic voxels of size `(v_x, v_y, v_z)` and replace all points in each voxel with their centroid.

**Algorithm:**
```
Input: Point cloud P, voxel size v
1. Compute bounding box of P
2. Divide bounding box into voxels of size v
3. For each occupied voxel:
   - Compute centroid of all points inside
   - Replace with single centroid point
Output: Downsampled point cloud P'
```

**Complexity:** O(N) where N = number of points

**Effect on data:**
- Voxel size 0.01m: minimal reduction, preserves detail
- Voxel size 0.05m: good balance for obstacle detection
- Voxel size 0.1m+: aggressive reduction, loses fine detail

**Open3D Implementation:**
```python
import open3d as o3d

pcd = o3d.io.read_point_cloud("scan.pcd")
downsampled = pcd.voxel_down_sample(voxel_size=0.05)
```

### 2.2 Statistical Outlier Removal

Removes sparse noise points that don't belong to any surface.

**Algorithm:**
```
For each point p_i:
  1. Find k nearest neighbors
  2. Compute mean distance d_mean to neighbors
  3. Compute global mean μ and std σ of all d_mean values
  4. Remove point if d_mean > μ + α·σ
```

Parameters: `k` (neighbors), `α` (std multiplier, typically 1.0-2.0)

### 2.3 Region of Interest (ROI) Cropping

For autonomous vehicles/drones, we only care about a relevant region:

```python
# Crop to region of interest
# x: forward, y: lateral, z: vertical
min_bound = np.array([-20, -10, -2])
max_bound = np.array([40, 10, 5])
```

---

## 3. Ground Plane Segmentation with RANSAC

### 3.1 The RANSAC Algorithm

**RANSAC** (Random Sample Consensus) is a robust model fitting algorithm that handles outliers.

**For plane fitting, a plane is defined as:**

```
ax + by + cz + d = 0
```

**Algorithm:**
```
Input: Point cloud P, distance threshold t, max iterations N
1. best_inliers = {}
2. For i = 1 to N:
   a. Randomly select 3 points (minimum for plane)
   b. Fit plane through these 3 points
   c. Count inliers: points within distance t of plane
   d. If |inliers| > |best_inliers|:
      best_inliers = inliers
3. Refit plane using all best_inliers (least squares)
Output: Plane parameters (a, b, c, d), inlier indices
```

### 3.2 Choosing RANSAC Parameters

**Number of iterations:**

```
N = log(1 - p) / log(1 - w^n)
```

Where:
- `p` = desired probability of success (e.g., 0.99)
- `w` = fraction of inliers in data
- `n` = minimum points for model (3 for plane)

**Example:** If 60% of points are ground (w=0.6), n=3:
```
N = log(1 - 0.99) / log(1 - 0.6^3) = log(0.01) / log(0.784) ≈ 19 iterations
```

In practice, use 100-1000 iterations for robustness.

**Distance threshold `t`:**
- Too small: misses ground points on uneven terrain
- Too large: includes low obstacles as ground
- Typical: 0.1m - 0.3m

### 3.3 Mathematical Derivation

Given 3 points `p1, p2, p3`:

```
v1 = p2 - p1
v2 = p3 - p1
normal = v1 × v2 = (a, b, c)
d = -(a·p1.x + b·p1.y + c·p1.z)
```

Distance from point `q` to plane:
```
dist(q) = |a·q.x + b·q.y + c·q.z + d| / sqrt(a² + b² + c²)
```

---

## 4. Spatial Indexing with KD-Trees

### 4.1 KD-Tree Data Structure

A **KD-Tree** (K-Dimensional Tree) is a binary space-partitioning tree for organizing points in k-dimensional space.

**Construction:**
```
function build_kdtree(points, depth=0):
    if points is empty: return null

    axis = depth % k  # alternate splitting axis
    sort points by axis coordinate
    median = len(points) // 2

    node = new Node(points[median])
    node.left = build_kdtree(points[:median], depth+1)
    node.right = build_kdtree(points[median+1:], depth+1)
    return node
```

**Complexity:**
| Operation | KD-Tree | Brute Force |
|-----------|---------|-------------|
| Build | O(N log N) | N/A |
| Nearest Neighbor | O(log N) avg | O(N) |
| Range Search | O(√N + k) avg | O(N) |
| Space | O(N) | O(N) |

### 4.2 Nearest Neighbor Search

```
function nn_search(node, target, depth=0, best=None):
    if node is null: return best

    if best is None or dist(node.point, target) < dist(best, target):
        best = node.point

    axis = depth % k
    if target[axis] < node.point[axis]:
        near, far = node.left, node.right
    else:
        near, far = node.right, node.left

    best = nn_search(near, target, depth+1, best)

    # Check if we need to explore far branch
    if |target[axis] - node.point[axis]| < dist(best, target):
        best = nn_search(far, target, depth+1, best)

    return best
```

---

## 5. Clustering Algorithms

### 5.1 Euclidean Clustering

Groups points that are within a distance threshold of each other.

**Algorithm:**
```
Input: Points P, distance threshold d, min_size, max_size
1. Build KD-Tree from P
2. Create visited set
3. clusters = []
4. For each unvisited point p:
   a. queue = [p], mark p visited
   b. cluster = []
   c. While queue not empty:
      - q = queue.pop()
      - cluster.append(q)
      - neighbors = kdtree.radius_search(q, d)
      - For each unvisited neighbor n:
        mark n visited, queue.append(n)
   d. If min_size <= |cluster| <= max_size:
      clusters.append(cluster)
Output: List of clusters
```

**Parameters:**
- `d` (cluster tolerance): 0.3m-1.0m for vehicle detection
- `min_size`: filter small noise clusters (e.g., 10 points)
- `max_size`: filter oversized merged clusters (e.g., 25000 points)

### 5.2 DBSCAN (Density-Based Spatial Clustering)

Similar to Euclidean clustering but adds the concept of **core points**:

```
A point is a CORE point if it has >= min_pts neighbors within radius eps
BORDER points are within eps of a core point but have < min_pts neighbors
NOISE points are neither core nor border
```

**Key Difference from Euclidean Clustering:**
- DBSCAN requires minimum density (min_pts), better at rejecting noise
- Euclidean clustering only uses distance threshold
- DBSCAN can find clusters of arbitrary shape

---

## 6. Bounding Box Fitting

### 6.1 Axis-Aligned Bounding Box (AABB)

Simplest approach - box aligned with coordinate axes:

```python
min_point = np.min(cluster_points, axis=0)  # (x_min, y_min, z_min)
max_point = np.max(cluster_points, axis=0)  # (x_max, y_max, z_max)
```

**Limitation:** Overestimates volume for rotated objects.

### 6.2 Oriented Bounding Box (OBB) using PCA

**Principal Component Analysis** finds the principal axes of the point distribution:

```
1. Compute centroid: μ = mean(points)
2. Center points: P_centered = P - μ
3. Compute covariance matrix: C = (1/N) * P_centered^T * P_centered
4. Eigendecomposition: C = V * Λ * V^T
5. V columns = principal axes (orientation of OBB)
6. Transform points to principal axes frame
7. Compute AABB in transformed frame
8. Transform back to original frame
```

**Result:** Tighter bounding box that follows the object's natural orientation.

### 6.3 Intersection over Union (IoU)

Used to evaluate detection quality against ground truth:

```
IoU = Volume(Intersection) / Volume(Union)
```

- IoU > 0.7: good detection
- IoU > 0.5: acceptable detection
- IoU < 0.3: poor detection

---

## 7. Complete Pipeline Summary

```
Raw Point Cloud
    │
    ▼
[1. Voxel Grid Downsampling] ──→ Reduce points for efficiency
    │
    ▼
[2. ROI Cropping] ──→ Focus on relevant area
    │
    ▼
[3. RANSAC Ground Removal] ──→ Separate ground from obstacles
    │
    ▼
[4. Euclidean Clustering] ──→ Group obstacle points
    │  (using KD-Tree)
    ▼
[5. Bounding Box Fitting] ──→ Enclose each cluster
    │  (AABB or OBB via PCA)
    ▼
Detected Obstacles with Bounding Boxes
```

---

## 8. Real-Time Performance Considerations

For autonomous driving, the pipeline must run at **10Hz** (100ms per frame):

| Stage | Typical Time | Optimization |
|-------|-------------|-------------|
| Voxel downsample | 5-10ms | GPU acceleration |
| RANSAC | 10-20ms | Early termination |
| KD-Tree build | 10-15ms | Incremental update |
| Clustering | 15-30ms | Parallel BFS |
| Bounding boxes | 2-5ms | Minimal overhead |
| **Total** | **42-80ms** | **Fits in 100ms budget** |

---

## 9. Interview Preparation Questions

### Conceptual Questions

1. **Explain RANSAC and its time complexity. How do you choose the number of iterations?**
   - RANSAC iteratively fits a model to random subsets, then counts inliers
   - Time: O(N * k) where k = iterations, N = points for distance check
   - Iterations: N = log(1-p) / log(1-w^n)

2. **Why use KD-Tree for clustering? What's the time complexity vs brute force?**
   - KD-Tree: O(log N) average for nearest neighbor vs O(N) brute force
   - Radius search: O(√N + k) vs O(N)
   - Critical for real-time clustering of large point clouds

3. **How do you handle dynamic objects vs static obstacles?**
   - Multi-frame tracking: associate clusters across frames
   - Velocity estimation via centroid displacement
   - Static objects: consistent position across frames
   - Dynamic: use motion prediction for collision avoidance

4. **Describe the difference between Euclidean clustering and DBSCAN.**
   - Euclidean: only distance threshold, simpler, faster
   - DBSCAN: adds min_pts requirement, better noise rejection
   - DBSCAN distinguishes core/border/noise points
   - Euclidean preferred when speed matters; DBSCAN when noise is high

5. **How would you optimize this pipeline for real-time performance (10Hz)?**
   - Voxel downsampling to reduce point count
   - GPU-accelerated RANSAC (CUDA)
   - Incremental KD-Tree updates between frames
   - Parallel clustering with thread pool
   - ROI cropping to limit search space

---

## 10. References & Resources

### GitHub Repositories
- [knaaga/lidar-obstacle-detection](https://github.com/knaaga/lidar-obstacle-detection) - C++ pipeline with PCL
- [arief25ramadhan/point-cloud-processing](https://github.com/arief25ramadhan/point-cloud-processing) - Python pipeline with Open3D

### Libraries
- **Open3D** (Python/C++): `pip install open3d` - Modern point cloud library
- **PCL** (C++): `sudo apt install libpcl-dev` - Industry standard
- **NumPy**: Fundamental for array operations
- **Matplotlib**: 3D visualization with `mpl_toolkits.mplot3d`

### Papers
- Fischler & Bolles (1981): "Random Sample Consensus" - Original RANSAC paper
- Ester et al. (1996): "A Density-Based Algorithm for Discovering Clusters" - DBSCAN
- Bentley (1975): "Multidimensional Binary Search Trees" - KD-Trees

### Lab Equipment
- **RPLiDAR S2**: 2D LiDAR, 30m range, 32K points/sec
- ROS2 Jazzy driver: `ros2 launch rplidar_ros rplidar_s2_launch.py`
