# Week 2 Lab: Sensor Fusion with Extended Kalman Filter

## Overview

In this lab you will implement a complete **Extended Kalman Filter (EKF)** for
robot state estimation by fusing data from three sensor modalities: an IMU
(inertial measurement unit), wheel encoders, and GPS. You will build the filter
incrementally -- starting with the prediction step, adding measurement updates
one sensor at a time, verifying correctness through Jacobian checks, tuning
noise parameters, and finally evaluating robustness to sensor dropout.

**State vector:**

```
x = [px, py, theta, vx, vy, omega]^T
```

| Symbol  | Description              | Units |
|---------|--------------------------|-------|
| px, py  | Position in world frame  | m     |
| theta   | Heading (yaw angle)      | rad   |
| vx, vy  | Velocity in body frame   | m/s   |
| omega   | Yaw rate                 | rad/s |

## Setup

### Prerequisites

```bash
pip install numpy matplotlib
```

### Generate Sensor Data

Before running the exercises, generate the simulated sensor data:

```bash
cd ../lab_exercises/data/
python generate_data.py
```

This will create `imu_data.npy`, `encoder_data.npy`, `gps_data.npy`,
`ground_truth.npy`, and `timestamps.npy` in the `data/` directory.

## Tasks

| Task | File | Topic | Key Concepts |
|------|------|-------|--------------|
| 1 | `task1_prediction.py` | EKF Prediction Step with IMU | Nonlinear motion model, Jacobian F, covariance propagation |
| 2 | `task2_encoder_update.py` | Encoder Measurement Update | Measurement model h(x), Jacobian H, Kalman gain, innovation |
| 3 | `task3_gps_update.py` | GPS Measurement Update | Absolute position correction, sparse measurements |
| 4 | `task4_jacobians.py` | Jacobian Verification | Numerical differentiation, finite differences, debugging |
| 5 | `task5_covariance_tuning.py` | Q and R Tuning | Process vs measurement noise, grid search, RMSE |
| 6 | `task6_full_ekf.py` | Complete EKF | Multi-sensor fusion, uncertainty ellipses |
| 7 | `task7_evaluation.py` | Evaluation and Sensor Dropout | RMSE, NEES, robustness, GPS denial |

### Recommended Order

Complete the tasks in numerical order. Each task builds on the previous ones:

1. **Task 1** -- Implement the prediction step and observe drift without corrections.
2. **Task 2** -- Add encoder updates and see velocity corrections.
3. **Task 3** -- Add GPS updates and see position corrections.
4. **Task 4** -- Verify all Jacobians before proceeding.
5. **Task 5** -- Tune Q and R for best performance.
6. **Task 6** -- Combine everything into a complete EKF.
7. **Task 7** -- Evaluate and test robustness to GPS dropout.

## Evaluation Criteria

| Criterion | Weight | Description |
|-----------|--------|-------------|
| Correctness | 40% | Functions produce mathematically correct results |
| Code Quality | 20% | Clean, well-documented code with meaningful variable names |
| Analysis | 20% | Plots, interpretation of results, and written observations |
| Completeness | 20% | All tasks attempted, all functions implemented |

## Submission Requirements

1. All seven completed Python files with implemented functions.
2. Generated plots saved as PNG files in a `plots/` subdirectory.
3. A brief written report (1-2 paragraphs per task) discussing:
   - Your approach and any design decisions.
   - Observations from the plots (e.g., drift behavior, effect of tuning).
   - Answers to any inline questions in the task files.
4. Ensure your code runs without errors when executed as:
   ```bash
   python task1_prediction.py
   python task2_encoder_update.py
   # ... etc.
   ```
