#!/usr/bin/env python3
"""Task 2 Solution: Sensor Fusion Integration"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from capstone_sim import QuadrotorDynamics, IMUSensor, GPSSensor, EKF


def run_sensor_fusion_comparison(duration=30.0, dt=0.01):
    drone = QuadrotorDynamics()
    drone.reset(np.array([0, 0, 2]))
    imu = IMUSensor(); gps = GPSSensor(noise_std=0.5, dropout_prob=0.0)
    ekf = EKF(); ekf.x[0:3] = drone.get_position(); ekf.x[2] = 2.0
    true_pos, gps_pos, fused_pos, imu_pos = [], [], [], []
    imu_dead = np.array([0.0, 0.0, 2.0])
    times = []
    for step in range(int(duration/dt)):
        t = step*dt
        # Circular trajectory control
        r, omega = 5.0, 0.3
        target = np.array([r*np.cos(omega*t), r*np.sin(omega*t), 2.0])
        pos = drone.get_position(); vel = drone.get_velocity()
        err = target - pos
        a_des = 5.0*err - 2.0*vel; a_des[2] += 9.81
        thrust = 1.5*np.linalg.norm(a_des)
        ctrl = np.array([thrust, 0.5*a_des[0], 0.5*a_des[1], 0])
        drone.step(ctrl, dt); drone.add_noise(0.0003)
        tp = drone.get_position(); tv = drone.get_velocity()
        am, gm = imu.measure(np.array([0,0,9.81]), drone.get_angular_velocity())
        imu_dead += tv*dt  # simple dead reckoning
        ekf.predict(dt, accel=am, gyro=gm)
        if step % 20 == 0:
            gm = gps.measure(tp)
            if gm is not None:
                ekf.update_gps(gm)
                gps_pos.append(gm.copy())
            else:
                gps_pos.append(np.full(3, np.nan))
        else:
            gps_pos.append(np.full(3, np.nan))
        true_pos.append(tp.copy()); fused_pos.append(ekf.get_position()); imu_pos.append(imu_dead.copy())
        times.append(t)
    true_pos = np.array(true_pos); gps_pos = np.array(gps_pos)
    fused_pos = np.array(fused_pos); imu_pos = np.array(imu_pos)
    gps_valid = ~np.isnan(gps_pos[:,0])
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes[0,0].plot(true_pos[:,0], true_pos[:,1], 'k-', lw=2, label='Truth')
    axes[0,0].plot(fused_pos[:,0], fused_pos[:,1], 'b-', lw=1, label='EKF Fused')
    axes[0,0].scatter(gps_pos[gps_valid,0], gps_pos[gps_valid,1], c='r', s=5, alpha=0.5, label='GPS')
    axes[0,0].set_title('XY Trajectory'); axes[0,0].legend(); axes[0,0].grid(True, alpha=0.3)
    axes[0,0].set_xlabel('X (m)'); axes[0,0].set_ylabel('Y (m)')
    err_fused = np.linalg.norm(fused_pos - true_pos, axis=1)
    err_imu = np.linalg.norm(imu_pos - true_pos, axis=1)
    axes[0,1].plot(times, err_fused, 'b-', lw=0.8, label=f'Fused (RMSE={np.sqrt(np.mean(err_fused**2)):.3f})')
    axes[0,1].plot(times, err_imu, 'g-', lw=0.8, label=f'IMU-only (RMSE={np.sqrt(np.mean(err_imu**2)):.3f})')
    axes[0,1].set_title('Position Error'); axes[0,1].legend(); axes[0,1].grid(True, alpha=0.3)
    axes[0,1].set_xlabel('Time (s)'); axes[0,1].set_ylabel('Error (m)')
    for i, lbl in enumerate(['X','Y','Z']):
        axes[1,0].plot(times, true_pos[:,i], 'k-', lw=1.5)
        axes[1,0].plot(times, fused_pos[:,i], '--', lw=0.8, label=f'{lbl} fused')
    axes[1,0].set_title('Position Components'); axes[1,0].legend(); axes[1,0].grid(True, alpha=0.3)
    cov_trace = [ekf.P[0,0]+ekf.P[1,1]+ekf.P[2,2]]  # just final
    axes[1,1].bar(['Fused RMSE', 'IMU RMSE'], [np.sqrt(np.mean(err_fused**2)), np.sqrt(np.mean(err_imu**2))], color=['blue','green'])
    axes[1,1].set_title('RMSE Comparison'); axes[1,1].set_ylabel('RMSE (m)')
    plt.tight_layout()
    plt.savefig('task2_fusion_comparison.png', dpi=150, bbox_inches='tight')
    print(f"Saved: task2_fusion_comparison.png")
    print(f"Fused RMSE: {np.sqrt(np.mean(err_fused**2)):.4f} m")
    print(f"IMU-only RMSE: {np.sqrt(np.mean(err_imu**2)):.4f} m")
    plt.close()


def test_sensor_failure_handling(duration=30.0, dt=0.01):
    drone = QuadrotorDynamics(); drone.reset(np.array([0,0,2]))
    imu = IMUSensor(); gps = GPSSensor(noise_std=0.3, dropout_prob=0.0)
    ekf = EKF(); ekf.x[0:3] = np.array([0,0,2])
    true_pos, fused_pos, times = [], [], []
    for step in range(int(duration/dt)):
        t = step*dt
        target = np.array([t*0.5, 2.0*np.sin(0.2*t), 2.0])
        pos = drone.get_position(); vel = drone.get_velocity()
        a_des = 5*(target-pos) - 2*vel; a_des[2] += 9.81
        ctrl = np.array([1.5*np.linalg.norm(a_des), 0.3*a_des[0], 0.3*a_des[1], 0])
        drone.step(ctrl, dt); drone.add_noise(0.0003)
        tp = drone.get_position()
        am, gm = imu.measure(np.array([0,0,9.81]), drone.get_angular_velocity())
        ekf.predict(dt, accel=am, gyro=gm)
        if step % 20 == 0:
            gps_available = not (10.0 <= t <= 20.0)  # dropout 10-20s
            if gps_available:
                gm = gps.measure(tp)
                if gm is not None:
                    ekf.update_gps(gm)
        true_pos.append(tp.copy()); fused_pos.append(ekf.get_position()); times.append(t)
    true_pos = np.array(true_pos); fused_pos = np.array(fused_pos); times = np.array(times)
    err = np.linalg.norm(fused_pos - true_pos, axis=1)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times, err, 'b-', lw=0.8)
    ax.axvspan(10, 20, alpha=0.2, color='red', label='GPS Dropout')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Position Error (m)')
    ax.set_title('EKF Error During GPS Dropout'); ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('task2_failure.png', dpi=150, bbox_inches='tight')
    print(f"Saved: task2_failure.png")
    dropout_mask = (times >= 10) & (times <= 20)
    print(f"Max drift during dropout: {np.max(err[dropout_mask]):.3f} m")
    recovery_mask = (times > 20) & (times < 22)
    if np.any(recovery_mask):
        print(f"Error 2s after recovery: {err[recovery_mask][-1]:.3f} m")
    plt.close()


def main():
    print("Task 2 Solution: Sensor Fusion Integration")
    print("=" * 50)
    run_sensor_fusion_comparison()
    print()
    test_sensor_failure_handling()
    print("\nTask 2 Solution complete.")


if __name__ == '__main__':
    main()
