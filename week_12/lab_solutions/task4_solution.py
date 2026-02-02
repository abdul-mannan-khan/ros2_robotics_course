#!/usr/bin/env python3
"""Task 4 Solution: Perception Pipeline"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from capstone_sim import (QuadrotorDynamics, EKF, OccupancyGrid3D, AStarPlanner,
                           BSplineTrajectory, LQRController, ObstacleDetector)


def run_perception_pipeline(duration=50.0, dt=0.01):
    bounds = ([-1,-1,-0.5],[15,15,8])
    known_obs = [([3,2,2],1.0)]
    unknown_obs = [([6,5,3],1.2), ([10,7,2.5],0.8)]
    all_obs = [(np.array(c),r) for c,r in known_obs+unknown_obs]
    grid = OccupancyGrid3D(bounds, resolution=0.5)
    for c,r in known_obs:
        grid.add_obstacle_sphere(np.array(c), r)
    planner = AStarPlanner(grid, safety_margin=0.8)
    start, goal = np.array([0,0,2]), np.array([12,10,2])
    path = planner.plan(start, goal)
    if not path:
        print("No initial path!"); return
    traj = BSplineTrajectory(); traj.generate(path, velocity=1.5)
    drone = QuadrotorDynamics(); drone.reset(start.copy())
    ekf = EKF(); ekf.x[0:3] = start.copy()
    ctrl = LQRController()
    detector = ObstacleDetector(max_range=6.0, noise_std=0.15, fp_rate=0.02)
    detected = []; positions = []; times = []; replan_times = []
    traj_t0 = 0.0
    for step in range(int(duration/dt)):
        t = step*dt
        tp = drone.get_position(); tv = drone.get_velocity()
        ekf.predict(dt)
        if step%20==0: ekf.update_gps(tp+np.random.randn(3)*0.3)
        ep = ekf.get_position(); ev = ekf.get_velocity()
        # Detection at 2Hz
        if step % 50 == 0:
            dets = detector.detect(ep, all_obs)
            new_added = False
            for dc, dr in dets:
                if not any(np.linalg.norm(dc-kc)<1.5 for kc,_ in detected):
                    detected.append((dc, dr))
                    grid.add_obstacle_sphere(dc, dr)
                    new_added = True
            if new_added:
                planner._inflate()
                new_path = planner.plan(ep, goal)
                if len(new_path) >= 2:
                    traj.generate(new_path, velocity=1.5); traj_t0 = t
                    replan_times.append(t)
                    print(f"  Replanned at t={t:.1f}s, detected {len(detected)} obstacles")
        tt = t - traj_t0
        pd = traj.evaluate(min(tt, traj.get_total_time()))
        vd = traj.evaluate_derivative(min(tt, traj.get_total_time()))
        u = ctrl.compute(ep, ev, pd, vd, drone.get_attitude()[2], dt)
        drone.add_noise(0.0003); drone.step(u, dt)
        positions.append(tp.copy()); times.append(t)
        if np.linalg.norm(tp-goal) < 0.5 and t > 3:
            print(f"  Reached goal at t={t:.1f}s"); break
    positions = np.array(positions)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    axes[0].plot(positions[:,0], positions[:,1], 'b-', lw=1.5, label='Path')
    for c,r in known_obs:
        axes[0].add_patch(plt.Circle((c[0],c[1]),r,color='red',alpha=0.4,label='Known'))
    for c,r in unknown_obs:
        axes[0].add_patch(plt.Circle((c[0],c[1]),r,color='orange',alpha=0.4,label='Unknown'))
    for dc,dr in detected:
        axes[0].add_patch(plt.Circle((dc[0],dc[1]),dr,color='yellow',alpha=0.2,linestyle='--',fill=False,lw=2))
    axes[0].scatter(*goal[:2],c='green',s=100,marker='*',zorder=5)
    axes[0].set_title('Trajectory with Obstacle Detection'); axes[0].set_aspect('equal'); axes[0].grid(True,alpha=0.3)
    handles, labels = axes[0].get_legend_handles_labels()
    by_label = dict(zip(labels, handles)); axes[0].legend(by_label.values(), by_label.keys())
    # Grid slice
    z_idx = grid.w2g(np.array([0,0,2]))[2]
    axes[1].imshow(grid.grid[:,:,z_idx].T, origin='lower', cmap='Greys', alpha=0.5,
                   extent=[bounds[0][0],bounds[1][0],bounds[0][1],bounds[1][1]])
    axes[1].plot(positions[:,0],positions[:,1],'b-',lw=1)
    axes[1].set_title('Occupancy Grid (z=2m slice)'); axes[1].set_aspect('equal')
    plt.tight_layout()
    plt.savefig('task4_perception.png', dpi=150, bbox_inches='tight')
    print(f"  Saved: task4_perception.png, Replanning events: {len(replan_times)}")
    plt.close()


def test_false_positive_handling():
    detector = ObstacleDetector(max_range=8.0, noise_std=0.1, fp_rate=0.1)
    true_obs = [(np.array([5,5,2]), 1.0)]
    fp_count = tp_count = 0
    for _ in range(500):
        dets = detector.detect(np.array([3,3,2]), true_obs)
        for dc, _ in dets:
            if np.linalg.norm(dc - np.array([5,5,2])) < 2.0:
                tp_count += 1
            else:
                fp_count += 1
    total = tp_count + fp_count
    print(f"  True positives: {tp_count}, False positives: {fp_count}")
    print(f"  FP rate: {fp_count/max(total,1)*100:.1f}%")
    print(f"  Mitigation: filter detections that don't persist across multiple scans")


def main():
    print("Task 4 Solution: Perception Pipeline")
    print("=" * 50)
    run_perception_pipeline()
    print()
    test_false_positive_handling()
    print("\nTask 4 Solution complete.")


if __name__ == '__main__':
    main()
