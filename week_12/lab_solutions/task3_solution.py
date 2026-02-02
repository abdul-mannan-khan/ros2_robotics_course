#!/usr/bin/env python3
"""Task 3 Solution: Planning-Control Integration"""
import numpy as np
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from capstone_sim import (QuadrotorDynamics, EKF, OccupancyGrid3D, AStarPlanner,
                           BSplineTrajectory, PIDController, LQRController)


def plan_and_track(controller_type='lqr', duration=40.0, dt=0.01):
    grid = OccupancyGrid3D(([-1,-1,-0.5],[12,12,6]), resolution=0.5)
    obstacles = [([3,3,2],1.0), ([6,6,3],1.0), ([9,4,2],0.8)]
    for c, r in obstacles:
        grid.add_obstacle_sphere(np.array(c), r)
    planner = AStarPlanner(grid, safety_margin=0.8)
    start, goal = np.array([0,0,2]), np.array([10,10,2])
    t0 = time.time()
    path = planner.plan(start, goal)
    plan_time = time.time() - t0
    print(f"  Planning time: {plan_time*1000:.1f} ms, path points: {len(path)}")
    if len(path) < 2:
        print("  No path found!"); return
    traj = BSplineTrajectory()
    t0 = time.time()
    traj.generate(path, velocity=1.5)
    traj_time = time.time() - t0
    print(f"  Trajectory generation: {traj_time*1000:.1f} ms, duration: {traj.get_total_time():.1f}s")
    drone = QuadrotorDynamics(); drone.reset(start.copy())
    ekf = EKF(); ekf.x[0:3] = start.copy()
    ctrl = LQRController() if controller_type == 'lqr' else PIDController()
    if hasattr(ctrl, 'reset'): ctrl.reset()
    positions, desired, errors, times, controls = [], [], [], [], []
    sim_dur = min(duration, traj.get_total_time() + 5)
    for step in range(int(sim_dur/dt)):
        t = step*dt
        tp = drone.get_position(); tv = drone.get_velocity()
        ekf.predict(dt)
        if step % 20 == 0:
            ekf.update_gps(tp + np.random.randn(3)*0.3)
        ep = ekf.get_position(); ev = ekf.get_velocity()
        pd = traj.evaluate(min(t, traj.get_total_time()))
        vd = traj.evaluate_derivative(min(t, traj.get_total_time()))
        yaw = drone.get_attitude()[2]
        u = ctrl.compute(ep, ev, pd, vd, yaw, dt)
        drone.add_noise(0.0003); drone.step(u, dt)
        positions.append(tp.copy()); desired.append(pd.copy())
        errors.append(np.linalg.norm(tp-pd)); times.append(t); controls.append(u.copy())
    positions = np.array(positions); desired = np.array(desired)
    errors = np.array(errors); controls = np.array(controls)
    rmse = np.sqrt(np.mean(errors**2))
    print(f"  RMSE: {rmse:.4f} m, Max error: {np.max(errors):.4f} m")
    fig = plt.figure(figsize=(14, 10))
    ax3d = fig.add_subplot(2,2,1, projection='3d')
    ax3d.plot(positions[:,0],positions[:,1],positions[:,2],'b-',lw=1,label='Actual')
    ax3d.plot(desired[:,0],desired[:,1],desired[:,2],'r--',lw=1,label='Desired')
    for c,r in obstacles:
        u_s,v_s = np.mgrid[0:2*np.pi:8j,0:np.pi:6j]
        ax3d.plot_surface(c[0]+r*np.cos(u_s)*np.sin(v_s),c[1]+r*np.sin(u_s)*np.sin(v_s),c[2]+r*np.cos(v_s),alpha=0.3,color='red')
    ax3d.set_title(f'{controller_type.upper()} Tracking'); ax3d.legend(fontsize=8)
    ax2 = fig.add_subplot(2,2,2)
    ax2.plot(times, errors, 'b-', lw=0.8)
    ax2.set_title(f'Tracking Error (RMSE={rmse:.3f}m)'); ax2.set_xlabel('Time (s)'); ax2.grid(True,alpha=0.3)
    ax3 = fig.add_subplot(2,2,3)
    ax3.plot(times, positions[:,0], label='x'); ax3.plot(times, positions[:,1], label='y')
    ax3.plot(times, positions[:,2], label='z'); ax3.legend(); ax3.set_title('Position'); ax3.grid(True,alpha=0.3)
    ax4 = fig.add_subplot(2,2,4)
    ax4.plot(times, controls[:,0], label='Thrust'); ax4.set_title('Control'); ax4.legend(); ax4.grid(True,alpha=0.3)
    plt.tight_layout()
    fn = f'task3_track_{controller_type}.png'
    plt.savefig(fn, dpi=150, bbox_inches='tight'); print(f"  Saved: {fn}"); plt.close()


def test_replanning(duration=50.0, dt=0.01):
    grid = OccupancyGrid3D(([-1,-1,-0.5],[12,12,6]), resolution=0.5)
    obstacles = [([3,3,2],1.0)]
    for c,r in obstacles:
        grid.add_obstacle_sphere(np.array(c), r)
    planner = AStarPlanner(grid, safety_margin=0.8)
    start, goal = np.array([0,0,2]), np.array([10,10,2])
    path = planner.plan(start, goal)
    traj = BSplineTrajectory(); traj.generate(path, velocity=1.5)
    drone = QuadrotorDynamics(); drone.reset(start.copy())
    ekf = EKF(); ekf.x[0:3] = start.copy()
    ctrl = LQRController()
    positions, times = [], []
    replanned = False; traj_t0 = 0.0
    for step in range(int(duration/dt)):
        t = step*dt
        tp = drone.get_position(); tv = drone.get_velocity()
        ekf.predict(dt)
        if step%20==0: ekf.update_gps(tp+np.random.randn(3)*0.3)
        ep = ekf.get_position(); ev = ekf.get_velocity()
        if t > 8.0 and not replanned:
            new_obs = np.array([6,6,2.5])
            grid.add_obstacle_sphere(new_obs, 1.2)
            planner._inflate()
            new_path = planner.plan(ep, goal)
            if len(new_path) >= 2:
                traj.generate(new_path, velocity=1.5); traj_t0 = t; replanned = True
                print(f"  Replanned at t={t:.1f}s")
        tt = t - traj_t0
        pd = traj.evaluate(min(tt, traj.get_total_time()))
        vd = traj.evaluate_derivative(min(tt, traj.get_total_time()))
        u = ctrl.compute(ep, ev, pd, vd, drone.get_attitude()[2], dt)
        drone.add_noise(0.0003); drone.step(u, dt)
        positions.append(tp.copy()); times.append(t)
        if np.linalg.norm(tp - goal) < 0.5 and t > 5:
            break
    positions = np.array(positions)
    print(f"  Final pos: {positions[-1]}, goal: {goal}")
    fig, ax = plt.subplots(figsize=(10,8))
    ax.plot(positions[:,0], positions[:,1], 'b-', lw=1.5, label='Trajectory')
    for c,r in obstacles:
        circle = plt.Circle((c[0],c[1]), r, color='red', alpha=0.3)
        ax.add_patch(circle)
    ax.add_patch(plt.Circle((6,6), 1.2, color='orange', alpha=0.3, label='New obstacle'))
    ax.scatter(*goal[:2], c='green', s=100, marker='*', zorder=5, label='Goal')
    ax.set_title('Replanning Test'); ax.legend(); ax.grid(True,alpha=0.3); ax.set_aspect('equal')
    plt.savefig('task3_replan.png', dpi=150, bbox_inches='tight'); print("  Saved: task3_replan.png"); plt.close()


def measure_latency():
    grid = OccupancyGrid3D(([-1,-1,-0.5],[15,15,8]), resolution=0.5)
    grid.add_obstacle_sphere(np.array([5,5,3]), 1.0)
    planner = AStarPlanner(grid, safety_margin=0.5)
    times_plan, times_traj, times_ctrl = [], [], []
    for _ in range(10):
        t0 = time.time(); planner.plan(np.array([0,0,2]), np.array([10,10,2])); times_plan.append(time.time()-t0)
    traj = BSplineTrajectory()
    path = [np.array([0,0,2]), np.array([5,5,3]), np.array([10,10,2])]
    for _ in range(100):
        t0 = time.time(); traj.generate(path, velocity=1.5); times_traj.append(time.time()-t0)
    ctrl = LQRController()
    for _ in range(1000):
        t0 = time.time()
        ctrl.compute(np.zeros(3), np.zeros(3), np.ones(3), np.zeros(3), 0, 0.01)
        times_ctrl.append(time.time()-t0)
    print(f"  Planning: {np.mean(times_plan)*1000:.1f} ms (std {np.std(times_plan)*1000:.1f})")
    print(f"  Trajectory: {np.mean(times_traj)*1000:.2f} ms")
    print(f"  Control: {np.mean(times_ctrl)*1000:.3f} ms")
    print(f"  Total pipeline: {(np.mean(times_plan)+np.mean(times_traj)+np.mean(times_ctrl))*1000:.1f} ms")


def main():
    print("Task 3 Solution: Planning-Control Integration")
    print("=" * 50)
    print("\nLQR:"); plan_and_track('lqr')
    print("\nPID:"); plan_and_track('pid')
    print("\nReplanning:"); test_replanning()
    print("\nLatency:"); measure_latency()
    print("\nTask 3 Solution complete.")


if __name__ == '__main__':
    main()
