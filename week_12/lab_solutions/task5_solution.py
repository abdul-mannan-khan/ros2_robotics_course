#!/usr/bin/env python3
"""Task 5 Solution: Full Autonomous Mission"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from capstone_sim import AutonomousDroneSystem, MissionState


def run_full_mission():
    system = AutonomousDroneSystem(controller_type='lqr', enable_replanning=True)
    bounds = ([-2,-2,-0.5], [15,15,8])
    obstacles = [([5,3,2],1.0), ([8,7,3],1.0), ([11,5,2.5],0.8)]
    waypoints = [[3,2,2], [6,5,3], [9,9,2.5], [12,10,2]]
    system.setup_environment(bounds, obstacles, waypoints)
    metrics = system.run_mission(max_time=80.0)
    data = system.perf.get_arrays()
    pos = data['positions']; des = data['desired']; t = data['times']
    fig = plt.figure(figsize=(16, 12))
    ax3d = fig.add_subplot(2,2,1, projection='3d')
    ax3d.plot(pos[:,0],pos[:,1],pos[:,2],'b-',lw=1,label='Actual')
    ax3d.plot(des[:,0],des[:,1],des[:,2],'r--',lw=1,alpha=0.6,label='Desired')
    for c,r in obstacles:
        u_s,v_s = np.mgrid[0:2*np.pi:10j,0:np.pi:6j]
        ax3d.plot_surface(c[0]+r*np.cos(u_s)*np.sin(v_s),c[1]+r*np.sin(u_s)*np.sin(v_s),c[2]+r*np.cos(v_s),alpha=0.3,color='red')
    for w in waypoints:
        ax3d.scatter(*w,c='green',s=80,marker='*')
    ax3d.set_xlabel('X (m)'); ax3d.set_ylabel('Y (m)'); ax3d.set_zlabel('Z (m)')
    ax3d.set_title('3D Trajectory'); ax3d.legend(fontsize=8)
    ax2 = fig.add_subplot(2,2,2)
    ax2.plot(t, data['tracking_errors'], 'b-', lw=0.8)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Error (m)')
    ax2.set_title(f"Tracking Error (RMSE={metrics.get('rmse',0):.3f}m)")
    ax2.grid(True, alpha=0.3)
    ax3 = fig.add_subplot(2,2,3)
    ax3.plot(t, pos[:,0], label='x'); ax3.plot(t, pos[:,1], label='y'); ax3.plot(t, pos[:,2], label='z')
    ax3.plot(t, des[:,0], '--', alpha=0.5); ax3.plot(t, des[:,1], '--', alpha=0.5); ax3.plot(t, des[:,2], '--', alpha=0.5)
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Position (m)'); ax3.set_title('Position Components')
    ax3.legend(); ax3.grid(True, alpha=0.3)
    ax4 = fig.add_subplot(2,2,4)
    states = system.perf.mission_states
    state_map = {MissionState.IDLE:0, MissionState.TAKEOFF:1, MissionState.NAVIGATE:2,
                 MissionState.REPLAN:3, MissionState.HOVER:4, MissionState.LAND:5, MissionState.COMPLETE:6}
    state_vals = [state_map.get(s, -1) for s in states]
    ax4.step(t[:len(state_vals)], state_vals, 'g-', lw=2)
    ax4.set_yticks(list(state_map.values())); ax4.set_yticklabels(list(state_map.keys()), fontsize=8)
    ax4.set_xlabel('Time (s)'); ax4.set_title('Mission State'); ax4.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('task5_mission.png', dpi=150, bbox_inches='tight')
    print(f"Saved: task5_mission.png")
    plt.close()
    print("\nMission Metrics:")
    for k,v in metrics.items():
        print(f"  {k}: {v}")
    return metrics


def run_challenging_mission():
    system = AutonomousDroneSystem(controller_type='lqr', enable_replanning=True)
    bounds = ([-2,-2,-0.5], [22,22,10])
    obstacles = [([3,2,2],0.8), ([5,5,3],1.0), ([8,4,2.5],0.9),
                 ([10,8,3],1.1), ([13,6,2],0.7), ([16,12,3.5],1.0)]
    waypoints = [[2,1,2], [5,3,3], [8,7,2.5], [11,5,3], [14,10,2.5], [18,15,2]]
    system.setup_environment(bounds, obstacles, waypoints, takeoff_alt=2.0)
    metrics = system.run_mission(max_time=120.0)
    data = system.perf.get_arrays()
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    pos = data['positions']
    ax.plot(pos[:,0],pos[:,1],pos[:,2],'b-',lw=1,label='Actual')
    for c,r in obstacles:
        u_s,v_s = np.mgrid[0:2*np.pi:8j,0:np.pi:5j]
        ax.plot_surface(c[0]+r*np.cos(u_s)*np.sin(v_s),c[1]+r*np.sin(u_s)*np.sin(v_s),c[2]+r*np.cos(v_s),alpha=0.25,color='red')
    for w in waypoints:
        ax.scatter(*w,c='green',s=80,marker='*')
    ax.set_title(f"Challenging Mission (RMSE={metrics.get('rmse',0):.3f}m, Complete={metrics.get('mission_complete',False)})")
    ax.legend()
    plt.tight_layout()
    plt.savefig('task5_challenging.png', dpi=150, bbox_inches='tight')
    print(f"Saved: task5_challenging.png")
    print(f"Challenging metrics: {metrics}")
    plt.close()


def main():
    print("Task 5 Solution: Full Autonomous Mission")
    print("=" * 50)
    print("\nStandard Mission:")
    run_full_mission()
    print("\nChallenging Mission:")
    run_challenging_mission()
    print("\nTask 5 Solution complete.")


if __name__ == '__main__':
    main()
